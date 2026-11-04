`timescale 1ns/1ps
// SC520CS-03 full-mode AE at 50 MHz. Exposure unit = half a sensor line.
// VTS=2000 -> maximum exposure 3991. Sensor baseline is 3991 half-lines, 1x gain.
// Group hold: 3812=00, six documented registers, 3812=30, delay, six readbacks.
// Analog gain 3E09 has codes 00/08/09/0B/0F/1F for 1/2/4/8/16/32x.
// 3E06 stays zero; 3E07 80..FC interpolates gain in 1/32 steps between analog stages.
// Total gain is bounded at 32x (1F/80). Legacy telemetry names coarse/fine are retained.
// FLICKER_HALF=0 retains continuous exposure. Nonzero selects one/two/three
// light periods, with BOOT_HALF=MAX_HALF=3*FLICKER_HALF and MIN_HALF<=FLICKER_HALF.
// At 30 fps / VTS 2000, 1200 half-lines is 10 ms. Below one period, bright
// scenes use the original short-exposure steps; flicker rejection is not assured.
// Compensated ladder transitions use constants, not division, and keep gain=1x
// below one period. The 140..165 metering deadband accommodates each down-step.
// An I2C/write/readback fault latches until system reset. A partial group is NEVER
// released as an attempted 'abort': the SC520CS data sheet documents no abort command.
// UART ownership is released on fault for diagnostics; motion remains inhibited.
module auto_exposure #(
 parameter SAMPLES=2700,LOW=105,HIGH=130,SETTLE_FRAMES=6,
 parameter VERIFY_DELAY=5000000,COMMAND_TIMEOUT=1500000,STALE_CYCLES=50000000,
 parameter BOOT_HALF=3991,MAX_HALF=3991,MIN_HALF=32,FLICKER_HALF=0,
 parameter RUNTIME_TARGET=0,HIGHLIGHT_DIV=64,DARK_HIGHLIGHT_DIV=128,
 parameter MAX_GAIN_COARSE=8'h0b,MAX_GAIN_FINE=8'h80
)(
 input wire clk,rst_n,enable,boot_ready,
 input wire [1:0] target_mode,
 input wire meter_valid,input wire [19:0] meter_sum,input wire [11:0] meter_count,meter_highlights,
 input wire [15:0] meter_sequence,
 output wire request,input wire grant,output wire cmd_valid,input wire cmd_ready,
 output wire cmd_read,output reg [15:0] cmd_address,output reg [7:0] cmd_value,
 input wire cmd_done,input wire [1:0] cmd_error,input wire [7:0] read_data,
 output wire busy,hold_motion,output reg fault,stable,at_limit,stale,
 output reg [7:0] fault_code,output reg [11:0] half_lines,
 output reg [7:0] gain_coarse,gain_fine,output reg [15:0] updates,
 output reg [3:0] settling,output wire [2:0] debug_state,output wire [3:0] debug_step,output wire debug_initialized
);
 localparam IDLE=0,ACQUIRE=1,SEND=2,WAIT_DONE=3,DELAY=4,READ_SEND=5,READ_WAIT=6,FAULT=7;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [2:0] state;
 reg [3:0] step;reg initialized;
 assign debug_state=state;assign debug_step=step;assign debug_initialized=initialized;
 reg [11:0] candidate_half;reg [7:0] candidate_coarse,candidate_fine;
 reg [15:0] seen_sequence;reg [31:0] age,timer;
 wire fresh=meter_valid&&(meter_count==SAMPLES)&&(meter_sequence!=seen_sequence);
 // Compile-time products feed a small selector; no runtime multiply/divide.
 // RUNTIME_TARGET=0 ignores an unconnected/unknown target_mode exactly as legacy.
 // Mode 0 and unknown modes retain the configured legacy LOW/HIGH constants.
 localparam [31:0] LEGACY_LOW_SUM=SAMPLES*LOW,LEGACY_HIGH_SUM=SAMPLES*HIGH;
 localparam [31:0] TARGET1_LOW_SUM=SAMPLES*110,TARGET1_HIGH_SUM=SAMPLES*130;
 localparam [31:0] TARGET2_LOW_SUM=SAMPLES*128,TARGET2_HIGH_SUM=SAMPLES*152;
 localparam [31:0] TARGET3_LOW_SUM=SAMPLES*152,TARGET3_HIGH_SUM=SAMPLES*180;
 wire [31:0] target_low_sum,target_high_sum;
 generate if(RUNTIME_TARGET) begin: runtime_threshold
  reg [31:0] selected_low,selected_high;
  always @* begin
   case(target_mode)
    2'd1:begin selected_low=TARGET1_LOW_SUM;selected_high=TARGET1_HIGH_SUM;end
    2'd2:begin selected_low=TARGET2_LOW_SUM;selected_high=TARGET2_HIGH_SUM;end
    2'd3:begin selected_low=TARGET3_LOW_SUM;selected_high=TARGET3_HIGH_SUM;end
    default:begin selected_low=LEGACY_LOW_SUM;selected_high=LEGACY_HIGH_SUM;end
   endcase
  end
  assign target_low_sum=selected_low;
  assign target_high_sum=selected_high;
 end else begin: fixed_threshold
  // Continuous constants have no procedural sensitivity-list dependency.
  assign target_low_sum=LEGACY_LOW_SUM;
  assign target_high_sum=LEGACY_HIGH_SUM;
 end endgenerate
 wire too_bright=(meter_sum>target_high_sum)||(meter_highlights>SAMPLES/HIGHLIGHT_DIV);
 wire too_dark=(meter_sum<target_low_sum)&&(meter_highlights<SAMPLES/DARK_HIGHLIGHT_DIV);
 wire severe_highlights=meter_highlights>SAMPLES/16;
 reg [11:0] next_half;reg [7:0] next_coarse,next_fine;
 wire [11:0] exposure_step=((half_lines>>3)==0)?12'd1:(half_lines>>3);
 wire [12:0] increased_half={1'b0,half_lines}+{1'b0,exposure_step};
 localparam [11:0] FLICKER_TWO=2*FLICKER_HALF,FLICKER_THREE=3*FLICKER_HALF;
 function [7:0] coarse_up(input [7:0] gain);
  begin case(gain) 0:coarse_up=8'h08;8'h08:coarse_up=8'h09;8'h09:coarse_up=8'h0b;
   8'h0b:coarse_up=8'h0f;default:coarse_up=8'h1f;endcase end
 endfunction
 function [7:0] coarse_down(input [7:0] gain);
  begin case(gain) 8'h1f:coarse_down=8'h0f;8'h0f:coarse_down=8'h0b;8'h0b:coarse_down=8'h09;
   8'h09:coarse_down=8'h08;default:coarse_down=0;endcase end
 endfunction
 always @* begin
  next_half=half_lines;next_coarse=gain_coarse;next_fine=gain_fine;
  if(too_bright) begin
   if(gain_fine>8'h80)next_fine=severe_highlights?(gain_fine>8'h90 ? gain_fine-8'd16:8'h80):gain_fine-8'd4;
   else if(gain_coarse!=0) begin next_coarse=coarse_down(gain_coarse);next_fine=severe_highlights?8'hc0:8'hfc;end
   else if(FLICKER_HALF!=0&&half_lines==FLICKER_THREE) begin
    // 3 periods * 1x -> 2 periods * 1.375x: total exposure decreases 8.33%.
    next_half=FLICKER_TWO;next_fine=severe_highlights?8'h80:8'hb0;
   end else if(FLICKER_HALF!=0&&half_lines==FLICKER_TWO) begin
    // 2 periods * 1x -> 1 period * 1.75x: total exposure decreases 12.5%.
    next_half=FLICKER_HALF;next_fine=severe_highlights?8'h80:8'he0;
   end
   else next_half=(half_lines-exposure_step<MIN_HALF)?MIN_HALF:half_lines-exposure_step;
  end else if(too_dark) begin
   if(FLICKER_HALF!=0) begin
    if(half_lines<FLICKER_HALF) next_half=(increased_half>FLICKER_HALF)?FLICKER_HALF:increased_half[11:0];
    else if(half_lines==FLICKER_HALF&&gain_coarse==0&&gain_fine==8'hfc) begin
     // Replace the next 2x gain step with 2 periods * 1x (+1.59% total).
     next_half=FLICKER_TWO;next_coarse=0;next_fine=8'h80;
    end else if(half_lines==FLICKER_TWO&&gain_coarse==0&&gain_fine==8'hbc) begin
     // Replace the next 1.5x gain step with 3 periods * 1x (+2.13% total).
     next_half=FLICKER_THREE;next_coarse=0;next_fine=8'h80;
    end else if(gain_coarse!=8'h1f) begin
     if(gain_fine<8'hfc) next_fine=gain_fine+8'd4;
     else begin next_coarse=coarse_up(gain_coarse);next_fine=8'h80;end
    end
   end else if(half_lines<MAX_HALF) next_half=(increased_half>MAX_HALF)?MAX_HALF:increased_half[11:0];
   else if(gain_coarse!=8'h1f) begin
    if(gain_fine<8'hfc) next_fine=gain_fine+8'd4;
    else begin next_coarse=coarse_up(gain_coarse);next_fine=8'h80;end
   end
  end
  if(next_coarse>MAX_GAIN_COARSE||(next_coarse==MAX_GAIN_COARSE&&next_fine>MAX_GAIN_FINE))begin
   next_coarse=MAX_GAIN_COARSE;next_fine=MAX_GAIN_FINE;
  end
 end
 assign busy=(state!=IDLE)&&(state!=FAULT);
 assign hold_motion=busy||(settling!=0)||fault;
 assign request=busy;
 assign cmd_valid=grant&&(state==SEND||state==READ_SEND);
 assign cmd_read=(state==READ_SEND)||(state==READ_WAIT);
 always @* begin
  cmd_address=16'h3812;cmd_value=0;
  case(step)
   1:begin cmd_address=16'h3e00;cmd_value=0;end
   2:begin cmd_address=16'h3e01;cmd_value=candidate_half[11:4];end
   3:begin cmd_address=16'h3e02;cmd_value={candidate_half[3:0],4'b0};end
   4:begin cmd_address=16'h3e09;cmd_value=candidate_coarse;end
   5:begin cmd_address=16'h3e06;cmd_value=0;end
   6:begin cmd_address=16'h3e07;cmd_value=candidate_fine;end
   7:cmd_value=8'h30;
   default:cmd_value=0;
  endcase
 end
 always @(posedge clk) begin
  if(!rst_n) begin
   state<=IDLE;step<=0;initialized<=0;candidate_half<=BOOT_HALF;candidate_coarse<=0;candidate_fine<=8'h80;
   half_lines<=BOOT_HALF;gain_coarse<=0;gain_fine<=8'h80;updates<=0;settling<=0;
   seen_sequence<=0;age<=0;timer<=0;fault<=0;fault_code<=0;stable<=0;at_limit<=0;stale<=1;
  end else begin
   if(fresh) begin seen_sequence<=meter_sequence;age<=0;stale<=0;end
   else if(age<STALE_CYCLES) age<=age+1'b1;
   else begin stale<=1;stable<=0;end
   if(!meter_valid) stable<=0;
   case(state)
    IDLE: begin
     if(!enable) begin initialized<=0;stable<=0;at_limit<=0;settling<=0;end
     else if(boot_ready&&!fault) begin
      // First group waits for a complete fresh frame after stream-on.
      if(!initialized&&fresh) begin
       candidate_half<=BOOT_HALF;candidate_coarse<=0;candidate_fine<=8'h80;state<=ACQUIRE;step<=0;stable<=0;
      end else if(fresh) begin
       if(settling!=0) settling<=settling-1'b1;
       else begin
        stable<=!too_dark&&!too_bright;
        at_limit<=(too_dark||too_bright)&&next_half==half_lines&&next_coarse==gain_coarse&&next_fine==gain_fine;
        if(next_half!=half_lines||next_coarse!=gain_coarse||next_fine!=gain_fine) begin
         candidate_half<=next_half;candidate_coarse<=next_coarse;candidate_fine<=next_fine;
         state<=ACQUIRE;step<=0;stable<=0;at_limit<=0;
        end
       end
      end
     end
    end
    // Cancel only before the hold command has entered the bus. Once a group starts,
    // disabling AE drains and verifies the whole group before releasing arbitration.
    ACQUIRE:if(!enable) state<=IDLE;else if(grant) begin state<=SEND;timer<=0;end
    SEND,READ_SEND:begin
     if(cmd_valid&&cmd_ready) begin state<=(state==SEND)?WAIT_DONE:READ_WAIT;timer<=0;end
     else if(timer>=COMMAND_TIMEOUT-1) begin fault<=1;fault_code<=8'h04;state<=FAULT;stable<=0;end
     else timer<=timer+1'b1;
    end
    WAIT_DONE:begin
     if(cmd_done) begin
      if(cmd_error!=0) begin fault<=1;fault_code<={6'd0,cmd_error};state<=FAULT;stable<=0;end
      else if(step==7) begin state<=DELAY;timer<=0;step<=1;end
      else begin step<=step+1'b1;state<=SEND;timer<=0;end
     end else if(timer>=COMMAND_TIMEOUT-1) begin fault<=1;fault_code<=8'h04;state<=FAULT;stable<=0;end
     else timer<=timer+1'b1;
    end
    DELAY:begin
     if(timer<VERIFY_DELAY-1)begin state<=DELAY;timer<=timer+1'b1;end
     else begin state<=READ_SEND;timer<=0;end
    end
    READ_WAIT:begin
     if(cmd_done) begin
      if(cmd_error!=0||read_data!=cmd_value) begin fault<=1;fault_code<=cmd_error!=0?{6'd0,cmd_error}:8'h03;state<=FAULT;stable<=0;end
      else if(step==6) begin
       half_lines<=candidate_half;gain_coarse<=candidate_coarse;gain_fine<=candidate_fine;
       updates<=updates+1'b1;settling<=SETTLE_FRAMES;initialized<=1;state<=IDLE;
      end else begin step<=step+1'b1;state<=READ_SEND;timer<=0;end
     end else if(timer>=COMMAND_TIMEOUT-1) begin fault<=1;fault_code<=8'h04;state<=FAULT;stable<=0;end
     else timer<=timer+1'b1;
    end
    FAULT:begin stable<=0;at_limit<=0;end
    default:begin state<=FAULT;fault<=1;fault_code<=8'h05;stable<=0;end
   endcase
  end
 end
endmodule
