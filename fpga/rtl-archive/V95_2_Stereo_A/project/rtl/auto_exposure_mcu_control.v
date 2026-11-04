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
module auto_exposure_mcu_control #(
 parameter SAMPLES=2700,LOW=105,HIGH=130,SETTLE_FRAMES=6,
 parameter VERIFY_DELAY=5000000,COMMAND_TIMEOUT=1500000,STALE_CYCLES=50000000,
 parameter BOOT_HALF=3991,MAX_HALF=3991,MIN_HALF=32,FLICKER_HALF=0,
 parameter RUNTIME_TARGET=0,HIGHLIGHT_DIV=64,DARK_HIGHLIGHT_DIV=128,
 parameter MAX_GAIN_COARSE=8'h0b,MAX_GAIN_FINE=8'h80
)(
 input wire clk,rst_n,enable,boot_ready,
 output wire compute_request,input wire compute_ready,compute_done,compute_fault,input wire[31:0]compute_result,
 input wire [1:0] target_mode,
 input wire meter_valid,input wire [19:0] meter_sum,input wire [11:0] meter_count,meter_highlights,
 input wire [15:0] meter_sequence,
 output wire request,input wire grant,output wire cmd_valid,input wire cmd_ready,
 output wire cmd_read,output reg [15:0] cmd_address,output reg [7:0] cmd_value,
 input wire cmd_done,input wire [1:0] cmd_error,input wire [7:0] read_data,
 output wire busy,hold_motion,output reg fault,stable,at_limit,stale,
 output reg [7:0] fault_code,output reg [11:0] half_lines,
 output reg [7:0] gain_coarse,gain_fine,output reg [15:0] updates,
 output reg [3:0] settling,output wire[2:0]debug_state,output wire[3:0]debug_step,output wire debug_initialized
);
 localparam IDLE=0,ACQUIRE=1,SEND=2,WAIT_DONE=3,DELAY=4,READ_SEND=5,READ_WAIT=6,FAULT=7,COMPUTE=8;
 reg [3:0] state;reg[15:0]compute_sequence;reg compute_cancelled;reg [3:0] step;reg initialized;
 assign debug_state=state==COMPUTE?3'd0:state[2:0];assign debug_step=step;assign debug_initialized=initialized;
 reg [11:0] candidate_half;reg [7:0] candidate_coarse,candidate_fine;
 reg [15:0] seen_sequence;reg [31:0] age,timer;
 wire fresh=meter_valid&&(meter_count==SAMPLES)&&(meter_sequence!=seen_sequence);
 // The worker owns its input record until completion. A newer metering
 // record withdraws this decision; an already-started I2C group still drains.
 wire[11:0]next_half=compute_result[11:0];
 wire[7:0]next_coarse=compute_result[23:16],next_fine=compute_result[31:24];
 wire too_bright=compute_result[12],too_dark=compute_result[13];
 assign compute_request=state==IDLE&&enable&&boot_ready&&!fault&&fresh&&initialized&&settling==0&&compute_ready;
 assign busy=(state!=IDLE)&&(state!=FAULT)&&(state!=COMPUTE);
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
   seen_sequence<=0;compute_sequence<=0;compute_cancelled<=0;age<=0;timer<=0;fault<=0;fault_code<=0;stable<=0;at_limit<=0;stale<=1;
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
       else if(compute_ready)begin
        state<=COMPUTE;compute_sequence<=meter_sequence;compute_cancelled<=0;timer<=0;stable<=0;
       end
      end
     end
    end
    COMPUTE:begin
     if(compute_fault||timer>=COMMAND_TIMEOUT-1)begin fault<=1;fault_code<=8'h06;state<=FAULT;stable<=0;end
     else begin
      if(!enable)begin compute_cancelled<=1;initialized<=0;stable<=0;at_limit<=0;settling<=0;end
      if(compute_done)begin
      state<=IDLE;
      if(enable&&!compute_cancelled&&meter_valid&&!stale&&meter_sequence==compute_sequence)begin
       stable<=!too_dark&&!too_bright;
       at_limit<=(too_dark||too_bright)&&next_half==half_lines&&next_coarse==gain_coarse&&next_fine==gain_fine;
       if(next_half!=half_lines||next_coarse!=gain_coarse||next_fine!=gain_fine)begin
        candidate_half<=next_half;candidate_coarse<=next_coarse;candidate_fine<=next_fine;
        state<=ACQUIRE;step<=0;stable<=0;at_limit<=0;
       end
      end
      end else timer<=timer+1'b1;
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
    DELAY:if(timer>=VERIFY_DELAY-1) begin state<=READ_SEND;timer<=0;end else timer<=timer+1'b1;
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
 // synthesis translate_off
 initial if(SAMPLES!=3600||LOW!=110||HIGH!=132||BOOT_HALF!=3600||MAX_HALF!=3600||MIN_HALF!=32||FLICKER_HALF!=1200||RUNTIME_TARGET!=1||HIGHLIGHT_DIV!=64||DARK_HIGHLIGHT_DIV!=128||MAX_GAIN_COARSE!=8'h0b||MAX_GAIN_FINE!=8'h80)$fatal(1,"MCU AE profile mismatch");
 // synthesis translate_on
endmodule
