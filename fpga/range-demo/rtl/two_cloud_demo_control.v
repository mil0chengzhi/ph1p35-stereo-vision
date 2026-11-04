`timescale 1ns/1ps
// B-only demo sequencing. KEY1: scene1/off/scene2/off/repeat.
// Requested controls repeat over the existing CRC-protected VIEW Ethernet
// channel; relative actions are never encoded. Both models auto rotate.
module two_cloud_demo_control #(parameter READY_CYCLES=10000000)(
 input wire clk,rst_n,show_pressed,range_pressed,
 input wire peer_valid,input wire[31:0]peer_session,input wire[15:0]applied_token,
 output wire[47:0]view_record,output wire peer_read_requested,
 output wire cloud_visible,output reg distance_enabled,
 output reg selected_slot,output reg next_slot,output reg[15:0]token,
 output reg[31:0]show_presses,exit_presses,range_presses,
 output wire busy,exhausted
);
 localparam OFF=0,WAIT_VIEW=1,SHOW=2;
 reg[1:0]state;reg[31:0]entry_session,stable_session;
 reg[(READY_CYCLES>1?$clog2(READY_CYCLES):1)-1:0]ready_age;
 wire requested=state!=OFF;
 assign peer_read_requested=requested;
 assign cloud_visible=state==SHOW;
 assign busy=state==WAIT_VIEW;
 assign exhausted=token==16'hffff;
 // Reserved VIEW bits:42 show,41 model bank. Existing bit24 is auto mode.
 // 0.75x / pitch8 keeps the captured objects within the native720 raster.
 assign view_record={5'd0,requested,selected_slot,token,requested,8'd12,8'd8,8'd0};
 wire ready=requested&&peer_valid&&peer_session[31:24]==8'hd1&&
                  peer_session!=entry_session&&applied_token==token;
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=OFF;distance_enabled<=0;selected_slot<=0;next_slot<=0;token<=1;
   show_presses<=0;exit_presses<=0;range_presses<=0;
   entry_session<=0;stable_session<=0;ready_age<=0;
  end else begin
   if(!ready||stable_session!=peer_session)begin ready_age<=0;stable_session<=peer_session;end
   else if(state==WAIT_VIEW)begin
    if(ready_age==READY_CYCLES-1)begin state<=SHOW;ready_age<=0;end
    else ready_age<=ready_age+1'b1;
   end
   if(!exhausted)begin
    if(range_pressed)begin
     distance_enabled<=!distance_enabled;state<=OFF;token<=token+1'b1;
     range_presses<=range_presses+1'b1;ready_age<=0;
     if(requested)next_slot<=!selected_slot;
    end else if(show_pressed)begin
     distance_enabled<=0;token<=token+1'b1;ready_age<=0;
     if(state==OFF)begin
      state<=WAIT_VIEW;selected_slot<=next_slot;entry_session<=peer_session;
      show_presses<=show_presses+1'b1;
     end else begin
      state<=OFF;next_slot<=!selected_slot;exit_presses<=exit_presses+1'b1;
     end
    end
   end
  end
 end
endmodule
