`timescale 1ns/1ps
// Packet ownership, including each MAC's complete interframe gap.
// A pending compact control message precedes the next video packet; it never
// interrupts a packet already admitted by the NV32 transmitter.
module follow_video_arbiter(
 input wire clk,rst_n,
 input wire video_valid,video_ready,video_busy,
 input wire [7:0]video_data,input wire video_enable,
 input wire control_valid,control_busy,
 input wire [7:0]control_data,input wire control_enable,
 output wire video_admit,video_accept,control_request,control_accept,
 output wire [7:0]gmii_data,output wire gmii_enable
);
 localparam IDLE=0,VIDEO=1,CONTROL=2;
 reg[1:0]owner;
 assign control_request=rst_n&&owner==IDLE&&!video_busy&&!control_busy&&control_valid;
 assign control_accept=control_request;
 assign video_admit=rst_n&&owner==IDLE&&!video_busy&&!control_busy&&!control_valid&&video_valid;
 assign video_accept=video_admit&&video_ready;
 assign gmii_enable=rst_n&&((owner==VIDEO&&video_enable)||(owner==CONTROL&&control_enable));
 assign gmii_data=owner==CONTROL?control_data:video_data;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)owner<=IDLE;
  else case(owner)
   IDLE:if(control_request)owner<=CONTROL;else if(video_accept)owner<=VIDEO;
   VIDEO:if(!video_busy)owner<=IDLE;
   CONTROL:if(!control_busy)owner<=IDLE;
   default:owner<=IDLE;
  endcase
 end
 // synthesis translate_off
 always @(posedge clk)if(rst_n)begin
  if(video_busy&&control_busy)$fatal(1,"Two MACs own the wire");
  if(video_enable&&owner!=VIDEO)$fatal(1,"Video output without ownership");
  if(control_enable&&owner!=CONTROL)$fatal(1,"Control output without ownership");
 end
 // synthesis translate_on
endmodule
