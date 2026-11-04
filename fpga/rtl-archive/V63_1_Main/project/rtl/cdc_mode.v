`timescale 1ns/1ps
// 功能：WIDTH位模式通过单项邮箱一致跨域，在目标消隐commit时生效并应答。
// src_clk为控制域，dst_clk像素域，rst_n共同异步低有效；两域各两拍同步释放。
// WIDTH>=1，默认2兼容原4模式；write&&!busy接受mode，busy时忽略，冻结数据保持至应答。
// src_mode为ACK后的已应用模式，busy解除时已更新；destination_mode无固定跨域延迟。
module cdc_mode #(parameter WIDTH=2)(input wire src_clk,input wire dst_clk,input wire rst_n,
 input wire write,input wire [WIDTH-1:0] mode,output wire busy,input wire commit,
 output reg [WIDTH-1:0] destination_mode,output reg [WIDTH-1:0] src_mode);
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] src_reset,dst_reset;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg request,ack;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] request_sync,ack_sync;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [WIDTH-1:0] held_mode,pending_mode;
 reg pending_valid,pending_request,src_seen_ack;
 wire src_rst_n=src_reset[1],dst_rst_n=dst_reset[1];
 assign busy=request!=src_seen_ack || !src_rst_n;
 always @(posedge src_clk or negedge rst_n)
  if(!rst_n) src_reset<=0;else src_reset<={src_reset[0],1'b1};
 always @(posedge dst_clk or negedge rst_n)
  if(!rst_n) dst_reset<=0;else dst_reset<={dst_reset[0],1'b1};
 always @(posedge src_clk or negedge src_rst_n) begin
  if(!src_rst_n) begin request<=0;ack_sync<=0;held_mode<=0;src_seen_ack<=0;src_mode<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(ack_sync[1]!=src_seen_ack) begin src_mode<=held_mode;src_seen_ack<=ack_sync[1];end
   if(write && !busy) begin held_mode<=mode;request<=!request;end
  end
 end
 always @(posedge dst_clk or negedge dst_rst_n) begin
  if(!dst_rst_n) begin request_sync<=0;ack<=0;pending_mode<=0;destination_mode<=0;pending_valid<=0;pending_request<=0;end
  else begin
   request_sync<={request_sync[0],request};
   if(!pending_valid && request_sync[1]!=ack) begin pending_mode<=held_mode;pending_request<=request_sync[1];pending_valid<=1;end
   if(commit && pending_valid) begin destination_mode<=pending_mode;ack<=pending_request;pending_valid<=0;end
  end
 end
endmodule
