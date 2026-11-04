`timescale 1ns/1ps
// 功能：调试读取域请求冻结源域整组状态；WIDTH>=1，两域共同异步低有效复位、各两拍释放。
// read_clk域take单拍且busy=0接受请求，busy保持至源域整组锁存并ACK返回。
// source_data[WIDTH-1:0]属于source_clk，调用方同拍更新关联字段；frozen保持至下一次请求。
// read_data在busy下降后有效，sequence每次完成加一（16位回绕）；忙时take被忽略。
// 时延取决于两时钟相位，约请求2源拍+ACK2读拍；多位数据靠冻结握手保证一致性。
module cdc_status_snapshot #(parameter WIDTH=320)(
 input wire source_clk,input wire read_clk,input wire rst_n,input wire take,
 input wire [WIDTH-1:0] source_data,output wire busy,output reg [WIDTH-1:0] read_data,
 output reg [15:0] completed_sequence
);
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] source_reset,read_reset;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] request_sync,ack_sync;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [WIDTH-1:0] frozen;
 reg request,ack,seen;
 assign busy=request!=seen;
 always @(posedge source_clk or negedge rst_n) begin
  if(!rst_n) source_reset<=0;else source_reset<={source_reset[0],1'b1};
 end
 always @(posedge read_clk or negedge rst_n) begin
  if(!rst_n) read_reset<=0;else read_reset<={read_reset[0],1'b1};
 end
 always @(posedge source_clk) begin
  if(!source_reset[1]) begin request_sync<=0;ack<=0;frozen<=0;end
  else begin
   request_sync<={request_sync[0],request};
   if(request_sync[1]!=ack) begin frozen<=source_data;ack<=request_sync[1];end
  end
 end
 always @(posedge read_clk) begin
  if(!read_reset[1]) begin request<=0;ack_sync<=0;seen<=0;read_data<=0;completed_sequence<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(take && !busy) request<=!request;
   if(ack_sync[1]!=seen) begin read_data<=frozen;seen<=ack_sync[1];completed_sequence<=completed_sequence+1'b1;end
  end
 end
endmodule

