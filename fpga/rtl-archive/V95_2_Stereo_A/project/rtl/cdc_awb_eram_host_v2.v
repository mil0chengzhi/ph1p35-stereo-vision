`timescale 1ns/1ps
// Automatic AWB capture: source byte clock -> read board50, two-FF control CDC.
// Source/read reset release is synchronized in each domain; host reset is independent.
// read_clk域take单拍且busy=0接受请求，busy保持至源域整组锁存并ACK返回。
// source_data[WIDTH-1:0]属于source_clk，调用方同拍更新关联字段；frozen保持至下一次请求。
// read_data在busy下降后有效，sequence每次完成加一（16位回绕）；忙时take被忽略。
// 时延取决于两时钟相位，约请求2源拍+ACK2读拍；多位数据靠冻结握手保证一致性。
module cdc_awb_eram_host_v2 #(parameter WIDTH=146)(
 input wire source_clk,input wire read_clk,input wire rst_n,input wire take,
 input wire host_reset_n,host_take,output wire[WIDTH-1:0]host_data,
 input wire [WIDTH-1:0] source_data,output wire busy,output wire [WIDTH-1:0] read_data,
 output reg [15:0] completed_sequence,output wire read_initialized
);
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] source_reset,read_reset;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] request_sync,ack_sync;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [WIDTH-1:0] frozen;
 reg request,ack,seen,ever_completed;
 // Live ERAM output retains the last completed registered value throughout
 // the next busy interval, with exactly the original ACK update cycle. MCU
 // auto-refresh samples remain unchanged. RAM outputs never reset: synchronous
 // live/host validity epochs mask them to zero with independent reset semantics.
 // At coincident ACK and host_take, portB captures old portA and old live_valid;
 // it must not forward the new source value. No source-domain CDC is removed.
 dual_eram_registers_validmask_v2 #(.WIDTH(WIDTH)) storage(
  .clk(read_clk),.live_reset_n(read_reset[1]),.host_reset_n(host_reset_n),
  .live_write(read_reset[1]&&ack_sync[1]!=seen),.host_write(host_take),
  .live_input(frozen),.live_output(read_data),.host_output(host_data));
 assign read_initialized=ever_completed;
 // Initialized/sequence/busy preserve the original snapshot controller.
 // Live output is zero before its first completion; host is a separately owned
 // compact snapshot, expanded into the unchanged192-bit host CSR layout above.
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
  if(!read_reset[1]) begin request<=0;ack_sync<=0;seen<=0;ever_completed<=0;completed_sequence<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(take && !busy) request<=!request;
   if(ack_sync[1]!=seen) begin ever_completed<=1;seen<=ack_sync[1];completed_sequence<=completed_sequence+1'b1;end
  end
 end
endmodule

