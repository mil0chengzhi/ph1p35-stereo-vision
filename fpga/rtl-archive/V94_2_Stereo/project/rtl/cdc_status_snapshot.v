`timescale 1ns/1ps
// 功能：调试读取域请求冻结源域整组状态；WIDTH>=1，两域共同异步低有效复位、各两拍释放。
// read_clk域take单拍且busy=0接受请求，busy保持至源域整组锁存并ACK返回。
// source_data[WIDTH-1:0]属于source_clk，调用方同拍更新关联字段；frozen保持至下一次请求。
// read_data在busy下降后有效，sequence每次完成加一（16位回绕）；忙时take被忽略。
// 时延取决于两时钟相位，约请求2源拍+ACK2读拍；多位数据靠冻结握手保证一致性。
module cdc_status_snapshot #(parameter WIDTH=320,DIRECT_HELD_READ=0)(
 input wire source_clk,input wire read_clk,input wire rst_n,input wire take,
 input wire [WIDTH-1:0] source_data,output wire busy,output wire [WIDTH-1:0] read_data,
 output reg [15:0] completed_sequence,output wire read_initialized
);
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] source_reset,read_reset;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] request_sync,ack_sync;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [WIDTH-1:0] frozen;
 reg request,ack,seen,ever_completed;reg[WIDTH-1:0]read_bank;
 // DIRECT_HELD_READ is only for host-owned, explicitly requested snapshots.
 // read_data is valid after busy falls, until the next take; while busy it
 // is unspecified. Frozen remains owned until a later accepted request.
 // AE/AWB auto-refresh consumers keep the default registered read bank.
 assign read_data=DIRECT_HELD_READ?frozen:read_bank;
 assign read_initialized=ever_completed;
 // Direct data is unspecified before its first completed request. Host CSR
 // byte selection masks that condition after the byte mux, avoiding a mask
 // on every bit of a wide frozen record. Default registered data is unchanged.
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
  if(!read_reset[1]) begin request<=0;ack_sync<=0;seen<=0;read_bank<=0;ever_completed<=0;completed_sequence<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(take && !busy) request<=!request;
   if(ack_sync[1]!=seen) begin read_bank<=frozen;ever_completed<=1;seen<=ack_sync[1];completed_sequence<=completed_sequence+1'b1;end
  end
 end
endmodule

