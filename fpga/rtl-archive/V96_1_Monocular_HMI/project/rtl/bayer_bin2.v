`timescale 1ns/1ps
// 功能：BGGR RAW10按相邻2x2块产生RGB565，两个绿色平均；分辨率宽高各减半，视野不裁切。
// WIDTH为4的倍数、<=4*2^ADDRESS_BITS；输入完整行、偶数行数，BGGR首行B/G、次行G/R。
// BLACK=0..1023，GAIN_R/G/B为Q8.8常量0..1024；APPLY_GAMMA、RUNTIME_CONFIG为0/1。
// RUNTIME_CONFIG=0忽略运行时输入，保持常量处理；=1使用runtime_black[9:0]及RGB增益[10:0]。
// 运行时增益合法范围0..1024(Q8.8)，调用方必须在frame_start原子提交并保持整帧稳定；本模块不做CDC。
// clk为MIPI字节域；rst_n同步低有效；frame_start清空行相位和流水线，line_start/end标记原始行。
// pixels[39:0]低10位先到，pixels_valid一拍四个连续像素，无背压；奇数原始行输出两个RGB565。
// rgb[31:0]低16位为左像素，输入第N沿采样后于第N+4沿输出（共5个采样沿），有效脉冲对齐。
// 流水线：行缓存读出/输入锁存→10位去黑及绿色平均→21位增益乘积→8位量化→Gamma及RGB565。
// 不判定CRC，帧提交由下游根据接收校验结果决定；坏帧不得发布。
module bayer_bin2 #(parameter WIDTH=2592,ADDRESS_BITS=10,BLACK=64,
 parameter GAIN_R=256,GAIN_G=256,GAIN_B=256,APPLY_GAMMA=1,RUNTIME_CONFIG=0)(
 input wire clk,input wire rst_n,input wire frame_start,input wire line_start,input wire line_end,
 input wire [39:0] pixels,input wire pixels_valid,output reg [31:0] rgb,output reg valid,
 input wire [9:0] runtime_black,input wire [10:0] runtime_gain_r,runtime_gain_g,runtime_gain_b,
 output wire[7:0]meter_linear_r,meter_linear_g,meter_linear_b,output wire meter_linear_valid
);
 reg [39:0] line_memory[0:(1<<ADDRESS_BITS)-1];
 reg [ADDRESS_BITS-1:0] group_index;reg row_odd;
 reg [39:0] previous_row,current_row;
 reg v0,v1,v2,v3;
 // 125MHz域单独保留扣黑寄存级，切开RAM读出、比较/减法与DSP乘法路径；不允许前后重定时吞并。
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [9:0] black_r0,black_g0,black_b0,black_r1,black_g1,black_b1;
 // 10位信号乘11位增益保留完整21位；合法最大值1023*1024，量化时再舍入和饱和。
 reg [20:0] r0,g0,b0,r1,g1,b1;
 reg [7:0] lr0,lg0,lb0,lr1,lg1,lb1;
 wire [7:0] sr0,sg0,sb0,sr1,sg1,sb1;
 assign meter_linear_r=lr0;assign meter_linear_g=lg0;assign meter_linear_b=lb0;assign meter_linear_valid=v3;
 wire [9:0] active_black=RUNTIME_CONFIG?runtime_black:BLACK;
 wire [10:0] active_gain_r=RUNTIME_CONFIG?runtime_gain_r:GAIN_R;
 wire [10:0] active_gain_g=RUNTIME_CONFIG?runtime_gain_g:GAIN_G;
 wire [10:0] active_gain_b=RUNTIME_CONFIG?runtime_gain_b:GAIN_B;
 wire [10:0] green0={1'b0,previous_row[19:10]}+{1'b0,current_row[9:0]}+1'b1;
 wire [10:0] green1={1'b0,previous_row[39:30]}+{1'b0,current_row[29:20]}+1'b1;
 // 功能：RAW10样本p减去black(均0..1023)，低于黑电平返回0；无指针或错误返回。
 function [9:0] subtract_black;input [9:0] p;input [9:0] black;
  begin subtract_black=(p>black)?p-black:0;end
 endfunction
 // 功能：Q8.8增益后的RAW10转8位，除1024并四舍五入，超过255饱和。
 function [7:0] quantize;input [20:0] p;reg [21:0] rounded;
  begin rounded={1'b0,p}+22'd512;quantize=(rounded>=22'd262144)?8'hff:rounded[17:10];end
 endfunction
 generate if(APPLY_GAMMA)begin:gamma
  wire[7:0]er0,eg0,eb0,er1,eg1,eb1;reg gamma_ready;
  always @(posedge clk)gamma_ready<=rst_n&&!frame_start;
  gamma_pair_rom red(.clk(clk),.linear0(quantize(r0)),.linear1(quantize(r1)),.encoded0(er0),.encoded1(er1));
  gamma_pair_rom green(.clk(clk),.linear0(quantize(g0)),.linear1(quantize(g1)),.encoded0(eg0),.encoded1(eg1));
  gamma_pair_rom blue(.clk(clk),.linear0(quantize(b0)),.linear1(quantize(b1)),.encoded0(eb0),.encoded1(eb1));
  // ROM registers occupy the existing quantize sampling edge. Linear meter
  // taps retain lr/lg/lb registers and their timing. No valid stage is added.
  assign sr0=gamma_ready?er0:8'd0;assign sg0=gamma_ready?eg0:8'd0;assign sb0=gamma_ready?eb0:8'd0;
  assign sr1=gamma_ready?er1:8'd0;assign sg1=gamma_ready?eg1:8'd0;assign sb1=gamma_ready?eb1:8'd0;
 end else begin: linear_output
  assign sr0=lr0;assign sg0=lg0;assign sb0=lb0;assign sr1=lr1;assign sg1=lg1;assign sb1=lb1;
 end endgenerate
 always @(posedge clk) if(rst_n && pixels_valid && !row_odd) line_memory[group_index]<=pixels;
 always @(posedge clk) previous_row<=line_memory[group_index];
 always @(posedge clk) begin
  if(!rst_n || frame_start) begin group_index<=0;row_odd<=0;v0<=0;v1<=0;v2<=0;v3<=0;valid<=0;rgb<=0;
   black_r0<=0;black_g0<=0;black_b0<=0;black_r1<=0;black_g1<=0;black_b1<=0;
   current_row<=0;r0<=0;g0<=0;b0<=0;r1<=0;g1<=0;b1<=0;lr0<=0;lg0<=0;lb0<=0;lr1<=0;lg1<=0;lb1<=0;
  end else begin
   if(line_start) group_index<=0;
   if(pixels_valid) begin group_index<=group_index+1'b1;current_row<=pixels;end
   if(line_end) row_odd<=!row_odd;
   v0<=pixels_valid && row_odd;v1<=v0;v2<=v1;v3<=v2;valid<=v3;
   black_r0<=subtract_black(current_row[19:10],active_black);
   black_g0<=subtract_black(green0[10:1],active_black);
   black_b0<=subtract_black(previous_row[9:0],active_black);
   black_r1<=subtract_black(current_row[39:30],active_black);
   black_g1<=subtract_black(green1[10:1],active_black);
   black_b1<=subtract_black(previous_row[29:20],active_black);
   r0<=black_r0*active_gain_r;g0<=black_g0*active_gain_g;b0<=black_b0*active_gain_b;
   r1<=black_r1*active_gain_r;g1<=black_g1*active_gain_g;b1<=black_b1*active_gain_b;
   lr0<=quantize(r0);lg0<=quantize(g0);lb0<=quantize(b0);
   lr1<=quantize(r1);lg1<=quantize(g1);lb1<=quantize(b1);
   rgb<={sr1[7:3],sg1[7:2],sb1[7:3],sr0[7:3],sg0[7:2],sb0[7:3]};
  end
 end
endmodule
