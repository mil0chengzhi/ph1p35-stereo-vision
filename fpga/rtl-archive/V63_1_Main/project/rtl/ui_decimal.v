`timescale 1ns/1ps
// 功能：在消隐期并行将COUNT个16位无符号数转换成5位十进制BCD，避免像素路径组合除法。
// clk域、rst_n同步低有效；COUNT>=1。values低16位为第0个数，每数有效范围0..65535。
// load在busy=0时接收快照，其余输入变化不影响计算；忙时忽略load，无背压重试。
// bcd低20位为第0个数，各4bit由低到高为个/十/百/千/万；结果16拍后原子更新。
// busy为计算状态；done为结果更新一拍脉冲；复位使输出0。无指针及错误返回。
module ui_decimal #(parameter COUNT=5)(
 input wire clk,input wire rst_n,input wire load,input wire [COUNT*16-1:0] values,
 output reg [COUNT*20-1:0] bcd,output reg busy,output reg done
);
 reg [COUNT*16-1:0] binary;reg [COUNT*20-1:0] work;
 reg [COUNT*20-1:0] corrected,next_bcd;
 reg [4:0] step;
 integer k,n;
 always @* begin
  corrected=work;next_bcd=0;
  for(k=0;k<COUNT;k=k+1)begin
   for(n=0;n<5;n=n+1)if(work[k*20+n*4+:4]>=5)corrected[k*20+n*4+:4]=work[k*20+n*4+:4]+4'd3;
   next_bcd[k*20+:20]={corrected[k*20+:19],binary[k*16+15]};
  end
 end
 integer j;
 always @(posedge clk)begin
  if(!rst_n)begin bcd<=0;busy<=0;done<=0;binary<=0;work<=0;step<=0;end
  else begin
   done<=0;
   if(load&&!busy)begin binary<=values;work<=0;step<=0;busy<=1;end
   if(busy)begin
    work<=next_bcd;step<=step+1'b1;
    for(j=0;j<COUNT;j=j+1)binary[j*16+:16]<={binary[j*16+:15],1'b0};
    if(step==15)begin bcd<=next_bcd;busy<=0;done<=1;end
   end
  end
 end
endmodule
