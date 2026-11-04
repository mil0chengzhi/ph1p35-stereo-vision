`timescale 1ns/1ps
// Display bypass matches the original disabled-NR three-register latency.
module stereo_video_delay3(input wire clk,rst_n,input wire[23:0]rgb,
 input wire[15:0]x,y,input wire de,valid,hs,vs,
 output wire[23:0]out_rgb,output wire[15:0]out_x,out_y,
 output wire out_de,out_valid,out_hs,out_vs);
 reg[23:0]r[0:2];reg[15:0]xx[0:2],yy[0:2];reg[2:0]d,v,h,s;integer i;
 always @(posedge clk)begin
  if(!rst_n)begin d<=0;v<=0;h<=3'b111;s<=3'b111;
   for(i=0;i<3;i=i+1)begin r[i]<=0;xx[i]<=0;yy[i]<=0;end
  end else begin
   r[0]<=rgb;xx[0]<=x;yy[0]<=y;d<={d[1:0],de};v<={v[1:0],valid};h<={h[1:0],hs};s<={s[1:0],vs};
   for(i=1;i<3;i=i+1)begin r[i]<=r[i-1];xx[i]<=xx[i-1];yy[i]<=yy[i-1];end
  end
 end
 assign out_rgb=r[2];assign out_x=xx[2];assign out_y=yy[2];
 assign out_de=d[2];assign out_valid=v[2];assign out_hs=h[2];assign out_vs=s[2];
endmodule

// Preserve the two HDMI stages used by the complete-frame publication ACK.
// No text/font/logo/status overlay can replace reconstructed pixels.
module stereo_hdmi_pipe2(input wire clk,rst_n,input wire[23:0]rgb,
 input wire de,hs,vs,output reg[23:0]out_rgb,output reg out_de,out_hs,out_vs);
 reg[23:0]r;reg d,h,s;
 always @(posedge clk)begin
  if(!rst_n)begin r<=0;d<=0;h<=1;s<=1;out_rgb<=0;out_de<=0;out_hs<=1;out_vs<=1;end
  else begin r<=rgb;d<=de;h<=hs;s<=vs;out_rgb<=r;out_de<=d;out_hs<=h;out_vs<=s;end
 end
endmodule

// Same continuous Gray-millisecond timebase formerly owned by the mono panel.
module stereo_gray_timebase #(parameter CLK_HZ=74000000)(input wire clk,rst_n,
 output reg[31:0]time_gray);
 localparam MS_CYCLES=CLK_HZ/1000;reg[$clog2(MS_CYCLES+1)-1:0]divider;reg[31:0]now_ms;
 wire[31:0]next_ms=now_ms+1'b1;
 always @(posedge clk)begin
  if(!rst_n)begin divider<=0;now_ms<=0;time_gray<=0;end
  else if(divider==MS_CYCLES-1)begin divider<=0;now_ms<=next_ms;time_gray<=next_ms^(next_ms>>1);end
  else divider<=divider+1'b1;
 end
endmodule
