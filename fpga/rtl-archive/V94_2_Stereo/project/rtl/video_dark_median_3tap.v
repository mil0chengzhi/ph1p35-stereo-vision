`timescale 1ns/1ps
// Standalone experimental display-only filter; not integrated or qualified.
// Horizontal centre needs one right neighbour; centre and output registers
// give TWO registered stages, equally in enabled and disabled modes.
// Each RGB median can erase a one-pixel dark line or synthesize a new colour.
// Do not put this stage in RAW DDR, AE statistics or metric stereo inputs.
module video_dark_median_3tap #(
 parameter WIDTH=960, HEIGHT=720, OFFSET_X=160
)(
 input wire clk,rst_n,enable,
 input wire [23:0] rgb_in,
 input wire [15:0] x_in,y_in,
 input wire de_in,valid_in,hs_in,vs_in,
 output reg [23:0] rgb_out,
 output reg [15:0] x_out,y_out,
 output reg de_out,valid_out,hs_out,vs_out
);
 // Fixed proxy rule, not programmable: round((77R+150G+29B)/256).
 // Widths are explicit (max sum with rounding = 65408), using shift/add.
 function [7:0] luma8;
  input [23:0] rgb;
  reg [15:0] r,g,b,total;
  begin
   r={8'd0,rgb[23:16]};g={8'd0,rgb[15:8]};b={8'd0,rgb[7:0]};
   total=(r<<6)+(r<<3)+(r<<2)+r+
         (g<<7)+(g<<4)+(g<<2)+(g<<1)+
         (b<<4)+(b<<3)+(b<<2)+b+16'd128;
   luma8=total[15:8];
  end
 endfunction
 function [7:0] median3;
  input [7:0] a,b,c;
  begin
   if(a<=b)begin
    if(b<=c)median3=b;else if(a<=c)median3=c;else median3=a;
   end else begin
    if(a<=c)median3=a;else if(b<=c)median3=c;else median3=b;
   end
  end
 endfunction
 wire [7:0] right_y=luma8(rgb_in);
 wire right_cap=rgb_in[23:16]<=8'd128&&rgb_in[15:8]<=8'd128&&rgb_in[7:0]<=8'd128;
 reg [23:0] center_rgb,left_rgb;
 reg [15:0] center_x,center_y,left_x,left_y;
 reg [7:0] center_luma,left_luma;
 reg center_de,center_valid,center_hs,center_vs,center_enable,center_cap;
 reg left_valid,left_cap;
 wire same_neighbours=left_valid&&center_de&&center_valid&&de_in&&valid_in&&
  left_y==center_y&&y_in==center_y&&
  ({1'b0,left_x}+17'd1=={1'b0,center_x})&&({1'b0,center_x}+17'd1=={1'b0,x_in})&&
  left_x>=OFFSET_X&&x_in<OFFSET_X+WIDTH&&center_y<HEIGHT;
 wire apply_median=center_enable&&same_neighbours&&
  center_luma<8'd80&&left_luma<8'd96&&right_y<8'd96&&left_cap&&center_cap&&right_cap;
 always @(posedge clk)begin
  if(!rst_n)begin
   center_rgb<=0;left_rgb<=0;center_x<=0;center_y<=0;left_x<=0;left_y<=0;
   center_luma<=0;left_luma<=0;center_de<=0;center_valid<=0;center_hs<=1;center_vs<=1;
   center_enable<=0;center_cap<=0;left_valid<=0;left_cap<=0;
   rgb_out<=0;x_out<=0;y_out<=0;de_out<=0;valid_out<=0;hs_out<=1;vs_out<=1;
  end else begin
   center_rgb<=rgb_in;center_x<=x_in;center_y<=y_in;
   center_luma<=right_y;center_cap<=right_cap;center_enable<=enable;
   center_de<=de_in;center_valid<=valid_in;center_hs<=hs_in;center_vs<=vs_in;
   left_rgb<=center_rgb;left_x<=center_x;left_y<=center_y;
   left_luma<=center_luma;left_cap<=center_cap;left_valid<=center_valid&&center_de;
   if(apply_median)rgb_out<={median3(left_rgb[23:16],center_rgb[23:16],rgb_in[23:16]),
    median3(left_rgb[15:8],center_rgb[15:8],rgb_in[15:8]),
    median3(left_rgb[7:0],center_rgb[7:0],rgb_in[7:0])};
   else rgb_out<=center_rgb;
   x_out<=center_x;y_out<=center_y;de_out<=center_de;valid_out<=center_valid;hs_out<=center_hs;vs_out<=center_vs;
  end
 end
 // synthesis translate_off
 initial if(WIDTH<1||HEIGHT<1||OFFSET_X<0||OFFSET_X+WIDTH>65535)
  $fatal(1,"Invalid dark median image geometry");
 // synthesis translate_on
endmodule
