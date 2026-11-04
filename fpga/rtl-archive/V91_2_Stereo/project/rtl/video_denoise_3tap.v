`timescale 1ns/1ps
// Display-only, symmetric horizontal [1 2 1]/4 in locally flat RGB regions.
// Center storage plus output register: two register stages, one lookahead clock.
// The center is emitted one clock after input sampling. First/last pixels of a run,
// invalid pixels, row boundaries and edges are passed unchanged. No RAM/DSP.
// All raster/valid signals accompany the center; no data from adjacent rows.
module video_denoise_3tap #(parameter ENABLE=1,THRESHOLD=32)(
 input wire clk,rst_n,
 input wire [23:0] rgb_in,input wire [15:0] x_in,y_in,
 input wire de_in,valid_in,hs_in,vs_in,
 output reg [23:0] rgb_out,output reg [15:0] x_out,y_out,
 output reg de_out,valid_out,hs_out,vs_out
);
 reg [23:0] left_rgb,center_rgb;
 reg [15:0] left_x,left_y,center_x,center_y;
 reg left_valid,center_valid,center_de,center_hs,center_vs;
 function close_channel;
  input [7:0] a,b,c;
  reg [7:0] maximum,minimum;
  begin
   maximum=a>b?a:b;maximum=maximum>c?maximum:c;
   minimum=a<b?a:b;minimum=minimum<c?minimum:c;
   close_channel=({1'b0,maximum}-{1'b0,minimum})<=THRESHOLD;
  end
 endfunction
 function [7:0] smooth_channel;
  input [7:0] a,b,c;
  reg [9:0] sum;
  begin sum={2'b00,a}+{1'b0,b,1'b0}+{2'b00,c}+10'd2;smooth_channel=sum[9:2];end
 endfunction
 wire neighbors=left_valid&&center_valid&&valid_in&&de_in&&center_de&&
  left_y==center_y&&y_in==center_y&&left_x+16'd1==center_x&&center_x+16'd1==x_in;
 wire flat=close_channel(left_rgb[23:16],center_rgb[23:16],rgb_in[23:16])&&
  close_channel(left_rgb[15:8],center_rgb[15:8],rgb_in[15:8])&&
  close_channel(left_rgb[7:0],center_rgb[7:0],rgb_in[7:0]);
 always @(posedge clk)begin
  if(!rst_n)begin
   left_rgb<=0;center_rgb<=0;left_x<=0;left_y<=0;center_x<=0;center_y<=0;
   left_valid<=0;center_valid<=0;center_de<=0;center_hs<=1;center_vs<=1;
   rgb_out<=0;x_out<=0;y_out<=0;de_out<=0;valid_out<=0;hs_out<=1;vs_out<=1;
  end else begin
   rgb_out<=ENABLE&&neighbors&&flat?{
    smooth_channel(left_rgb[23:16],center_rgb[23:16],rgb_in[23:16]),
    smooth_channel(left_rgb[15:8],center_rgb[15:8],rgb_in[15:8]),
    smooth_channel(left_rgb[7:0],center_rgb[7:0],rgb_in[7:0])}:center_rgb;
   x_out<=center_x;y_out<=center_y;de_out<=center_de;valid_out<=center_valid;
   hs_out<=center_hs;vs_out<=center_vs;
   left_rgb<=center_rgb;left_x<=center_x;left_y<=center_y;left_valid<=center_valid&&center_de;
   center_rgb<=rgb_in;center_x<=x_in;center_y<=y_in;
   center_valid<=valid_in;center_de<=de_in;center_hs<=hs_in;center_vs<=vs_in;
  end
 end
endmodule
