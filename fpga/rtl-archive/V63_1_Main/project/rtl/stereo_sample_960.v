`timescale 1ns/1ps
// Preserve every local960x720 display pixel for the peer; one pipeline stage,
// matching the old sampler latency. This crops the screen margins only.
module stereo_sample_960 #(parameter OFFSET_X=160)(
 input wire clk,rst_n,input wire [15:0] x,y,input wire de,valid,
 input wire [23:0] rgb,
 output reg [15:0] out_x,out_y,output reg out_de,output reg [23:0] out_rgb
);
 always @(posedge clk)begin
  if(!rst_n)begin out_x<=0;out_y<=0;out_de<=0;out_rgb<=0;end
  else begin
   out_x<=x-OFFSET_X;out_y<=y;out_rgb<=rgb;
   out_de<=de&&valid&&y<720&&x>=OFFSET_X&&x<OFFSET_X+960;
  end
 end
endmodule
