`timescale 1ns/1ps
// One registered stage; transmit every valid active pixel without resizing.
module stereo_sample_720p(
 input wire clk,rst_n,input wire [15:0] x,y,input wire de,valid,
 input wire [23:0] rgb,
 output reg [15:0] out_x,out_y,output reg out_de,output reg [23:0] out_rgb
);
 always @(posedge clk) begin
  if(!rst_n)begin out_x<=0;out_y<=0;out_de<=0;out_rgb<=0;end
  else begin
   out_x<=x;out_y<=y;out_rgb<=rgb;
   out_de<=de&&valid&&x<1280&&y<720;
  end
 end
endmodule
