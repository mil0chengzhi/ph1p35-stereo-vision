`timescale 1ns/1ps
// Preserve full 4:3 field: 960x720 displayed local image -> 640x480.
// Pick phases0/2 and skip1: combined1280->960->640 mapping is uniform source2*x/2*y.
module stereo_sample_640 #(parameter OFFSET_X=160)(
 input wire clk,rst_n,input wire [15:0] x,y,input wire de,valid,
 input wire [23:0] rgb,
 output reg [15:0] out_x,out_y,output reg out_de,output reg [23:0] out_rgb
);
 reg [1:0] col_phase,row_phase;reg [9:0] col;reg [8:0] row;
 always @(posedge clk)begin
  if(!rst_n)begin col_phase<=0;row_phase<=0;col<=0;row<=0;
   out_x<=0;out_y<=0;out_de<=0;out_rgb<=0;end
  else begin
   out_de<=0;out_rgb<=rgb;out_x<={6'd0,col};out_y<={7'd0,row};
   if(x==0)begin col_phase<=0;col<=0;
    if(y==0)begin row_phase<=0;row<=0;end
    else if(y<720)begin
     row_phase<=row_phase==2?0:row_phase+1'b1;
     if(row_phase!=1)row<=row+1'b1;
    end
   end
   if(de&&y<720&&x>=OFFSET_X&&x<OFFSET_X+960)begin
    if(col_phase!=1&&row_phase!=1&&valid)out_de<=1;
    if(col_phase!=1)col<=col+1'b1;
    col_phase<=col_phase==2?0:col_phase+1'b1;
   end
  end
 end
endmodule
