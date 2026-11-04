`timescale 1ns/1ps
// Same leased logical-left image supplies both the Census geometry and RGB.
// Eight rows bound the Gaussian/Census + matcher lookahead. Each slot carries
// an explicit row tag and written-column count; stale colours fail closed.
module stereo_rgb_join #(parameter WIDTH=320,HEIGHT=180,X_BITS=9,Y_BITS=8)(
 input wire clk,rst,start,input wire[31:0]frame_tag,
 input wire colour_valid,input wire[31:0]colour_index,input wire[15:0]colour_rgb,
 input wire in_valid,output wire in_ready,input wire[15:0]in_z,input wire[7:0]in_disparity,
 input wire[31:0]in_tag,input wire[X_BITS-1:0]in_x,input wire[Y_BITS-1:0]in_y,
 output reg out_valid,input wire out_ready,output reg[15:0]out_z,out_rgb,output reg[7:0]out_disparity,
 output reg[31:0]out_tag,output reg[X_BITS-1:0]out_x,output reg[Y_BITS-1:0]out_y,
 output reg fault
);
 localparam AB=$clog2(WIDTH*8);
 (* ram_style="bram" *)reg[15:0]colours[0:8*WIDTH-1];
 reg[Y_BITS-1:0]row_tag[0:7];reg[X_BITS:0]written[0:7];
 reg[31:0]active_tag,write_count;reg[X_BITS-1:0]wx;reg[Y_BITS-1:0]wy;
 reg pending;reg[15:0]read_rgb;integer k;
 wire[AB-1:0]wa=wy[2:0]*WIDTH+wx;
 wire[AB-1:0]ra=in_y[2:0]*WIDTH+in_x;
 wire available=row_tag[in_y[2:0]]==in_y&&written[in_y[2:0]]>in_x;
 assign in_ready=!rst&&!fault&&!pending&&!out_valid&&available;
 always @(posedge clk)begin
  if(colour_valid&&!rst&&!fault)colours[wa]<=colour_rgb;
  if(in_valid&&in_ready)read_rgb<=colours[ra];
 end
 always @(posedge clk)begin
  if(rst||start)begin
   active_tag<=frame_tag;write_count<=0;wx<=0;wy<=0;pending<=0;
   out_valid<=0;out_z<=0;out_rgb<=0;out_disparity<=0;out_tag<=0;out_x<=0;out_y<=0;fault<=0;
   for(k=0;k<8;k=k+1)begin row_tag[k]<=0;written[k]<=0;end
  end else if(!fault)begin
   if(out_valid&&out_ready)out_valid<=0;
   if(colour_valid)begin
    if(colour_index!=write_count||wy>=HEIGHT)fault<=1;
    else begin
     row_tag[wy[2:0]]<=wy;written[wy[2:0]]<={1'b0,wx}+1'b1;
     write_count<=write_count+1'b1;
     if(wx==WIDTH-1)begin wx<=0;wy<=wy+1'b1;end else wx<=wx+1'b1;
    end
   end
   if(in_valid&&(in_tag!=active_tag||in_x>=WIDTH||in_y>=HEIGHT))fault<=1;
   if(in_valid&&!available&&wy>in_y+7)fault<=1;
   if(in_valid&&in_ready)begin
    pending<=1;out_z<=in_z;out_disparity<=in_disparity;out_tag<=in_tag;out_x<=in_x;out_y<=in_y;
   end
   if(pending)begin pending<=0;out_valid<=1;out_rgb<=read_rgb;end
  end else out_valid<=0;
 end
endmodule
