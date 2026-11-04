`timescale 1ns/1ps
// Small ROM conversion, integer disparity only. No fill or extrapolation.
// Invalid match/range produces Z=0, black, but still one output for that pixel.
// Default calibration disabled. Outer complete-frame publication is mandatory.
module stereo_metric_colour #(
 parameter CALIBRATION_READY=0,parameter[31:0]MAP_ID=0,
 parameter LUT_FILE="real_depth_colour33.mem",parameter X_BITS=9,Y_BITS=8
)(
 input wire clk,rst,input wire[31:0]calibration_id,
 input wire in_valid,output wire in_ready,input wire[7:0]in_disparity,input wire in_usable,
 input wire[31:0]in_frame_tag,input wire[X_BITS-1:0]in_x,input wire[Y_BITS-1:0]in_y,
 output reg out_valid,input wire out_ready,output wire[15:0]out_Z_mm,out_colour,
 output wire out_usable,output reg[7:0]out_disparity,output reg[31:0]out_frame_tag,
 output reg[X_BITS-1:0]out_x,output reg[Y_BITS-1:0]out_y,
 output reg fault,output wire configuration_error
);
 reg[32:0]depth_rom[0:255];reg[32:0]value;reg usable;
 initial $readmemh(LUT_FILE,depth_rom);
 assign configuration_error=!CALIBRATION_READY||MAP_ID==0;
 wire step=!rst&&!fault&&!configuration_error&&(!out_valid||out_ready);
 assign in_ready=step&&calibration_id==MAP_ID;
 assign out_usable=usable&&value[32]&&value[31:16]!=0;
 assign out_Z_mm=out_usable?value[31:16]:16'd0;
 assign out_colour=out_usable?value[15:0]:16'd0;
 always @(posedge clk)begin
  if(rst)begin out_valid<=0;value<=0;usable<=0;out_disparity<=0;out_frame_tag<=0;out_x<=0;out_y<=0;fault<=0;end
  else if(!configuration_error&&in_valid&&calibration_id!=MAP_ID)begin fault<=1;out_valid<=0;end
  else if(step)begin
   out_valid<=in_valid&&calibration_id==MAP_ID;
   if(in_valid&&in_ready)begin
    value<=depth_rom[in_disparity];usable<=in_usable;out_disparity<=in_disparity;
    out_frame_tag<=in_frame_tag;out_x<=in_x;out_y<=in_y;
   end
  end
 end
endmodule
