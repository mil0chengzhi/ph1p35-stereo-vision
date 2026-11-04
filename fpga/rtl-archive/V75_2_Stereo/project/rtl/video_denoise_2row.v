`timescale 1ns/1ps
// Display/debug only. Flat RGB neighbourhoods use two causal rows:
// [1 2 1] previous + [1 2 1] current, /8, nearest rounding.
// The vertical kernel is centred half a row above the tagged current pixel;
// weak horizontal edges can therefore have a half-row phase bias. Metric
// stereo/calibration must bypass this optional display filter.
// Three register stages (READ_FIRST + centre + output), including bypass.
// enable belongs to the centre pixel; callers commit it during frame blanking.
// One WIDTH*25 line RAM; no RAM reset, DDR access or frame buffer. Explicit
// valid0 writes and complete/in-order row ownership prevent stale-row reuse.
module video_denoise_2row #(
 parameter WIDTH=960,HEIGHT=720,OFFSET_X=160,THRESHOLD_2D=48,THRESHOLD_1D=32
)(
 input wire clk,rst_n,enable,
 input wire [23:0] rgb_in,input wire [15:0] x_in,y_in,
 input wire de_in,valid_in,hs_in,vs_in,
 output reg [23:0] rgb_out,output reg [15:0] x_out,y_out,
 output reg de_out,valid_out,hs_out,vs_out
);
 localparam ADDRESS_BITS=WIDTH>1?$clog2(WIDTH):1;
 (* ram_style="bram" *) reg [24:0] previous_line[0:(1<<ADDRESS_BITS)-1];
 reg [24:0] previous_data;
 wire in_roi=de_in&&x_in>=OFFSET_X&&x_in<OFFSET_X+WIDTH&&y_in<HEIGHT;
 wire [ADDRESS_BITS-1:0] address=x_in-OFFSET_X;
 // Both operations use the old same-address word: never WRITE_THROUGH.
 // Keep RAM read/reset-independent so TD can infer synchronous ERAM.
 always @(posedge clk) previous_data<=previous_line[address];
 always @(posedge clk) if(rst_n&&in_roi) previous_line[address]<={valid_in,rgb_in};

 reg frame_started,previous_complete,line_active,line_good,line_uses_previous;
 reg [15:0] previous_y,line_y,next_x;
 wire start_row=in_roi&&x_in==OFFSET_X&&!line_active;
 wire follows_row=in_roi&&line_active&&y_in==line_y&&x_in==next_x;
 wire previous_for_start=frame_started&&y_in!=0&&previous_complete&&previous_y+16'd1==y_in;
 wire previous_for_input=in_roi&&(start_row?previous_for_start:
      (follows_row&&line_good&&line_uses_previous));
 always @(posedge clk)begin
  if(!rst_n)begin
   frame_started<=0;previous_complete<=0;line_active<=0;line_good<=0;line_uses_previous<=0;
   previous_y<=0;line_y<=0;next_x<=0;
  end else if(de_in&&x_in==0&&y_in==0)begin
   frame_started<=1;previous_complete<=0;line_active<=0;line_good<=0;line_uses_previous<=0;
  end else if(in_roi)begin
   if(start_row)begin
    // Freeze eligibility for THIS row before clearing the reusable RAM tag.
    line_y<=y_in;next_x<=x_in+16'd1;line_good<=1;
    line_uses_previous<=previous_for_start;previous_complete<=0;
    if(WIDTH==1)begin previous_y<=y_in;previous_complete<=frame_started;line_active<=0;end
    else line_active<=1;
   end else if(line_active)begin
    next_x<=x_in+16'd1;
    if(!follows_row)begin line_good<=0;line_uses_previous<=0;previous_complete<=0;end
    if(x_in==OFFSET_X+WIDTH-1)begin
     previous_y<=y_in;previous_complete<=frame_started&&line_good&&follows_row;
     line_active<=0;line_uses_previous<=0;
    end
   end else begin
    // A row that did not start at its first ROI coordinate cannot own old RAM.
    previous_complete<=0;line_uses_previous<=0;
   end
  end else if(line_active)begin
   // Missing DE/coordinate segments invalidate both the unfinished row and
   // any remaining previous-row reads; valid0 with DE=1 is instead written.
   previous_complete<=0;line_active<=0;line_good<=0;line_uses_previous<=0;
  end
 end

 reg [23:0] s0_rgb,center_rgb,left_rgb,center_previous,left_previous;
 reg [15:0] s0_x,s0_y,center_x,center_y,left_x,left_y;
 reg s0_de,s0_valid,s0_hs,s0_vs,s0_enable,s0_previous_allowed;
 reg center_de,center_valid,center_hs,center_vs,center_enable,center_previous_allowed,center_previous_valid;
 reg left_valid,left_previous_allowed,left_previous_valid;
 wire current_neighbors=left_valid&&center_valid&&center_de&&s0_valid&&s0_de&&
   left_y==center_y&&s0_y==center_y&&left_x+16'd1==center_x&&center_x+16'd1==s0_x&&
   left_x>=OFFSET_X&&s0_x<OFFSET_X+WIDTH&&center_y<HEIGHT;
 wire previous_neighbors=current_neighbors&&left_previous_allowed&&center_previous_allowed&&s0_previous_allowed&&
   left_previous_valid&&center_previous_valid&&previous_data[24];

 function close3;
  input [7:0] a,b,c;reg [7:0] high,low;
  begin high=a>b?a:b;high=high>c?high:c;low=a<b?a:b;low=low<c?low:c;
   close3=({1'b0,high}-{1'b0,low})<=THRESHOLD_1D;end
 endfunction
 function close6;
  input [7:0] a,b,c,d,e,f;reg [7:0] high,low;
  begin
   high=a>b?a:b;high=high>c?high:c;high=high>d?high:d;high=high>e?high:e;high=high>f?high:f;
   low=a<b?a:b;low=low<c?low:c;low=low<d?low:d;low=low<e?low:e;low=low<f?low:f;
   close6=({1'b0,high}-{1'b0,low})<=THRESHOLD_2D;
  end
 endfunction
 function [7:0] smooth3;
  input [7:0] a,b,c;reg [9:0] total;
  begin total={2'b0,a}+{1'b0,b,1'b0}+{2'b0,c}+10'd2;smooth3=total[9:2];end
 endfunction
 function [7:0] smooth6;
  input [7:0] a,b,c,d,e,f;reg [10:0] total;
  begin total={3'b0,a}+{2'b0,b,1'b0}+{3'b0,c}+
    {3'b0,d}+{2'b0,e,1'b0}+{3'b0,f}+11'd4;smooth6=total[10:3];end
 endfunction
 wire flat3=close3(left_rgb[23:16],center_rgb[23:16],s0_rgb[23:16])&&
   close3(left_rgb[15:8],center_rgb[15:8],s0_rgb[15:8])&&close3(left_rgb[7:0],center_rgb[7:0],s0_rgb[7:0]);
 wire flat6=close6(left_previous[23:16],center_previous[23:16],previous_data[23:16],left_rgb[23:16],center_rgb[23:16],s0_rgb[23:16])&&
   close6(left_previous[15:8],center_previous[15:8],previous_data[15:8],left_rgb[15:8],center_rgb[15:8],s0_rgb[15:8])&&
   close6(left_previous[7:0],center_previous[7:0],previous_data[7:0],left_rgb[7:0],center_rgb[7:0],s0_rgb[7:0]);
 always @(posedge clk)begin
  if(!rst_n)begin
   s0_rgb<=0;s0_x<=0;s0_y<=0;s0_de<=0;s0_valid<=0;s0_hs<=1;s0_vs<=1;s0_enable<=0;s0_previous_allowed<=0;
   center_rgb<=0;left_rgb<=0;center_previous<=0;left_previous<=0;
   center_x<=0;center_y<=0;left_x<=0;left_y<=0;
   center_de<=0;center_valid<=0;center_hs<=1;center_vs<=1;center_enable<=0;
   center_previous_allowed<=0;center_previous_valid<=0;left_valid<=0;left_previous_allowed<=0;left_previous_valid<=0;
   rgb_out<=0;x_out<=0;y_out<=0;de_out<=0;valid_out<=0;hs_out<=1;vs_out<=1;
  end else begin
   s0_rgb<=rgb_in;s0_x<=x_in;s0_y<=y_in;s0_de<=de_in;s0_valid<=valid_in;
   s0_hs<=hs_in;s0_vs<=vs_in;s0_enable<=enable;s0_previous_allowed<=previous_for_input;
   center_rgb<=s0_rgb;center_x<=s0_x;center_y<=s0_y;center_de<=s0_de;center_valid<=s0_valid;
   center_hs<=s0_hs;center_vs<=s0_vs;center_enable<=s0_enable;
   center_previous<=previous_data[23:0];center_previous_valid<=previous_data[24];center_previous_allowed<=s0_previous_allowed;
   left_rgb<=center_rgb;left_x<=center_x;left_y<=center_y;left_valid<=center_valid&&center_de;
   left_previous<=center_previous;left_previous_valid<=center_previous_valid;left_previous_allowed<=center_previous_allowed;
   if(center_enable&&previous_neighbors&&flat6)rgb_out<={
    smooth6(left_previous[23:16],center_previous[23:16],previous_data[23:16],left_rgb[23:16],center_rgb[23:16],s0_rgb[23:16]),
    smooth6(left_previous[15:8],center_previous[15:8],previous_data[15:8],left_rgb[15:8],center_rgb[15:8],s0_rgb[15:8]),
    smooth6(left_previous[7:0],center_previous[7:0],previous_data[7:0],left_rgb[7:0],center_rgb[7:0],s0_rgb[7:0])};
   else if(center_enable&&current_neighbors&&flat3)rgb_out<={
    smooth3(left_rgb[23:16],center_rgb[23:16],s0_rgb[23:16]),
    smooth3(left_rgb[15:8],center_rgb[15:8],s0_rgb[15:8]),smooth3(left_rgb[7:0],center_rgb[7:0],s0_rgb[7:0])};
   else rgb_out<=center_rgb;
   x_out<=center_x;y_out<=center_y;de_out<=center_de;valid_out<=center_valid;hs_out<=center_hs;vs_out<=center_vs;
  end
 end
 // synthesis translate_off
 initial if(WIDTH<1||HEIGHT<1||OFFSET_X<0||OFFSET_X+WIDTH>65535||THRESHOLD_1D<0||THRESHOLD_1D>255||THRESHOLD_2D<0||THRESHOLD_2D>255)
  $fatal(1,"Invalid two-row image filter parameters");
 // synthesis translate_on
endmodule
