`timescale 1ns/1ps
// Four registers from input sample to tagged output (N->N+3).
// Parallel live original, grayscale, threshold, Sobel and posterized edge style.
// Recognition/motion consume raw_rgb_out, never the drawn/stylized image.
// No region fill and no additional frame buffer.
module contest_effects_720p(
 input wire clk,rst_n,input wire [2:0] mode,input wire [1:0] adjustment,
 input wire [15:0] x,y,input wire [23:0] rgb,
 input wire de,valid,hs,vs,
 output wire [15:0] out_x,out_y,output wire [23:0] out_rgb,raw_rgb_out,
 output wire out_de,out_valid,out_hs,out_vs,output wire [2:0] out_mode,
 output wire [7:0] edge_strength,output wire edge_valid
);
 wire [15:0] pixel565={rgb[23:19],rgb[15:10],rgb[7:3]};
 video_sobel #(.WIDTH(1280),.HEIGHT(720)) sobel(
  .clk(clk),.rst_n(rst_n),.x(x),.y(y),.de(de),.valid(valid),.rgb565(pixel565),
  .edge_strength(edge_strength),.edge_valid(edge_valid),.out_x(out_x),.out_y(out_y),.out_de(out_de));
 reg [23:0] raw[0:3];reg [2:0] modes[0:3];reg [1:0] params[0:3];
 reg [3:0] valids,hss,vss;integer i;
 always @(posedge clk)begin
  if(!rst_n)begin
   for(i=0;i<4;i=i+1)begin raw[i]<=0;modes[i]<=0;params[i]<=0;end
   valids<=0;hss<=15;vss<=15;
  end else begin
   raw[0]<=rgb;modes[0]<=mode;params[0]<=adjustment;
   for(i=1;i<4;i=i+1)begin raw[i]<=raw[i-1];modes[i]<=modes[i-1];params[i]<=params[i-1];end
   valids<={valids[2:0],valid};hss<={hss[2:0],hs};vss<={vss[2:0],vs};
  end
 end
 wire [7:0] r=raw[0][23:16],g=raw[0][15:8],b=raw[0][7:0];
 // Constant products are explicit shift/add terms, sharing no DSP multipliers.
 wire [15:0] luma_sum=({8'd0,r}<<6)+({8'd0,r}<<3)+({8'd0,r}<<2)+r+
  ({8'd0,g}<<7)+({8'd0,g}<<4)+({8'd0,g}<<2)+({8'd0,g}<<1)+
  ({8'd0,b}<<4)+({8'd0,b}<<3)+({8'd0,b}<<2)+b+16'd128;
 wire [7:0] gray0=luma_sum[15:8];
 reg [7:0] gray1,gray2,gray3;
 reg [23:0] poster2,poster3;
 wire [7:0] gray=gray3;
 // Quantize only luminance. The same signed delta is added to R/G/B,
 // preserving their differences until a channel reaches the output limits.
 function [7:0] poster_component;
  input [7:0] value,luminance;
  reg [7:0] quantized;reg signed [9:0] corrected;
  begin
   quantized={luminance[7:5],5'b10000};
   corrected=$signed({2'b00,value})+$signed({2'b00,quantized})-$signed({2'b00,luminance});
   if(corrected<0)poster_component=0;
   else if(corrected>255)poster_component=255;
   else poster_component=corrected[7:0];
  end
 endfunction
 always @(posedge clk)begin
  if(!rst_n)begin gray1<=0;gray2<=0;gray3<=0;poster2<=0;poster3<=0;end
  else begin
   gray1<=gray0;gray2<=gray1;gray3<=gray2;
   poster2<={poster_component(raw[1][23:16],gray1),poster_component(raw[1][15:8],gray1),poster_component(raw[1][7:0],gray1)};
   poster3<=poster2;
  end
 end
 wire [7:0] binary_threshold=8'd96+{1'b0,params[3],5'b0};
 wire [7:0] edge_threshold=8'd64+{1'b0,params[3],5'b0};
 wire [23:0] poster=poster3;
 reg [23:0] processed;
 always @*begin
  processed=raw[3];
  case(modes[3])
   2:processed={3{gray}};
   3:processed=gray>=binary_threshold?24'hffffff:24'h000000;
   4:processed=edge_valid?{3{edge_strength}}:24'd0;
   5:processed=edge_valid&&edge_strength>=edge_threshold?24'd0:poster;
   default:processed=raw[3];
  endcase
 end
 assign out_rgb=valids[3]&&out_de?processed:24'd0;
 assign raw_rgb_out=raw[3];assign out_mode=modes[3];
 assign out_valid=valids[3];assign out_hs=hss[3];assign out_vs=vss[3];
endmodule
