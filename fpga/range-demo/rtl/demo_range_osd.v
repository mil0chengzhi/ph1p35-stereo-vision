`timescale 1ns/1ps
// Pixel-domain red target box + one-line ruler/rangefinder distance overlay.
// Upstream pixels are the real local B camera, before OSD.
module demo_range_osd(input wire clk,rst_n,enable,number_valid,input wire[11:0]digits,
 input wire[15:0]x,y,input wire[23:0]rgb,input wire de,hs,vs,
 output reg[23:0]out_rgb,output reg out_de,out_hs,out_vs);
 wire red=de&&rgb[23:16]>=80&&{2'd0,rgb[23:16]}>({2'd0,rgb[15:8]}+({2'd0,rgb[15:8]}>>2))&&
        {2'd0,rgb[23:16]}>({2'd0,rgb[7:0]}+({2'd0,rgb[7:0]}>>2));
 reg[15:0]minx,miny,maxx,maxy,bx0,by0,bx1,by1;reg[19:0]red_count;reg found;
 always @(posedge clk)begin
  if(!rst_n)begin minx<=1280;miny<=720;maxx<=0;maxy<=0;red_count<=0;found<=0;bx0<=0;by0<=0;bx1<=0;by1<=0;end
  else if(x==0&&y==720)begin
   found<=red_count>=100&&maxx>minx&&maxy>miny;bx0<=minx;by0<=miny;bx1<=maxx;by1<=maxy;
   minx<=1280;miny<=720;maxx<=0;maxy<=0;red_count<=0;
  end else if(enable&&red)begin
   if(x<minx)minx<=x;if(x>maxx)maxx<=x;if(y<miny)miny<=y;if(y>maxy)maxy<=y;
   if(red_count!=20'hfffff)red_count<=red_count+1'b1;
  end
 end
 wire box=enable&&found&&de&&x>=bx0&&x<=bx1&&y>=by0&&y<=by1&&
       ((x-bx0<2)||(bx1-x<2)||(y-by0<2)||(by1-y<2));
 wire panel=enable&&de&&x>=16&&x<424&&y>=16&&y<72;
 wire text=panel&&x>=24&&x<408&&y>=24&&y<56;
 wire[3:0]cell_index=(x-24)>>5;wire[3:0]gx=(x-24)>>1,gy=(y-24)>>1;
 reg[4:0]glyph;
 always @*begin
  glyph=0;
  case(cell_index)
   0:glyph=14;1:glyph=15;2:glyph=16;3:glyph=17;
   5:glyph=number_valid?{1'b0,digits[11:8]}+1:18;
   6:glyph=number_valid?{1'b0,digits[7:4]}+1:18;
   7:glyph=11;
   8:glyph=number_valid?{1'b0,digits[3:0]}+1:18;
   10:glyph=12;11:glyph=13;
   default:glyph=0;
  endcase
 end
 wire ink;demo_range_font font(.clk(clk),.glyph(glyph),.x(gx),.y(gy),.foreground(ink));
 reg[23:0]rgb_q;reg panel_q,text_q,box_q,de_q,hs_q,vs_q;
 always @(posedge clk)begin
  if(!rst_n)begin rgb_q<=0;panel_q<=0;text_q<=0;box_q<=0;de_q<=0;hs_q<=0;vs_q<=0;out_rgb<=0;out_de<=0;out_hs<=0;out_vs<=0;end
  else begin
   rgb_q<=rgb;panel_q<=panel;text_q<=text;box_q<=box;de_q<=de;hs_q<=hs;vs_q<=vs;
   out_rgb<=panel_q?(text_q&&ink?24'h81f3db:24'h0d2032):box_q?24'h36ff83:rgb_q;
   out_de<=de_q;out_hs<=hs_q;out_vs<=vs_q;
  end
 end
endmodule
