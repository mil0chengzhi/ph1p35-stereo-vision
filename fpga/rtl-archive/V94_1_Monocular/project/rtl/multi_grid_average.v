`timescale 1ns/1ps
// Four by four RGB565 mean with full eight-bit output precision. Repacking
// this mean into RGB565 before low-contrast line detection destroys details.
module multi_grid_average #(
 parameter OFFSET_X=160,OFFSET_Y=0,IMAGE_WIDTH=960,IMAGE_HEIGHT=720
)(input wire clk,rst_n,input wire [15:0] x,y,input wire de,valid,
 input wire [15:0] rgb565,output reg [23:0] averaged_rgb888,output reg averaged_valid);
 localparam COLS=IMAGE_WIDTH/4,IB=$clog2(COLS);
 wire[15:0] rx=x-OFFSET_X,ry=y-OFFSET_Y;
 wire roi=de&&x>=OFFSET_X&&x<OFFSET_X+IMAGE_WIDTH&&y>=OFFSET_Y&&y<OFFSET_Y+IMAGE_HEIGHT;
 wire[IB-1:0] column=rx>>2;
 wire[7:0] r={rgb565[15:11],rgb565[15:13]},g={rgb565[10:5],rgb565[10:9]},b={rgb565[4:0],rgb565[4:2]};
 reg[36:0] sums[0:COLS-1];reg[36:0] previous;
 reg[9:0] row_r,row_g,row_b;reg row_valid;
 wire[11:0] total_r=(ry[1:0]==0?12'd0:previous[35:24])+row_r+r;
 wire[11:0] total_g=(ry[1:0]==0?12'd0:previous[23:12])+row_g+g;
 wire[11:0] total_b=(ry[1:0]==0?12'd0:previous[11:0])+row_b+b;
 wire cell_valid=row_valid&&valid&&(ry[1:0]==0||previous[36]);
 wire[11:0] rounded_r=total_r+12'd8,rounded_g=total_g+12'd8,rounded_b=total_b+12'd8;
 always @(posedge clk)begin
  if(!rst_n)begin averaged_rgb888<=0;averaged_valid<=0;end
  else begin
   averaged_valid<=roi&&rx[1:0]==3&&ry[1:0]==3&&cell_valid;
   if(roi&&rx[1:0]==3&&ry[1:0]==3)averaged_rgb888<={rounded_r[11:4],rounded_g[11:4],rounded_b[11:4]};
  end
  if(roi&&rx[1:0]==0)previous<=sums[column];
  if(rst_n&&roi&&rx[1:0]==3)sums[column]<={cell_valid,total_r,total_g,total_b};
  if(!rst_n)begin row_r<=0;row_g<=0;row_b<=0;row_valid<=0;end
  else if(roi)begin
   if(rx[1:0]==0)begin row_r<=r;row_g<=g;row_b<=b;row_valid<=valid;end
   else if(rx[1:0]!=3)begin row_r<=row_r+r;row_g<=row_g+g;row_b<=row_b+b;row_valid<=row_valid&&valid;end
  end
 end
endmodule
