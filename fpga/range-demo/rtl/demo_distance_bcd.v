`timescale 1ns/1ps
// Input is externally measured millimetres. 500..750mm ->50.0..75.0cm.
// Serial conversion keeps the display path out of the main video timing.
module demo_distance_bcd(input wire clk,rst_n,input wire[9:0]millimetres,
 output reg[11:0]digits,output wire ready);
 reg[9:0]seen,shift;reg[11:0]bcd;reg[3:0]remaining;reg busy;
 reg[11:0]adjusted;integer n;
 assign ready=!busy&&seen==millimetres;
 always @*begin adjusted=bcd;for(n=0;n<3;n=n+1)if(bcd[n*4+:4]>=5)adjusted[n*4+:4]=bcd[n*4+:4]+3;end
 always @(posedge clk)begin
  if(!rst_n)begin seen<=1023;shift<=0;bcd<=0;remaining<=0;busy<=0;digits<=0;end
  else if(!busy&&seen!=millimetres)begin seen<=millimetres;shift<=millimetres;bcd<=0;remaining<=10;busy<=1;end
  else if(busy)begin
   bcd<={adjusted[10:0],shift[9]};shift<={shift[8:0],1'b0};remaining<=remaining-1'b1;
   if(remaining==1)begin digits<={adjusted[10:0],shift[9]};busy<=0;end
  end
 end
endmodule
