`timescale 1ns/1ps
// One shared converter; COUNT*16 clocks fit comfortably in HDMI blanking.
// bcd is a held working record, valid on done until the next accepted load.
// Its consumer must capture only on done. Busy loads are ignored.
module ui_decimal_serial_hold #(parameter COUNT=6)(
 input wire clk,rst_n,load,input wire[COUNT*16-1:0]values,
 output reg[COUNT*20-1:0]bcd,output reg busy,done
);
 localparam FB=COUNT>1?$clog2(COUNT):1;
 reg[COUNT*16-1:0]binary;reg[19:0]work,corrected;
 reg[3:0]step;reg[FB-1:0]field;
 wire[19:0]next_bcd={corrected[18:0],binary[COUNT*16-1]};
 integer n;
 always @*begin
  corrected=work;
  for(n=0;n<5;n=n+1)if(work[n*4+:4]>=5)corrected[n*4+:4]=work[n*4+:4]+4'd3;
 end
 always @(posedge clk)begin
  if(!rst_n)begin bcd<=0;busy<=0;done<=0;binary<=0;work<=0;step<=0;field<=0;end
  else begin
   done<=0;
   if(load&&!busy)begin binary<=values;work<=0;step<=0;field<=COUNT-1;busy<=1;end
   if(busy)begin
    binary<=binary<<1;work<=next_bcd;step<=step+1'b1;
    if(step==15)begin
     bcd[field*20+:20]<=next_bcd;work<=0;
     if(field==0)begin busy<=0;done<=1;end else field<=field-1'b1;
    end
   end
  end
 end
 // synthesis translate_off
 initial if(COUNT<1)$fatal(1,"Invalid decimal count");
 // synthesis translate_on
endmodule
