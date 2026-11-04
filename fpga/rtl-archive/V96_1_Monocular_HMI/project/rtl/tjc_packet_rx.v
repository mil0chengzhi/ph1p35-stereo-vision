`timescale 1ns/1ps
// TJC sendme(66) and sendxy(67) packets; validate length, suffix and bounds.
module tjc_packet_rx #(parameter GAP_CYCLES=1000000)(
 input wire clk,rst_n,input wire[7:0]data,input wire valid,framing_error,
 output reg page_valid,output reg[2:0]page,
 output reg touch_valid,touch_down,output reg[15:0]touch_x,touch_y,
 output reg[31:0]bytes_received,packets_received,framing_errors
);
 reg[3:0]index;reg kind,bad;reg[19:0]gap;
 reg[15:0]px,py;reg down;reg[2:0]pg;
 always @(posedge clk)begin
  if(!rst_n)begin index<=0;kind<=0;bad<=0;gap<=0;px<=0;py<=0;down<=0;pg<=0;
   page_valid<=0;page<=0;touch_valid<=0;touch_down<=0;touch_x<=0;touch_y<=0;
   bytes_received<=0;packets_received<=0;framing_errors<=0;end
  else begin
   page_valid<=0;touch_valid<=0;
   if(index!=0)begin
    if(gap==GAP_CYCLES-1)begin index<=0;gap<=0;end else gap<=gap+1'b1;
   end else gap<=0;
   if(framing_error)begin index<=0;framing_errors<=framing_errors+1'b1;end
   else if(valid)begin
    bytes_received<=bytes_received+1'b1;gap<=0;
    if(index==0)begin
     if(data==8'h66||data==8'h67)begin kind<=data==8'h67;index<=1;bad<=0;end
    end else if(!kind)begin
     if(index==1)begin pg<=data[2:0];if(data>5)bad<=1;end
     else if(data!=8'hff)bad<=1;
     if(index==4)begin
      index<=0;
      if(!bad&&data==8'hff)begin page_valid<=1;page<=pg;packets_received<=packets_received+1'b1;end
     end else index<=index+1'b1;
    end else begin
     case(index)
      1:px[15:8]<=data;
      2:px[7:0]<=data;
      3:py[15:8]<=data;
      4:py[7:0]<=data;
      5:begin down<=data[0];if(data>1)bad<=1;end
      default:if(data!=8'hff)bad<=1;
     endcase
     if(index==8)begin
      index<=0;
      if(!bad&&data==8'hff&&px<480&&py<272)begin
       touch_valid<=1;touch_down<=down;touch_x<=px;touch_y<=py;packets_received<=packets_received+1'b1;
      end
     end else index<=index+1'b1;
    end
   end
  end
 end
endmodule
