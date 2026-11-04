`timescale 1ns/1ps
// HC-05 measured-value transport. Never estimates a range from imagery.
// Request:A5 5A seq op valueLE16 reserved0 CRC8(poly07,init0).
module bluetooth_distance_control #(parameter REVISION=100,VALUE_TIMEOUT=250000000)(
 input wire clk,rst_n,rx,output wire tx,input wire[15:0]baud_div,
 input wire[7:0]system_flags,input wire[15:0]requested_token,applied_token,
 input wire[31:0]selected_model,
 output reg[9:0]distance_mm,output reg distance_valid,output reg status_latch,
 output reg[15:0]valid_requests,bad_requests,output wire framing_error
);
 wire[7:0]rx_data,tx_data;wire rx_valid,tx_valid,tx_ready;
 uart_byte_variable uart(.clk(clk),.rst_n(rst_n),.div(baud_div),.rx(rx),.tx(tx),
  .tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),.rx_data(rx_data),.rx_valid(rx_valid),.framing_error(framing_error));
 reg[7:0]frame[0:6];reg[2:0]index,state;reg[7:0]crc,reply_crc,result;
 reg[23:0]age;reg[27:0]value_age;reg[191:0]reply;reg[4:0]send_index;
 function[7:0]crc_byte(input[7:0]previous,input[7:0]value);
  reg[7:0]v;integer i;begin v=previous^value;for(i=0;i<8;i=i+1)v=v[7]?(v<<1)^8'h07:v<<1;crc_byte=v;end
 endfunction
 wire[15:0]input_mm={frame[5],frame[4]};
 assign tx_valid=state==3;
 assign tx_data=send_index==23?reply_crc:reply[191:184];
 integer n;
 always @(posedge clk)begin
  if(!rst_n)begin
   index<=0;state<=0;crc<=0;reply_crc<=0;result<=0;age<=0;value_age<=0;
   reply<=0;send_index<=0;distance_mm<=0;distance_valid<=0;status_latch<=0;valid_requests<=0;bad_requests<=0;
   for(n=0;n<7;n=n+1)frame[n]<=0;
  end else begin
   status_latch<=0;
   if(distance_valid)begin
    if(value_age==VALUE_TIMEOUT-1)distance_valid<=0;else value_age<=value_age+1'b1;
   end
   if(framing_error&&bad_requests!=65535)bad_requests<=bad_requests+1'b1;
   case(state)
    0:begin
     if(index!=0)begin if(age==9999999)begin index<=0;age<=0;end else age<=age+1'b1;end
     if(rx_valid)begin
      age<=0;
      if(index==0)begin if(rx_data==8'ha5)begin frame[0]<=rx_data;crc<=crc_byte(0,rx_data);index<=1;end end
      else if(index==1&&rx_data!=8'h5a)index<=0;
      else if(index==7)begin
       index<=0;
       if(rx_data==crc)begin state<=1;if(valid_requests!=65535)valid_requests<=valid_requests+1'b1;status_latch<=1;end
       else if(bad_requests!=65535)bad_requests<=bad_requests+1'b1;
      end else begin frame[index]<=rx_data;crc<=crc_byte(crc,rx_data);index<=index+1'b1;end
     end
    end
    1:begin
     result<=0;
     case(frame[3])
      8'h00,8'h21:begin end
      8'h20:if(frame[6]==0&&input_mm>=500&&input_mm<=750)begin
       distance_mm<=input_mm[9:0];distance_valid<=1;value_age<=0;
      end else result<=1;
      8'h22:begin distance_valid<=0;value_age<=0;end
      default:result<=3;
     endcase
     state<=2;
    end
    2:begin
     reply<={8'h5a,8'ha5,frame[2],result,8'd1,8'd2,8'd100,8'h30,
      distance_mm[7:0],{6'd0,distance_mm[9:8]},{7'd0,distance_valid},system_flags,
      requested_token[7:0],requested_token[15:8],applied_token[7:0],applied_token[15:8],
      selected_model[7:0],selected_model[15:8],selected_model[23:16],selected_model[31:24],
      bad_requests[7:0],bad_requests[15:8],8'd0,8'd0};
     send_index<=0;reply_crc<=0;state<=3;
    end
    3:if(tx_ready)begin
     if(send_index==23)state<=0;
     else begin reply_crc<=crc_byte(reply_crc,reply[191:184]);reply<={reply[183:0],8'd0};send_index<=send_index+1'b1;end
    end
    default:state<=0;
   endcase
  end
 end
endmodule
