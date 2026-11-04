`timescale 1ns/1ps
// HC-05 control channel. 8-byte requests and CRC8 protected 24-byte replies.
// A5 5A seq op yaw pitch zoomQ4 CRC8. No generic register or motor writes.
module bluetooth_view_control #(parameter REVISION=76)(
 input wire clk,rst_n,input wire rx,output wire tx,input wire[15:0]baud_div,
 input wire capture_ready,input wire[7:0]system_flags,input wire[31:0]depth_job,
 input wire[15:0]applied_token,
 output reg capture_request,exit_request,status_latch,
 output wire[47:0]view_record,output reg view_commit,
 output reg[15:0]valid_requests,bad_requests,output wire framing_error
);
 wire[7:0]rx_data;wire rx_valid,tx_ready;wire[7:0]tx_data;wire tx_valid;
 uart_byte_variable uart(.clk(clk),.rst_n(rst_n),.div(baud_div),.rx(rx),.tx(tx),
  .tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),.rx_data(rx_data),.rx_valid(rx_valid),.framing_error(framing_error));
 reg[7:0]frame[0:6];reg[2:0]index;reg[7:0]crc,reply_crc;
 reg[23:0]age;reg[2:0]state;reg[5:0]wait_count;reg[4:0]send_index;
 reg[191:0]reply;reg[7:0]result;
 reg[15:0]token;reg signed[7:0]yaw,pitch;reg[7:0]zoom;reg auto_mode;
 localparam[7:0]BUILD_REVISION=REVISION;
 assign view_record={7'd0,token,auto_mode,zoom,pitch,yaw};
 function[7:0]crc_byte(input[7:0]previous,input[7:0]value);
  reg[7:0]v;integer i;begin v=previous^value;for(i=0;i<8;i=i+1)v=v[7]?(v<<1)^8'h07:v<<1;crc_byte=v;end
 endfunction
 assign tx_valid=state==4;
 assign tx_data=send_index==23?reply_crc:reply[191:184];
 wire pose_ok=$signed(frame[4])>=-24&&$signed(frame[4])<=24&&frame[4][1:0]==0&&$signed(frame[5])>=-24&&$signed(frame[5])<=24&&frame[5][1:0]==0&&frame[6]>=8&&frame[6]<=24;
 integer n;
 always @(posedge clk)begin
  if(!rst_n)begin
   index<=0;crc<=0;reply_crc<=0;age<=0;state<=0;wait_count<=0;send_index<=0;reply<=0;result<=0;
   token<=0;yaw<=0;pitch<=16;zoom<=16;auto_mode<=0;view_commit<=0;
   capture_request<=0;exit_request<=0;status_latch<=0;valid_requests<=0;bad_requests<=0;
   for(n=0;n<7;n=n+1)frame[n]<=0;
  end else begin
   view_commit<=0;capture_request<=0;exit_request<=0;status_latch<=0;
   if(framing_error&&bad_requests!=65535)bad_requests<=bad_requests+1'b1;
   case(state)
    0:begin
     if(index!=0)begin if(age==9999999)begin index<=0;age<=0;end else age<=age+1'b1;end
     if(rx_valid)begin
      age<=0;
      if(index==0)begin if(rx_data==8'ha5)begin frame[0]<=rx_data;crc<=crc_byte(0,rx_data);index<=1;end end
      else if(index==1&&rx_data!=8'h5a)begin index<=0;end
      else if(index==7)begin
       index<=0;
       if(rx_data==crc)begin state<=1;valid_requests<=valid_requests+1'b1;status_latch<=1;end
       else if(bad_requests!=65535)bad_requests<=bad_requests+1'b1;
      end else begin frame[index]<=rx_data;crc<=crc_byte(crc,rx_data);index<=index+1'b1;end
     end
    end
    1:begin
     result<=0;wait_count<=0;
     case(frame[3])
      0:begin end
      1:if(pose_ok&&token!=65535)begin yaw<=frame[4];pitch<=frame[5];zoom<=frame[6];auto_mode<=0;token<=token+1'b1;state<=2;end else result<=1;
      2:if(frame[4]<=1&&token!=65535)begin auto_mode<=frame[4][0];token<=token+1'b1;state<=2;end else result<=1;
      3:if(token!=65535)begin yaw<=0;pitch<=16;zoom<=16;auto_mode<=0;token<=token+1'b1;state<=2;end else result<=1;
      4:if(capture_ready)capture_request<=1;else result<=2;
      5:exit_request<=1;
      default:result<=3;
     endcase
     if(frame[3]==0||frame[3]>=4||((frame[3]==1&&!pose_ok)||token==65535)||(frame[3]==2&&frame[4]>1))state<=3;
    end
    2:begin view_commit<=1;state<=3;end
    3:if(wait_count!=63)wait_count<=wait_count+1'b1;
      else begin
       // Immutable response. Requested != applied means A HDMI has not ACKed.
       reply<={8'h5a,8'ha5,frame[2],result,8'd1,8'd2,BUILD_REVISION,8'h0f,
        yaw,pitch,zoom,{system_flags[7:3],auto_mode,system_flags[1:0]},token[7:0],token[15:8],applied_token[7:0],applied_token[15:8],
        depth_job[7:0],depth_job[15:8],depth_job[23:16],depth_job[31:24],bad_requests[7:0],bad_requests[15:8],8'd0,8'd0};
       send_index<=0;reply_crc<=0;state<=4;
      end
    4:if(tx_ready)begin
     if(send_index==23)state<=0;
     else begin reply_crc<=crc_byte(reply_crc,reply[191:184]);reply<={reply[183:0],8'd0};send_index<=send_index+1'b1;end
    end
    default:state<=0;
   endcase
  end
 end
endmodule
