`timescale 1ns/1ps
// Native1280x720 Z16 diagnostic export. Each logical row is exactly two
// physical1280-word result rows; each128-bit beat contains four Z/RGB points.
// The existing global export HOLD must remain confirmed through completion.
module hd_depth_export #(parameter TIMEOUT=50000000)(
 input wire host_clk,memory_clk,rst_n,
 input wire host_request,input wire[9:0]host_row,input wire[31:0]host_expected_job,
 input wire[11:0]host_read_address,output wire[7:0]host_read_data,
 output wire host_busy,output reg host_valid,output reg[3:0]host_error,output reg host_overrun,
 output reg[31:0]host_job,output reg[9:0]host_returned_row,
 input wire hold_confirmed,
 output wire line_request,input wire line_ready,output wire line_new,output wire[15:0]line_row,
 input wire line_write,input wire[127:0]line_data,input wire[7:0]line_address,
 input wire line_done,line_good,input wire[31:0]line_job,line_session
);
 localparam IDLE=0,REQUEST=1,READ_LINE=2;
 reg[63:0]payload[0:319];reg[63:0]host_word;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg request,ack;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]request_sync,ack_sync,host_reset,memory_reset;
 reg request_seen,ack_seen;reg[9:0]row_mailbox,target_row,return_row;
 reg[31:0]job_mailbox,target_job,return_job,age;reg return_good,half,bad_order;
 reg[3:0]return_error;reg[1:0]state;reg[8:0]beats;
 // One memory-clock input stage cuts the DDR response -> completion path.
 reg event_write,event_done,event_good;
 reg[63:0]event_z;reg[7:0]event_address;
 reg[31:0]event_job,event_session;
 always @(posedge memory_clk or negedge memory_reset[1])begin
  if(!memory_reset[1])begin event_write<=0;event_done<=0;event_good<=0;end
  else begin event_write<=line_write;event_done<=line_done;event_good<=line_good;end
 end
 always @(posedge memory_clk)begin
  if(line_write)begin
   event_z<={line_data[127:112],line_data[95:80],line_data[63:48],line_data[31:16]};
   event_address<=line_address;
  end
  if(line_done)begin event_job<=line_job;event_session<=line_session;end
 end
 always @(posedge host_clk or negedge rst_n)if(!rst_n)host_reset<=0;else host_reset<={host_reset[0],1'b1};
 always @(posedge memory_clk or negedge rst_n)if(!rst_n)memory_reset<=0;else memory_reset<={memory_reset[0],1'b1};
 assign host_busy=!host_reset[1]||request!=ack_sync[1]||ack_seen!=ack_sync[1];
 always @(posedge host_clk)host_word<=payload[host_read_address<2560?host_read_address[11:3]:9'd0];
 assign host_read_data=host_valid&&!host_busy&&host_read_address<2560?(host_word>>(host_read_address[2:0]*8)):8'd0;
 assign line_request=state==REQUEST&&hold_confirmed&&target_row<720&&target_job!=0;assign line_new=target_row==0&&!half;
 assign line_row={5'd0,target_row,half};
 always @(posedge host_clk or negedge host_reset[1])begin
  if(!host_reset[1])begin request<=0;ack_sync<=0;ack_seen<=0;row_mailbox<=0;job_mailbox<=0;host_valid<=0;host_error<=0;host_overrun<=0;host_job<=0;host_returned_row<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(host_request)begin
    if(host_busy)host_overrun<=1;
    else begin row_mailbox<=host_row;job_mailbox<=host_expected_job;request<=!request;host_valid<=0;host_error<=0;host_overrun<=0;end
   end
   if(ack_sync[1]!=ack_seen)begin ack_seen<=ack_sync[1];host_valid<=return_good;host_error<=return_error;host_job<=return_job;host_returned_row<=return_row;end
  end
 end
 task complete(input good,input[3:0]reason,input[31:0]job);
 begin state<=IDLE;return_good<=good;return_error<=reason;return_job<=job;return_row<=target_row;ack<=request_seen;end
 endtask
 always @(posedge memory_clk or negedge memory_reset[1])begin
  if(!memory_reset[1])begin request_sync<=0;request_seen<=0;ack<=0;target_row<=0;target_job<=0;return_row<=0;return_job<=0;return_good<=0;return_error<=0;age<=0;half<=0;bad_order<=0;state<=IDLE;beats<=0;end
  else begin
   request_sync<={request_sync[0],request};
   if(request_sync[1]!=request_seen)begin
    request_seen<=request_sync[1];target_row<=row_mailbox;target_job<=job_mailbox;age<=0;half<=0;bad_order<=0;beats<=0;state<=REQUEST;
   end else if(state!=IDLE)begin
    age<=age+1'b1;
    if(state==REQUEST)begin
     if(!hold_confirmed)complete(0,1,0);
     else if(target_row>=720)complete(0,2,0);
     else if(target_job==0)complete(0,4,0);
     else if(line_ready)begin state<=READ_LINE;beats<=0;bad_order<=0;end
    end else if(state==READ_LINE)begin
     // Once accepted, finish consuming that line even if HOLD is withdrawn.
     // The outer arbiter retains posted DDR commands independently of this tap.
     if(!hold_confirmed)bad_order<=1;
     if(event_write)begin
      if(beats>=160||event_address!=beats)bad_order<=1;
      else payload[(half?160:0)+event_address]<=event_z;
      if(beats<=160)beats<=beats+1'b1;
     end
     if(event_done)begin
      if(!hold_confirmed)complete(0,1,event_job);
      else if(bad_order||(event_write&&(beats>=160||event_address!=beats)))complete(0,3,event_job);
      else if(!event_good||beats+(event_write?1:0)!=160)complete(0,6,event_job);
      else if(event_job!=target_job)complete(0,4,event_job);
      else if(event_session!=32'hD2800001)complete(0,5,event_job);
      else if(!half)begin half<=1;state<=REQUEST;end
      else complete(1,0,event_job);
     end
    end
    if(age==TIMEOUT-1)complete(0,7,0);
   end
  end
 end
endmodule
