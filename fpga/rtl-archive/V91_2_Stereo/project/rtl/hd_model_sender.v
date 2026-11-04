`timescale 1ns/1ps
// Stop-and-wait model transport. The result store freezes its front on row0;
// all1440 physical rows must retain that job. Completion means A has published
// its first complete720p view, not merely that the last packet was sent.
module hd_model_sender #(parameter RETRY_CYCLES=1000000,MAX_RETRIES=255,CANCEL_PROTOCOL=0)(
 input wire clk,rst_n,enable,cancel_job,input wire[31:0]result_published,
 output reg busy,done,fault,completed_canceled,output reg[31:0]completed_job,retries,
 output wire line_request,input wire line_ready,output wire line_new,output wire[15:0]line_row,output wire line_bank,
 input wire line_write,input wire[127:0]line_data,input wire[7:0]line_address,input wire line_done,line_good,input wire[31:0]line_job,
 output wire payload_write,output wire[10:0]payload_address,output wire[7:0]payload_data,
 output wire send_valid,input wire send_ready,output wire[7:0]send_kind,output wire[31:0]send_job,output wire[15:0]send_sequence,
 input wire reply_valid,input wire[191:0]reply_header,output wire reply_consume
);
 localparam IDLE=0,REQUEST=1,WAIT_LINE=2,READ_BEAT=3,LOAD_BEAT=4,COPY=5,SEND=6,WAIT_ACK=7,POLL=8,WAIT_PUBLISH=9,FAILED=10,BEGIN_SEND=11,WAIT_BEGIN=12;
 reg[3:0]state;reg canceled,poll_canceled;
 wire cancel_requested=canceled||(CANCEL_PROTOCOL&&cancel_job);reg[127:0]line_ram[0:159];reg[127:0]read_q; // Held until the next READ_BEAT; COPY needs no duplicate.
 reg[31:0]job,age;reg[15:0]sequence_q;reg[6:0]beat;reg[3:0]lane;reg[7:0]attempts;
 wire[7:0]reply_kind=reply_header[151:144];wire[31:0]reply_job=reply_header[127:96];wire[15:0]reply_sequence=reply_header[95:80];
 wire valid_ack=reply_valid&&reply_job==job&&reply_kind==2&&reply_sequence==sequence_q;
 wire valid_begin=reply_valid&&reply_job==job&&reply_kind==2&&reply_sequence==2880;
 wire valid_publish=reply_valid&&reply_job==job&&reply_kind==3&&reply_sequence==(poll_canceled?16'd1:16'd2880);
 assign reply_consume=reply_valid;
 assign line_request=state==REQUEST&&!fault;assign line_new=sequence_q==0;assign line_row=sequence_q>>1;assign line_bank=0;
 assign payload_write=state==COPY&&!fault;assign payload_address={beat,lane};assign payload_data=read_q>>(lane*8);
 assign send_valid=!fault&&(state==SEND||state==POLL||state==BEGIN_SEND);assign send_kind=state==SEND?1:4;
 assign send_job=job;assign send_sequence=state==POLL?(poll_canceled?16'd1:16'd2880):(state==BEGIN_SEND?16'd0:sequence_q);
 always @(posedge clk)begin
  if(line_write&&state==WAIT_LINE&&line_address<160)line_ram[line_address]<=line_data;
  if(state==READ_BEAT)read_q<=line_ram[(sequence_q[0]?80:0)+beat];
 end
 always @(posedge clk)begin
  if(!rst_n)begin state<=IDLE;canceled<=0;poll_canceled<=0;completed_canceled<=0;job<=0;age<=0;sequence_q<=0;beat<=0;lane<=0;attempts<=0;busy<=0;done<=0;fault<=0;completed_job<=0;retries<=0;end
  else begin
   done<=0;
   if(busy&&CANCEL_PROTOCOL&&cancel_job)canceled<=1;
   if(!fault)case(state)
    IDLE:if(enable&&result_published!=0&&result_published!=completed_job)begin job<=result_published;canceled<=CANCEL_PROTOCOL&&cancel_job;poll_canceled<=0;sequence_q<=0;busy<=1;attempts<=0;state<=BEGIN_SEND;end
    BEGIN_SEND:if(send_ready)begin age<=0;state<=WAIT_BEGIN;end
    WAIT_BEGIN:if(valid_begin)begin attempts<=0;state<=REQUEST;end
     else if(age==RETRY_CYCLES-1)begin
      if(attempts==MAX_RETRIES)begin fault<=1;state<=FAILED;end
      else begin attempts<=attempts+1'b1;retries<=retries+1'b1;state<=BEGIN_SEND;end
     end else age<=age+1'b1;
    REQUEST:if(line_ready)state<=WAIT_LINE;
    WAIT_LINE:if(line_done)begin
     if(!line_good||line_job!=job)begin fault<=1;state<=FAILED;end
     else begin beat<=0;lane<=0;state<=READ_BEAT;end
    end
    READ_BEAT:state<=LOAD_BEAT;
    LOAD_BEAT:begin lane<=0;state<=COPY;end
    COPY:if(lane==15)begin if(beat==79)state<=SEND;else begin beat<=beat+1'b1;state<=READ_BEAT;end end else lane<=lane+1'b1;
    SEND:if(send_ready)begin age<=0;state<=WAIT_ACK;end
    WAIT_ACK:if(valid_ack)begin
     attempts<=0;
     if(sequence_q==2879)begin age<=0;poll_canceled<=cancel_requested;state<=POLL;end
     else begin sequence_q<=sequence_q+1'b1;beat<=0;lane<=0;state<=sequence_q[0]?REQUEST:READ_BEAT;end
    end else if(age==RETRY_CYCLES-1)begin
     if(attempts==MAX_RETRIES)begin fault<=1;state<=FAILED;end
     else begin attempts<=attempts+1'b1;retries<=retries+1'b1;state<=SEND;end
    end else age<=age+1'b1;
    POLL:if(send_ready)begin age<=0;state<=WAIT_PUBLISH;end
    WAIT_PUBLISH:if(cancel_requested&&!poll_canceled)begin
      // Finish an already offered normal poll before offering a cancel poll.
      // The descriptor remains stable while send_ready is low.
      poll_canceled<=1;attempts<=0;age<=0;state<=POLL;
     end else if(valid_publish)begin completed_job<=job;completed_canceled<=poll_canceled;busy<=0;done<=1;state<=IDLE;end
     else if(age==RETRY_CYCLES-1)begin
      if(attempts==MAX_RETRIES)begin fault<=1;state<=FAILED;end
      else begin attempts<=attempts+1'b1;retries<=retries+1'b1;state<=POLL;end
     end else age<=age+1'b1;
    FAILED:begin end
    default:begin fault<=1;state<=FAILED;end
   endcase
  end
 end
endmodule
