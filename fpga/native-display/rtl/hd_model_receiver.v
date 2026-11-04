`timescale 1ns/1ps
// CRC-qualified HD72 packets -> immutable double-buffered native720 model.
// Packet ownership is retained through a DDR readback fence. No partial model
// is published, and the model currently being read is never overwritten.
module hd_model_receiver #(parameter BASE=16777216,STRIDE=2097152,TIMEOUT=200000,ALLOW_CANCEL=0)(
 input wire clk,rst_n,enable,
 input wire packet_valid,input wire[191:0]packet_header,output wire packet_consume,
 output wire[10:0]payload_address,input wire[7:0]payload_data,
 output wire reply_valid,input wire reply_ready,output reg[7:0]reply_kind,
 output reg[31:0]reply_job,output reg[15:0]reply_sequence,
 input wire reader_busy,input wire reader_bank,input wire[31:0]published_job,rendered_job,
 output reg latest_valid,output reg[31:0]latest_job,bank0_job,bank1_job,
 output reg[31:0]latest_capture_job,
 output reg busy,fault,output reg[31:0]accepted_packets,duplicate_packets,
 output wire[24:0]awaddr,araddr,output wire[127:0]wdata,output wire[15:0]wstrb,
 output wire awvalid,wvalid,wlast,arvalid,rready,read_active,
 input wire awready,wready,arready,input wire[127:0]rdata,input wire rvalid,rlast
);
 localparam IDLE=0,BYTE_ADDRESS=1,BYTE_CAPTURE=2,WRITE=3,FENCE_AR=4,FENCE_R=5,REPLY=6,CONSUME=7,FAILED=8;
 reg[3:0]state;reg[31:0]job;reg[(TIMEOUT>1?$clog2(TIMEOUT):1)-1:0]age;reg bank;reg[15:0]expected_sequence,saved_sequence;
 // Remote capture ordinals can restart at one after a B-only reset.
 // Local generations identify immutable models independently of those ordinals.
 reg begin_pending;reg[31:0]begin_job,version_counter,write_version;
 reg[6:0]beat;reg[3:0]lane;reg[127:0]word_q; // Last packet beat stays held through the readback fence.
 reg aw_done,w_done,command_live;reg[24:0]address_q;
 wire[7:0]kind=packet_header[151:144];wire[31:0]incoming_job=packet_header[127:96];wire[15:0]incoming_sequence=packet_header[95:80];
 wire data_packet=kind==1&&incoming_sequence<2880;
 wire begin_data=begin_pending&&incoming_job==begin_job&&incoming_sequence==0;
 wire next_bank=latest_valid?(latest_job==bank0_job):1'b0;
 wire target_available=!reader_busy||reader_bank!=next_bank;
 assign packet_consume=state==CONSUME;
 assign payload_address={beat,lane};
 assign reply_valid=state==REPLY;
 assign awaddr=address_q;assign wdata=word_q;assign wstrb=0;
 assign awvalid=rst_n&&state==WRITE&&!aw_done;assign wvalid=rst_n&&state==WRITE&&!w_done;assign wlast=1;
 assign araddr=address_q;assign arvalid=rst_n&&command_live;
 assign rready=rst_n&&(state==FENCE_R||fault);
 assign read_active=command_live||state==FENCE_R;
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;job<=0;bank<=0;expected_sequence<=0;saved_sequence<=0;beat<=0;lane<=0;word_q<=0;
   aw_done<=0;w_done<=0;command_live<=0;address_q<=0;age<=0;reply_kind<=0;reply_job<=0;reply_sequence<=0;
   latest_valid<=0;latest_job<=0;bank0_job<=0;bank1_job<=0;busy<=0;fault<=0;accepted_packets<=0;duplicate_packets<=0;
   latest_capture_job<=0;begin_pending<=0;begin_job<=0;version_counter<=0;write_version<=0;
  end else begin
   if(state==WRITE||read_active)begin if(age==TIMEOUT-1)fault<=1;else age<=age+1'b1;end else age<=0;
   if((awvalid&&awready)||(wvalid&&wready)||(arvalid&&arready)||(rvalid&&rready))age<=0;
   if(arvalid&&arready)begin command_live<=0;state<=FENCE_R;end
   // A failure drains every previously posted command before stopping.
   if(fault)begin
    if(state==WRITE)begin
     if(awvalid&&awready)aw_done<=1;if(wvalid&&wready)w_done<=1;
     if((aw_done||(awvalid&&awready))&&(w_done||(wvalid&&wready)))state<=FAILED;
    end else if(state==FENCE_R&&rvalid&&rready)state<=FAILED;
    else if(!read_active)state<=FAILED;
   end else case(state)
    IDLE:if(enable&&packet_valid)begin
     reply_job<=incoming_job;reply_sequence<=incoming_sequence;
     if(kind==4&&incoming_sequence==0)begin
      // BEGIN ACK is distinct from every DATA ACK and publication ACK.
      // Retries before DATA0 are idempotent; BEGIN never publishes a model.
      begin_pending<=1;begin_job<=incoming_job;
      reply_kind<=2;reply_sequence<=2880;state<=REPLY;
     end else if(ALLOW_CANCEL&&kind==4&&incoming_sequence==1)begin
      // CANCEL-DRAIN is distinct from a selected full-HDMI publication.
      if(!busy&&!begin_pending&&latest_valid&&incoming_job==latest_capture_job&&rendered_job==latest_job)begin reply_kind<=3;reply_sequence<=1;state<=REPLY;end
      else state<=CONSUME;
     end else if(kind==4&&incoming_sequence==2880)begin
      if(!busy&&!begin_pending&&latest_valid&&incoming_job==latest_capture_job&&published_job==latest_job)begin reply_kind<=3;reply_sequence<=2880;state<=REPLY;end
      else state<=CONSUME;
     end else if(!data_packet)state<=CONSUME;
     else if(begin_data)begin
      if(target_available)begin
       job<=incoming_job;write_version<=version_counter+1'b1;begin_pending<=0;
       bank<=next_bank;expected_sequence<=0;saved_sequence<=0;beat<=0;lane<=0;busy<=1;
       // Invalidate the destination before the first write, preserving the
       // independently held latest bank and reader's immutable bank.
       if(next_bank)bank1_job<=0;else bank0_job<=0;
       address_q<=BASE+(next_bank?STRIDE:0);state<=BYTE_ADDRESS;
      end else state<=CONSUME;
     end else if(!busy&&!begin_pending&&latest_valid&&incoming_job==latest_capture_job)begin
      reply_kind<=2;duplicate_packets<=duplicate_packets+1'b1;state<=REPLY;
     end else if(!busy||begin_pending||incoming_job!=job)state<=CONSUME;
     else if(incoming_sequence<expected_sequence)begin reply_kind<=2;duplicate_packets<=duplicate_packets+1'b1;state<=REPLY;end
     else if(incoming_sequence!=expected_sequence)state<=CONSUME;
     else begin
      saved_sequence<=incoming_sequence;beat<=0;lane<=0;
      address_q<=BASE+(bank?STRIDE:0)+(incoming_sequence<<9)+(incoming_sequence<<7);state<=BYTE_ADDRESS;
     end
    end
    BYTE_ADDRESS:state<=BYTE_CAPTURE;
    BYTE_CAPTURE:begin
     word_q[lane*8+:8]<=payload_data;
     if(lane==15)begin aw_done<=0;w_done<=0;state<=WRITE;end
     else begin lane<=lane+1'b1;state<=BYTE_ADDRESS;end
    end
    WRITE:begin
     if(awvalid&&awready)aw_done<=1;if(wvalid&&wready)w_done<=1;
     if((aw_done||(awvalid&&awready))&&(w_done||(wvalid&&wready)))begin
      if(beat==79)begin command_live<=1;state<=FENCE_AR;end
      else begin beat<=beat+1'b1;lane<=0;address_q<=address_q+8;state<=BYTE_ADDRESS;end
     end
    end
    FENCE_AR:begin end
    FENCE_R:if(rvalid&&rready)begin
     if(!rlast||rdata!==word_q)begin fault<=1;state<=FAILED;end
     else begin
      expected_sequence<=saved_sequence+1'b1;accepted_packets<=accepted_packets+1'b1;
      if(saved_sequence==2879)begin
       if(bank)bank1_job<=write_version;else bank0_job<=write_version;
       latest_job<=write_version;version_counter<=write_version;
       latest_capture_job<=job;latest_valid<=1;busy<=0;
      end
      reply_kind<=2;reply_job<=job;reply_sequence<=saved_sequence;state<=REPLY;
     end
    end
    REPLY:if(reply_ready)state<=CONSUME;
    CONSUME:state<=IDLE;
    FAILED:begin end
    default:begin fault<=1;state<=FAILED;end
   endcase
  end
 end
endmodule
