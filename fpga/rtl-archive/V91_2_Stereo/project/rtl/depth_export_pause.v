`timescale 1ns/1ps
// B26 source candidate, pending board qualification. memory_block_start gates ONLY new job admission.
// Existing work must finish naturally. memory_quiescent must prove WAITING,
// engine idle, publication fence complete and both camera leases released.
// A generation handshake acknowledges release before another HOLD can start;
// a stale ACK from a cancelled request cannot satisfy a subsequent request.
module depth_export_pause #(parameter CLOCK_HZ=50000000,TIMEOUT_MS=120000)(
 input wire host_clk,memory_clk,rst_n,
 input wire host_write,host_value,
 output reg host_requested,output wire host_held,host_rearm_ready,
 output reg host_timed_out,host_fault,host_rearm_rejected,
 output reg[31:0]host_job,output reg[127:0]host_source_pair,
 input wire memory_quiescent,input wire[31:0]last_completed_job,input wire[127:0]last_source_pair,
 output wire memory_block_start,output reg memory_hold_confirmed
);
 // Both domains share asynchronous assertion and release locally after two clocks.
 // No independent domain reset; RAM contents are exposed only after a new ACK.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]host_reset,memory_reset;
 wire host_rst_n=host_reset[1],memory_rst_n=memory_reset[1];
 always @(posedge host_clk or negedge rst_n)
  if(!rst_n)host_reset<=0;else host_reset<={host_reset[0],1'b1};
 always @(posedge memory_clk or negedge rst_n)
  if(!rst_n)memory_reset<=0;else memory_reset<={memory_reset[0],1'b1};

 localparam[63:0]CYCLES=CLOCK_HZ*64'd1*TIMEOUT_MS/1000;
 localparam BITS=CYCLES<4?2:$clog2(CYCLES);
 reg[BITS-1:0]age;
 reg release_toggle,release_ack;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]request_sync,release_sync,held_sync,release_ack_sync;
 reg[2:0]host_init;
 reg descriptor_ready,memory_fault;
 reg[31:0]held_job;reg[127:0]held_sources;
 assign memory_block_start=request_sync[1];
 assign host_rearm_ready=host_init==7&&!host_requested&&!held_sync[1]&&release_ack_sync[1]==release_toggle;
 assign host_held=host_requested&&held_sync[1]&&descriptor_ready&&!host_fault&&!host_timed_out;
 always @(posedge memory_clk or negedge memory_rst_n)begin
  if(!memory_rst_n)begin request_sync<=0;release_sync<=0;release_ack<=0;memory_hold_confirmed<=0;memory_fault<=0;held_job<=0;held_sources<=0;end
  else begin
   request_sync<={request_sync[0],host_requested};release_sync<={release_sync[0],release_toggle};
   if(!request_sync[1])begin
    memory_hold_confirmed<=0;memory_fault<=0;
    if(release_sync[1]!=release_ack)release_ack<=release_sync[1];
   end else if(memory_hold_confirmed)begin
    if(!memory_quiescent||last_completed_job!=held_job||last_source_pair!=held_sources)begin memory_fault<=1;memory_hold_confirmed<=0;end
   end else if(!memory_fault&&memory_quiescent&&last_completed_job!=0)begin
    held_job<=last_completed_job;held_sources<=last_source_pair;memory_hold_confirmed<=1;
   end
  end
 end
 always @(posedge host_clk or negedge host_rst_n)begin
  if(!host_rst_n)begin host_requested<=0;release_toggle<=0;held_sync<=0;release_ack_sync<=0;
   host_init<=0;age<=0;descriptor_ready<=0;host_timed_out<=0;host_fault<=0;host_rearm_rejected<=0;host_job<=0;host_source_pair<=0;end
  else begin
   held_sync<={held_sync[0],memory_hold_confirmed};release_ack_sync<={release_ack_sync[0],release_ack};
   if(host_init!=7)host_init<=host_init+1'b1;
   if(host_write&&!host_value)begin
    if(host_requested)begin host_requested<=0;release_toggle<=!release_toggle;end
   end else if(host_write&&host_value&&!host_requested)begin
    if(host_rearm_ready)begin host_requested<=1;age<=0;descriptor_ready<=0;host_timed_out<=0;host_fault<=0;host_rearm_rejected<=0;end
    else host_rearm_rejected<=1;
   end else if(host_requested)begin
    if(age==CYCLES-1)begin host_requested<=0;release_toggle<=!release_toggle;host_timed_out<=1;end
    else age<=age+1'b1;
   end
   if(!host_requested)descriptor_ready<=0;
   else if(held_sync[1]&&!descriptor_ready)begin host_job<=held_job;host_source_pair<=held_sources;descriptor_ready<=1;end
   else if(descriptor_ready&&!held_sync[1])host_fault<=1;
  end
 end
 // synthesis translate_off
 initial if(CLOCK_HZ<1000||TIMEOUT_MS<1||CYCLES<4)$fatal(1,"Invalid metric pause watchdog");
 // synthesis translate_on
endmodule
