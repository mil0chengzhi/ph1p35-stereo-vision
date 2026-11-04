`timescale 1ns/1ps
// One request owns one new depth job. Cancel drains an already-started job.
// Cancel carries a held request epoch so CDC ordering cannot revive a canceled request.
module single_capture_gate #(parameter REQUIRE_QUIESCENT=0)(
 input wire board_clk,memory_clk,rst_n,memory_rst_n,
 input wire request,cancel,pipeline_quiescent,input wire[31:0]jobs_started,jobs_completed,
 output wire ready,output wire run_budget,output wire busy,output wire canceled_started_job
);
 reg request_toggle,cancel_toggle,cancel_payload,cancel_was;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]request_sync,cancel_sync,request_ack_sync,cancel_ack_sync,busy_sync;
 reg request_seen,cancel_seen,budget,active,canceled_valid,canceled_epoch;
 reg[31:0]started_before;reg[1:0]cancel_settle;
 assign canceled_started_job=canceled_valid&&canceled_epoch==request_seen&&jobs_started!=started_before;
 assign ready=rst_n&&!cancel&&request_ack_sync[1]==request_toggle&&cancel_ack_sync[1]==cancel_toggle&&!busy_sync[1];
 assign busy=!ready;assign run_budget=budget;
 always @(posedge board_clk)begin
  if(!rst_n)begin request_toggle<=0;cancel_toggle<=0;cancel_payload<=0;cancel_was<=0;request_ack_sync<=0;cancel_ack_sync<=0;busy_sync<=0;end
  else begin
   request_ack_sync<={request_ack_sync[0],request_seen};cancel_ack_sync<={cancel_ack_sync[0],cancel_seen};busy_sync<={busy_sync[0],active};
   cancel_was<=cancel;
   if(cancel&&!cancel_was&&cancel_ack_sync[1]==cancel_toggle)begin cancel_payload<=request_toggle;cancel_toggle<=!cancel_toggle;end
   else if(request&&ready)request_toggle<=!request_toggle;
  end
 end
 wire new_request=request_sync[1]!=request_seen;
 wire new_cancel=cancel_sync[1]!=cancel_seen;
 wire canceled_request=(canceled_valid&&canceled_epoch==request_sync[1])||(new_cancel&&cancel_payload==request_sync[1]);
 always @(posedge memory_clk)begin
  if(!memory_rst_n)begin request_sync<=0;cancel_sync<=0;request_seen<=0;cancel_seen<=0;budget<=0;active<=0;started_before<=0;canceled_valid<=0;canceled_epoch<=0;cancel_settle<=0;end
  else begin
   request_sync<={request_sync[0],request_toggle};cancel_sync<={cancel_sync[0],cancel_toggle};
   if(budget&&jobs_started!=started_before)budget<=0;
   // Budget removal and a scheduler launch may meet on one clock edge.
   // Wait for that launch ledger and for pre-launch leases to drain.
   if(cancel_settle>1)cancel_settle<=cancel_settle-1'b1;
   else if(cancel_settle==1)begin
    if(jobs_started!=started_before)cancel_settle<=0;
    else if(!REQUIRE_QUIESCENT||pipeline_quiescent)begin cancel_settle<=0;active<=0;end
   end
   if(active&&!budget&&jobs_started!=started_before&&jobs_completed==jobs_started)active<=0;
   if(new_request)begin
    request_seen<=request_sync[1];started_before<=jobs_started;
    budget<=!canceled_request;active<=1;cancel_settle<=canceled_request?2:0;
    if(!new_cancel&&!canceled_request)canceled_valid<=0;
   end
   if(new_cancel)begin
    cancel_seen<=cancel_sync[1];canceled_epoch<=cancel_payload;canceled_valid<=1;
    if(cancel_payload==request_seen||(new_request&&cancel_payload==request_sync[1]))begin
     budget<=0;active<=1;cancel_settle<=2;
    end
   end
  end
 end
endmodule
