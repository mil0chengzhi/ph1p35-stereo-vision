`timescale 1ns/1ps
// Isolated candidate. Global reset MUST also reset the real MCU; pixel_reset_n
// may assert independently and while pixel_clk is stopped. Its reset_pending
// latch survives the pulse until an explicit CPU history-reset acknowledgement.
// The detector's source bbox buses are MMIO inputs directly, never copied to a
// 292bit FPGA input bank. Changes during a live request cancel that request;
// every cancelled result is drained and followed by a separate history reset.
// CPU words0..15 are to be decoded at shared MMIO words16..31 by the parent.
module box_tracker_mcu_owner_draft(
 input wire pixel_clk,service_clk,global_reset_n,pixel_reset_n,worker_ready,
 input wire source_valid,stale,overflow,result_valid,input wire[31:0]result_frame_id,
 input wire[3:0]valid,input wire[63:0]bbox_x0,bbox_x1,bbox_y0,bbox_y1,
 output wire busy,output wire[3:0]filtered_valid,
 output reg filtered_commit,output reg[31:0]filtered_frame_id,
 output wire[63:0]filtered_x0,filtered_x1,filtered_y0,filtered_y1,
 input wire[3:0]cpu_read_word,cpu_write_word,input wire cpu_write,input wire[31:0]cpu_wdata,
 input wire track_enable,track_source_valid,track_stale,track_changed,track_overflow,
 input wire detection_commit,detection_from_service,detection_red_valid,
 input wire[31:0]detection_frame,input wire[35:0]red_cells,
 output wire track_visible,output wire[15:0]track_id,track_center_x,track_center_y,
 output reg filtered_from_service,
 output reg[31:0]cpu_rdata
);
 // Global reset defines the MCU/ticket epoch. Local pixel reset does NOT
 // reset this epoch. Release is synchronized separately in both clock domains.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]pixel_global_reset,service_global_reset;
 wire pixel_global_reset_n=pixel_global_reset[1];
 wire service_global_reset_n=service_global_reset[1];
 always @(posedge pixel_clk or negedge global_reset_n)begin
  if(!global_reset_n)pixel_global_reset<=0;else pixel_global_reset<={pixel_global_reset[0],1'b1};
 end
 always @(posedge service_clk or negedge global_reset_n)begin
  if(!global_reset_n)service_global_reset<=0;else service_global_reset<={service_global_reset[0],1'b1};
 end
 reg request_toggle,pending,request_is_reset,request_live;
 // Reservation keeps the original pending/token/raw-busy boundary. Only
 // dispatch into the existing request synchronizer may wait for the prior
 // tracker event; no second mailbox/epoch/input bbox snapshot is introduced.
 reg dispatch_waiting;
 reg[3:0]publication_guard;
 wire[7:0]track_request_meta;
 wire track_phase_busy;
 wire[2:0]track_empty_count;
 wire[31:0]track_last_event_frame;
 reg[31:0]request_token,request_frame;
 reg[3:0]request_valid,published_valid;
 reg[143:0]published_cells;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]ack_sync,ready_sync;
 reg ack_seen,cancel_latched;
 reg reset_pending;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]request_sync,cancel_sync;
 reg request_seen,service_active,ack_toggle,ack_reset_done;
 reg[143:0]staged_cells;
 reg[7:0]staged_mask;
 reg[3:0]staged_valid;
 reg staged_publish,staged_meta;
 wire source_cancel=!pixel_reset_n||!source_valid||stale||overflow||!ready_sync[1];
 wire ack_arrived=pending&&ack_sync[1]!=ack_seen;
 wire busy_violation=result_valid&&pending;
 wire reset_ack=ack_arrived&&request_is_reset&&ack_reset_done&&!source_cancel&&!busy_violation;
 wire dispatch_blocked=publication_guard!=0||filtered_commit||detection_commit||track_phase_busy;
 wire reserve_request=!pending&&!source_cancel&&(reset_pending||result_valid);
 wire dispatch_request=(dispatch_waiting||reserve_request)&&!dispatch_blocked;
 wire dispatch_is_reset=dispatch_waiting?request_is_reset:reset_pending;
 wire tracker_reset_ack=ack_arrived&&request_is_reset&&ack_reset_done;
 assign busy=pending||reset_pending||!pixel_reset_n||!ready_sync[1];
 assign filtered_valid=source_cancel||reset_pending?4'd0:published_valid;
 genvar g;
 generate for(g=0;g<4;g=g+1)begin:out_slots
  assign filtered_x0[g*16+:16]={5'd0,published_cells[g*36+:9],2'b00};
  assign filtered_x1[g*16+:16]={5'd0,published_cells[g*36+9+:9],2'b11};
  assign filtered_y0[g*16+:16]={5'd0,published_cells[g*36+18+:9],2'b00};
  assign filtered_y1[g*16+:16]={5'd0,published_cells[g*36+27+:9],2'b11};
 end endgenerate
 // This latch is deliberately separate from request ownership and token.
 // A local reset cannot reset/reuse a ticket belonging to an active CPU job.
 always @(posedge pixel_clk or negedge pixel_global_reset_n or negedge pixel_reset_n)begin
  if(!pixel_global_reset_n||!pixel_reset_n)reset_pending<=1;
  else if(source_cancel||busy_violation)reset_pending<=1;
  else if(reset_ack)reset_pending<=0;
 end
 always @(posedge pixel_clk or negedge pixel_global_reset_n)begin
  if(!pixel_global_reset_n)begin
   request_toggle<=0;pending<=0;request_is_reset<=0;request_live<=0;
   request_token<=0;request_frame<=0;request_valid<=0;published_valid<=0;
   published_cells<=0;filtered_commit<=0;filtered_frame_id<=0;
   ack_sync<=0;ack_seen<=0;ready_sync<=0;cancel_latched<=0;
   dispatch_waiting<=0;publication_guard<=0;filtered_from_service<=0;
  end else begin
   ack_sync<={ack_sync[0],ack_toggle};ready_sync<={ready_sync[0],worker_ready};
   cancel_latched<=source_cancel||reset_pending;
   filtered_commit<=0;filtered_from_service<=0;
   if(filtered_commit)publication_guard<=12;
   else if(publication_guard!=0)publication_guard<=publication_guard-1'b1;
   if(dispatch_request)begin
    request_toggle<=!request_toggle;dispatch_waiting<=0;
    // Empty events were already visible in pixel time. Carry their real
    // final frame with the existing reset request, after the phase finishes.
    if(dispatch_is_reset&&track_empty_count!=0)request_frame<=track_last_event_frame;
   end
   if(ack_arrived)begin
    pending<=0;ack_seen<=ack_sync[1];
    if(!request_is_reset&&request_live&&!reset_pending&&!source_cancel&&!busy_violation&&!ack_reset_done)begin
     published_valid<=staged_valid&request_valid;
     if(staged_publish)begin
      published_cells<=staged_cells;filtered_frame_id<=request_frame;filtered_commit<=1;filtered_from_service<=1;
     end
    end
    request_live<=0;
   end
   if(source_cancel||reset_pending)begin
    request_live<=0;published_valid<=0;
    if(!cancel_latched||published_valid!=0)begin filtered_commit<=1;filtered_frame_id<=result_frame_id;end
   end else if(busy_violation)begin
    request_live<=0;published_valid<=0;filtered_commit<=1;filtered_frame_id<=result_frame_id;
   end
   if(!pending&&!source_cancel)begin
    if(reset_pending)begin
     request_token<=request_token+1'b1;request_frame<=result_frame_id;request_valid<=0;
     request_is_reset<=1;request_live<=0;dispatch_waiting<=!dispatch_request;pending<=1;
    end else if(result_valid)begin
     request_token<=request_token+1'b1;request_frame<=result_frame_id;request_valid<=valid;
     request_is_reset<=0;request_live<=1;dispatch_waiting<=!dispatch_request;pending<=1;
     published_valid<=published_valid&valid;
    end
   end
  end
 end
 always @(posedge service_clk or negedge service_global_reset_n)begin
  if(!service_global_reset_n)begin
   request_sync<=0;cancel_sync<=0;request_seen<=0;service_active<=0;
   ack_toggle<=0;ack_reset_done<=0;staged_cells<=0;staged_mask<=0;
   staged_valid<=0;staged_publish<=0;staged_meta<=0;
  end else begin
   request_sync<={request_sync[0],request_toggle};
   cancel_sync<={cancel_sync[0],reset_pending||!request_live};
   if(worker_ready&&!service_active&&request_sync[1]!=request_seen)begin
    request_seen<=request_sync[1];service_active<=1;staged_mask<=0;staged_meta<=0;
   end
   if(cpu_write&&worker_ready&&service_active)begin
    if(cpu_write_word>=4&&cpu_write_word<=11&&!request_is_reset)begin
     case(cpu_write_word)
      4'd4:begin staged_cells[0+:18]<=cpu_wdata[17:0];staged_mask[0]<=1;end
      4'd5:begin staged_cells[18+:18]<=cpu_wdata[17:0];staged_mask[1]<=1;end
      4'd6:begin staged_cells[36+:18]<=cpu_wdata[17:0];staged_mask[2]<=1;end
      4'd7:begin staged_cells[54+:18]<=cpu_wdata[17:0];staged_mask[3]<=1;end
      4'd8:begin staged_cells[72+:18]<=cpu_wdata[17:0];staged_mask[4]<=1;end
      4'd9:begin staged_cells[90+:18]<=cpu_wdata[17:0];staged_mask[5]<=1;end
      4'd10:begin staged_cells[108+:18]<=cpu_wdata[17:0];staged_mask[6]<=1;end
      4'd11:begin staged_cells[126+:18]<=cpu_wdata[17:0];staged_mask[7]<=1;end
     endcase
    end
    if(cpu_write_word==12&&!request_is_reset)begin
     staged_valid<=cpu_wdata[3:0];staged_publish<=cpu_wdata[8];staged_meta<=1;
    end
    if(cpu_write_word==13&&!request_is_reset&&staged_mask==8'hff&&staged_meta&&cpu_wdata==request_token)begin
     service_active<=0;ack_reset_done<=0;ack_toggle<=!ack_toggle;
    end
    if(cpu_write_word==14&&cpu_wdata==request_token)begin
     service_active<=0;ack_reset_done<=1;ack_toggle<=!ack_toggle;
    end
   end
  end
 end
 always @*begin
  cpu_rdata=0;
  case(cpu_read_word)
   0:cpu_rdata={service_active&&worker_ready,request_is_reset,cancel_sync[1],staged_meta,12'd0,track_request_meta,request_valid,4'd0};
   1:cpu_rdata=request_token;
   2:cpu_rdata=request_frame;
   3:cpu_rdata={24'd0,staged_mask};
   4:cpu_rdata={bbox_x1[15:0],bbox_x0[15:0]};5:cpu_rdata={bbox_y1[15:0],bbox_y0[15:0]};
   6:cpu_rdata={bbox_x1[31:16],bbox_x0[31:16]};7:cpu_rdata={bbox_y1[31:16],bbox_y0[31:16]};
   8:cpu_rdata={bbox_x1[47:32],bbox_x0[47:32]};9:cpu_rdata={bbox_y1[47:32],bbox_y0[47:32]};
   10:cpu_rdata={bbox_x1[63:48],bbox_x0[63:48]};11:cpu_rdata={bbox_y1[63:48],bbox_y0[63:48]};
  endcase
 end
 tracker_pixel_owner_replica_draft tracker_publication(
  .pixel_clk(pixel_clk),.service_clk(service_clk),
  .pixel_global_reset_n(pixel_global_reset_n),.service_global_reset_n(service_global_reset_n),.pixel_reset_n(pixel_reset_n),
  .enable(track_enable),.source_valid(track_source_valid),.stale(track_stale),.changed(track_changed),.overflow(track_overflow),
  .request_begin(dispatch_request),.request_is_reset(dispatch_is_reset),.reset_ack(tracker_reset_ack),
  .request_meta(track_request_meta),.phase_busy(track_phase_busy),.queued_empty(track_empty_count),.last_event_frame(track_last_event_frame),
  .service_active(service_active&&worker_ready&&!request_is_reset),.cpu_write(cpu_write),.cpu_word(cpu_write_word),.cpu_wdata(cpu_wdata),
  .detection_commit(detection_commit),.detection_from_service(detection_from_service),.detection_red_valid(detection_red_valid),.detection_frame(detection_frame),.red_cells(red_cells),
  .visible(track_visible),.track_id(track_id),.center_x(track_center_x),.center_y(track_center_y),.previous_accepted(),.previous_fresh());
endmodule
