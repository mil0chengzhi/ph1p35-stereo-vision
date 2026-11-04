`timescale 1ns/1ps
// Isolated component DRAFT, not wired into A89. request_begin/reset_ack must
// come from the existing BOX owner. The owner must not reuse staged return
// data or capture next feedback before this event's phase completes.
module tracker_pixel_owner_replica_draft(
 input wire pixel_clk,service_clk,pixel_global_reset_n,service_global_reset_n,pixel_reset_n,
 input wire enable,source_valid,stale,changed,overflow,
 input wire request_begin,request_is_reset,reset_ack,
 output reg[7:0]request_meta,output wire phase_busy,
 input wire service_active,cpu_write,input wire[3:0]cpu_word,
 input wire[31:0]cpu_wdata,
 input wire detection_commit,detection_from_service,detection_red_valid,
 input wire[31:0]detection_frame,
 input wire[35:0]red_cells,
 output wire visible,output wire[15:0]track_id,center_x,center_y,
 output reg previous_accepted,previous_fresh,
 output wire[2:0]queued_empty,output wire[31:0]last_event_frame
);
 reg[40:0]staged_packet;
 always @(posedge service_clk or negedge service_global_reset_n)begin
  if(!service_global_reset_n)begin staged_packet<=0;end
  else begin
   if(service_active&&cpu_write)case(cpu_word)
    4:begin staged_packet[13:0]<=cpu_wdata[31:18];end
    5:begin staged_packet[27:14]<=cpu_wdata[31:18];end
    6:begin staged_packet[40:28]<=cpu_wdata[30:18];end
   endcase
  end
 end
 reg[39:0]published;
 reg[2:0]phase,empty_count;
 reg[31:0]frame_key;
 reg have_frame,duplicate_saved,from_service_saved,track_live,reset_pending;
 reg zero_pending;
 wire cancel=!enable||!source_valid||stale||changed||overflow;
 wire[10:0]raw_x=({1'b0,red_cells[8:0]}+{1'b0,red_cells[17:9]})*2+1;
 wire[9:0]raw_y=({1'b0,red_cells[26:18]}+{1'b0,red_cells[35:27]})*2+1;
 assign visible=published[37];assign track_id=published[36:21];
 assign center_x={5'd0,published[10:0]};assign center_y={6'd0,published[20:11]};
 assign phase_busy=phase!=0;assign queued_empty=empty_count;assign last_event_frame=frame_key;
 always @(posedge pixel_clk or negedge pixel_global_reset_n or negedge pixel_reset_n)begin
  if(!pixel_global_reset_n||!pixel_reset_n)zero_pending<=1;
  else if(reset_ack&&request_meta[3])zero_pending<=0;
 end
 always @(posedge pixel_clk or negedge pixel_global_reset_n)begin
  if(!pixel_global_reset_n)begin
   published<=0;phase<=0;frame_key<=0;have_frame<=0;duplicate_saved<=0;
   from_service_saved<=0;track_live<=0;reset_pending<=1;empty_count<=0;
   request_meta<=0;previous_accepted<=0;previous_fresh<=0;
  end else begin
   if(request_begin)begin
    request_meta<={empty_count,previous_fresh,zero_pending,reset_pending,previous_accepted,enable};
    previous_accepted<=0;previous_fresh<=0;track_live<=enable;
    if(request_is_reset)empty_count<=0;
   end
   if(reset_ack&&request_meta[2]&&track_live&&!cancel)reset_pending<=0;
   if(!pixel_reset_n||cancel)begin
    published<=0;phase<=0;have_frame<=0;track_live<=0;reset_pending<=1;empty_count<=0;
   end else if(detection_commit&&phase==0)begin
    phase<=1;duplicate_saved<=have_frame&&detection_frame==frame_key;
    frame_key<=detection_frame;from_service_saved<=detection_from_service;
   end else if(phase!=0)begin
    if(phase==7)begin
     phase<=0;have_frame<=1;
     if(from_service_saved)begin
      if(track_live)begin published<=staged_packet[39:0];previous_fresh<=0;end
      else begin
       published<={2'd0,1'b0,16'd0,(staged_packet[40]?raw_y:10'd0),(staged_packet[40]?raw_x:11'd0)};
       previous_fresh<=1;
      end
      previous_accepted<=1;reset_pending<=0;
     end else if(!duplicate_saved)begin
      if(published[36:21]!=0&&published[39:38]!=3)begin published[37]<=0;published[39:38]<=published[39:38]+1'b1;end
      else published<=0;
      if(empty_count<4)empty_count<=empty_count+1'b1;
     end
    end else phase<=phase+1'b1;
   end
  end
 end
endmodule
