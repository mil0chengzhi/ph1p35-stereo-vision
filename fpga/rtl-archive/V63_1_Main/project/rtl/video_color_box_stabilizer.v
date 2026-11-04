`timescale 1ns/1ps
// Four colour slots only: red, green, blue, yellow. All ports use clk domain.
// Input boxes are closed HDMI coordinates on the detector's four-pixel grid.
// result_valid is a commit pulse; its buses MUST remain stable while busy.
// A repeated nonempty source ID never advances confirmation or filter history.
// Empty/withdrawal commits, invalid source, stale and overflow always withdraw.
// No predicted/missing box is displayed. Two distinct nearby source results
// acquire a colour. Three measured boxes supply a median with a 4px deadband.
// Coherent translation bypasses the median on that axis; a one-sided boundary
// excursion does not qualify as translation. No pixel-stream latency is added.
// New commits finish in <=42 clocks. Published buses change atomically except
// validity withdrawal, which has priority and never waits for the work loop.
// Existing OSD source-age checks remain required; this module adds no timer.
module video_color_box_stabilizer #(
 parameter ROI_LEFT=160,ROI_RIGHT=1119,SCREEN_HEIGHT=720,
 parameter MATCH_CELLS=16,DEADBAND_CELLS=1,MAX_SOURCE_GAP=2
)(
 input wire clk,rst_n,source_valid,stale,overflow,
 input wire result_valid,input wire [31:0] result_frame_id,
 input wire [3:0] valid,
 input wire [63:0] bbox_x0,bbox_x1,bbox_y0,bbox_y1,
 output reg filtered_commit,output reg [31:0] filtered_frame_id,
 output wire [3:0] filtered_valid,
 output wire [63:0] filtered_x0,filtered_x1,filtered_y0,filtered_y1,
 output wire busy
);
 localparam IDLE=0,LOAD=1,CLASSIFY=2,EDGE=3,SAVE=4,PUBLISH=5;
 reg [2:0] state;
 reg [1:0] slot,edge_index;
 wire [3:0] coordinate={slot,edge_index};
 reg [3:0] published_valid,work_valid,transaction_valid;
 reg [31:0] transaction_frame,last_input_frame;
 reg seen_input,gap_good,aborted;
 reg [1:0] history_count[0:3];
 // History has one read/write coordinate per cycle and no RAM reset. Its
 // validity is carried separately; old RAM contents can never confirm a box.
 reg [8:0] history1[0:15],history2[0:15];
 reg [8:0] published[0:15],work_box[0:15];
 reg [8:0] current[0:3];
 reg accepted,three_samples,translate_x,translate_y;
 reg match_all;
 reg signed [9:0] first_edge_delta;
 reg [1:0] next_count;
 assign busy=state!=IDLE;
 wire cancel=!source_valid||stale||overflow;
 assign filtered_valid=cancel?4'd0:published_valid;
 wire [31:0] source_gap=result_frame_id-last_input_frame;
 wire duplicate=seen_input&&result_frame_id==last_input_frame;
 wire [15:0] in_x0=bbox_x0[slot*16+:16],in_x1=bbox_x1[slot*16+:16];
 wire [15:0] in_y0=bbox_y0[slot*16+:16],in_y1=bbox_y1[slot*16+:16];
 wire geometry_ok=in_x0>=ROI_LEFT&&in_x1<=ROI_RIGHT&&in_x0<=in_x1&&
  in_y0<=in_y1&&in_y1<SCREEN_HEIGHT&&
  in_x0[1:0]==0&&in_y0[1:0]==0&&in_x1[1:0]==3&&in_y1[1:0]==3;
 genvar g;
 generate for(g=0;g<4;g=g+1)begin:outputs
  assign filtered_x0[g*16+:16]={5'd0,published[g*4],2'b00};
  assign filtered_x1[g*16+:16]={5'd0,published[g*4+1],2'b11};
  assign filtered_y0[g*16+:16]={5'd0,published[g*4+2],2'b00};
  assign filtered_y1[g*16+:16]={5'd0,published[g*4+3],2'b11};
 end endgenerate
 function [9:0] magnitude;input signed [9:0] value;
  begin magnitude=value[9]?-value:value;end
 endfunction
 function [10:0] wide_magnitude;input signed [10:0] value;
  begin wide_magnitude=value[10]?-value:value;end
 endfunction
 function [8:0] median3;input [8:0] a,b,c;
  begin
   if(a<=b)begin if(b<=c)median3=b;else if(a<=c)median3=c;else median3=a;end
   else begin if(a<=c)median3=a;else if(b<=c)median3=c;else median3=b;end
  end
 endfunction
 wire signed [9:0] current_delta=$signed({1'b0,current[edge_index]})-$signed({1'b0,history1[coordinate]});
 wire near_edge=magnitude(current_delta)<=MATCH_CELLS;
 wire signed [10:0] extent_change=$signed({first_edge_delta[9],first_edge_delta})-$signed({current_delta[9],current_delta});
 wire moving_axis=first_edge_delta!=0&&current_delta!=0&&first_edge_delta[9]==current_delta[9]&&
  wide_magnitude(extent_change)<=1;
 wire [8:0] median_value=median3(current[edge_index],history1[coordinate],history2[coordinate]);
 wire signed [9:0] median_delta=$signed({1'b0,median_value})-$signed({1'b0,published[coordinate]});
 wire follow_motion=edge_index[1]?translate_y:translate_x;
 wire [8:0] filtered_coordinate=(!three_samples||follow_motion)?current[edge_index]:
  magnitude(median_delta)<=DEADBAND_CELLS?published[coordinate]:median_value;
 integer i;
 task clear_state;
  begin
   state<=IDLE;published_valid<=0;work_valid<=0;transaction_valid<=0;
   seen_input<=0;accepted<=0;three_samples<=0;translate_x<=0;translate_y<=0;
   for(i=0;i<4;i=i+1)history_count[i]<=0;
  end
 endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   clear_state;filtered_commit<=0;filtered_frame_id<=0;transaction_frame<=0;
   last_input_frame<=0;gap_good<=0;aborted<=0;slot<=0;edge_index<=0;next_count<=0;
   match_all<=0;first_edge_delta<=0;
   for(i=0;i<16;i=i+1)published[i]<=0;
  end else begin
   filtered_commit<=0;aborted<=cancel;
   if(cancel)begin
    clear_state;
    if(!aborted||busy||published_valid!=0)begin filtered_commit<=1;filtered_frame_id<=result_frame_id;end
   end else if(result_valid&&busy)begin
    // A producer contract violation must never publish a mixture of frames.
    clear_state;filtered_commit<=1;filtered_frame_id<=result_frame_id;
   end else if(result_valid&&valid==0)begin
    clear_state;seen_input<=1;last_input_frame<=result_frame_id;
    filtered_commit<=1;filtered_frame_id<=result_frame_id;
   end else case(state)
    IDLE:if(result_valid)begin
     // Even a duplicate ID may explicitly withdraw a formerly valid slot.
     published_valid<=published_valid&valid;
     for(i=0;i<4;i=i+1)if(!valid[i])history_count[i]<=0;
     if(!duplicate)begin
      gap_good<=seen_input&&source_gap!=0&&source_gap<=MAX_SOURCE_GAP;
      transaction_frame<=result_frame_id;transaction_valid<=valid;work_valid<=0;
      seen_input<=1;last_input_frame<=result_frame_id;slot<=0;state<=LOAD;
     end else if((published_valid&~valid)!=0)begin
      filtered_commit<=1;filtered_frame_id<=result_frame_id;
     end
    end
    LOAD:begin
     if(!transaction_valid[slot])begin
      history_count[slot]<=0;work_valid[slot]<=0;
      if(slot==3)state<=PUBLISH;else slot<=slot+1'b1;
     end else if(!geometry_ok)begin
      clear_state;filtered_commit<=1;filtered_frame_id<=transaction_frame;
     end else begin
      current[0]<=in_x0[10:2];current[1]<=in_x1[10:2];
      current[2]<={1'b0,in_y0[9:2]};current[3]<={1'b0,in_y1[9:2]};
      match_all<=history_count[slot]!=0&&gap_good;edge_index<=0;state<=CLASSIFY;
     end
    end
    CLASSIFY:begin
     match_all<=match_all&&near_edge;
     if(!edge_index[0])first_edge_delta<=current_delta;
     else if(!edge_index[1])translate_x<=moving_axis;
     else translate_y<=moving_axis;
     if(edge_index==3)begin
      accepted<=match_all&&near_edge;
      three_samples<=history_count[slot]==2&&match_all&&near_edge;
      next_count<=match_all&&near_edge?2'd2:2'd1;
      edge_index<=0;state<=EDGE;
     end else edge_index<=edge_index+1'b1;
    end
    EDGE:begin
     work_box[coordinate]<=filtered_coordinate;
     history2[coordinate]<=history1[coordinate];history1[coordinate]<=current[edge_index];
     if(edge_index==3)state<=SAVE;else edge_index<=edge_index+1'b1;
    end
    SAVE:begin
     history_count[slot]<=next_count;work_valid[slot]<=accepted;
     // Independent deadbands must not invert an exceptionally thin box.
     if(work_box[{slot,2'd0}]>work_box[{slot,2'd1}]||work_box[{slot,2'd2}]>work_box[{slot,2'd3}])
      for(i=0;i<4;i=i+1)work_box[{slot,2'd0}+i]<=current[i];
     if(slot==3)state<=PUBLISH;else begin slot<=slot+1'b1;state<=LOAD;end
    end
    PUBLISH:begin
     for(i=0;i<16;i=i+1)published[i]<=work_box[i];
     published_valid<=work_valid;filtered_frame_id<=transaction_frame;filtered_commit<=1;state<=IDLE;
    end
    default:clear_state;
   endcase
  end
 end
endmodule
