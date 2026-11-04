`timescale 1ns/1ps
// One selected red component, reusing the tested identity/EMA tracker. Area is
// an explicit BBOX grid-area proxy, not a new segmentation count or depth.
// Candidate pipelines finish before a persistent result_sequence is committed.
module contest_track_adapter(
 input wire clk,rst_n,enable,source_valid,source_stale,source_changed,overflow,
 input wire detection_commit,input wire [31:0] detection_frame,
 input wire [7:0] detection_valid,input wire [127:0] detection_x0,detection_x1,detection_y0,detection_y1,
 output wire visible,output wire [15:0] track_id,center_x,center_y,min_x,max_x,min_y,max_y,
 output wire [1:0] track_state
);
 reg [3:0] phase;reg [15:0] result_sequence_reg;reg good;
 reg [31:0] saved_frame;reg have_red;
 reg [15:0] x0,x1,y0,y1;
 wire [16:0] bw={1'b0,x1}-{1'b0,x0}+1'b1,bh={1'b0,y1}-{1'b0,y0}+1'b1;
 wire [8:0] grid_width=(bw+3)>>2;wire [7:0] grid_height=(bh+3)>>2;
 wire [16:0] bbox_grid_area=grid_width*grid_height;
 wire cancel=!enable||!source_valid||source_stale||source_changed||overflow;
 always @(posedge clk)begin
  if(!rst_n)begin phase<=0;result_sequence_reg<=0;good<=0;saved_frame<=0;have_red<=0;x0<=0;x1<=0;y0<=0;y1<=0;end
  else if(cancel)begin phase<=0;good<=0;end
  else begin
   if(detection_commit&&phase==0)begin
    saved_frame<=detection_frame;have_red<=detection_valid[0];
    x0<=detection_x0[15:0];x1<=detection_x1[15:0];y0<=detection_y0[15:0];y1<=detection_y1[15:0];phase<=1;
   end else if(phase!=0)begin
    if(phase==6)begin result_sequence_reg<=result_sequence_reg+1'b1;good<=1;phase<=0;end
    else phase<=phase+1'b1;
   end
  end
 end
 color_tracker #(.ROI_LEFT(0),.ROI_RIGHT(1279),.ROI_TOP(0),.ROI_BOTTOM(719),.MAX_COUNT(57600)) tracker(
  .clk(clk),.rst_n(rst_n),.enable(enable),.color(2'd0),.source_valid(source_valid&&!source_changed),
  .result_valid(good),.stale(source_stale),.overflow(overflow),.result_frame_id(saved_frame),.result_sequence(result_sequence_reg),
  .batch_start(phase==1),.candidate_valid(phase==2&&have_red),.candidate_count(bbox_grid_area[15:0]),
  .candidate_min_x(x0),.candidate_max_x(x1),.candidate_min_y(y0),.candidate_max_y(y1),
  .candidate_seed_x(7'd0),.candidate_seed_y(7'd0),.state(track_state),.visible(visible),.track_id(track_id),
  .age(),.center_x(center_x),.center_y(center_y),.min_x(min_x),.max_x(max_x),.min_y(min_y),.max_y(max_y),.count(),
  .misses(),.hits(),.frame_id(),.processed_sequence(),.trail_x(),.trail_y(),.trail_valid(),
  .track_seed_x(),.track_seed_y(),.track_seed_valid());
endmodule
