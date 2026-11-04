`timescale 1ns/1ps
// Single selected-color track. All ports are in the pixel clock domain.
// A batch starts before the component stream; commit is a new valid result_sequence/frame_id.
// Leave at least three full clocks after the final candidate before changing result_sequence.
// Search chooses the largest region. Acquisition/lock chooses nearest center within 160 pixels
// Manhattan distance and 1/4..4x last measured area. Equal distance favors larger area.
// Three consecutive measured frames confirm a track; three missing frames retain its identity.
// A missing measurement immediately hides the box; no predicted box is presented as a detection.
// Center EMA = round((3*old+measurement)/4). Four historical centers are visual annotations.
// Only a complete, fresh, non-overflow frame may commit candidates. Duplicate frames do not age.
// Source invalidity, stale data, a color change or leaving color mode cancels the track immediately.
// No frame RAM, multiplier or new clock domain. Counts are grid cells, not distance in meters.
module color_tracker #(
 parameter MAX_DISTANCE=160,MIN_COUNT=16,ACQUIRE_FRAMES=3,HOLD_FRAMES=3,
 parameter ROI_LEFT=112,ROI_RIGHT=911,ROI_TOP=0,ROI_BOTTOM=599,MAX_COUNT=7500
)(
 input wire clk,input wire rst_n,input wire enable,input wire [1:0] color,
 input wire source_valid,input wire result_valid,input wire stale,input wire overflow,
 input wire [31:0] result_frame_id,input wire [15:0] result_sequence,
 input wire batch_start,input wire candidate_valid,input wire [15:0] candidate_count,
 input wire [15:0] candidate_min_x,candidate_max_x,candidate_min_y,candidate_max_y,
  input wire [6:0] candidate_seed_x,candidate_seed_y,
 output wire [1:0] state,output reg visible,output reg [15:0] track_id,age,
 output reg [15:0] center_x,center_y,min_x,max_x,min_y,max_y,count,
 output reg [2:0] misses,output reg [1:0] hits,
 output reg [31:0] frame_id,output reg [15:0] processed_sequence,
  output reg [63:0] trail_x,trail_y,output reg [3:0] trail_valid,
  output reg [6:0] track_seed_x,track_seed_y,output reg track_seed_valid
);
 reg [1:0] saved_color;
 reg reference_valid,confirmed,have_frame;
 reg [15:0] identity_counter,last_sequence,reference_x,reference_y,reference_count;
 reg [31:0] last_frame;
 reg best_valid;
 reg [15:0] best_count,best_min_x,best_max_x,best_min_y,best_max_y,best_x,best_y;
 reg [16:0] best_distance;
 reg valid_p1,valid_p2;
 reg [15:0] count_p1,x0_p1,x1_p1,y0_p1,y1_p1,cx_p1,cy_p1;
 reg [15:0] count_p2,x0_p2,x1_p2,y0_p2,y1_p2,cx_p2,cy_p2;
 reg [16:0] distance_p2;
  reg [6:0] seed_x_p1,seed_y_p1,seed_x_p2,seed_y_p2,best_seed_x,best_seed_y;
 wire [16:0] sum_x={1'b0,candidate_min_x}+{1'b0,candidate_max_x};
 wire [16:0] sum_y={1'b0,candidate_min_y}+{1'b0,candidate_max_y};
 wire [15:0] cx=sum_x[16:1],cy=sum_y[16:1];
 wire [15:0] dx=cx_p1>=reference_x?cx_p1-reference_x:reference_x-cx_p1;
 wire [15:0] dy=cy_p1>=reference_y?cy_p1-reference_y:reference_y-cy_p1;
 wire [16:0] distance={1'b0,dx}+{1'b0,dy};
 wire geometry_ok=x0_p1>=ROI_LEFT&&x1_p1<=ROI_RIGHT&&x0_p1<=x1_p1&&y0_p1>=ROI_TOP&&y0_p1<=y1_p1&&y1_p1<=ROI_BOTTOM;
 wire area_ok=({2'b0,count_p1}<={reference_count,2'b0})&&({count_p1,2'b0}>={2'b0,reference_count});
 wire acceptable=geometry_ok&&(count_p1>=MIN_COUNT)&&(count_p1<=MAX_COUNT)&&
                 (!reference_valid||((distance<=MAX_DISTANCE)&&area_ok));
 wire better=!best_valid || (!reference_valid?(count_p2>best_count):
             ((distance_p2<best_distance)||((distance_p2==best_distance)&&(count_p2>best_count))));
 wire [17:0] smooth_x={2'b0,center_x}+{1'b0,center_x,1'b0}+{2'b0,best_x}+18'd2;
 wire [17:0] smooth_y={2'b0,center_y}+{1'b0,center_y,1'b0}+{2'b0,best_y}+18'd2;
 wire [15:0] next_x=reference_valid?smooth_x[17:2]:best_x;
 wire [15:0] next_y=reference_valid?smooth_y[17:2]:best_y;
 wire [15:0] next_identity=(identity_counter==16'hffff)?16'd1:identity_counter+16'd1;
 assign state=!reference_valid?2'd0:(!confirmed?2'd1:(visible?2'd2:2'd3));
 always @(posedge clk) begin
  if(!rst_n) begin
   saved_color<=0;reference_valid<=0;confirmed<=0;have_frame<=0;identity_counter<=0;
   last_sequence<=0;last_frame<=0;reference_x<=0;reference_y<=0;reference_count<=0;
   best_valid<=0;best_count<=0;best_min_x<=0;best_max_x<=0;best_min_y<=0;best_max_y<=0;
   best_x<=0;best_y<=0;best_distance<=0;
   valid_p1<=0;valid_p2<=0;count_p1<=0;x0_p1<=0;x1_p1<=0;y0_p1<=0;y1_p1<=0;cx_p1<=0;cy_p1<=0;
   count_p2<=0;x0_p2<=0;x1_p2<=0;y0_p2<=0;y1_p2<=0;cx_p2<=0;cy_p2<=0;distance_p2<=0;
   visible<=0;track_id<=0;age<=0;center_x<=0;center_y<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;count<=0;
   misses<=0;hits<=0;frame_id<=0;processed_sequence<=0;trail_x<=0;trail_y<=0;trail_valid<=0;
   track_seed_x<=0;track_seed_y<=0;track_seed_valid<=0;
   seed_x_p1<=0;seed_y_p1<=0;seed_x_p2<=0;seed_y_p2<=0;best_seed_x<=0;best_seed_y<=0;
  end else begin
   saved_color<=color;
   if(!enable||saved_color!=color||!source_valid||stale||overflow) begin
    reference_valid<=0;confirmed<=0;have_frame<=0;best_valid<=0;
    valid_p1<=0;valid_p2<=0;
    last_sequence<=result_sequence;visible<=0;track_id<=0;age<=0;hits<=0;misses<=0;
    center_x<=0;center_y<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;count<=0;
    trail_x<=0;trail_y<=0;trail_valid<=0;frame_id<=0;processed_sequence<=result_sequence;
     track_seed_valid<=0;track_seed_x<=0;track_seed_y<=0;
   end else begin
    valid_p1<=candidate_valid;valid_p2<=valid_p1&&acceptable;
    if(candidate_valid) begin
     count_p1<=candidate_count;x0_p1<=candidate_min_x;x1_p1<=candidate_max_x;y0_p1<=candidate_min_y;y1_p1<=candidate_max_y;
     cx_p1<=cx;cy_p1<=cy;
      seed_x_p1<=candidate_seed_x;seed_y_p1<=candidate_seed_y;
    end
    if(valid_p1) begin
     count_p2<=count_p1;x0_p2<=x0_p1;x1_p2<=x1_p1;y0_p2<=y0_p1;y1_p2<=y1_p1;cx_p2<=cx_p1;cy_p2<=cy_p1;distance_p2<=distance;
      seed_x_p2<=seed_x_p1;seed_y_p2<=seed_y_p1;
    end
    if(batch_start)begin best_valid<=0;valid_p1<=0;valid_p2<=0;end
    else if(valid_p2&&better) begin
     best_valid<=1;best_count<=count_p2;best_distance<=distance_p2;
     best_min_x<=x0_p2;best_max_x<=x1_p2;
     best_min_y<=y0_p2;best_max_y<=y1_p2;best_x<=cx_p2;best_y<=cy_p2;
      best_seed_x<=seed_x_p2;best_seed_y<=seed_y_p2;
    end
    if(!result_valid) begin
     // Candidates can be collected while the first complete result is still pending.
     reference_valid<=0;confirmed<=0;have_frame<=0;last_sequence<=result_sequence;
     visible<=0;track_id<=0;age<=0;hits<=0;misses<=0;trail_valid<=0;
     center_x<=0;center_y<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;count<=0;
      track_seed_valid<=0;track_seed_x<=0;track_seed_y<=0;
    end else if(result_sequence!=last_sequence) begin
     last_sequence<=result_sequence;best_valid<=0;
     if(!have_frame||result_frame_id!=last_frame) begin
      have_frame<=1;last_frame<=result_frame_id;frame_id<=result_frame_id;processed_sequence<=result_sequence;
      if(best_valid) begin
       reference_valid<=1;reference_x<=best_x;reference_y<=best_y;reference_count<=best_count;
       center_x<=next_x;center_y<=next_y;
       min_x<=best_min_x;max_x<=best_max_x;min_y<=best_min_y;max_y<=best_max_y;count<=best_count;
        track_seed_x<=best_seed_x;track_seed_y<=best_seed_y;
       misses<=0;if(hits<ACQUIRE_FRAMES)hits<=hits+1'b1;
       if(confirmed) begin
        visible<=1;if(age!=16'hffff)age<=age+1'b1;
         track_seed_valid<=1;
        trail_x<={trail_x[47:0],next_x};trail_y<={trail_y[47:0],next_y};trail_valid<={trail_valid[2:0],1'b1};
       end else if(hits>=ACQUIRE_FRAMES-1) begin
        confirmed<=1;visible<=1;age<=1;identity_counter<=next_identity;track_id<=next_identity;
         track_seed_valid<=1;
        trail_x<={48'd0,next_x};trail_y<={48'd0,next_y};trail_valid<=1;
        end else begin visible<=0;age<=0;trail_valid<=0;track_seed_valid<=0;end
      end else begin
       visible<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;count<=0;
        track_seed_valid<=0;
       if(confirmed&&misses<HOLD_FRAMES) misses<=misses+1'b1;
       else begin
        reference_valid<=0;confirmed<=0;track_id<=0;age<=0;hits<=0;misses<=0;
        center_x<=0;center_y<=0;trail_x<=0;trail_y<=0;trail_valid<=0;
       end
      end
     end
    end
   end
  end
 end
endmodule
