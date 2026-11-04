`timescale 1ns/1ps
// The only selected-view processing pipeline. Detector/motion always consume
// pre-graphics RGB; all displayed modes retain native1280x720 pixel positions.
module contest_video_pipeline #(parameter USE_ERAM=1,CLK_HZ=74000000)(
 input wire box_service_clk,box_global_rst_n,box_worker_ready,
 input wire[3:0]box_read_word,box_write_word,input wire box_write,input wire[31:0]box_wdata,
 output wire[31:0]box_rdata,
 input wire clk,rst_n,input wire [2:0] mode,input wire [1:0] adjustment,
 input wire [15:0] x,y,input wire [23:0] rgb,input wire de,valid,hs,vs,
 input wire camera_remote,source_valid,source_stale,input wire [32:0] source_epoch,
 input wire [31:0] source_frame_id,source_start_ms,
 output wire [15:0] out_x,out_y,output wire [23:0] out_rgb,
 output wire out_de,out_valid,out_hs,out_vs,
 output wire [31:0] time_gray,
 output wire [15:0] fps,latency_ms,output wire fps_valid,latency_valid,
 output wire [7:0] shown_valid,output wire detector_overflow,
 output wire track_visible,output wire [15:0] track_id,center_x,center_y,
 output wire follow_commit,follow_overflow,output wire[31:0]follow_frame,
 output wire[3:0]follow_valid,output wire[63:0]follow_x0,follow_x1,follow_y0,follow_y1,
 output wire motion_found,lighting_change,output wire [15:0] motion_count
);
 wire [15:0] fx,fy;wire [23:0] processed,analysis_rgb;wire fde,fvalid,fhs,fvs;
 wire [2:0] fmode;wire [7:0] edge_strength;wire edge_valid;
 contest_effects_720p effects(.clk(clk),.rst_n(rst_n),.mode(mode),.adjustment(adjustment),
  .x(x),.y(y),.rgb(rgb),.de(de),.valid(valid),.hs(hs),.vs(vs),
  .out_x(fx),.out_y(fy),.out_rgb(processed),.raw_rgb_out(analysis_rgb),
  .out_de(fde),.out_valid(fvalid),.out_hs(fhs),.out_vs(fvs),.out_mode(fmode),
  .edge_strength(edge_strength),.edge_valid(edge_valid));
 reg [32:0] saved_epoch;reg epoch_seen;
 wire source_changed=!epoch_seen||source_epoch!=saved_epoch;
 always @(posedge clk)begin
  if(!rst_n)begin saved_epoch<=0;epoch_seen<=0;end
  else if(fde&&fx==0&&fy==0)begin saved_epoch<=source_epoch;epoch_seen<=source_valid;end
 end
 contest_performance #(.CLK_HZ(CLK_HZ)) performance(.clk(clk),.rst_n(rst_n),
  .first_pixel(out_de&&out_x==0&&out_y==0),.frame_valid(source_valid&&out_valid&&!source_stale),
  .source_epoch(source_epoch),.frame_id(source_frame_id),.frame_start_ms(source_start_ms),
  .time_gray(time_gray),.fps(fps),.latency_ms(latency_ms),.fps_valid(fps_valid),.latency_valid(latency_valid));
 wire motion_commit,motion_good;wire [31:0] motion_frame;
 wire [15:0] motion_x0,motion_x1,motion_y0,motion_y1;
 video_motion_720p motion(.clk(clk),.rst_n(rst_n),.x(fx),.y(fy),.rgb(analysis_rgb),.de(fde),.valid(fvalid),
  .source_valid(source_valid),.source_stale(source_stale),.source_changed(source_changed),
  .source_frame_id(source_frame_id),.threshold(8'd16+{4'd0,adjustment,2'b00}),
  .result_commit(motion_commit),.result_good(motion_good),.motion_found(motion_found),.lighting_change(lighting_change),
  .result_frame_id(motion_frame),.motion_count(motion_count),.min_x(motion_x0),.max_x(motion_x1),.min_y(motion_y0),.max_y(motion_y1));
 wire detection_commit;wire [31:0] detection_frame;wire [7:0] detection_valid;
 wire [127:0] dx0,dx1,dy0,dy1;
 wire motion_box=mode==7&&motion_found&&motion_good&&!source_changed&&!source_stale&&source_valid&&
  fx>=motion_x0&&fx<=motion_x1&&fy>=motion_y0&&fy<=motion_y1&&
  (fx-motion_x0<2||motion_x1-fx<2||fy-motion_y0<2||motion_y1-fy<2);
 wire center_mark=mode==6&&track_visible&&!source_changed&&!source_stale&&source_valid&&
  (((fx+7>=center_x&&fx<=center_x+7)&&fy==center_y)||
   ((fy+7>=center_y&&fy<=center_y+7)&&fx==center_x));
 wire [23:0] drawing_base=center_mark?24'h00ffff:(motion_box?24'hffff00:processed);
 wire [15:0] ox,oy;wire [23:0] orgb;wire ode,ovalid,ohs,ovs;
 contest_detection_osd #(.USE_ERAM(USE_ERAM)) detection(.clk(clk),.rst_n(rst_n),.mode(fmode),
  .tracker_enable(fmode==6),.tracker_source_changed(source_changed),
  .tracker_visible(track_visible),.tracker_id(track_id),.tracker_center_x(center_x),.tracker_center_y(center_y),
  .box_service_clk(box_service_clk),.box_global_rst_n(box_global_rst_n),.box_worker_ready(box_worker_ready),
  .box_read_word(box_read_word),.box_write_word(box_write_word),.box_write(box_write),.box_wdata(box_wdata),.box_rdata(box_rdata),
  .x(fx),.y(fy),.rgb(analysis_rgb),.base_rgb(drawing_base),.de(fde),.valid(fvalid),.hs(fhs),.vs(fvs),
  .source_valid(source_valid),.source_stale(source_stale),.source_epoch(source_epoch),.source_frame_id(source_frame_id),
  .out_x(ox),.out_y(oy),.out_rgb(orgb),.out_de(ode),.out_valid(ovalid),.out_hs(ohs),.out_vs(ovs),
  .follow_commit(follow_commit),.follow_overflow(follow_overflow),.follow_frame(follow_frame),
  .follow_valid(follow_valid),.follow_x0(follow_x0),.follow_x1(follow_x1),.follow_y0(follow_y0),.follow_y1(follow_y1),
  .shown_valid(shown_valid),.shown_result_valid(),.shown_frame_id(),.overflow(detector_overflow),.detector_busy(),
  .detection_commit(detection_commit),.detection_frame(detection_frame),.detection_valid(detection_valid),
  .detection_x0(dx0),.detection_x1(dx1),.detection_y0(dy0),.detection_y1(dy1));
 contest_status_panel panel(.clk(clk),.rst_n(rst_n),.mode(mode),.camera_remote(camera_remote),
  .x(ox),.y(oy),.rgb(orgb),.de(ode),.valid(ovalid),.hs(ohs),.vs(ovs),
  .fps(fps),.latency_ms(latency_ms),.fps_valid(fps_valid),.latency_valid(latency_valid),
  .track_visible(track_visible),.track_id(track_id),.center_x(center_x),.center_y(center_y),
  .motion_found(motion_found),.lighting_change(lighting_change),.motion_count(motion_count),
  .out_x(out_x),.out_y(out_y),.out_rgb(out_rgb),.out_de(out_de),.out_valid(out_valid),.out_hs(out_hs),.out_vs(out_vs));
endmodule
