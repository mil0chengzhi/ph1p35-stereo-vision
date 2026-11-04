`timescale 1ns/1ps
// Single pixel clock. Analysis sees the picture before drawing any graphics.
// mode0 is bit-exact picture passthrough, with the same two-register latency.
// Source epoch is {cameraID,sourceSession}; input frameID is that source's ID.
// A source/session change, stale source or missing pixel cannot retain old boxes.
module contest_detection_osd #(
 parameter USE_ERAM=1,
 parameter STALE_DISPLAY_FRAMES=12
)(
 input wire box_service_clk,box_global_rst_n,box_worker_ready,
 input wire[3:0]box_read_word,box_write_word,input wire box_write,input wire[31:0]box_wdata,
 output wire[31:0]box_rdata,
 input wire tracker_enable,tracker_source_changed,
 output wire tracker_visible,output wire[15:0]tracker_id,tracker_center_x,tracker_center_y,
 input wire clk,rst_n,input wire [2:0] mode,
 input wire [15:0] x,y,input wire [23:0] rgb,base_rgb,
 input wire de,valid,hs,vs,source_valid,source_stale,
 input wire [32:0] source_epoch,input wire [31:0] source_frame_id,
 output wire [15:0] out_x,out_y,output wire [23:0] out_rgb,
 output wire out_de,out_valid,out_hs,out_vs,
 output wire [7:0] shown_valid,output wire shown_result_valid,
 output wire [31:0] shown_frame_id,output wire overflow,detector_busy,
 output wire follow_commit,follow_overflow,output wire[31:0]follow_frame,
 output wire[3:0]follow_valid,output wire[63:0]follow_x0,follow_x1,follow_y0,follow_y1,
 output reg detection_commit,output reg [31:0] detection_frame,
 output reg [7:0] detection_valid,
 output reg [127:0] detection_x0,detection_x1,detection_y0,detection_y1
);
 wire first_pixel=de&&x==0&&y==0;
 wire frame_end=x==0&&y==720;
 reg [32:0] saved_epoch;
 reg [31:0] last_source_frame;
 reg epoch_seen;reg [3:0] repeated_frames;
 wire changed=!epoch_seen||source_epoch!=saved_epoch;
 wire stale=source_stale||repeated_frames>=STALE_DISPLAY_FRAMES;
 wire cancel=!source_valid||stale||changed;
 always @(posedge clk)begin
  if(!rst_n)begin saved_epoch<=0;epoch_seen<=0;last_source_frame<=0;repeated_frames<=0;end
  else if(first_pixel)begin
   saved_epoch<=source_epoch;epoch_seen<=source_valid;last_source_frame<=source_frame_id;
   if(!source_valid||changed||source_frame_id!=last_source_frame)repeated_frames<=0;
   else if(repeated_frames<15)repeated_frames<=repeated_frames+1'b1;
  end
 end
 wire raw_commit,raw_overflow;wire [31:0] raw_frame;
 wire [7:0] raw_valid;wire [127:0] raw_x0,raw_x1,raw_y0,raw_y1;
 wire [31:0] raw_class;wire [23:0] raw_colour;
 wire [15:0] pixel565={rgb[23:19],rgb[15:10],rgb[7:3]};
 video_colour_detector #(.ROI_X(0),.ROI_Y(0),.ROI_W(1280),.ROI_H(720),.USE_ERAM(USE_ERAM)) detector(
  .clk(clk),.rst_n(rst_n),.frame_start(first_pixel),.frame_end(frame_end),
  .frame_good(source_valid&&!stale),.abort(cancel),.frame_id(source_frame_id),
  .de(de),.pixel_valid(valid),.x(x),.y(y),.rgb565(pixel565),
  .result_valid(raw_commit),.result_frame_id(raw_frame),.valid(raw_valid),
  .bbox_x0(raw_x0),.bbox_x1(raw_x1),.bbox_y0(raw_y0),.bbox_y1(raw_y1),
  .class_id(raw_class),.color_id(raw_colour),.overflow(raw_overflow),.busy(detector_busy));
 assign follow_commit=raw_commit;assign follow_overflow=raw_overflow;assign follow_frame=raw_frame;
 assign follow_valid=raw_valid[3:0];assign follow_x0=raw_x0[63:0];assign follow_x1=raw_x1[63:0];
 assign follow_y0=raw_y0[63:0];assign follow_y1=raw_y1[63:0];
 wire stable_commit,stable_busy;wire [31:0] stable_frame;wire [3:0] stable_valid;
 wire stable_from_service;reg detection_from_service;
 wire [63:0] stable_x0,stable_x1,stable_y0,stable_y1;
 box_tracker_mcu_owner_draft stabilizer(
  .pixel_clk(clk),.service_clk(box_service_clk),.global_reset_n(box_global_rst_n),.pixel_reset_n(rst_n),.worker_ready(box_worker_ready),.source_valid(source_valid&&!changed),.stale(stale),.overflow(raw_overflow),
  .result_valid(raw_commit),.result_frame_id(raw_frame),.valid(raw_valid[3:0]),
  .bbox_x0(raw_x0[63:0]),.bbox_x1(raw_x1[63:0]),.bbox_y0(raw_y0[63:0]),.bbox_y1(raw_y1[63:0]),
  .filtered_commit(stable_commit),.filtered_frame_id(stable_frame),.filtered_valid(stable_valid),
  .filtered_x0(stable_x0),.filtered_x1(stable_x1),.filtered_y0(stable_y0),.filtered_y1(stable_y1),.busy(stable_busy),
  .cpu_read_word(box_read_word),.cpu_write_word(box_write_word),.cpu_write(box_write),.cpu_wdata(box_wdata),.cpu_rdata(box_rdata),
  .track_enable(tracker_enable),.track_source_valid(source_valid),.track_stale(source_stale),.track_changed(tracker_source_changed),.track_overflow(raw_overflow),
  .detection_commit(detection_commit),.detection_from_service(detection_from_service),.detection_red_valid(detection_valid[0]),.detection_frame(detection_frame),
  .red_cells({detection_y1[10:2],detection_y0[10:2],detection_x1[10:2],detection_x0[10:2]}),
  .track_visible(tracker_visible),.track_id(tracker_id),.track_center_x(tracker_center_x),.track_center_y(tracker_center_y),.filtered_from_service(stable_from_service));
 reg [31:0] detection_class;reg [23:0] detection_colour;reg saved_overflow;
 // Publish four colour boxes atomically; unused external slots remain zero.
 always @(posedge clk)begin
  if(!rst_n)begin
   detection_commit<=0;detection_frame<=0;detection_valid<=0;detection_from_service<=0;
   detection_x0<=0;detection_x1<=0;detection_y0<=0;detection_y1<=0;
   detection_class<=0;detection_colour<=0;saved_overflow<=0;
  end else begin
   detection_commit<=0;detection_from_service<=0;
   if(cancel||raw_overflow)begin detection_valid<=0;saved_overflow<=raw_overflow;end
   else if(stable_commit)begin
    detection_commit<=1;detection_frame<=stable_frame;detection_from_service<=stable_from_service;
    detection_valid<={raw_valid[7:4],stable_valid};
    detection_x0<={raw_x0[127:64],stable_x0};detection_x1<={raw_x1[127:64],stable_x1};
    detection_y0<={raw_y0[127:64],stable_y0};detection_y1<={raw_y1[127:64],stable_y1};
    detection_class<=raw_class;detection_colour<=raw_colour;saved_overflow<=raw_overflow;
   end
  end
 end
 assign overflow=raw_overflow;
 wire [15:0] object_x,object_y;wire [23:0] object_rgb;wire object_de;
 wire shown_overflow,label_crowded;
 video_object_osd #(.SCREEN_WIDTH(1280),.SCREEN_HEIGHT(720),.ROI_LEFT(0),.ROI_RIGHT(1279),.RESERVE_CONTEST_HEADER(1),.OBJECT_SLOTS(4),.GRID4_COORDS(1)) objects(
  .clk(clk),.rst_n(rst_n),.x(x),.y(y),.de(de),.base_rgb(base_rgb),
  .enable(mode==1||mode==6),.signal_valid(source_valid&&!changed),.stale(stale||raw_overflow),.snapshot(frame_end),
  .source_frame_id(source_frame_id),.result_valid(detection_commit),.result_frame_id(detection_frame),
  .overflow(saved_overflow),.valid({4'd0,detection_valid[3:0]}),.bbox_x0(detection_x0),.bbox_x1(detection_x1),
  .bbox_y0(detection_y0),.bbox_y1(detection_y1),.class_id(detection_class),.color_id(detection_colour),
  .x_out(object_x),.y_out(object_y),.de_out(object_de),.rgb(object_rgb),
  .shown_valid(shown_valid),.shown_result_valid(shown_result_valid),.shown_frame_id(shown_frame_id),
  .shown_overflow(shown_overflow),.label_crowded(label_crowded));
 reg object_valid,object_hs,object_vs,logo_enable;
 always @(posedge clk)begin
  if(!rst_n)begin object_valid<=0;object_hs<=1;object_vs<=1;logo_enable<=0;end
  else begin object_valid<=valid;object_hs<=hs;object_vs<=vs;logo_enable<=mode!=0;end
 end
 video_anlogic_logo logo(
  .clk(clk),.rst_n(rst_n),.enable(logo_enable),.x(object_x),.y(object_y),.rgb(object_rgb),
  .de(object_de),.valid(object_valid),.hs(object_hs),.vs(object_vs),
  .out_x(out_x),.out_y(out_y),.out_rgb(out_rgb),.out_de(out_de),
  .out_valid(out_valid),.out_hs(out_hs),.out_vs(out_vs));
endmodule
