`timescale 1ns/1ps
// Four-colour connected components only: red, green, blue and yellow.
// Eight-slot external bus retained for compatibility; upper four are zero.
// No grayscale contour, polygon classifier or shape history is present.
module video_colour_detector #(
 parameter ROI_X=0,ROI_Y=0,ROI_W=1280,ROI_H=720,MAX_OBJECTS=8,
 parameter STACK_DEPTH=1024,MAX_WORK_CYCLES=2000000,STALE_CYCLES=7425000,USE_ERAM=1
)(
 input wire clk,rst_n,frame_start,frame_end,frame_good,abort,
 input wire [31:0] frame_id,
 input wire de,pixel_valid,input wire [15:0] x,y,rgb565,
 output reg result_valid,output reg [31:0] result_frame_id,
 output reg [MAX_OBJECTS-1:0] valid,
 output reg [MAX_OBJECTS*16-1:0] bbox_x0,bbox_x1,bbox_y0,bbox_y1,
 output reg [MAX_OBJECTS*4-1:0] class_id,
 output reg [MAX_OBJECTS*3-1:0] color_id,
 output reg overflow,output wire busy
);
 localparam COLS=ROI_W/4,ROWS=ROI_H/4,N=COLS*ROWS,AB=$clog2(N),SB=$clog2(STACK_DEPTH);
 localparam XB=$clog2(ROI_X+ROI_W+1),YB=$clog2(ROI_Y+ROI_H+1);
 localparam CLEAR=0,READY=1,CAPTURE=2,DRAIN=3,SCAN_READ=4,SCAN_TEST=5,
 SEED_READ=6,SEED_TEST=7,LEFT_READ=8,LEFT_TEST=9,PIX_READ=10,PIX_TEST=11,
 ABOVE_READ=12,ABOVE_TEST=13,BELOW_READ=14,BELOW_TEST=15,NEXT_PIXEL=16,
 POP=17,POP_WAIT=18,POP_GET=19,CHECK_COMPONENT=20,SLOT_BEGIN=21,STORE=22,COMMIT=23;
 reg [4:0] state;
 assign busy=state!=READY;
 reg [2:0] mask_memory[0:N-1];
 wire [16:0] stack_q;
 reg [AB-1:0] clear_address,read_address;
 reg [2:0] map_q;
 reg [SB:0] stack_pointer;
 reg [8:0] scan_x,cur_x;reg [7:0] scan_y,cur_y;
 reg [AB-1:0] scan_address;
 reg [2:0] component_code;
 reg first_border,last_border,above_previous,below_previous;
 reg [31:0] captured_id;reg [15:0] cell_count;
 reg [19:0] pixel_count;reg [XB-1:0] expected_x;reg [YB-1:0] expected_y;reg abort_seen,seen_source;
 reg [31:0] last_started_id;
 reg bad_frame;reg [2:0] drain_count;
 localparam WORK_BITS=$clog2(MAX_WORK_CYCLES+1),STALE_BITS=$clog2(STALE_CYCLES+1);
 reg [WORK_BITS-1:0] work_cycles;reg [STALE_BITS-1:0] stale_cycles;
 wire working=state>=SCAN_READ;
 wire expired=stale_cycles>=STALE_CYCLES-1;
 // Explicit shifts prevent TD from consuming DSPs for the constant stride.
 function [AB-1:0] row_base;input [7:0] row_value;
  reg [AB-1:0] extended;integer bit_number;
  begin
   extended=row_value;
   if(COLS==320)row_base=(extended<<8)+(extended<<6);
   else if(COLS==240)row_base=(extended<<8)-(extended<<4);
   else begin row_base=0;for(bit_number=0;bit_number<9;bit_number=bit_number+1)
    if((COLS>>bit_number)&1)row_base=row_base+(extended<<bit_number);
   end
  end
 endfunction
 wire [AB-1:0] current_address=row_base(cur_y)+cur_x;
 // Full 4x4 averaging reduces single-pixel sensor noise before thresholding.
 wire [23:0] average_rgb;wire average_valid;
 multi_grid_average #(.OFFSET_X(ROI_X),.OFFSET_Y(ROI_Y),.IMAGE_WIDTH(ROI_W),
 .IMAGE_HEIGHT(ROI_H)) average(
 .clk(clk),.rst_n(rst_n),.x(x),.y(y),.de(de),.valid(pixel_valid),
 .rgb565(rgb565),.averaged_rgb888(average_rgb),.averaged_valid(average_valid));
 reg [8:0] average_x;reg [7:0] average_y;
 always @(posedge clk) begin average_x<=(x-ROI_X)>>2;average_y<=(y-ROI_Y)>>2;end
 wire [7:0] ar=average_rgb[23:16];
 wire [7:0] ag=average_rgb[15:8];
 wire [7:0] ab=average_rgb[7:0];
 wire [7:0] max_rb=ar>ab?ar:ab,min_rb=ar<ab?ar:ab;
 wire [7:0] green_dominance=ag-max_rb;
 wire [2:0] average_color_raw=(ar>ab+9'd18&&ag>ab+9'd18&&ar+9'd20>=ag&&ag+9'd28>=ar)?3'd4:
 (ar>ag+9'd14&&ar>ab+9'd14)?3'd1:
 (ag>ar+9'd9&&ag>ab+9'd5&&{green_dominance,4'b0000}>={4'd0,ag})?3'd2:
 (ab>ar+9'd7&&ab+9'd12>=ag)?3'd3:3'd0;
 // Reject weak colour casts before connectivity. Relative contrast preserves
 // the same test across brightness changes; constant products are shifts/adds.
 wire [7:0] maximum_channel=ag>max_rb?ag:max_rb;
 wire [7:0] minimum_channel=ag<min_rb?ag:min_rb;
 wire [7:0] channel_span=maximum_channel-minimum_channel;
 wire [11:0] span_times12={1'b0,channel_span,3'b000}+{2'b00,channel_span,2'b00};
 wire [11:0] span_times16={channel_span,4'b0000};
 wire colour_confident=(average_color_raw==3?span_times16:span_times12)>={4'd0,maximum_channel}&&
  (average_color_raw!=2||(ag>=max_rb&&{green_dominance,4'b0000}>={4'd0,ag}));
 // Code 7 marks rejected colour casts and is not a new object class.
 wire [2:0] average_color=average_color_raw==0?3'd0:
  (colour_confident?average_color_raw:3'd7);
 // Four colour rows; no grayscale rows or adaptive-threshold sums.
 reg [11:0] row_memory[0:COLS-1];reg [11:0] row_q;
 reg front_pending;reg [8:0] front_x;reg [7:0] front_y;reg [2:0] front_color;
 reg [17:0] colour_columns;reg [5:0] filtered_colour_delay;
 // Columns x-2/x-1 and the RAM column x form a 3x3 colour window centred
 // at (x-1,y-3). Two cell delays retain the original output centre (x-3,y-3).
 wire [2:0] colour_center=colour_columns[5:3];
 wire [3:0] same_colour_support=4'd1+
  {3'd0,(colour_columns[17:15]==colour_center)}+
  {3'd0,(colour_columns[14:12]==colour_center)}+
  {3'd0,(colour_columns[11:9]==colour_center)}+
  {3'd0,(colour_columns[8:6]==colour_center)}+
  {3'd0,(colour_columns[2:0]==colour_center)}+
  {3'd0,(row_q[11:9]==colour_center)}+
  {3'd0,(row_q[8:6]==colour_center)}+
  {3'd0,(row_q[5:3]==colour_center)};
 // Only remove unsupported centres; never manufacture a colour by voting.
 wire [2:0] supported_colour=colour_center==0?3'd0:
  ((colour_center<=4&&same_colour_support>=5)?colour_center:3'd7);
 wire front_write=front_pending&&state==CAPTURE&&front_x>=6&&front_y>=6;
 wire [AB-1:0] front_address=row_base(front_y-8'd3)+front_x-3;
 wire [2:0] front_class=filtered_colour_delay[5:3]<=4?filtered_colour_delay[5:3]:3'd0;
 always @(posedge clk) begin
  if(average_valid&&state==CAPTURE) row_q<=row_memory[average_x];
  if(front_pending&&state==CAPTURE) row_memory[front_x]<=
   {row_q[8:0],front_color};
 if(!rst_n||abort||frame_start) begin front_pending<=0;colour_columns<=0;filtered_colour_delay<=0;end
  else begin
   front_pending<=average_valid&&state==CAPTURE;
   if(average_valid&&state==CAPTURE)begin
    front_x<=average_x;front_y<=average_y;front_color<=average_color;
   end
   if(front_pending&&state==CAPTURE)begin
    colour_columns<=front_x==0?{9'd0,row_q[11:3]}:{colour_columns[8:0],row_q[11:3]};
    filtered_colour_delay<=front_x==0?6'd0:{filtered_colour_delay[2:0],supported_colour};
   end
  end
 end
 // Lab-only green opening. All non-green map cells are unchanged.
 wire green_filter_valid,green_filter_value;
 wire [8:0] green_centre_x;wire [7:0] green_centre_y;
 green_open_cross_grid #(.COLS(COLS),.ROWS(ROWS)) green_filter(
  .clk(clk),.rst_n(rst_n),.frame_start(frame_start),.abort(abort),
  .cell_valid(average_valid&&state==CAPTURE),.cell_x(average_x),.cell_y(average_y),
  .raw_green(average_color==2),.filtered_valid(green_filter_valid),
  .filtered_green(green_filter_value),.centre_x(green_centre_x),.centre_y(green_centre_y));
 reg map_front_write;reg [AB-1:0] map_front_address;reg [2:0] map_front_class;
 always @(posedge clk)begin
  if(!rst_n||abort||frame_start)begin map_front_write<=0;map_front_address<=0;map_front_class<=0;end
  else begin
   map_front_write<=front_write;
   if(front_write)begin map_front_address<=front_address;map_front_class<=front_class;end
  end
 end
 wire [2:0] cleaned_front_class=map_front_class==2&&!green_filter_value?3'd0:map_front_class;
 // Single read port is shared by scan, expansion and neighbours. The two
 // states for every read explicitly respect synchronous block-RAM latency.
 always @* begin
  read_address=current_address;
  case(state)
   SCAN_READ,SCAN_TEST:read_address=scan_address;
   LEFT_READ,LEFT_TEST:read_address=current_address-1'b1;
   ABOVE_READ,ABOVE_TEST:read_address=current_address-COLS;
   BELOW_READ,BELOW_TEST:read_address=current_address+COLS;
  endcase
 end
 wire remove_pixel=state==PIX_TEST&&!first_border&&map_q==component_code;
 always @(posedge clk) begin
  if(read_address<N)map_q<=mask_memory[read_address];
  if(state==CLEAR)mask_memory[clear_address]<=0;
  else if(map_front_write)mask_memory[map_front_address]<=cleaned_front_class;
  else if(remove_pixel)mask_memory[current_address]<=0;
 end
 wire stack_push_above=state==ABOVE_TEST&&cur_y!=0&&map_q==component_code&&!above_previous;
 wire stack_push_below=state==BELOW_TEST&&cur_y<ROWS-1&&map_q==component_code&&!below_previous;
 wire stack_write=(stack_push_above||stack_push_below)&&stack_pointer<STACK_DEPTH;
 wire [16:0] stack_data={stack_push_above?cur_y-8'd1:cur_y+8'd1,cur_x};
 wire stack_read=state==POP&&stack_pointer!=0;
 wire [SB-1:0] stack_read_address=stack_pointer-1'b1;
 generate if(USE_ERAM)begin: device_stack
  PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(17),.DATA_WIDTH_B(17),.ADDR_WIDTH_A(SB),.ADDR_WIDTH_B(SB),
   .DATA_DEPTH_A(STACK_DEPTH),.DATA_DEPTH_B(STACK_DEPTH),.BYTE_ENABLE(0),.BYTE_A(1),.BYTE_B(1),
   .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.CLKMODE("SYNC")) stack_ram(
   .dia(stack_data),.dib(17'd0),.addra(stack_pointer[SB-1:0]),.addrb(stack_read_address),.clka(clk),.clkb(clk),
   .cea(stack_write),.ceb(stack_read),.ocea(1'b1),.oceb(1'b1),.wea(stack_write),.web(1'b0),
   .rsta(1'b0),.rstb(1'b0),.bea(1'b1),.beb(1'b1),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),
   .ecc_sbiterr(),.ecc_dbiterr(),.doa(),.dob(stack_q));
 end else begin: portable_stack
  reg [16:0] memory[0:STACK_DEPTH-1];reg [16:0] q;assign stack_q=q;
  always @(posedge clk)begin
   if(stack_write)memory[stack_pointer[SB-1:0]]<=stack_data;
   if(stack_read)q<=memory[stack_read_address];
  end
 end endgenerate
 reg [15:0] component_area;reg [8:0] min_x,max_x;reg [7:0] min_y,max_y;
 wire [8:0] box_width=max_x-min_x+1'b1;
 wire [7:0] box_height=max_y-min_y+1'b1;
 wire [15:0] box_area=box_width*box_height;
 // Original colour ranking and candidate eligibility are unchanged.
 wire [15:0] colour_area_score=component_area-{2'b00,box_area[15:2]};
 wire colour_dense=({component_area,2'b00}>={1'b0,box_area,1'b0}+{2'b00,box_area});
 wire [15:0] color_score={3'b000,colour_dense,colour_area_score[11:0]};
 wire [9:0] width_times3={1'b0,box_width,1'b0}+{2'b00,box_width};
 wire [9:0] height_times3={1'b0,box_height,1'b0}+{2'b00,box_height};
 wire [16:0] component_times3={component_area,1'b0}+{1'b0,component_area};
 reg [1:0] selected_slot;
 reg [12:0] slot_score[0:3];
 reg [3:0] next_valid;
 reg [63:0] next_x0,next_x1,next_y0,next_y1;
 reg [15:0] next_class;reg [11:0] next_color;
 integer i;
 task advance_scan;begin
  scan_address<=scan_address+1'b1;
  if(scan_x==COLS-1)begin scan_x<=0;if(scan_y==ROWS-1)begin
   state<=COMMIT;
  end else begin scan_y<=scan_y+1'b1;state<=SCAN_READ;end end
  else begin scan_x<=scan_x+1'b1;state<=SCAN_READ;end
 end endtask
 task fail_frame;begin
  overflow<=1;valid<=0;result_valid<=1;result_frame_id<=captured_id;
  state<=CLEAR;clear_address<=0;
 end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=CLEAR;clear_address<=0;result_valid<=0;result_frame_id<=0;valid<=0;
   bbox_x0<=0;bbox_x1<=0;bbox_y0<=0;bbox_y1<=0;class_id<=0;color_id<=0;overflow<=0;
   captured_id<=0;cell_count<=0;pixel_count<=0;expected_x<=ROI_X;expected_y<=ROI_Y;abort_seen<=0;seen_source<=0;last_started_id<=0;
   bad_frame<=0;drain_count<=0;work_cycles<=0;stale_cycles<=0;
   scan_x<=0;scan_y<=0;scan_address<=0;cur_x<=0;cur_y<=0;stack_pointer<=0;component_code<=0;
   first_border<=0;last_border<=0;above_previous<=0;below_previous<=0;
   component_area<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;
   selected_slot<=0;
   next_valid<=0;next_x0<=0;next_x1<=0;next_y0<=0;next_y1<=0;next_class<=0;next_color<=0;
   for(i=0;i<4;i=i+1)slot_score[i]<=0;
  end else begin
   result_valid<=0;
   abort_seen<=abort;
   if(stale_cycles<STALE_CYCLES)stale_cycles<=stale_cycles+1'b1;
   if(working)work_cycles<=work_cycles+1'b1;
   if(abort)begin valid<=0;result_valid<=!abort_seen;overflow<=0;state<=CLEAR;clear_address<=0;end
   else if(expired)begin if(valid!=0)begin valid<=0;result_valid<=1;end end
   if(!abort)case(state)
    CLEAR:begin if(clear_address==N-1)begin clear_address<=0;state<=READY;end else clear_address<=clear_address+1'b1;end
    READY:if(frame_start&&(!seen_source||frame_id!=last_started_id))begin
     state<=CAPTURE;captured_id<=frame_id;cell_count<=0;pixel_count<=0;
     seen_source<=1;last_started_id<=frame_id;
     expected_x<=ROI_X;expected_y<=ROI_Y;bad_frame<=0;
     if(de&&x>=ROI_X&&x<ROI_X+ROI_W&&y>=ROI_Y&&y<ROI_Y+ROI_H)begin
      pixel_count<=1;
      if(!pixel_valid||x!=ROI_X||y!=ROI_Y)bad_frame<=1;
      if(ROI_W==1)expected_y<=ROI_Y+1'b1;else expected_x<=ROI_X+1'b1;
     end
     next_valid<=0;next_x0<=0;next_x1<=0;next_y0<=0;next_y1<=0;next_class<=0;next_color<=0;
     for(i=0;i<4;i=i+1)slot_score[i]<=0;
    end
    CAPTURE:begin
     if(de&&x>=ROI_X&&x<ROI_X+ROI_W&&y>=ROI_Y&&y<ROI_Y+ROI_H)begin
      if(!pixel_valid||x!=expected_x||y!=expected_y||frame_id!=captured_id)bad_frame<=1;
      pixel_count<=pixel_count+1'b1;
      if(expected_x==ROI_X+ROI_W-1)begin expected_x<=ROI_X;expected_y<=expected_y+1'b1;end
      else expected_x<=expected_x+1'b1;
     end
     if(average_valid)begin
      cell_count<=cell_count+1'b1;
      // Full pixel order is checked above; no duplicate stride multiplier.
     end
     if(frame_end)begin if(!frame_good)bad_frame<=1;drain_count<=7;state<=DRAIN;end
     else if(frame_start)fail_frame;
    end
    DRAIN:begin
     if(drain_count!=0)drain_count<=drain_count-1'b1;
     else if(bad_frame||cell_count!=N||pixel_count!=ROI_W*ROI_H)fail_frame;
     else begin scan_x<=0;scan_y<=0;scan_address<=0;work_cycles<=0;state<=SCAN_READ;end
    end
    SCAN_READ:state<=SCAN_TEST;
    SCAN_TEST:begin
     if(map_q==0)advance_scan;
     else begin
      component_code<=map_q;cur_x<=scan_x;cur_y<=scan_y;stack_pointer<=0;component_area<=0;
      min_x<=COLS-1;max_x<=0;min_y<=ROWS-1;max_y<=0;state<=SEED_READ;
     end
    end
    SEED_READ:state<=SEED_TEST;
    SEED_TEST:if(map_q!=component_code)state<=POP;
     else if(cur_x==0)begin first_border<=0;last_border<=0;above_previous<=0;below_previous<=0;state<=PIX_READ;end
     else state<=LEFT_READ;
    LEFT_READ:state<=LEFT_TEST;
    LEFT_TEST:begin
     if(map_q==component_code)begin
      cur_x<=cur_x-1'b1;
      if(cur_x==1)begin first_border<=0;last_border<=0;above_previous<=0;below_previous<=0;state<=PIX_READ;end
      else state<=LEFT_READ;
     end else begin cur_x<=cur_x-1'b1;first_border<=1;last_border<=0;above_previous<=0;below_previous<=0;state<=PIX_READ;end
    end
    PIX_READ:state<=PIX_TEST;
    PIX_TEST:begin
     last_border<=!first_border&&map_q!=component_code;
     if(!first_border&&map_q==component_code)begin
      component_area<=component_area+1'b1;
      if(cur_x<min_x)min_x<=cur_x;if(cur_x>max_x)max_x<=cur_x;
      if(cur_y<min_y)min_y<=cur_y;if(cur_y>max_y)max_y<=cur_y;
     end
     state<=ABOVE_READ;
    end
    ABOVE_READ:state<=ABOVE_TEST;
    ABOVE_TEST:begin
     above_previous<=cur_y!=0&&map_q==component_code;
     if(cur_y!=0&&map_q==component_code&&!above_previous)begin
      if(stack_pointer==STACK_DEPTH)fail_frame;else begin stack_pointer<=stack_pointer+1'b1;state<=BELOW_READ;end
     end else state<=BELOW_READ;
    end
    BELOW_READ:state<=BELOW_TEST;
    BELOW_TEST:begin
     below_previous<=cur_y<ROWS-1&&map_q==component_code;
     if(cur_y<ROWS-1&&map_q==component_code&&!below_previous)begin
      if(stack_pointer==STACK_DEPTH)fail_frame;else begin stack_pointer<=stack_pointer+1'b1;state<=NEXT_PIXEL;end
     end else state<=NEXT_PIXEL;
    end
    NEXT_PIXEL:begin first_border<=0;if(last_border||cur_x==COLS-1)state<=POP;else begin cur_x<=cur_x+1'b1;state<=PIX_READ;end end
    POP:begin if(stack_pointer==0)state<=CHECK_COMPONENT;else begin stack_pointer<=stack_pointer-1'b1;state<=POP_WAIT;end end
    POP_WAIT:state<=POP_GET;
    POP_GET:begin cur_x<=stack_q[8:0];cur_y<=stack_q[16:9];state<=SEED_READ;end
    CHECK_COMPONENT:begin
     if(box_width<5||box_height<5||box_width>64||box_height>64||box_width>height_times3||box_height>width_times3||component_area<10)advance_scan;
     else if({box_width,1'b0}>height_times3||{box_height,1'b0}>width_times3)advance_scan;
     else state<=SLOT_BEGIN;
    end
    SLOT_BEGIN:begin selected_slot<=component_code-1'b1;state<=STORE;end
    STORE:begin
     if(color_score>slot_score[selected_slot]&&component_times3>=box_area)begin
      slot_score[selected_slot]<=color_score;next_valid[selected_slot]<=1;
      next_x0[selected_slot*16+:16]<=ROI_X+min_x*4;next_x1[selected_slot*16+:16]<=ROI_X+max_x*4+3;
      next_y0[selected_slot*16+:16]<=ROI_Y+min_y*4;next_y1[selected_slot*16+:16]<=ROI_Y+max_y*4+3;
      next_class[selected_slot*4+:4]<=4'd4;
      next_color[selected_slot*3+:3]<=component_code;
     end
     advance_scan;
    end
    COMMIT:begin
     valid<={4'd0,next_valid};
     bbox_x0<={64'd0,next_x0};bbox_x1<={64'd0,next_x1};bbox_y0<={64'd0,next_y0};bbox_y1<={64'd0,next_y1};
     class_id<={16'd0,next_class};color_id<={12'd0,next_color};result_frame_id<=captured_id;result_valid<=1;
     stale_cycles<=0;overflow<=0;state<=CLEAR;clear_address<=0;
    end
   endcase
   if(!abort&&working&&work_cycles>=MAX_WORK_CYCLES-1)fail_frame;
  end
 end
 // synthesis translate_off
 initial if(ROI_W<4||ROI_W>1280||ROI_H<4||ROI_H>720||ROI_W%4!=0||ROI_H%4!=0||MAX_OBJECTS!=8)
  $fatal(1,"Unsupported detector geometry/slots");
 // synthesis translate_on
endmodule
