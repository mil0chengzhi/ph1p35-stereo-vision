`timescale 1ns/1ps

// Resource bounded multi-object detector. No RGB frame buffer: a320x180

// 3-bit class map is collected on alternate display frames, then consumed by

// an iterative eight-connected scan-line flood fill. Overflow invalidates the

// complete result; partial region sets are never published as successful.

// Class: 0 unknown, 1 triangle, 2 circle, 3 quadrilateral, 4 colour block.

// Colour: 0 none, 1 red, 2 green, 3 blue, 4 yellow.

// Screen-coordinate closed bboxes, low packed slice is object zero.

module video_multi_detector #(

 parameter ROI_X=0,ROI_Y=0,ROI_W=1280,ROI_H=720,MAX_OBJECTS=8,

 parameter STACK_DEPTH=1024,MAX_WORK_CYCLES=2000000,STALE_CYCLES=7425000,USE_ERAM=1,SHAPE_CONFIRM=1

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

 POP=17,POP_WAIT=18,POP_GET=19,POLY_BEGIN=20,POLY_POINT=21,POLY_MUL=22,

 POLY_TEST=23,POLY_PASS=24,STORE=25,COMMIT=26,AREA_POINT=27,AREA_MUL=28,AREA_ACC=29,CONFIRM_LOAD=30,CONFIRM_TEST=31,

 EXTREME_UPDATE=32,AREA_NEXT=33,POLY_PREVIOUS=34,POLY_NEXT=35,SLOT_BEGIN=36,SLOT_SEARCH=37;

 reg [5:0] state;

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

 reg [23:0] work_cycles,stale_cycles;

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

 wire [9:0] gray_sum={2'b0,ar}+{1'b0,ag,1'b0}+{2'b0,ab};

 wire [7:0] average_gray=gray_sum[9:2];

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

 // Code 7 remembers a rejected coloured pixel. It must not turn into a new

 // grey contour: the pre-existing grey mask is preserved bit for bit.

 wire [2:0] average_color=average_color_raw==0?3'd0:

  (colour_confident?average_color_raw:3'd7);

 // One packed row memory supplies six preceding grey rows and four colour

 // rows. The extra colour row is 720 bits at 240 columns, not a frame map.

 // No reset on inferred RAM; output is gated until all rows are fresh.

 reg [59:0] row_memory[0:COLS-1];reg [59:0] row_q;

 reg front_pending;reg [8:0] front_x;reg [7:0] front_y,front_gray;reg [2:0] front_color;

 reg [65:0] horizontal_sums;reg [13:0] horizontal_total;reg [23:0] center_gray;

 reg [17:0] colour_columns;reg [5:0] filtered_colour_delay;

 // Columns x-2/x-1 and the RAM column x form a 3x3 colour window centred

 // at (x-1,y-3). Two cell delays align it with the grey centre (x-3,y-3).

 wire [2:0] colour_center=colour_columns[5:3];

 wire [3:0] same_colour_support=4'd1+

  {3'd0,(colour_columns[17:15]==colour_center)}+

  {3'd0,(colour_columns[14:12]==colour_center)}+

  {3'd0,(colour_columns[11:9]==colour_center)}+

  {3'd0,(colour_columns[8:6]==colour_center)}+

  {3'd0,(colour_columns[2:0]==colour_center)}+

  {3'd0,(row_q[59:57]==colour_center)}+

  {3'd0,(row_q[56:54]==colour_center)}+

  {3'd0,(row_q[53:51]==colour_center)};

 // Only remove unsupported centres; never manufacture a colour by voting.

 wire [2:0] supported_colour=colour_center==0?3'd0:

  ((colour_center<=4&&same_colour_support>=5)?colour_center:3'd7);

 wire [10:0] vertical_sum={3'd0,front_gray}+{3'd0,row_q[7:0]}+{3'd0,row_q[15:8]}+

 {3'd0,row_q[23:16]}+{3'd0,row_q[31:24]}+{3'd0,row_q[39:32]}+{3'd0,row_q[47:40]};

 // Maintain the preceding six-column sum rather than rebuilding a seven-input

 // adder tree for every cell. 7*7*255=12495 fits exactly within 14 bits.

 wire [13:0] square_sum=horizontal_total+{3'd0,vertical_sum};

 wire [13:0] center_times_49=({6'd0,center_gray[23:16]}<<5)+

 ({6'd0,center_gray[23:16]}<<4)+{6'd0,center_gray[23:16]};

 wire front_write=front_pending&&state==CAPTURE&&front_x>=6&&front_y>=6;

 wire [AB-1:0] front_address=row_base(front_y-8'd3)+front_x-3;

 wire [2:0] front_class=filtered_colour_delay[5:3]!=0?

  (filtered_colour_delay[5:3]<=4?filtered_colour_delay[5:3]:3'd0):

  ((square_sum>=center_times_49+14'd98)?3'd5:3'd0);

 always @(posedge clk) begin

  if(average_valid&&state==CAPTURE) row_q<=row_memory[average_x];

  if(front_pending&&state==CAPTURE) row_memory[front_x]<=

   {row_q[56:48],front_color,row_q[39:0],front_gray};

 if(!rst_n||abort||frame_start) begin front_pending<=0;horizontal_sums<=0;horizontal_total<=0;center_gray<=0;colour_columns<=0;filtered_colour_delay<=0;end

  else begin

   front_pending<=average_valid&&state==CAPTURE;

   if(average_valid&&state==CAPTURE)begin

    front_x<=average_x;front_y<=average_y;front_gray<=average_gray;front_color<=average_color;

   end

   if(front_pending&&state==CAPTURE)begin

    horizontal_sums<=front_x==0?{55'd0,vertical_sum}:{horizontal_sums[54:0],vertical_sum};

    horizontal_total<=front_x==0?{3'd0,vertical_sum}:square_sum-{3'd0,horizontal_sums[65:55]};

    center_gray<=front_x==0?{16'd0,row_q[23:16]}:{center_gray[15:0],row_q[23:16]};

    colour_columns<=front_x==0?{9'd0,row_q[59:51]}:{colour_columns[8:0],row_q[59:51]};

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

 // One asynchronous read and one write port replaces the former four parallel

 // array readers. The first grey pixel initializes every entry before use;

 // no reset mux is needed on this small distributed memory.

 reg [16:0] extreme_memory[0:7];reg [2:0] extreme_index,walk_index,vertex_index;

 reg extreme_first;reg [8:0] center_x;reg [7:0] center_y;

 wire [2:0] extreme_address=state==EXTREME_UPDATE?extreme_index:

  ((state==AREA_POINT||state==POLY_POINT)?vertex_index:walk_index);

 wire [16:0] extreme_point=extreme_memory[extreme_address];

 wire [8:0] extreme_px=extreme_point[8:0];wire [7:0] extreme_py=extreme_point[16:9];

 wire area_point_state=state==AREA_POINT||state==AREA_NEXT;

 wire previous_point_state=state==POLY_PREVIOUS;

 wire [8:0] difference_lx=previous_point_state?center_x:extreme_px;

 wire [7:0] difference_ly=previous_point_state?center_y:extreme_py;

 wire [8:0] difference_rx=previous_point_state?extreme_px:(area_point_state?min_x:center_x);

 wire [7:0] difference_ry=previous_point_state?extreme_py:(area_point_state?min_y:center_y);

 // All shape differences are in [-63,63]; area coordinates are [0,63].

 wire signed [7:0] point_dx=difference_lx-difference_rx;

 wire signed [7:0] point_dy=difference_ly-difference_ry;

 function signed [9:0] projection;input [1:0] direction;input [8:0] px;input [7:0] py;

  begin case(direction)

   0:projection=$signed({2'd0,px});1:projection=$signed({2'd0,px})+$signed({2'd0,py});

   2:projection=$signed({2'd0,py});3:projection=$signed({2'd0,py})-$signed({2'd0,px});

  endcase end

 endfunction

 wire signed [9:0] current_projection=projection(extreme_index[1:0],cur_x,cur_y);

 wire signed [9:0] stored_projection=projection(extreme_index[1:0],extreme_px,extreme_py);

 wire replace_extreme=extreme_index[2]?current_projection<stored_projection:current_projection>stored_projection;

 always @(posedge clk)if(!abort&&state==EXTREME_UPDATE&&(extreme_first||replace_extreme))

  extreme_memory[extreme_index]<={cur_y,cur_x};

 reg [7:0] alive;reg [3:0] vertices;

 reg [2:0] smallest_index;reg [12:0] smallest_cross;

 reg [8:0] smallest_limit;

 reg signed [15:0] area_accumulator;reg [12:0] hull_area_twice;

 reg signed [7:0] mul_a,mul_b,mul_c,mul_d;

 reg signed [15:0] product_a,product_b;

 wire signed [16:0] cross_signed=$signed(product_a)-$signed(product_b);

 wire [16:0] cross_abs=cross_signed[16]?-cross_signed:cross_signed;

 wire [8:0] previous_edge_length=(mul_a[7]?-mul_a:mul_a)+(mul_c[7]?-mul_c:mul_c);

 wire [8:0] next_edge_length=(mul_b[7]?-mul_b:mul_b)+(mul_d[7]?-mul_d:mul_d);

 wire [12:0] simplified_cross=(previous_edge_length<=2||next_edge_length<=2)?13'd0:cross_abs[12:0];

 wire signed [8:0] chord_x_signed=$signed(mul_a)+$signed(mul_d);

 wire signed [8:0] chord_y_signed=$signed(mul_b)+$signed(mul_c);

 wire [8:0] chord_x=chord_x_signed[8]?-chord_x_signed:chord_x_signed;

 wire [8:0] chord_y=chord_y_signed[8]?-chord_y_signed:chord_y_signed;

 wire [8:0] box_width=max_x-min_x+1'b1;

 wire [7:0] box_height=max_y-min_y+1'b1;

 wire [15:0] box_area=box_width*box_height;

 // Prefer a compact colour component over a slightly larger, sparse colour

 // cast on the background. Eligibility below still requires area*3>=box_area,

 // so this subtraction is positive for every stored colour candidate. At the

 // largest accepted 64x64 box its maximum is 3072: slot_score needs no widening.

 // Keep the existing13-bit colour slot score and candidate eligibility.

 // A >=75% filled component outranks sparse colour casts. This helps a small

 // block compete with a larger pink chair or background region. Shape ranking

 // and source-age/withdrawal/confirmation rules are unchanged.

 wire [15:0] colour_area_score=component_area-{2'b00,box_area[15:2]};

 wire colour_dense=({component_area,2'b00}>={1'b0,box_area,1'b0}+{2'b00,box_area});

 wire [15:0] color_score={3'b000,colour_dense,colour_area_score[11:0]};

 wire [7:0] side_sum={1'b0,box_width[6:0]}+{1'b0,box_height[6:0]};

 wire [6:0] longest_chord=chord_x>chord_y?chord_x[6:0]:chord_y[6:0];

 wire [14:0] corner_product=side_sum*longest_chord;

 wire [8:0] corner_limit=corner_product[12:4];

 wire signed [16:0] area_next=$signed(area_accumulator)+$signed(cross_signed);

 wire [15:0] hull_times5={1'b0,hull_area_twice,2'b00}+{3'b000,hull_area_twice};

 wire [14:0] hull_times3={1'b0,hull_area_twice,1'b0}+{2'b00,hull_area_twice};

 wire [15:0] component_times5={1'b0,component_area[12:0],2'b00}+{3'b000,component_area[12:0]};

 wire [16:0] component_times3={component_area,1'b0}+{1'b0,component_area};

 wire [15:0] box_times9={box_area[12:0],3'b000}+{3'b000,box_area[12:0]};

 wire [9:0] width_times3={1'b0,box_width,1'b0}+{2'b00,box_width};

 wire [9:0] height_times3={1'b0,box_height,1'b0}+{2'b00,box_height};

 reg [3:0] detected_class;reg [2:0] selected_slot,search_slot;reg [12:0] best_score;

 reg [12:0] slot_score[0:MAX_OBJECTS-1];

 reg [MAX_OBJECTS-1:0] next_valid;

 reg [MAX_OBJECTS*16-1:0] next_x0,next_x1,next_y0,next_y1;

 reg [MAX_OBJECTS*4-1:0] next_class;

 reg [MAX_OBJECTS*3-1:0] next_color;

 // At most four shape candidates, independent of colour slots. Match any

 // previous slot because area ranking may reorder the candidates. Store raw

 // candidates (not only confirmed outputs) so first sightings can mature.

 reg [3:0] history_valid,confirmed_shapes,history_used;

 reg [8:0] history_x0[0:3],history_x1[0:3];

 reg [7:0] history_y0[0:3],history_y1[0:3];reg [1:0] history_class[0:3];

 reg [1:0] confirm_slot,history_index,current_class;

 reg [8:0] current_x0,current_x1;reg [7:0] current_y0,current_y1;

 wire [2:0] current_slot={1'b1,confirm_slot};

 wire [31:0] source_gap=captured_id-result_frame_id;

 wire history_fresh=source_gap!=0&&source_gap<=2;

 function near_x;input [8:0] a,b;reg signed [9:0] delta;

  begin delta=$signed({1'b0,a})-$signed({1'b0,b});near_x=delta>=-10'sd3&&delta<=10'sd3;end

 endfunction

 function near_y;input [7:0] a,b;reg signed [8:0] delta;

  begin delta=$signed({1'b0,a})-$signed({1'b0,b});near_y=delta>=-9'sd3&&delta<=9'sd3;end

 endfunction

 wire history_match=history_valid[history_index]&&!history_used[history_index]&&

  current_class==history_class[history_index]&&near_x(current_x0,history_x0[history_index])&&

  near_x(current_x1,history_x1[history_index])&&near_y(current_y0,history_y0[history_index])&&

  near_y(current_y1,history_y1[history_index]);

 integer i;

 always @* begin

  // Vertex count alone is insufficient: fragments/text must have a genuine

  // two-dimensional hull, and round targets must occupy most of their bbox.

  detected_class=4'd0;

  if(vertices==3&&{hull_area_twice,1'b0}>=box_area&&component_times5<={hull_area_twice,1'b0})detected_class=1;

  if(vertices==4&&hull_times5>={box_area[12:0],2'b00}&&component_times5<=hull_times3[14:1])detected_class=3;

  if(vertices>=5&&{hull_area_twice,3'b000}>=box_times9&&component_times5<=hull_times3[14:1]&&{box_width,1'b0}<=height_times3&&{box_height,1'b0}<=width_times3)detected_class=2;

 end



 task advance_scan;begin

  scan_address<=scan_address+1'b1;

  if(scan_x==COLS-1)begin scan_x<=0;if(scan_y==ROWS-1)begin

   state<=SHAPE_CONFIRM?CONFIRM_LOAD:COMMIT;confirm_slot<=0;confirmed_shapes<=0;history_used<=0;

  end else begin scan_y<=scan_y+1'b1;state<=SCAN_READ;end end

  else begin scan_x<=scan_x+1'b1;state<=SCAN_READ;end

 end endtask

 task fail_frame;begin

  overflow<=1;valid<=0;result_valid<=1;result_frame_id<=captured_id;

  history_valid<=0;

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

   component_area<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;alive<=0;vertices<=0;

   vertex_index<=0;smallest_index<=0;smallest_cross<=0;smallest_limit<=0;area_accumulator<=0;hull_area_twice<=0;

   extreme_index<=0;walk_index<=0;extreme_first<=0;center_x<=0;center_y<=0;

   selected_slot<=0;search_slot<=0;best_score<=0;

   mul_a<=0;mul_b<=0;mul_c<=0;mul_d<=0;product_a<=0;product_b<=0;

   next_valid<=0;next_x0<=0;next_x1<=0;next_y0<=0;next_y1<=0;next_class<=0;next_color<=0;

   history_valid<=0;confirmed_shapes<=0;history_used<=0;confirm_slot<=0;history_index<=0;

   current_x0<=0;current_x1<=0;current_y0<=0;current_y1<=0;current_class<=0;

   for(i=0;i<MAX_OBJECTS;i=i+1)slot_score[i]<=0;

  end else begin

   result_valid<=0;

   abort_seen<=abort;

   if(stale_cycles<STALE_CYCLES)stale_cycles<=stale_cycles+1'b1;

   if(working)work_cycles<=work_cycles+1'b1;

   if(abort)begin valid<=0;history_valid<=0;result_valid<=!abort_seen;overflow<=0;state<=CLEAR;clear_address<=0;end

   else if(expired)begin history_valid<=0;if(valid!=0)begin valid<=0;result_valid<=1;end end

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

     for(i=0;i<MAX_OBJECTS;i=i+1)slot_score[i]<=0;

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

      extreme_first<=component_area==0;extreme_index<=0;

     end

     // Colour components need only their area and bbox, not a contour hull.

     state<=!first_border&&map_q==component_code&&component_code==5?EXTREME_UPDATE:ABOVE_READ;

    end

    EXTREME_UPDATE:begin

     if(extreme_index==7)state<=ABOVE_READ;else extreme_index<=extreme_index+1'b1;

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

    POP:begin if(stack_pointer==0)state<=POLY_BEGIN;else begin stack_pointer<=stack_pointer-1'b1;state<=POP_WAIT;end end

    POP_WAIT:state<=POP_GET;

    POP_GET:begin cur_x<=stack_q[8:0];cur_y<=stack_q[16:9];state<=SEED_READ;end

    POLY_BEGIN:begin

     if(box_width<5||box_height<5||box_width>64||box_height>64||box_width>height_times3||box_height>width_times3||component_area<10)advance_scan;

     else if(component_code<=4)begin

      // Colour-block mode: reject long strips/cylinders before area ranking.

      if({box_width,1'b0}>height_times3||{box_height,1'b0}>width_times3)advance_scan;

      else begin alive<=0;vertices<=0;state<=SLOT_BEGIN;end

     end

     else if(box_width<10||box_height<10||{box_width,1'b0}>height_times3||{box_height,1'b0}>width_times3)advance_scan;

     else begin alive<=8'hff;vertices<=8;vertex_index<=0;smallest_cross<=13'h1fff;area_accumulator<=0;state<=AREA_POINT;end

    end

    AREA_POINT:begin

     // Translate into the <=64-cell candidate bbox before multiplying.

     mul_a<=point_dx;mul_c<=point_dy;

     walk_index<=vertex_index+1'b1;state<=AREA_NEXT;

    end

    AREA_NEXT:begin mul_b<=point_dy;mul_d<=point_dx;state<=AREA_MUL;end

    AREA_MUL:begin product_a<=mul_a*mul_b;product_b<=mul_c*mul_d;state<=AREA_ACC;end

    AREA_ACC:begin

     area_accumulator<=area_next;

     if(vertex_index==7)begin hull_area_twice<=area_next[16]?-area_next:area_next;vertex_index<=0;state<=POLY_POINT;end

     else begin vertex_index<=vertex_index+1'b1;state<=AREA_POINT;end

    end

    POLY_POINT:begin

     center_x<=extreme_px;center_y<=extreme_py;walk_index<=vertex_index-1'b1;state<=POLY_PREVIOUS;

    end

    POLY_PREVIOUS:if(alive[walk_index])begin

     mul_a<=point_dx;mul_c<=point_dy;

     walk_index<=vertex_index+1'b1;state<=POLY_NEXT;

    end else walk_index<=walk_index-1'b1;

    POLY_NEXT:if(alive[walk_index])begin

     mul_b<=point_dy;mul_d<=point_dx;state<=POLY_MUL;

    end else walk_index<=walk_index+1'b1;

    POLY_MUL:begin product_a<=mul_a*mul_b;product_b<=mul_c*mul_d;state<=POLY_TEST;end

    POLY_TEST:begin

     if(alive[vertex_index]&&simplified_cross<smallest_cross)begin smallest_cross<=simplified_cross;smallest_index<=vertex_index;smallest_limit<=corner_limit;end

     if(vertex_index==7)state<=POLY_PASS;else begin vertex_index<=vertex_index+1'b1;state<=POLY_POINT;end

    end

    POLY_PASS:begin

     if(vertices>3&&smallest_cross<=smallest_limit)begin

      alive[smallest_index]<=0;vertices<=vertices-1'b1;vertex_index<=0;smallest_cross<=13'h1fff;state<=POLY_POINT;

     end else state<=SLOT_BEGIN;

    end

    SLOT_BEGIN:begin

     if(component_code<=4)begin selected_slot<=component_code-1'b1;state<=STORE;end

     else begin selected_slot<=4;search_slot<=5;best_score<=slot_score[4];state<=SLOT_SEARCH;end

    end

    SLOT_SEARCH:begin

     if(slot_score[search_slot]<best_score)begin best_score<=slot_score[search_slot];selected_slot<=search_slot;end

     if(search_slot==MAX_OBJECTS-1)state<=STORE;else search_slot<=search_slot+1'b1;

    end

    STORE:begin

     if((component_code==5?box_area:color_score)>slot_score[selected_slot]&&(component_code==5?detected_class!=0:component_times3>=box_area))begin

      slot_score[selected_slot]<=component_code==5?box_area:color_score;next_valid[selected_slot]<=1;

      next_x0[selected_slot*16+:16]<=ROI_X+min_x*4;next_x1[selected_slot*16+:16]<=ROI_X+max_x*4+3;

      next_y0[selected_slot*16+:16]<=ROI_Y+min_y*4;next_y1[selected_slot*16+:16]<=ROI_Y+max_y*4+3;

      next_class[selected_slot*4+:4]<=component_code==5?detected_class:4'd4;

      next_color[selected_slot*3+:3]<=component_code==5?3'd0:component_code;

     end

     advance_scan;

    end

    CONFIRM_LOAD:begin

     current_x0<=next_x0[current_slot*16+2+:9];current_x1<=next_x1[current_slot*16+2+:9];

     current_y0<=next_y0[current_slot*16+2+:8];current_y1<=next_y1[current_slot*16+2+:8];

     current_class<=next_class[current_slot*4+:2];history_index<=0;

     if(!next_valid[current_slot]||!history_fresh)begin

      if(confirm_slot==3)state<=COMMIT;else confirm_slot<=confirm_slot+1'b1;

     end else state<=CONFIRM_TEST;

    end

    CONFIRM_TEST:begin

     if(history_match)begin

      confirmed_shapes[confirm_slot]<=1;history_used[history_index]<=1;

      if(confirm_slot==3)state<=COMMIT;else begin confirm_slot<=confirm_slot+1'b1;state<=CONFIRM_LOAD;end

     end else if(history_index==3)begin

      if(confirm_slot==3)state<=COMMIT;else begin confirm_slot<=confirm_slot+1'b1;state<=CONFIRM_LOAD;end

     end else history_index<=history_index+1'b1;

    end

    COMMIT:begin

     valid<=SHAPE_CONFIRM?{confirmed_shapes,next_valid[3:0]}:next_valid;

     bbox_x0<=next_x0;bbox_x1<=next_x1;bbox_y0<=next_y0;bbox_y1<=next_y1;

     class_id<=next_class;color_id<=next_color;result_frame_id<=captured_id;result_valid<=1;

     history_valid<=next_valid[7:4];

     for(i=0;i<4;i=i+1)begin

      history_x0[i]<=next_x0[(i+4)*16+2+:9];history_x1[i]<=next_x1[(i+4)*16+2+:9];

      history_y0[i]<=next_y0[(i+4)*16+2+:8];history_y1[i]<=next_y1[(i+4)*16+2+:8];

      history_class[i]<=next_class[(i+4)*4+:2];

     end

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

