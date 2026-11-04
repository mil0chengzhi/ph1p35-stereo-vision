`timescale 1ns/1ps
// Streaming8x8 mean/frame difference on native1280x720 picture.
// One160x90x8 history,160x15 mean partials and two160-bit mask rows.
// Only distinct, complete source frames own history. A malformed frame forces
// one whole-frame rebuild. No CCL scan or vertical-blank time budget is needed.
// 5/9 neighbourhood rejection excludes the outermost8px grid border.
// The published bbox describes changed image cells, not semantic object identity.
module video_motion_720p #(
 parameter WIDTH=1280,HEIGHT=720,MIN_CELLS=12
)(
 input wire clk,rst_n,input wire [15:0] x,y,input wire [23:0] rgb,
 input wire de,valid,source_valid,source_stale,source_changed,
 input wire [31:0] source_frame_id,input wire [7:0] threshold,
 output reg result_commit,result_good,motion_found,lighting_change,
 output reg [31:0] result_frame_id,
 output reg [15:0] motion_count,min_x,max_x,min_y,max_y
);
 localparam COLS=WIDTH/8,ROWS=HEIGHT/8,N=COLS*ROWS,AB=$clog2(N),CB=$clog2(COLS);
 wire in_range=x<WIDTH&&y<HEIGHT;
 wire first=de&&x==0&&y==0,finish=x==0&&y==HEIGHT;
 wire cancel=!source_valid||source_stale||source_changed;
 reg active,bad,history_good,compare_history,seen;
 reg [31:0] last_id,working_id;
 reg [19:0] pixel_count;
 reg [15:0] expected_x,expected_y,sample_count;
 reg [7:0] expected_cell_x,expected_cell_y;
 reg [7:0] frame_threshold;
 wire [9:0] gray_sum={2'd0,rgb[23:16]}+{1'd0,rgb[15:8],1'b0}+{2'd0,rgb[7:0]}+10'd2;
 wire [7:0] gray=gray_sum[9:2];
 wire [CB-1:0] column=x>>3;
 (* ram_style="bram" *) reg [14:0] partials[0:COLS-1];
 reg [14:0] previous;
 reg [10:0] row_sum;reg row_good;
 wire [13:0] total=(y[2:0]==0?14'd0:previous[13:0])+row_sum+gray;
 wire cell_good=row_good&&valid&&(y[2:0]==0||previous[14]);
 wire [13:0] rounded=total+14'd32;
 reg [7:0] mean;reg mean_valid;
 reg [7:0] mean_x,mean_y;
 always @(posedge clk)begin
  if(de&&in_range&&x[2:0]==0)previous<=partials[column];
  if(rst_n&&de&&in_range&&x[2:0]==7)partials[column]<={cell_good,total};
  if(!rst_n)begin row_sum<=0;row_good<=0;mean<=0;mean_valid<=0;mean_x<=0;mean_y<=0;end
  else begin
   mean_valid<=de&&in_range&&x[2:0]==7&&y[2:0]==7&&cell_good;
   if(de&&in_range)begin
    if(x[2:0]==0)begin row_sum<=gray;row_good<=valid;end
    else if(x[2:0]!=7)begin row_sum<=row_sum+gray;row_good<=row_good&&valid;end
   end
   if(de&&in_range&&x[2:0]==7&&y[2:0]==7)begin mean<=rounded[13:6];mean_x<=x>>3;mean_y<=y>>3;end
  end
 end
 // Direct row-major stream counter avoids a dynamic row stride multiplier.
 wire take_mean=active&&mean_valid&&!cancel;
 wire [AB-1:0] address=sample_count[AB-1:0];
 (* ram_style="bram" *) reg [7:0] history[0:N-1];
 reg [7:0] old_mean,pending_mean,pending_x,pending_y;
 reg pending,pending_compare;
 always @(posedge clk)begin
  if(take_mean&&sample_count<N)old_mean<=history[address];
  if(rst_n&&take_mean&&sample_count<N)history[address]<=mean;
  if(!rst_n||cancel||first)begin pending<=0;pending_compare<=0;pending_mean<=0;pending_x<=0;pending_y<=0;end
  else begin
   pending<=take_mean&&sample_count<N;
   if(take_mean)begin pending_mean<=mean;pending_x<=mean_x;pending_y<=mean_y;pending_compare<=compare_history;end
  end
 end
 wire [7:0] delta=pending_mean>=old_mean?pending_mean-old_mean:old_mean-pending_mean;
 wire moved=pending_compare&&delta>=frame_threshold;
 reg [COLS-1:0] row1,row2;
 reg top2,top1,mid2,mid1,bot2,bot1;
 wire top_now=row2[COLS-1],mid_now=row1[COLS-1];
 wire [3:0] votes={3'd0,top2}+{3'd0,top1}+{3'd0,top_now}+
  {3'd0,mid2}+{3'd0,mid1}+{3'd0,mid_now}+{3'd0,bot2}+{3'd0,bot1}+{3'd0,moved};
 wire keep_motion=pending_compare&&pending_x>=2&&pending_y>=2&&mid1&&votes>=5;
 wire [15:0] cell_x0=({8'd0,pending_x}-16'd1)<<3;
 wire [15:0] cell_y0=({8'd0,pending_y}-16'd1)<<3;
 reg [15:0] raw_count,filtered_count,work_x0,work_x1,work_y0,work_y1;
 always @(posedge clk)begin
  if(!rst_n||cancel||first)begin
   top2<=0;top1<=0;mid2<=0;mid1<=0;bot2<=0;bot1<=0;
   raw_count<=0;filtered_count<=0;work_x0<=16'hffff;work_x1<=0;work_y0<=16'hffff;work_y1<=0;
  end else if(pending)begin
   // No reset on row history: only y>=2 can consume it after complete warmup.
   row2<={row2[COLS-2:0],mid_now};row1<={row1[COLS-2:0],moved};
   if(pending_x==0)begin top2<=0;mid2<=0;bot2<=0;end
   else begin top2<=top1;mid2<=mid1;bot2<=bot1;end
   top1<=top_now;mid1<=mid_now;bot1<=moved;
   if(moved)raw_count<=raw_count+1'b1;
   if(keep_motion)begin
    filtered_count<=filtered_count+1'b1;
    if(cell_x0<work_x0)work_x0<=cell_x0;if(cell_x0+7>work_x1)work_x1<=cell_x0+7;
    if(cell_y0<work_y0)work_y0<=cell_y0;if(cell_y0+7>work_y1)work_y1<=cell_y0+7;
   end
  end
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   active<=0;bad<=0;history_good<=0;compare_history<=0;seen<=0;last_id<=0;working_id<=0;
   pixel_count<=0;expected_x<=0;expected_y<=0;sample_count<=0;frame_threshold<=24;expected_cell_x<=0;expected_cell_y<=0;
   result_commit<=0;result_good<=0;motion_found<=0;lighting_change<=0;result_frame_id<=0;
   motion_count<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;
  end else begin
   result_commit<=0;
   if(cancel)begin
    active<=0;history_good<=0;compare_history<=0;seen<=0;
    result_good<=0;motion_found<=0;lighting_change<=0;motion_count<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;
   end else if(first)begin
    active<=!seen||source_frame_id!=last_id;
    compare_history<=history_good&&!active;
    if(active)history_good<=0;
    working_id<=source_frame_id;last_id<=source_frame_id;seen<=1;
    pixel_count<=1;expected_x<=1;expected_y<=0;sample_count<=0;bad<=!valid;frame_threshold<=threshold;expected_cell_x<=0;expected_cell_y<=0;
   end else if(active)begin
    if(in_range)begin
     if(!de||!valid||x!=expected_x||y!=expected_y||source_frame_id!=working_id)bad<=1;
     pixel_count<=pixel_count+1'b1;
     if(expected_x==WIDTH-1)begin expected_x<=0;expected_y<=expected_y+1'b1;end
     else expected_x<=expected_x+1'b1;
    end
    if(take_mean)begin
     if(sample_count>=N||mean_x!=expected_cell_x||mean_y!=expected_cell_y)bad<=1;
     sample_count<=sample_count+1'b1;
     if(expected_cell_x==COLS-1)begin expected_cell_x<=0;expected_cell_y<=expected_cell_y+1'b1;end
     else expected_cell_x<=expected_cell_x+1'b1;
    end
    if(finish)begin
     active<=0;result_commit<=1;result_frame_id<=working_id;
     history_good<=!bad&&pixel_count==WIDTH*HEIGHT&&sample_count==N;
     result_good<=!bad&&pixel_count==WIDTH*HEIGHT&&sample_count==N&&compare_history&&raw_count<=N*3/4;
     lighting_change<=!bad&&compare_history&&raw_count>N*3/4;
     if(!bad&&pixel_count==WIDTH*HEIGHT&&sample_count==N&&compare_history&&raw_count<=N*3/4)begin
      motion_count<=filtered_count;motion_found<=filtered_count>=MIN_CELLS;
      min_x<=filtered_count!=0?work_x0:16'd0;max_x<=filtered_count!=0?work_x1:16'd0;
      min_y<=filtered_count!=0?work_y0:16'd0;max_y<=filtered_count!=0?work_y1:16'd0;
     end else begin motion_count<=0;motion_found<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;end
    end
   end
  end
 end
 // synthesis translate_off
 initial if(WIDTH!=1280||HEIGHT!=720)$fatal(1,"Only native1280x720 motion geometry reviewed");
 // synthesis translate_on
endmodule
