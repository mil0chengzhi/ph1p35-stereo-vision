`timescale 1ns/1ps
// Pre-OSD RGB565 metering. One result per complete, NEW source frame; repeated
// HDMI frames, missing pixels and mid-frame identity changes never drive AE.
// Sparse 16x16 sampling, but validity/order is checked across the entire ROI.
// Published fields change together and are transferred with cdc_status_snapshot.
module exposure_meter #(
 parameter SCREEN_HEIGHT=720,OFFSET_X=160,IMAGE_WIDTH=960,IMAGE_HEIGHT=720,SAMPLE_SHIFT=4
)(
 input wire clk,rst_n,input wire [15:0] x,y,input wire de,valid,
 input wire [15:0] rgb565,input wire [31:0] source_frame_id,input wire source_frame_valid,
 output reg result_valid,output reg [19:0] luminance_sum,
 output reg [11:0] sample_count,highlight_count,output reg [31:0] frame_id,
 output reg [15:0] result_sequence
);
 localparam LAST_X=OFFSET_X+IMAGE_WIDTH-1,LAST_Y=IMAGE_HEIGHT-1;
 localparam N=((IMAGE_WIDTH+(1<<SAMPLE_SHIFT)-1)>>SAMPLE_SHIFT)*((IMAGE_HEIGHT+(1<<SAMPLE_SHIFT)-1)>>SAMPLE_SHIFT);
 wire start=de&&x==0&&y==0;
 wire roi=x>=OFFSET_X&&x<=LAST_X&&y<IMAGE_HEIGHT;
 wire [15:0] rx=x-OFFSET_X;
 wire sample=roi&&(rx[SAMPLE_SHIFT-1:0]==0)&&(y[SAMPLE_SHIFT-1:0]==0);
 wire [7:0] r={rgb565[15:11],rgb565[15:13]},g={rgb565[10:5],rgb565[10:9]},b={rgb565[4:0],rgb565[4:2]};
 wire [9:0] gray={2'b0,r}+{1'b0,g,1'b0}+{2'b0,b}+10'd2;
 reg seen,active,bad,complete;reg [31:0] last_id,working_id;
 reg [15:0] expected_x,expected_y;reg [19:0] total;reg [11:0] count,highlights;
 wire fresh=source_frame_valid&&(!seen||source_frame_id!=last_id);
 wire taking=(start?fresh:active)&&source_frame_valid;
 always @(posedge clk) begin
  if(!rst_n) begin
   seen<=0;active<=0;bad<=0;complete<=0;last_id<=0;working_id<=0;
   expected_x<=OFFSET_X;expected_y<=0;total<=0;count<=0;highlights<=0;
   result_valid<=0;luminance_sum<=0;sample_count<=0;highlight_count<=0;frame_id<=0;result_sequence<=0;
  end else begin
   if(start) begin
    active<=fresh;bad<=0;complete<=0;working_id<=source_frame_id;
    expected_x<=OFFSET_X;expected_y<=0;total<=0;count<=0;highlights<=0;
    if(source_frame_valid) begin seen<=1;last_id<=source_frame_id;end
    if(active) result_valid<=0;
   end
   if(taking&&de&&roi) begin
    if(!valid||(!start&&(complete||x!=expected_x||y!=expected_y))) bad<=1;
    if(x==LAST_X) begin expected_x<=OFFSET_X;expected_y<=y+1'b1;if(y==LAST_Y)complete<=1;end
    else expected_x<=x+1'b1;
    if(sample) begin
     total<=(start?20'd0:total)+gray[9:2];count<=(start?12'd0:count)+1'b1;
     highlights<=(start?12'd0:highlights)+((r>=232||g>=232||b>=232)?12'd1:12'd0);
    end
   end
   if(active&&!de&&y>=SCREEN_HEIGHT) begin
    active<=0;result_valid<=!bad&&complete&&(count==N);
    if(!bad&&complete&&count==N) begin
     luminance_sum<=total;sample_count<=count;highlight_count<=highlights;
     frame_id<=working_id;result_sequence<=result_sequence+1'b1;
    end
   end
   if(!source_frame_valid||(active&&!start&&source_frame_id!=working_id)) begin
    active<=0;bad<=1;result_valid<=0;
   end
  end
 end
endmodule
