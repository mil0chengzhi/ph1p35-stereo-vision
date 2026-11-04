`timescale 1ns/1ps
// Registered Gray milliseconds, continuous across view changes. DDR records a
// local START timestamp in the same timebase. This measures local buffering,
// excluding sensor exposure/MIPI, the upstream peer and panel/USB capture delay.
module contest_performance #(parameter CLK_HZ=74000000)(
 input wire clk,rst_n,first_pixel,frame_valid,input wire [32:0] source_epoch,
 input wire [31:0] frame_id,frame_start_ms,
 output reg [31:0] time_gray,
 output reg [15:0] fps,latency_ms,output reg fps_valid,latency_valid
);
 localparam MS_CYCLES=CLK_HZ/1000;
 reg [$clog2(MS_CYCLES+1)-1:0] divider;
 reg [31:0] now_ms,last_arrival,last_id;reg [32:0] last_epoch;
 reg [9:0] window_ms;reg [15:0] count;reg seen;
 wire tick=divider==MS_CYCLES-1;
 wire [31:0] next_ms=now_ms+1'b1,age=now_ms-frame_start_ms;
 wire epoch_change=seen&&source_epoch!=last_epoch;
 wire new_frame=first_pixel&&frame_valid&&(!seen||epoch_change||frame_id!=last_id);
 wire stale=seen&&now_ms-last_arrival>=1000;
 wire [15:0] next_count=new_frame&&count<9999?count+1'b1:count;
 always @(posedge clk)begin
  if(!rst_n)begin divider<=0;now_ms<=0;time_gray<=0;window_ms<=0;count<=0;
   fps<=0;latency_ms<=0;fps_valid<=0;latency_valid<=0;seen<=0;last_epoch<=0;last_id<=0;last_arrival<=0;end
  else begin
   if(tick)begin
    divider<=0;now_ms<=next_ms;time_gray<=next_ms^(next_ms>>1);
    if(window_ms==999)begin window_ms<=0;fps<=next_count;fps_valid<=1;count<=0;end
    else begin window_ms<=window_ms+1'b1;count<=next_count;end
   end else begin divider<=divider+1'b1;count<=next_count;end
   if(stale&&!new_frame)begin fps<=0;latency_valid<=0;end
   if(first_pixel&&epoch_change)begin
    // Start a fresh statistics window; the Gray clock never jumps on a view change.
    window_ms<=0;count<=frame_valid?1:0;fps<=0;fps_valid<=0;latency_valid<=0;seen<=0;
   end
   if(new_frame)begin
    seen<=1;last_epoch<=source_epoch;last_id<=frame_id;last_arrival<=now_ms;
    latency_ms<=age<1000?age[15:0]:16'd0;latency_valid<=age<1000;
   end
  end
 end
endmodule
