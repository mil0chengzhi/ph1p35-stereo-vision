`timescale 1ns/1ps
// Lab candidate only. Conservative radius-two opening of the raw green mask.
// Input is the320x180 four-pixel averaged grid, one cell_valid pulse per cell.
// Output corresponds to(current input x-3,y-3), matching the existing colour
// and grey front-end centre. No new green pixel can be invented by opening.
// One320x5 unreset row store; row/column gating prevents stale-frame data use.
module green_open_cross_grid #(
 parameter COLS=320,ROWS=180,AB=$clog2(COLS)
)(
 input wire clk,rst_n,frame_start,abort,
 input wire cell_valid,input wire [8:0] cell_x,input wire [7:0] cell_y,
 input wire raw_green,
 output reg filtered_valid,output reg filtered_green,
 output reg [8:0] centre_x,output reg [7:0] centre_y
);
 reg [4:0] row_memory[0:COLS-1];
 reg [4:0] row_q;
 reg pending,pending_green;
 reg [8:0] pending_x;
 reg [7:0] pending_y;
 reg [24:0] columns;
 reg previous_column_open;
 wire [4:0] new_column={row_q[3:0],pending_green};
 wire [24:0] window_now=pending_x==0?{20'd0,new_column}:{columns[19:0],new_column};
 function cross_full;
  input [24:0] w;input integer dx,dy;
  integer c,r;
  begin
   c=2-dx;r=2-dy;
   cross_full=w[c*5+r]&&w[(c-1)*5+r]&&w[(c+1)*5+r]&&w[c*5+r-1]&&w[c*5+r+1];
  end
 endfunction
 wire opened_now=cross_full(window_now,0,0)||cross_full(window_now,1,0)||
  cross_full(window_now,-1,0)||cross_full(window_now,0,1)||cross_full(window_now,0,-1);
 // The extra saved open bit supplies one previous row and previous column.
 // Its centre is therefore x-3,y-3 rather than the window's x-2,y-2.
 always @(posedge clk)begin
  if(cell_valid&&cell_x<COLS)row_q<=row_memory[cell_x[AB-1:0]];
  if(rst_n&&!frame_start&&!abort&&pending)
   row_memory[pending_x[AB-1:0]]<={(pending_y>=4&&pending_x>=4)?opened_now:1'b0,row_q[2:0],pending_green};
  if(!rst_n||frame_start||abort)begin
   pending<=0;pending_green<=0;pending_x<=0;pending_y<=0;columns<=0;
   previous_column_open<=0;filtered_valid<=0;filtered_green<=0;centre_x<=0;centre_y<=0;
  end else begin
   filtered_valid<=pending&&pending_x>=6&&pending_y>=6;
   filtered_green<=pending&&pending_x>=6&&pending_y>=6&&previous_column_open;
   if(pending)begin
    columns<=window_now;
    previous_column_open<=row_q[4];
    centre_x<=pending_x-9'd3;centre_y<=pending_y-8'd3;
   end
   pending<=cell_valid&&cell_x<COLS&&cell_y<ROWS;
   if(cell_valid)begin pending_x<=cell_x;pending_y<=cell_y;pending_green<=raw_green;end
  end
 end
endmodule
