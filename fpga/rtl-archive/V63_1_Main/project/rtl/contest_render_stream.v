`timescale 1ns/1ps
// One registered pixel stage. Source identity and effect mode are atomic per
// rendered frame. Wire frame IDs count distinct rendered source identities;
// they are NOT exposure timestamps or synchronized stereo frame numbers.
// Session high bit retains A's actual-camera ACK. Low2 bits identify board1;
// bits15:2 are a bounded local view epoch; upper15 bits are protocol tag4e56.
module contest_render_stream(
 input wire clk,rst_n,input wire [15:0] x,y,input wire [23:0] rgb,
 input wire de,valid,source_valid,source_stale,camera_remote,
 input wire [32:0] source_epoch,input wire [31:0] source_frame_id,
 input wire [2:0] mode,
 output reg [15:0] out_x,out_y,output reg [23:0] out_rgb,output reg out_de,
 output reg [31:0] render_frame_id,render_session,
 output reg [31:0] bad_frames
);
 wire first=de&&x==0&&y==0;
 reg active,bad,seen;
 reg [32:0] saved_epoch,working_epoch;
 reg [31:0] saved_frame,working_frame;
 reg [2:0] saved_mode,working_mode;
 reg working_camera;
 reg [13:0] view_epoch;
 reg [15:0] expected_x,expected_y;
 reg [19:0] pixels;
 wire identity_change=!seen||source_epoch!=saved_epoch||mode!=saved_mode;
 wire different=identity_change||source_frame_id!=saved_frame;
 wire [13:0] next_epoch=view_epoch+(identity_change?14'd1:14'd0);
 wire usable=source_valid&&!source_stale;
 wire mismatch=source_epoch!=working_epoch||source_frame_id!=working_frame||
               mode!=working_mode||camera_remote!=working_camera;
 always @(posedge clk)begin
  if(!rst_n)begin
   out_x<=0;out_y<=0;out_rgb<=0;out_de<=0;render_frame_id<=0;
   render_session<=32'h4e563101;view_epoch<=14'h0c40;bad_frames<=0;
   active<=0;bad<=0;seen<=0;saved_epoch<=0;saved_frame<=0;saved_mode<=0;
   working_epoch<=0;working_frame<=0;working_mode<=0;working_camera<=0;
   expected_x<=0;expected_y<=0;pixels<=0;
  end else begin
   out_x<=x;out_y<=y;out_rgb<=rgb;out_de<=0;
   if(first)begin
    if(active&&(bad||pixels!=921600))bad_frames<=bad_frames+1'b1;
    active<=usable&&valid;bad<=!usable||!valid;
    working_epoch<=source_epoch;working_frame<=source_frame_id;
    working_mode<=mode;working_camera<=camera_remote;
    expected_x<=1;expected_y<=0;pixels<=usable&&valid?1:0;
    out_de<=usable&&valid;
    if(usable&&valid)begin
     if(different)render_frame_id<=render_frame_id+1'b1;
     view_epoch<=next_epoch;render_session<={camera_remote,15'h4e56,next_epoch,2'b01};
     saved_epoch<=source_epoch;saved_frame<=source_frame_id;saved_mode<=mode;seen<=1;
    end
   end else if(de&&active)begin
    if(!usable||!valid||mismatch||x!=expected_x||y!=expected_y)bad<=1;
    else if(!bad)begin
     out_de<=1;pixels<=pixels+1'b1;
     if(x==1279)begin expected_x<=0;expected_y<=y+1'b1;end
     else expected_x<=x+1'b1;
    end
   end
   if(x==0&&y==720)begin
    if(active&&(bad||pixels!=921600))bad_frames<=bad_frames+1'b1;
    active<=0;
   end
  end
 end
endmodule
