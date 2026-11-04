`timescale 1ns/1ps
// Select only at HDMI blanking, from a complete previously displayed peer
// frame's trusted session. Static camera frame IDs time out after eight blanks.
// A's session control bit22 acknowledges its real source; B's bit22 requests it.
module stereo_view_control #(parameter BOARD_ID=1)(
 input wire clk,rst_n,commit,local_valid,peer_valid,
 input wire [31:0] local_frame,peer_frame,peer_session,
 output reg selected_remote,output wire local_stale,peer_stale
);
 reg [3:0] local_same,peer_same;reg [31:0] last_local_frame,last_peer_frame;
 assign local_stale=local_same>=8;assign peer_stale=peer_same>=8;
 always @(posedge clk)begin
  if(!rst_n)begin selected_remote<=0;local_same<=0;peer_same<=0;last_local_frame<=0;last_peer_frame<=0;end
  else if(commit)begin
   last_local_frame<=local_frame;last_peer_frame<=peer_frame;
   local_same<=local_valid&&local_frame!=last_local_frame?0:(local_same==15?15:local_same+1'b1);
   peer_same<=peer_valid&&peer_frame!=last_peer_frame?0:(peer_same==15?15:peer_same+1'b1);
   if(BOARD_ID==1)begin
    if(peer_valid&&!peer_stale)selected_remote<=peer_session[22];
    else if(peer_stale)selected_remote<=0;
   end else begin
    if(peer_valid&&!peer_stale)selected_remote<=!peer_session[22];
    else selected_remote<=0;
   end
  end
 end
endmodule
