`timescale 1ns/1ps
// ISOLATED EXPERIMENT. Completion is sixteen source writes after capture.
// No source/host data-path/clock/reset or hardware qualification is inherited.
module snapshot_pdp_candidate(
 input wire source_clk,host_clk,rst_n,take,
 input wire [255:0] local_live,remote_live,
 input wire read_peer,input wire [4:0] read_index,
 output wire [7:0] read_byte,output reg led,
 output wire busy,output wire request_state,source_ack_state
);
 reg request,ack,seen;
 (* keep=1,preserve=1 *)reg[1:0]request_sync,ack_sync;
 reg active,token;
 reg[3:0]write_index;
 reg[451:0]held_payload;
 reg held_led,source_led,initialized,read_valid;
 reg[1:0]read_lane;
 wire capture=rst_n&&!active&&(request_sync[1]!=ack);
 wire[451:0]packed_live={remote_live[255:128],remote_live[97:0],local_live[255:128],local_live[97:0]};
 // Data storage needs no reset: no write can occur before a complete capture.
 // All452 effective input bits and the LED are sampled on the SAME source edge.
 always @(posedge source_clk)if(capture)begin held_payload<=packed_live;held_led<=local_live[96];end
 wire[225:0]held_local=held_payload[225:0],held_remote=held_payload[451:226];
 wire[255:0]local_words={held_local[225:98],30'd0,held_local[97:0]};
 wire[255:0]remote_words={held_remote[225:98],30'd0,held_remote[97:0]};
 wire[31:0]write_word=write_index[3]?remote_words[{write_index[2:0],5'b00000}+:32]:local_words[{write_index[2:0],5'b00000}+:32];
 wire write_enable=rst_n&&active;
 always @(posedge source_clk or negedge rst_n)begin
  if(!rst_n)begin request_sync<=0;active<=0;token<=0;write_index<=0;ack<=0;source_led<=0;end
  else begin
   request_sync<={request_sync[0],request};
   if(capture)begin active<=1;token<=request_sync[1];write_index<=0;end
   else if(active)begin
    if(write_index==15)begin
     // This edge performs the last RAM write. Only now may ACK/LED change.
     active<=0;ack<=token;source_led<=held_led;
    end else write_index<=write_index+1'b1;
   end
  end
 end
 always @(posedge host_clk or negedge rst_n)begin
  // Explicit experimental reset contract: both domains asynchronously
  // invalidate ownership/validity. A short global reset cannot publish a
  // partially written bank when the source clock is stopped.
  if(!rst_n)begin request<=0;ack_sync<=0;seen<=0;initialized<=0;read_valid<=0;read_lane<=0;led<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   // A high take is accepted only at an idle host edge. Busy takes are lost,
   // not queued. A sustained take may request a new job after completion.
   if(take&&request==ack_sync[1])request<=!request;
   if(ack_sync[1]!=seen)begin seen<=ack_sync[1];initialized<=1;led<=source_led;end
   // RAM B and these metadata FFs sample OLD seen/initialized on this edge.
   read_valid<=initialized;read_lane<=read_index[1:0];
  end
 end
 wire[39:0]read_word;
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(40),.DATA_WIDTH_B(40),.ADDR_WIDTH_A(9),.ADDR_WIDTH_B(9),
  .DATA_DEPTH_A(512),.DATA_DEPTH_B(512),.BYTE_ENABLE(0),.MODE("PDPW"),
  .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
  .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),.OREGSET_A("RESET"),.OREGSET_B("RESET"),
  .SSROVERCE("ENABLE"),.CLKMODE("ASYNC"),.ASYNC_RESET_RELEASE_A("SYNC"),.ASYNC_RESET_RELEASE_B("SYNC"),
  .FILL_ALL("00000000000000000000")) ram(
  .clka(source_clk),.dia({8'd0,write_word}),.addra({4'd0,token,write_index}),.cea(write_enable),.ocea(1'b1),.wea(write_enable),.rsta(1'b0),.bea(1'b1),.doa(),
  .clkb(host_clk),.dib(40'd0),.addrb({4'd0,seen,read_peer,read_index[4:2]}),.ceb(1'b1),.oceb(1'b1),.web(1'b0),.rstb(1'b0),.beb(1'b1),.dob(read_word),
  .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr());
 assign read_byte=read_valid?read_word[{read_lane,3'b000}+:8]:8'd0;
 assign busy=request!=ack_sync[1];
 assign request_state=request;
 assign source_ack_state=ack;
endmodule
