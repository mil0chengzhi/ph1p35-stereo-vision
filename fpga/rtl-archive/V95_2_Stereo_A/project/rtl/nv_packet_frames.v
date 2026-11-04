`timescale 1ns/1ps
// Two private packet slots. Speculative MAC words never reach DDR until FCS
// and the complete header/length have passed. Ordered complete frames only.
// token START: [63:32] session, [31:0] frame; END: [64:33] session,
// [32:1] frame, [0] good. DATA holds eight little-endian RGB565 pixels.
module nv_packet_frames #(
 parameter PACKET_PIXELS=640, PACKETS_PER_FRAME=1440
)(input wire clk,rst_n,
 output wire packet_accept,input wire packet_start,packet_word_write,
 input wire [9:0] packet_word_addr,input wire [15:0] packet_word_data,
 input wire packet_done,packet_good,input wire [31:0] packet_session,packet_frame,
 input wire [15:0] packet_index,
 input wire fifo_full,output wire fifo_write,output wire [129:0] fifo_data,
 output reg [31:0] completed_frames,dropped_frames,queue_losses,
 output reg [31:0] last_session,last_frame,
 output wire [7:0] state_flags
);
 localparam IDLE=0,START=1,READ=2,COLLECT=3,DATA=4,END=5,RELEASE=6;
 localparam SLOT_BITS=$clog2(PACKET_PIXELS);
 (* ram_style="bram" *) reg [15:0] memory[0:(2<<SLOT_BITS)-1];
 reg [1:0] occupied,complete,slot_good;
 reg write_bank,read_bank,rx_owned,rx_bank,rx_bad;
 reg [31:0] sessions[0:1],frames[0:1];reg [15:0] indices[0:1];
 reg [3:0] state;reg active,bad;
 reg [31:0] active_session,active_frame;
 reg [15:0] expected_index;
 reg [9:0] offset;reg [2:0] lane;
 reg [15:0] ram_data;reg [127:0] assembled;
 // One synchronous, unreset RAM read port permits ERAM inference. The first
 // READ fills ram_data; COLLECT prefetches the next word without a new cycle.
 // Slot ownership and CRC acceptance, not stale RAM contents, control use.
 wire ram_read_enable=rst_n && (state==READ || (state==COLLECT && lane!=7));
 wire [9:0] ram_read_offset=state==COLLECT?offset+10'd1:offset;
 always @(posedge clk)
  if(ram_read_enable) ram_data<=memory[{read_bank,ram_read_offset[SLOT_BITS-1:0]}];
 wire loss_event=packet_done && (!rx_owned || !packet_good || rx_bad);
 wire end_good=active && !bad && !loss_event;
 assign packet_accept=!occupied[write_bank];
 assign fifo_write=!fifo_full && (state==START || state==DATA || state==END);
 assign fifo_data=state==START?{2'b01,64'b0,active_session,active_frame}:
                  state==END?{2'b10,63'b0,active_session,active_frame,end_good}:
                  {2'b00,assembled};
 assign state_flags={rx_owned,occupied,active,bad,state[2:0]};
 always @(posedge clk) begin
  if(!rst_n) begin
   occupied<=0;complete<=0;slot_good<=0;write_bank<=0;read_bank<=0;rx_owned<=0;rx_bank<=0;rx_bad<=0;
   state<=IDLE;active<=0;bad<=0;active_session<=0;active_frame<=0;expected_index<=0;
   offset<=0;lane<=0;assembled<=0;
   completed_frames<=0;dropped_frames<=0;queue_losses<=0;last_session<=0;last_frame<=0;
   sessions[0]<=0;sessions[1]<=0;frames[0]<=0;frames[1]<=0;indices[0]<=0;indices[1]<=0;
  end else begin
   if(packet_start) begin
    if(packet_accept) begin occupied[write_bank]<=1;complete[write_bank]<=0;
     rx_owned<=1;rx_bank<=write_bank;write_bank<=!write_bank;rx_bad<=0;end
    else begin rx_owned<=0;rx_bad<=1;end
   end
   if(packet_word_write && rx_owned) begin
    if(packet_word_addr<PACKET_PIXELS) memory[{rx_bank,packet_word_addr[SLOT_BITS-1:0]}]<=packet_word_data;
    else rx_bad<=1;
   end
   if(packet_done) begin
    if(rx_owned) begin
     complete[rx_bank]<=1;slot_good[rx_bank]<=packet_good && !rx_bad;
     sessions[rx_bank]<=packet_session;frames[rx_bank]<=packet_frame;indices[rx_bank]<=packet_index;
    end
    if(loss_event) begin queue_losses<=queue_losses+1'b1;if(active) bad<=1;end
    rx_owned<=0;
   end
   case(state)
    IDLE: if(complete[read_bank]) begin
     if(!slot_good[read_bank]) begin
      if(active) begin bad<=1;state<=END;end else state<=RELEASE;
     end else if(indices[read_bank]==0) begin
      // A new SOF is the only recovery point. START makes DDR reject any
      // unfinished predecessor; already displayed front is never overwritten.
      if(active) dropped_frames<=dropped_frames+1'b1;
      active<=1;bad<=loss_event;active_session<=sessions[read_bank];active_frame<=frames[read_bank];
      expected_index<=1;offset<=0;lane<=0;state<=START;
     end else if(active && !bad && !loss_event && indices[read_bank]==expected_index &&
                 sessions[read_bank]==active_session && frames[read_bank]==active_frame) begin
      expected_index<=expected_index+1'b1;offset<=0;lane<=0;state<=READ;
     end else begin
      if(active) begin bad<=1;state<=END;end else state<=RELEASE;
     end
    end
    START: if(!fifo_full) state<=READ;
    READ: state<=COLLECT;
    COLLECT: begin
     assembled[lane*16+:16]<=ram_data;
     if(lane==7) state<=DATA;
     else begin lane<=lane+1'b1;offset<=offset+1'b1;end
    end
    DATA: if(!fifo_full) begin
     if(offset==PACKET_PIXELS-1) begin
      if(indices[read_bank]==PACKETS_PER_FRAME-1)state<=END;
      else state<=RELEASE;
     end else begin offset<=offset+1'b1;lane<=0;state<=READ;end
    end
    END: if(!fifo_full) begin
     if(end_good) begin completed_frames<=completed_frames+1'b1;last_session<=active_session;last_frame<=active_frame;end
     else dropped_frames<=dropped_frames+1'b1;
     active<=0;bad<=0;state<=RELEASE;
    end
    RELEASE: begin occupied[read_bank]<=0;complete[read_bank]<=0;read_bank<=!read_bank;state<=IDLE;end
    default: begin state<=IDLE;active<=0;bad<=1;end
   endcase
  end
 end
 // synthesis translate_off
 initial if(PACKET_PIXELS<8 || PACKET_PIXELS>1024 || PACKET_PIXELS%8 || PACKETS_PER_FRAME<1)
  $fatal(1,"invalid packet/frame geometry");
 // synthesis translate_on
endmodule
