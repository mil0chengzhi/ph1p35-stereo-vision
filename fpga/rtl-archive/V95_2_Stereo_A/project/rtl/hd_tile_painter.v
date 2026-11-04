`timescale 1ns/1ps
// RGB565 double-buffered1280x720 view, full16-bit Z occlusion in DDR.
// Direct-mapped8x8 tiles have a bounded write-back cache. Untouched Z tiles
// are initialized locally; a per-render touched bitmap prevents stale depth.
// Flush drains every dirty tile and verifies the last accepted DDR write.
module hd_tile_painter #(
 parameter COLOUR_BASE=20971520,COLOUR_STRIDE=1048576,Z_BASE=23068672,CACHE_BITS=7,TIMEOUT=200000
)(
 input wire clk,rst_n,frame_begin,frame_bank,output wire frame_ready,frame_clear_done,
 input wire draw_valid,output wire draw_ready,input wire[10:0]draw_x,input wire[9:0]draw_y,
 input wire[15:0]draw_rgb,draw_z,input wire frame_flush,output wire frame_flush_done,
 output reg fault,output reg[31:0]cache_hits,cache_misses,
 output wire[24:0]awaddr,araddr,output wire[127:0]wdata,output wire[15:0]wstrb,
 output wire awvalid,wvalid,wlast,arvalid,rready,read_active,
 input wire awready,wready,arready,input wire[127:0]rdata,input wire rvalid,rlast
);
 localparam ENTRIES=1<<CACHE_BITS,PIXELS=ENTRIES*64,CB=CACHE_BITS+6,CLEAR_ENTRIES=ENTRIES>14400?ENTRIES:14400,CLEAR_BITS=$clog2(CLEAR_ENTRIES);
 localparam IDLE=0,CLEAR=1,VALID_CLEAR=2,RUN=3,LOOKUP=4,CHECK=5,INIT_TILE=6,
 PACK_READ=7,PACK_GATHER=8,WRITE_START=9,WRITE_WAIT=10,LOAD_START=11,LOAD_AR=12,LOAD_DATA=13,LOAD_COPY=14,
 FLUSH_FIND=15,FENCE_START=16,FENCE_AR=17,FENCE_DATA=18,DONE=19,FAILED=20,FLUSH_CHECK=21;
 reg[4:0]state;reg bank,cleared,flushing;
 reg[15:0]colour_cache[0:PIXELS-1],z_cache[0:PIXELS-1];reg[15:0]colour_q,z_q;
 (* ram_style="bram" *)reg[16:0]tags[0:ENTRIES-1];reg[16:0]entry_q;
 wire[14:0]tag_q=entry_q[14:0];wire entry_valid=entry_q[16],entry_dirty=entry_q[15];
 reg touched[0:14399];reg touched_q,wanted_touched;
 reg[CLEAR_BITS-1:0]valid_clear_index;reg[16:0]clear_beat;
 reg[CACHE_BITS-1:0]slot;reg[CACHE_BITS:0]flush_index;
 reg[14:0]wanted_tile,operation_tile;reg[5:0]pixel_offset,init_offset;
 reg[15:0]saved_rgb,saved_z;reg[2:0]row,col;reg plane;
 reg[127:0]packed_word,load_word;reg[CB-1:0]ram_read_address,ram_write_address;
 reg[24:0]write_address,read_address;reg last_write_clear;
 wire[127:0]write_data=last_write_clear?{8{16'h0842}}:packed_word;
 // PACK_GATHER never changes an offered write; the final packed word also
 // remains held through its DDR fence. Empty renders fence the clear word.
 reg write_live,aw_sent,w_sent,read_live,read_pending;reg[(TIMEOUT>1?$clog2(TIMEOUT):1)-1:0]age;
 wire aw_take=awvalid&&awready,w_take=wvalid&&wready,ar_take=arvalid&&arready;
 wire write_complete=write_live&&(aw_sent||aw_take)&&(w_sent||w_take);
 wire read_take=rvalid&&rready;
 // One write port is essential: two state-exclusive writes at different
 // addresses otherwise become a dual-write RAM and can expand into FFs.
 wire touched_write=(state==VALID_CLEAR&&(ENTRIES<=14400||valid_clear_index<14400))||(state==WRITE_WAIT&&write_complete&&row==7&&plane);
 wire[13:0]operation_index=({7'd0,operation_tile[14:8]}<<7)+({7'd0,operation_tile[14:8]}<<5)+operation_tile[7:0];
 wire[13:0]touched_write_address=state==VALID_CLEAR?valid_clear_index:operation_index;
 wire[13:0]wanted_index=({7'd0,wanted_tile[14:8]}<<7)+({7'd0,wanted_tile[14:8]}<<5)+wanted_tile[7:0];
 wire[13:0]incoming_index=({7'd0,draw_y[9:3]}<<7)+({7'd0,draw_y[9:3]}<<5)+draw_x[10:3];
 wire[24:0]tile_row={15'd0,operation_tile[14:8],row};
 wire[24:0]tile_offset=(tile_row<<10)+(tile_row<<8)+{14'd0,operation_tile[7:0],3'b0};
 wire[24:0]tile_address=(plane?Z_BASE:COLOUR_BASE+(bank?COLOUR_STRIDE:0))+tile_offset;
 assign frame_ready=!fault&&(state==IDLE||state==DONE);
 assign frame_clear_done=cleared&&!fault;
 assign draw_ready=state==RUN&&!fault&&!frame_flush;
 assign frame_flush_done=state==DONE&&!fault;
 assign awaddr=write_address;assign wdata=write_data;assign wstrb=16'd0;assign wlast=1'b1;
 assign awvalid=rst_n&&write_live&&!aw_sent;assign wvalid=rst_n&&write_live&&!w_sent;
 assign araddr=read_address;assign arvalid=rst_n&&read_live;
 assign rready=rst_n&&(read_pending||fault);assign read_active=read_live||read_pending;
 always @*begin
  ram_read_address={slot,pixel_offset};
  if(state==PACK_READ)ram_read_address={slot,row,col};
  ram_write_address={slot,pixel_offset};
  if(state==INIT_TILE)ram_write_address={slot,init_offset};
  if(state==LOAD_COPY)ram_write_address={slot,row,col};
 end
 // One tag read port and one state-multiplexed write port. No RAM reset.
 wire[CACHE_BITS-1:0]tag_read_address=state==FLUSH_FIND?flush_index[CACHE_BITS-1:0]:slot;
 // Frame metadata clear runs with the existing touched-map clear, before
 // draw_ready can rise. One synchronous tag port supplies tag/valid/dirty.
 wire tag_install=(state==INIT_TILE&&init_offset==63)||(state==LOAD_COPY&&col==7&&row==7&&plane);
 wire tag_mark_dirty=state==CHECK&&entry_valid&&tag_q==wanted_tile&&saved_z<=z_q;
 wire tag_mark_clean=state==WRITE_WAIT&&write_complete&&row==7&&plane;
 wire tag_clear=state==VALID_CLEAR;
 wire tag_write=!fault&&(tag_clear||tag_install||tag_mark_dirty||tag_mark_clean);
 wire[CACHE_BITS-1:0]tag_write_address=tag_clear?valid_clear_index[CACHE_BITS-1:0]:slot;
 wire[16:0]tag_write_data=tag_clear?17'd0:tag_install?{1'b1,1'b0,wanted_tile}:
  tag_mark_dirty?{1'b1,1'b1,tag_q}:{1'b1,1'b0,operation_tile};
 always @(posedge clk)begin
  if(state==LOOKUP||state==FLUSH_FIND)entry_q<=tags[tag_read_address];
  if(tag_write)tags[tag_write_address]<=tag_write_data;
 end
 always @(posedge clk)begin
  if(state==LOOKUP||state==PACK_READ)begin colour_q<=colour_cache[ram_read_address];z_q<=z_cache[ram_read_address];end
  if(state==LOOKUP)touched_q<=touched[wanted_index];
  if(!fault)begin
   if(state==INIT_TILE)begin colour_cache[ram_write_address]<=16'h0842;z_cache[ram_write_address]<=16'hffff;end
   if(state==LOAD_COPY)begin if(plane)z_cache[ram_write_address]<=load_word[col*16+:16];else colour_cache[ram_write_address]<=load_word[col*16+:16];end
   if(state==CHECK&&entry_valid&&tag_q==wanted_tile&&saved_z<=z_q)begin colour_cache[ram_write_address]<=saved_rgb;z_cache[ram_write_address]<=saved_z;end
   if(touched_write)touched[touched_write_address]<=state!=VALID_CLEAR;
  end
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;bank<=0;cleared<=0;flushing<=0;valid_clear_index<=0;clear_beat<=0;slot<=0;flush_index<=0;
   wanted_tile<=0;operation_tile<=0;pixel_offset<=0;init_offset<=0;saved_rgb<=0;saved_z<=0;wanted_touched<=0;
   row<=0;col<=0;plane<=0;packed_word<=0;load_word<=0;write_address<=0;read_address<=0;last_write_clear<=1;
   write_live<=0;aw_sent<=0;w_sent<=0;read_live<=0;read_pending<=0;age<=0;fault<=0;cache_hits<=0;cache_misses<=0;
  end else begin
   if(aw_take)aw_sent<=1;if(w_take)w_sent<=1;
   if(write_complete)begin write_live<=0;aw_sent<=0;w_sent<=0;end
   if(ar_take)begin read_live<=0;read_pending<=1;end
   if(read_take)begin if(!read_pending||!rlast)fault<=1;else read_pending<=0;end
   if(write_live||read_live||read_pending)begin age<=age+1'b1;if(age==TIMEOUT-1)fault<=1;end else age<=0;
   if(aw_take||w_take||ar_take||read_take)age<=0;
   if(!fault)case(state)
    IDLE,DONE:if(frame_begin&&frame_ready)begin bank<=frame_bank;cleared<=0;clear_beat<=0;flushing<=0;state<=CLEAR;cache_hits<=0;cache_misses<=0;end
    CLEAR:begin
     if(!write_live)begin write_address<=COLOUR_BASE+(bank?COLOUR_STRIDE:0)+{clear_beat,3'b0};last_write_clear<=1;aw_sent<=0;w_sent<=0;write_live<=1;end
     if(write_complete)begin if(clear_beat==115199)begin valid_clear_index<=0;state<=VALID_CLEAR;end else clear_beat<=clear_beat+1'b1;end
    end
    VALID_CLEAR:if(valid_clear_index==CLEAR_ENTRIES-1)begin cleared<=1;state<=RUN;end else valid_clear_index<=valid_clear_index+1'b1;
    RUN:if(frame_flush)begin flushing<=1;flush_index<=0;state<=FLUSH_FIND;end
     else if(draw_valid&&draw_ready)begin
      if(draw_x>=1280||draw_y>=720||draw_z==0||draw_z==65535)fault<=1;
      else begin slot<=incoming_index[CACHE_BITS-1:0];wanted_tile<={draw_y[9:3],draw_x[10:3]};pixel_offset<={draw_y[2:0],draw_x[2:0]};saved_rgb<=draw_rgb;saved_z<=draw_z;state<=LOOKUP;end
     end
    LOOKUP:state<=CHECK;
    CHECK:if(entry_valid&&tag_q==wanted_tile)begin
      cache_hits<=cache_hits+1'b1;state<=RUN;
     end else begin
      cache_misses<=cache_misses+1'b1;wanted_touched<=touched_q;
      if(entry_valid&&entry_dirty)begin operation_tile<=tag_q;row<=0;col<=0;plane<=0;state<=PACK_READ;end
      else if(touched_q)begin operation_tile<=wanted_tile;row<=0;col<=0;plane<=0;state<=LOAD_START;end
      else begin init_offset<=0;state<=INIT_TILE;end
     end
    INIT_TILE:if(init_offset==63)begin state<=LOOKUP;end else init_offset<=init_offset+1'b1;
    PACK_READ:state<=PACK_GATHER;
    PACK_GATHER:begin packed_word[col*16+:16]<=plane?z_q:colour_q;if(col==7)state<=WRITE_START;else begin col<=col+1'b1;state<=PACK_READ;end end
    WRITE_START:begin write_address<=tile_address;last_write_clear<=0;write_live<=1;aw_sent<=0;w_sent<=0;state<=WRITE_WAIT;end
    WRITE_WAIT:if(write_complete)begin
     if(row!=7)begin row<=row+1'b1;col<=0;state<=PACK_READ;end
     else if(!plane)begin plane<=1;row<=0;col<=0;state<=PACK_READ;end
     else begin
      
      if(flushing)begin flush_index<=flush_index+1'b1;state<=FLUSH_FIND;end
      else if(wanted_touched)begin operation_tile<=wanted_tile;row<=0;col<=0;plane<=0;state<=LOAD_START;end
      else begin init_offset<=0;state<=INIT_TILE;end
     end
    end
    LOAD_START:begin read_address<=tile_address;read_live<=1;state<=LOAD_AR;end
    LOAD_AR:if(ar_take)state<=LOAD_DATA;
    LOAD_DATA:if(read_take&&rlast)begin load_word<=rdata;col<=0;state<=LOAD_COPY;end
    LOAD_COPY:if(col!=7)col<=col+1'b1;
     else if(row!=7)begin row<=row+1'b1;state<=LOAD_START;end
     else if(!plane)begin plane<=1;row<=0;state<=LOAD_START;end
     else begin state<=LOOKUP;end
    FLUSH_FIND:if(flush_index==ENTRIES)state<=FENCE_START;
     else begin slot<=flush_index[CACHE_BITS-1:0];state<=FLUSH_CHECK;end
    FLUSH_CHECK:if(entry_valid&&entry_dirty)begin operation_tile<=tag_q;row<=0;col<=0;plane<=0;state<=PACK_READ;end
     else begin flush_index<=flush_index+1'b1;state<=FLUSH_FIND;end
    FENCE_START:begin read_address<=write_address;read_live<=1;state<=FENCE_AR;end
    FENCE_AR:if(ar_take)state<=FENCE_DATA;
    FENCE_DATA:if(read_take&&rlast)begin if(rdata!==write_data)fault<=1;else state<=DONE;end
    FAILED:begin end
    default:begin fault<=1;state<=FAILED;end
   endcase
  end
 end
endmodule
