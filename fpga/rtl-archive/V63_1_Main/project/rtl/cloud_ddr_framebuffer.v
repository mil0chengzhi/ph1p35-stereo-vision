`timescale 1ns/1ps
// Two RGB565 images in the unused DDR tail after the six existing video slots.
// Word addresses, active-low byte masks, immutable front/back ownership.
// A readback fence confirms the final accepted write before bank publication.
module cloud_ddr_framebuffer #(parameter BASE=12582912,STRIDE=32768,WIDTH=240,HEIGHT=135)(
 input wire clk,rst_n,input wire draw_valid,output wire draw_ready,
 input wire draw_clear,draw_bank,input wire[15:0]draw_address,draw_rgb,
 input wire flush_request,output reg flush_done,
 input wire front_bank_pixel,input wire[31:0]front_job_pixel,
 input wire line_request,output wire line_ready,input wire new_frame,
 input wire[15:0]line_row,input wire request_bank,
 output wire line_write,output wire[127:0]line_data,output wire[7:0]line_address,
 output wire line_bank,output reg line_done,line_good,
 output reg[31:0]line_frame,line_session,
 output wire[24:0]awaddr,araddr,output wire[127:0]wdata,
 output wire[15:0]wstrb,output wire awvalid,wvalid,wlast,arvalid,rready,read_active,
 input wire awready,wready,arready,input wire[127:0]rdata,input wire rvalid,rlast,
 output reg fault
);
 localparam ROW_WORDS=WIDTH/8,IDLE=0,COMMAND=1,RESPONSE=2;
 reg write_pending,aw_sent,w_sent;reg[24:0]write_address,last_address;
 reg[127:0]write_data;reg[15:0]write_mask,last_colour;
 reg[2:0]last_lane;reg[19:0]write_age,read_age;
 reg[1:0]state;reg is_fence,scan_bank,cache_bank;
 reg[5:0]beat;reg[24:0]row_base;reg[31:0]scan_job;
 reg[1:0]bank_sync;reg[31:0]job_sync0,job_sync1;
 wire needs_write=!draw_clear||draw_address[2:0]==7;
 assign draw_ready=!fault&&(!needs_write||(!write_pending&&state==IDLE&&!line_request&&!flush_request));
 wire take_draw=draw_valid&&draw_ready;
 assign awaddr=write_address;assign wdata=write_data;assign wstrb=write_mask;
 assign awvalid=write_pending&&!aw_sent&&!fault;
 assign wvalid=write_pending&&!w_sent&&!fault;assign wlast=1;
 assign line_ready=!fault&&state==IDLE&&!write_pending&&!flush_request;
 assign araddr=is_fence?last_address:row_base+{beat,3'd0};
 assign arvalid=state==COMMAND&&!fault;
 assign rready=state==RESPONSE||fault;
 assign read_active=state!=IDLE;
 assign line_write=state==RESPONSE&&!is_fence&&rvalid&&!fault;
 assign line_data=rdata;assign line_address={2'd0,beat};assign line_bank=cache_bank;
 wire[24:0]target_address=BASE+(draw_bank?STRIDE:0)+{draw_address[15:3],3'd0};
 wire[15:0]fenced_colour=rdata>>(last_lane*16);
 always @(posedge clk)begin
  if(!rst_n)begin bank_sync<=0;job_sync0<=0;job_sync1<=0;end
  else begin bank_sync<={bank_sync[0],front_bank_pixel};job_sync0<=front_job_pixel;job_sync1<=job_sync0;end
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   write_pending<=0;aw_sent<=0;w_sent<=0;write_address<=0;write_data<=0;write_mask<=0;
   last_address<=0;last_lane<=0;last_colour<=0;write_age<=0;read_age<=0;
   state<=IDLE;is_fence<=0;scan_bank<=0;cache_bank<=0;beat<=0;row_base<=0;scan_job<=0;
   flush_done<=0;line_done<=0;line_good<=0;line_frame<=0;line_session<=0;fault<=0;
  end else if(!fault)begin
   line_done<=0;
   if(!flush_request)flush_done<=0;
   if(write_pending)begin
    write_age<=write_age+1'b1;
    if(awvalid&&awready)aw_sent<=1;
    if(wvalid&&wready)w_sent<=1;
    if((aw_sent||(awvalid&&awready))&&(w_sent||(wvalid&&wready)))begin
     write_pending<=0;aw_sent<=0;w_sent<=0;write_age<=0;
    end
    if(write_age==1000000)fault<=1;
   end
   if(take_draw&&needs_write)begin
    if(draw_address>=WIDTH*HEIGHT)fault<=1;
    else begin
     write_pending<=1;write_address<=target_address;aw_sent<=0;w_sent<=0;write_age<=0;
     write_data<={8{draw_rgb}};
     write_mask<=draw_clear?16'd0:(16'hffff^(16'h0003<<(draw_address[2:0]*2)));
     last_address<=target_address;last_lane<=draw_address[2:0];last_colour<=draw_rgb;
    end
   end
   if(state!=IDLE)begin
    read_age<=read_age+1'b1;if(read_age==1000000)fault<=1;
   end else read_age<=0;
   case(state)
    IDLE:if(!write_pending)begin
     if(flush_request&&!flush_done)begin is_fence<=1;state<=COMMAND;read_age<=0;end
     else if(line_request&&line_ready)begin
      if(line_row>=HEIGHT)begin line_done<=1;line_good<=0;end
      else begin
       is_fence<=0;cache_bank<=request_bank;beat<=0;state<=COMMAND;
       if(new_frame)begin scan_bank<=bank_sync[1];scan_job<=job_sync1;end
       row_base<=BASE+((new_frame?bank_sync[1]:scan_bank)?STRIDE:0)+line_row*WIDTH;
       line_good<=0;read_age<=0;
      end
     end
    end
    COMMAND:if(arvalid&&arready)state<=RESPONSE;
    RESPONSE:if(rvalid&&rready)begin
     if(!rlast)fault<=1;
     if(is_fence)begin
      if(fenced_colour!=last_colour)fault<=1;else flush_done<=1;
      state<=IDLE;
     end else if(beat==ROW_WORDS-1)begin
      line_done<=1;line_good<=rlast;line_frame<=scan_job;
      line_session<=32'hD3800000|scan_job;state<=IDLE;
     end else begin beat<=beat+1'b1;state<=COMMAND;end
    end
    default:begin fault<=1;state<=IDLE;end
   endcase
   if(rvalid&&state!=RESPONSE)fault<=1;
  end
 end
 // synthesis translate_off
 initial begin
  if(WIDTH%8!=0||WIDTH*HEIGHT>STRIDE||BASE<6*2097152||BASE+2*STRIDE>33554432)
   $fatal(1,"Cloud DDR overlaps existing frame slots or exceeds word address space");
 end
 // synthesis translate_on
endmodule
