`timescale 1ns/1ps
// Explicit static320x180 depth status. RX GAP is counted in display rasters,
// not sensor time, computation time or end-to-end latency. First observed job
// has UNKNOWN age until a later different nonzero job is actually observed.
// One pixel of RGB/DE/HS/VS latency applies equally to enabled/disabled modes.
module video_depth_status #(
 parameter FONT_FILE="depth_ascii_8x16.mem",STALE_FRAMES=180
)(
 input wire clk,rst_n,enable,input wire[15:0]x,y,input wire[23:0]rgb,
 input wire de,hs,vs,input wire job_valid,link_ok,input wire[21:0]job_id,
 output wire[23:0]out_rgb,output wire out_de,out_hs,out_vs,
 output reg seen_job,gap_known,output reg[21:0]shown_job,output reg[19:0]gap_frames
);
 reg[7:0]font[0:1535];initial $readmemh(FONT_FILE,font);
 wire tick=x==0&&y==1;
 always @(posedge clk)begin
  if(!rst_n)begin seen_job<=0;gap_known<=0;shown_job<=0;gap_frames<=0;end
  else if(tick)begin
   if(!job_valid||job_id==0)begin seen_job<=0;gap_known<=0;shown_job<=0;gap_frames<=0;end
   else if(!seen_job)begin seen_job<=1;gap_known<=0;shown_job<=job_id;gap_frames<=0;end
   else if(shown_job!=job_id)begin shown_job<=job_id;gap_known<=1;gap_frames<=0;end
   else if(gap_known&&gap_frames!=20'hfffff)gap_frames<=gap_frames+1'b1;
  end
 end
 wire[15:0]px=x-16,py=y-12;
 wire panel=enable&&x>=16&&x<608&&y>=12&&y<108;
 wire[5:0]column=px[9:4];wire[1:0]row=py[6:5];
 wire[7:0]ascii;
 function[7:0]hex_ascii;
  input[3:0]n;
  begin hex_ascii=n<10?8'd48+n:8'd65+n-10;end
 endfunction
 localparam[159:0]TITLE="STATIC DEPTH 320x180";
 localparam[279:0]DETAIL="JOB 0x------  RX GAP 0x----- FRAMES";
 localparam[71:0]LOST="LINK LOST";
 localparam[151:0]WAITING="WAITING NEXT RESULT";
 localparam[191:0]STALE="NO NEW DEPTH >180 FRAMES";
 localparam[175:0]ACTIVE="STATIC RESULT RECEIVED";
 reg[7:0]character;
 always @*begin
  character=8'd32;
  if(row==0&&column<20)character=TITLE[(19-column)*8+:8];
  else if(row==1&&column<35)begin
   character=DETAIL[(34-column)*8+:8];
   if(column>=6&&column<12&&seen_job)character=hex_ascii(({2'd0,shown_job}>>((11-column)*4))&4'hf);
   if(column>=23&&column<28&&gap_known)character=hex_ascii((gap_frames>>((27-column)*4))&4'hf);
  end else if(row==2)begin
   if(!link_ok&&column<9)character=LOST[(8-column)*8+:8];
   else if(link_ok&&!gap_known&&column<19)character=WAITING[(18-column)*8+:8];
   else if(link_ok&&gap_known&&gap_frames>STALE_FRAMES&&column<24)character=STALE[(23-column)*8+:8];
   else if(link_ok&&gap_known&&gap_frames<=STALE_FRAMES&&column<22)character=ACTIVE[(21-column)*8+:8];
  end
 end
 assign ascii=character;
 wire[10:0]font_address={ascii[6:0]-7'd32,py[4:1]};
 reg[7:0]glyph;reg[2:0]gx;reg draw_panel,de_q,hs_q,vs_q;reg[23:0]rgb_q,ink;
 always @(posedge clk)begin
  if(!rst_n)begin glyph<=0;gx<=0;draw_panel<=0;de_q<=0;hs_q<=1;vs_q<=1;rgb_q<=0;ink<=0;end
  else begin
   glyph<=font[font_address];gx<=px[3:1];draw_panel<=panel;
   de_q<=de;hs_q<=hs;vs_q<=vs;rgb_q<=rgb;
   ink<=row==2?(!link_ok?24'hff6050:!gap_known?24'hffd050:gap_frames>STALE_FRAMES?24'hffd050:24'h80efb0):24'hffffff;
  end
 end
 assign out_rgb=draw_panel&&de_q?(glyph[7-gx]?ink:24'h101820):rgb_q;
 assign out_de=de_q;assign out_hs=hs_q;assign out_vs=vs_q;
endmodule
