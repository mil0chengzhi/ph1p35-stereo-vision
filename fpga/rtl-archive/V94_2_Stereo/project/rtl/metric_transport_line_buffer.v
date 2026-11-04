`timescale 1ns/1ps
// 功能：HDMI双行缓存、最近邻缩放及像素域/DDR域请求握手；保持源图视野和独立行所有权。
// SCREEN/DEST尺寸>0，显示ROI必须在屏幕内；SOURCE_WIDTH<=2048且为8倍数，SOURCE_HEIGHT<=65535。
// rst_n共同异步低有效，两域各两拍同步释放；RAM不复位，只有本显示帧相应行就绪才输出valid。
// 像素输入x/y/de来自光栅，pixel_data RGB565/pixel_valid延迟1拍；ROI外和缺行输出无效，不沿用旧行。
// 每个显示垂直消隐开始预取首行并请求换帧；有效行开始预取下一行，目标缓存始终为另一半。
// DDR接口mem_request/ready握手后等待mem_done，mem_good表示该行完整；request_*保持至done。
// mem_write时将128位数据写入mem_bank/mem_address；源端返回done后跨两拍才让像素端读取。
// underflows为像素域饱和32位计数；用于诊断，不证明图像内容正确。
// mem_frame_id为DDR域当前显示源帧号，在mem_done时冻结并随原有ACK整体传递。
// source_frame_id/source_frame_valid属像素域，在首行返回后更新并保持整帧；新垂直消隐先失效。
module metric_transport_line_buffer #(parameter SCREEN_WIDTH=1024,SCREEN_HEIGHT=600,
 parameter DEST_WIDTH=800,DEST_HEIGHT=600,OFFSET_X=112,OFFSET_Y=0,SOURCE_WIDTH=1296,SOURCE_HEIGHT=972,
 parameter SOURCE_Y_OFFSET=0,SOURCE_WINDOW_HEIGHT=SOURCE_HEIGHT)(
 input wire pixel_clk,input wire memory_clk,input wire rst_n,
 input wire [15:0] x,input wire [15:0] y,input wire de,
 output wire [15:0] pixel_data,output reg pixel_valid,output reg [31:0] underflows,
 output reg [31:0] source_frame_id,output reg source_frame_valid,
 output reg [31:0] source_start_ms,input wire [31:0] mem_start_ms,
 output wire mem_request,input wire mem_ready,output wire request_new_frame,output wire [15:0] request_row,output wire request_bank,
 input wire mem_write,input wire [127:0] mem_data,input wire [7:0] mem_address,input wire mem_bank,
 input wire mem_done,input wire mem_good,input wire [31:0] mem_frame_id,
 input wire [31:0] mem_session,output reg [31:0] source_session
);
 localparam X_BASE=SOURCE_WIDTH/DEST_WIDTH,X_REMAINDER=SOURCE_WIDTH%DEST_WIDTH;
 localparam Y_BASE=SOURCE_WINDOW_HEIGHT/DEST_HEIGHT,Y_REMAINDER=SOURCE_WINDOW_HEIGHT%DEST_HEIGHT;
 localparam RAM_ADDRESS_BITS=$clog2(SOURCE_WIDTH/8);
 reg [127:0] memory[0:(2<<RAM_ADDRESS_BITS)-1];reg [127:0] read_word;reg [2:0] lane;
 reg [1:0] pixel_reset,memory_reset;
 reg req_toggle,ack_toggle,ack_seen,req_seen,mem_pending,mem_inflight;
 reg [1:0] req_sync,ack_sync;
 reg requested_new,requested_bank,response_good;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [31:0] response_frame_id;
 // Board B leaves source_start_ms unused; do not preserve a dead mailbox.
 reg [31:0] response_start_ms;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [31:0] response_session;
 // 收件寄存器固定在DDR域，先捕获邮箱，再交给地址运算；禁止综合吸收到跨域DSP输入寄存器。
 (* keep=1, preserve=1, allow_backward_retiming=0, allow_forward_retiming=0 *) reg [15:0] memory_row;
 (* keep=1, preserve=1 *) reg memory_new,memory_bank;
 reg [15:0] requested_row,requested_tag;
 reg [31:0] frame_number,requested_frame;
 reg [1:0] ready_bits;
 reg [15:0] ready_tag0,ready_tag1;
 reg [31:0] ready_frame0,ready_frame1;
 reg [15:0] source_x,source_y;
 reg [31:0] x_fraction,y_fraction;
 reg line_valid;
 wire in_row=y>=OFFSET_Y && y<OFFSET_Y+DEST_HEIGHT;
 wire in_image=de && in_row && x>=OFFSET_X && x<OFFSET_X+DEST_WIDTH;
 wire [15:0] target_y=y-OFFSET_Y;
 wire bank_now=target_y[0];
 wire row_ready=bank_now?(ready_bits[1] && ready_tag1==target_y && ready_frame1==frame_number):
                         (ready_bits[0] && ready_tag0==target_y && ready_frame0==frame_number);
 wire request_busy=req_toggle!=ack_sync[1];
 wire [15:0] next_source_y=source_y+Y_BASE+((y_fraction+Y_REMAINDER>=DEST_HEIGHT)?1:0);
 assign pixel_data=read_word>>(lane*16);
 assign mem_request=mem_pending && !mem_inflight;
 assign request_new_frame=memory_new;assign request_row=memory_row;assign request_bank=memory_bank;
 always @(posedge pixel_clk or negedge rst_n) begin
  if(!rst_n) pixel_reset<=0;else pixel_reset<={pixel_reset[0],1'b1};
 end
 always @(posedge memory_clk or negedge rst_n) begin
  if(!rst_n) memory_reset<=0;else memory_reset<={memory_reset[0],1'b1};
 end
 always @(posedge memory_clk) if(mem_write) memory[{mem_bank,mem_address[RAM_ADDRESS_BITS-1:0]}]<=mem_data;
 always @(posedge pixel_clk) read_word<=memory[{bank_now,source_x[RAM_ADDRESS_BITS+2:3]}];
 always @(posedge memory_clk or negedge memory_reset[1]) begin
  if(!memory_reset[1]) begin req_sync<=0;req_seen<=0;ack_toggle<=0;mem_pending<=0;mem_inflight<=0;response_good<=0;response_frame_id<=0;response_start_ms<=0;response_session<=0;memory_row<=0;memory_new<=0;memory_bank<=0;end
  else begin
   req_sync<={req_sync[0],req_toggle};
   if(req_sync[1]!=req_seen) begin req_seen<=req_sync[1];mem_pending<=1;memory_row<=requested_row;memory_new<=requested_new;memory_bank<=requested_bank;end
   if(mem_request && mem_ready) begin mem_pending<=0;mem_inflight<=1;end
   if(mem_done && mem_inflight) begin response_good<=mem_good;response_frame_id<=mem_frame_id;response_start_ms<=mem_start_ms;response_session<=mem_session;ack_toggle<=req_seen;mem_inflight<=0;end
  end
 end
 always @(posedge pixel_clk or negedge pixel_reset[1]) begin
  if(!pixel_reset[1]) begin
   req_toggle<=0;ack_sync<=0;ack_seen<=0;requested_new<=0;requested_bank<=0;requested_row<=0;requested_tag<=0;
   requested_frame<=0;frame_number<=0;ready_bits<=0;ready_tag0<=0;ready_tag1<=0;ready_frame0<=0;ready_frame1<=0;
   source_x<=0;source_y<=SOURCE_Y_OFFSET;x_fraction<=0;y_fraction<=0;line_valid<=0;pixel_valid<=0;lane<=0;underflows<=0;source_frame_id<=0;source_start_ms<=0;source_session<=0;source_frame_valid<=0;
  end else begin
   ack_sync<={ack_sync[0],ack_toggle};
   if(ack_sync[1]!=ack_seen) begin
    ack_seen<=ack_sync[1];ready_bits[requested_bank]<=response_good;
    if(requested_new && requested_frame==frame_number) begin source_frame_id<=response_frame_id;source_start_ms<=response_start_ms;source_session<=response_session;source_frame_valid<=response_good;end
    if(requested_bank) begin ready_tag1<=requested_tag;ready_frame1<=requested_frame;end
    else begin ready_tag0<=requested_tag;ready_frame0<=requested_frame;end
   end
   lane<=source_x[2:0];pixel_valid<=in_image && (x==0?row_ready:line_valid);
   if(!in_image) begin source_x<=0;x_fraction<=0;end
   else begin
    source_x<=source_x+X_BASE+((x_fraction+X_REMAINDER>=DEST_WIDTH)?1:0);
    x_fraction<=(x_fraction+X_REMAINDER>=DEST_WIDTH)?x_fraction+X_REMAINDER-DEST_WIDTH:x_fraction+X_REMAINDER;
   end
   if(x==0 && y==SCREEN_HEIGHT) begin
    frame_number<=frame_number+1'b1;ready_bits<=0;source_y<=SOURCE_Y_OFFSET;y_fraction<=0;source_frame_valid<=0;
    if(!request_busy) begin
     requested_new<=1;requested_row<=SOURCE_Y_OFFSET;requested_tag<=0;requested_bank<=0;
     requested_frame<=frame_number+1'b1;req_toggle<=!req_toggle;
    end
   end else if(x==0 && in_row) begin
    line_valid<=row_ready;
    if(!row_ready && !(&underflows)) underflows<=underflows+1'b1;
    if(target_y<DEST_HEIGHT-1) begin
     source_y<=next_source_y;
     y_fraction<=(y_fraction+Y_REMAINDER>=DEST_HEIGHT)?y_fraction+Y_REMAINDER-DEST_HEIGHT:y_fraction+Y_REMAINDER;
     if(!request_busy) begin
      requested_new<=0;requested_row<=next_source_y;requested_tag<=target_y+1'b1;requested_bank<=!bank_now;
      requested_frame<=frame_number;req_toggle<=!req_toggle;ready_bits[!bank_now]<=0;
     end
    end
   end
  end
 end
 // synthesis translate_off
 initial begin
  if(SOURCE_Y_OFFSET<0 || SOURCE_WINDOW_HEIGHT<1 ||
     SOURCE_Y_OFFSET+SOURCE_WINDOW_HEIGHT>SOURCE_HEIGHT)
   $fatal(1,"invalid source crop window");
  if(DEST_WIDTH<1 || DEST_HEIGHT<1 || OFFSET_X<0 || OFFSET_Y<0 ||
     OFFSET_X+DEST_WIDTH>SCREEN_WIDTH || OFFSET_Y+DEST_HEIGHT>SCREEN_HEIGHT)
   $fatal(1,"invalid display window");
 end
 // synthesis translate_on
endmodule

