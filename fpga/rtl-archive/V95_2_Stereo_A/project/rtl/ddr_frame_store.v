`timescale 1ns/1ps
// 功能：DDR三帧所有权、128位写入、提交前读回屏障及显示行预取；全部属于dhi_clk域。
// WIDTH为8倍数、HEIGHT>0，WIDTH*HEIGHT<BUFFER_STRIDE，三槽总范围不超过25位DDR16位字地址。
// token接口为同步读FIFO：read&&!empty发起读，稍后token_valid返回130位START/DATA/END。
// 只有尺寸正确、END有效且最后一个DDR字读回一致的帧才发布；显示槽整帧固定，绝不被写端复用。
// line_req_valid/ready握手，new_frame仅在显示垂直消隐发起；source_row为0..HEIGHT-1，bank为双行缓存号。
// line_write时输出128位line_data、8位line_address；LINE_WORDS<=256，line_done脉冲伴随line_good。
// pAXI地址为16位DDR字，步进8；wstrb全0写全部字节，wlast每拍1；AR最多16个未返回请求。
// 读总线超时锁存fault并停止发布，需复位恢复，防止迟到响应被误认作新数据。无PC控制依赖。
module ddr_frame_store #(parameter WIDTH=1296,HEIGHT=972,BUFFER_STRIDE=2097152,
 parameter FRAME_WORDS=WIDTH*HEIGHT/8,LINE_WORDS=WIDTH/8,TIMEOUT=100000,READ_WINDOW=16,WRITE_DRAIN=128)(
 input wire clk,input wire rst_n,input wire memory_ready,
 input wire [31:0] time_gray,output reg [31:0] front_start_ms,
 input wire token_empty,output wire token_read,input wire token_valid,input wire [129:0] token_data,
 input wire line_req_valid,output wire line_req_ready,input wire new_frame,input wire [15:0] source_row,input wire request_bank,
 output wire line_write,output wire [127:0] line_data,output wire [7:0] line_address,output wire line_bank,
 output reg line_done,output reg line_good,output reg front_valid,output reg [31:0] front_id,output reg [31:0] front_session,
 output wire latest_valid,output wire[31:0]latest_frame_id,latest_session,
 output wire [24:0] awaddr,output wire awvalid,input wire awready,
 output wire [127:0] wdata,output wire [15:0] wstrb,output wire wvalid,output wire wlast,input wire wready,
 output wire [24:0] araddr,output wire arvalid,input wire arready,
 input wire [127:0] rdata,input wire rvalid,input wire rlast,output wire rready,
 output reg fault,output reg [31:0] published_frames,output reg [31:0] rejected_frames,
 output wire [127:0] debug_state,output wire read_active
);
 localparam FETCH=0,WAIT_TOKEN=1,WRITE=2,FENCE_REQ=3,FENCE_WAIT=4;
 localparam IDLE=0,LINE=1,FENCE_ADDR=2,FENCE_DATA=3;
 reg [2:0] writer;reg [1:0] reader;
 reg [1:0] front_buffer,ready_buffer,write_buffer;
 reg ready_valid,write_active,aw_sent,w_sent;
 reg [31:0] ready_id,write_id;reg[(FRAME_WORDS>0?$clog2(FRAME_WORDS+1):1)-1:0]write_count;wire[31:0]write_count_debug=write_count;
 reg [31:0] ready_session,write_session;
 // Registered Gray timer crosses into DDR; second stage remains timed normally.
 // Board B does not consume timestamps; allow its constant input chain to trim.
 reg [31:0] time_sync0,time_sync1;
 reg [31:0] write_start_ms,ready_start_ms;
 function [31:0] ungray(input [31:0] g);
  integer n;begin ungray[31]=g[31];for(n=30;n>=0;n=n-1)ungray[n]=ungray[n+1]^g[n];end
 endfunction
 always @(posedge clk) begin
  if(!rst_n)begin time_sync0<=0;time_sync1<=0;end
  else begin time_sync0<=time_gray;time_sync1<=time_sync0;end
 end
 reg [24:0] write_address,line_base;
 reg [127:0] write_word;
 reg [8:0] issued,returned;
 reg task_bank,line_bad,fence_done,fence_good;
 reg[(TIMEOUT>1?$clog2(TIMEOUT):1)-1:0]write_age,read_age;
 // 官方MC在本板读写交叠测试出现丢返回；行读取前暂停新写并留排空周期，读完再恢复写。
 reg [7:0] read_guard;
 // 仅诊断：超时来源1=等待FIFO/写通道/屏障，2=读通道；错误时状态冻结，需复位恢复。
 reg [2:0] fault_reason;
 assign debug_state={8'b0,fault_reason,8'b0,reader,writer,awready,wready,arready,rvalid,rlast,token_empty,aw_sent,w_sent,
                     7'b0,line_base,14'b0,issued,returned,write_count_debug};
 assign read_active=reader!=IDLE;
 wire fence_request=(writer==FENCE_REQ);
 wire fence_accept=(reader==IDLE && fence_request && memory_ready && !fault);
 wire publish=(writer==FENCE_WAIT && fence_done && fence_good && !fault);
 wire take_line=line_req_valid && line_req_ready;
 wire select_front=take_line && new_frame && ready_valid;
 wire [1:0] selected_front=select_front?ready_buffer:front_buffer;
 wire selected_valid=front_valid || select_front;
 // Read-only publication notice, independent of the display's next line read.
 assign latest_valid=ready_valid||front_valid;
 assign latest_frame_id=ready_valid?ready_id:front_id;
 assign latest_session=ready_valid?ready_session:front_session;
 // 三槽中排除显示槽及最新待显示槽；新帧开始才取得空闲槽，未完成旧帧可安全覆盖。
 wire [1:0] free_buffer=(front_buffer!=0 && (!ready_valid || ready_buffer!=0))?0:
                        (front_buffer!=1 && (!ready_valid || ready_buffer!=1))?1:2;
 assign token_read=memory_ready && !fault && writer==FETCH && !token_empty;
 assign awaddr=write_address;assign wdata=write_word;assign wstrb=0;assign wlast=1;
 assign awvalid=memory_ready && !fault && writer==WRITE && !aw_sent && reader!=LINE;
 assign wvalid=memory_ready && !fault && writer==WRITE && !w_sent && reader!=LINE;
 assign line_req_ready=memory_ready && !fault && reader==IDLE && !fence_request && writer!=WRITE;
 assign arvalid=!fault && ((reader==LINE && read_guard==0 && issued<LINE_WORDS && issued-returned<READ_WINDOW) || reader==FENCE_ADDR);
 assign araddr=(reader==LINE)?line_base+{issued,3'b0}:write_address;
 assign rready=(reader==LINE || reader==FENCE_DATA || fault);
 assign line_write=(reader==LINE && rvalid && !fault);
 assign line_data=rdata;assign line_address=returned[7:0];assign line_bank=task_bank;
 always @(posedge clk) begin
  if(!rst_n) begin
   writer<=FETCH;reader<=IDLE;front_buffer<=0;ready_buffer<=0;write_buffer<=1;ready_valid<=0;front_valid<=0;
   ready_session<=0;write_session<=0;front_session<=0;ready_id<=0;front_id<=0;write_start_ms<=0;ready_start_ms<=0;front_start_ms<=0;write_id<=0;write_count<=0;write_active<=0;aw_sent<=0;w_sent<=0;
   write_address<=0;write_word<=0;line_base<=0;issued<=0;returned<=0;task_bank<=0;line_bad<=0;read_guard<=0;
   fence_done<=0;fence_good<=0;line_done<=0;line_good<=0;fault<=0;fault_reason<=0;published_frames<=0;rejected_frames<=0;write_age<=0;read_age<=0;
  end else begin
   line_done<=0;fence_done<=0;
   if(select_front) begin front_buffer<=ready_buffer;front_id<=ready_id;front_session<=ready_session;front_start_ms<=ready_start_ms;front_valid<=1;ready_valid<=0;end
   if(publish) begin ready_buffer<=write_buffer;ready_id<=write_id;ready_session<=write_session;ready_start_ms<=write_start_ms;ready_valid<=1;published_frames<=published_frames+1'b1;end
   if(!fault && memory_ready) begin
    if(writer==WRITE || writer==WAIT_TOKEN || writer==FENCE_WAIT) begin
     if(write_age==TIMEOUT-1) begin fault<=1;fault_reason<=1;end else write_age<=write_age+1'b1;
    end else write_age<=0;
    case(writer)
     FETCH: if(!token_empty) writer<=WAIT_TOKEN;
     WAIT_TOKEN: if(token_valid) begin
      write_age<=0;writer<=FETCH;
      case(token_data[129:128])
       1: begin
        if(write_active) rejected_frames<=rejected_frames+1'b1;
        write_buffer<=free_buffer;write_active<=1;write_count<=0;write_id<=token_data[31:0];write_session<=token_data[63:32];write_start_ms<=ungray(time_sync1);
       end
       0: if(write_active) begin
        if(write_count<FRAME_WORDS) begin
         write_address<=write_buffer*BUFFER_STRIDE+write_count*8;write_word<=token_data[127:0];
         aw_sent<=0;w_sent<=0;writer<=WRITE;
        end else begin write_active<=0;rejected_frames<=rejected_frames+1'b1;end
       end
       2: begin
        if(write_active && token_data[0] && token_data[32:1]==write_id && token_data[64:33]==write_session && write_count==FRAME_WORDS) writer<=FENCE_REQ;
        else begin write_active<=0;rejected_frames<=rejected_frames+1'b1;end
       end
       default: begin write_active<=0;rejected_frames<=rejected_frames+1'b1;end
      endcase
     end
     WRITE: begin
      if(awvalid && awready) begin aw_sent<=1;write_age<=0;end
      if(wvalid && wready) begin w_sent<=1;write_age<=0;end
      if((aw_sent || (awvalid && awready)) && (w_sent || (wvalid && wready))) begin writer<=FETCH;write_count<=write_count+1'b1;end
     end
     FENCE_REQ: if(fence_accept) begin writer<=FENCE_WAIT;write_age<=0;end
     FENCE_WAIT: if(fence_done) begin
      writer<=FETCH;write_active<=0;write_age<=0;
      if(!fence_good) rejected_frames<=rejected_frames+1'b1;
     end
     default: writer<=FETCH;
    endcase
    if(reader!=IDLE) begin
     if(read_age==TIMEOUT-1) begin fault<=1;fault_reason<=2;line_done<=1;line_good<=0;end
     else read_age<=read_age+1'b1;
    end else read_age<=0;
    case(reader)
     IDLE: begin
      if(fence_accept) begin reader<=FENCE_ADDR;read_age<=0;end
      else if(take_line) begin
       if(selected_valid && source_row<HEIGHT) begin
        line_base<=selected_front*BUFFER_STRIDE+source_row*WIDTH;task_bank<=request_bank;
        issued<=0;returned<=0;line_bad<=0;reader<=LINE;read_age<=0;read_guard<=WRITE_DRAIN;
       end else begin line_done<=1;line_good<=0;end
      end
     end
     LINE: begin
      if(read_guard!=0) read_guard<=read_guard-1'b1;
      if(arvalid && arready) begin issued<=issued+1'b1;read_age<=0;end
      if(rvalid) begin
       returned<=returned+1'b1;read_age<=0;if(!rlast) line_bad<=1;
       if(returned==LINE_WORDS-1) begin reader<=IDLE;line_done<=1;line_good<=!line_bad && rlast;end
      end
     end
     FENCE_ADDR: if(arready) begin reader<=FENCE_DATA;read_age<=0;end
     FENCE_DATA: if(rvalid) begin
      reader<=IDLE;fence_done<=1;fence_good<=rlast && rdata==write_word;read_age<=0;
     end
    endcase
   end
  end
 end
endmodule
