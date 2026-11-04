`timescale 1ns/1ps
// 功能：将2像素RGB565流打包为128位DDR字，并用START/DATA/END标记经校验的完整帧。
// RAW_WIDTH为16倍数，RAW_HEIGHT为偶数；输出宽高为原始各半，WORDS=RAW_WIDTH*RAW_HEIGHT/32。
// clk字节域，rst_n同步低有效；frame/line标记来自CSI包解析，line_bytes为RAW10头中的字节长度。
// pair[31:0]/pair_valid来自ISP，最多1拍2像素；frame_end后等待8拍及最后pair排空。
// fifo_data[129:128]：00数据/01开始/10结束；END低位为完整性，1才允许显示。
// fifo_write与!fifo_full同拍表示接受，无回压到摄像头；满时标记整帧丢弃，下一START恢复。
// 不发布半帧；accepted/dropped为饱和32位帧计数，frame_id单调回绕32位，仅作序号。
module camera_frame_pack #(parameter RAW_WIDTH=2592,RAW_HEIGHT=1944,
 parameter WORDS=RAW_WIDTH*RAW_HEIGHT/32)(
 input wire clk,input wire rst_n,input wire frame_start,input wire frame_end,
 input wire line_start,input wire line_end,input wire line_good,input wire [15:0] line_bytes,
 input wire [31:0] pair,input wire pair_valid,input wire fifo_full,
 output wire [129:0] fifo_data,output wire fifo_write,
 output reg [31:0] accepted,output reg [31:0] dropped,
 output reg[31:0] last_drop_info,last_drop_words,fifo_full_events
);
 reg[7:0] reasons;
 reg in_frame,bad,dropping,line_active,ending;
 reg [3:0] drain;
 reg [15:0] lines;
 reg [31:0] words,frame_id;
 reg [1:0] count;
 reg [95:0] accumulated;
 wire emit_end=in_frame && ending && drain==0 && !pair_valid;
 wire emit_word=in_frame && !dropping && pair_valid && count==3;
 wire good_frame=!bad && !dropping && !line_active && lines==RAW_HEIGHT && words==WORDS && count==0;
 assign fifo_write=!fifo_full && (frame_start || emit_word || emit_end);
 assign fifo_data=frame_start?{2'b01,96'b0,frame_id+1'b1}:
                  emit_end?{2'b10,95'b0,frame_id,good_frame}:{2'b00,pair,accumulated};
 always @(posedge clk) begin
  if(!rst_n) begin in_frame<=0;bad<=0;dropping<=0;line_active<=0;ending<=0;drain<=0;lines<=0;words<=0;
   frame_id<=0;count<=0;accumulated<=0;accepted<=0;dropped<=0;reasons<=0;last_drop_info<=0;last_drop_words<=0;fifo_full_events<=0;end
  else if(frame_start) begin
   if(in_frame)begin
    if(!(&dropped))dropped<=dropped+1'b1;
    last_drop_info<={12'd0,lines[11:0],reasons|8'h01};last_drop_words<=words;
   end
   in_frame<=1;bad<=0;dropping<=fifo_full;reasons<=fifo_full?8'h08:8'd0;line_active<=0;ending<=0;drain<=0;lines<=0;words<=0;
   frame_id<=frame_id+1'b1;count<=0;
  end else if(in_frame) begin
   if(line_start) begin
    if(line_active || line_bytes!=RAW_WIDTH*5/4 || lines>=RAW_HEIGHT)begin bad<=1;reasons[1]<=1;end
    line_active<=1;
   end
   if(line_end) begin
    if(!line_active || !line_good)begin bad<=1;reasons[2]<=1;end
    line_active<=0;lines<=lines+1'b1;
   end
   if(pair_valid && !dropping) begin
    if(count==3) begin
     count<=0;words<=words+1'b1;
     if(fifo_full)begin dropping<=1;reasons[3]<=1;if(!(&fifo_full_events))fifo_full_events<=fifo_full_events+1'b1;end
    end else begin accumulated[count*32 +:32]<=pair;count<=count+1'b1;end
   end
   if(frame_end) begin ending<=1;drain<=8;end
   else if(ending && (pair_valid || drain!=0)) begin
    if(pair_valid) drain<=8;else drain<=drain-1'b1;
   end
   if(emit_end && !fifo_full) begin
    in_frame<=0;ending<=0;
    if(good_frame) begin if(!(&accepted)) accepted<=accepted+1'b1;end
    else begin
     if(!(&dropped))dropped<=dropped+1'b1;
     last_drop_info<={12'd0,lines[11:0],(reasons|{line_active,count!=0,words!=WORDS,lines!=RAW_HEIGHT,4'd0})};
     last_drop_words<=words;
    end
   end
  end
 end
endmodule
