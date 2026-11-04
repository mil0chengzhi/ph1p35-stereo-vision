`timescale 1ns/1ps
// 功能：RGB565转灰度后执行流式3x3 Sobel，输出L1强度min(255,abs(Gx)+abs(Gy))，不做阈值化。
// WIDTH/HEIGHT为屏幕有效区尺寸，范围3..65535，默认1024x600，可配置1280x800。
// 全部端口属于clk像素域，目标50MHz；rst_n同步低有效。每拍可接收1个像素，无反压。
// x/y为16位屏幕坐标，de标记有效区，valid标记RGB565数据可用；即使valid=0仍须提供该位置及de。
// 每行从x=0开始依次到WIDTH-1；行间允许任意消隐，消隐不算缺像素；帧从(0,0)开始。
// 输入第N个上升沿采样，对应out_x/out_y/out_de/edge_valid/edge_strength在第N+3沿输出，延迟3个周期。
// 行缓存使窗口右下角对应out_x/out_y，计算中心为(out_x-1,out_y-1)：内容在输出坐标上晚1行1像素。
// 为保持显示流水线，不提前输出坐标、不在消隐补尾；首两行/列及最后一行/列强制0且edge_valid=0。
// 完整窗口内9个像素均有效时edge_valid=1；缺像素、缺行、乱序行均屏蔽，不将其作为黑色制造假边。
// 连续收到两条完整有效坐标行后可恢复；像素valid=0只影响覆盖该像素的3x3窗口，不污染后续行。
// 两块WIDTH深9位RAM保存灰度与像素有效位，按行奇偶轮换；同步读、同址读写取旧值，需推断READ_FIRST。
// 复位仅清控制/行标签/流水线，不清RAM；完整行标签和每像素有效位隔离旧帧及未初始化存储。
module video_sobel #(parameter WIDTH=1024,HEIGHT=600)(
 input wire clk,input wire rst_n,input wire [15:0] x,input wire [15:0] y,
 input wire de,input wire valid,input wire [15:0] rgb565,
 output reg [7:0] edge_strength,output reg edge_valid,
 output reg [15:0] out_x,output reg [15:0] out_y,output reg out_de
);
 // 功能：计算RAM地址宽度ceil(log2(size))；size合法范围3..65535，返回整数位数，无错误返回。
 function integer address_bits;input integer size;integer remaining;
  begin remaining=size-1;address_bits=0;
   while(remaining>0) begin remaining=remaining>>1;address_bits=address_bits+1;end
  end
 endfunction
 localparam ADDRESS_BITS=address_bits(WIDTH);
 wire in_range=(x<WIDTH)&&(y<HEIGHT);
 wire [ADDRESS_BITS-1:0] address=(x<WIDTH)?x[ADDRESS_BITS-1:0]:{ADDRESS_BITS{1'b0}};
 wire [7:0] red={rgb565[15:11],rgb565[15:13]};
 wire [7:0] green={rgb565[10:5],rgb565[10:9]};
 wire [7:0] blue={rgb565[4:0],rgb565[4:2]};
 wire [15:0] gray_sum=red*16'd77+green*16'd150+blue*16'd29+16'd128;
 wire [7:0] gray=gray_sum[15:8];
 reg [8:0] line_even[0:WIDTH-1];reg [8:0] line_odd[0:WIDTH-1];
 reg [8:0] even_read,odd_read;
 reg [15:0] tag_even,tag_odd,row_y,next_x;
 reg complete_even,complete_odd,row_active,row_good,top_history,middle_history;
 wire start_row=de&&in_range&&(x==0);
 wire next_pixel=row_active&&row_good&&(y==row_y)&&(x==next_x);
 wire sequence_good=de&&in_range&&(start_row||next_pixel);
 wire previous_rows=(y>=2)&&
  (y[0]?(complete_odd&&(tag_odd==y-2)&&complete_even&&(tag_even==y-1)):
        (complete_even&&(tag_even==y-2)&&complete_odd&&(tag_odd==y-1)));
 // 只在首次接收该行前读取标签；写本行会立即撤销被覆盖的两行前标签，历史有效性须另存整行。
 wire history_good=start_row?previous_rows:(top_history&&middle_history);

 // 同步读与同地址写位于同一时序过程，非阻塞赋值描述读旧值；无RAM复位分支。
 always @(posedge clk) begin
  even_read<=line_even[address];odd_read<=line_odd[address];
  if(rst_n && de && in_range) begin
   if(!y[0]) line_even[address]<={valid&&sequence_good,gray};
   else line_odd[address]<={valid&&sequence_good,gray};
  end
 end

 reg [15:0] x0,y0,x1,y1,x2,y2;
 reg de0,de1,de2,seq0,history0;
 reg [8:0] current0;
 wire [8:0] top_column=y0[0]?odd_read:even_read;
 wire [8:0] middle_column=y0[0]?even_read:odd_read;
 // _left2/_left1分别为当前列之前两列/一列；第9位为样本有效，不靠灰度值判定有效性。
 reg [8:0] top_left2,top_left1,middle_left2,middle_left1,bottom_left2,bottom_left1;
 wire window_good=de0&&seq0&&history0&&(x0>=2)&&(y0>=2)&&
  (x0<WIDTH-1)&&(y0<HEIGHT-1)&&top_left2[8]&&top_left1[8]&&top_column[8]&&
  middle_left2[8]&&middle_left1[8]&&middle_column[8]&&bottom_left2[8]&&bottom_left1[8]&&current0[8];
 reg [9:0] left_sum,right_sum,upper_sum,lower_sum;
 reg window1,window2;
 reg [9:0] abs_x,abs_y;
 wire [10:0] magnitude={1'b0,abs_x}+{1'b0,abs_y};

 always @(posedge clk) begin
  if(!rst_n) begin
   tag_even<=0;tag_odd<=0;row_y<=0;next_x<=0;
   complete_even<=0;complete_odd<=0;row_active<=0;row_good<=0;top_history<=0;middle_history<=0;
   x0<=0;y0<=0;x1<=0;y1<=0;x2<=0;y2<=0;de0<=0;de1<=0;de2<=0;seq0<=0;history0<=0;current0<=0;
   top_left2<=0;top_left1<=0;middle_left2<=0;middle_left1<=0;bottom_left2<=0;bottom_left1<=0;
   left_sum<=0;right_sum<=0;upper_sum<=0;lower_sum<=0;window1<=0;window2<=0;abs_x<=0;abs_y<=0;
   edge_strength<=0;edge_valid<=0;out_x<=0;out_y<=0;out_de<=0;
  end else begin
   if(start_row) begin
    row_y<=y;next_x<=1;row_active<=1;row_good<=1;
    top_history<=previous_rows;middle_history<=previous_rows;
    if(y==0) begin complete_even<=0;complete_odd<=0;end
    if(y[0]) begin tag_odd<=y;complete_odd<=0;end
    else begin tag_even<=y;complete_even<=0;end
   end else if(de) begin
    if(!sequence_good) row_good<=0;
    if(sequence_good) next_x<=next_x+1'b1;
    if(in_range && x==WIDTH-1) begin
     // 行有效性描述坐标是否完整；像素有效位仍逐点保留，故单个无效像素不会丢整行。
     if(y[0]) complete_odd<=sequence_good;
     else complete_even<=sequence_good;
     row_active<=0;
    end
   end

   x0<=x;y0<=y;de0<=de;seq0<=sequence_good;history0<=history_good;current0<={valid&&sequence_good,gray};
   x1<=x0;y1<=y0;de1<=de0;x2<=x1;y2<=y1;de2<=de1;
   if(!de0 || !seq0 || x0==0) begin
    top_left2<=0;middle_left2<=0;bottom_left2<=0;
    top_left1<=(de0&&seq0)?top_column:9'd0;
    middle_left1<=(de0&&seq0)?middle_column:9'd0;
    bottom_left1<=(de0&&seq0)?current0:9'd0;
   end else begin
    top_left2<=top_left1;top_left1<=top_column;
    middle_left2<=middle_left1;middle_left1<=middle_column;
    bottom_left2<=bottom_left1;bottom_left1<=current0;
   end
   // 各正权重和最大1020，用10位无符号数；相减先比较后减，避免有符号扩展和溢出。
   left_sum<={2'b0,top_left2[7:0]}+{1'b0,middle_left2[7:0],1'b0}+{2'b0,bottom_left2[7:0]};
   right_sum<={2'b0,top_column[7:0]}+{1'b0,middle_column[7:0],1'b0}+{2'b0,current0[7:0]};
   upper_sum<={2'b0,top_left2[7:0]}+{1'b0,top_left1[7:0],1'b0}+{2'b0,top_column[7:0]};
   lower_sum<={2'b0,bottom_left2[7:0]}+{1'b0,bottom_left1[7:0],1'b0}+{2'b0,current0[7:0]};
   window1<=window_good;window2<=window1;
   abs_x<=(right_sum>=left_sum)?right_sum-left_sum:left_sum-right_sum;
   abs_y<=(lower_sum>=upper_sum)?lower_sum-upper_sum:upper_sum-lower_sum;
   out_x<=x2;out_y<=y2;out_de<=de2;edge_valid<=window2;
   edge_strength<=window2?((magnitude>11'd255)?8'hff:magnitude[7:0]):8'd0;
  end
 end
endmodule
