`timescale 1ns/1ps
// 功能：连续视频跨时钟FIFO，二进制RAM地址和Gray跨域指针；不进行逐位数据同步。
// WIDTH>=1、ADDRESS_BITS=2..16，深度2^ADDRESS_BITS。write_clk/read_clk互异，rst_n共同异步低有效。
// 两域复位各两拍同步释放；复位清空所有权但不复位RAM，任一域不得单独复位。
// write_en&&!full接受一字；write_en&&full时拒绝并输出overflow一拍。write_data宽WIDTH。
// read_en&&!empty接受读取请求，read_valid/read_data于该沿后有效一拍；消费者需保留返回空间。
// Gray指针在每个源域最多每拍变一位，板级需约束到目标第一级同步器的总线偏斜。
module video_async_fifo #(parameter WIDTH=130,ADDRESS_BITS=10)(
 input wire write_clk,input wire read_clk,input wire rst_n,
 input wire [WIDTH-1:0] write_data,input wire write_en,output reg full,output reg overflow,
 input wire read_en,output wire [WIDTH-1:0] read_data,output reg read_valid,output reg empty
);
 reg [ADDRESS_BITS:0] write_binary,read_binary;
 (* keep=1, preserve=1, allow_backward_retiming=0, allow_forward_retiming=0 *) reg [ADDRESS_BITS:0] write_gray,read_gray;
 (* keep=1, preserve=1, allow_backward_retiming=0, allow_forward_retiming=0 *) reg [ADDRESS_BITS:0] rgray_sync0,rgray_sync1,wgray_sync0,wgray_sync1;
 // 原始复位仅进入释放链；业务寄存器用本域同步释放的复位，避免异步撤销触发恢复时间违例。
 (* keep=1, preserve=1, allow_backward_retiming=0, allow_forward_retiming=0 *) reg [1:0] write_reset,read_reset;
 wire write_rst_n=write_reset[1],read_rst_n=read_reset[1];
 wire write_take=write_en && !full && write_reset[1];
 wire read_take=read_en && !empty && read_reset[1];
 wire [ADDRESS_BITS:0] next_write=write_binary+write_take,next_read=read_binary+read_take;
 wire [ADDRESS_BITS:0] next_wgray=(next_write>>1)^next_write,next_rgray=(next_read>>1)^next_read;
 always @(posedge write_clk or negedge rst_n) begin
  if(!rst_n) write_reset<=0;else write_reset<={write_reset[0],1'b1};
 end
 always @(posedge read_clk or negedge rst_n) begin
  if(!rst_n) read_reset<=0;else read_reset<={read_reset[0],1'b1};
 end
 always @(posedge write_clk or negedge write_rst_n) begin
  if(!write_rst_n) begin write_binary<=0;write_gray<=0;rgray_sync0<=0;rgray_sync1<=0;full<=0;overflow<=0;end
  else begin
   rgray_sync0<=read_gray;rgray_sync1<=rgray_sync0;
   write_binary<=next_write;write_gray<=next_wgray;
   full<=(next_wgray=={~rgray_sync1[ADDRESS_BITS:ADDRESS_BITS-1],rgray_sync1[ADDRESS_BITS-2:0]});
   overflow<=write_en && full;
  end
 end
 always @(posedge read_clk or negedge read_rst_n) begin
  if(!read_rst_n) begin read_binary<=0;read_gray<=0;wgray_sync0<=0;wgray_sync1<=0;empty<=1;read_valid<=0;end
  else begin
   wgray_sync0<=write_gray;wgray_sync1<=wgray_sync0;
   read_binary<=next_read;read_gray<=next_rgray;empty<=(next_rgray==wgray_sync1);
   read_valid<=read_take;
  end
 end
 // Only the board's existing 130x1024 FIFO uses explicit device RAM. Each
 // 20x1024 bank is one ERAM20K. The seventh bank contains ten payload bits;
 // unused upper bits are zero padding. Keep queue ownership/reset unchanged.
 // NOREG is the synchronous RAM output, not an additional OUTREG stage:
 // read_data is available after the same read edge as read_valid.
 generate if(WIDTH==130&&ADDRESS_BITS==10)begin:device_memory
  wire [139:0] padded_write={10'd0,write_data};wire [139:0] bank_read;
  genvar bank;
  for(bank=0;bank<7;bank=bank+1)begin:bank_memory
   PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),
    .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
    .BYTE_ENABLE(0),.BYTE_A(1),.BYTE_B(1),.MODE("DP"),
    .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
    .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.CLKMODE("ASYNC"),
    .IMPLEMENT("20K"),.FORCE_KEEP("ON"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) memory_bank(
    .dia(padded_write[bank*20+:20]),.dib(20'd0),
    .addra(write_binary[9:0]),.addrb(read_binary[9:0]),
    .clka(write_clk),.clkb(read_clk),.cea(write_take),.ceb(1'b1),
    .ocea(1'b1),.oceb(1'b1),.wea(write_take),.web(1'b0),
    .rsta(1'b0),.rstb(1'b0),.bea(1'b1),.beb(1'b1),
    .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr(),
    .doa(),.dob(bank_read[bank*20+:20]));
  end
  assign read_data=bank_read[129:0];
 end else begin:portable_memory
  (* ram_style="bram" *) reg [WIDTH-1:0] memory[0:(1<<ADDRESS_BITS)-1];
  reg [WIDTH-1:0] data;
  always @(posedge write_clk) if(write_take) memory[write_binary[ADDRESS_BITS-1:0]]<=write_data;
  always @(posedge read_clk) data<=memory[read_binary[ADDRESS_BITS-1:0]];
  assign read_data=data;
 end endgenerate
endmodule
