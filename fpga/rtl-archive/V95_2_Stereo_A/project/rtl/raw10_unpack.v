`timescale 1ns/1ps
// 功能：CSI-2 RAW10五字节组解包成4个10位像素，不丢弃低2位。
// 参数无。clk域、rst_n同步低有效；line_start在一行载荷之前置1，清除上一行残余。
// bytes[15:0]低字节先到，keep=00/01/11；line_end应在最后一组载荷之后输入。
// pixels[39:0]低10位为最先像素，valid=1表示四像素同时有效，无背压；最大2字节/拍。
// partial_error在行结束仍不足五字节时输出1拍；组完成到输出1拍。下游须依据包CRC提交行。
module raw10_unpack(
    input wire clk,input wire rst_n,input wire line_start,input wire line_end,
    input wire [15:0] bytes,input wire [1:0] keep,
    output reg [39:0] pixels,output reg valid,output reg partial_error
);
    reg [31:0] high_bytes,next_high;
    reg [2:0] count,next_count;
    reg [39:0] result;
    reg result_valid,result_error;
    reg [7:0] b;
    integer lane;
    always @* begin
        next_high=high_bytes; next_count=count; result=0; result_valid=0; result_error=0; b=0;
        if(line_start) begin next_high=0; next_count=0; end
        for(lane=0;lane<2;lane=lane+1) begin
            b=bytes[lane*8 +: 8];
            if(keep[lane]) begin
                if(next_count==4) begin
                    result={next_high[31:24],b[7:6],next_high[23:16],b[5:4],
                            next_high[15:8],b[3:2],next_high[7:0],b[1:0]};
                    result_valid=1; next_count=0;
                end else begin next_high[next_count*8 +: 8]=b; next_count=next_count+1'b1; end
            end
        end
        if(line_end) begin result_error=(next_count!=0); next_count=0; end
    end
    always @(posedge clk) begin
        if(!rst_n) begin high_bytes<=0; count<=0; pixels<=0; valid<=0; partial_error<=0; end
        else begin high_bytes<=next_high; count<=next_count; pixels<=result;
            valid<=result_valid; partial_error<=result_error; end
    end
endmodule
