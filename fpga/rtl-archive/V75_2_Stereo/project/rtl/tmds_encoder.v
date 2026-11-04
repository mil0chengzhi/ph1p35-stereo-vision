`timescale 1ns/1ps
// 功能：RGB单通道8b/10b TMDS视频编码；当前仅视频/控制期，不含音频或数据岛。
// 参数：无。输入clk属于像素域，rst_n同步低有效；data为8位，de为1位，ctrl为2位。
// 输出symbol为10位，最低位先发送；吞吐率1像素/拍，输入到输出1拍，无背压。
// disparity为有符号累计直流不平衡；控制期清零，所有通道使用相同复位与de。
module tmds_encoder(
    input wire clk, input wire rst_n, input wire [7:0] data,
    input wire de, input wire [1:0] ctrl, output reg [9:0] symbol
);
    reg signed [5:0] disparity;
    reg [8:0] transition;
    reg [9:0] encoded;
    integer i, ones, delta, next_disparity;
    reg use_xnor;
    always @* begin
        ones=0;
        for(i=0;i<8;i=i+1) ones=ones+data[i];
        use_xnor=(ones>4 || (ones==4 && !data[0]));
        transition[0]=data[0];
        for(i=1;i<8;i=i+1) transition[i]=transition[i-1]^data[i]^use_xnor;
        transition[8]=!use_xnor;
        ones=0;
        for(i=0;i<8;i=i+1) ones=ones+transition[i];
        delta=2*ones-8;
        encoded=0; next_disparity=disparity;
        if(disparity==0 || delta==0) begin
            encoded={~transition[8],transition[8],transition[8]?transition[7:0]:~transition[7:0]};
            next_disparity=disparity+(transition[8]?delta:-delta);
        end else if((disparity>0 && delta>0) || (disparity<0 && delta<0)) begin
            encoded={1'b1,transition[8],~transition[7:0]};
            next_disparity=disparity+(transition[8]?2:0)-delta;
        end else begin
            encoded={1'b0,transition[8],transition[7:0]};
            next_disparity=disparity-(transition[8]?0:2)+delta;
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin symbol<=10'b1101010100; disparity<=0; end
        else if(!de) begin
            disparity<=0;
            case(ctrl)
                0: symbol<=10'b1101010100;
                1: symbol<=10'b0010101011;
                2: symbol<=10'b0101010100;
                3: symbol<=10'b1010101011;
            endcase
        end else begin symbol<=encoded; disparity<=next_disparity; end
    end
endmodule
