`timescale 1ns/1ps
// 功能：两路MIPI原始字节流搜索0xB8同步字，校正0..7bit边界，并用四字节FIFO吸收lane偏移。
// 参数无。clk为硬核字节时钟，rst_n低有效同步复位；active为本域已同步的HS接收窗口。
// lane0/lane1[7:0]每拍一个原始字节、低bit先收；active降为0时清空所有对齐/队列状态。
// 输出bytes低字节属于lane0，pair_valid每拍一对；packet_start与首对数据同拍。
// overflow出现时本窗口终止输出，由上层取消包。无背压；最大允许lane提前约3字节。
// 延迟：搜索完成后排队并输出，典型3拍，依赖两lane同步相对位置。
module mipi_pair_align(
    input wire clk,input wire rst_n,input wire active,input wire [7:0] lane0,input wire [7:0] lane1,
    output reg [15:0] bytes,output reg pair_valid,output reg packet_start,output reg overflow,
    output wire [1:0] aligned
);
    reg [7:0] previous0,previous1;
    reg have_previous,found0,found1,sent_first,failed;
    reg [2:0] offset0,offset1;
    reg [7:0] fifo0[0:3],fifo1[0:3];
    reg [1:0] wr0,wr1,rd0,rd1;
    reg [2:0] count0,count1;
    wire [15:0] window0={lane0,previous0},window1={lane1,previous1};
    wire pop=(count0!=0 && count1!=0 && !failed);
    integer bit_no;
    assign aligned={found1,found0};
    always @(posedge clk) begin
        if(!rst_n || !active) begin
            previous0<=0; previous1<=0; have_previous<=0; found0<=0; found1<=0;
            offset0<=0; offset1<=0; sent_first<=0; failed<=0;
            wr0<=0; wr1<=0; rd0<=0; rd1<=0; count0<=0; count1<=0;
            bytes<=0; pair_valid<=0; packet_start<=0; overflow<=0;
        end else begin
            previous0<=lane0; previous1<=lane1; have_previous<=1;
            pair_valid<=0; packet_start<=0; overflow<=0;
            if(!found0 && have_previous) for(bit_no=0;bit_no<8;bit_no=bit_no+1)
                if(((window0>>bit_no)&16'h00ff)==16'h00b8) begin found0<=1; offset0<=bit_no; end
            if(!found1 && have_previous) for(bit_no=0;bit_no<8;bit_no=bit_no+1)
                if(((window1>>bit_no)&16'h00ff)==16'h00b8) begin found1<=1; offset1<=bit_no; end
            if(!failed) begin
                if(found0 && (count0<4 || pop)) begin fifo0[wr0]<=window0>>offset0; wr0<=wr0+1'b1; end
                if(found1 && (count1<4 || pop)) begin fifo1[wr1]<=window1>>offset1; wr1<=wr1+1'b1; end
                if((found0 && count0==4 && !pop) || (found1 && count1==4 && !pop)) begin
                    overflow<=1; failed<=1;
                end else begin
                    count0<=count0+(found0?1'b1:1'b0)-(pop?1'b1:1'b0);
                    count1<=count1+(found1?1'b1:1'b0)-(pop?1'b1:1'b0);
                    if(pop) begin
                        bytes<={fifo1[rd1],fifo0[rd0]}; pair_valid<=1; packet_start<=!sent_first;
                        sent_first<=1; rd0<=rd0+1'b1; rd1<=rd1+1'b1;
                    end
                end
            end
        end
    end
endmodule
