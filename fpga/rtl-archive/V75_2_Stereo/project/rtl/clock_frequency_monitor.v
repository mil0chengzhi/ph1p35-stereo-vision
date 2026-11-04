`timescale 1ns/1ps
// 功能：被测时钟先256分频，单比特同步到参考时钟域，在固定窗口计上升沿。
// 参数WINDOW_CYCLES>=256；参考50MHz默认5M拍=100ms，频率估计=count*256*10 Hz。
// rst_n低有效，异步置位复位、各域计数；source_clk可停止，参考域仍输出0。
// 输出edges[31:0]为完整窗口计数，fresh每个窗口1拍；连续无握手，跨域误差约±1计数。
module clock_frequency_monitor #(parameter WINDOW_CYCLES=5000000)(
    input wire reference_clk,input wire source_clk,input wire rst_n,
    output reg [31:0] edges,output reg fresh
);
    reg [7:0] source_div;
    reg [1:0] source_reset;
    reg [2:0] sync_div;
    reg [31:0] window_count,edge_count;
    always @(posedge source_clk or negedge rst_n) begin
        if(!rst_n) source_reset<=0; else source_reset<={source_reset[0],1'b1};
    end
    always @(posedge source_clk) begin
        if(!source_reset[1]) source_div<=0; else source_div<=source_div+1'b1;
    end
    always @(posedge reference_clk or negedge rst_n) begin
        if(!rst_n) begin sync_div<=0; window_count<=0; edge_count<=0; edges<=0; fresh<=0; end
        else begin
            sync_div<={sync_div[1:0],source_div[7]}; fresh<=0;
            if(window_count==WINDOW_CYCLES-1) begin
                edges<=edge_count+((sync_div[1]&&!sync_div[2])?1'b1:1'b0);
                edge_count<=0; window_count<=0; fresh<=1;
            end else begin
                window_count<=window_count+1'b1;
                if(sync_div[1]&&!sync_div[2]) edge_count<=edge_count+1'b1;
            end
        end
    end
endmodule
