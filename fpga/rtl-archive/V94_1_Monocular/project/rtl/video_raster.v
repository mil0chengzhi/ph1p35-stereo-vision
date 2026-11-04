`timescale 1ns/1ps
// 功能：产生逐像素光栅坐标及同步信号；所有输出属于 clk_pix 时钟域。
// 参数：各有效尺寸/同步宽度>0，前后沿>=0，总行列长度<=65535；极性为0或1。
// 输入：clk_pix 1位像素时钟；rst_n 1位同步低有效复位；enable 1位时钟使能。
// 输出：x/y 16位计数，消隐期亦有效；de/sof/eol/hsync/vsync 均1位。
// 握手：无ready；enable=1时每拍一个位置，sof/eol只在有效首/末像素置位。
// 延迟：输出由当前计数直接译码，0拍；暂停时坐标保持，同步保持，de/标记为0。
module video_raster #(
    parameter H_ACTIVE=1024, H_FRONT=1, H_SYNC=1, H_BACK=1,
    parameter V_ACTIVE=600, V_FRONT=1, V_SYNC=1, V_BACK=1,
    parameter H_POL=0, V_POL=0
)(
    input wire clk_pix, input wire rst_n, input wire enable,
    output reg [15:0] x, output reg [15:0] y,
    output wire de, output wire sof, output wire eol,
    output wire hsync, output wire vsync
);
    // 默认消隐仅为模块仿真占位，板级实例必须显式配置经过验证的完整时序。
    localparam H_TOTAL=H_ACTIVE+H_FRONT+H_SYNC+H_BACK;
    localparam V_TOTAL=V_ACTIVE+V_FRONT+V_SYNC+V_BACK;
    always @(posedge clk_pix) begin
        if (!rst_n) begin x<=0; y<=0; end
        else if (enable) begin
            if (x==H_TOTAL-1) begin
                x<=0;
                if (y==V_TOTAL-1) y<=0; else y<=y+1'b1;
            end else x<=x+1'b1;
        end
    end
    assign de=rst_n && enable && x<H_ACTIVE && y<V_ACTIVE;
    assign sof=de && x==0 && y==0;
    assign eol=de && x==H_ACTIVE-1;
    assign hsync=(rst_n && x>=H_ACTIVE+H_FRONT && x<H_ACTIVE+H_FRONT+H_SYNC) ? H_POL : !H_POL;
    assign vsync=(rst_n && y>=V_ACTIVE+V_FRONT && y<V_ACTIVE+V_FRONT+V_SYNC) ? V_POL : !V_POL;
endmodule
