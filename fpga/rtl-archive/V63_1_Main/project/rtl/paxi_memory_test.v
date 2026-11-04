`timescale 1ns/1ps
// 功能：pAXI存储自检，先写后读WORDS个128位字，交替两组互补数据；不是AXI4完整接口。
// 参数WORDS=2..65536，BASE_ADDR为16位DDR字地址、8字对齐；WORDS*8不得超25位空间。
// clk属于IP输出dhi_clk域，rst_n同步低有效；ready为同域DDR校准完成。
// 地址25位，以DDR16位字编址；每次128位传输地址+8。AW/W独立握手，均完成才前进。
// 写数据从AW发起前即有效，满足本IP不晚于地址两拍的限制。wlast始终1，每地址一拍。
// 本IP的paxi_wstrb是高有效写屏蔽位（IPUG034表2-9），全0表示写全部字节。
// 输入rdata[127:0]/rvalid/rlast，输出rready；不接收未请求数据。
// 输出done/failed保持至复位，errors饱和计数；first_address/expected/observed记录首个错误。
// 无写响应，写阶段结束后等待1024拍再读；读写等待超时后failed，不无限挂起。
module paxi_memory_test #(
    parameter WORDS=4096,parameter [24:0] BASE_ADDR=0,parameter TIMEOUT=1000000
)(
    input wire clk,input wire rst_n,input wire ready,
    output wire [24:0] awaddr,output wire awvalid,input wire awready,
    output wire [127:0] wdata,output wire [15:0] wstrb,output wire wvalid,output wire wlast,input wire wready,
    output wire [24:0] araddr,output wire arvalid,input wire arready,
    input wire [127:0] rdata,input wire rvalid,input wire rlast,output wire rready,
    output reg done,output reg failed,output reg [31:0] errors,output reg [24:0] first_address,
    output reg [127:0] expected,output reg [127:0] observed,output wire [3:0] phase
);
    localparam WAIT_READY=0,WRITE=1,DRAIN=2,READ_CMD=3,READ_DATA=4,FINISH=5;
    reg [2:0] state;
    reg [15:0] index;
    reg aw_sent,w_sent,pass;
    reg [31:0] age;
    wire [24:0] address=BASE_ADDR+{index,3'b000};
    // 功能：生成与地址有关的四个不同32位字；a为地址，p为互补轮次，返回128位期望数据。
    function [127:0] pattern;
        input [24:0] a; input p; reg [31:0] v;
        begin v={7'b0,a}; pattern={v^32'hf00f55aa,~v^32'h13579bdf,v^32'h2468ace0,~v^32'hdeadbeef}^{128{p}}; end
    endfunction
    assign awaddr=address; assign araddr=address;
    assign awvalid=(state==WRITE && !aw_sent);
    assign wvalid=(state==WRITE && !w_sent);
    assign wdata=pattern(address,pass); assign wstrb=16'h0000; assign wlast=1'b1;
    assign arvalid=(state==READ_CMD); assign rready=(state==READ_DATA);
    assign phase={pass,state};
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=WAIT_READY; index<=0; aw_sent<=0; w_sent<=0; age<=0; pass<=0;
            done<=0; failed<=0; errors<=0; first_address<=0; expected<=0; observed<=0;
        end else if(state!=FINISH) begin
            if(age==TIMEOUT-1) begin failed<=1; done<=1; state<=FINISH; end
            else begin
                age<=age+1'b1;
                case(state)
                    WAIT_READY: if(ready) begin state<=WRITE;age<=0;end
                    WRITE: begin
                        if(awvalid && awready) aw_sent<=1;
                        if(wvalid && wready) w_sent<=1;
                        if((aw_sent || awready) && (w_sent || wready)) begin
                            age<=0; aw_sent<=0; w_sent<=0;
                            if(index==WORDS-1) begin index<=0;state<=DRAIN;end
                            else index<=index+1'b1;
                        end
                    end
                    DRAIN: if(age==1023) begin age<=0;state<=READ_CMD;end
                    READ_CMD: if(arready) begin age<=0;state<=READ_DATA;end
                    READ_DATA: if(rvalid) begin
                        age<=0;
                        if(rdata!=pattern(address,pass) || !rlast) begin
                            failed<=1;
                            if(!(&errors)) errors<=errors+1'b1;
                            if(errors==0) begin first_address<=address;expected<=pattern(address,pass);observed<=rdata;end
                        end
                        if(index==WORDS-1) begin
                            if(pass) begin done<=1;state<=FINISH;end
                            else begin pass<=1;index<=0;state<=WRITE;end
                        end else begin index<=index+1'b1;state<=READ_CMD;end
                    end
                    default: begin done<=1;failed<=1;state<=FINISH;end
                endcase
            end
        end
    end
endmodule
