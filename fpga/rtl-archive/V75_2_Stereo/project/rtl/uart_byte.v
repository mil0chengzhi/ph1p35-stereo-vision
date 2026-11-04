`timescale 1ns/1ps
// 功能：UART 8N1逐字节收发。参数DIV>=8，为clk频率/波特率四舍五入后的周期数。
// clk单时钟域，rst_n同步低有效；rx异步输入内部两级同步；tx空闲高。
// tx_data 8位，在tx_valid&&tx_ready边沿接收；rx_data 8位在rx_valid一拍脉冲有效。
// framing_error为停止位错误一拍脉冲，坏字节不发rx_valid。无接收背压。
// 收发独立；接收先确认起始位中心，再每DIV周期采样，停止位后可连续接收。
module uart_byte #(parameter DIV=50)(
    input wire clk,input wire rst_n,input wire rx,output reg tx,
    input wire [7:0] tx_data,input wire tx_valid,output wire tx_ready,
    output reg [7:0] rx_data,output reg rx_valid,output reg framing_error
);
    reg [1:0] rx_sync;
    reg [1:0] rx_state;
    reg [15:0] rx_count,tx_count;
    reg [3:0] rx_bit,tx_bit;
    reg [7:0] rx_shift;
    reg [9:0] tx_shift;
    reg tx_busy;
    assign tx_ready=!tx_busy;
    always @(posedge clk) begin
        if(!rst_n) rx_sync<=3; else rx_sync<={rx_sync[0],rx};
        if(!rst_n) begin
            tx<=1; tx_busy<=0; tx_count<=0; tx_bit<=0; tx_shift<=10'h3ff;
            rx_state<=0; rx_count<=0; rx_bit<=0; rx_shift<=0;
            rx_data<=0; rx_valid<=0; framing_error<=0;
        end else begin
            rx_valid<=0; framing_error<=0;
            if(!tx_busy) begin
                if(tx_valid) begin
                    tx_shift<={1'b1,tx_data,1'b0}; tx<=0; tx_bit<=0;
                    tx_count<=DIV-1; tx_busy<=1;
                end
            end else if(tx_count!=0) tx_count<=tx_count-1'b1;
            else if(tx_bit==9) begin tx_busy<=0; tx<=1; end
            else begin
                tx_shift<={1'b1,tx_shift[9:1]}; tx<=tx_shift[1];
                tx_bit<=tx_bit+1'b1; tx_count<=DIV-1;
            end
            case(rx_state)
                0: if(!rx_sync[1]) begin rx_state<=1; rx_count<=DIV/2-1; end
                1: if(rx_count!=0) rx_count<=rx_count-1'b1;
                   else if(rx_sync[1]) rx_state<=0;
                   else begin rx_state<=2; rx_count<=DIV-1; rx_bit<=0; end
                2: if(rx_count!=0) rx_count<=rx_count-1'b1;
                   else begin
                       rx_shift[rx_bit]<=rx_sync[1]; rx_count<=DIV-1;
                       if(rx_bit==7) rx_state<=3; else rx_bit<=rx_bit+1'b1;
                   end
                3: if(rx_count!=0) rx_count<=rx_count-1'b1;
                   else begin
                       rx_state<=0;
                       if(rx_sync[1]) begin rx_data<=rx_shift; rx_valid<=1; end
                       else framing_error<=1;
                   end
            endcase
        end
    end
endmodule
