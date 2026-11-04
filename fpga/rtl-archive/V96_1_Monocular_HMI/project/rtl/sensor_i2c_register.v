`timescale 1ns/1ps
// 功能：单主机I2C寄存器事务，7位设备地址、16位寄存器地址、8位数据。
// 参数：REG_ADDR_BYTES为1或2（DDC为1，相机为2）；HALF_CYCLES>=4，半周期长度（clk周期）；TIMEOUT_CYCLES>
// 100*HALF_CYCLES，总事务时间上限（clk周期），包含等待总线及拉伸。
// 时钟/复位：clk域，同步低有效rst_n；scl_in/sda_in内部两级同步。
// 输入：cmd_valid/read 1位、dev_addr 7位、reg_addr 16位、write_data 8位。
// 握手：cmd_valid&&cmd_ready上升沿锁存请求；busy期间忽略新请求。
// 输出：done 1拍完成脉冲；error 2位：0成功、1从机NACK、2总线/事务超时；
// read_data 8位只在读事务done且error=0时有效。结果保持到下一次请求。
// 物理端口：scl_low/sda_low为1时仅拉低，0时必须在顶层释放为Z，禁止推高。
// 支持重复START和时钟拉伸；不支持多主机仲裁；超时释放线路，不自动清总线。
// 延迟：依赖事务长度和拉伸，不固定；最长TIMEOUT_CYCLES周期内完成。
module sensor_i2c_register #(
    parameter HALF_CYCLES=250, TIMEOUT_CYCLES=1000000, REG_ADDR_BYTES=2
)(
    input wire clk, input wire rst_n,
    input wire cmd_valid, output wire cmd_ready, input wire read,
    input wire [6:0] dev_addr, input wire [15:0] reg_addr,
    input wire [7:0] write_data,
    output reg busy, output reg done, output reg [1:0] error,
    output reg [7:0] read_data,
    input wire scl_in, input wire sda_in,
    output reg scl_low, output reg sda_low
);
    localparam IDLE=0, START_FREE=1, START_HOLD=2, TX_LOW=3, TX_HIGH=4,
        ACK_LOW=5, ACK_HIGH=6, RESTART_LOW=7, RESTART_HIGH=8,
        RX_LOW=9, RX_HIGH=10, NACK_LOW=11, NACK_HIGH=12,
        STOP_LOW=13, STOP_HIGH=14, STOP_FREE=15;
    reg [3:0] state;
    reg [1:0] scl_sync, sda_sync;
    reg [31:0] phase_count, elapsed;
    reg read_latched;
    reg [6:0] device_latched;
    reg [15:0] address_latched;
    reg [7:0] data_latched, tx_byte, rx_byte;
    reg [2:0] bit_index;
    reg [1:0] byte_index;
    wire wait_line=(!scl_low && !scl_sync[1]) ||
        ((state==START_FREE || state==STOP_FREE) && !sda_sync[1]);
    assign cmd_ready=!busy;
    always @(posedge clk) begin
        if (!rst_n) begin scl_sync<=2'b11; sda_sync<=2'b11; end
        else begin scl_sync<={scl_sync[0],scl_in}; sda_sync<={sda_sync[0],sda_in}; end
    end
    always @* begin
        scl_low=0; sda_low=0;
        case (state)
            START_HOLD: sda_low=1;
            TX_LOW, TX_HIGH: begin scl_low=(state==TX_LOW); sda_low=!tx_byte[bit_index]; end
            ACK_LOW, RESTART_LOW, RX_LOW, NACK_LOW: scl_low=1;
            STOP_LOW: begin scl_low=1; sda_low=1; end
            STOP_HIGH: sda_low=1;
            default: begin scl_low=0; sda_low=0; end
        endcase
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            state<=IDLE; busy<=0; done<=0; error<=0; read_data<=0;
            phase_count<=0; elapsed<=0; read_latched<=0; device_latched<=0;
            address_latched<=0; data_latched<=0; tx_byte<=0; rx_byte<=0;
            bit_index<=7; byte_index<=0;
        end else begin
            done<=0;
            if (!busy) begin
                if (cmd_valid) begin
                    busy<=1; error<=0; read_data<=0; elapsed<=0; phase_count<=0;
                    read_latched<=read; device_latched<=dev_addr;
                    address_latched<=reg_addr; data_latched<=write_data;
                    tx_byte<={dev_addr,1'b0}; byte_index<=0; bit_index<=7;
                    state<=START_FREE;
                end
            end else if (elapsed==TIMEOUT_CYCLES-1) begin
                state<=IDLE; busy<=0; done<=1; error<=2; phase_count<=0;
            end else begin
                elapsed<=elapsed+1'b1;
                // SCL真正变高后重新计满高电平时间，兼容从机拉伸及同步延迟。
                if (wait_line) phase_count<=0;
                else if (phase_count<HALF_CYCLES-1) phase_count<=phase_count+1'b1;
                else begin
                    phase_count<=0;
                    case (state)
                        START_FREE: state<=START_HOLD;
                        START_HOLD: state<=TX_LOW;
                        TX_LOW: state<=TX_HIGH;
                        TX_HIGH: begin
                            if (bit_index==0) state<=ACK_LOW;
                            else begin bit_index<=bit_index-1'b1; state<=TX_LOW; end
                        end
                        ACK_LOW: state<=ACK_HIGH;
                        ACK_HIGH: begin
                            if (sda_sync[1]) begin error<=1; state<=STOP_LOW; end
                            else begin
                                bit_index<=7;
                                case (byte_index)
                                    0: begin
                                        byte_index<=(REG_ADDR_BYTES==1)?2:1;
                                        tx_byte<=(REG_ADDR_BYTES==1)?address_latched[7:0]:address_latched[15:8];
                                        state<=TX_LOW;
                                    end
                                    1: begin byte_index<=2; tx_byte<=address_latched[7:0]; state<=TX_LOW; end
                                    2: begin
                                        byte_index<=3;
                                        if (read_latched) begin tx_byte<={device_latched,1'b1}; state<=RESTART_LOW; end
                                        else begin tx_byte<=data_latched; state<=TX_LOW; end
                                    end
                                    3: begin
                                        if (read_latched) begin rx_byte<=0; state<=RX_LOW; end
                                        else state<=STOP_LOW;
                                    end
                                endcase
                            end
                        end
                        RESTART_LOW: state<=RESTART_HIGH;
                        RESTART_HIGH: state<=START_HOLD;
                        RX_LOW: state<=RX_HIGH;
                        RX_HIGH: begin
                            rx_byte<={rx_byte[6:0],sda_sync[1]};
                            if (bit_index==0) begin
                                read_data<={rx_byte[6:0],sda_sync[1]}; state<=NACK_LOW;
                            end else begin bit_index<=bit_index-1'b1; state<=RX_LOW; end
                        end
                        NACK_LOW: state<=NACK_HIGH;
                        NACK_HIGH: state<=STOP_LOW;
                        STOP_LOW: state<=STOP_HIGH;
                        STOP_HIGH: state<=STOP_FREE;
                        STOP_FREE: begin busy<=0; done<=1; state<=IDLE; end
                        default: begin busy<=0; done<=1; error<=2; state<=IDLE; end
                    endcase
                end
            end
        end
    end
endmodule
