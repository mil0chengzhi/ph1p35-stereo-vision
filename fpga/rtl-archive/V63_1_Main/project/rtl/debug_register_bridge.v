`timescale 1ns/1ps
// 功能：UART调试帧到相机/DDC寄存器事务桥接；clk域，rst_n同步低有效。
// 参数CLOCK_HZ>=1000；输入字节rx_data[7:0]/rx_valid，无背压；tx接口valid/ready。
// 请求8字节：A5,操作,序号,地址高,地址低,数据,设备地址,前7字节异或。
// 操作01读状态；02调试地址读；03调试地址写；10相机读；11相机写；20 DDC读。
// 30调试地址连续读：请求数据/设备字段组成16位大端长度(1..65535)；应答后紧跟原始字节及CRC16低/高。
// 调试读地址经reg_addr输出，允许probe_data有1拍读延迟。PC必须逐次请求/应答。
// 应答8字节：5A,操作,序号,错误码,数据,状态低,状态高,前7字节异或。
// 错误码0成功/1NACK/2总线超时/3非法操作/4校验错；收包20ms超时丢弃半包。
// i2c_valid为1拍请求；等待done后回包。接口相机和DDC共享地址数据，独立valid/done。
// status[15:0]需已同步至clk域；返回时锁存。不支持应答期间排队，不自动重发写操作。
module debug_register_bridge #(parameter CLOCK_HZ=50000000)(
    input wire clk,input wire rst_n,
    output wire busy,
    input wire [7:0] rx_data,input wire rx_valid,
    output reg [7:0] tx_data,output wire tx_valid,input wire tx_ready,
    input wire [15:0] status,
    input wire [7:0] probe_data,output reg probe_write,
    output reg cam_valid,output reg ddc_valid,output reg read,
    output reg [6:0] dev_addr,output reg [15:0] reg_addr,output reg [7:0] write_data,
    input wire cam_done,input wire [1:0] cam_error,input wire [7:0] cam_data,
    input wire ddc_done,input wire [1:0] ddc_error,input wire [7:0] ddc_data
);
    localparam RECEIVE=0,DISPATCH=1,WAIT_CAM=2,WAIT_DDC=3,PREPARE=4,SEND=5,
        PROBE_WAIT=6,PROBE_READ=7,BURST_SEND=8,BURST_CRC0=9,BURST_CRC1=10;
    reg [3:0] state;
    reg [2:0] index,send_index;
    reg burst;
    reg [15:0] burst_left,burst_crc;
    reg [7:0] request[0:6];
    reg [7:0] response[0:7];
    reg [7:0] checksum,result_error,result_data;
    reg [31:0] age;
    integer n;
    assign tx_valid=(state==SEND || state==BURST_SEND || state==BURST_CRC0 || state==BURST_CRC1);
    assign busy=(state!=RECEIVE)||(index!=0);
    always @* begin
        case(state)
            BURST_SEND: tx_data=result_data;
            BURST_CRC0: tx_data=burst_crc[7:0];
            BURST_CRC1: tx_data=burst_crc[15:8];
            default: tx_data=response[send_index];
        endcase
    end
    // 功能：调试数据CRC16，参数c为前值、b为字节；返回CSI-2同多项式的校验值。
    function [15:0] update_crc;
        input [15:0] c; input [7:0] b; reg [15:0] r; integer i;
        begin r=c; for(i=0;i<8;i=i+1) r=(r>>1)^((r[0]^b[i])?16'h8408:16'h0000); update_crc=r; end
    endfunction
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=RECEIVE; index<=0; send_index<=0; checksum<=0; age<=0;
            cam_valid<=0; ddc_valid<=0; read<=0; dev_addr<=0; reg_addr<=0; write_data<=0;
            result_error<=0; result_data<=0;
            probe_write<=0; burst<=0; burst_left<=0; burst_crc<=16'hffff;
            for(n=0;n<7;n=n+1) request[n]<=0;
            for(n=0;n<8;n=n+1) response[n]<=0;
        end else begin
            cam_valid<=0; ddc_valid<=0; probe_write<=0;
            case(state)
                RECEIVE: begin
                    if(index!=0) begin
                        if(age==CLOCK_HZ/50-1) begin index<=0; age<=0; end
                        else age<=age+1'b1;
                    end
                    if(rx_valid) begin
                        age<=0;
                        if(index==0) begin
                            if(rx_data==8'ha5) begin request[0]<=rx_data; checksum<=rx_data; index<=1; end
                        end else if(index==7) begin
                            index<=0; result_data<=0;
                            if(rx_data!=checksum) begin result_error<=4; state<=PREPARE; end
                            else begin result_error<=0; state<=DISPATCH; end
                        end else begin
                            request[index]<=rx_data; checksum<=checksum^rx_data; index<=index+1'b1;
                        end
                    end
                end
                DISPATCH: begin
                    burst<=0;
                    reg_addr<={request[3],request[4]}; write_data<=request[5];
                    dev_addr<=request[6][6:0]; read<=(request[1]!=8'h11);
                    case(request[1])
                        8'h01: begin result_data<=8'h01; state<=PREPARE; end
                        8'h02: state<=PROBE_WAIT;
                        8'h03: begin probe_write<=1; result_data<=request[5]; state<=PREPARE; end
                        8'h30: begin
                            burst_left<={request[5],request[6]}; burst_crc<=16'hffff;
                            if({request[5],request[6]}==0) result_error<=3;
                            else burst<=1;
                            state<=PREPARE;
                        end
                        8'h10,8'h11: begin cam_valid<=1; state<=WAIT_CAM; end
                        8'h20: begin ddc_valid<=1; state<=WAIT_DDC; end
                        default: begin result_error<=3; state<=PREPARE; end
                    endcase
                end
                WAIT_CAM: if(cam_done) begin result_data<=cam_data; result_error<={6'b0,cam_error}; state<=PREPARE; end
                WAIT_DDC: if(ddc_done) begin result_data<=ddc_data; result_error<={6'b0,ddc_error}; state<=PREPARE; end
                PREPARE: begin
                    response[0]<=8'h5a; response[1]<=request[1]; response[2]<=request[2];
                    response[3]<=result_error; response[4]<=result_data;
                    response[5]<=status[7:0]; response[6]<=status[15:8];
                    response[7]<=8'h5a^request[1]^request[2]^result_error^result_data^status[7:0]^status[15:8];
                    send_index<=0; state<=SEND;
                end
                SEND: if(tx_ready) begin
                    if(send_index==7) state<=burst?PROBE_WAIT:RECEIVE; else send_index<=send_index+1'b1;
                end
                PROBE_WAIT: state<=PROBE_READ;
                PROBE_READ: begin result_data<=probe_data; state<=burst?BURST_SEND:PREPARE; end
                BURST_SEND: if(tx_ready) begin
                    burst_crc<=update_crc(burst_crc,result_data);
                    burst_left<=burst_left-1'b1; reg_addr<=reg_addr+1'b1;
                    state<=(burst_left==1)?BURST_CRC0:PROBE_WAIT;
                end
                BURST_CRC0: if(tx_ready) state<=BURST_CRC1;
                BURST_CRC1: if(tx_ready) begin burst<=0; state<=RECEIVE; end
                default: state<=RECEIVE;
            endcase
        end
    end
endmodule
