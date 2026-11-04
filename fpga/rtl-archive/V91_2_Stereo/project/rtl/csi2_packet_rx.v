`timescale 1ns/1ps
// 功能：接收已按lane0/lane1合并的CSI-2字节对，核对24位包头ECC与载荷CRC16。
// 参数：MAX_BYTES=1..65535，允许的长包上限；不执行ECC纠错，任何包头错误均丢弃。
// 时钟：clk为MIPI字节时钟；rst_n同步低有效。输入start为1拍，仅在一包首字节前置1；
// start与valid可同拍。bytes[7:0]先到、[15:8]后到；keep只允许01/11，valid=0忽略。
// 输出payload[15:0]/payload_keep逐拍标记原始载荷；header_valid、packet_done、frame_start/
// frame_end为1拍。data_type[5:0]、virtual_channel[1:0]、word_count[15:0]在头有效后保持。
// packet_good只在packet_done有效；payload是暂存数据，必须等CRC通过才能提交给帧缓冲。
// abort优先取消当前包并输出失败，避免截断包成为有效图像。无背压，处理能力2字节/拍。
// 延迟：载荷输入到输出1拍，头4字节后报告，CRC末字节后报告完成。协议依据见docs/protocol_sources.md。
module csi2_packet_rx #(parameter MAX_BYTES=8192)(
    input wire clk,input wire rst_n,input wire start,input wire abort,
    input wire valid,input wire [1:0] keep,input wire [15:0] bytes,
    output reg [15:0] payload,output reg [1:0] payload_keep,
    output reg header_valid,output reg [5:0] data_type,output reg [1:0] virtual_channel,
    output reg [15:0] word_count,output reg frame_start,output reg frame_end,
    output reg packet_done,output reg packet_good,output reg ecc_error,output reg crc_error
);
    localparam IDLE=0,HEADER=1,DATA=2,CRC_LO=3,CRC_HI=4;
    reg [2:0] state,next_state;
    reg [1:0] header_index,next_header_index;
    reg [23:0] header,next_header;
    reg [15:0] remaining,next_remaining,crc,next_crc;
    reg [7:0] received_crc_lo,next_received_crc_lo;
    reg [15:0] out_payload;
    reg [1:0] out_keep;
    reg out_header,out_done,out_good,out_ecc,out_crc,out_fs,out_fe;
    reg [5:0] next_type;
    reg [1:0] next_vc;
    reg [15:0] next_wc;
    reg [7:0] current_byte;
    integer lane;
    // 功能：对一个字节更新CSI-2反射CRC，参数c为前值、b为传输字节；返回16位CRC。
    function [15:0] crc_byte;
        input [15:0] c; input [7:0] b;
        reg [15:0] r; integer bit_no;
        begin r=c; for(bit_no=0;bit_no<8;bit_no=bit_no+1)
            r=(r>>1)^((r[0]^b[bit_no])?16'h8408:16'h0000);
            crc_byte=r;
        end
    endfunction
    // 功能：24位包头的六位ECC；参数h低字节为DI，返回ECC[5:0]，保留位由调用者核对。
    function [5:0] header_ecc;
        input [23:0] h;
        begin
            header_ecc[0]=^(h & 24'hf12cb7);
            header_ecc[1]=^(h & 24'hf2555b);
            header_ecc[2]=^(h & 24'h749a6d);
            header_ecc[3]=^(h & 24'hb8e38e);
            header_ecc[4]=^(h & 24'hdf03f0);
            header_ecc[5]=^(h & 24'heffc00);
        end
    endfunction
    always @* begin
        next_state=state; next_header=header; next_header_index=header_index;
        next_remaining=remaining; next_crc=crc; next_received_crc_lo=received_crc_lo;
        next_type=data_type; next_vc=virtual_channel; next_wc=word_count;
        out_payload=0; out_keep=0; out_header=0; out_done=0; out_good=0;
        out_ecc=0; out_crc=0; out_fs=0; out_fe=0; current_byte=0;
        if(start) begin next_state=HEADER; next_header=0; next_header_index=0; next_crc=16'hffff; end
        for(lane=0;lane<2;lane=lane+1) begin
            current_byte=(lane==0)?bytes[7:0]:bytes[15:8];
            if(valid && keep[lane]) begin
                case(next_state)
                    HEADER: begin
                        case(next_header_index)
                            0: next_header[7:0]=current_byte;
                            1: next_header[15:8]=current_byte;
                            2: next_header[23:16]=current_byte;
                            3: begin
                                if(current_byte!={2'b0,header_ecc(next_header)}) begin
                                    out_ecc=1; out_done=1; next_state=IDLE;
                                end else begin
                                    next_type=next_header[5:0]; next_vc=next_header[7:6];
                                    next_wc=next_header[23:8]; out_header=1;
                                    if(next_header[5:0]<=6'h0f) begin
                                        out_fs=(next_header[5:0]==0); out_fe=(next_header[5:0]==1);
                                        out_done=1; out_good=1; next_state=IDLE;
                                    end else if(next_header[23:8]>MAX_BYTES) begin out_done=1; next_state=IDLE; end
                                    else begin
                                        next_remaining=next_header[23:8]; next_crc=16'hffff;
                                        next_state=(next_header[23:8]==0)?CRC_LO:DATA;
                                    end
                                end
                            end
                        endcase
                        next_header_index=next_header_index+1'b1;
                    end
                    DATA: begin
                        out_payload[lane*8 +: 8]=current_byte; out_keep[lane]=1;
                        next_crc=crc_byte(next_crc,current_byte); next_remaining=next_remaining-1'b1;
                        if(next_remaining==0) next_state=CRC_LO;
                    end
                    CRC_LO: begin next_received_crc_lo=current_byte; next_state=CRC_HI; end
                    CRC_HI: begin
                        out_done=1; out_good=({current_byte,next_received_crc_lo}==next_crc);
                        out_crc=!out_good; next_state=IDLE;
                    end
                    default: next_state=IDLE;
                endcase
            end
        end
        if(abort) begin next_state=IDLE; out_keep=0; out_header=0; out_fs=0; out_fe=0;
            out_done=(state!=IDLE); out_good=0; end
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE; header<=0; header_index<=0; remaining<=0; crc<=16'hffff; received_crc_lo<=0;
            payload<=0; payload_keep<=0; header_valid<=0; data_type<=0; virtual_channel<=0; word_count<=0;
            packet_done<=0; packet_good<=0; ecc_error<=0; crc_error<=0; frame_start<=0; frame_end<=0;
        end else begin
            state<=next_state; header<=next_header; header_index<=next_header_index;
            remaining<=next_remaining; crc<=next_crc; received_crc_lo<=next_received_crc_lo;
            payload<=out_payload; payload_keep<=out_keep; header_valid<=out_header;
            data_type<=next_type; virtual_channel<=next_vc; word_count<=next_wc;
            packet_done<=out_done; packet_good<=out_good; ecc_error<=out_ecc; crc_error<=out_crc;
            frame_start<=out_fs; frame_end<=out_fe;
        end
    end
endmodule
