`timescale 1ns/1ps
// 功能：按UG1316及TD原语独立连接DPHY0两路MIPI接收。未使用示例IP或其控制RTL。
// 控制域clk_50=50MHz，rst_n低有效；enable由控制域寄存器提供。当前要求传感器连续时钟模式。
// 时钟Lane进入HS后保持至enable关闭；数据Lane在LP11→LP01→LP00后切HS，包结束返回LP。
// LP/HS使能互斥，HS终端00，LP终端11；数据HS最长200us超时退出，避免失步后一直占用。
// 字节域byte_clk由硬核给出，data[15:0]低字节lane0高字节lane1；没有预设字节边界。
// packet_done_toggle为字节域翻转事件，内部两拍同步；hs_active输出控制域状态需消费域同步。
// lp_state[5:0]={clockP,clockN,lane1P,lane1N,lane0P,lane0N}是未同步硬核电平，外部不可直接计数。
module ph1p_mipi_rx(
    input wire clk_50,input wire rst_n,input wire enable,input wire packet_done_toggle,
    inout wire clock_p,inout wire clock_n,inout wire [3:0] data_p,inout wire [3:0] data_n,
    output wire byte_clk,output wire [15:0] data,output reg hs_active,output wire [5:0] lp_state,
    output reg [31:0] timeouts
);
    wire cp,cn,p0,n0,p1,n1;
    reg clock_hs,seen_stop,seen_prepare;
    reg [5:0] lp_sync0,lp_sync1;
    reg [1:0] done_sync;
    reg done_seen;
    reg [13:0] timer;
    wire [15:0] d0,d1;
    assign lp_state={cp,cn,p1,n1,p0,n0};
    assign data={d1[7:0],d0[7:0]};
    always @(posedge clk_50) begin
        if(!rst_n || !enable) begin
            lp_sync0<=6'h3f; lp_sync1<=6'h3f; done_sync<=0; done_seen<=0;
            clock_hs<=0; hs_active<=0; seen_stop<=0; seen_prepare<=0; timer<=0; timeouts<=0;
        end else begin
            lp_sync0<=lp_state; lp_sync1<=lp_sync0; done_sync<={done_sync[0],packet_done_toggle};
            done_seen<=done_sync[1];
            if(lp_sync1[5:4]==0) clock_hs<=1;
            if(!hs_active) begin
                timer<=0;
                if(lp_sync1[3:0]==4'hf) begin seen_stop<=1; seen_prepare<=0; end
                if(seen_stop && lp_sync1[1:0]==2'b01) seen_prepare<=1;
                if(seen_prepare && lp_sync1[3:0]==0) begin hs_active<=1; seen_stop<=0; seen_prepare<=0; end
            end else begin
                timer<=timer+1'b1;
                if(done_sync[1]!=done_seen) hs_active<=0;
                else if(timer==9999) begin hs_active<=0; timeouts<=timeouts+1'b1; end
            end
        end
    end
    PH1P_LOGIC_DPHY_MIPI_RX #(.DPHY_RX_LOCATION("DPHY0"),.HS_S2P_RATIO("1:8"),
        .CK_HS_VGA_GAIN("12dB"),.L0_HS_VGA_GAIN("12dB"),.L1_HS_VGA_GAIN("12dB")) phy(
        .io_clk_pad_p(clock_p),.io_clk_pad_n(clock_n),.io_data_pad_p(data_p),.io_data_pad_n(data_n),
        .o_fabric_div4_8_clk(byte_clk),.i_phy_rst_n(rst_n && enable),
        .i_ck_lp_rx_en(!clock_hs),.o_ck_lp_rx_p(cp),.o_ck_lp_rx_n(cn),
        .i_ck_hs_rx_en(clock_hs),.i_ck_hs_terminator_sel(clock_hs?2'b00:2'b11),
        .i_ck_hs_idelay_en(1'b0),.i_ck_hs_idelay_value(6'b0),
        .i_l0_lp_rx_en(!hs_active),.i_l0_lp_tx_en(1'b0),.i_l0_lp_tx_p(1'b0),.i_l0_lp_tx_n(1'b0),
        .o_l0_lp_rx_p(p0),.o_l0_lp_rx_n(n0),.i_l0_hs_rx_en(hs_active),
        .i_l0_hs_terminator_sel(hs_active?2'b00:2'b11),.i_l0_hs_slave_deser_clk_en(1'b0),
        .i_l0_hs_idelay_en(1'b0),.i_l0_hs_idelay_value(6'b0),.o_l0_hs_rx_data(d0),.o_l0_hs_rx_slave_data(),
        .i_l1_lp_rx_en(!hs_active),.i_l1_lp_tx_en(1'b0),.i_l1_lp_tx_p(1'b0),.i_l1_lp_tx_n(1'b0),
        .o_l1_lp_rx_p(p1),.o_l1_lp_rx_n(n1),.i_l1_hs_rx_en(hs_active),
        .i_l1_hs_terminator_sel(hs_active?2'b00:2'b11),.i_l1_hs_slave_deser_clk_en(1'b0),
        .i_l1_hs_idelay_en(1'b0),.i_l1_hs_idelay_value(6'b0),.o_l1_hs_rx_data(d1),.o_l1_hs_rx_slave_data(),
        .i_l2_lp_rx_en(1'b0),.i_l2_lp_tx_en(1'b0),.i_l2_lp_tx_p(1'b0),.i_l2_lp_tx_n(1'b0),
        .i_l2_hs_rx_en(1'b0),.i_l2_hs_terminator_sel(2'b11),.i_l2_hs_slave_deser_clk_en(1'b0),
        .i_l2_hs_idelay_en(1'b0),.i_l2_hs_idelay_value(6'b0),.o_l2_lp_rx_p(),.o_l2_lp_rx_n(),
        .o_l2_hs_rx_data(),.o_l2_hs_rx_slave_data(),
        .i_l3_lp_rx_en(1'b0),.i_l3_lp_tx_en(1'b0),.i_l3_lp_tx_p(1'b0),.i_l3_lp_tx_n(1'b0),
        .i_l3_hs_rx_en(1'b0),.i_l3_hs_terminator_sel(2'b11),.i_l3_hs_slave_deser_clk_en(1'b0),
        .i_l3_hs_idelay_en(1'b0),.i_l3_hs_idelay_value(6'b0),.o_l3_lp_rx_p(),.o_l3_lp_rx_n(),
        .o_l3_hs_rx_data(),.o_l3_hs_rx_slave_data());
endmodule
