`timescale 1ns/1ps
// Video-only copy of qualified PLL: 74MHz pixel /370MHz serializer, no B camera PLL.
module ph1p_display_clocks(input wire clk_50,reset,output wire pixel,serial,output wire lock_video);
 wire [6:0] video_clocks;
 PH1P_LOGIC_BUFG video_feedback_buffer(.i(video_clocks[1]),.o(serial));
    PH1P_PHY_PLL_V2 #(
        .FIN("50.0000"),.FBKCLK("CLKC1_EXT"),.FBKCLK_INT("CLKC1_EXT"),.PLL_FEED_TYPE("EXTERNAL"),
        .LPF_CAP(28),.ICP_CUR(0),.LPF_RES(3),.PLL_FASTLOOP("DISABLE"),
        .REFCLK_DIV(5),.FBCLK_DIV(37),
        .CLKC0_DIV(20),.CLKC0_CPHASE(19),.CLKC0_DUTY_INT(10),.CLKC0_ENABLE("ENABLE"),
        .CLKC1_DIV(4),.CLKC1_CPHASE(3),.CLKC1_DUTY_INT(2),.CLKC1_ENABLE("ENABLE")
    ) video_pll(
        .refclk(clk_50),.refclk_rst(1'b0),.fbclk(serial),.clkc(video_clocks),.clkcb(),
        .lock(lock_video),.pllreset(reset),.clkc_rst(2'b0),.clkc_en(7'h03),
        .wakeup(1'b1),.cps_step(1'b0),.pllpd(1'b0),
        .drp_clk(1'b0),.drp_rstn(1'b1),.drp_sel(1'b0),.drp_rd(1'b0),.drp_wr(1'b0),
        .drp_addr(8'b0),.drp_wdata(8'b0),.drp_err(),.drp_rdy(),.drp_rdata(),
        .psdone(),.psclk(1'b0),.psdown(1'b0),.psstep(1'b0),.psclksel(3'b0),
        .ssc_reset(1'b0),.scan_clk(1'b1),.scan_enable(1'b1),.scan_mode(1'b0),.scan_resetn(1'b1));
 assign pixel=video_clocks[0];
endmodule
