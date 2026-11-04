`timescale 1ns/1ps
// 功能：按UG1310例化PH1P35的PLL_V2原语，自行计算分频，不引用示例生成IP。
// 输入clk_50为原理图C4上的50MHz；reset高有效。输出pixel=74MHz、serial=370MHz、
// camera=19.2MHz；lock_video/lock_camera异步于消费域，必须同步后用于状态/复位释放。
// 参数：无。CLKC1输出连接fbclk反馈：视频VCO=(50/5*37)*4=1480MHz；相机VCO=(50/5*24)*4=960MHz。
// 像素/串行时钟来自同一PLL，比例1:5；摄像头时钟来自另一PLL，960MHz/50=19.2MHz。
// 环路参数由本机官方PLLF6F1D0P0_1.0/Dependency.tcl的Calc_BandWidth计算，见scripts/calculate_pll.tcl。
module ph1p_clocks(
    input wire clk_50,input wire reset,
    output wire pixel,output wire serial,output wire camera,
    output wire lock_video,output wire lock_camera
);
    wire [6:0] video_clocks,camera_clocks;
    wire camera_feedback;
    // 官方生成器NORMAL反馈模式要求经过BUFG后回送，不能把PLL原始输出直接短接fbclk。
    PH1P_LOGIC_BUFG video_feedback_buffer(.i(video_clocks[1]),.o(serial));
    PH1P_LOGIC_BUFG camera_feedback_buffer(.i(camera_clocks[1]),.o(camera_feedback));
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
    PH1P_PHY_PLL_V2 #(
        .FIN("50.0000"),.FBKCLK("CLKC1_EXT"),.FBKCLK_INT("CLKC1_EXT"),.PLL_FEED_TYPE("EXTERNAL"),
        .LPF_CAP(16),.ICP_CUR(0),.LPF_RES(3),.PLL_FASTLOOP("DISABLE"),
        .REFCLK_DIV(5),.FBCLK_DIV(24),
        .CLKC0_DIV(50),.CLKC0_CPHASE(49),.CLKC0_DUTY_INT(25),.CLKC0_ENABLE("ENABLE"),
        .CLKC1_DIV(4),.CLKC1_CPHASE(3),.CLKC1_DUTY_INT(2),.CLKC1_ENABLE("ENABLE")
    ) camera_pll(
        .refclk(clk_50),.refclk_rst(1'b0),.fbclk(camera_feedback),.clkc(camera_clocks),.clkcb(),
        .lock(lock_camera),.pllreset(reset),.clkc_rst(2'b0),.clkc_en(7'h03),
        .wakeup(1'b1),.cps_step(1'b0),.pllpd(1'b0),
        .drp_clk(1'b0),.drp_rstn(1'b1),.drp_sel(1'b0),.drp_rd(1'b0),.drp_wr(1'b0),
        .drp_addr(8'b0),.drp_wdata(8'b0),.drp_err(),.drp_rdy(),.drp_rdata(),
        .psdone(),.psclk(1'b0),.psdown(1'b0),.psstep(1'b0),.psclksel(3'b0),
        .ssc_reset(1'b0),.scan_clk(1'b1),.scan_enable(1'b1),.scan_mode(1'b0),.scan_resetn(1'b1));
    assign pixel=video_clocks[0];
    assign camera=camera_clocks[0];
endmodule
