// HX1P35A/PH1P35MDG324 + YT8511C, 1G-only GMII/RGMII bridge.
// Primitive usage derives from vendor 33_ethernet_udp_led/uisrc/03_ip/
// uiudp_stack/rgmii_interface.v (MiLianKe Electronic Technology, 2022/12/23).
// This wrapper intentionally uses ONE RX BUFG for IDDR and its byte registers,
// unlike the vendor BUFG->LCLK crossing. P&R must still prove IO setup/hold.
// TXC: PLL +90 degrees = +2ns; no ADC OUTDEL. YT8511 delay readback must
// equal default5. Its p85 receiver setup/hold requirements are at external
// GTX_CLK/TXD pins; do not add its internal delay again. See ethernet.sdc.
module db_rgmii_phy #(
    parameter integer RESET_CYCLES = 5000000,
    parameter integer SETTLE_CYCLES = 50000,
    // Native PH1P functional model: 60ps + 40ps/tap, i.e. 860ps nominal.
    // Routed slow/fast reports, NOT this nominal equation, qualify timing.
    parameter [4:0] RX_DELAY_TAPS = 5'd20,
    parameter [4:0] PHY_ADDR = 5'd4
) (
    input wire clk_50, input wire rst_n,
    output wire tx_clk, output wire tx_rst_n,
    output wire rx_clk, output wire rx_rst_n,
    input wire [7:0] gmii_txd, input wire gmii_tx_en,
    output reg [7:0] gmii_rxd,
    output reg gmii_rx_dv, output reg gmii_rx_er,
    input wire eth_rxc, input wire eth_rx_ctl, input wire [3:0] eth_rxd,
    output wire eth_txc, output wire eth_tx_ctl, output wire [3:0] eth_txd,
    output reg eth_reset_n, output wire phy_ready,
    output wire eth_mdc, inout wire eth_mdio,
    output wire [31:0] phy_id32, output wire [15:0] phy_status16,
    output wire [15:0] phy_delay16,
    output wire mdio_done, output wire mdio_error
);
    localparam integer RESET_BITS = $clog2(RESET_CYCLES + SETTLE_CYCLES + 1);
    wire pll_locked, tx_clk_shifted;
    wire pll_125_raw, pll_125_shifted_raw;
    wire unused_clk25;
    // C0 already includes a feedback BUFG in db_eth_pll.
    db_eth_pll eth_pll (.refclk(clk_50), .reset(!rst_n),
        .clk0_out(pll_125_raw), .clk1_out(pll_125_shifted_raw),
        .clk2_out(unused_clk25), .lock(pll_locked));
    assign tx_clk = pll_125_raw;
    PH1P_LOGIC_BUFG txc_buf (.i(pll_125_shifted_raw), .o(tx_clk_shifted));
    PH1P_LOGIC_BUFG rx_buf (.i(eth_rxc), .o(rx_clk));

    (* ASYNC_REG = "TRUE" *) reg [1:0] lock_sync;
    reg [RESET_BITS-1:0] reset_count;
    reg mdio_rst_n;
    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin lock_sync <= 0; reset_count <= 0;
            eth_reset_n <= 0; mdio_rst_n <= 0; end
        else begin
            lock_sync <= {lock_sync[0], pll_locked};
            if (!lock_sync[1]) begin reset_count <= 0; eth_reset_n <= 0; mdio_rst_n <= 0; end
            else if (reset_count < RESET_CYCLES + SETTLE_CYCLES) begin
                reset_count <= reset_count + 1'b1;
                if (reset_count == RESET_CYCLES-1) eth_reset_n <= 1;
                if (reset_count == RESET_CYCLES + SETTLE_CYCLES-1) mdio_rst_n <= 1;
            end
        end
    end
    wire mdio_phy_ready;
    db_phy_mdio #(.PHY_ADDR(PHY_ADDR)) phy_mdio (
        .clk_50(clk_50), .rst_n(mdio_rst_n), .eth_mdc(eth_mdc), .eth_mdio(eth_mdio),
        .phy_id32(phy_id32), .phy_status16(phy_status16), .phy_delay16(phy_delay16),
        .mdio_done(mdio_done), .mdio_error(mdio_error), .phy_ready(mdio_phy_ready));

    // Public ready and MAC domain reset MUST have identical qualification.
    // Async assertion catches even a brief PLL drop; two clk_50 release stages
    // keep ready low long enough for the system-domain link state to observe it.
    wire raw_ready = rst_n && pll_locked && mdio_phy_ready;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ready_sync;
    always @(posedge clk_50 or negedge raw_ready)
        if (!raw_ready) ready_sync <= 0;
        else ready_sync <= {ready_sync[0], 1'b1};
    assign phy_ready = ready_sync[1];

    wire domain_enable = phy_ready;
    (* ASYNC_REG = "TRUE" *) reg [2:0] tx_reset_sync, rx_reset_sync, txc_reset_sync;
    always @(posedge tx_clk or negedge domain_enable)
        if (!domain_enable) tx_reset_sync <= 0;
        else tx_reset_sync <= {tx_reset_sync[1:0], 1'b1};
    always @(posedge rx_clk or negedge domain_enable)
        if (!domain_enable) rx_reset_sync <= 0;
        else rx_reset_sync <= {rx_reset_sync[1:0], 1'b1};
    // Keep TXC running during autonegotiation. Only PLL/reset controls clock ODDR.
    wire txc_enable = rst_n && pll_locked;
    always @(posedge tx_clk_shifted or negedge txc_enable)
        if (!txc_enable) txc_reset_sync <= 0;
        else txc_reset_sync <= {txc_reset_sync[1:0], 1'b1};
    assign tx_rst_n = tx_reset_sync[2];
    assign rx_rst_n = rx_reset_sync[2];

    PH1P_LOGIC_HD_ODDR tx_clock_oddr (.q(eth_txc), .clk(tx_clk_shifted),
        .d0(1'b1), .d1(1'b0), .rst(!txc_reset_sync[2]));
    genvar k;
    generate for (k=0; k<4; k=k+1) begin: tx_pads
        PH1P_LOGIC_HD_ODDR oddr (.q(eth_txd[k]), .clk(tx_clk),
            .d0(gmii_txd[k]), .d1(gmii_txd[k+4]), .rst(!tx_rst_n));
    end endgenerate
    PH1P_LOGIC_HD_ODDR tx_ctl_oddr (.q(eth_tx_ctl), .clk(tx_clk),
        .d0(gmii_tx_en && tx_rst_n), .d1(gmii_tx_en && tx_rst_n), .rst(!tx_rst_n));
    wire [7:0] rx_ddr;
    wire rx_dv_ddr, rx_ctl_fall;
    wire [3:0] rx_data_delayed;
    wire rx_ctl_delayed;
    generate for (k=0; k<4; k=k+1) begin: rx_pads
        // UG1310 pp.71-72: HD-bank pad delay, prior to the input DDR cell.
        PH1P_LOGIC_DYNAMIC_HD_IDELAY #(.CAS_MODE("NONE")) data_delay (
            .i(eth_rxd[k]), .o(rx_data_delayed[k]), .dly(RX_DELAY_TAPS),
            .cas_i(1'b0), .cas_rtn_i(1'b0), .cas_o(), .cas_rtn_o());
        // UG1310 pp.75-76: OPPOSITE_EDGE q0/rising, q1/falling.
        // The common-clock byte register below aligns the previous full byte.
        PH1P_LOGIC_HD_IDDR #(.MODE("OPPOSITE_EDGE"), .ASYNCRST("ENABLE")) iddr (
            .q0(rx_ddr[k]), .q1(rx_ddr[k+4]),
            .clk(rx_clk), .d(rx_data_delayed[k]), .rst(!rx_rst_n));
    end endgenerate
    PH1P_LOGIC_DYNAMIC_HD_IDELAY #(.CAS_MODE("NONE")) ctl_delay (
        .i(eth_rx_ctl), .o(rx_ctl_delayed), .dly(RX_DELAY_TAPS),
        .cas_i(1'b0), .cas_rtn_i(1'b0), .cas_o(), .cas_rtn_o());
    PH1P_LOGIC_HD_IDDR #(.MODE("OPPOSITE_EDGE"), .ASYNCRST("ENABLE")) rx_ctl_iddr (
        .q0(rx_dv_ddr), .q1(rx_ctl_fall),
        .clk(rx_clk), .d(rx_ctl_delayed), .rst(!rx_rst_n));
    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin gmii_rxd <= 0; gmii_rx_dv <= 0; gmii_rx_er <= 0; end
        else begin gmii_rxd <= rx_ddr; gmii_rx_dv <= rx_dv_ddr;
            gmii_rx_er <= rx_dv_ddr ^ rx_ctl_fall; end
    end
endmodule
