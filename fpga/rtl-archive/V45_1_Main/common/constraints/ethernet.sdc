# Draft constraints: not board-tested, not a timing sign-off.
# Top-level SDC must create the REAL 50MHz primary clock (20ns), then derive_clocks.
# Set DB_ETH_PHY_INSTANCE before sourcing for a different wrapper hierarchy.
if {![info exists DB_ETH_PHY_INSTANCE]} {set DB_ETH_PHY_INSTANCE net_link/phy}
create_clock -name db_eth_rx_pad -period 8.000 -waveform {0.000 4.000} [get_ports eth_rxc]

# YT8511 datasheet p85, RGMII timing with internal delay: transmitter
# guaranteed TsetupT_min=1.65ns and TholdT_min=1.65ns. Data arrival after
# the current edge: min=TholdT_min-0.2=1.45ns; max=4-TsetupT_min+0.2=2.55ns.
# The +/-0.2ns board skew budget is an ASSUMPTION pending PCB measurement.
# Use TholdT_min directly, not 4-TsetupT_max: PHY duty need not be exactly50%.
# Positive [1.45,2.55]ns describes the NEXT transition after each clock edge:
# setup uses the next opposite edge; hold uses the present same edge.
# Negative [-2.4,-1.45] describes the previous transition and incorrectly
# asks TD to compare it against same-edge hold, subtracting a whole DDR UI.
# Both rise/fall constraints remain active. No multicycle/false-path repair.
set_input_delay -clock db_eth_rx_pad -max 2.550 [get_ports {eth_rx_ctl eth_rxd[*]}]
set_input_delay -clock db_eth_rx_pad -min 1.450 [get_ports {eth_rx_ctl eth_rxd[*]}]
set_input_delay -clock db_eth_rx_pad -max 2.550 -clock_fall -add_delay [get_ports {eth_rx_ctl eth_rxd[*]}]
set_input_delay -clock db_eth_rx_pad -min 1.450 -clock_fall -add_delay [get_ports {eth_rx_ctl eth_rxd[*]}]

# The forwarded clock inherits PLL C1=90deg (2ns); do not add phase again.
set db_eth_c1 "${DB_ETH_PHY_INSTANCE}/eth_pll/pll_core/u_PH1P_PHY_PLL.clkc\[1\]"
create_generated_clock -name db_eth_tx_pad -source [get_pins $db_eth_c1] -multiply_by 1 [get_ports eth_txc]

# YT8511 datasheet p85 draws TsetupR and TholdR at external TXD/TX_EN versus
# GTX_CLK, each minimum1.0ns, with the clock delay integrated at transmitter
# (here FPGA TXC). These are PHY PIN requirements in the default configuration.
# max=1.0+0.2=1.2ns; min=-(1.0+0.2)=-1.2ns. Do NOT add the typical internal
# TX clock delay a second time: internal sampling is already covered by the
# receiver pin specification. MDIO checks default ext0Ch TXdelay code5.
# Board +/-0.2ns remains an explicit unmeasured assumption. Both slow/fast
# routed timing MUST pass these pin requirements; no blanket false paths.
set_output_delay -clock db_eth_tx_pad -max 1.200 [get_ports {eth_tx_ctl eth_txd[*]}]
set_output_delay -clock db_eth_tx_pad -min -1.200 [get_ports {eth_tx_ctl eth_txd[*]}]
set_output_delay -clock db_eth_tx_pad -max 1.200 -clock_fall -add_delay [get_ports {eth_tx_ctl eth_txd[*]}]
set_output_delay -clock db_eth_tx_pad -min -1.200 -clock_fall -add_delay [get_ports {eth_tx_ctl eth_txd[*]}]

# No RX/TX clock-group false-path: data CDC must be constrained at specific
# mailbox synchronizers by integration SDC. Never hide IDDR->register hold.
# MDIO input is sampled through two registers, with >=460ns settling before
# the sample enable at 1MHz MDC. Cut only the asynchronous pad->first register.
set_false_path -from [get_ports eth_mdio] -to [get_regs "${DB_ETH_PHY_INSTANCE}/phy_mdio/master/mdio_sync\[0\]"]
