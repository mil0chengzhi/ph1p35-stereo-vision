create_clock -name mcu_feedback -period 20.000 [get_pins ae/arithmetic/hard_mcu.ahb_clk]
create_clock -name board_50 -period 20.000 [get_ports clk_50]
create_clock -name mipi_hs -period 2.000 [get_ports mipi_clock_p]
derive_clocks
set DB_ETH_PHY_INSTANCE phy
source [file join [file dirname [info script]] .. .. common constraints ethernet.sdc]
# External button/reset and UART first stages, preserving downstream timing.
set_false_path -from [get_ports {key_n[3]}] -to [get_regs {tracking_key/key_sync[0]}]
set_false_path -from [get_ports uart_rx] -to [get_regs {uart/rx_sync[0]}]
set_output_delay -clock board_50 -max 0.000 [get_ports {step1 dir1 en1 step2 dir2 en2 uart_tx led[*]}]
set_output_delay -clock board_50 -min 0.000 [get_ports {step1 dir1 en1 step2 dir2 en2 uart_tx led[*]}]

# Preserve real PHY/PLL domains. No asynchronous whole-clock groups.
# TD gate evidence: startup_reg[19].q is renamed to the exact DDR sys_rst_n
# alias below. The two exact names identify the same source before/after mapping;
# no wildcard source or additional reset/data path is introduced.
set nv_pll_lock [get_pins {phy/eth_pll/pll_core/u_PH1P_PHY_PLL.lock}]
set_false_path -from $nv_pll_lock -to [get_regs {phy/lock_sync[0] phy/ready_sync[*] phy/txc_reset_sync[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {phy/ready_sync[*] phy/txc_reset_sync[*]}]
set_false_path -from [get_regs {phy/mdio_rst_n phy/phy_mdio/mdio_error phy/phy_mdio/phy_id32[*] phy/phy_mdio/phy_status16[*] phy/phy_mdio/phy_delay16[*]}] -to [get_regs {phy/ready_sync[*]}]
# Local-only V4 does not consume the MAC RX reset/data outputs. The unused
# RX reset chain disappears during placement; do not constrain absent cells.
# TX data/control are compile-time zero. Their unused reset chain may be trimmed.
# No absent optional TX registers are queried and no valid TX path is cut.
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {pixel_reset[*] pixel_meter/source_reset[*] rx_meter/source_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {pixel_reset[*] video_lock_sync[0]}]
set_false_path -to [get_regs {memory_ready_sync[0]}]
set_false_path -from [get_regs {pixel_meter/source_div[7]}] -to [get_regs {pixel_meter/sync_div[0]}]
set_false_path -from [get_regs {rx_meter/source_div[7]}] -to [get_regs {rx_meter/sync_div[0]}]
# PHY loss cancels MAC/private packets, but cannot reset an in-flight DDR FIFO
# response. FIFO ownership survives until the common system/DDR reset only.
set_max_delay -datapath_only 8.000 -from [get_regs {video_fifo/write_gray[*]}] -to [get_regs {video_fifo/wgray_sync0[*]}]
set_max_delay -datapath_only 8.000 -from [get_regs {video_fifo/read_gray[*]}] -to [get_regs {video_fifo/rgray_sync0[*]}]

# Host snapshots freeze the source until request/ACK synchronization completes.
foreach nv_mb {camera_capture pixel_capture front_capture ae_capture peer_capture link_pixel_capture link_tx_capture} {
 set_false_path -from [get_regs ${nv_mb}/request] -to [get_regs [format {%s/request_sync[0]} $nv_mb]]
 set_false_path -from [get_regs ${nv_mb}/ack] -to [get_regs [format {%s/ack_sync[0]} $nv_mb]]
 set_max_delay -datapath_only 20.000 -from [get_regs [format {%s/frozen[*]} $nv_mb]] -to [get_regs [format {%s/read_data[*]} $nv_mb]]
 set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs [list [format {%s/source_reset[*]} $nv_mb] [format {%s/read_reset[*]} $nv_mb]]]
}
set_false_path -from [get_regs {storage/reset_sync[*]}] -to [get_regs {front_capture/source_reset[*] front_capture/read_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {pixel_capture/source_reset[*] pixel_capture/read_reset[*]}]

# Qualified DDR store/display CDC, restricted to modules present on B.
set_false_path -from [get_regs {storage/meter/source_div[7]}] -to [get_regs {storage/meter/sync_div[0]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {storage/meter/source_reset[*] storage/reset_sync[*] storage/ready_sync[*]}]
set_false_path -to [get_regs {storage/done_sync[0]}]
set_false_path -from [get_regs {storage/test/errors[*] storage/test/first_address[*] storage/test/expected[*] storage/test/observed[*] storage/test/failed}] -to [get_regs {storage/debug[*]}]
set_false_path -to [get_regs {lines/req_sync[0] lines/ack_sync[0] storage/debug_req_sync[0] storage/debug_ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {lines/requested_row[*] lines/requested_bank lines/requested_new lines/requested_row[*]_dup_* lines/requested_bank_dup_* lines/requested_new_dup_*}] -to [get_regs {lines/memory_row[*] lines/memory_bank lines/memory_new lines/memory_row[*]_dup_* lines/memory_bank_dup_* lines/memory_new_dup_*}]
set_max_delay -datapath_only 20.000 -from [get_regs {lines/response_good}] -to [get_regs {lines/ready_bits[*]}]
set_max_delay -datapath_only 20.000 -from [get_regs {lines/response_frame_id[*]}] -to [get_regs {lines/source_frame_id[*]}]
set_max_delay -datapath_only 20.000 -from [get_regs {lines/response_good}] -to [get_regs {lines/source_frame_valid}]
set_max_delay -datapath_only 20.000 -from [get_regs {storage/video_snapshot[*]}] -to [get_regs {storage/debug[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n storage/reset_sync[1]}] -to [get_regs {lines/* storage/debug_req_sync[*] storage/debug_ack storage/video_snapshot[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {lines/*}]

# V4 local camera only. Dedicated DPHY0 is located by its primitive, not GPIO.
set_false_path -from [get_ports camera_scl] -to [get_regs {cam_i2c/scl_sync[0]}]
set_false_path -from [get_ports camera_sda] -to [get_regs {cam_i2c/sda_sync[0]}]
set_false_path -from [get_pins {clocks/camera_pll.lock}] -to [get_regs {pc_sync[0] camera_reset[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {camera_reset[*] camera_meter/source_reset[*] byte_meter/source_reset[*]}]
set_false_path -from [get_regs {camera_meter/source_div[7]}] -to [get_regs {camera_meter/sync_div[0]}]
set_false_path -from [get_regs {byte_meter/source_div[7]}] -to [get_regs {byte_meter/sync_div[0]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n rx_enable}] -to [get_regs {byte_reset[*] hs_sync[*] packet_toggle good_toggle ecc_toggle crc_toggle fs_toggle fe_toggle}]
set_false_path -from [get_regs {receiver/hs_active}] -to [get_regs {hs_sync[0]}]
set_false_path -from [get_regs {packet_toggle}] -to [get_regs {receiver/done_sync[0] event_sync0[0]}]
set_false_path -from [get_regs {good_toggle}] -to [get_regs {event_sync0[1]}]
set_false_path -from [get_regs {ecc_toggle}] -to [get_regs {event_sync0[2]}]
set_false_path -from [get_regs {crc_toggle}] -to [get_regs {event_sync0[3]}]
set_false_path -from [get_regs {fs_toggle}] -to [get_regs {event_sync0[4]}]
set_false_path -from [get_regs {fe_toggle}] -to [get_regs {event_sync0[5]}]
set_false_path -from [get_pins {receiver/phy.o_ck_ls_rx_p receiver/phy.o_ck_ls_rx_n receiver/phy.o_l0_ls_rx_p receiver/phy.o_l0_ls_rx_n receiver/phy.o_l1_ls_rx_p receiver/phy.o_l1_ls_rx_n}] -to [get_regs {receiver/lp_sync0[*]}]
# Shared FIFO reset never follows a source selector or camera HS/LP state.
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n storage/reset_sync[1]}] -to [get_regs {video_fifo/write_reset[*] video_fifo/read_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {ae_capture/source_reset[*] ae_capture/read_reset[*]}]

# Stereo peer line mailbox: frozen complete-row data, request/ACK first stages.
set_false_path -to [get_regs {peer_lines/req_sync[0] peer_lines/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {peer_lines/requested_row[*] peer_lines/requested_bank peer_lines/requested_new peer_lines/requested_row[*]_dup_* peer_lines/requested_bank_dup_* peer_lines/requested_new_dup_*}] -to [get_regs {peer_lines/memory_row[*] peer_lines/memory_bank peer_lines/memory_new peer_lines/memory_row[*]_dup_* peer_lines/memory_bank_dup_* peer_lines/memory_new_dup_*}]
set_max_delay -datapath_only 20.000 -from [get_regs {peer_lines/response_good}] -to [get_regs {peer_lines/ready_bits[*] peer_lines/source_frame_valid}]
set_max_delay -datapath_only 20.000 -from [get_regs {peer_lines/response_frame_id[*]}] -to [get_regs {peer_lines/source_frame_id[*]}]
set_max_delay -datapath_only 20.000 -from [get_regs {peer_lines/response_session[*]}] -to [get_regs {peer_lines/source_session[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n storage/reset_sync[1]}] -to [get_regs {peer_lines/* storage/remote_video_snapshot[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {peer_lines/*}]
set_max_delay -datapath_only 20.000 -from [get_regs {storage/remote_video_snapshot[*]}] -to [get_regs {storage/remote_debug[*]}]
set_false_path -to [get_regs {storage/arb_fault_sync[0]}]
# Two independent token FIFOs; Gray pointers bounded to first stage only.
set_max_delay -datapath_only 8.000 -from [get_regs {peer_fifo/write_gray[*]}] -to [get_regs {peer_fifo/wgray_sync0[*]}]
set_max_delay -datapath_only 8.000 -from [get_regs {peer_fifo/read_gray[*]}] -to [get_regs {peer_fifo/rgray_sync0[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n storage/reset_sync[1]}] -to [get_regs {peer_fifo/write_reset[*] peer_fifo/read_reset[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n phy/ready_sync[1]}] -to [get_regs {peer_pipeline_reset[*] peer_capture/source_reset[*] peer_capture/read_reset[*]}]
# Packet slots remain privately owned until synchronized release.
set_false_path -from [get_regs {queue/published[*]}] -to [get_regs {queue/publish_sync0[*]}]
set_false_path -from [get_regs {queue/released[*]}] -to [get_regs {queue/release_sync0[*]}]
set_max_delay -datapath_only 8.000 -from [get_regs {queue/frame_meta* queue/index_meta* queue/session_meta*}] -to [get_regs {queue/packet_frame[*] queue/packet_index[*] queue/packet_session[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n phy/ready_sync[1]}] -to [get_regs {queue/pixel_reset[*] queue/tx_reset[*] link_tx_capture/source_reset[*] link_tx_capture/read_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {queue/pixel_reset[*] queue/tx_reset[*] link_tx_capture/source_reset[*] link_tx_capture/read_reset[*] link_pixel_capture/source_reset[*] link_pixel_capture/read_reset[*]}]
# Real GMII RX/TX reset chains are active in the stereo image.
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n phy/ready_sync[1]}] -to [get_regs {phy/tx_reset_sync[*] phy/rx_reset_sync[*]}]

set_false_path -from [get_ports {key_n[0]}] -to [get_regs {switch_key/key_sync[0]}]
set_false_path -from [get_regs {control_mode/request}] -to [get_regs {control_mode/request_sync[0]}]
set_false_path -from [get_regs {control_mode/ack}] -to [get_regs {control_mode/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {control_mode/held_mode[*]}] -to [get_regs {control_mode/pending_mode[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {control_mode/src_reset[*] control_mode/dst_reset[*] switch_key/*}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {control_mode/src_reset[*] control_mode/dst_reset[*]}]

# DDR-ready contributes only to peer raw reset entry; keep all payload/Gray timing.
set_false_path -from [get_regs {storage/reset_sync[*]}] -to [get_regs {peer_pipeline_reset[*] peer_capture/source_reset[*] peer_capture/read_reset[*]}]

# Optional display NR: exact KEY2 and mailbox first stages; held payload remains timed.
set_false_path -from [get_ports {key_n[1]}] -to [get_regs {noise_control/key_sync[0]}]
set_false_path -from [get_regs {noise_mode/request}] -to [get_regs {noise_mode/request_sync[0]}]
set_false_path -from [get_regs {noise_mode/ack}] -to [get_regs {noise_mode/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {noise_mode/held_mode[*]}] -to [get_regs {noise_mode/pending_mode[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {noise_mode/src_reset[*] noise_mode/dst_reset[*] noise_control_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {noise_mode/src_reset[*] noise_mode/dst_reset[*] noise_control_reset[*]}]

# The above20ns frozen-payload mailbox bounds cover its same-clock register clones.
# No whole-domain exception: FF side/Q ownership is independently reviewed.

# B15 candidate: exact first stages and held command/result buses.
# Command is frozen before a two-memory-clock request synchronizer; response is
# frozen before a two-board50-clock ACK synchronizer. Physical proof is pending.
# Boot probe publishes stable fields before done passes its existing two-flop
# synchronizer. Keep the frozen payload bounded instead of cutting a whole clock.
set_max_delay -datapath_only 20.000 -from [get_regs {storage/address_probe/errors[*] storage/address_probe/first_address[*] storage/address_probe/expected[*] storage/address_probe/observed[*] storage/address_probe/failed}] -to [get_regs {storage/debug[*]}]

# B23 exact new mailbox/cache first stages and frozen buses; no whole-clock cut.
set_false_path -to [get_regs {depth_lines/req_sync[0] depth_lines/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {depth_lines/requested_row[*] depth_lines/requested_bank depth_lines/requested_new depth_lines/requested_row[*]_dup_* depth_lines/requested_bank_dup_* depth_lines/requested_new_dup_*}] -to [get_regs {depth_lines/memory_row[*] depth_lines/memory_bank depth_lines/memory_new depth_lines/memory_row[*]_dup_* depth_lines/memory_bank_dup_* depth_lines/memory_new_dup_*}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n storage/reset_sync[1]}] -to [get_regs {depth_lines/*}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {depth_lines/*}]
set_false_path -from [get_regs {depth_run_mode/request}] -to [get_regs {depth_run_mode/request_sync[0]}]
set_false_path -from [get_regs {depth_run_mode/ack}] -to [get_regs {depth_run_mode/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {depth_run_mode/held_mode[*]}] -to [get_regs {depth_run_mode/pending_mode[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {depth_run_mode/src_reset[*] depth_run_mode/dst_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {depth_run_mode/src_reset[*] depth_run_mode/dst_reset[*]}]
set_false_path -from [get_regs {depth_capture/request}] -to [get_regs {depth_capture/request_sync[0]}]
set_false_path -from [get_regs {depth_capture/ack}] -to [get_regs {depth_capture/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {depth_capture/frozen[*]}] -to [get_regs {depth_capture/read_data[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {depth_capture/source_reset[*] depth_capture/read_reset[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n phy/ready_sync[1]}] -to [get_regs {queue/pixel_reset[*] queue/tx_reset[*] depth_capture/source_reset[*] depth_capture/read_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {queue/pixel_reset[*] queue/tx_reset[*] depth_capture/source_reset[*] depth_capture/read_reset[*] link_pixel_capture/source_reset[*] link_pixel_capture/read_reset[*]}]
set_false_path -from [get_regs {startup[19] storage/reset_sync[1]}] -to [get_regs {depth_run_mode/src_reset[*] depth_run_mode/dst_reset[*] depth_capture/source_reset[*] depth_capture/read_reset[*] depth_key_control/*}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {depth_run_mode/src_reset[*] depth_run_mode/dst_reset[*]}]
# Scalar HUD status CDC; second stage and all HUD logic remain timed.
set_false_path -to [get_regs {depth_fault_sync[0]}]

# ISP statistics/controls; no business-path or second-stage blanket exceptions.
set_false_path -from [get_regs {awb_enable}] -to [get_regs {awb_enable_sync[0]}]
set_false_path -from [get_regs {ae/* ae_enable}] -to [get_regs {awb_hold_sync[0]}]
set_false_path -from [get_regs {awb_capture/request}] -to [get_regs {awb_capture/request_sync[0]}]
set_false_path -from [get_regs {awb_capture/ack}] -to [get_regs {awb_capture/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {awb_capture/frozen[*]}] -to [get_regs {awb_capture/read_data[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {awb_capture/source_reset[*] awb_capture/read_reset[*]}]
set_false_path -from [get_regs {hist_capture/request}] -to [get_regs {hist_capture/request_sync[0]}]
set_false_path -from [get_regs {hist_capture/ack}] -to [get_regs {hist_capture/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {hist_capture/frozen[*]}] -to [get_regs {hist_capture/read_data[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {hist_capture/source_reset[*] hist_capture/read_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {hist_capture/source_reset[*] hist_capture/read_reset[*]}]
set_false_path -from [get_regs {hist_mode/request}] -to [get_regs {hist_mode/request_sync[0]}]
set_false_path -from [get_regs {hist_mode/ack}] -to [get_regs {hist_mode/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {hist_mode/held_mode[*]}] -to [get_regs {hist_mode/pending_mode[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {hist_mode/src_reset[*] hist_mode/dst_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {hist_mode/src_reset[*] hist_mode/dst_reset[*]}]

# B28 faster DPHY LP-state controller, exact first-stage/event/reset entries.
set_false_path -from [get_regs {rx_enable}] -to [get_regs {receiver/enable_sync[0]}]
set_false_path -from [get_regs {receiver/timeout_event}] -to [get_regs {phy_timeout_sync[0]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {receiver/control_reset[*]}]

# Selected-colour mailbox: same frozen request/ack protocol.
set_false_path -from [get_regs {gimbal_colour_mode/request}] -to [get_regs {gimbal_colour_mode/request_sync[0]}]
set_false_path -from [get_regs {gimbal_colour_mode/ack}] -to [get_regs {gimbal_colour_mode/ack_sync[0]}]
set_max_delay -datapath_only 20.000 -from [get_regs {gimbal_colour_mode/held_mode[*]}] -to [get_regs {gimbal_colour_mode/pending_mode[*]}]
set_false_path -from [get_regs {startup[19] storage/memory/ph1p_ddrmc_wrapper/ph1p_ddrmc_top/u_ddr_phy/u_ddrphy_top/sys_rst_n}] -to [get_regs {gimbal_colour_mode/src_reset[*] gimbal_colour_mode/dst_reset[*]}]
set_false_path -from [get_pins {clocks/video_pll.lock}] -to [get_regs {gimbal_colour_mode/src_reset[*] gimbal_colour_mode/dst_reset[*]}]

# HC-05: only the first RX synchronizer entry is asynchronous.
set_false_path -from [get_ports {hc05_rx}] -to [get_regs {bluetooth/uart/rx_sync[0]}]
set_output_delay -clock board_50 -max 0.000 [get_ports {hc05_tx}]
set_output_delay -clock board_50 -min 0.000 [get_ports {hc05_tx}]
set_false_path -from [get_regs {bt_view_mailbox/request}] -to [get_regs {bt_view_mailbox/request_sync[0]}]
set_false_path -from [get_regs {bt_view_mailbox/ack}] -to [get_regs {bt_view_mailbox/ack_sync[0]}]
set_max_delay -datapath_only 8.000 -from [get_regs {bt_view_mailbox/held_data[*]}] -to [get_regs {follow_mac/tx_shift[*]}]
# One-shot mailbox: exact first stages only, held cancel epoch has a timed bound.
set_false_path -from [get_regs {capture_gate/request_toggle}] -to [get_regs {capture_gate/request_sync[0]}]
set_false_path -from [get_regs {capture_gate/cancel_toggle}] -to [get_regs {capture_gate/cancel_sync[0]}]
set_false_path -from [get_regs {capture_gate/request_seen}] -to [get_regs {capture_gate/request_ack_sync[0]}]
set_false_path -from [get_regs {capture_gate/cancel_seen}] -to [get_regs {capture_gate/cancel_ack_sync[0]}]
set_false_path -from [get_regs {capture_gate/active}] -to [get_regs {capture_gate/busy_sync[0]}]
set_max_delay -datapath_only 12.000 -from [get_regs {capture_gate/cancel_payload}] -to [get_regs {capture_gate/canceled_epoch capture_gate/budget capture_gate/active}]
