`timescale 1ns/1ps
// B31 bounded visual servo candidate: sampled spatial support, explicit ARM and heartbeat.
module network_gimbal_top #(parameter BOARD_ID=2,REVISION=94)(
 input wire clk_50,input wire [3:0] key_n,input wire uart_rx,output wire uart_tx,
 input wire hc05_rx,output wire hc05_tx,
 output wire step1,dir1,en1,step2,dir2,en2,output wire [3:0] led,
 input wire eth_rxc,eth_rx_ctl,input wire [3:0] eth_rxd,
 output wire eth_txc,eth_tx_ctl,output wire [3:0] eth_txd,
 output wire eth_reset_n,eth_mdc,inout wire eth_mdio,
 inout wire hdmi_scl,hdmi_sda,output wire hdmi_power_enable,
 output wire [2:0] hdmi_data_p,hdmi_data_n,output wire hdmi_clock_p,hdmi_clock_n,
 inout wire mipi_clock_p,mipi_clock_n,inout wire [3:0] mipi_data_p,mipi_data_n,
 output wire camera_xclk,camera_pwdn_n,inout wire camera_scl,camera_sda,
 output wire ddr_ck_p,ddr_ck_n,ddr_cke,ddr_odt,ddr_cs_n,ddr_ras_n,ddr_cas_n,ddr_we_n,
 output wire [1:0] ddr_ba,output wire [12:0] ddr_addr,
 inout wire [1:0] ddr_dqs_p,ddr_dqs_n,inout wire [15:0] ddr_dq,inout wire [1:0] ddr_dm
);
 localparam [47:0] LOCAL_MAC=BOARD_ID==1?48'h020000002501:48'h020000002502;
 localparam [47:0] PEER_MAC=BOARD_ID==1?48'h020000002502:48'h020000002501;
 localparam [7:0] LOCAL_STREAM=BOARD_ID,PEER_STREAM=BOARD_ID==1?2:1;
 wire board50_global;
 PH1P_LOGIC_BUFG board50_buffer(.i(clk_50),.o(board50_global));
 reg [19:0] startup=0;reg [1:0] key_sync=0;
 always @(posedge board50_global)begin
  key_sync<={key_sync[0],1'b1};
  if(!key_sync[1])startup<=0;else if(!startup[19])startup<=startup+1'b1;
 end
 wire sys_rst_n=startup[19];
 // K1 source, K2 stereo, K3 motor release, K4 follow/pause (pause holds).
 // Tracking and stereo acquisition are mutually exclusive; starts wait for drain.
 wire [63:0]motor_diagnostic; // B30: explicit-arm bounded servo below, boot outputs0.
 wire[511:0]gimbal_status;wire[31:0]gimbal_session_status;wire[31:0]gimbal_tuning_status;
 assign hdmi_scl=1'bz;assign hdmi_sda=1'bz;assign hdmi_power_enable=1'b1;
 wire tx_clk,tx_rst_n,rx_clk,rx_rst_n,phy_ready,mdio_done,mdio_error;
 wire [7:0] gmii_rxd,gmii_txd;wire gmii_rx_dv,gmii_rx_er,gmii_tx_en;
 wire[7:0]legacy_gmii_data,hd_gmii_data;wire legacy_gmii_enable,hd_gmii_enable,hd_tx_offer,hd_tx_accept,hd_tx_busy,legacy_gate;

 wire [31:0] phy_id;wire [15:0] phy_status,phy_delay;
 db_rgmii_phy phy(.clk_50(board50_global),.rst_n(sys_rst_n),
 .tx_clk(tx_clk),.tx_rst_n(tx_rst_n),.rx_clk(rx_clk),.rx_rst_n(rx_rst_n),
 .gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),.gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),
 .eth_rxc(eth_rxc),.eth_rx_ctl(eth_rx_ctl),.eth_rxd(eth_rxd),.eth_txc(eth_txc),.eth_tx_ctl(eth_tx_ctl),.eth_txd(eth_txd),
 .eth_reset_n(eth_reset_n),.phy_ready(phy_ready),.eth_mdc(eth_mdc),.eth_mdio(eth_mdio),
 .phy_id32(phy_id),.phy_status16(phy_status),.phy_delay16(phy_delay),.mdio_done(mdio_done),.mdio_error(mdio_error));
 wire memory_clk,memory_rst_n,memory_verified;wire [639:0] memory_debug;
 wire depth_at_pixel,depth_ack,depth_run_at_memory,depth_run_busy,depth_run_ack;
 reg depth_requested;wire depth_key;wire depth_epoch_exhausted;
 wire depth_line_request,depth_line_ready,depth_line_new,depth_line_bank_request,depth_line_write,depth_line_bank,depth_line_done,depth_line_good;
 wire[15:0]depth_line_row;wire[127:0]depth_line_data;wire[7:0]depth_line_address;
 wire[31:0]depth_line_frame,depth_line_session,depth_line_start;
 wire[31:0]depth_jobs_started,depth_jobs_completed,depth_jobs_skipped,depth_published;
 wire depth_system_fault,depth_config_error;
 wire[15:0]depth_image;wire depth_image_valid,depth_source_valid;
 wire[31:0]depth_source_frame,depth_source_session,depth_source_start,depth_underflows;
 reg display_depth_active;
 wire fifo_empty,fifo_read,fifo_valid,fifo_full,fifo_overflow;
 wire [129:0] fifo_data,packed_data;wire packed_write;
 // Canonical B metadata only; the actual ISP RGB565 DATA beats are unchanged.
 wire[129:0]canonical_packed_data=packed_data[129:128]==1?{2'b01,64'd0,32'hB1000001,packed_data[31:0]}:
   packed_data[129:128]==2?{2'b10,63'd0,32'hB1000001,packed_data[32:1],packed_data[0]}:packed_data;
 wire line_request,line_ready,line_new,line_request_bank,line_write,line_bank,line_done,line_good;
 wire [15:0] line_row;wire [127:0] line_data;wire [7:0] line_address;
 wire [31:0] line_frame_id,line_session,line_start_ms;
 wire peer_fifo_empty,peer_fifo_read,peer_fifo_valid,peer_fifo_full,peer_fifo_overflow;
 wire [129:0] peer_fifo_data,peer_packed_data;wire peer_packed_write;
 wire peer_line_request,peer_line_ready,peer_line_new,peer_line_request_bank;
 wire peer_line_write,peer_line_bank,peer_line_done,peer_line_good;
 wire [15:0] peer_line_row;wire [127:0] peer_line_data;wire [7:0] peer_line_address;
 wire [31:0] peer_line_frame_id,peer_line_session,peer_line_start_ms;
 wire [255:0] peer_memory_debug;
 wire peer_packet_accept,peer_packet_start,peer_word_write,peer_packet_done,peer_packet_good;
 wire [9:0] peer_word_address;wire [15:0] peer_word_data,peer_packet_index;
 wire [31:0] peer_packet_session,peer_packet_frame,peer_rx_packets,peer_bad_crc,peer_rejected,peer_no_slot;
 wire [31:0] peer_completed,peer_dropped,peer_queue_loss,peer_last_session,peer_last_frame;
 wire [7:0] peer_assembly_flags;
 wire peer_pipeline_raw=sys_rst_n&&phy_ready&&memory_rst_n;
 reg [1:0] peer_pipeline_reset;
 always @(posedge rx_clk or negedge peer_pipeline_raw)
  if(!peer_pipeline_raw)peer_pipeline_reset<=0;else peer_pipeline_reset<={peer_pipeline_reset[0],1'b1};
 nv32_video_rx #(.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC),.WIDTH(1280),.HEIGHT(720),.PACKET_PIXELS(640),.STREAM_ID(PEER_STREAM)) peer_mac(
  .clk(rx_clk),.rst_n(peer_pipeline_reset[1]),.gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),.packet_accept(peer_packet_accept),
  .packet_start(peer_packet_start),.packet_word_write(peer_word_write),.packet_word_addr(peer_word_address),.packet_word_data(peer_word_data),
  .packet_done(peer_packet_done),.packet_good(peer_packet_good),.packet_session(peer_packet_session),.packet_frame(peer_packet_frame),.packet_index(peer_packet_index),
  .rx_packets(peer_rx_packets),.rx_bad_crc(peer_bad_crc),.rx_rejected(peer_rejected),.rx_no_slot(peer_no_slot));
 wire peer_session_ok=(peer_packet_session[31:24]==8'hA1||peer_packet_session[31:24]==8'hD1)&&peer_packet_session[21:0]!=0;
 nv_packet_frames #(.PACKET_PIXELS(640),.PACKETS_PER_FRAME(1440)) peer_assembly(
  .clk(rx_clk),.rst_n(peer_pipeline_reset[1]),.packet_accept(peer_packet_accept),.packet_start(peer_packet_start),.packet_word_write(peer_word_write),
  .packet_word_addr(peer_word_address),.packet_word_data(peer_word_data),.packet_done(peer_packet_done),.packet_good(peer_packet_good&&peer_session_ok),
  .packet_session(peer_packet_session),.packet_frame(peer_packet_frame),.packet_index(peer_packet_index),
  .fifo_full(peer_fifo_full),.fifo_write(peer_packed_write),.fifo_data(peer_packed_data),
  .completed_frames(peer_completed),.dropped_frames(peer_dropped),.queue_losses(peer_queue_loss),
  .last_session(peer_last_session),.last_frame(peer_last_frame),.state_flags(peer_assembly_flags));
 // PHY restart never clears a FIFO that the DDR reader may already own.
 video_async_fifo #(.ADDRESS_BITS(7)) peer_fifo(.write_clk(rx_clk),.read_clk(memory_clk),.rst_n(sys_rst_n&&memory_rst_n),
  .write_data(peer_packed_data),.write_en(peer_packed_write&&peer_pipeline_reset[1]),.full(peer_fifo_full),.overflow(peer_fifo_overflow),
  .read_en(peer_fifo_read),.read_data(peer_fifo_data),.read_valid(peer_fifo_valid),.empty(peer_fifo_empty));
 // Independent local/remote frame slots share one physical DDR controller.
 wire [15:0] reg_addr;wire probe_write;wire [7:0] write_data;
 wire tracking_requested;wire[127:0]tracking_information;wire[255:0]tracking_statistics;
 wire bt_status_latch;
 wire take=(probe_write&&reg_addr==16'h0100&&write_data==1)||bt_status_latch;
 wire metric_memory_pause,metric_memory_held,metric_memory_quiescent,metric_hold_requested;wire[127:0]metric_job_sources;
 wire [7:0] snapshot_probe_data;wire snapshot_probe_selected;
 // Standalone visible-surface point cloud. Six motor outputs remain constant0.
 wire cloud_at_pixel,orbit_at_pixel,cloud_at_memory,orbit_at_memory;
 wire display_depth_request,display_depth_ready,display_depth_new,display_depth_bank,display_depth_write,display_depth_done;
 wire[15:0]display_depth_row;
 wire cloud_line_request,cloud_line_ready,cloud_line_new,cloud_line_write,cloud_line_done;
 wire[15:0]cloud_line_row,cloud_image;wire cloud_image_valid,cloud_source_valid;
 wire[31:0]cloud_source_job,cloud_render_count,cloud_reject_count;wire[15:0]cloud_point_count;
 wire cloud_fault;wire[5:0]cloud_view,cloud_state;
 wire cloud_read_active,cloud_read_owner;
 reg display_cloud_active;
 // A frozen cloud owns its raster, validity, job and transport frame counter.
 // The independent live depth-preview cache must never select camera pixels
 // under a D3 cloud tag if its first row happens to be unavailable.
 wire display_metric_active=display_cloud_active||display_depth_active;
 wire[31:0]shown_metric_job=display_cloud_active?cloud_source_job:depth_source_frame;
 
 wire bt_capture_request,bt_exit_request,bt_view_commit;
 wire[47:0]bt_view_record;reg[15:0]bt_applied_token;
 wire[15:0]bt_valid_requests,bt_bad_requests;wire bt_framing_error;
 wire capture_budget,capture_ready_internal,capture_busy_internal,capture_gate_ready;
 wire[31:0]hd_completed_job,hd_confirmed_job;
 // Declare the request sources before the instance: TD must not infer
 // separate implicit nets when later continuous assignments are elaborated.
 wire control_key_accept,depth_key_accept,depth_host_accept,bt_capture_accept;
 reg[1:0]memory_ready_sync;wire tr_bank;
 reg display_de,display_hs,display_vs;reg[15:0]display_x,display_y;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]gimbal_stop_sync;
 wire[3:0]cloud_settle_jobs={3'd0,capture_budget};
 reg[1:0]bt_baud_select;
 wire[15:0]bt_baud_div=bt_baud_select==1?16'd1302:bt_baud_select==2?16'd434:16'd5208;
 always @(posedge board50_global)begin
  if(!sys_rst_n)bt_baud_select<=0;
  else if(probe_write&&reg_addr==16'h0b20&&write_data<=2)bt_baud_select<=write_data[1:0];
 end


 ph1p_video_memory #(.WIDTH(1280),.HEIGHT(960),.REMOTE_WIDTH(1280),.REMOTE_HEIGHT(720),.STEREO_ENABLED(1),.STEREO_AUTOMATIC(1),.METRIC_EXPORT_ENABLED(1)) storage(.clk_50(board50_global),.rst_n(sys_rst_n),.debug_latch(take),.debug(memory_debug),
 .memory_clk(memory_clk),.memory_rst_n(memory_rst_n),.memory_verified(memory_verified),
 .stereo_run_enable(depth_run_at_memory&&capture_budget),.metric_export_pause(metric_memory_pause),.metric_export_quiescent(metric_memory_quiescent),.metric_job_sources(metric_job_sources),
 .depth_line_req_valid(depth_line_request),.depth_line_req_ready(depth_line_ready),.depth_new_frame(depth_line_new),.depth_source_row(depth_line_row),.depth_request_bank(depth_line_bank_request),
 .depth_line_write(depth_line_write),.depth_line_data(depth_line_data),.depth_line_address(depth_line_address),.depth_line_bank(depth_line_bank),.depth_line_done(depth_line_done),.depth_line_good(depth_line_good),
 .depth_line_frame_id(depth_line_frame),.depth_line_session(depth_line_session),.depth_line_start_ms(depth_line_start),
 .stereo_jobs_started(depth_jobs_started),.stereo_jobs_completed(depth_jobs_completed),.stereo_jobs_skipped(depth_jobs_skipped),.depth_result_published(depth_published),
 .stereo_system_fault(depth_system_fault),.stereo_configuration_error(depth_config_error),
 .stereo_quiesce(1'b0),.stereo_rearm(1'b0),.stereo_epoch(32'd1),
 .stereo_pipeline_idle(1'b1),.stereo_reader_idle(1'b1),.stereo_acquire(2'b0),.stereo_release(2'b0),.stereo_release_cookies(96'b0),
 .stereo_araddr(25'b0),.stereo_arvalid(1'b0),.stereo_rready(1'b0),.stereo_read_active(1'b0),
 .snapshot_address(reg_addr),.snapshot_write(probe_write),.snapshot_write_data(write_data),
 .snapshot_read_data(snapshot_probe_data),.snapshot_selected(snapshot_probe_selected),.token_empty(fifo_empty),.token_read(fifo_read),.token_valid(fifo_valid),.token_data(fifo_data),
 .line_req_valid(line_request),.line_req_ready(line_ready),.new_frame(line_new),.source_row(line_row),.request_bank(line_request_bank),
 .line_write(line_write),.line_data(line_data),.line_address(line_address),.line_bank(line_bank),.line_done(line_done),.line_good(line_good),
 .line_frame_id(line_frame_id),.line_session(line_session),.time_gray(32'b0),.line_start_ms(line_start_ms),
 .remote_token_empty(peer_fifo_empty),.remote_token_read(peer_fifo_read),.remote_token_valid(peer_fifo_valid),.remote_token_data(peer_fifo_data),
 .remote_line_req_valid(peer_line_request),.remote_line_req_ready(peer_line_ready),.remote_new_frame(peer_line_new),.remote_source_row(peer_line_row),.remote_request_bank(peer_line_request_bank),
 .remote_line_write(peer_line_write),.remote_line_data(peer_line_data),.remote_line_address(peer_line_address),.remote_line_bank(peer_line_bank),
 .remote_line_done(peer_line_done),.remote_line_good(peer_line_good),.remote_line_frame_id(peer_line_frame_id),.remote_line_session(peer_line_session),
 .remote_line_start_ms(peer_line_start_ms),.remote_debug(peer_memory_debug),
 .ddr_ck_p(ddr_ck_p),.ddr_ck_n(ddr_ck_n),.ddr_cke(ddr_cke),.ddr_odt(ddr_odt),.ddr_cs_n(ddr_cs_n),.ddr_ras_n(ddr_ras_n),
 .ddr_cas_n(ddr_cas_n),.ddr_we_n(ddr_we_n),.ddr_ba(ddr_ba),.ddr_addr(ddr_addr),.ddr_dqs_p(ddr_dqs_p),.ddr_dqs_n(ddr_dqs_n),.ddr_dq(ddr_dq),.ddr_dm(ddr_dm));
 wire pixel,serial,pll_v,pll_c,camera_clock_internal;
 ph1p_clocks clocks(.clk_50(board50_global),.reset(!sys_rst_n),.pixel(pixel),.serial(serial),.lock_video(pll_v),.camera(camera_clock_internal),.lock_camera(pll_c));
 reg [3:0] pixel_reset;
 wire pixel_raw=sys_rst_n&&pll_v;
 always @(posedge pixel or negedge pixel_raw)if(!pixel_raw)pixel_reset<=0;else pixel_reset<={pixel_reset[2:0],1'b1};
 wire pixel_rst_n=pixel_reset[3];
 wire [15:0] x,y;wire de,sof,eol,hs,vs;
 video_raster #(.H_ACTIVE(1280),.H_FRONT(110),.H_SYNC(40),.H_BACK(220),.V_ACTIVE(720),.V_FRONT(5),.V_SYNC(5),.V_BACK(20),.H_POL(1),.V_POL(1)) timing(
 .clk_pix(pixel),.rst_n(pixel_rst_n),.enable(1'b1),.x(x),.y(y),.de(de),.sof(sof),.eol(eol),.hsync(hs),.vsync(vs));
 wire [15:0] image_pixel;wire image_valid;wire [31:0] underflows,source_frame_id,source_start_ms,local_source_session;wire source_valid;
 display_line_buffer #(.SCREEN_WIDTH(1280),.SCREEN_HEIGHT(720),.DEST_WIDTH(1280),.DEST_HEIGHT(720),.OFFSET_X(0),.OFFSET_Y(0),.SOURCE_WIDTH(1280),.SOURCE_HEIGHT(960),.SOURCE_Y_OFFSET(120),.SOURCE_WINDOW_HEIGHT(720)) lines(
 .pixel_clk(pixel),.memory_clk(memory_clk),.rst_n(sys_rst_n&&pll_v&&memory_rst_n),.x(x),.y(y),.de(de),
 .pixel_data(image_pixel),.pixel_valid(image_valid),.underflows(underflows),.source_frame_id(source_frame_id),.source_frame_valid(source_valid),
 .source_start_ms(source_start_ms),.mem_start_ms(line_start_ms),.mem_request(line_request),.mem_ready(line_ready),.request_new_frame(line_new),
 .request_row(line_row),.request_bank(line_request_bank),.mem_write(line_write),.mem_data(line_data),.mem_address(line_address),.mem_bank(line_bank),
 .mem_done(line_done),.mem_good(line_good),.mem_frame_id(line_frame_id),.mem_session(line_session),.source_session(local_source_session));
 wire [15:0] peer_image;wire peer_image_valid,peer_source_valid;
 wire [31:0] peer_underflows,peer_source_frame,peer_source_start_ms,peer_source_session;

 // Host0c00: B HDMI diagnostic source0=B native,1=A raw peer. This mux is
 // only at TMDS; local_rgb, AE, NV32 and canonical B tokens are untouched.
 reg calibration_peer_requested;
 wire calibration_peer_pixel,calibration_peer_ack,calibration_peer_busy;
 wire calibration_peer_write=probe_write&&reg_addr==16'h0c00&&write_data[7:1]==0&&!calibration_peer_busy;
 always @(posedge board50_global)begin
  if(!sys_rst_n)calibration_peer_requested<=0;
  else if(calibration_peer_write)calibration_peer_requested<=write_data[0];
 end
 cdc_mode #(.WIDTH(1))calibration_view(.src_clk(board50_global),.dst_clk(pixel),.rst_n(sys_rst_n&&pll_v),
  .write(calibration_peer_write),.mode(write_data[0]),.busy(calibration_peer_busy),.commit(x==0&&y==720),.destination_mode(calibration_peer_pixel),.src_mode(calibration_peer_ack));
 display_line_buffer #(.SCREEN_WIDTH(1280),.SCREEN_HEIGHT(720),.DEST_WIDTH(1280),.DEST_HEIGHT(720),.OFFSET_X(0),.OFFSET_Y(0),.SOURCE_WIDTH(1280),.SOURCE_HEIGHT(720)) calibration_lines(
  .pixel_clk(pixel),.memory_clk(memory_clk),.rst_n(sys_rst_n&&pll_v&&memory_rst_n),
  .x(calibration_peer_pixel?x:16'hffff),.y(calibration_peer_pixel?y:16'hffff),.de(de&&calibration_peer_pixel),
  .pixel_data(peer_image),.pixel_valid(peer_image_valid),.underflows(peer_underflows),.source_frame_id(peer_source_frame),.source_frame_valid(peer_source_valid),.source_session(peer_source_session),
  .source_start_ms(peer_source_start_ms),.mem_start_ms(peer_line_start_ms),.mem_request(peer_line_request),.mem_ready(peer_line_ready),.request_new_frame(peer_line_new),.request_row(peer_line_row),.request_bank(peer_line_request_bank),
  .mem_write(peer_line_write),.mem_data(peer_line_data),.mem_address(peer_line_address),.mem_bank(peer_line_bank),.mem_done(peer_line_done),.mem_good(peer_line_good),.mem_frame_id(peer_line_frame_id),.mem_session(peer_line_session));
 wire[159:0]calibration_host_status;wire[129:0]calibration_compact_status;wire calibration_host_busy;
 assign calibration_host_status={30'd0,calibration_compact_status};
 cdc_status_snapshot #(.WIDTH(130))calibration_snapshot(.source_clk(pixel),.read_clk(board50_global),.rst_n(sys_rst_n&&pll_v),
  .take(probe_write&&reg_addr==16'h0c01&&write_data==1),
  .source_data({calibration_peer_pixel,peer_source_valid,peer_underflows,peer_source_session,peer_source_frame,source_frame_id}),
  .busy(calibration_host_busy),.read_data(calibration_compact_status),.completed_sequence());
 // The optional D2 HDMI preview is retired; D4/D5 Z+RGB transport remains.
 assign depth_image=0;assign depth_image_valid=0;assign depth_underflows=0;
 assign depth_source_frame=0;assign depth_source_valid=0;assign depth_source_session=0;assign depth_source_start=0;
 assign display_depth_bank=0;

 wire hd_capture_canceled,hd_completed_canceled;
 single_capture_gate #(.REQUIRE_QUIESCENT(1))capture_gate(.board_clk(board50_global),.memory_clk(memory_clk),
  .rst_n(sys_rst_n&&memory_ready_sync[1]),.memory_rst_n(memory_rst_n),
  .request(bt_capture_accept||depth_key_accept||(depth_host_accept&&write_data[0])),
  .cancel(bt_exit_request||control_key_accept||(depth_host_accept&&!write_data[0])||tracking_requested),
  .pipeline_quiescent(metric_memory_quiescent),.canceled_started_job(hd_capture_canceled),
  .jobs_started(depth_jobs_started),.jobs_completed(hd_confirmed_job),.ready(capture_gate_ready),.run_budget(capture_budget),.busy(capture_busy_internal));
 cloud_line_arbiter cloud_reads(.clk(memory_clk),.rst_n(memory_rst_n),
  .d_request(display_depth_request),.d_new(display_depth_new),.d_bank(display_depth_bank),.d_row(display_depth_row),
  .d_ready(display_depth_ready),.d_write(display_depth_write),.d_done(display_depth_done),
  .c_request(cloud_line_request),.c_new(cloud_line_new),.c_bank(tr_bank),.c_row(cloud_line_row),
  .c_ready(cloud_line_ready),.c_write(cloud_line_write),.c_done(cloud_line_done),
  .m_request(depth_line_request),.m_new(depth_line_new),.m_bank(depth_line_bank_request),.m_row(depth_line_row),
  .m_ready(depth_line_ready),.m_write(depth_line_write),.m_done(depth_line_done),.active(cloud_read_active),.owner(cloud_read_owner));
 // No 3D framebuffer on B: transmit measured Z and its same-source RGB.
 wire tr_new,tr_ready,tr_write,tr_done,tr_good,tr_valid,tr_frame_valid;
 wire[15:0]tr_row,tr_rgb;wire[31:0]tr_frame,tr_session,tr_underflows;
 wire tr_request;wire[31:0]tr_start;
 reg tr_ready_memory;(* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]tr_ready_sync;
 always @(posedge memory_clk)if(!memory_rst_n)tr_ready_memory<=0;
 else tr_ready_memory<=cloud_settle_jobs==0&&metric_memory_quiescent;
 always @(posedge pixel)if(!pixel_rst_n)tr_ready_sync<=0;
 else tr_ready_sync<={tr_ready_sync[0],tr_ready_memory};

 wire hd_send_valid,hd_send_ready,hd_reply_valid,hd_reply_consume,hd_payload_write,hd_sender_busy,hd_sender_done,hd_sender_fault;
 wire[7:0]hd_send_kind,hd_payload_data;wire[31:0]hd_send_job,hd_retry_count;
 wire[15:0]hd_send_sequence;wire[10:0]hd_payload_address;wire[191:0]hd_reply_header;
 hd_model_sender #(.CANCEL_PROTOCOL(1))hd_transport(.clk(memory_clk),.rst_n(memory_rst_n),.enable(memory_verified),.cancel_job(hd_capture_canceled),.completed_canceled(hd_completed_canceled),.result_published(depth_jobs_completed),
  .busy(hd_sender_busy),.done(hd_sender_done),.fault(hd_sender_fault),.completed_job(hd_completed_job),.retries(hd_retry_count),
  .line_request(cloud_line_request),.line_ready(cloud_line_ready),.line_new(cloud_line_new),.line_row(cloud_line_row),.line_bank(tr_bank),
  .line_write(cloud_line_write),.line_data(depth_line_data),.line_address(depth_line_address),.line_done(cloud_line_done),.line_good(depth_line_good),.line_job(depth_line_frame),
  .payload_write(hd_payload_write),.payload_address(hd_payload_address),.payload_data(hd_payload_data),
  .send_valid(hd_send_valid),.send_ready(hd_send_ready),.send_kind(hd_send_kind),.send_job(hd_send_job),.send_sequence(hd_send_sequence),
  .reply_valid(hd_reply_valid),.reply_header(hd_reply_header),.reply_consume(hd_reply_consume));
 assign tr_rgb=0;assign tr_valid=0;assign tr_underflows=0;assign tr_frame=0;assign tr_frame_valid=0;assign tr_session=0;assign tr_start=0;
 assign cloud_image=0;assign cloud_image_valid=0;assign cloud_source_valid=0;
 assign cloud_source_job=0;assign cloud_render_count=0;assign cloud_reject_count=0;assign cloud_point_count=0;
 assign cloud_fault=hd_sender_fault;assign cloud_view=0;assign cloud_state=0;
 always @(posedge pixel)if(!pixel_rst_n)display_cloud_active<=0;else display_cloud_active<=0;
 // The first result row has a full blanking line to arrive. Selection is
 // frozen before visible pixels, never at the first active pixel itself.
 always @(posedge pixel)if(!pixel_rst_n)display_depth_active<=0;
 else if(x==0&&y==721)display_depth_active<=depth_at_pixel&&depth_source_valid;
 wire selected_remote,local_stale,peer_stale;
 stereo_view_control #(.BOARD_ID(BOARD_ID)) view_control(.clk(pixel),.rst_n(pixel_rst_n),.commit(x==0&&y==720),
  .local_valid(source_valid),.peer_valid(peer_source_valid),.local_frame(source_frame_id),.peer_frame(peer_source_frame),.peer_session(peer_source_session),
  .selected_remote(selected_remote),.local_stale(local_stale),.peer_stale(peer_stale));
 wire key_switch;wire control_busy,requested_at_pixel,control_ack;
 wire [1:0] display_at_pixel,display_ack;
 reg [1:0] display_requested;
 reg [31:0] display_accepted,display_rejected;
 wire [5:0] control_destination,control_applied;
 assign requested_at_pixel=control_destination[0];assign depth_at_pixel=control_destination[3];assign depth_ack=control_applied[3];
 assign control_ack=control_applied[0];
 assign display_at_pixel=control_destination[2:1];
 assign display_ack=control_applied[2:1];
 reg requested_remote;reg [31:0] key_switch_count;
 reg cloud_requested,orbit_requested;
 assign cloud_at_pixel=control_destination[4];assign orbit_at_pixel=control_destination[5];
 reg [31:0] control_host_accepted,control_host_rejected;
 gimbal_press_event #(.STABLE_CYCLES(500000)) switch_key(.clk(board50_global),.rst_n(sys_rst_n),.key_n(key_n[0]),.pressed(key_switch));
 // B-only host source request. KEY1 wins a simultaneous write; busy/invalid
 // writes are rejected, never queued. Even a same-value request gets an ACK.
 wire control_host_write=probe_write&&reg_addr==16'h0244;
 wire display_host_write=probe_write&&reg_addr==16'h00e3;
 assign control_key_accept=BOARD_ID==2&&!tracking_requested&&key_switch&&!control_busy;
 wire control_host_accept=BOARD_ID==2&&!tracking_requested&&control_host_write&&write_data[7:1]==0&&!key_switch&&!control_busy;
 wire display_host_accept=BOARD_ID==2&&!tracking_requested&&display_host_write&&write_data[7:2]==0&&!key_switch&&!control_busy;
 gimbal_press_event #(.STABLE_CYCLES(500000))depth_key_control(.clk(board50_global),.rst_n(sys_rst_n),.key_n(key_n[1]),.pressed(depth_key));
 wire depth_host_write=probe_write&&reg_addr==16'h02c0;
 wire depth_enter_safe=!en1&&!en2&&!step1&&!step2;
 assign depth_key_accept=depth_key&&!tracking_requested&&!control_busy&&depth_enter_safe&&capture_ready_internal;
 assign depth_host_accept=depth_host_write&&!tracking_requested&&write_data[7:1]==0&&!control_busy&&!depth_key&&(write_data[0]==0||depth_enter_safe);
 assign bt_capture_accept=bt_capture_request&&!tracking_requested&&!control_busy&&depth_enter_safe&&capture_ready_internal;
 wire depth_value=bt_capture_accept||depth_key_accept?1'b1:bt_exit_request?1'b0:write_data[0];
 wire depth_write=depth_key_accept||depth_host_accept||bt_capture_accept||bt_exit_request;


 wire cloud_key,orbit_key;
 gimbal_press_event #(.STABLE_CYCLES(500000)) cloud_capture_key(.clk(board50_global),.rst_n(sys_rst_n),.key_n(1'b1),.pressed(cloud_key));
 gimbal_press_event #(.STABLE_CYCLES(500000)) cloud_orbit_key(.clk(board50_global),.rst_n(sys_rst_n),.key_n(1'b1),.pressed(orbit_key));
 wire cloud_host_write=probe_write&&reg_addr==16'h03a0;
 wire cloud_key_accept=cloud_key&&!control_busy&&!depth_key&&!key_switch;
 wire orbit_key_accept=orbit_key&&!control_busy&&!cloud_key&&!depth_key&&!key_switch;
 wire cloud_host_accept=cloud_host_write&&!tracking_requested&&write_data[7:2]==0&&!control_busy&&!cloud_key&&!orbit_key&&!depth_key&&!key_switch;
 wire cloud_control_write=cloud_key_accept||orbit_key_accept||cloud_host_accept;
 wire cloud_value=cloud_key_accept?!cloud_requested:cloud_host_accept?write_data[0]:cloud_requested;
 wire orbit_value=orbit_key_accept?!orbit_requested:cloud_host_accept?write_data[1]:orbit_requested;
 wire control_write=control_key_accept||control_host_accept||display_host_accept||depth_write||cloud_control_write||(!control_busy&&control_applied!={orbit_requested,cloud_requested,depth_requested,display_requested,requested_remote});
 wire control_value=control_key_accept?!requested_remote:write_data[0];
 wire[5:0]control_payload=tracking_requested?6'd0:{cloud_control_write?orbit_value:orbit_requested,
  control_key_accept?1'b0:depth_write?depth_value:cloud_control_write?cloud_value:cloud_requested,
  control_key_accept?1'b0:cloud_control_write&&cloud_value?1'b1:depth_write?depth_value:depth_requested,
  display_host_accept?write_data[1:0]:display_requested,(control_key_accept||control_host_accept)?control_value:requested_remote};
 always @(posedge board50_global)begin
  if(!sys_rst_n)begin requested_remote<=0;key_switch_count<=0;control_host_accepted<=0;control_host_rejected<=0;
   display_requested<=0;display_accepted<=0;display_rejected<=0;depth_requested<=0;cloud_requested<=0;orbit_requested<=0;end
  else begin
   if(control_key_accept||control_host_accept)requested_remote<=control_value;
   if(display_host_accept)display_requested<=write_data[1:0];
   if(control_key_accept)begin depth_requested<=0;cloud_requested<=0;end
   else if(depth_write)begin depth_requested<=depth_value;cloud_requested<=depth_value;if(depth_value)orbit_requested<=0;end
   else if(cloud_control_write)begin cloud_requested<=cloud_value;orbit_requested<=orbit_value;if(cloud_value)depth_requested<=1;end
   if(tracking_requested)begin requested_remote<=0;display_requested<=0;depth_requested<=0;cloud_requested<=0;orbit_requested<=0;end
   if(display_host_accept)display_accepted<=display_accepted+1'b1;
   if(display_host_write&&!display_host_accept)display_rejected<=display_rejected+1'b1;
   if(control_key_accept)key_switch_count<=key_switch_count+1'b1;
   if(control_host_accept)control_host_accepted<=control_host_accepted+1'b1;
   if(control_host_write&&!control_host_accept)control_host_rejected<=control_host_rejected+1'b1;
  end
 end
 // One held three-bit payload crosses the existing handshake. Source request
 // remains bit0; profile bits commit atomically in the same vertical blank.
 cdc_mode #(.WIDTH(6)) control_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(sys_rst_n&&pll_v),
  .write(control_write),.mode(control_payload),.busy(control_busy),.commit(x==0&&y==720),
  .destination_mode(control_destination),.src_mode(control_applied));
 // The acknowledged pixel-domain static-mode request reaches DHI through
 // a second coherent mailbox. Disabling drains the current complete job.
 cdc_mode #(.WIDTH(1))depth_run_mode(.src_clk(pixel),.dst_clk(memory_clk),.rst_n(sys_rst_n&&pll_v&&memory_rst_n),
  .write(!depth_run_busy&&depth_run_ack!=depth_at_pixel),.mode(depth_at_pixel),.busy(depth_run_busy),.commit(1'b1),
  .destination_mode(depth_run_at_memory),.src_mode(depth_run_ack));

 wire cloud_run_busy;wire[1:0]cloud_run_ack,cloud_memory_mode;
 assign cloud_at_memory=cloud_memory_mode[0];assign orbit_at_memory=cloud_memory_mode[1];
 cdc_mode #(.WIDTH(2))cloud_run_mode(.src_clk(pixel),.dst_clk(memory_clk),.rst_n(sys_rst_n&&pll_v&&memory_rst_n),
  .write(!cloud_run_busy&&cloud_run_ack!={orbit_at_pixel,cloud_at_pixel}),.mode({orbit_at_pixel,cloud_at_pixel}),
  .busy(cloud_run_busy),.commit(1'b1),.destination_mode(cloud_memory_mode),.src_mode(cloud_run_ack));
 wire[127:0]cloud_snapshot;wire[4:0]cloud_compact_status;wire cloud_snapshot_busy;
 assign cloud_snapshot={27'd0,cloud_compact_status,96'd0};
 cdc_status_snapshot #(.WIDTH(5))cloud_capture(.source_clk(memory_clk),.read_clk(board50_global),.rst_n(sys_rst_n&&memory_rst_n),
  .take(probe_write&&reg_addr==16'h03a1),.source_data({capture_budget,cloud_fault,cloud_read_owner,cloud_read_active,metric_memory_quiescent}),.busy(cloud_snapshot_busy),.read_data(cloud_compact_status),.completed_sequence());
 // NR changes commit in vertical blank; requested/applied/busy are separately readable.
 // Two source-clock release stages also reset the startup request after video PLL loss.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] noise_control_reset;
 wire noise_raw_reset=sys_rst_n&&pll_v;
 always @(posedge board50_global or negedge noise_raw_reset)
  if(!noise_raw_reset)noise_control_reset<=0;else noise_control_reset<={noise_control_reset[0],1'b1};
 wire noise_write,noise_value,noise_requested,noise_busy,noise_ack,noise_at_pixel;
 wire [31:0] noise_key_count;
 video_noise_control noise_control(.clk(board50_global),.rst_n(noise_control_reset[1]),.key_n(1'b1),
  .host_write(probe_write&&reg_addr==16'h0280),.host_data(write_data),.busy(noise_busy),.applied_mode(noise_ack),
  .mode_write(noise_write),.mode_value(noise_value),.requested_mode(noise_requested),.key_count(noise_key_count));
 cdc_mode #(.WIDTH(1)) noise_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(noise_raw_reset),
  .write(noise_write),.mode(noise_value),.busy(noise_busy),.commit(x==0&&y==720),
  .destination_mode(noise_at_pixel),.src_mode(noise_ack));
 // Both image line caches align to this one-pixel-delayed raster.
 reg [31:0] display_source_frame_id;reg display_source_frame_valid;reg [31:0] displayed_frames;
 always @(posedge pixel)if(!pixel_rst_n)begin display_de<=0;display_hs<=1;display_vs<=1;displayed_frames<=0;display_x<=0;display_y<=0;display_source_frame_id<=0;display_source_frame_valid<=0;end
 else begin display_x<=x;display_y<=y;
 if(x==0&&y==0)begin display_source_frame_id<=source_frame_id;display_source_frame_valid<=source_valid;end
 display_de<=de;display_hs<=hs;display_vs<=vs;if(sof)displayed_frames<=displayed_frames+1'b1;end
 wire [23:0] local_rgb=image_valid?{image_pixel[15:11],image_pixel[15:13],image_pixel[10:5],image_pixel[10:9],image_pixel[4:0],image_pixel[4:2]}:24'h000000;
 wire[23:0]calibration_hdmi_rgb=calibration_peer_pixel?(peer_image_valid?{peer_image[15:11],peer_image[15:13],peer_image[10:5],peer_image[10:9],peer_image[4:0],peer_image[4:2]}:24'd0):local_rgb;

 // Register302: bit0 requested bypass, bit1 acknowledged bypass, bit2 busy.
 // Zero at reset means automatic histogram enhancement enabled.
 wire hist_bypass_pixel,hist_bypass_ack,hist_bypass_busy;
 reg hist_bypass_requested;
 wire hist_mode_write=probe_write&&reg_addr==16'h0302&&write_data[7:1]==0&&!hist_bypass_busy;
 always @(posedge board50_global)if(!noise_control_reset[1])hist_bypass_requested<=0;
 else if(hist_mode_write)hist_bypass_requested<=write_data[0];
 cdc_mode #(.WIDTH(1))hist_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(noise_raw_reset),
 .write(hist_mode_write),.mode(write_data[0]),.busy(hist_bypass_busy),.commit(x==0&&y==720),
 .destination_mode(hist_bypass_pixel),.src_mode(hist_bypass_ack));
 wire[15:0]hist_updates;wire hist_confidence,hist_busy;
 wire[11:0]hist_samples;wire[7:0]hist_low,hist_high;
 wire[63:0]hist_control_status;wire hist_capture_busy,hist_constants_ready;
 assign hist_control_status=hist_constants_ready?{18'd0,1'b0,8'hff,8'd0,12'd0,1'b0,16'd0}:64'd0;
 cdc_status_snapshot #(.WIDTH(1))hist_capture(.source_clk(pixel),.read_clk(board50_global),.rst_n(sys_rst_n&&pll_v),
 .take(!hist_capture_busy),.source_data(1'b1),
 .busy(hist_capture_busy),.read_data(hist_constants_ready),.completed_sequence());
 wire [23:0] local_chroma,local_tone;wire chroma_valid,tone_valid;
 wire [1:0] chroma_profile=display_at_pixel==1?2'd0:2'd1;
 wire [1:0] tone_profile=display_at_pixel==1?2'd0:((display_at_pixel==2||display_at_pixel==3)?2'd3:2'd2);
 // Stereo profile sends native AWB-corrected RGB565. The main board owns OSD/algorithms.
 reg[23:0]stereo_rgb_delay[0:5];reg[5:0]stereo_valid_delay;integer stereo_delay_index;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin stereo_valid_delay<=0;for(stereo_delay_index=0;stereo_delay_index<6;stereo_delay_index=stereo_delay_index+1)stereo_rgb_delay[stereo_delay_index]<=0;end
  else begin stereo_rgb_delay[0]<=local_rgb;stereo_valid_delay<={stereo_valid_delay[4:0],display_de};
   for(stereo_delay_index=1;stereo_delay_index<6;stereo_delay_index=stereo_delay_index+1)stereo_rgb_delay[stereo_delay_index]<=stereo_rgb_delay[stereo_delay_index-1];end
 end
 assign local_chroma=stereo_rgb_delay[1];assign chroma_valid=stereo_valid_delay[1];
 assign local_tone=stereo_rgb_delay[5];assign tone_valid=stereo_valid_delay[5];
 assign hist_updates=0;assign hist_confidence=0;assign hist_samples=0;assign hist_low=0;assign hist_high=255;assign hist_busy=0;
 reg [15:0] colour_x[0:5],colour_y[0:5],remote_pixels[0:5];
 reg [5:0] colour_de,colour_hs,colour_vs,local_valids,remote_valids;
integer colour_index;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin
   colour_de<=0;colour_hs<=6'b111111;colour_vs<=6'b111111;local_valids<=0;remote_valids<=0;
   for(colour_index=0;colour_index<6;colour_index=colour_index+1)begin colour_x[colour_index]<=0;colour_y[colour_index]<=0;remote_pixels[colour_index]<=0;end
  end else begin
   colour_de<={colour_de[4:0],display_de};colour_hs<={colour_hs[4:0],display_hs};colour_vs<={colour_vs[4:0],display_vs};
   local_valids<={local_valids[4:0],image_valid};remote_valids<={remote_valids[4:0],display_cloud_active?cloud_image_valid:display_depth_active?depth_image_valid:peer_image_valid};
   colour_x[0]<=display_x;colour_y[0]<=display_y;remote_pixels[0]<=display_cloud_active?cloud_image:display_depth_active?depth_image:peer_image;
   for(colour_index=1;colour_index<6;colour_index=colour_index+1)begin colour_x[colour_index]<=colour_x[colour_index-1];colour_y[colour_index]<=colour_y[colour_index-1];remote_pixels[colour_index]<=remote_pixels[colour_index-1];end
  end
 end
 // Filter once on the local display/transmit branch; camera DDR and AE meter stay unfiltered.
 wire [23:0] filtered_local;wire [15:0] filtered_x,filtered_y;
 wire filtered_de,filtered_valid,filtered_hs,filtered_vs;
 video_denoise_2row #(.WIDTH(1280),.HEIGHT(720),.OFFSET_X(0),.THRESHOLD_2D(48),.THRESHOLD_1D(32)) display_noise(
  .clk(pixel),.rst_n(pixel_rst_n),.enable(1'b0),.rgb_in(local_tone),.x_in(colour_x[5]),.y_in(colour_y[5]),
  .de_in(colour_de[5]),.valid_in(local_valids[5]),.hs_in(colour_hs[5]),.vs_in(colour_vs[5]),
  .rgb_out(filtered_local),.x_out(filtered_x),.y_out(filtered_y),.de_out(filtered_de),
  .valid_out(filtered_valid),.hs_out(filtered_hs),.vs_out(filtered_vs));
 // V11 candidate: mode2 and mode3 have identical chroma/tone. Mode2 adds the
 // optional cap128 median only when the existing pixel-domain NR is enabled.
 // Old filtered_* and display_noise remain intact for independent review.
 wire dark_enable=noise_at_pixel&&(display_at_pixel==2);
 wire [23:0] post_local;wire [15:0] post_x,post_y;
 wire post_de,post_valid,post_hs,post_vs;
 video_dark_median_3tap #(.WIDTH(1280),.HEIGHT(720),.OFFSET_X(0)) display_dark(
  .clk(pixel),.rst_n(pixel_rst_n),.enable(1'b0),.rgb_in(filtered_local),.x_in(filtered_x),.y_in(filtered_y),
  .de_in(filtered_de),.valid_in(filtered_valid),.hs_in(filtered_hs),.vs_in(filtered_vs),
  .rgb_out(post_local),.x_out(post_x),.y_out(post_y),.de_out(post_de),.valid_out(post_valid),.hs_out(post_hs),.vs_out(post_vs));
 // Received image already carries its source-board filter. Delay only, never filter it twice.
 reg [15:0] remote565_read,remote565_stage,remote565;
 reg peer_valid_read,peer_valid_stage,filtered_peer_valid;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin remote565_read<=0;remote565_stage<=0;remote565<=0;
   peer_valid_read<=0;peer_valid_stage<=0;filtered_peer_valid<=0;end
  else begin remote565_read<=remote_pixels[5];remote565_stage<=remote565_read;remote565<=remote565_stage;
   peer_valid_read<=remote_valids[5];peer_valid_stage<=peer_valid_read;filtered_peer_valid<=peer_valid_stage;end
 end
 wire [23:0] remote_rgb={remote565[15:11],remote565[15:13],remote565[10:5],remote565[10:9],remote565[4:0],remote565[4:2]};
 // Match the new local module's D=2 on the received branch, delay only.
 reg [15:0] remote565_post0,remote565_post;reg peer_valid_post0,peer_valid_post;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin remote565_post0<=0;remote565_post<=0;peer_valid_post0<=0;peer_valid_post<=0;end
  else begin remote565_post0<=remote565;remote565_post<=remote565_post0;
   peer_valid_post0<=filtered_peer_valid;peer_valid_post<=peer_valid_post0;end
 end
 wire [23:0] post_remote_rgb={remote565_post[15:11],remote565_post[15:13],remote565_post[10:5],remote565_post[10:9],remote565_post[4:0],remote565_post[4:2]};
 // Demo relay: HDMI always shows A's final picture; local TX remains post_local.
 wire [23:0] rgb=peer_valid_post&&(display_metric_active||!peer_stale)?post_remote_rgb:24'd0;
 wire [23:0] sample_rgb;wire [15:0] sample_x,sample_y;wire sample_de;
 stereo_sample_720p sample(.clk(pixel),.rst_n(pixel_rst_n),.x(post_x),.y(post_y),.de(post_de),
  .valid(!depth_epoch_exhausted&&(display_metric_active?peer_valid_post:(post_valid&&display_source_frame_valid&&!local_stale))),.rgb(display_metric_active?post_remote_rgb:post_local),.out_x(sample_x),.out_y(sample_y),.out_de(sample_de),.out_rgb(sample_rgb));
 wire queue_reset_n=sys_rst_n&&phy_ready&&pll_v;
 wire tx_packet_valid,tx_packet_ready,tx_packet_done,tx_packet_good,queue_tx_reset_n;
 wire [31:0] tx_session,tx_frame,queued_frames,queued_drops,tx_frame_counter,sent_packets,sent_frames,last_sent_frame,mac_packets,mac_drops;
 wire [15:0] tx_packet_index,tx_word_data;wire [9:0] tx_word_addr;wire [2:0] tx_high_water;
 wire request_or_ack=BOARD_ID==1?selected_remote:requested_at_pixel;
 wire[21:0]typed_epoch;wire typed_epoch_exhausted;
 nv32_session_epoch typed_epoch_counter(.clk(pixel),.rst_n(pixel_rst_n),.frame_origin(x==0&&y==0),
  .context_word({26'd0,display_depth_active,depth_at_pixel,requested_at_pixel,noise_at_pixel,display_at_pixel}),.epoch(typed_epoch),.exhausted(typed_epoch_exhausted));
 // D2 frame32 is transport refresh sequence. Nonzero session epoch22 is the
 // actual computed depth job ID; repeated video must not imply30 depth updates/s.
 assign depth_epoch_exhausted=typed_epoch_exhausted||(display_metric_active&&(shown_metric_job[31:22]!=0||shown_metric_job[21:0]==0));
 wire[31:0]source_session_word={display_cloud_active?(orbit_at_pixel?8'hD4:8'hD5):display_depth_active?8'hD2:8'hB2,depth_at_pixel,(requested_at_pixel||depth_at_pixel),display_metric_active?shown_metric_job[21:0]:typed_epoch};
 wire[31:0]typed_frame=display_metric_active?displayed_frames:display_source_frame_id;
 nv_display_queue #(.WIDTH(1280),.HEIGHT(720),.PACKET_PIXELS(640),.DYNAMIC_META(1)) queue(
 .pixel_clk(pixel),.tx_clk(tx_clk),.rst_n(queue_reset_n),.tx_domain_rst_n(queue_tx_reset_n),
 .rgb(sample_rgb),.x(sample_x),.y(sample_y),.de(sample_de),.source_frame_id(typed_frame),.source_session(source_session_word),
 .packet_valid(tx_packet_valid),.packet_ready(tx_packet_ready),.packet_session(tx_session),.packet_frame(tx_frame),.packet_index(tx_packet_index),
 .packet_word_addr(tx_word_addr),.packet_word_data(tx_word_data),.packet_done(tx_packet_done),.packet_good(tx_packet_good),
 .frames_queued(queued_frames),.frames_dropped(queued_drops),.frame_counter(tx_frame_counter),.packets_sent(sent_packets),.frames_sent(sent_frames),.last_frame_sent(last_sent_frame),.high_water(tx_high_water));

 wire bt_control_valid,bt_control_request,bt_control_busy,bt_control_enable;
 wire[7:0]bt_control_data,bt_video_data;wire bt_video_enable,bt_video_admit,bt_video_ready,bt_video_busy;
 wire[255:0]bt_payload;wire bt_mailbox_busy;reg bt_publish;reg[47:0]bt_last_sent,bt_publish_record;reg[24:0]bt_retry_age;
 always @(posedge board50_global)begin
  if(!sys_rst_n)begin bt_publish<=0;bt_last_sent<=0;bt_publish_record<=0;bt_retry_age<=0;end
  else begin
   bt_publish<=0;if(bt_retry_age!=24999999)bt_retry_age<=bt_retry_age+1'b1;
   if(!bt_mailbox_busy&&bt_view_record[40:25]!=0&&(bt_last_sent!=bt_view_record||bt_retry_age==24999999))begin
    bt_publish<=1;bt_last_sent<=bt_view_record;bt_publish_record<=bt_view_record;bt_retry_age<=0;
   end
  end
 end
 wire[40:0]bt_held_record;
 assign bt_payload={32'h56494557,7'd0,bt_held_record,2'b11,174'd0};
 follow_payload_mailbox #(.WIDTH(41))bt_view_mailbox(.source_clk(board50_global),.destination_clk(tx_clk),.rst_n(queue_reset_n),
  .source_write(bt_publish),.source_data(bt_publish_record[40:0]),.source_busy(bt_mailbox_busy),
  .valid(bt_control_valid),.data(bt_held_record),.consume(bt_control_request));
 follow_video_arbiter bt_arbiter(.clk(tx_clk),.rst_n(queue_tx_reset_n),
  .video_valid(tx_packet_valid&&legacy_gate),.video_ready(bt_video_ready),.video_busy(bt_video_busy),.video_data(bt_video_data),.video_enable(bt_video_enable),
  .control_valid(bt_control_valid&&legacy_gate),.control_busy(bt_control_busy),.control_data(bt_control_data),.control_enable(bt_control_enable),
  .video_admit(bt_video_admit),.video_accept(tx_packet_ready),.control_request(bt_control_request),.control_accept(),.gmii_data(legacy_gmii_data),.gmii_enable(legacy_gmii_enable));

 hd_link #(.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC))hd_network(.memory_clk(memory_clk),.tx_clk(tx_clk),.rx_clk(rx_clk),.rst_n(sys_rst_n&&memory_rst_n&&pll_v),
  .payload_write(hd_payload_write),.payload_address(hd_payload_address),.payload_data(hd_payload_data),.send_valid(hd_send_valid),.send_ready(hd_send_ready),.send_kind(hd_send_kind),.send_job(hd_send_job),.send_sequence(hd_send_sequence),
  .packet_valid(hd_reply_valid),.packet_header(hd_reply_header),.packet_consume(hd_reply_consume),.read_address(11'd0),.read_data(),
  .tx_offer(hd_tx_offer),.tx_accept(hd_tx_accept),.tx_busy(hd_tx_busy),.gmii_txd(hd_gmii_data),.gmii_tx_en(hd_gmii_enable),
  .gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),.rx_good(),.rx_bad_crc(),.rx_rejected());
 hd_wire_arbiter hd_wire(.clk(tx_clk),.rst_n(sys_rst_n&&memory_rst_n&&pll_v),.enabled(phy_ready),
  .legacy_busy(bt_video_busy||bt_control_busy),.legacy_data(legacy_gmii_data),.legacy_enable(legacy_gmii_enable),.legacy_gate(legacy_gate),
  .hd_offer(hd_tx_offer),.hd_busy(hd_tx_busy),.hd_data(hd_gmii_data),.hd_enable(hd_gmii_enable),.hd_accept(hd_tx_accept),.gmii_data(gmii_txd),.gmii_enable(gmii_tx_en));
 nv32_video_tx #(.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC),.WIDTH(1280),.HEIGHT(720),.PACKET_PIXELS(640),.STREAM_ID(LOCAL_STREAM),.HELD_PACKET_METADATA(1)) transmitter(
 .clk(tx_clk),.rst_n(queue_tx_reset_n),.packet_valid(bt_video_admit),.packet_ready(bt_video_ready),.packet_session(tx_session),.packet_frame(tx_frame),.packet_index(tx_packet_index),
 .packet_word_addr(tx_word_addr),.packet_word_data(tx_word_data),.packet_done(tx_packet_done),.packet_good(tx_packet_good),.busy(bt_video_busy),.tx_packets(mac_packets),.tx_dropped(mac_drops),.gmii_txd(bt_video_data),.gmii_tx_en(bt_video_enable));
 wire [9:0] ch0,ch1,ch2;
 // Rare one-bit status is synchronized before the pixel-domain HUD.
 // Stage0 has exactly one consumer (stage1); only stage0 is a CDC entry.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg[1:0]depth_fault_sync;
 always @(posedge pixel)begin
  if(!pixel_rst_n)depth_fault_sync<=2'b11;
  else depth_fault_sync<={depth_fault_sync[0],depth_system_fault||depth_config_error};
 end
 // HDMI-only status; canonical/network samples above remain unmodified.
 wire[23:0]hdmi_rgb;wire hdmi_de,hdmi_hs,hdmi_vs;
 video_depth_status #(.FONT_FILE("V74_2_Stereo/project/rtl/depth_ascii_8x16.mem")) depth_status(
  .clk(pixel),.rst_n(pixel_rst_n),.enable(display_depth_active&&!display_cloud_active),.x(post_x),.y(post_y),.rgb(rgb),.de(post_de),.hs(post_hs),.vs(post_vs),
  .job_valid(depth_source_valid&&!depth_epoch_exhausted&&depth_source_frame!=0),.link_ok(!depth_fault_sync[1]&&!peer_stale&&!local_stale),.job_id(depth_source_frame[21:0]),
  .out_rgb(hdmi_rgb),.out_de(hdmi_de),.out_hs(hdmi_hs),.out_vs(hdmi_vs),
  .seen_job(),.gap_known(),.shown_job(),.gap_frames());
 // B-local diagnostic preview: reuse the existing native line cache/raster.
 // No peer cache, extra framebuffer, OSD or second cloud renderer is enabled.
 tmds_encoder enc0(.clk(pixel),.rst_n(pixel_rst_n),.data(calibration_hdmi_rgb[7:0]),.de(display_de),.ctrl({display_vs,display_hs}),.symbol(ch0));
 tmds_encoder enc1(.clk(pixel),.rst_n(pixel_rst_n),.data(calibration_hdmi_rgb[15:8]),.de(display_de),.ctrl(2'b0),.symbol(ch1));
 tmds_encoder enc2(.clk(pixel),.rst_n(pixel_rst_n),.data(calibration_hdmi_rgb[23:16]),.de(display_de),.ctrl(2'b0),.symbol(ch2));
 ph1p_tmds_output output_phy(.pixel(pixel),.serial(serial),.reset(!pixel_rst_n),
  .ch0(ch0),.ch1(ch1),.ch2(ch2),.data_p(hdmi_data_p),.data_n(hdmi_data_n),
  .clock_p(hdmi_clock_p),.clock_n(hdmi_clock_n));

    // 用专用I/O双沿寄存器转发时钟，避免PLL时钟经普通数据布线直接到输出。
    reg [3:0] camera_reset;
    wire camera_async_reset=!sys_rst_n || !pll_c;
    always @(posedge camera_clock_internal or posedge camera_async_reset) begin
        if(camera_async_reset) camera_reset<=0; else camera_reset<={camera_reset[2:0],1'b1};
    end
    PH1P_LOGIC_HD_ODDR camera_clock_output(.clk(camera_clock_internal),.rst(!camera_reset[3]),
        .d0(1'b1),.d1(1'b0),.q(camera_xclk));
    // 原理图HDMI1_HPD实际连接TPS2051B使能（高有效），不是屏幕HPD输入。
    reg [1:0] pv_sync,pc_sync;
    always @(posedge board50_global) begin
        if(!sys_rst_n) begin pv_sync<=0; pc_sync<=0; end
        else begin pv_sync<={pv_sync[0],pll_v}; pc_sync<={pc_sync[0],pll_c}; end
    end
    reg [19:0] camera_wait;
    always @(posedge board50_global) begin
        if(!sys_rst_n || !pc_sync[1]) camera_wait<=0;
        else if(!camera_wait[19]) camera_wait<=camera_wait+1'b1;
    end
    assign camera_pwdn_n=camera_wait[19];
 wire cam_valid,ddc_valid,cam_read,bridge_busy;wire [6:0] dev_addr;
 wire [7:0] cam_data;wire cam_done;wire [1:0] cam_error;
 wire cam_scl_low,cam_sda_low,ae_hold_motion;
    wire boot_active,boot_valid,boot_ready,boot_read,boot_bus_done,boot_done,boot_failed,boot_stream;
    wire [15:0] boot_address,boot_chip;wire [7:0] boot_value;wire [7:0] boot_step;wire [2:0] boot_code;
    wire bus_valid,bus_ready,bus_read,bus_done;wire [6:0] bus_device;wire [15:0] bus_address;wire [7:0] bus_value;
    sensor_boot boot(.clk(board50_global),.rst_n(sys_rst_n),.power_ready(camera_pwdn_n),
        .cmd_valid(boot_valid),.cmd_ready(boot_ready),.read(boot_read),.address(boot_address),.value(boot_value),
        .cmd_done(boot_bus_done),.cmd_error(cam_error),.read_data(cam_data),.active(boot_active),
        .done(boot_done),.failed(boot_failed),.code(boot_code),.stream_start(boot_stream),.step(boot_step),.chip_id(boot_chip));
    // v13 autonomous AE: pre-OSD complete-frame statistics, atomic CDC, independent I2C owner.
    wire meter_valid;wire [19:0] meter_sum;wire [11:0] meter_count,meter_highlights;
    wire [31:0] meter_frame;wire [15:0] meter_sequence;
    exposure_meter #(.OFFSET_X(0),.IMAGE_WIDTH(1280)) meter(.clk(pixel),.rst_n(pixel_rst_n),.x(display_x),.y(display_y),.de(display_de),
        .valid(image_valid),.rgb565(image_pixel),.source_frame_id(display_source_frame_id),.source_frame_valid(display_source_frame_valid),
        .result_valid(meter_valid),.luminance_sum(meter_sum),.sample_count(meter_count),.highlight_count(meter_highlights),
        .frame_id(meter_frame),.result_sequence(meter_sequence));
    wire [92:0] ae_meter;wire ae_meter_busy;
    cdc_status_snapshot #(.WIDTH(93)) ae_capture(.source_clk(pixel),.read_clk(board50_global),.rst_n(sys_rst_n&&pll_v),
        .take(!ae_meter_busy),.source_data({meter_sequence,meter_frame,meter_highlights,meter_count,meter_sum,meter_valid}),
        .busy(ae_meter_busy),.read_data(ae_meter),.completed_sequence());
    reg ae_enable;reg [1:0] ae_target_mode;
    reg [31:0] ae_target_accepted,ae_target_rejected;
    wire ae_target_write=probe_write&&reg_addr==16'h00e2;
    wire ae_target_accept=BOARD_ID==2&&ae_target_write&&write_data[7:2]==0;
    wire ae_request,ae_grant,ae_cmd_valid,ae_cmd_ready,ae_cmd_read,ae_cmd_done;
    wire [15:0] ae_address;wire [7:0] ae_value;
    wire ae_busy,ae_fault,ae_stable,ae_limit,ae_stale;wire [7:0] ae_fault_code,ae_coarse,ae_fine;
    wire [11:0] ae_half;wire [15:0] ae_updates;wire [3:0] ae_settling;
    always @(posedge board50_global) begin
        if(!sys_rst_n) ae_enable<=1;
        else begin
            if(probe_write&&reg_addr==16'h00e0) ae_enable<=write_data[0];
            // Manual camera writes freeze AE before the queued host command is issued.
            if(cam_valid&&!cam_read) ae_enable<=0;
        end
    end
    // Threshold choice is entirely in the AE control clock domain. It cannot
    // alter a sensor group already frozen in candidate_* registers.
    always @(posedge board50_global)begin
      if(!sys_rst_n)begin ae_target_mode<=0;ae_target_accepted<=0;ae_target_rejected<=0;end
      else begin
        if(ae_target_accept)begin ae_target_mode<=write_data[1:0];ae_target_accepted<=ae_target_accepted+1'b1;end
        if(ae_target_write&&!ae_target_accept)ae_target_rejected<=ae_target_rejected+1'b1;
      end
    end
    // 2026-10-10: 30 ms exposure suppressed measured room-light banding by99%.
    // At30 fps/VTS2000,1200 half-lines =10 ms. Switch10/20/30 ms atomically
    // with gain compensation; preserve short exposure for very bright scenes.
    wire[2:0]ae_debug_state;wire[3:0]ae_debug_step;wire ae_debug_initialized;
    mcu_auto_exposure #(.FIRMWARE_FILE("V94_2_Stereo/project/rtl/mcu_ae_firmware.mem"),.SAMPLES(3600),.LOW(110),.HIGH(132),.SETTLE_FRAMES(3),.BOOT_HALF(3600),.MAX_HALF(3600),.FLICKER_HALF(1200),.RUNTIME_TARGET(1)) ae(.clk(board50_global),.rst_n(sys_rst_n),.enable(ae_enable),.boot_ready(boot_done&&!boot_failed),.target_mode(ae_target_mode),
        .meter_valid(ae_meter[0]),.meter_sum(ae_meter[20:1]),.meter_count(ae_meter[32:21]),
        .meter_highlights(ae_meter[44:33]),.meter_sequence(ae_meter[92:77]),
        .request(ae_request),.grant(ae_grant),.cmd_valid(ae_cmd_valid),.cmd_ready(ae_cmd_ready),.cmd_read(ae_cmd_read),
        .cmd_address(ae_address),.cmd_value(ae_value),.cmd_done(ae_cmd_done),.cmd_error(cam_error),.read_data(cam_data),
        .busy(ae_busy),.hold_motion(ae_hold_motion),.fault(ae_fault),.stable(ae_stable),.at_limit(ae_limit),.stale(ae_stale),
        .fault_code(ae_fault_code),.half_lines(ae_half),.gain_coarse(ae_coarse),.gain_fine(ae_fine),.updates(ae_updates),.settling(ae_settling),.debug_state(ae_debug_state),.debug_step(ae_debug_step),.debug_initialized(ae_debug_initialized));
    // E1 write freezes this 24-byte control-domain record for a coherent burst read at 0150.
    reg [191:0] ae_snapshot;
    always @(posedge board50_global) begin
        if(!sys_rst_n) ae_snapshot<=0;
        else if(probe_write&&reg_addr==16'h00e1) ae_snapshot<={ae_debug_state,ae_debug_step,ae_debug_initialized,4'd0,ae_settling,ae_meter[92:77],ae_updates,
            ae_meter[76:45],4'd0,ae_meter[44:33],4'd0,ae_meter[32:21],4'd0,ae_meter[20:1],
            ae_fine,ae_coarse,4'd0,ae_half,ae_fault_code,
            1'b0,ae_stale,ae_stable,ae_limit,(ae_settling!=0),ae_fault,ae_busy,ae_enable};
    end
    camera_ae_mux camera_owner(.clk(board50_global),.rst_n(sys_rst_n),.boot_active(boot_active),
        .boot_valid(boot_valid),.boot_ready(boot_ready),.boot_read(boot_read),.boot_address(boot_address),.boot_value(boot_value),.boot_done(boot_bus_done),
        .host_valid(cam_valid),.host_read(cam_read),.host_device(dev_addr),.host_address(reg_addr),.host_value(write_data),.host_done(cam_done),
        .ae_request(ae_request),.ae_grant(ae_grant),.ae_valid(ae_cmd_valid),.ae_ready(ae_cmd_ready),.ae_read(ae_cmd_read),
        .ae_address(ae_address),.ae_value(ae_value),.ae_done(ae_cmd_done),
        .bus_valid(bus_valid),.bus_ready(bus_ready),.bus_read(bus_read),.bus_device(bus_device),.bus_address(bus_address),.bus_value(bus_value),.bus_done(bus_done));
    sensor_i2c_register cam_i2c(.clk(board50_global),.rst_n(sys_rst_n),.cmd_valid(bus_valid),.cmd_ready(bus_ready),
        .read(bus_read),.dev_addr(bus_device),.reg_addr(bus_address),.write_data(bus_value),
        .busy(),.done(bus_done),.error(cam_error),.read_data(cam_data),
        .scl_in(camera_scl),.sda_in(camera_sda),.scl_low(cam_scl_low),.sda_low(cam_sda_low));
 assign camera_scl=cam_scl_low?1'b0:1'bz;
 assign camera_sda=cam_sda_low?1'b0:1'bz;
 // No host receiver reset: preserve shared FIFO/DDR ownership while a byte
 // clock is stopped. A later valid START replaces any incomplete old frame.
 reg rx_enable;
 always @(posedge board50_global)if(!sys_rst_n)rx_enable<=0;else if(boot_stream)rx_enable<=1;
    wire byte_clk,hs_active;
    wire [15:0] raw_bytes;
    wire [5:0] lp_state;
    wire phy_timeout_event;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]phy_timeout_sync;
 reg phy_timeout_seen;reg[31:0]rx_timeouts;
 always @(posedge board50_global)if(!sys_rst_n||!rx_enable)begin phy_timeout_sync<=0;phy_timeout_seen<=0;rx_timeouts<=0;end
 else begin phy_timeout_sync<={phy_timeout_sync[0],phy_timeout_event};phy_timeout_seen<=phy_timeout_sync[1];
 if(phy_timeout_sync[1]!=phy_timeout_seen)rx_timeouts<=rx_timeouts+1'b1;end
    reg packet_toggle,good_toggle,ecc_toggle,crc_toggle,fs_toggle,fe_toggle;
    ph1p_mipi_rx #(.CONTROL_HZ(50000000))receiver(.clk_50(board50_global),.rst_n(sys_rst_n),.enable(rx_enable),
        .packet_done_toggle(packet_toggle),.clock_p(mipi_clock_p),.clock_n(mipi_clock_n),
        .data_p(mipi_data_p),.data_n(mipi_data_n),.byte_clk(byte_clk),.data(raw_bytes),
        .hs_active(hs_active),.lp_state(lp_state),.timeouts(),.timeout_event(phy_timeout_event));
    reg [3:0] byte_reset;
    wire byte_async_reset=!sys_rst_n || !rx_enable;
    always @(posedge byte_clk or posedge byte_async_reset) begin
        if(byte_async_reset) byte_reset<=0; else byte_reset<={byte_reset[2:0],1'b1};
    end
    wire byte_rst_n=byte_reset[3];
    reg [2:0] hs_sync;
    always @(posedge byte_clk or posedge byte_async_reset) begin
        if(byte_async_reset) hs_sync<=0; else hs_sync<={hs_sync[1:0],hs_active};
    end
    wire [15:0] paired_bytes,payload_bytes,packet_wc;
    wire pair_valid,pair_start,align_overflow,p_header,p_done,p_good,p_ecc,p_crc,p_fs,p_fe;
    wire [1:0] aligned,payload_keep,packet_vc;
    wire [5:0] packet_type;
    mipi_pair_align aligner(.clk(byte_clk),.rst_n(byte_rst_n),.active(hs_sync[1]),
        .lane0(raw_bytes[7:0]),.lane1(raw_bytes[15:8]),.bytes(paired_bytes),
        .pair_valid(pair_valid),.packet_start(pair_start),.overflow(align_overflow),.aligned(aligned));
    csi2_packet_rx packets(.clk(byte_clk),.rst_n(byte_rst_n),.start(pair_start),
        .abort(align_overflow || (hs_sync[2]&&!hs_sync[1])),.valid(pair_valid),.keep(2'b11),.bytes(paired_bytes),
        .payload(payload_bytes),.payload_keep(payload_keep),.header_valid(p_header),.data_type(packet_type),
        .virtual_channel(packet_vc),.word_count(packet_wc),.frame_start(p_fs),.frame_end(p_fe),
        .packet_done(p_done),.packet_good(p_good),.ecc_error(p_ecc),.crc_error(p_crc));
    // 包事件间隔远大于控制时钟周期，使用翻转事件跨域；事件计数在50MHz域完成。
    always @(posedge byte_clk or posedge byte_async_reset) begin
        if(byte_async_reset) begin packet_toggle<=0; good_toggle<=0; ecc_toggle<=0; crc_toggle<=0; fs_toggle<=0; fe_toggle<=0; end
        else begin
            if(p_done || align_overflow) packet_toggle<=!packet_toggle;
            if(p_done && p_good) good_toggle<=!good_toggle;
            if(p_ecc) ecc_toggle<=!ecc_toggle;
            if(p_crc) crc_toggle<=!crc_toggle;
            if(p_fs) fs_toggle<=!fs_toggle;
            if(p_fe) fe_toggle<=!fe_toggle;
        end
    end
    reg [5:0] event_sync0,event_sync1,event_previous;
    reg [31:0] good_packets,ecc_errors,crc_errors,total_packets,frame_starts,frame_ends;
    always @(posedge board50_global) begin
        if(!sys_rst_n || !rx_enable) begin
            event_sync0<=0; event_sync1<=0; event_previous<=0;
            good_packets<=0; ecc_errors<=0; crc_errors<=0; total_packets<=0;
            frame_starts<=0; frame_ends<=0;
        end else begin
            event_sync0<={fe_toggle,fs_toggle,crc_toggle,ecc_toggle,good_toggle,packet_toggle};
            event_sync1<=event_sync0; event_previous<=event_sync1;
            if(event_sync1[0]!=event_previous[0]) total_packets<=total_packets+1'b1;
            if(event_sync1[1]!=event_previous[1]) good_packets<=good_packets+1'b1;
            if(event_sync1[2]!=event_previous[2]) ecc_errors<=ecc_errors+1'b1;
            if(event_sync1[3]!=event_previous[3]) crc_errors<=crc_errors+1'b1;
            if(event_sync1[4]!=event_previous[4]) frame_starts<=frame_starts+1'b1;
            if(event_sync1[5]!=event_previous[5]) frame_ends<=frame_ends+1'b1;
        end
    end
    wire [31:0] camera_frequency,byte_frequency;
    clock_frequency_monitor camera_meter(.reference_clk(board50_global),.source_clk(camera_clock_internal),.rst_n(sys_rst_n),.edges(camera_frequency),.fresh());
    clock_frequency_monitor byte_meter(.reference_clk(board50_global),.source_clk(byte_clk),.rst_n(sys_rst_n),.edges(byte_frequency),.fresh());
    wire [39:0] raw_pixels;
    wire raw_valid,raw_partial,frame_done;
    wire [19:0] frame_word;
    wire raw_packet=(packet_type==6'h2b && packet_vc==0);
    raw10_unpack unpacker(.clk(byte_clk),.rst_n(byte_rst_n),.line_start(p_header && raw_packet),
        .line_end(p_done && raw_packet),.bytes(payload_bytes),.keep(raw_packet?payload_keep:2'b0),
        .pixels(raw_pixels),.valid(raw_valid),.partial_error(raw_partial));
 // Linear-domain neutral statistics reuse the original bin2/gain pipeline.
 wire[10:0]awb_gain_r,awb_gain_b;
 wire[7:0]awb_linear_r,awb_linear_g,awb_linear_b;wire awb_linear_valid;
 reg awb_enable;
 always @(posedge board50_global)if(!sys_rst_n)awb_enable<=1;
 else if(probe_write&&reg_addr==16'h0300&&write_data[7:1]==0)awb_enable<=write_data[0];
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]awb_enable_sync,awb_hold_sync;
 always @(posedge byte_clk)if(!byte_rst_n)begin awb_enable_sync<=0;awb_hold_sync<=2'b11;end
 else begin awb_enable_sync<={awb_enable_sync[0],awb_enable};
 awb_hold_sync<={awb_hold_sync[0],ae_hold_motion||!ae_stable||!ae_enable};end
 wire awb_result_valid,awb_confidence,awb_at_limit;
 wire[15:0]awb_sequence,awb_updates;wire[12:0]awb_count;
 wire[19:0]awb_sum_r,awb_sum_g,awb_sum_b;wire[7:0]awb_min,awb_max;
 neutral_awb awb(.clk(byte_clk),.rst_n(byte_rst_n),.frame_start(p_fs&&packet_vc==0),
 .input_fault((p_done&&raw_packet&&!p_good)||(p_header&&raw_packet&&packet_wc!=3200)||raw_partial||fifo_overflow),
 .enable(awb_enable_sync[1]),.allow_update(!awb_hold_sync[1]),.sample_valid(awb_linear_valid),
 .linear_r(awb_linear_r),.linear_g(awb_linear_g),.linear_b(awb_linear_b),
 .gain_r(awb_gain_r),.gain_b(awb_gain_b),.result_valid(awb_result_valid),.confidence(awb_confidence),.at_limit(awb_at_limit),
 .result_sequence(awb_sequence),.updates(awb_updates),.neutral_count(awb_count),
 .sum_r(awb_sum_r),.sum_g(awb_sum_g),.sum_b(awb_sum_b),.neutral_min(awb_min),.neutral_max(awb_max));
 wire[191:0]awb_byte_status={12'd0,awb_sum_b,12'd0,awb_sum_g,12'd0,awb_sum_r,
 3'd0,awb_max,awb_min,awb_count,awb_updates,awb_sequence,7'd0,awb_gain_b,awb_gain_r,awb_at_limit,awb_confidence,awb_result_valid};
 wire[191:0]awb_control_status;wire[145:0]awb_compact_status;wire awb_capture_busy;
 cdc_status_snapshot #(.WIDTH(146))awb_capture(.source_clk(byte_clk),.read_clk(board50_global),.rst_n(sys_rst_n&&rx_enable),
 .take(!awb_capture_busy),.source_data({awb_sum_b,awb_sum_g,awb_sum_r,awb_max,awb_min,awb_count,awb_updates,awb_sequence,awb_gain_b,awb_gain_r,awb_at_limit,awb_confidence,awb_result_valid}),.busy(awb_capture_busy),.read_data(awb_compact_status),.completed_sequence());
 assign awb_control_status={12'd0,awb_compact_status[145:126],12'd0,awb_compact_status[125:106],12'd0,awb_compact_status[105:86],
 3'd0,awb_compact_status[85:78],awb_compact_status[77:70],awb_compact_status[69:57],awb_compact_status[56:41],awb_compact_status[40:25],7'd0,awb_compact_status[24:0]};
 reg[191:0]awb_host_snapshot;reg[63:0]hist_host_snapshot;
 always @(posedge board50_global)if(!sys_rst_n)begin awb_host_snapshot<=0;hist_host_snapshot<=0;end
 else if(probe_write&&reg_addr==16'h0301)begin awb_host_snapshot<=awb_control_status;hist_host_snapshot<=hist_control_status;end
 wire [31:0] rgb_pair;wire rgb_pair_valid;
 // Temporary WB imported from A's current measured profile. B calibration is
 // outstanding; these values are not a claim that both sensors match.
 bayer_bin2 #(.WIDTH(2560),.BLACK(64),.GAIN_R(335),.GAIN_G(256),.GAIN_B(346),.RUNTIME_CONFIG(1)) isp(
 .clk(byte_clk),.rst_n(byte_rst_n),.frame_start(p_fs&&packet_vc==0),
 .line_start(p_header&&raw_packet),.line_end(p_done&&raw_packet),.pixels(raw_pixels),.pixels_valid(raw_valid),
 .rgb(rgb_pair),.valid(rgb_pair_valid),.runtime_black(10'd64),
 .runtime_gain_r(awb_gain_r),.runtime_gain_g(11'd256),.runtime_gain_b(awb_gain_b),
 .meter_linear_r(awb_linear_r),.meter_linear_g(awb_linear_g),.meter_linear_b(awb_linear_b),.meter_linear_valid(awb_linear_valid));
 wire [31:0] camera_accepted,camera_dropped,camera_drop_info,camera_drop_words,camera_full_events;
    camera_frame_pack #(.RAW_WIDTH(2560),.RAW_HEIGHT(1920)) packer(.clk(byte_clk),.rst_n(byte_rst_n),.frame_start(p_fs && packet_vc==0),.frame_end(p_fe && packet_vc==0),
        .line_start(p_header && raw_packet),.line_end(p_done && raw_packet),.line_good(p_good && !raw_partial),.line_bytes(packet_wc),
        .pair(rgb_pair),.pair_valid(rgb_pair_valid),.fifo_full(fifo_full),.fifo_data(packed_data),.fifo_write(packed_write),.accepted(camera_accepted),.dropped(camera_dropped),.last_drop_info(camera_drop_info),.last_drop_words(camera_drop_words),.fifo_full_events(camera_full_events));
    video_async_fifo video_fifo(.write_clk(byte_clk),.read_clk(memory_clk),.rst_n(sys_rst_n && memory_rst_n),
        .write_data(canonical_packed_data),.write_en(packed_write),.full(fifo_full),.overflow(fifo_overflow),
        .read_en(fifo_read),.read_data(fifo_data),.read_valid(fifo_valid),.empty(fifo_empty));
 // All camera packet counters below already belong to board50_global.
 // Camera accepted/dropped cross only through the frozen source mailbox.
 wire [127:0] pixel_snapshot;wire [63:0] front_snapshot;wire[159:0]camera_snapshot;
 wire pixel_busy,front_busy,camera_busy;wire [15:0] pixel_sequence,front_sequence,camera_sequence;
 wire camera_capture_initialized;
 cdc_status_snapshot #(.WIDTH(160),.DIRECT_HELD_READ(1))camera_capture(.read_initialized(camera_capture_initialized),.source_clk(byte_clk),.read_clk(board50_global),.rst_n(sys_rst_n),.take(take),
 .source_data({camera_full_events,camera_drop_words,camera_drop_info,camera_dropped,camera_accepted}),.busy(camera_busy),.read_data(camera_snapshot),.completed_sequence(camera_sequence));
 wire[96:0]pixel_compact_snapshot;
 assign pixel_snapshot={pixel_compact_snapshot[96:65],31'd0,pixel_compact_snapshot[64:0]};
 wire pixel_capture_initialized;
 cdc_status_snapshot #(.WIDTH(97),.DIRECT_HELD_READ(1))pixel_capture(.read_initialized(pixel_capture_initialized),.source_clk(pixel),.read_clk(board50_global),.rst_n(pixel_raw),.take(take),
 .source_data({displayed_frames,source_valid,source_frame_id,underflows}),.busy(pixel_busy),.read_data(pixel_compact_snapshot),.completed_sequence(pixel_sequence));
 wire front_capture_initialized;
 cdc_status_snapshot #(.WIDTH(64),.DIRECT_HELD_READ(1))front_capture(.read_initialized(front_capture_initialized),.source_clk(memory_clk),.read_clk(board50_global),.rst_n(sys_rst_n&&memory_rst_n),.take(take),
 .source_data({line_session,line_frame_id}),.busy(front_busy),.read_data(front_snapshot),.completed_sequence(front_sequence));
 wire [31:0] pixel_frequency,rx_frequency;
 clock_frequency_monitor pixel_meter(.reference_clk(board50_global),.source_clk(pixel),.rst_n(sys_rst_n),.edges(pixel_frequency),.fresh());
 clock_frequency_monitor rx_meter(.reference_clk(board50_global),.source_clk(rx_clk),.rst_n(sys_rst_n),.edges(rx_frequency),.fresh());
 reg [1:0] video_lock_sync;
 always @(posedge board50_global)if(!sys_rst_n)begin video_lock_sync<=0;memory_ready_sync<=0;end
 else begin video_lock_sync<={video_lock_sync[0],pll_v};memory_ready_sync<={memory_ready_sync[0],memory_rst_n};end
 wire [15:0] status={8'b0,front_busy,pixel_busy,camera_busy,mdio_error,memory_ready_sync[1],video_lock_sync[1],mdio_done,phy_ready};
 assign led={tracking_information[9],phy_ready,boot_failed,boot_done&&!boot_failed};
 reg [255:0] camera_control_snapshot;
 always @(posedge board50_global)begin
  if(!sys_rst_n)camera_control_snapshot<=0;
  else if(take)camera_control_snapshot<={16'd0,boot_chip,frame_ends,frame_starts,crc_errors,ecc_errors,good_packets,total_packets,rx_timeouts};
 end
 wire [255:0] peer_rx_snapshot;wire [159:0] link_pixel_snapshot;wire [159:0] link_tx_snapshot;
 wire peer_rx_busy,link_pixel_busy,link_tx_busy;
 wire [15:0] peer_rx_sequence,link_pixel_sequence,link_tx_sequence;
 // NB55: legacy host-only peer_capture diagnostic page unavailable.
 assign peer_rx_snapshot=0;assign peer_rx_busy=0;assign peer_rx_sequence=0;
 // NB55: legacy host-only link_pixel_capture diagnostic page unavailable.
 assign link_pixel_snapshot=0;assign link_pixel_busy=0;assign link_pixel_sequence=0;
 // NB55: legacy host-only link_tx_capture diagnostic page unavailable.
 assign link_tx_snapshot=0;assign link_tx_busy=0;assign link_tx_sequence=0;
 wire[159:0]depth_status_snapshot;wire[129:0]depth_compact_status;
 assign depth_status_snapshot={depth_compact_status[129:2],30'd0,depth_compact_status[1:0]};wire depth_status_busy;wire[15:0]depth_status_sequence;
 cdc_status_snapshot #(.WIDTH(130))depth_capture(.source_clk(memory_clk),.read_clk(board50_global),.rst_n(sys_rst_n&&memory_rst_n),.take(take),
  .source_data({depth_jobs_started,depth_jobs_completed,depth_jobs_skipped,depth_published,depth_system_fault,depth_config_error}),
  .busy(depth_status_busy),.read_data(depth_compact_status),.completed_sequence(depth_status_sequence));

 // TG26 is a separate 88B6 Ethernet channel; NV32 camera/depth data is unchanged.
 wire follow_rx_valid,follow_rx_busy,follow_host_valid;wire[255:0]follow_rx_payload,follow_held_payload;
 wire[31:0]follow_packets,follow_crc_errors,follow_rejected;
 follow_control_mac #(.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC),.LOCAL_ROLE(2),.PEER_ROLE(1),.VIEW_TX_ONLY(1),.DIRECT_RX_PAYLOAD(1))follow_mac(
  .tx_clk(tx_clk),.tx_rst_n(queue_tx_reset_n),.tx_request(bt_control_request),.tx_payload(bt_payload),.tx_busy(bt_control_busy),.tx_done(),.gmii_txd(bt_control_data),.gmii_tx_en(bt_control_enable),
  .rx_clk(rx_clk),.rx_rst_n(peer_pipeline_reset[1]),.gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),
  .rx_valid(follow_rx_valid),.rx_payload(follow_rx_payload),.rx_good(follow_packets),.rx_bad_crc(follow_crc_errors),.rx_rejected(follow_rejected));
 follow_payload_mailbox follow_target_capture(.source_clk(rx_clk),.destination_clk(board50_global),.rst_n(peer_pipeline_raw),
  .source_write(follow_rx_valid&&!follow_rx_busy),.source_data(follow_rx_payload),.source_busy(follow_rx_busy),
  .valid(follow_host_valid),.data(follow_held_payload),.consume(follow_host_valid));
 // B93 stereo-only: target/servo processing is absent. Use NB91 with AM94.
 // STEP/DIR/EN all match the original d36a reset-asleep output state.
 // EN is SLEEP#: low disables holding. No key, CSR or packet can arm a motor.
 assign step1=1'b0;assign dir1=1'b0;assign en1=1'b0;
 assign step2=1'b0;assign dir2=1'b0;assign en2=1'b0;
 assign tracking_requested=1'b0;
 assign tracking_information=128'd0;assign tracking_statistics=256'd0;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]tracking_host_reset,tracking_quiet_sync;
 reg tracking_quiet_memory;
 always @(posedge memory_clk)if(!memory_rst_n)tracking_quiet_memory<=0;else tracking_quiet_memory<=!depth_run_at_memory&&metric_memory_quiescent;
 wire tracking_raw_reset=sys_rst_n&&memory_rst_n&&phy_ready;
 always @(posedge board50_global or negedge tracking_raw_reset)
  if(!tracking_raw_reset)tracking_host_reset<=0;else tracking_host_reset<={tracking_host_reset[0],1'b1};
 always @(posedge board50_global or negedge tracking_host_reset[1])
  if(!tracking_host_reset[1])tracking_quiet_sync<=0;else tracking_quiet_sync<={tracking_quiet_sync[0],tracking_quiet_memory};
 wire tracking_link_ok=tracking_host_reset[1];
 always @(posedge board50_global)begin
  if(!sys_rst_n)bt_applied_token<=0;
  else if(follow_host_valid&&follow_held_payload[255:224]==32'h5641434b&&follow_held_payload[175:174]==3)bt_applied_token<=follow_held_payload[191:176];
 end
 assign gimbal_status=512'd0;
 assign gimbal_session_status=32'd0;assign gimbal_tuning_status=32'd0;
 assign motor_diagnostic={56'd0,en2,dir2,step2,en1,dir1,step1,tracking_requested,tracking_information[9]};
 wire gimbal_colour_busy=1'b0;
 wire[1:0]gimbal_colour_select=2'd0;
 wire[1:0]gimbal_colour_ack=gimbal_colour_select;
 wire[511:0]gimbal_host_status=512'd0;
 wire[199:0]gimbal_host_target=200'd0;
 wire[287:0]gimbal_host_extended=288'd0;
 wire[15:0]gimbal_key_presses=16'd0;
 wire[15:0]gimbal_software_presses=16'd0;
 always @(posedge board50_global)if(!sys_rst_n)gimbal_stop_sync<=0;else gimbal_stop_sync<={gimbal_stop_sync[0],key_n[2]};

 // Optional host-only metric export. No motor or video-mode command here.
 // Common reset is the same qualified sys/memory reset used by peer FIFO.
 wire metric_reset_n=sys_rst_n&&memory_rst_n;
 reg[7:0]metric_requested_row;
 always @(posedge board50_global)begin
  if(!metric_reset_n)metric_requested_row<=0;
  else if(probe_write&&reg_addr==16'h02e2)metric_requested_row<=write_data;
 end
 wire metric_held,metric_rearm_ready,metric_timeout,metric_hold_fault,metric_rearm_rejected;
 wire[31:0]metric_held_job,metric_row_job;wire[127:0]metric_held_sources;
 wire metric_row_busy,metric_row_valid,metric_row_overrun;wire[3:0]metric_row_error;
 wire[7:0]metric_returned_row,metric_read_byte;
 depth_export_pause metric_pause(.host_clk(board50_global),.memory_clk(memory_clk),.rst_n(metric_reset_n),
  .host_write(probe_write&&reg_addr==16'h02e0&&write_data[7:1]==0),.host_value(write_data[0]),
  .host_requested(metric_hold_requested),.host_held(metric_held),.host_rearm_ready(metric_rearm_ready),
  .host_timed_out(metric_timeout),.host_fault(metric_hold_fault),.host_rearm_rejected(metric_rearm_rejected),
  .host_job(metric_held_job),.host_source_pair(metric_held_sources),
  .memory_quiescent(metric_memory_quiescent),.last_completed_job(depth_jobs_completed),.last_source_pair(metric_job_sources),
  .memory_block_start(metric_memory_pause),.memory_hold_confirmed(metric_memory_held));

 // Legacy320x180 passive export is retired. It must fail explicitly rather
 // than exposing a cropped part of a native720 model under the old schema.
 assign metric_row_busy=0;assign metric_row_valid=0;assign metric_row_overrun=0;assign metric_row_error=4'hf;
 assign metric_row_job=0;assign metric_returned_row=0;assign metric_read_byte=0;
 reg[9:0]hd_export_requested_row;
 wire hd_export_busy,hd_export_valid,hd_export_overrun;wire[3:0]hd_export_error;
 wire[31:0]hd_export_job;wire[9:0]hd_export_returned_row;wire[7:0]hd_export_byte;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]hd_fault_host;
 always @(posedge board50_global)begin
  if(!sys_rst_n)begin hd_export_requested_row<=0;hd_fault_host<=0;end
  else begin
   hd_fault_host<={hd_fault_host[0],hd_sender_fault};
   if(probe_write&&reg_addr==16'h0d00)hd_export_requested_row[7:0]<=write_data;
   if(probe_write&&reg_addr==16'h0d01&&write_data[7:2]==0)hd_export_requested_row[9:8]<=write_data[1:0];
  end
 end
 assign capture_ready_internal=capture_gate_ready&&!metric_hold_requested&&!hd_fault_host[1];
 hd_depth_export hd_export(.host_clk(board50_global),.memory_clk(memory_clk),.rst_n(metric_reset_n),
  .host_request(probe_write&&reg_addr==16'h0d02&&write_data==1),.host_row(hd_export_requested_row),.host_expected_job(metric_held_job),
  .host_read_address(reg_addr[11:0]),.host_read_data(hd_export_byte),.host_busy(hd_export_busy),.host_valid(hd_export_valid),.host_error(hd_export_error),.host_overrun(hd_export_overrun),.host_job(hd_export_job),.host_returned_row(hd_export_returned_row),
  .hold_confirmed(metric_memory_held),.line_request(display_depth_request),.line_ready(display_depth_ready),.line_new(display_depth_new),.line_row(display_depth_row),
  .line_write(display_depth_write),.line_data(depth_line_data),.line_address(depth_line_address),.line_done(display_depth_done),.line_good(depth_line_good),.line_job(depth_line_frame),.line_session(depth_line_session));



 wire hd_meter_active,hd_meter_success,hd_meter_cancel,hd_meter_overrun,hd_meter_5s;
 wire[31:0]hd_meter_job,hd_meter_ticks;
 hd_capture_meter #(.DISTINGUISH_DRAIN(1))hd_elapsed(.board_clk(board50_global),.memory_clk(memory_clk),.rst_n(sys_rst_n&&memory_rst_n),
  .capture(capture_ready_internal&&(bt_capture_accept||depth_key_accept||(depth_host_accept&&write_data[0]))),
  .cancel(bt_exit_request||control_key_accept||(depth_host_accept&&!write_data[0])||tracking_requested),
  .model_displayed(hd_sender_done),.displayed_successful(!hd_completed_canceled),.displayed_job(hd_completed_job),.active(hd_meter_active),.last_job(hd_meter_job),.last_ticks(hd_meter_ticks),
  .successful(hd_meter_success),.canceled(hd_meter_cancel),.notice_overrun(hd_meter_overrun),.within_5s(hd_meter_5s),.confirmed_job_memory(hd_confirmed_job));
 reg [7:0] probe_data;

 wire[5:0]probe_byte_opt_gimbal_host_status_0410=reg_addr[5:0]-6'd16;
 wire[4:0]probe_byte_opt_gimbal_host_target_0450=reg_addr[4:0]-5'd16;
 wire[5:0]probe_byte_opt_gimbal_host_extended_0480=reg_addr[5:0]-6'd0;
 wire[1:0]probe_byte_opt_gimbal_session_status_0474=reg_addr[1:0]-2'd0;
 wire[0:0]probe_byte_opt_gimbal_software_presses_0478=reg_addr[0:0]-1'd0;
 wire[1:0]probe_byte_opt_gimbal_tuning_status_047b=reg_addr[1:0]-2'd3;
 wire[3:0]probe_byte_opt_cloud_snapshot_03a8=reg_addr[3:0]-4'd8;
 wire[1:0]probe_byte_opt_metric_held_job_02e4=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_metric_row_job_02e8=reg_addr[1:0]-2'd0;
 wire[3:0]probe_byte_opt_metric_held_sources_02f0=reg_addr[3:0]-4'd0;
 wire[4:0]probe_byte_opt_depth_status_snapshot_02c4=reg_addr[4:0]-5'd4;
 wire[2:0]probe_byte_opt_motor_diagnostic_0010=reg_addr[2:0]-3'd0;
 wire[1:0]probe_byte_opt_pixel_frequency_0020=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_rx_frequency_0024=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_camera_frequency_0028=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_byte_frequency_002c=reg_addr[1:0]-2'd0;
 wire[6:0]probe_byte_opt_memory_debug_0040=reg_addr[6:0]-7'd64;
 wire[4:0]probe_byte_opt_awb_host_snapshot_0310=reg_addr[4:0]-5'd16;
 wire[2:0]probe_byte_opt_hist_host_snapshot_0330=reg_addr[2:0]-3'd0;
 wire[1:0]probe_byte_opt_ae_target_accepted_00e4=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_ae_target_rejected_00e8=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_display_accepted_00ec=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_display_rejected_00f0=reg_addr[1:0]-2'd0;
 wire[0:0]probe_byte_opt_camera_sequence_0102=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_opt_pixel_sequence_0104=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_opt_front_sequence_0106=reg_addr[0:0]-1'd0;
 wire[2:0]probe_byte_opt_camera_snapshot_0110=reg_addr[2:0]-3'd0;
 wire[4:0]probe_byte_opt_camera_control_snapshot_0120=reg_addr[4:0]-5'd0;
 wire[3:0]probe_byte_opt_pixel_snapshot_0140=reg_addr[3:0]-4'd0;
 wire[4:0]probe_byte_opt_ae_snapshot_0150=reg_addr[4:0]-5'd16;
 wire[2:0]probe_byte_opt_front_snapshot_0170=reg_addr[2:0]-3'd0;
 wire[4:0]probe_byte_opt_peer_rx_snapshot_01a0=reg_addr[4:0]-5'd0;
 wire[4:0]probe_byte_opt_link_pixel_snapshot_01c0=reg_addr[4:0]-5'd0;
 wire[4:0]probe_byte_opt_link_tx_snapshot_01e0=reg_addr[4:0]-5'd0;
 wire[4:0]probe_byte_opt_peer_memory_debug_0200=reg_addr[4:0]-5'd0;
 wire[1:0]probe_byte_opt_key_switch_count_0240=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_noise_key_count_0284=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_control_host_accepted_0288=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_opt_control_host_rejected_028c=reg_addr[1:0]-2'd0;
 wire[0:0]probe_byte_opt_peer_rx_sequence_0248=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_opt_link_pixel_sequence_024a=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_opt_link_tx_sequence_024c=reg_addr[0:0]-1'd0;
 wire[1:0]probe_byte_opt_phy_id_0180=reg_addr[1:0]-2'd0;
 wire[0:0]probe_byte_opt_phy_status_0184=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_opt_phy_delay_0186=reg_addr[0:0]-1'd0;
 // Mutually exclusive addresses, separately sized 8-bit RHS before parallel selection.
 wire[7:0]csr_value_0=0;
 wire[7:0]csr_value_1=1;
 wire[7:0]csr_value_2={6'd0,tracking_quiet_sync[1],tracking_requested};
 wire[7:0]csr_value_3=gimbal_status[7:0];
 wire[7:0]csr_value_4={3'd0,gimbal_colour_busy,gimbal_colour_ack,gimbal_colour_select};
 wire[7:0]csr_value_5=gimbal_host_status[{probe_byte_opt_gimbal_host_status_0410,3'b000}+:8];
 wire[7:0]csr_value_6=gimbal_host_target[{probe_byte_opt_gimbal_host_target_0450,3'b000}+:8];
 wire[7:0]csr_value_7=gimbal_status[7:0];
 wire[7:0]csr_value_8=8'hc1;
 wire[7:0]csr_value_9=8'd0;
 wire[7:0]csr_value_10=gimbal_host_extended[{probe_byte_opt_gimbal_host_extended_0480,3'b000}+:8];
 wire[7:0]csr_value_11=gimbal_session_status[{probe_byte_opt_gimbal_session_status_0474,3'b000}+:8];
 wire[7:0]csr_value_12=gimbal_key_presses[7:0];
 wire[7:0]csr_value_13=gimbal_key_presses[15:8];
 wire[7:0]csr_value_14=gimbal_software_presses[{probe_byte_opt_gimbal_software_presses_0478,3'b000}+:8];
 wire[7:0]csr_value_15={7'd0,gimbal_stop_sync[1]};
 wire[7:0]csr_value_16=gimbal_tuning_status[{probe_byte_opt_gimbal_tuning_status_047b,3'b000}+:8];
 wire[7:0]csr_value_17=28;
 wire[7:0]csr_value_18=BOARD_ID;
 wire[7:0]csr_value_19=status[7:0];
 wire[7:0]csr_value_20=REVISION;
 wire[7:0]csr_value_21={3'd0,control_busy,control_applied[5:4],orbit_requested,cloud_requested};
 wire[7:0]csr_value_22=cloud_snapshot[{probe_byte_opt_cloud_snapshot_03a8,3'b000}+:8];
 wire[7:0]csr_value_23={2'd0,metric_rearm_rejected,metric_hold_fault,metric_timeout,metric_rearm_ready,metric_held,metric_hold_requested};
 wire[7:0]csr_value_24={metric_row_error,1'b0,metric_row_overrun,metric_row_valid,metric_row_busy};
 wire[7:0]csr_value_25=metric_requested_row;
 wire[7:0]csr_value_26=metric_returned_row;
 wire[7:0]csr_value_27=metric_held_job[{probe_byte_opt_metric_held_job_02e4,3'b000}+:8];
 wire[7:0]csr_value_28=metric_row_job[{probe_byte_opt_metric_row_job_02e8,3'b000}+:8];
 wire[7:0]csr_value_29=32'hc633449a>>((reg_addr-16'h02ec)*8);
 wire[7:0]csr_value_30=metric_held_sources[{probe_byte_opt_metric_held_sources_02f0,3'b000}+:8];
 wire[7:0]csr_value_31=metric_read_byte;
 wire[7:0]csr_value_32={5'd0,control_busy,depth_ack,depth_requested};
 wire[7:0]csr_value_33=depth_status_snapshot[{probe_byte_opt_depth_status_snapshot_02c4,3'b000}+:8];
 wire[7:0]csr_value_34=32'h3439424e>>((reg_addr-4)*8);
 wire[7:0]csr_value_35=8'h17;
 wire[7:0]csr_value_36={4'd0,rx_enable,boot_failed,boot_done,boot_active};
 wire[7:0]csr_value_37=motor_diagnostic[{probe_byte_opt_motor_diagnostic_0010,3'b000}+:8];
 wire[7:0]csr_value_38=pixel_frequency[{probe_byte_opt_pixel_frequency_0020,3'b000}+:8];
 wire[7:0]csr_value_39=rx_frequency[{probe_byte_opt_rx_frequency_0024,3'b000}+:8];
 wire[7:0]csr_value_40=camera_frequency[{probe_byte_opt_camera_frequency_0028,3'b000}+:8];
 wire[7:0]csr_value_41=byte_frequency[{probe_byte_opt_byte_frequency_002c,3'b000}+:8];
 wire[7:0]csr_value_42=memory_debug[{probe_byte_opt_memory_debug_0040,3'b000}+:8];
 wire[7:0]csr_value_43=(camera_capture_initialized?(camera_snapshot>>((reg_addr-16'h0340)*8+64)):8'd0);
 wire[7:0]csr_value_44={7'd0,awb_enable};
 wire[7:0]csr_value_45={5'd0,hist_bypass_busy,hist_bypass_ack,hist_bypass_requested};
 wire[7:0]csr_value_46=awb_host_snapshot[{probe_byte_opt_awb_host_snapshot_0310,3'b000}+:8];
 wire[7:0]csr_value_47=hist_host_snapshot[{probe_byte_opt_hist_host_snapshot_0330,3'b000}+:8];
 wire[7:0]csr_value_48={7'd0,ae_enable};
 wire[7:0]csr_value_49={6'd0,ae_target_mode};
 wire[7:0]csr_value_50={3'd0,control_busy,display_ack,display_requested};
 wire[7:0]csr_value_51=ae_target_accepted[{probe_byte_opt_ae_target_accepted_00e4,3'b000}+:8];
 wire[7:0]csr_value_52=ae_target_rejected[{probe_byte_opt_ae_target_rejected_00e8,3'b000}+:8];
 wire[7:0]csr_value_53=display_accepted[{probe_byte_opt_display_accepted_00ec,3'b000}+:8];
 wire[7:0]csr_value_54=display_rejected[{probe_byte_opt_display_rejected_00f0,3'b000}+:8];
 wire[7:0]csr_value_55={2'd0,link_tx_busy,link_pixel_busy,peer_rx_busy,front_busy,pixel_busy,camera_busy};
 wire[7:0]csr_value_56=camera_sequence[{probe_byte_opt_camera_sequence_0102,3'b000}+:8];
 wire[7:0]csr_value_57=pixel_sequence[{probe_byte_opt_pixel_sequence_0104,3'b000}+:8];
 wire[7:0]csr_value_58=front_sequence[{probe_byte_opt_front_sequence_0106,3'b000}+:8];
 wire[7:0]csr_value_59=(camera_capture_initialized?(camera_snapshot[{probe_byte_opt_camera_snapshot_0110,3'b000}+:8]):8'd0);
 wire[7:0]csr_value_60=camera_control_snapshot[{probe_byte_opt_camera_control_snapshot_0120,3'b000}+:8];
 wire[7:0]csr_value_61=(pixel_capture_initialized?(pixel_snapshot[{probe_byte_opt_pixel_snapshot_0140,3'b000}+:8]):8'd0);
 wire[7:0]csr_value_62=ae_snapshot[{probe_byte_opt_ae_snapshot_0150,3'b000}+:8];
 wire[7:0]csr_value_63=(front_capture_initialized?(front_snapshot[{probe_byte_opt_front_snapshot_0170,3'b000}+:8]):8'd0);
 wire[7:0]csr_value_64=peer_rx_snapshot[{probe_byte_opt_peer_rx_snapshot_01a0,3'b000}+:8];
 wire[7:0]csr_value_65=link_pixel_snapshot[{probe_byte_opt_link_pixel_snapshot_01c0,3'b000}+:8];
 wire[7:0]csr_value_66=link_tx_snapshot[{probe_byte_opt_link_tx_snapshot_01e0,3'b000}+:8];
 wire[7:0]csr_value_67=peer_memory_debug[{probe_byte_opt_peer_memory_debug_0200,3'b000}+:8];
 wire[7:0]csr_value_68=key_switch_count[{probe_byte_opt_key_switch_count_0240,3'b000}+:8];
 wire[7:0]csr_value_69={5'd0,control_busy,control_ack,requested_remote};
 wire[7:0]csr_value_70=8'h03;
 wire[7:0]csr_value_71={5'd0,noise_busy,noise_ack,noise_requested};
 wire[7:0]csr_value_72=noise_key_count[{probe_byte_opt_noise_key_count_0284,3'b000}+:8];
 wire[7:0]csr_value_73=control_host_accepted[{probe_byte_opt_control_host_accepted_0288,3'b000}+:8];
 wire[7:0]csr_value_74=control_host_rejected[{probe_byte_opt_control_host_rejected_028c,3'b000}+:8];
 wire[7:0]csr_value_75=peer_rx_sequence[{probe_byte_opt_peer_rx_sequence_0248,3'b000}+:8];
 wire[7:0]csr_value_76=link_pixel_sequence[{probe_byte_opt_link_pixel_sequence_024a,3'b000}+:8];
 wire[7:0]csr_value_77=link_tx_sequence[{probe_byte_opt_link_tx_sequence_024c,3'b000}+:8];
 wire[7:0]csr_value_78=phy_id[{probe_byte_opt_phy_id_0180,3'b000}+:8];
 wire[7:0]csr_value_79=phy_status[{probe_byte_opt_phy_status_0184,3'b000}+:8];
 wire[7:0]csr_value_80=phy_delay[{probe_byte_opt_phy_delay_0186,3'b000}+:8];
 // Full CSR pages, one allowed host-read clock; PROBE_WAIT is unchanged.
 wire[15:0]csr_address_00={8'h00,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_00;
 always @(posedge board50_global)begin
  if(!sys_rst_n)csr_bank_00<=0;
  else csr_bank_00<=
({8{csr_address_00==0}} & (28)) |
({8{csr_address_00==1||csr_address_00==16'h0b04}} & (BOARD_ID)) |
({8{csr_address_00==2}} & (status[7:0])) |
({8{csr_address_00==3}} & (REVISION)) |
({8{csr_address_00>=4&&csr_address_00<8}} & (32'h3439424e>>((csr_address_00-4)*8))) |
({8{csr_address_00==8}} & (8'h17)) |
({8{csr_address_00==9}} & ({4'd0,rx_enable,boot_failed,boot_done,boot_active})) |
({8{csr_address_00>=16'h0010&&csr_address_00<16'h0018}} & (motor_diagnostic[{probe_byte_opt_motor_diagnostic_0010,3'b000}+:8])) |
({8{csr_address_00>=16'h0020&&csr_address_00<16'h0024}} & (pixel_frequency[{probe_byte_opt_pixel_frequency_0020,3'b000}+:8])) |
({8{csr_address_00>=16'h0024&&csr_address_00<16'h0028}} & (rx_frequency[{probe_byte_opt_rx_frequency_0024,3'b000}+:8])) |
({8{csr_address_00>=16'h0028&&csr_address_00<16'h002c}} & (camera_frequency[{probe_byte_opt_camera_frequency_0028,3'b000}+:8])) |
({8{csr_address_00>=16'h002c&&csr_address_00<16'h0030}} & (byte_frequency[{probe_byte_opt_byte_frequency_002c,3'b000}+:8])) |
({8{csr_address_00>=16'h0040&&csr_address_00<16'h0090}} & (memory_debug[{probe_byte_opt_memory_debug_0040,3'b000}+:8])) |
({8{csr_address_00==16'h00e0}} & ({7'd0,ae_enable})) |
({8{csr_address_00==16'h00e2}} & ({6'd0,ae_target_mode})) |
({8{csr_address_00==16'h00e3}} & ({3'd0,control_busy,display_ack,display_requested})) |
({8{csr_address_00>=16'h00e4&&csr_address_00<16'h00e8}} & (ae_target_accepted[{probe_byte_opt_ae_target_accepted_00e4,3'b000}+:8])) |
({8{csr_address_00>=16'h00e8&&csr_address_00<16'h00ec}} & (ae_target_rejected[{probe_byte_opt_ae_target_rejected_00e8,3'b000}+:8])) |
({8{csr_address_00>=16'h00ec&&csr_address_00<16'h00f0}} & (display_accepted[{probe_byte_opt_display_accepted_00ec,3'b000}+:8])) |
({8{csr_address_00>=16'h00f0&&csr_address_00<16'h00f4}} & (display_rejected[{probe_byte_opt_display_rejected_00f0,3'b000}+:8]));
 end
 wire[15:0]csr_address_01={8'h01,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_01;
 always @(posedge board50_global)begin
  if(!sys_rst_n)csr_bank_01<=0;
  else csr_bank_01<=
({8{csr_address_01==16'h0100}} & ({2'd0,link_tx_busy,link_pixel_busy,peer_rx_busy,front_busy,pixel_busy,camera_busy})) |
({8{csr_address_01>=16'h0102&&csr_address_01<16'h0104}} & (camera_sequence[{probe_byte_opt_camera_sequence_0102,3'b000}+:8])) |
({8{csr_address_01>=16'h0104&&csr_address_01<16'h0106}} & (pixel_sequence[{probe_byte_opt_pixel_sequence_0104,3'b000}+:8])) |
({8{csr_address_01>=16'h0106&&csr_address_01<16'h0108}} & (front_sequence[{probe_byte_opt_front_sequence_0106,3'b000}+:8])) |
({8{csr_address_01>=16'h0110&&csr_address_01<16'h0118}} & ((camera_capture_initialized?(camera_snapshot[{probe_byte_opt_camera_snapshot_0110,3'b000}+:8]):8'd0))) |
({8{csr_address_01>=16'h0120&&csr_address_01<16'h0140}} & (camera_control_snapshot[{probe_byte_opt_camera_control_snapshot_0120,3'b000}+:8])) |
({8{csr_address_01>=16'h0140&&csr_address_01<16'h0150}} & ((pixel_capture_initialized?(pixel_snapshot[{probe_byte_opt_pixel_snapshot_0140,3'b000}+:8]):8'd0))) |
({8{csr_address_01>=16'h0150&&csr_address_01<16'h0168}} & (ae_snapshot[{probe_byte_opt_ae_snapshot_0150,3'b000}+:8])) |
({8{csr_address_01>=16'h0170&&csr_address_01<16'h0178}} & ((front_capture_initialized?(front_snapshot[{probe_byte_opt_front_snapshot_0170,3'b000}+:8]):8'd0))) |
({8{csr_address_01>=16'h01a0&&csr_address_01<16'h01c0}} & (peer_rx_snapshot[{probe_byte_opt_peer_rx_snapshot_01a0,3'b000}+:8])) |
({8{csr_address_01>=16'h01c0&&csr_address_01<16'h01d4}} & (link_pixel_snapshot[{probe_byte_opt_link_pixel_snapshot_01c0,3'b000}+:8])) |
({8{csr_address_01>=16'h01e0&&csr_address_01<16'h01f4}} & (link_tx_snapshot[{probe_byte_opt_link_tx_snapshot_01e0,3'b000}+:8])) |
({8{csr_address_01>=16'h0180&&csr_address_01<16'h0184}} & (phy_id[{probe_byte_opt_phy_id_0180,3'b000}+:8])) |
({8{csr_address_01>=16'h0184&&csr_address_01<16'h0186}} & (phy_status[{probe_byte_opt_phy_status_0184,3'b000}+:8])) |
({8{csr_address_01>=16'h0186&&csr_address_01<16'h0188}} & (phy_delay[{probe_byte_opt_phy_delay_0186,3'b000}+:8]));
 end
 wire[15:0]csr_address_02={8'h02,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_02;
 always @(posedge board50_global)begin
  if(!sys_rst_n)csr_bank_02<=0;
  else csr_bank_02<=
({8{csr_address_02==16'h02e0}} & ({2'd0,metric_rearm_rejected,metric_hold_fault,metric_timeout,metric_rearm_ready,metric_held,metric_hold_requested})) |
({8{csr_address_02==16'h02e1}} & ({metric_row_error,1'b0,metric_row_overrun,metric_row_valid,metric_row_busy})) |
({8{csr_address_02==16'h02e2}} & (metric_requested_row)) |
({8{csr_address_02==16'h02e3}} & (metric_returned_row)) |
({8{csr_address_02>=16'h02e4&&csr_address_02<16'h02e8}} & (metric_held_job[{probe_byte_opt_metric_held_job_02e4,3'b000}+:8])) |
({8{csr_address_02>=16'h02e8&&csr_address_02<16'h02ec}} & (metric_row_job[{probe_byte_opt_metric_row_job_02e8,3'b000}+:8])) |
({8{csr_address_02>=16'h02ec&&csr_address_02<16'h02f0}} & (32'hc633449a>>((csr_address_02-16'h02ec)*8))) |
({8{csr_address_02>=16'h02f0&&csr_address_02<16'h0300}} & (metric_held_sources[{probe_byte_opt_metric_held_sources_02f0,3'b000}+:8])) |
({8{csr_address_02==16'h02c0}} & ({5'd0,control_busy,depth_ack,depth_requested})) |
({8{csr_address_02>=16'h02c4&&csr_address_02<16'h02d8}} & (depth_status_snapshot[{probe_byte_opt_depth_status_snapshot_02c4,3'b000}+:8])) |
({8{csr_address_02>=16'h0200&&csr_address_02<16'h0220}} & (peer_memory_debug[{probe_byte_opt_peer_memory_debug_0200,3'b000}+:8])) |
({8{csr_address_02>=16'h0240&&csr_address_02<16'h0244}} & (key_switch_count[{probe_byte_opt_key_switch_count_0240,3'b000}+:8])) |
({8{csr_address_02==16'h0244}} & ({5'd0,control_busy,control_ack,requested_remote})) |
({8{csr_address_02==16'h0292}} & (8'h03)) |
({8{csr_address_02==16'h0280}} & ({5'd0,noise_busy,noise_ack,noise_requested})) |
({8{csr_address_02>=16'h0284&&csr_address_02<16'h0288}} & (noise_key_count[{probe_byte_opt_noise_key_count_0284,3'b000}+:8])) |
({8{csr_address_02>=16'h0288&&csr_address_02<16'h028c}} & (control_host_accepted[{probe_byte_opt_control_host_accepted_0288,3'b000}+:8])) |
({8{csr_address_02>=16'h028c&&csr_address_02<16'h0290}} & (control_host_rejected[{probe_byte_opt_control_host_rejected_028c,3'b000}+:8])) |
({8{csr_address_02>=16'h0248&&csr_address_02<16'h024a}} & (peer_rx_sequence[{probe_byte_opt_peer_rx_sequence_0248,3'b000}+:8])) |
({8{csr_address_02>=16'h024a&&csr_address_02<16'h024c}} & (link_pixel_sequence[{probe_byte_opt_link_pixel_sequence_024a,3'b000}+:8])) |
({8{csr_address_02>=16'h024c&&csr_address_02<16'h024e}} & (link_tx_sequence[{probe_byte_opt_link_tx_sequence_024c,3'b000}+:8]));
 end
 wire[15:0]csr_address_03={8'h03,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_03;
 always @(posedge board50_global)begin
  if(!sys_rst_n)csr_bank_03<=0;
  else csr_bank_03<=
({8{csr_address_03==16'h03f0}} & (0)) |
({8{csr_address_03==16'h03a0}} & ({3'd0,control_busy,control_applied[5:4],orbit_requested,cloud_requested})) |
({8{csr_address_03>=16'h03a8&&csr_address_03<16'h03b8}} & (cloud_snapshot[{probe_byte_opt_cloud_snapshot_03a8,3'b000}+:8])) |
({8{csr_address_03>=16'h0340&&csr_address_03<16'h034c}} & ((camera_capture_initialized?(camera_snapshot>>((csr_address_03-16'h0340)*8+64)):8'd0))) |
({8{csr_address_03==16'h0300}} & ({7'd0,awb_enable})) |
({8{csr_address_03==16'h0302}} & ({5'd0,hist_bypass_busy,hist_bypass_ack,hist_bypass_requested})) |
({8{csr_address_03>=16'h0310&&csr_address_03<16'h0328}} & (awb_host_snapshot[{probe_byte_opt_awb_host_snapshot_0310,3'b000}+:8])) |
({8{csr_address_03>=16'h0330&&csr_address_03<16'h0338}} & (hist_host_snapshot[{probe_byte_opt_hist_host_snapshot_0330,3'b000}+:8]));
 end
 wire[15:0]csr_address_04={8'h04,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_04;
 always @(posedge board50_global)begin
  if(!sys_rst_n)csr_bank_04<=0;
  else csr_bank_04<=
({8{csr_address_04==16'h04c0}} & (1)) |
({8{csr_address_04==16'h04c1}} & ({6'd0,tracking_quiet_sync[1],tracking_requested})) |
({8{csr_address_04==16'h0400}} & (gimbal_status[7:0])) |
({8{csr_address_04==16'h0405}} & ({3'd0,gimbal_colour_busy,gimbal_colour_ack,gimbal_colour_select})) |
({8{csr_address_04>=16'h0410&&csr_address_04<16'h0450}} & (gimbal_host_status[{probe_byte_opt_gimbal_host_status_0410,3'b000}+:8])) |
({8{csr_address_04>=16'h0450&&csr_address_04<16'h0469}} & (gimbal_host_target[{probe_byte_opt_gimbal_host_target_0450,3'b000}+:8])) |
({8{csr_address_04==16'h0470}} & (gimbal_status[7:0])) |
({8{csr_address_04==16'h0473}} & (8'hc1)) |
({8{csr_address_04==16'h047f}} & (8'd0)) |
({8{csr_address_04>=16'h0480&&csr_address_04<16'h04a4}} & (gimbal_host_extended[{probe_byte_opt_gimbal_host_extended_0480,3'b000}+:8])) |
({8{csr_address_04>=16'h0474&&csr_address_04<16'h0478}} & (gimbal_session_status[{probe_byte_opt_gimbal_session_status_0474,3'b000}+:8])) |
({8{csr_address_04==16'h0471}} & (gimbal_key_presses[7:0])) |
({8{csr_address_04==16'h0472}} & (gimbal_key_presses[15:8])) |
({8{csr_address_04>=16'h0478&&csr_address_04<16'h047a}} & (gimbal_software_presses[{probe_byte_opt_gimbal_software_presses_0478,3'b000}+:8])) |
({8{csr_address_04==16'h047a}} & ({7'd0,gimbal_stop_sync[1]})) |
({8{csr_address_04>=16'h047b&&csr_address_04<16'h047f}} & (gimbal_tuning_status[{probe_byte_opt_gimbal_tuning_status_047b,3'b000}+:8]));
 end
 wire[15:0]csr_address_0b={8'h0b,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_0b;
 always @(posedge board50_global)begin
  if(!sys_rst_n)csr_bank_0b<=0;
  else csr_bank_0b<=
({8{csr_address_0b==1||csr_address_0b==16'h0b04}} & (BOARD_ID));
 end
 always @*begin
  if(reg_addr==16'h0e00)probe_data={4'd0,hd_meter_5s,hd_meter_cancel,hd_meter_success,hd_meter_active};
  else if(reg_addr>=16'h0e04&&reg_addr<16'h0e08)probe_data=hd_meter_job[(reg_addr-16'h0e04)*8+:8];
  else if(reg_addr>=16'h0e08&&reg_addr<16'h0e0c)probe_data=hd_meter_ticks[(reg_addr-16'h0e08)*8+:8];
  else if(reg_addr>=16'h1000&&reg_addr<16'h1a00)probe_data=hd_export_byte;
  else if(reg_addr==16'h0d00)probe_data=hd_export_requested_row[7:0];
  else if(reg_addr==16'h0d01)probe_data={6'd0,hd_export_requested_row[9:8]};
  else if(reg_addr==16'h0d02)probe_data={hd_export_error,1'b0,hd_export_overrun,hd_export_valid,hd_export_busy};
  else if(reg_addr>=16'h0d04&&reg_addr<16'h0d08)probe_data=hd_export_job[(reg_addr-16'h0d04)*8+:8];
  else if(reg_addr==16'h0d08)probe_data=hd_export_returned_row[7:0];
  else if(reg_addr==16'h0d09)probe_data={6'd0,hd_export_returned_row[9:8]};
  else if(reg_addr>=16'h0d0a&&reg_addr<16'h0d0e)probe_data=32'hc633449a>>((reg_addr-16'h0d0a)*8);
  else if(reg_addr==16'h0d0e)probe_data=8'h00;
  else if(reg_addr==16'h0d0f)probe_data=8'h05;
  else if(reg_addr==16'h0d10)probe_data=8'hd0;
  else if(reg_addr==16'h0d11)probe_data=8'h02;
  else if(reg_addr==16'h0d12)probe_data=8'h21;
  else if(reg_addr==16'h0c00)probe_data={5'd0,calibration_peer_busy,calibration_peer_ack,calibration_peer_requested};
  else if(reg_addr==16'h0c01)probe_data={7'd0,calibration_host_busy};
  else if(reg_addr>=16'h0c10&&reg_addr<16'h0c24)probe_data=calibration_host_status[(reg_addr-16'h0c10)*8+:8];
  else if(reg_addr==16'h0b20)probe_data={6'd0,bt_baud_select};
  else if(reg_addr==16'h0b21)probe_data={6'd0,capture_ready_internal,bt_framing_error};
  else if(reg_addr==16'h0b22)probe_data=bt_bad_requests[7:0];
  else if(reg_addr==16'h0b23)probe_data=bt_bad_requests[15:8];
  else if(reg_addr==16'h0b24)probe_data=bt_valid_requests[7:0];
  else if(reg_addr==16'h0b25)probe_data=bt_valid_requests[15:8];
  else if(snapshot_probe_selected)probe_data=snapshot_probe_data;

  else if(reg_addr>=16'hb000&&reg_addr<16'hb280)probe_data=metric_read_byte;
  else begin
  case(reg_addr[15:8])
   8'h00:probe_data=csr_bank_00;
   8'h01:probe_data=csr_bank_01;
   8'h02:probe_data=csr_bank_02;
   8'h03:probe_data=csr_bank_03;
   8'h04:probe_data=csr_bank_04;
   8'h0b:probe_data=csr_bank_0b;
   default:probe_data=8'd0;
  endcase
  end
 end

 bluetooth_view_control #(.REVISION(94))bluetooth(.clk(board50_global),.rst_n(sys_rst_n),.rx(hc05_rx),.tx(hc05_tx),.baud_div(bt_baud_div),
  .capture_ready(capture_ready_internal&&!tracking_requested&&!control_busy&&depth_enter_safe),
  .system_flags({1'b0,depth_status_snapshot[1],tracking_requested,control_busy,!capture_ready_internal,orbit_requested,cloud_requested,depth_requested}),
  .depth_job(depth_status_snapshot[127:96]),.applied_token(bt_applied_token),
  .capture_request(bt_capture_request),.exit_request(bt_exit_request),.status_latch(bt_status_latch),
  .view_record(bt_view_record),.view_commit(bt_view_commit),.valid_requests(bt_valid_requests),.bad_requests(bt_bad_requests),.framing_error(bt_framing_error));
 wire [7:0] uart_rx_data,uart_tx_data;wire uart_rx_valid,uart_tx_valid,uart_tx_ready,uart_error;
 uart_byte #(.DIV(50)) uart(.clk(board50_global),.rst_n(sys_rst_n),.rx(uart_rx),.tx(uart_tx),
 .tx_data(uart_tx_data),.tx_valid(uart_tx_valid),.tx_ready(uart_tx_ready),.rx_data(uart_rx_data),.rx_valid(uart_rx_valid),.framing_error(uart_error));
 debug_register_bridge bridge(.clk(board50_global),.rst_n(sys_rst_n),.busy(bridge_busy),
 .rx_data(uart_rx_data),.rx_valid(uart_rx_valid),.tx_data(uart_tx_data),.tx_valid(uart_tx_valid),.tx_ready(uart_tx_ready),
 .status(status),.probe_data(probe_data),.probe_write(probe_write),.cam_valid(cam_valid),.ddc_valid(ddc_valid),.read(cam_read),
 .dev_addr(dev_addr),.reg_addr(reg_addr),.write_data(write_data),.cam_done(cam_done),.cam_error(cam_error),.cam_data(cam_data),
 .ddc_done(ddc_valid),.ddc_error(2'd1),.ddc_data(8'b0));
endmodule
