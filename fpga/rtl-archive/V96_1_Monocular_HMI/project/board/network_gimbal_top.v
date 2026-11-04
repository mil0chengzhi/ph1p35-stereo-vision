`timescale 1ns/1ps
// AM96 monocular HMI profile / physical A / protocol28 revision96.
// Real AE/AWB/BOX tracker MCU; all A motor outputs remain zero.
module network_gimbal_top #(parameter BOARD_ID=1,REVISION=96)(
 input wire clk_50,input wire [3:0] key_n,input wire uart_rx,output wire uart_tx,
 input wire hmi_rx,output wire hmi_tx,
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
  key_sync<={key_sync[0],key_n[3]};
  if(!key_sync[1])startup<=0;else if(!startup[19])startup<=startup+1'b1;
 end
 wire sys_rst_n=startup[19];
 // Camera commissioning image: no motor module is instantiated. KEY1..3
 // KEY1 switches sources on B; KEY2 toggles local display NR; KEY4 remains reset. EN=0 releases hold (D36A SLEEP#).
 assign step1=1'b0;assign dir1=1'b0;assign en1=1'b0;
 assign step2=1'b0;assign dir2=1'b0;assign en2=1'b0;
 wire [63:0] motor_diagnostic=64'd0;
 assign hdmi_scl=1'bz;assign hdmi_sda=1'bz;assign hdmi_power_enable=sys_rst_n;
 wire tx_clk,tx_rst_n,rx_clk,rx_rst_n,phy_ready,mdio_done,mdio_error;
 wire [7:0] gmii_rxd,gmii_txd;wire gmii_rx_dv,gmii_rx_er,gmii_tx_en;
 wire[7:0]legacy_gmii_data,hd_gmii_data;wire legacy_gmii_enable,hd_gmii_enable,hd_tx_offer,hd_tx_accept,hd_tx_busy;
 wire legacy_gate,legacy_packet_ready;

 wire [31:0] phy_id;wire [15:0] phy_status,phy_delay;
 db_rgmii_phy phy(.clk_50(board50_global),.rst_n(sys_rst_n),
 .tx_clk(tx_clk),.tx_rst_n(tx_rst_n),.rx_clk(rx_clk),.rx_rst_n(rx_rst_n),
 .gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),.gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),
 .eth_rxc(eth_rxc),.eth_rx_ctl(eth_rx_ctl),.eth_rxd(eth_rxd),.eth_txc(eth_txc),.eth_tx_ctl(eth_tx_ctl),.eth_txd(eth_txd),
 .eth_reset_n(eth_reset_n),.phy_ready(phy_ready),.eth_mdc(eth_mdc),.eth_mdio(eth_mdio),
 .phy_id32(phy_id),.phy_status16(phy_status),.phy_delay16(phy_delay),.mdio_done(mdio_done),.mdio_error(mdio_error));

 wire memory_clk,memory_rst_n,memory_verified;wire [639:0] memory_debug;
 wire fifo_empty,fifo_read,fifo_valid,fifo_full,fifo_overflow;
 wire [129:0] fifo_data,packed_data;wire packed_write;
 wire line_request,line_ready,line_new,line_request_bank,line_write,line_bank,line_done,line_good;
 wire [15:0] line_row;wire [127:0] line_data;wire [7:0] line_address;
 wire [31:0] line_frame_id,line_session,line_start_ms;

 wire peer_fifo_empty,peer_fifo_read,peer_fifo_valid,peer_fifo_full,peer_fifo_overflow;
 wire [129:0] peer_fifo_data,peer_packed_data;wire peer_packed_write;
 wire peer_line_request,peer_line_ready,peer_line_new,peer_line_request_bank;
 wire peer_line_write,peer_line_bank,peer_line_done,peer_line_good;
 wire [15:0] peer_line_row;wire [127:0] peer_line_data;wire [7:0] peer_line_address;
 wire [31:0] peer_line_frame_id,peer_line_session,peer_line_start_ms;
 wire peer_latest_valid;wire[31:0]peer_latest_frame,peer_latest_session;
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
 // AM94: peer_mac is absent; its diagnostics advertise no 3D/peer publication.
 assign peer_packet_start=0;
 assign peer_word_write=0;
 assign peer_word_address=0;
 assign peer_word_data=0;
 assign peer_packet_done=0;
 assign peer_packet_good=0;
 assign peer_packet_session=0;
 assign peer_packet_frame=0;
 assign peer_packet_index=0;
 assign peer_rx_packets=0;
 assign peer_bad_crc=0;
 assign peer_rejected=0;
 assign peer_no_slot=0;
 wire peer_session_ok=(peer_packet_session[31:24]==8'hB2||peer_packet_session[31:24]==8'hD2||peer_packet_session[31:24]==8'hD3||peer_packet_session[31:24]==8'hD4||peer_packet_session[31:24]==8'hD5)&&peer_packet_session[21:0]!=0;
 // AM94: peer_assembly is absent; its diagnostics advertise no 3D/peer publication.
 assign peer_packet_accept=0;
 assign peer_packed_write=0;
 assign peer_packed_data=0;
 assign peer_completed=0;
 assign peer_dropped=0;
 assign peer_queue_loss=0;
 assign peer_last_session=0;
 assign peer_last_frame=0;
 assign peer_assembly_flags=0;
 // PHY restart never clears a FIFO that the DDR reader may already own.
 // AM94: peer_fifo is absent; its diagnostics advertise no 3D/peer publication.
 assign peer_fifo_full=0;
 assign peer_fifo_overflow=0;
 assign peer_fifo_data=0;
 assign peer_fifo_valid=0;
 assign peer_fifo_empty=1'b1;
 // Independent local/remote frame slots share one physical DDR controller.
 wire [15:0] reg_addr;wire probe_write;wire [7:0] write_data;
 wire hmi_control_write;wire[15:0]hmi_control_address;wire[7:0]hmi_control_data;wire[255:0]hmi_diagnostics;
 // The HMI waits for USB writes to finish before presenting a control strobe.
 wire image_control_write=probe_write||hmi_control_write;
 wire[15:0]image_control_address=probe_write?reg_addr:hmi_control_address;
 wire[7:0]image_control_data=probe_write?write_data:hmi_control_data;
 wire [31:0] contest_time_gray;
 wire take=probe_write&&reg_addr==16'h0100&&write_data==1;
 wire[7:0]memory_snapshot_byte;wire memory_snapshot_led;
 wire memory_snapshot_read_peer=reg_addr[15:8]==8'h02;
 wire[4:0]memory_snapshot_read_index=memory_snapshot_read_peer?reg_addr[4:0]:(reg_addr[4:0]-5'd16);
 reg memory_snapshot_local_selected,memory_snapshot_peer_selected;
 always @(posedge board50_global)begin
  if(!sys_rst_n)begin memory_snapshot_local_selected<=0;memory_snapshot_peer_selected<=0;end
  else begin
   memory_snapshot_local_selected<=reg_addr>=16'h0070&&reg_addr<16'h0090;
   memory_snapshot_peer_selected<=reg_addr>=16'h0200&&reg_addr<16'h0220;
  end
 end

 wire camera_capture_initialized,pixel_capture_initialized,front_capture_initialized,peer_capture_initialized,contest_capture_initialized,cloud_capture_initialized;
 wire[24:0]cloud_awaddr,cloud_araddr;wire[127:0]cloud_wdata,cloud_rdata;wire[15:0]cloud_wstrb;
 wire cloud_awvalid,cloud_wvalid,cloud_wlast,cloud_arvalid,cloud_rready,cloud_read_active;
 wire cloud_awready,cloud_wready,cloud_arready,cloud_rvalid,cloud_rlast;
 wire selected_remote;wire[31:0]peer_source_session;
 wire[191:0]cloud_host_snapshot;wire cloud_host_busy;wire[31:0]cloud_published_job_memory;
 wire cloud_control_valid;wire[31:0]cloud_control_frame,cloud_control_session;
 reg cloud_snapshot_selected;reg[31:0]hd_seen_job;wire cloud_source_valid;
 wire hold_last_snapshot=cloud_snapshot_selected&&cloud_source_valid;
 wire cloud_picture=hold_last_snapshot;
 wire cloud_memory_enable=memory_verified;
 wire cloud_line_req,cloud_line_new,cloud_line_ready,cloud_line_write,cloud_line_done;
 wire[15:0]cloud_line_row;wire source_request,source_new,source_bank;wire[15:0]source_row;
 wire source_ready,source_write,source_done,source_active,source_owner;
 wire[15:0]cloud_rgb;wire cloud_pixel_valid;wire[31:0]cloud_job;
 wire[31:0]cloud_renders,cloud_rejects;wire[15:0]cloud_points;wire cloud_fault;wire[5:0]cloud_view,cloud_state;
 wire draw_valid,draw_ready,draw_clear,draw_bank,flush_request,flush_done,front_bank;
 wire[15:0]draw_address,draw_rgb,scan_rgb;wire scan_valid,scan_request,scan_ready,scan_new,scan_bank;
 wire[15:0]scan_row;wire scan_write,scan_done,scan_good,scan_return_bank;
 wire[127:0]scan_data;wire[7:0]scan_address;wire[31:0]scan_job,scan_session,scan_underflows;wire framebuffer_fault;
 wire[15:0]received_transport_row;wire[31:0]received_transport_id,received_transport_start,received_transport_session;
 wire[32:0]hd_scan_publication;wire[31:0]hd_scan_job,hd_scan_session;
 ph1p_video_memory #(.WIDTH(1280),.HEIGHT(960),.REMOTE_WIDTH(1280),.REMOTE_HEIGHT(720)) storage(.clk_50(board50_global),.rst_n(sys_rst_n),.debug_latch(take),.debug(memory_debug),
 .debug_read_peer(memory_snapshot_read_peer),.debug_read_index(memory_snapshot_read_index),.debug_read_byte(memory_snapshot_byte),.debug_snapshot_led(memory_snapshot_led),
 .memory_clk(memory_clk),.memory_rst_n(memory_rst_n),.memory_verified(memory_verified),.token_empty(fifo_empty),.token_read(fifo_read),.token_valid(fifo_valid),.token_data(fifo_data),
 .line_req_valid(line_request),.line_req_ready(line_ready),.new_frame(line_new),.source_row(line_row),.request_bank(line_request_bank),
 .line_write(line_write),.line_data(line_data),.line_address(line_address),.line_bank(line_bank),.line_done(line_done),.line_good(line_good),
 .line_frame_id(line_frame_id),.line_session(line_session),.time_gray(contest_time_gray),.line_start_ms(line_start_ms),
 .remote_token_empty(peer_fifo_empty),.remote_token_read(peer_fifo_read),.remote_token_valid(peer_fifo_valid),.remote_token_data(peer_fifo_data),
 .remote_line_req_valid(source_request),.remote_line_req_ready(source_ready),.remote_new_frame(source_new),.remote_source_row(source_row),.remote_request_bank(source_bank),
 .remote_line_write(source_write),.remote_line_data(peer_line_data),.remote_line_address(peer_line_address),.remote_line_bank(peer_line_bank),
 .remote_line_done(source_done),.remote_line_good(peer_line_good),.remote_line_frame_id(peer_line_frame_id),.remote_line_session(peer_line_session),
 .remote_latest_valid(peer_latest_valid),.remote_latest_frame_id(peer_latest_frame),.remote_latest_session(peer_latest_session),
 .remote_line_start_ms(peer_line_start_ms),.remote_debug(peer_memory_debug),
  .cloud_awaddr(cloud_awaddr),.cloud_araddr(cloud_araddr),.cloud_wdata(cloud_wdata),.cloud_wstrb(cloud_wstrb),
  .cloud_awvalid(cloud_awvalid),.cloud_wvalid(cloud_wvalid),.cloud_wlast(cloud_wlast),.cloud_arvalid(cloud_arvalid),.cloud_rready(cloud_rready),.cloud_read_active(cloud_read_active),
  .cloud_awready(cloud_awready),.cloud_wready(cloud_wready),.cloud_arready(cloud_arready),.cloud_rvalid(cloud_rvalid),.cloud_rlast(cloud_rlast),.cloud_rdata(cloud_rdata),
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
 wire [31:0] peer_underflows,peer_source_frame,peer_source_start_ms;
 // The source decision is sampled at x8, after x0 publication, in the same blank.
 wire native_requested_cloud=cloud_source_valid&&cloud_job!=hd_seen_job&&cloud_control_valid&&cloud_control_session[23]?1'b1:
  cloud_control_valid&&!cloud_control_session[23]?1'b0:cloud_snapshot_selected;
 wire native_request,native_ready,native_new,native_bank,native_memory_cloud;
 wire[15:0]native_row,native_rgb;wire native_valid,native_frame_valid;wire[31:0]native_frame,native_session;
 // AM94: native_lines is absent; its diagnostics advertise no 3D/peer publication.
 assign native_memory_cloud=0;
 assign peer_underflows=0;
 assign hd_scan_publication=0;
 assign native_rgb=0;
 assign native_valid=0;
 assign scan_underflows=0;
 assign native_frame=0;
 assign native_frame_valid=0;
 assign native_session=0;
 assign native_request=0;
 assign native_new=0;
 assign native_row=0;
 assign native_bank=0;
 assign native_ready=native_memory_cloud?scan_ready:peer_line_ready;
 assign peer_line_request=native_request&&!native_memory_cloud;assign peer_line_row=native_row;assign peer_line_new=native_new;assign peer_line_request_bank=native_bank;
 assign scan_request=native_request&&native_memory_cloud;assign scan_row=native_row;assign scan_new=native_new;assign scan_bank=native_bank;
 assign peer_image=native_rgb;assign peer_image_valid=native_valid&&!cloud_picture;
 assign cloud_rgb=native_rgb;assign cloud_pixel_valid=native_valid&&cloud_picture;
 assign scan_job=native_frame;assign scan_session=native_session;assign scan_valid=native_frame_valid&&cloud_picture;
 reg[161:0]transport_meta_requested;wire[161:0]transport_meta,transport_meta_ack;wire transport_meta_busy;
 always @(posedge memory_clk)if(!memory_rst_n)transport_meta_requested<=0;
 else begin
  transport_meta_requested[161:97]<={peer_latest_valid,peer_latest_frame,peer_latest_session};
  if(source_done&&peer_line_good)transport_meta_requested[96:0]<={1'b1,peer_line_frame_id,peer_line_start_ms,peer_line_session};
 end
 // AM94: transport_metadata is absent; its diagnostics advertise no 3D/peer publication.
 assign transport_meta_busy=0;
 assign transport_meta=0;
 assign transport_meta_ack=0;
 assign peer_source_valid=transport_meta[96];assign peer_source_frame=transport_meta[95:64];
 assign peer_source_start_ms=transport_meta[63:32];assign peer_source_session=transport_meta[31:0];
 assign cloud_control_valid=transport_meta[161];assign cloud_control_frame=transport_meta[160:129];assign cloud_control_session=transport_meta[128:97];
 // AM94: cloud_source_reader is absent; its diagnostics advertise no 3D/peer publication.
 assign peer_line_ready=0;
 assign peer_line_write=0;
 assign peer_line_done=0;
 assign cloud_line_ready=0;
 assign cloud_line_write=0;
 assign cloud_line_done=0;
 assign source_request=0;
 assign source_new=0;
 assign source_bank=0;
 assign source_row=0;
 assign source_active=0;
 assign source_owner=0;

 wire view_rx_valid,view_rx_busy,view_memory_valid,view_memory_ready;
 wire[255:0]view_rx_payload;wire[47:0]view_memory_record;wire[15:0]view_applied_token;
 wire[31:0]view_rx_good,view_rx_crc,view_rx_rejected;
 // AM94: view_capture is absent; its diagnostics advertise no 3D/peer publication.
 assign view_rx_busy=1'b1;
 assign view_memory_valid=0;
 assign view_memory_record=0;

 wire hd_packet_valid,hd_packet_consume,hd_reply_valid,hd_reply_ready;
 wire[191:0]hd_packet_header;wire[10:0]hd_rx_address;wire[7:0]hd_rx_byte,hd_reply_kind;
 wire[31:0]hd_reply_job,hd_latest_job,hd_latest_capture_job,hd_accepted_packets,hd_projected_points,hd_scan_complete_job;
 wire[15:0]hd_reply_sequence;wire hd_latest_valid;
 // AM94: hd_reconstruction is absent; its diagnostics advertise no 3D/peer publication.
 assign hd_packet_consume=0;
 assign hd_rx_address=0;
 assign hd_reply_valid=0;
 assign hd_reply_kind=0;
 assign hd_reply_job=0;
 assign hd_reply_sequence=0;
 assign view_memory_ready=0;
 assign view_applied_token=0;
 assign front_bank=0;
 assign cloud_job=0;
 assign cloud_published_job_memory=0;
 assign cloud_source_valid=0;
 assign hd_latest_valid=0;
 assign hd_latest_job=0;
 assign hd_latest_capture_job=0;
 assign scan_ready=0;
 assign scan_write=0;
 assign scan_return_bank=0;
 assign scan_done=0;
 assign scan_good=0;
 assign scan_data=0;
 assign scan_address=0;
 assign hd_scan_job=0;
 assign hd_scan_session=0;
 assign cloud_awaddr=0;
 assign cloud_araddr=0;
 assign cloud_wdata=0;
 assign cloud_wstrb=0;
 assign cloud_awvalid=0;
 assign cloud_wvalid=0;
 assign cloud_wlast=0;
 assign cloud_arvalid=0;
 assign cloud_rready=0;
 assign cloud_read_active=0;
 assign cloud_fault=0;
 assign cloud_renders=0;
 assign cloud_rejects=0;
 assign hd_projected_points=0;
 assign hd_accepted_packets=0;
 assign cloud_state=0;
 // Native raw-peer and cloud rows use the shared native_lines cache above.
 // Completion is checked after the actual HDMI selection and output registers below.
 assign cloud_points=hd_projected_points>65535?16'hffff:hd_projected_points[15:0];assign cloud_view=0;
 assign cloud_line_req=0;assign cloud_line_new=0;assign cloud_line_row=0;
 assign draw_ready=1;assign flush_done=1;assign scan_rgb=0;assign framebuffer_fault=cloud_fault;



 wire local_stale,peer_stale,peer_link_stale;
 stereo_view_control #(.BOARD_ID(BOARD_ID)) view_control(.clk(pixel),.rst_n(pixel_rst_n),.commit(x==0&&y==720),
  .local_valid(source_valid),.peer_valid(1'b0),.local_frame(source_frame_id),.peer_frame(32'd0),.peer_session(32'd0),
  .hold_peer(1'b0),
  .selected_remote(selected_remote),.local_stale(local_stale),.peer_stale(peer_stale),.peer_link_stale(peer_link_stale));
 // A published snapshot survives B's next calculation and video timeouts.
 // Only a complete native frame with depth-mode cleared exits the snapshot.
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin cloud_snapshot_selected<=0;hd_seen_job<=0;end
  // hd_present commits at x0. Sample the new publication in the same blank.
  else if(x==8&&y==720)begin
   if(cloud_source_valid&&cloud_job!=hd_seen_job&&cloud_control_valid&&cloud_control_session[23])begin cloud_snapshot_selected<=1;hd_seen_job<=cloud_job;end
   else if(cloud_control_valid&&!cloud_control_session[23])cloud_snapshot_selected<=0;
  end
 end

 wire key_switch;wire control_busy,requested_at_pixel,control_ack;
 reg requested_remote;reg [31:0] key_switch_count;
 gimbal_press_event #(.STABLE_CYCLES(500000)) switch_key(.clk(board50_global),.rst_n(sys_rst_n),.key_n(key_n[0]),.pressed(key_switch));
 wire control_write=BOARD_ID==2&&key_switch&&!control_busy;
 always @(posedge board50_global)begin
  if(!sys_rst_n)begin requested_remote<=0;key_switch_count<=0;end
  else if(control_write)begin requested_remote<=!requested_remote;key_switch_count<=key_switch_count+1'b1;end
 end
 cdc_mode #(.WIDTH(1)) control_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(sys_rst_n&&pll_v),
  .write(control_write),.mode(!requested_remote),.busy(control_busy),.commit(x==0&&y==720),
  .destination_mode(requested_at_pixel),.src_mode(control_ack));
 // NR changes commit in vertical blank; requested/applied/busy are separately readable.
 // Two source-clock release stages also reset the startup request after video PLL loss.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] noise_control_reset;
 wire noise_raw_reset=sys_rst_n&&pll_v;
 always @(posedge board50_global or negedge noise_raw_reset)
  if(!noise_raw_reset)noise_control_reset<=0;else noise_control_reset<={noise_control_reset[0],1'b1};
 wire noise_write,noise_value,noise_requested,noise_busy,noise_ack,noise_at_pixel;
 wire [31:0] noise_key_count;
 video_noise_control noise_control(.clk(board50_global),.rst_n(noise_control_reset[1]),.key_n(key_n[1]),
  .host_write(image_control_write&&image_control_address==16'h0280),.host_data(image_control_data),.busy(noise_busy),.applied_mode(noise_ack),
  .mode_write(noise_write),.mode_value(noise_value),.requested_mode(noise_requested),.key_count(noise_key_count));
 cdc_mode #(.WIDTH(1)) noise_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(noise_raw_reset),
  .write(noise_write),.mode(noise_value),.busy(noise_busy),.commit(x==0&&y==720),
  .destination_mode(noise_at_pixel),.src_mode(noise_ack));
 // A KEY3 selects eight modes; A KEY1 adjusts thresholds, B KEY1 selects camera.
 wire ui_write,ui_busy;wire [2:0] ui_value,ui_requested,ui_ack,ui_at_pixel;wire [31:0] ui_key_count;
 video_mode_control ui_control(.clk(board50_global),.rst_n(noise_control_reset[1]),.key_n(key_n[2]),
  .host_write(image_control_write&&image_control_address==16'h0290&&image_control_data[7:3]==0),.host_mode(image_control_data[2:0]),
  .applied_mode(ui_ack),.busy(ui_busy),.mode_write(ui_write),.mode_value(ui_value),.requested_mode(ui_requested),.key_count(ui_key_count));
 cdc_mode #(.WIDTH(3)) ui_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(noise_raw_reset),
  .write(ui_write),.mode(ui_value),.busy(ui_busy),.commit(x==0&&y==720),.destination_mode(ui_at_pixel),.src_mode(ui_ack));
 wire adjust_write,adjust_busy;wire [1:0] adjust_value,adjust_requested,adjust_ack,adjust_at_pixel;wire [31:0] adjust_key_count;
 video_mode_control #(.WIDTH(2)) adjust_control(.clk(board50_global),.rst_n(noise_control_reset[1]),.key_n(key_n[0]),
  .host_write(image_control_write&&image_control_address==16'h0291&&image_control_data[7:2]==0),.host_mode(image_control_data[1:0]),
  .applied_mode(adjust_ack),.busy(adjust_busy),.mode_write(adjust_write),.mode_value(adjust_value),.requested_mode(adjust_requested),.key_count(adjust_key_count));
 cdc_mode #(.WIDTH(2)) adjust_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(noise_raw_reset),
  .write(adjust_write),.mode(adjust_value),.busy(adjust_busy),.commit(x==0&&y==720),.destination_mode(adjust_at_pixel),.src_mode(adjust_ack));
 // Both image line caches align to this one-pixel-delayed raster.
 reg display_de,display_hs,display_vs;reg [15:0] display_x,display_y;
 reg [31:0] display_source_frame_id;reg display_source_frame_valid;reg [31:0] displayed_frames;
 // Latch both source identities at the display origin. Line-cache metadata
 // invalidates/refills in vertical blank, while analysis still finishes the
 // previous raster. A live remote-valid flag would abort every remote result.
 reg [31:0] display_source_session,display_source_start_ms;
 reg [31:0] display_peer_frame_id,display_peer_session,display_peer_start_ms;
 reg display_peer_frame_valid;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin display_source_session<=0;display_source_start_ms<=0;
   display_peer_frame_id<=0;display_peer_session<=0;display_peer_start_ms<=0;display_peer_frame_valid<=0;end
  else if(x==0&&y==0)begin display_source_session<=local_source_session;display_source_start_ms<=source_start_ms;
   display_peer_frame_id<=peer_source_frame;display_peer_session<=peer_source_session;
   display_peer_start_ms<=peer_source_start_ms;display_peer_frame_valid<=peer_source_valid;end
 end
 always @(posedge pixel)if(!pixel_rst_n)begin display_de<=0;display_hs<=1;display_vs<=1;displayed_frames<=0;display_x<=0;display_y<=0;display_source_frame_id<=0;display_source_frame_valid<=0;end
 else begin display_x<=x;display_y<=y;
 if(x==0&&y==0)begin display_source_frame_id<=source_frame_id;display_source_frame_valid<=source_valid;end
 display_de<=de;display_hs<=hs;display_vs<=vs;if(sof)displayed_frames<=displayed_frames+1'b1;end
 wire [23:0] local_rgb=image_valid?{image_pixel[15:11],image_pixel[15:13],image_pixel[10:5],image_pixel[10:9],image_pixel[4:0],image_pixel[4:2]}:24'h000000;
 
 // Supported controls stay in the existing 50MHz camera-control domain.
 // E2: 0=110..132,1=110..130,2=128..152,3=152..180; default2 matches B29 running preset.
 // E3: 0/3 tone bypass,1 moderate,2 bright; default0 avoids a second brightness lift.
 reg[1:0]ae_target_mode,tone_profile;
 always @(posedge board50_global)begin
  if(!sys_rst_n)begin ae_target_mode<=2;end
  else if(probe_write&&reg_addr==16'h00e2&&write_data[7:2]==0)ae_target_mode<=write_data[1:0];
 end
 wire tone_profile_busy;wire[1:0]tone_profile_pixel,tone_profile_ack;
 wire tone_profile_write=probe_write&&reg_addr==16'h00e3&&write_data[7:2]==0&&!tone_profile_busy;
 always @(posedge board50_global)begin
  if(!noise_control_reset[1])tone_profile<=0;
  else if(tone_profile_write)tone_profile<=write_data[1:0];
 end
 cdc_mode #(.WIDTH(2))tone_mode(.src_clk(board50_global),.dst_clk(pixel),.rst_n(noise_raw_reset),
 .write(tone_profile_write),.mode(write_data[1:0]),.busy(tone_profile_busy),.commit(x==0&&y==720),
 .destination_mode(tone_profile_pixel),.src_mode(tone_profile_ack));

 wire [23:0] local_chroma,local_tone;wire chroma_valid,tone_valid;
 video_chroma_correct chroma(.clk(pixel),.rst_n(pixel_rst_n),.flush(1'b0),.mode(2'd1),.rgb_in(local_rgb),.valid_in(display_de),.rgb_out(local_chroma),.valid_out(chroma_valid));
 video_tone_map tone(.clk(pixel),.rst_n(pixel_rst_n),.mode(tone_profile_pixel),.rgb_in(local_chroma),.valid_in(chroma_valid),.rgb_out(local_tone),.valid_out(tone_valid));
 reg [15:0] colour_x[0:5],colour_y[0:5],remote_pixels[0:5];
 reg [5:0] colour_de,colour_hs,colour_vs,local_valids,remote_valids;
 integer colour_index;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin
   colour_de<=0;colour_hs<=6'b111111;colour_vs<=6'b111111;local_valids<=0;remote_valids<=0;
   for(colour_index=0;colour_index<6;colour_index=colour_index+1)begin colour_x[colour_index]<=0;colour_y[colour_index]<=0;remote_pixels[colour_index]<=0;end
  end else begin
   colour_de<={colour_de[4:0],display_de};colour_hs<={colour_hs[4:0],display_hs};colour_vs<={colour_vs[4:0],display_vs};
   local_valids<={local_valids[4:0],image_valid};remote_valids<={remote_valids[4:0],cloud_picture?cloud_pixel_valid:peer_image_valid};
   colour_x[0]<=display_x;colour_y[0]<=display_y;remote_pixels[0]<=cloud_picture?cloud_rgb:peer_image;
   for(colour_index=1;colour_index<6;colour_index=colour_index+1)begin colour_x[colour_index]<=colour_x[colour_index-1];colour_y[colour_index]<=colour_y[colour_index-1];remote_pixels[colour_index]<=remote_pixels[colour_index-1];end
  end
 end
 // Filter once on the local display/transmit branch; camera DDR and AE meter stay unfiltered.
 wire [23:0] filtered_local;wire [15:0] filtered_x,filtered_y;
 wire filtered_de,filtered_valid,filtered_hs,filtered_vs;
 video_denoise_2row #(.WIDTH(1280),.HEIGHT(720),.OFFSET_X(0),.THRESHOLD_2D(48),.THRESHOLD_1D(32)) display_noise(
  .clk(pixel),.rst_n(pixel_rst_n),.enable(noise_at_pixel),.rgb_in(local_tone),.x_in(colour_x[5]),.y_in(colour_y[5]),
  .de_in(colour_de[5]),.valid_in(local_valids[5]),.hs_in(colour_hs[5]),.vs_in(colour_vs[5]),
  .rgb_out(filtered_local),.x_out(filtered_x),.y_out(filtered_y),.de_out(filtered_de),
  .valid_out(filtered_valid),.hs_out(filtered_hs),.vs_out(filtered_vs));
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
 wire [23:0] rgb=cloud_picture?(filtered_peer_valid?remote_rgb:24'd0):selected_remote?(filtered_peer_valid&&!peer_stale?remote_rgb:24'd0):(filtered_valid&&!local_stale?filtered_local:24'd0);
 // Process the selected picture once. A->B transmits this final HDMI view;
 // B->A remains B's own camera, so a returned video cannot feed itself.
 wire selected_source_valid=cloud_picture?cloud_source_valid:selected_remote?display_peer_frame_valid:display_source_frame_valid;
 wire selected_stale=cloud_picture?1'b0:selected_remote?peer_stale:local_stale;
 wire selected_pixel_valid=cloud_picture?filtered_peer_valid:selected_remote?filtered_peer_valid:filtered_valid;
 wire [31:0] selected_frame=cloud_picture?cloud_job:selected_remote?display_peer_frame_id:display_source_frame_id;
 wire [31:0] selected_start_ms=cloud_picture?32'd0:selected_remote?display_peer_start_ms:display_source_start_ms;
 wire [32:0] selected_epoch=cloud_picture?{1'b1,32'hD7200001}:{selected_remote,selected_remote?display_peer_session:display_source_session};
 wire [15:0] final_x,final_y;wire [23:0] final_rgb;wire final_de,final_valid,final_hs,final_vs;
 wire follow_commit,follow_overflow;wire[31:0]follow_frame;wire[3:0]follow_valid;
 wire[63:0]follow_x0,follow_x1,follow_y0,follow_y1;
 wire [15:0] contest_fps,contest_latency,contest_track_id,contest_cx,contest_cy,contest_motion_count;
 wire contest_fps_valid,contest_latency_valid,contest_overflow,contest_track_visible,contest_motion,contest_lighting;
 wire [7:0] contest_shown;
 wire[31:0]box_mcu_rdata,box_mcu_wdata;wire[3:0]box_mcu_read_word,box_mcu_write_word;
 wire box_mcu_write,box_mcu_worker_ready;
 contest_video_pipeline contest(.clk(pixel),.rst_n(pixel_rst_n),
  .box_service_clk(board50_global),.box_global_rst_n(sys_rst_n),.box_worker_ready(box_mcu_worker_ready),
  .box_read_word(box_mcu_read_word),.box_write_word(box_mcu_write_word),.box_write(box_mcu_write),.box_wdata(box_mcu_wdata),.box_rdata(box_mcu_rdata),.mode(cloud_picture||(selected_remote&&(display_peer_session[31:24]==8'hD2||(display_peer_session[31:24]==8'hD3||display_peer_session[31:24]==8'hD4||display_peer_session[31:24]==8'hD5))) ? 3'd0:ui_at_pixel),.adjustment(adjust_at_pixel),
  .x(filtered_x),.y(filtered_y),.rgb(rgb),.de(filtered_de),.valid(selected_pixel_valid&&!selected_stale),
  .hs(filtered_hs),.vs(filtered_vs),.camera_remote(selected_remote),
  .source_valid(selected_source_valid),.source_stale(selected_stale),.source_epoch(selected_epoch),
  .source_frame_id(selected_frame),.source_start_ms(selected_start_ms),
  .out_x(final_x),.out_y(final_y),.out_rgb(final_rgb),.out_de(final_de),.out_valid(final_valid),.out_hs(final_hs),.out_vs(final_vs),
  .time_gray(contest_time_gray),.fps(contest_fps),.latency_ms(contest_latency),.fps_valid(contest_fps_valid),.latency_valid(contest_latency_valid),
  .follow_commit(follow_commit),.follow_overflow(follow_overflow),.follow_frame(follow_frame),
  .follow_valid(follow_valid),.follow_x0(follow_x0),.follow_x1(follow_x1),.follow_y0(follow_y0),.follow_y1(follow_y1),
  .shown_valid(contest_shown),.detector_overflow(contest_overflow),.track_visible(contest_track_visible),
  .track_id(contest_track_id),.center_x(contest_cx),.center_y(contest_cy),.motion_found(contest_motion),
  .lighting_change(contest_lighting),.motion_count(contest_motion_count));
 // Mono D1 carries this A final processed1280x720 view to B91 for display.
 // The source frame/epoch latch below follows final x/y/de, including OSD latency.
 wire raw_for_stereo=1'b0;wire[31:0]typed_session;
 // Legacy diagnostic fields now describe the canonical transport, not a
 // second render stream. Keep register widths compatible with the host.
 reg[31:0]final_stream_frame,final_stream_session;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin final_stream_frame<=0;final_stream_session<=0;end
  else if(final_de&&final_x==0&&final_y==0)begin final_stream_frame<=selected_frame;final_stream_session<=typed_session;end
 end
 wire[31:0]render_frame=final_stream_frame,render_session=final_stream_session,render_bad=32'd0;
 wire[15:0]raw_sample_x,raw_sample_y;wire[23:0]raw_sample_rgb;wire raw_sample_de;
 stereo_sample_720p raw_sample(.clk(pixel),.rst_n(pixel_rst_n),.x(final_x),.y(final_y),.de(final_de),
  .valid(final_valid&&selected_source_valid&&!selected_stale),.rgb(final_rgb),
  .out_x(raw_sample_x),.out_y(raw_sample_y),.out_rgb(raw_sample_rgb),.out_de(raw_sample_de));
 wire[21:0]typed_epoch;wire typed_epoch_exhausted;
 nv32_session_epoch typed_epoch_counter(.clk(pixel),.rst_n(pixel_rst_n),.frame_origin(x==0&&y==0),
  .context_word({31'd0,selected_remote}),.epoch(typed_epoch),.exhausted(typed_epoch_exhausted));
 assign typed_session={raw_for_stereo?8'hA1:8'hD1,raw_for_stereo,selected_remote,typed_epoch};
 wire queue_reset_n=sys_rst_n&&phy_ready&&pll_v;
 wire tx_packet_valid,tx_packet_ready,tx_packet_done,tx_packet_good,queue_tx_reset_n;
 wire [31:0] tx_session,tx_frame,queued_frames,queued_drops,tx_frame_counter,sent_packets,sent_frames,last_sent_frame,mac_packets,mac_drops;
 wire [15:0] tx_packet_index,tx_word_data;wire [9:0] tx_word_addr;wire [2:0] tx_high_water;
 wire request_or_ack=BOARD_ID==1?selected_remote:requested_at_pixel;
 wire [31:0] source_session_word={request_or_ack,31'h4e563100}|BOARD_ID;
 nv_display_queue #(.WIDTH(1280),.HEIGHT(720),.PACKET_PIXELS(640),.DYNAMIC_META(1)) queue(
 .pixel_clk(pixel),.tx_clk(tx_clk),.rst_n(queue_reset_n),.tx_domain_rst_n(queue_tx_reset_n),
 .rgb(raw_sample_rgb),.x(raw_sample_x),.y(raw_sample_y),
 .de(!typed_epoch_exhausted&&raw_sample_de),.source_frame_id(final_stream_frame),.source_session(final_stream_session),
 .packet_valid(tx_packet_valid),.packet_ready(tx_packet_ready),.packet_session(tx_session),.packet_frame(tx_frame),.packet_index(tx_packet_index),
 .packet_word_addr(tx_word_addr),.packet_word_data(tx_word_data),.packet_done(tx_packet_done),.packet_good(tx_packet_good),
 .frames_queued(queued_frames),.frames_dropped(queued_drops),.frame_counter(tx_frame_counter),.packets_sent(sent_packets),.frames_sent(sent_frames),.last_frame_sent(last_sent_frame),.high_water(tx_high_water));
 wire follow_publish,follow_mailbox_busy,follow_control_valid,follow_control_request,follow_control_busy;
 wire follow_control_done;reg follow_tx_is_ack;
 wire[255:0]follow_outgoing,follow_payload,follow_target_payload;wire follow_target_valid;wire follow_video_admit,follow_video_ready,follow_video_busy;
 wire view_ack_busy,view_ack_valid;wire[15:0]view_ack_data;reg[15:0]view_last_ack;
 always @(posedge tx_clk or negedge queue_tx_reset_n)
  if(!queue_tx_reset_n)follow_tx_is_ack<=0;else if(follow_control_request)follow_tx_is_ack<=view_ack_valid;
 wire view_ack_write=!view_ack_busy&&view_applied_token!=0&&(view_applied_token!=view_last_ack||view_memory_valid);
 always @(posedge memory_clk)begin if(!memory_rst_n)view_last_ack<=0;else if(view_ack_write)view_last_ack<=view_applied_token;end
 // AM94: view_ack_mailbox is absent; its diagnostics advertise no 3D/peer publication.
 assign view_ack_busy=0;
 assign view_ack_valid=0;
 assign view_ack_data=0;
 assign follow_control_valid=view_ack_valid||follow_target_valid;
 assign follow_payload=(follow_control_busy?follow_tx_is_ack:view_ack_valid)?{32'h5641434b,32'd0,view_ack_data,2'b11,174'd0}:follow_target_payload;
 wire[7:0]follow_video_data,follow_control_data;wire follow_video_enable,follow_control_enable;
 follow_target_packet follow_publisher(.clk(pixel),.rst_n(queue_reset_n),.result_commit(follow_commit),
  .source_valid(selected_source_valid&&!selected_stale&&!selected_remote&&!cloud_picture),.overflow(follow_overflow),
  .epoch(selected_remote?display_peer_session:display_source_session),.frame_id(follow_frame),.valid(follow_valid),
  .x0(follow_x0),.x1(follow_x1),.y0(follow_y0),.y1(follow_y1),.mailbox_busy(follow_mailbox_busy),
  .mailbox_write(follow_publish),.mailbox_data(follow_outgoing));
 follow_payload_mailbox follow_mailbox(.source_clk(pixel),.destination_clk(tx_clk),.rst_n(queue_reset_n),
  .source_write(follow_publish),.source_data(follow_outgoing),.source_busy(follow_mailbox_busy),
  .valid(follow_target_valid),.data(follow_target_payload),.consume(follow_control_done&&!follow_tx_is_ack));
 follow_control_mac #(.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC),.LOCAL_ROLE(1),.PEER_ROLE(2),.HELD_TX_PAYLOAD(1),.DIRECT_RX_PAYLOAD(1),.RX_PREFIX82(1))follow_mac(
  .tx_clk(tx_clk),.tx_rst_n(queue_tx_reset_n),.tx_request(follow_control_request),.tx_payload(follow_payload),
  .tx_busy(follow_control_busy),.tx_done(follow_control_done),.gmii_txd(follow_control_data),.gmii_tx_en(follow_control_enable),
  .rx_clk(rx_clk),.rx_rst_n(peer_pipeline_reset[1]),.gmii_rxd(8'd0),.gmii_rx_dv(1'b0),.gmii_rx_er(1'b0),
  .rx_valid(view_rx_valid),.rx_payload(view_rx_payload),.rx_good(view_rx_good),.rx_bad_crc(view_rx_crc),.rx_rejected(view_rx_rejected));
 follow_video_arbiter follow_arbiter(.clk(tx_clk),.rst_n(queue_tx_reset_n),
  .video_valid(tx_packet_valid&&legacy_gate),.video_ready(follow_video_ready),.video_busy(follow_video_busy),
  .video_data(follow_video_data),.video_enable(follow_video_enable),.control_valid(follow_control_valid&&legacy_gate),
  .control_busy(follow_control_busy),.control_data(follow_control_data),.control_enable(follow_control_enable),
  .video_admit(follow_video_admit),.video_accept(tx_packet_ready),.control_request(follow_control_request),.control_accept(),
  .gmii_data(legacy_gmii_data),.gmii_enable(legacy_gmii_enable));

 // AM94: hd_network is absent; its diagnostics advertise no 3D/peer publication.
 assign hd_reply_ready=0;
 assign hd_packet_valid=0;
 assign hd_packet_header=0;
 assign hd_rx_byte=0;
 assign hd_tx_offer=0;
 assign hd_tx_busy=0;
 assign hd_gmii_data=0;
 assign hd_gmii_enable=0;
 hd_wire_arbiter hd_wire(.clk(tx_clk),.rst_n(sys_rst_n&&memory_rst_n&&pll_v),.enabled(phy_ready),
  .legacy_busy(follow_video_busy||follow_control_busy),.legacy_data(legacy_gmii_data),.legacy_enable(legacy_gmii_enable),.legacy_gate(legacy_gate),
  .hd_offer(hd_tx_offer),.hd_busy(hd_tx_busy),.hd_data(hd_gmii_data),.hd_enable(hd_gmii_enable),.hd_accept(hd_tx_accept),.gmii_data(gmii_txd),.gmii_enable(gmii_tx_en));
 nv32_video_tx #(.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC),.WIDTH(1280),.HEIGHT(720),.PACKET_PIXELS(640),.STREAM_ID(LOCAL_STREAM),.HELD_PACKET_METADATA(1)) transmitter(
 .clk(tx_clk),.rst_n(queue_tx_reset_n),.packet_valid(follow_video_admit),.packet_ready(follow_video_ready),.packet_session(tx_session),.packet_frame(tx_frame),.packet_index(tx_packet_index),
 .packet_word_addr(tx_word_addr),.packet_word_data(tx_word_data),.packet_done(tx_packet_done),.packet_good(tx_packet_good),.busy(follow_video_busy),.tx_packets(mac_packets),.tx_dropped(mac_drops),.gmii_txd(follow_video_data),.gmii_tx_en(follow_video_enable));

 wire [9:0] ch0,ch1,ch2;
 // HDMI-only status; canonical/network samples above remain unmodified.
 wire[23:0]hdmi_rgb;wire hdmi_de,hdmi_hs,hdmi_vs;
 // Preserve both register stages of the status/official-logo path.
 reg[15:0]hdmi_x0,hdmi_y0,hdmi_x1,hdmi_y1;reg[1:0]hdmi_valid;
 always @(posedge pixel)begin
  if(!pixel_rst_n)begin hdmi_x0<=0;hdmi_y0<=0;hdmi_x1<=0;hdmi_y1<=0;hdmi_valid<=0;end
  else begin hdmi_x0<=final_x;hdmi_y0<=final_y;hdmi_x1<=hdmi_x0;hdmi_y1<=hdmi_y0;hdmi_valid<={hdmi_valid[0],final_valid&&cloud_picture};end
 end
 // AM94: depth_status is absent; its diagnostics advertise no 3D/peer publication.
 assign hdmi_rgb=final_rgb;
 assign hdmi_de=final_de;
 assign hdmi_hs=final_hs;
 assign hdmi_vs=final_vs;
 // AM94: hd_completed_view is absent; its diagnostics advertise no 3D/peer publication.
 assign hd_scan_complete_job=0;
 tmds_encoder enc0(.clk(pixel),.rst_n(pixel_rst_n),.data(hdmi_rgb[7:0]),.de(hdmi_de),.ctrl({hdmi_vs,hdmi_hs}),.symbol(ch0));
 tmds_encoder enc1(.clk(pixel),.rst_n(pixel_rst_n),.data(hdmi_rgb[15:8]),.de(hdmi_de),.ctrl(2'b0),.symbol(ch1));
 tmds_encoder enc2(.clk(pixel),.rst_n(pixel_rst_n),.data(hdmi_rgb[23:16]),.de(hdmi_de),.ctrl(2'b0),.symbol(ch2));
 ph1p_tmds_output output_phy(.pixel(pixel),.serial(serial),.reset(!pixel_rst_n),.ch0(ch0),.ch1(ch1),.ch2(ch2),
 .data_p(hdmi_data_p),.data_n(hdmi_data_n),.clock_p(hdmi_clock_p),.clock_n(hdmi_clock_n));

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
    reg ae_enable;wire ae_request,ae_grant,ae_cmd_valid,ae_cmd_ready,ae_cmd_read,ae_cmd_done;
    wire [15:0] ae_address;wire [7:0] ae_value;
    wire ae_busy,ae_fault,ae_stable,ae_limit,ae_stale;wire [7:0] ae_fault_code,ae_coarse,ae_fine;
    wire [11:0] ae_half;wire [15:0] ae_updates;wire [3:0] ae_settling;
    always @(posedge board50_global) begin
        if(!sys_rst_n) ae_enable<=1;
        else begin
            if(image_control_write&&image_control_address==16'h00e0) ae_enable<=image_control_data[0];
            // Manual camera writes freeze AE before the queued host command is issued.
            if(cam_valid&&!cam_read) ae_enable<=0;
        end
    end
    // 2026-10-10: 30 ms exposure suppressed measured room-light banding by99%.
    // At30 fps/VTS2000,1200 half-lines =10 ms. Switch10/20/30 ms atomically
    // with gain compensation; preserve short exposure for very bright scenes.
    wire awb_mcu_done,awb_mcu_mailbox_busy,awb_classification_valid;
    wire[19:0]awb_mcu_classification,awb_classification;
    wire[145:0]awb_compact_status;
    mcu_auto_exposure #(.FIRMWARE_FILE("V96_1_Monocular_HMI/project/rtl/mcu_ae_firmware.mem"),.SAMPLES(3600),.LOW(110),.HIGH(132),.SETTLE_FRAMES(3),.BOOT_HALF(3600),.MAX_HALF(3600),.FLICKER_HALF(1200),.RUNTIME_TARGET(1)) ae(.clk(board50_global),.rst_n(sys_rst_n),
        .box_rdata(box_mcu_rdata),.box_read_word(box_mcu_read_word),.box_write_word(box_mcu_write_word),
        .box_write(box_mcu_write),.box_wdata(box_mcu_wdata),.box_worker_ready(box_mcu_worker_ready),.enable(ae_enable),.boot_ready(boot_done&&!boot_failed),.target_mode(ae_target_mode),
        .awb_red(awb_compact_status[105:86]),.awb_green(awb_compact_status[125:106]),.awb_blue(awb_compact_status[145:126]),
        .awb_sequence(awb_compact_status[40:25]),.awb_valid(awb_compact_status[0]),.awb_confidence(awb_compact_status[1]),
        .awb_consumer_ready(!awb_mcu_mailbox_busy),.awb_done(awb_mcu_done),.awb_classification(awb_mcu_classification),
        .meter_valid(ae_meter[0]),.meter_sum(ae_meter[20:1]),.meter_count(ae_meter[32:21]),
        .meter_highlights(ae_meter[44:33]),.meter_sequence(ae_meter[92:77]),
        .request(ae_request),.grant(ae_grant),.cmd_valid(ae_cmd_valid),.cmd_ready(ae_cmd_ready),.cmd_read(ae_cmd_read),
        .cmd_address(ae_address),.cmd_value(ae_value),.cmd_done(ae_cmd_done),.cmd_error(cam_error),.read_data(cam_data),
        .busy(ae_busy),.hold_motion(ae_hold_motion),.fault(ae_fault),.stable(ae_stable),.at_limit(ae_limit),.stale(ae_stale),
        .fault_code(ae_fault_code),.half_lines(ae_half),.gain_coarse(ae_coarse),.gain_fine(ae_fine),.updates(ae_updates),.settling(ae_settling));
    // E1 write freezes this 24-byte control-domain record for a coherent burst read at 0150.
    reg [191:0] ae_snapshot;
    always @(posedge board50_global) begin
        if(!sys_rst_n) ae_snapshot<=0;
        else if(probe_write&&reg_addr==16'h00e1) ae_snapshot<={12'd0,ae_settling,ae_meter[92:77],ae_updates,
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
    wire [31:0] rx_timeouts;
    reg packet_toggle,good_toggle,ecc_toggle,crc_toggle,fs_toggle,fe_toggle;
    ph1p_mipi_rx receiver(.clk_50(board50_global),.rst_n(sys_rst_n),.enable(rx_enable),
        .packet_done_toggle(packet_toggle),.clock_p(mipi_clock_p),.clock_n(mipi_clock_n),
        .data_p(mipi_data_p),.data_n(mipi_data_n),.byte_clk(byte_clk),.data(raw_bytes),
        .hs_active(hs_active),.lp_state(lp_state),.timeouts(rx_timeouts));
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
    wire[31:0]good_packets=32'd0,total_packets=32'd0;
    reg [31:0]ecc_errors,crc_errors,frame_starts,frame_ends;
    always @(posedge board50_global) begin
        if(!sys_rst_n || !rx_enable) begin
            event_sync0<=0; event_sync1<=0; event_previous<=0;
             ecc_errors<=0; crc_errors<=0; 
            frame_starts<=0; frame_ends<=0;
        end else begin
            event_sync0<={fe_toggle,fs_toggle,crc_toggle,ecc_toggle,good_toggle,packet_toggle};
            event_sync1<=event_sync0; event_previous<=event_sync1;
            
            
            if(event_sync1[2]!=event_previous[2]) ecc_errors<=ecc_errors+1'b1;
            if(event_sync1[3]!=event_previous[3]) crc_errors<=crc_errors+1'b1;
            if(event_sync1[4]!=event_previous[4]) frame_starts<=frame_starts+1'b1;
            if(event_sync1[5]!=event_previous[5]) frame_ends<=frame_ends+1'b1;
        end
    end
    wire [31:0] camera_frequency,byte_frequency;
    // Retired redundant camera_meter; real clocks remain connected to image/PHY logic.
    assign camera_frequency=32'd0;
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
 else if(image_control_write&&image_control_address==16'h0300&&image_control_data[7:1]==0)awb_enable<=image_control_data[0];
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]awb_enable_sync,awb_hold_sync;
 always @(posedge byte_clk)if(!byte_rst_n)begin awb_enable_sync<=0;awb_hold_sync<=2'b11;end
 else begin awb_enable_sync<={awb_enable_sync[0],awb_enable};
 awb_hold_sync<={awb_hold_sync[0],ae_hold_motion||!ae_stable||!ae_enable};end
 wire awb_result_valid,awb_confidence,awb_at_limit;
 wire[15:0]awb_sequence,awb_updates;wire[12:0]awb_count;
 wire[19:0]awb_sum_r,awb_sum_g,awb_sum_b;wire[7:0]awb_min,awb_max;
 neutral_awb_offload awb(.classification_valid(awb_classification_valid),.classification(awb_classification),.clk(byte_clk),.rst_n(byte_rst_n),.frame_start(p_fs&&packet_vc==0),
 .input_fault((p_done&&raw_packet&&!p_good)||raw_partial||fifo_overflow),
 .enable(awb_enable_sync[1]),.allow_update(!awb_hold_sync[1]),.sample_valid(awb_linear_valid),
 .linear_r(awb_linear_r),.linear_g(awb_linear_g),.linear_b(awb_linear_b),
 .gain_r(awb_gain_r),.gain_b(awb_gain_b),.result_valid(awb_result_valid),.confidence(awb_confidence),.at_limit(awb_at_limit),
 .result_sequence(awb_sequence),.updates(awb_updates),.neutral_count(awb_count),
 .sum_r(awb_sum_r),.sum_g(awb_sum_g),.sum_b(awb_sum_b),.neutral_min(awb_min),.neutral_max(awb_max));
 follow_payload_mailbox #(.WIDTH(20))awb_decision_mailbox(.source_clk(board50_global),.destination_clk(byte_clk),.rst_n(sys_rst_n&&rx_enable),
 .source_write(awb_mcu_done),.source_data(awb_mcu_classification),.source_busy(awb_mcu_mailbox_busy),
 .valid(awb_classification_valid),.data(awb_classification),.consume(p_fs&&packet_vc==0));
 wire[191:0]awb_byte_status={12'd0,awb_sum_b,12'd0,awb_sum_g,12'd0,awb_sum_r,
 3'd0,awb_max,awb_min,awb_count,awb_updates,awb_sequence,7'd0,awb_gain_b,awb_gain_r,awb_at_limit,awb_confidence,awb_result_valid};
 wire[145:0]awb_host_compact;wire[191:0]awb_control_status;wire awb_capture_busy;
 cdc_awb_eram_host_v2 #(.WIDTH(146))awb_capture(.source_clk(byte_clk),.read_clk(board50_global),.rst_n(sys_rst_n&&rx_enable),
 .host_reset_n(sys_rst_n),.host_take(probe_write&&reg_addr==16'h0301),.host_data(awb_host_compact),
 .take(!awb_capture_busy),.source_data({awb_sum_b,awb_sum_g,awb_sum_r,awb_max,awb_min,awb_count,awb_updates,awb_sequence,awb_gain_b,awb_gain_r,awb_at_limit,awb_confidence,awb_result_valid}),.busy(awb_capture_busy),.read_data(awb_compact_status),.completed_sequence());
 assign awb_control_status={12'd0,awb_compact_status[145:126],12'd0,awb_compact_status[125:106],12'd0,awb_compact_status[105:86],
 3'd0,awb_compact_status[85:78],awb_compact_status[77:70],awb_compact_status[69:57],awb_compact_status[56:41],awb_compact_status[40:25],7'd0,awb_compact_status[24:0]};
 wire[191:0]awb_host_snapshot={12'd0,awb_host_compact[145:126],12'd0,awb_host_compact[125:106],12'd0,awb_host_compact[105:86],
 3'd0,awb_host_compact[85:78],awb_host_compact[77:70],awb_host_compact[69:57],awb_host_compact[56:41],awb_host_compact[40:25],7'd0,awb_host_compact[24:0]};

 wire [31:0] rgb_pair;wire rgb_pair_valid;
 // Temporary WB imported from A's current measured profile. B calibration is
 // outstanding; these values are not a claim that both sensors match.
 bayer_bin2 #(.WIDTH(2560),.BLACK(64),.GAIN_R(335),.GAIN_G(256),.GAIN_B(346),.RUNTIME_CONFIG(1)) isp(
 .clk(byte_clk),.rst_n(byte_rst_n),.frame_start(p_fs&&packet_vc==0),
 .line_start(p_header&&raw_packet),.line_end(p_done&&raw_packet),.pixels(raw_pixels),.pixels_valid(raw_valid),
 .rgb(rgb_pair),.valid(rgb_pair_valid),.runtime_black(10'd64),
 .runtime_gain_r(awb_gain_r),.runtime_gain_g(11'd256),.runtime_gain_b(awb_gain_b),
 .meter_linear_r(awb_linear_r),.meter_linear_g(awb_linear_g),.meter_linear_b(awb_linear_b),.meter_linear_valid(awb_linear_valid));
 wire [31:0] camera_accepted,camera_dropped;
    camera_frame_pack #(.RAW_WIDTH(2560),.RAW_HEIGHT(1920)) packer(.clk(byte_clk),.rst_n(byte_rst_n),.frame_start(p_fs && packet_vc==0),.frame_end(p_fe && packet_vc==0),
        .line_start(p_header && raw_packet),.line_end(p_done && raw_packet),.line_good(p_good && !raw_partial),.line_bytes(packet_wc),
        .pair(rgb_pair),.pair_valid(rgb_pair_valid),.fifo_full(fifo_full),.fifo_data(packed_data),.fifo_write(packed_write),.accepted(camera_accepted),.dropped(camera_dropped));
    video_async_fifo video_fifo(.write_clk(byte_clk),.read_clk(memory_clk),.rst_n(sys_rst_n && memory_rst_n),
        .write_data(packed_data),.write_en(packed_write),.full(fifo_full),.overflow(fifo_overflow),
        .read_en(fifo_read),.read_data(fifo_data),.read_valid(fifo_valid),.empty(fifo_empty));
 // All camera packet counters below already belong to board50_global.
 // Camera accepted/dropped cross only through the frozen source mailbox.
 wire [127:0] pixel_snapshot;wire [63:0] front_snapshot,camera_snapshot;
 wire pixel_busy,front_busy,camera_busy;wire [15:0] pixel_sequence,front_sequence,camera_sequence;
 cdc_status_snapshot #(.WIDTH(64),.DIRECT_HELD_READ(1))camera_capture(.read_initialized(camera_capture_initialized),.source_clk(byte_clk),.read_clk(board50_global),.rst_n(sys_rst_n),.take(take),
 .source_data({camera_dropped,camera_accepted}),.busy(camera_busy),.read_data(camera_snapshot),.completed_sequence(camera_sequence));
 wire[96:0]pixel_compact_snapshot;
 assign pixel_snapshot={pixel_compact_snapshot[96:65],31'd0,pixel_compact_snapshot[64:0]};
 cdc_status_snapshot #(.WIDTH(97),.DIRECT_HELD_READ(1))pixel_capture(.read_initialized(pixel_capture_initialized),.source_clk(pixel),.read_clk(board50_global),.rst_n(pixel_raw),.take(take),
 .source_data({displayed_frames,source_valid,source_frame_id,underflows}),.busy(pixel_busy),.read_data(pixel_compact_snapshot),.completed_sequence(pixel_sequence));
 cdc_status_snapshot #(.WIDTH(64),.DIRECT_HELD_READ(1))front_capture(.read_initialized(front_capture_initialized),.source_clk(memory_clk),.read_clk(board50_global),.rst_n(sys_rst_n&&memory_rst_n),.take(take),
 .source_data({line_session,line_frame_id}),.busy(front_busy),.read_data(front_snapshot),.completed_sequence(front_sequence));
 wire [31:0] pixel_frequency,rx_frequency;
 // Retired redundant pixel_meter; real clocks remain connected to image/PHY logic.
    assign pixel_frequency=32'd0;
 // Retired redundant rx_meter; real clocks remain connected to image/PHY logic.
    assign rx_frequency=32'd0;
 reg [1:0] video_lock_sync,memory_ready_sync;
 always @(posedge board50_global)if(!sys_rst_n)begin video_lock_sync<=0;memory_ready_sync<=0;end
 else begin video_lock_sync<={video_lock_sync[0],pll_v};memory_ready_sync<={memory_ready_sync[0],memory_rst_n};end
 wire [15:0] status={8'b0,front_busy,pixel_busy,camera_busy,mdio_error,memory_ready_sync[1],video_lock_sync[1],mdio_done,phy_ready};
 assign led={memory_snapshot_led,phy_ready,boot_failed,boot_done&&!boot_failed};
 reg [255:0] camera_control_snapshot;
 always @(posedge board50_global)begin
  if(!sys_rst_n)camera_control_snapshot<=0;
  else if(take)camera_control_snapshot<={16'd0,boot_chip,frame_ends,frame_starts,crc_errors,ecc_errors,good_packets,total_packets,rx_timeouts};
 end
 
 wire [255:0] peer_rx_snapshot;wire [159:0] link_pixel_snapshot;wire [159:0] link_tx_snapshot;
 wire peer_rx_busy,link_pixel_busy,link_tx_busy;
 wire [15:0] peer_rx_sequence,link_pixel_sequence,link_tx_sequence;
 // AM94: peer_capture is absent; its diagnostics advertise no 3D/peer publication.
 assign peer_capture_initialized=0;
 assign peer_rx_busy=0;
 assign peer_rx_snapshot=0;
 assign peer_rx_sequence=0;
 // Legacy link_pixel snapshot page retired; main video/receiver/AE diagnostics remain.
 assign link_pixel_snapshot=0;assign link_pixel_busy=0;assign link_pixel_sequence=0;
 // Legacy link_tx snapshot page retired; main video/receiver/AE diagnostics remain.
 assign link_tx_snapshot=0;assign link_tx_busy=0;assign link_tx_sequence=0;
 wire [207:0] contest_snapshot;wire contest_snapshot_busy;wire [15:0] contest_snapshot_sequence;
 cdc_status_snapshot #(.WIDTH(208),.DIRECT_HELD_READ(1))contest_capture(.read_initialized(contest_capture_initialized),.source_clk(pixel),.read_clk(board50_global),.rst_n(pixel_raw),.take(take),
  .source_data({contest_fps,contest_latency,
   1'b1,selected_remote,contest_fps_valid,contest_latency_valid,contest_track_visible,contest_motion,contest_lighting,contest_overflow,
   contest_track_id,contest_cx,contest_cy,contest_motion_count,render_frame,render_session,render_bad,contest_shown}),
  .busy(contest_snapshot_busy),.read_data(contest_snapshot),.completed_sequence(contest_snapshot_sequence));
 reg [7:0] probe_data;

 wire[2:0]probe_byte_motor_diagnostic=reg_addr[2:0]-3'd0;
 wire[1:0]probe_byte_pixel_frequency=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_rx_frequency=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_camera_frequency=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_byte_frequency=reg_addr[1:0]-2'd0;
 wire[6:0]probe_byte_memory_debug=reg_addr[6:0]-7'd64;
 wire[4:0]probe_byte_awb_host_snapshot=reg_addr[4:0]-5'd16;
 wire[0:0]probe_byte_camera_sequence=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_pixel_sequence=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_front_sequence=reg_addr[0:0]-1'd0;
 wire[2:0]probe_byte_camera_snapshot=reg_addr[2:0]-3'd0;
 wire[4:0]probe_byte_camera_control_snapshot=reg_addr[4:0]-5'd0;
 wire[3:0]probe_byte_pixel_snapshot=reg_addr[3:0]-4'd0;
 wire[4:0]probe_byte_ae_snapshot=reg_addr[4:0]-5'd16;
 wire[2:0]probe_byte_front_snapshot=reg_addr[2:0]-3'd0;
 wire[4:0]probe_byte_peer_rx_snapshot=reg_addr[4:0]-5'd0;
 wire[4:0]probe_byte_link_pixel_snapshot=reg_addr[4:0]-5'd0;
 wire[4:0]probe_byte_link_tx_snapshot=reg_addr[4:0]-5'd0;
 wire[4:0]probe_byte_peer_memory_debug=reg_addr[4:0]-5'd0;
 wire[1:0]probe_byte_key_switch_count=reg_addr[1:0]-2'd0;
 wire[1:0]probe_byte_ui_key_count=reg_addr[1:0]-2'd0;
 wire[0:0]probe_byte_contest_snapshot_sequence=reg_addr[0:0]-1'd0;
 wire[1:0]probe_byte_adjust_key_count=reg_addr[1:0]-2'd0;
 wire[4:0]probe_byte_contest_snapshot=reg_addr[4:0]-5'd0;
 wire[1:0]probe_byte_noise_key_count=reg_addr[1:0]-2'd0;
 wire[0:0]probe_byte_peer_rx_sequence=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_link_pixel_sequence=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_link_tx_sequence=reg_addr[0:0]-1'd0;
 wire[1:0]probe_byte_phy_id=reg_addr[1:0]-2'd0;
 wire[0:0]probe_byte_phy_status=reg_addr[0:0]-1'd0;
 wire[0:0]probe_byte_phy_delay=reg_addr[0:0]-1'd0;
  // Mutually exclusive CSR addresses: parallel masked byte selection.
  wire [7:0] csr_value_0=28;
  wire [7:0] csr_value_1=BOARD_ID;
  wire [7:0] csr_value_2=status[7:0];
  wire [7:0] csr_value_3=REVISION;
  wire [7:0] csr_value_4=32'h36394d41>>((reg_addr-4)*8);
  wire [7:0] csr_value_5=8'h07;
  wire [7:0] csr_value_6={4'd0,rx_enable,boot_failed,boot_done,boot_active};
  wire [7:0] csr_value_7=motor_diagnostic[{probe_byte_motor_diagnostic,3'b000}+:8];
  wire [7:0] csr_value_8=pixel_frequency[{probe_byte_pixel_frequency,3'b000}+:8];
  wire [7:0] csr_value_9=rx_frequency[{probe_byte_rx_frequency,3'b000}+:8];
  wire [7:0] csr_value_10=camera_frequency[{probe_byte_camera_frequency,3'b000}+:8];
  wire [7:0] csr_value_11=byte_frequency[{probe_byte_byte_frequency,3'b000}+:8];
  wire [7:0] csr_value_12=memory_debug[{probe_byte_memory_debug,3'b000}+:8];
  wire [7:0] csr_value_13={6'd0,ae_target_mode};
  wire [7:0] csr_value_14={3'd0,tone_profile_busy,tone_profile_ack,tone_profile};
  wire [7:0] csr_value_15={7'd0,awb_enable};
  wire [7:0] csr_value_16=awb_host_snapshot[{probe_byte_awb_host_snapshot,3'b000}+:8];
  wire [7:0] csr_value_17={7'd0,ae_enable};
  wire [7:0] csr_value_18={2'd0,link_tx_busy,link_pixel_busy,peer_rx_busy,front_busy,pixel_busy,camera_busy};
  wire [7:0] csr_value_19=camera_sequence[{probe_byte_camera_sequence,3'b000}+:8];
  wire [7:0] csr_value_20=pixel_sequence[{probe_byte_pixel_sequence,3'b000}+:8];
  wire [7:0] csr_value_21=front_sequence[{probe_byte_front_sequence,3'b000}+:8];
  wire [7:0] csr_value_22=camera_snapshot[{probe_byte_camera_snapshot,3'b000}+:8];
  wire [7:0] csr_value_23=camera_control_snapshot[{probe_byte_camera_control_snapshot,3'b000}+:8];
  wire [7:0] csr_value_24=pixel_snapshot[{probe_byte_pixel_snapshot,3'b000}+:8];
  wire [7:0] csr_value_25=ae_snapshot[{probe_byte_ae_snapshot,3'b000}+:8];
  wire [7:0] csr_value_26=front_snapshot[{probe_byte_front_snapshot,3'b000}+:8];
  wire [7:0] csr_value_27=peer_rx_snapshot[{probe_byte_peer_rx_snapshot,3'b000}+:8];
  wire [7:0] csr_value_28=link_pixel_snapshot[{probe_byte_link_pixel_snapshot,3'b000}+:8];
  wire [7:0] csr_value_29=link_tx_snapshot[{probe_byte_link_tx_snapshot,3'b000}+:8];
  wire [7:0] csr_value_30=8'd0; // snapshot bytes bypass the old CSR capture FF
  wire [7:0] csr_value_31=key_switch_count[{probe_byte_key_switch_count,3'b000}+:8];
  wire [7:0] csr_value_32={5'd0,control_busy,control_ack,requested_remote};
  wire [7:0] csr_value_33={1'b0,ui_busy,ui_ack,ui_requested};
  wire [7:0] csr_value_34={3'd0,adjust_busy,adjust_ack,adjust_requested};
  wire [7:0] csr_value_35=8'h00;
  wire [7:0] csr_value_36=ui_key_count[{probe_byte_ui_key_count,3'b000}+:8];
  wire [7:0] csr_value_37=contest_snapshot_sequence[{probe_byte_contest_snapshot_sequence,3'b000}+:8];
  wire [7:0] csr_value_38=adjust_key_count[{probe_byte_adjust_key_count,3'b000}+:8];
  wire [7:0] csr_value_39=contest_snapshot[{probe_byte_contest_snapshot,3'b000}+:8];
  wire [7:0] csr_value_40={5'd0,noise_busy,noise_ack,noise_requested};
  wire [7:0] csr_value_41=noise_key_count[{probe_byte_noise_key_count,3'b000}+:8];
  wire [7:0] csr_value_42=peer_rx_sequence[{probe_byte_peer_rx_sequence,3'b000}+:8];
  wire [7:0] csr_value_43=link_pixel_sequence[{probe_byte_link_pixel_sequence,3'b000}+:8];
  wire [7:0] csr_value_44=link_tx_sequence[{probe_byte_link_tx_sequence,3'b000}+:8];
  wire [7:0] csr_value_45=phy_id[{probe_byte_phy_id,3'b000}+:8];
  wire [7:0] csr_value_46=phy_status[{probe_byte_phy_status,3'b000}+:8];
  wire [7:0] csr_value_47=phy_delay[{probe_byte_phy_delay,3'b000}+:8];
 // Page registers isolate CSR selection from UART control logic.
 wire[15:0]csr_address_00={8'h00,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_00;
 always @(posedge board50_global)if(!sys_rst_n)csr_bank_00<=0;else csr_bank_00<=
({8{csr_address_00==0}} & csr_value_0)  |
({8{csr_address_00==1||csr_address_00==16'h0b04}} & csr_value_1)  |
({8{csr_address_00==2}} & csr_value_2)  |
({8{csr_address_00==3}} & csr_value_3)  |
({8{csr_address_00>=4&&csr_address_00<8}} & csr_value_4)  |
({8{csr_address_00==8}} & csr_value_5)  |
({8{csr_address_00==9}} & csr_value_6)  |
({8{csr_address_00>=16'h0010&&csr_address_00<16'h0018}} & csr_value_7)  |
({8{csr_address_00>=16'h0020&&csr_address_00<16'h0024}} & csr_value_8)  |
({8{csr_address_00>=16'h0024&&csr_address_00<16'h0028}} & csr_value_9)  |
({8{csr_address_00>=16'h0028&&csr_address_00<16'h002c}} & csr_value_10)  |
({8{csr_address_00>=16'h002c&&csr_address_00<16'h0030}} & csr_value_11)  |
({8{csr_address_00>=16'h0040&&csr_address_00<16'h0070}} & csr_value_12)  |
({8{csr_address_00==16'h00e2}} & csr_value_13)  |
({8{csr_address_00==16'h00e3}} & csr_value_14)  |
({8{csr_address_00==16'h00e0}} & csr_value_17) ;
 wire[15:0]csr_address_01={8'h01,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_01;
 always @(posedge board50_global)if(!sys_rst_n)csr_bank_01<=0;else csr_bank_01<=
({8{csr_address_01==16'h0100}} & csr_value_18)  |
({8{csr_address_01>=16'h0102&&csr_address_01<16'h0104}} & csr_value_19)  |
({8{csr_address_01>=16'h0104&&csr_address_01<16'h0106}} & csr_value_20)  |
({8{csr_address_01>=16'h0106&&csr_address_01<16'h0108}} & csr_value_21)  |
({8{camera_capture_initialized&&(csr_address_01>=16'h0110&&csr_address_01<16'h0118)}} & csr_value_22)  |
({8{csr_address_01>=16'h0120&&csr_address_01<16'h0140}} & csr_value_23)  |
({8{pixel_capture_initialized&&(csr_address_01>=16'h0140&&csr_address_01<16'h0150)}} & csr_value_24)  |
({8{csr_address_01>=16'h0150&&csr_address_01<16'h0168}} & csr_value_25)  |
({8{front_capture_initialized&&(csr_address_01>=16'h0170&&csr_address_01<16'h0178)}} & csr_value_26)  |
({8{peer_capture_initialized&&(csr_address_01>=16'h01a0&&csr_address_01<16'h01c0)}} & csr_value_27)  |
({8{csr_address_01>=16'h01c0&&csr_address_01<16'h01d4}} & csr_value_28)  |
({8{csr_address_01>=16'h01e0&&csr_address_01<16'h01f4}} & csr_value_29)  |
({8{csr_address_01>=16'h0180&&csr_address_01<16'h0184}} & csr_value_45)  |
({8{csr_address_01>=16'h0184&&csr_address_01<16'h0186}} & csr_value_46)  |
({8{csr_address_01>=16'h0186&&csr_address_01<16'h0188}} & csr_value_47) ;
 wire[15:0]csr_address_02={8'h02,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_02;
 always @(posedge board50_global)if(!sys_rst_n)csr_bank_02<=0;else csr_bank_02<=
 // Snapshot term moved to one-cycle RAM byte output; no extra capture FF.
({8{csr_address_02>=16'h0240&&csr_address_02<16'h0244}} & csr_value_31)  |
({8{csr_address_02==16'h0244}} & csr_value_32)  |
({8{csr_address_02==16'h0290}} & csr_value_33)  |
({8{csr_address_02==16'h0291}} & csr_value_34)  |
({8{csr_address_02==16'h0292}} & csr_value_35)  |
({8{csr_address_02>=16'h0294&&csr_address_02<16'h0298}} & csr_value_36)  |
({8{csr_address_02>=16'h0298&&csr_address_02<16'h029a}} & csr_value_37)  |
({8{csr_address_02>=16'h029c&&csr_address_02<16'h02a0}} & csr_value_38)  |
({8{contest_capture_initialized&&(csr_address_02>=16'h02a0&&csr_address_02<16'h02ba)}} & csr_value_39)  |
({8{csr_address_02==16'h0280}} & csr_value_40)  |
({8{csr_address_02>=16'h0284&&csr_address_02<16'h0288}} & csr_value_41)  |
({8{csr_address_02>=16'h0248&&csr_address_02<16'h024a}} & csr_value_42)  |
({8{csr_address_02>=16'h024a&&csr_address_02<16'h024c}} & csr_value_43)  |
({8{csr_address_02>=16'h024c&&csr_address_02<16'h024e}} & csr_value_44) ;
 wire[15:0]csr_address_03={8'h03,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_03;
 always @(posedge board50_global)if(!sys_rst_n)csr_bank_03<=0;else csr_bank_03<=
({8{csr_address_03==16'h0300}} & csr_value_15)  |
({8{csr_address_03>=16'h0310&&csr_address_03<16'h0328}} & csr_value_16)  |
({8{cloud_capture_initialized&&(csr_address_03>=16'h03a8&&csr_address_03<16'h03b8)}} & cloud_host_snapshot[{csr_address_03[3:0]-4'd8,3'b000}+:8])  |
({8{cloud_capture_initialized&&(csr_address_03>=16'h03c0&&csr_address_03<16'h03c8)}} & cloud_host_snapshot[128+{csr_address_03[2:0],3'b000}+:8]);
 wire[15:0]csr_address_0b={8'h0b,reg_addr[7:0]};
 (* keep=1,preserve=1,allow_forward_retiming=0,allow_backward_retiming=0 *)reg[7:0]csr_bank_0b;
 always @(posedge board50_global)if(!sys_rst_n)csr_bank_0b<=0;else csr_bank_0b<=
({8{csr_address_0b==1||csr_address_0b==16'h0b04}} & csr_value_1) ;
 reg[7:0]csr_bank_0f;
 always @(posedge board50_global)if(!sys_rst_n)csr_bank_0f<=0;
 else csr_bank_0f<=reg_addr[7:5]==0?hmi_diagnostics[{reg_addr[4:0],3'b000}+:8]:8'd0;
 always @*begin
  case(reg_addr[15:8])
   8'h00:probe_data=csr_bank_00|({8{memory_snapshot_local_selected}}&memory_snapshot_byte);
   8'h01:probe_data=csr_bank_01;
   8'h02:probe_data=csr_bank_02|({8{memory_snapshot_peer_selected}}&memory_snapshot_byte);
   8'h03:probe_data=csr_bank_03;
   8'h0b:probe_data=csr_bank_0b;
   8'h0f:probe_data=csr_bank_0f;
   default:probe_data=0;
  endcase
 end
 // Read-only acceptance telemetry: no extra image/framebuffer storage.
 // AM94: cloud_capture is absent; its diagnostics advertise no 3D/peer publication.
 assign cloud_capture_initialized=0;
 assign cloud_host_busy=0;
 assign cloud_host_snapshot=0;
 tjc_hmi_control hmi(.clk(board50_global),.rst_n(sys_rst_n),.rx(hmi_rx),.tx(hmi_tx),
 .mode_applied(ui_ack),.mode_busy(ui_busy),.adjust_applied(adjust_ack),.adjust_busy(adjust_busy),
 .noise_applied(noise_ack),.noise_requested(noise_requested),.noise_busy(noise_busy),
 .ae_enabled(ae_enable),.awb_enabled(awb_enable),.debug_write(probe_write),
 .control_write(hmi_control_write),.control_address(hmi_control_address),.control_data(hmi_control_data),.diagnostics(hmi_diagnostics));
 wire [7:0] uart_rx_data,uart_tx_data;wire uart_rx_valid,uart_tx_valid,uart_tx_ready,uart_error;
 uart_byte #(.DIV(50)) uart(.clk(board50_global),.rst_n(sys_rst_n),.rx(uart_rx),.tx(uart_tx),
 .tx_data(uart_tx_data),.tx_valid(uart_tx_valid),.tx_ready(uart_tx_ready),.rx_data(uart_rx_data),.rx_valid(uart_rx_valid),.framing_error(uart_error));
 debug_register_bridge bridge(.clk(board50_global),.rst_n(sys_rst_n),.busy(bridge_busy),
 .rx_data(uart_rx_data),.rx_valid(uart_rx_valid),.tx_data(uart_tx_data),.tx_valid(uart_tx_valid),.tx_ready(uart_tx_ready),
 .status(status),.probe_data(probe_data),.probe_write(probe_write),.cam_valid(cam_valid),.ddc_valid(ddc_valid),.read(cam_read),
 .dev_addr(dev_addr),.reg_addr(reg_addr),.write_data(write_data),.cam_done(cam_done),.cam_error(cam_error),.cam_data(cam_data),
 .ddc_done(ddc_valid),.ddc_error(2'd1),.ddc_data(8'b0));
endmodule
