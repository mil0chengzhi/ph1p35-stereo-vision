`timescale 1ns/1ps
// 功能：新生成MIS DDRMC IP的存储自检及三帧视频包装；内置DDR虚拟端口由IP专用约束定位。
// clk_50为参考/调试域，rst_n低有效；DDR输出dhi_clk由IP生成，禁止用普通逻辑分频。
// debug[511:384]冻结发布/拒绝帧数、显示帧ID和flags(front_valid/fault)；debug_latch为50MHz单拍。
// debug[383:0]为控制域结果：低32位flags，随后errors、low_first_address、expected128、observed128、频率计数。
// flags bit0=ready、1=low_done、2=low_failed；多位结果只在done同步完成后锁存，此后自检停止写入。
// DDR16位数据、2位DQS/DM、13位地址、2位bank；均是SiP内部专用信号，不要求用户接线。
module ph1p_video_memory #(parameter WIDTH=1280,HEIGHT=720,REMOTE_WIDTH=640,REMOTE_HEIGHT=480,STEREO_ENABLED=0,STEREO_AUTOMATIC=0,METRIC_EXPORT_ENABLED=0)(
 input wire clk_50,input wire rst_n,input wire debug_latch,output reg [639:0] debug,
 output wire memory_clk,output wire memory_rst_n,output wire memory_verified,
 input wire[15:0]snapshot_address,input wire snapshot_write,input wire[7:0]snapshot_write_data,
 output wire[7:0]snapshot_read_data,output wire snapshot_selected,
 // All stereo ports are in memory_clk/DHI domain. Disabled by default;
 // no asynchronous status/control may be wired directly here.
 input wire stereo_run_enable,metric_export_pause,output wire metric_export_quiescent,output wire[127:0]metric_job_sources,
 input wire depth_line_req_valid,depth_new_frame,input wire[15:0]depth_source_row,input wire depth_request_bank,
 output wire depth_line_req_ready,depth_line_write,depth_line_done,depth_line_good,
 output wire[127:0]depth_line_data,output wire[7:0]depth_line_address,output wire depth_line_bank,
 output wire[31:0]depth_line_frame_id,depth_line_session,depth_line_start_ms,
 output wire[31:0]stereo_jobs_started,stereo_jobs_completed,stereo_jobs_skipped,depth_result_published,
 output wire stereo_system_fault,stereo_configuration_error,
 input wire stereo_quiesce,stereo_rearm,input wire[31:0]stereo_epoch,
 input wire stereo_pipeline_idle,stereo_reader_idle,
 input wire[1:0]stereo_acquire,stereo_release,
 input wire[95:0]stereo_release_cookies,
 output wire[1:0]stereo_initialized,stereo_rearm_ack,stereo_rearm_rejected,
 output wire[1:0]stereo_acquire_ready,stereo_acquire_ack,stereo_lease_valid,
 output wire[3:0]stereo_lease_slots,output wire[191:0]stereo_lease_meta,
 output wire[95:0]stereo_lease_cookies,
 output wire[1:0]stereo_release_ack,stereo_release_rejected,
 input wire[24:0]stereo_araddr,input wire stereo_arvalid,stereo_rready,stereo_read_active,
 output wire stereo_arready,stereo_rvalid,stereo_rlast,output wire[127:0]stereo_rdata,
 input wire token_empty,output wire token_read,input wire token_valid,input wire [129:0] token_data,
 input wire line_req_valid,output wire line_req_ready,input wire new_frame,input wire [15:0] source_row,input wire request_bank,
 output wire line_write,output wire [127:0] line_data,output wire [7:0] line_address,output wire line_bank,
 output wire line_done,output wire line_good,output wire [31:0] line_frame_id,output wire [31:0] line_session,
 input wire [31:0] time_gray,output wire [31:0] line_start_ms,
 input wire remote_token_empty,output wire remote_token_read,input wire remote_token_valid,input wire [129:0] remote_token_data,
 input wire remote_line_req_valid,output wire remote_line_req_ready,input wire remote_new_frame,input wire [15:0] remote_source_row,input wire remote_request_bank,
 output wire remote_line_write,output wire [127:0] remote_line_data,output wire [7:0] remote_line_address,output wire remote_line_bank,
 output wire remote_line_done,remote_line_good,output wire [31:0] remote_line_frame_id,remote_line_session,
 output wire [31:0] remote_line_start_ms,output reg [255:0] remote_debug,
 output wire ddr_ck_p,output wire ddr_ck_n,output wire ddr_cke,output wire ddr_odt,
 output wire ddr_cs_n,output wire ddr_ras_n,output wire ddr_cas_n,output wire ddr_we_n,
 output wire [1:0] ddr_ba,output wire [12:0] ddr_addr,
 inout wire [1:0] ddr_dqs_p,inout wire [1:0] ddr_dqs_n,inout wire [15:0] ddr_dq,inout wire [1:0] ddr_dm
);
 wire dhi_clk,phy_reset,phy_ready;
 wire[24:0]mc_awaddr,mc_araddr;wire[127:0]mc_wdata,mc_rdata;wire[15:0]mc_wstrb;
 wire mc_awvalid,mc_awready,mc_wvalid,mc_wready,mc_wlast,mc_arvalid,mc_arready,mc_rvalid,mc_rlast,mc_rready;
 wire main_read_epoch,auto_pipeline_idle;wire[1:0]auto_acquire,auto_release;wire[95:0]auto_release_cookies;
 wire[24:0]auto_araddr;wire auto_arvalid,auto_rready,auto_read_active;
 wire inner_arb_fault;
 wire [24:0] awaddr,araddr,low_first_address;
 wire [127:0] wdata,rdata,low_expected,low_observed;
 wire [15:0] wstrb;
 wire awvalid,awready,wvalid,wready,wlast,arvalid,arready,rvalid,rlast,rready,low_done,low_failed;
 wire [31:0] low_errors,frequency;
 wire [3:0] phase;
 video_ddr memory(
  .ref_clk(clk_50),.sys_rst_n(rst_n),.apb_pclk(clk_50),.apb_prst_n(rst_n),
  .apb_psel(1'b0),.apb_penable(1'b0),.apb_pwrite(1'b0),.apb_paddr(16'b0),.apb_pwdata(32'b0),
  .apb_pready(),.apb_prdata(),.dhi_clk(dhi_clk),.cfg_clk(),.cfg_rst_n(),
  .ddrphy_rst_n(phy_reset),.ddrphy_rdy(phy_ready),
  .paxi_awaddr(mc_awaddr),.paxi_awvalid(mc_awvalid),.paxi_awready(mc_awready),
  .paxi_wdata(mc_wdata),.paxi_wstrb(mc_wstrb),.paxi_wvalid(mc_wvalid),.paxi_wlast(mc_wlast),.paxi_wready(mc_wready),
  .paxi_araddr(mc_araddr),.paxi_arvalid(mc_arvalid),.paxi_arready(mc_arready),
  .paxi_rdata(mc_rdata),.paxi_rlast(mc_rlast),.paxi_rvalid(mc_rvalid),.paxi_rready(mc_rready),
  .ddr_ck_p(ddr_ck_p),.ddr_ck_n(ddr_ck_n),.ddr_cke(ddr_cke),.ddr_odt(ddr_odt),
  .ddr_cs_n(ddr_cs_n),.ddr_ras_n(ddr_ras_n),.ddr_cas_n(ddr_cas_n),.ddr_we_n(ddr_we_n),
  .ddr_ba(ddr_ba),.ddr_addr(ddr_addr),.ddr_dqs_p(ddr_dqs_p),.ddr_dqs_n(ddr_dqs_n),.ddr_dq(ddr_dq),.ddr_dm(ddr_dm));
 reg [1:0] reset_sync,ready_sync;
 wire async_reset=!rst_n || !phy_reset;
 always @(posedge dhi_clk or posedge async_reset) begin
  if(async_reset) begin reset_sync<=0;ready_sync<=0;end
  else begin reset_sync<={reset_sync[0],1'b1};ready_sync<={ready_sync[0],phy_ready};end
 end

  wire [24:0] t_awaddr,t_araddr,v_awaddr,v_araddr;
 wire [127:0] t_wdata,v_wdata;
 wire [15:0] t_wstrb,v_wstrb;
 wire t_awvalid,t_awready,t_wvalid,t_wlast,t_wready,t_arvalid,t_arready,t_rready;
 wire v_awvalid,v_wvalid,v_wlast,v_arvalid,v_rready;

 wire alias_done,alias_failed;wire[31:0]alias_errors;wire[24:0]alias_first_address;
 wire[127:0]alias_expected,alias_observed;wire[3:0]alias_phase;
 wire[24:0]a_awaddr,a_araddr;wire[127:0]a_wdata;wire[15:0]a_wstrb;
 wire a_awvalid,a_awready,a_wvalid,a_wready,a_wlast,a_arvalid,a_arready,a_rready;
 wire alias_selected=low_done&&!low_failed;
 wire done=low_done&&(low_failed||alias_done),failed=low_failed||alias_failed;
 wire[31:0]errors=low_failed?low_errors:alias_errors;
 wire[24:0]first_address=low_failed?low_first_address:alias_first_address;
 wire[127:0]expected=low_failed?low_expected:alias_expected,observed=low_failed?low_observed:alias_observed;
 wire video_active=done&&!failed;
 assign memory_verified=video_active&&ready_sync[1];
 wire[24:0]aux_awaddr,aux_araddr;wire[127:0]aux_wdata;wire[15:0]aux_wstrb;
 wire aux_awvalid,aux_wvalid,aux_wlast,aux_arvalid,aux_rready,aux_read_active,aux_fault,aux_config,arb_config;
 assign snapshot_read_data=8'd0;assign snapshot_selected=1'b0;
 wire arb_fault=inner_arb_fault||stereo_system_fault;wire [31:0] arb_debug;
 wire [2:0] clients_awready,clients_wready,clients_arready,clients_rvalid,clients_rlast,clients_read_active;
 assign clients_read_active[2]=aux_read_active;
 wire [24:0] remote_awaddr,remote_araddr;
 wire [127:0] remote_wdata;wire [15:0] remote_wstrb;
 wire remote_awvalid,remote_wvalid,remote_wlast,remote_arvalid,remote_rready;
 wire [24:0] bus_awaddr,bus_araddr;wire [127:0] bus_wdata;wire [15:0] bus_wstrb;
 wire bus_awvalid,bus_wvalid,bus_wlast,bus_arvalid,bus_rready;
 wire [383:0] clients_rdata;
 wire [127:0] remote_frame_debug;wire remote_front_valid,remote_fault;
 wire [31:0] remote_front_id,remote_published,remote_rejected;
 wire [31:0] remote_front_session;
 assign remote_line_frame_id=remote_front_id;
 assign remote_line_session=remote_front_session;
 // Eight non-overlapping slots, 2 Mi 16-bit words per slot, total32MiB.
 // Local canonical B slots0..3, remote A slots4..7. Boot probe retained.
 // Nested mode retains the complete physical drain in shared_physical_MC.
 hd_ddr_arbiter #(.CLIENTS(3),.WRITE_DRAIN((STEREO_ENABLED&&STEREO_AUTOMATIC)?1:128),.PRIORITY_MASK(3'b000)) arbitration(.clk(dhi_clk),.rst_n(reset_sync[1]),.enable(video_active&&ready_sync[1]),
  .c_awaddr({aux_awaddr,remote_awaddr+25'd8388608,v_awaddr}),.c_araddr({aux_araddr,remote_araddr+25'd8388608,v_araddr}),
  .c_awvalid({aux_awvalid,remote_awvalid,v_awvalid}),.c_wvalid({aux_wvalid,remote_wvalid,v_wvalid}),.c_wlast({aux_wlast,remote_wlast,v_wlast}),
  .c_arvalid({aux_arvalid,remote_arvalid,v_arvalid}),.c_rready({aux_rready,remote_rready,v_rready}),.c_read_active(clients_read_active),
  .c_wdata({aux_wdata,remote_wdata,v_wdata}),.c_wstrb({aux_wstrb,remote_wstrb,v_wstrb}),
  .c_awready(clients_awready),.c_wready(clients_wready),.c_arready(clients_arready),
  .c_rvalid(clients_rvalid),.c_rlast(clients_rlast),.c_rdata(clients_rdata),
  .awaddr(bus_awaddr),.araddr(bus_araddr),.wdata(bus_wdata),.wstrb(bus_wstrb),
  .awvalid(bus_awvalid),.wvalid(bus_wvalid),.wlast(bus_wlast),.arvalid(bus_arvalid),.rready(bus_rready),
  .awready(awready),.wready(wready),.arready(arready),.rdata(rdata),.rvalid(rvalid&&video_active),.rlast(rlast),
  .fault(inner_arb_fault),.configuration_error(arb_config),.debug(arb_debug),.transaction_read_active(main_read_epoch));
 
 
 
 
 
 wire [127:0] frame_debug;wire front_valid,fault;wire [31:0] front_id,published_frames,rejected_frames;
 // No native snapshot copying. Read-only leased stereo client2.
 assign aux_awaddr=0;assign aux_araddr=STEREO_ENABLED?(STEREO_AUTOMATIC?auto_araddr:stereo_araddr):25'd0;assign aux_wdata=0;assign aux_wstrb=0;
 assign aux_awvalid=0;assign aux_wvalid=0;assign aux_wlast=0;assign aux_arvalid=STEREO_ENABLED?(STEREO_AUTOMATIC?auto_arvalid:stereo_arvalid):1'b0;
 assign aux_rready=STEREO_ENABLED?(STEREO_AUTOMATIC?auto_rready:stereo_rready):1'b0;assign aux_read_active=STEREO_ENABLED?(STEREO_AUTOMATIC?auto_read_active:stereo_read_active):1'b0;assign aux_fault=0;assign aux_config=0;
 assign stereo_arready=STEREO_ENABLED&&clients_arready[2];
 assign stereo_rvalid=STEREO_ENABLED&&clients_rvalid[2];
 assign stereo_rlast=STEREO_ENABLED&&clients_rlast[2];assign stereo_rdata=clients_rdata[383:256];
 wire[1:0]store_config;
 wire lease_pipeline_idle=!STEREO_ENABLED||(STEREO_AUTOMATIC?auto_pipeline_idle:stereo_pipeline_idle);
 wire lease_reader_idle=!STEREO_ENABLED||(STEREO_AUTOMATIC?auto_pipeline_idle:stereo_reader_idle);
 wire lease_quiesce=STEREO_ENABLED&&!STEREO_AUTOMATIC&&stereo_quiesce;
 wire lease_rearm=STEREO_ENABLED&&!STEREO_AUTOMATIC&&stereo_rearm;
 wire[31:0]lease_epoch=STEREO_ENABLED?stereo_epoch:32'd1;
 // Bootstrap only after the two physical-memory boot checks. Whole memory,
 // FIFOs and consumers must share the verified global reset; local reset alone
 // cannot authorize cookie replay. Later rearm requires both returned ACKs.
 wire local_rearm=ready_sync[1]&&video_active&&(!stereo_initialized[0]||lease_rearm);
 wire remote_rearm=ready_sync[1]&&video_active&&(!stereo_initialized[1]||lease_rearm);
 // 与line_done一同交给行缓存邮箱；front_id在一次完整行响应内保持不变。
 assign line_frame_id=front_id;
 assign memory_clk=dhi_clk;assign memory_rst_n=reset_sync[1];
 assign awaddr=video_active?bus_awaddr:(alias_selected?a_awaddr:t_awaddr);assign awvalid=video_active?bus_awvalid:(alias_selected?a_awvalid:t_awvalid);
 assign wdata=video_active?bus_wdata:(alias_selected?a_wdata:t_wdata);assign wstrb=video_active?bus_wstrb:(alias_selected?a_wstrb:t_wstrb);
 assign wvalid=video_active?bus_wvalid:(alias_selected?a_wvalid:t_wvalid);assign wlast=video_active?bus_wlast:(alias_selected?a_wlast:t_wlast);
 assign araddr=video_active?bus_araddr:(alias_selected?a_araddr:t_araddr);assign arvalid=video_active?bus_arvalid:(alias_selected?a_arvalid:t_arvalid);
 assign rready=video_active?bus_rready:(alias_selected?a_rready:t_rready);
 assign t_awready=awready && !video_active&&!alias_selected;
 assign a_awready=awready&&!video_active&&alias_selected;assign t_wready=wready && !video_active&&!alias_selected;
 assign a_wready=wready&&!video_active&&alias_selected;assign t_arready=arready && !video_active&&!alias_selected;
 assign a_arready=arready&&!video_active&&alias_selected;
 leased_ddr_frame_store #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.CAMERA_ID(2),.LINE_READ_GUARD((STEREO_ENABLED&&STEREO_AUTOMATIC)?0:128)) frames(.clk(dhi_clk),.rst_n(reset_sync[1]),.memory_ready(ready_sync[1] && video_active && !arb_fault),
  .time_gray(time_gray),.front_start_ms(line_start_ms),
  .owner_rearm(local_rearm),.owner_quiesce(!stereo_initialized[0]||lease_quiesce),
  .owner_epoch(!stereo_initialized[0]?32'd1:lease_epoch),
  .external_pipeline_idle(lease_pipeline_idle),.lease_reader_idle(lease_reader_idle),
  .owner_initialized(stereo_initialized[0]),.owner_rearm_ack(stereo_rearm_ack[0]),.owner_rearm_rejected(stereo_rearm_rejected[0]),
  .lease_acquire(STEREO_ENABLED&&(STEREO_AUTOMATIC?auto_acquire[0]:stereo_acquire[0])),.lease_acquire_ready(stereo_acquire_ready[0]),.lease_acquire_ack(stereo_acquire_ack[0]),
  .lease_valid(stereo_lease_valid[0]),.lease_slot(stereo_lease_slots[0+:2]),.lease_meta(stereo_lease_meta[0+:96]),.lease_cookie(stereo_lease_cookies[0+:48]),
  .lease_release(STEREO_ENABLED&&(STEREO_AUTOMATIC?auto_release[0]:stereo_release[0])),.release_camera(8'd2),.release_cookie(STEREO_AUTOMATIC?auto_release_cookies[0+:48]:stereo_release_cookies[0+:48]),
  .lease_release_ack(stereo_release_ack[0]),.lease_release_rejected(stereo_release_rejected[0]),.configuration_error(store_config[0]),
  .token_empty(token_empty),.token_read(token_read),.token_valid(token_valid),.token_data(token_data),
  .line_req_valid(line_req_valid),.line_req_ready(line_req_ready),.new_frame(new_frame),.source_row(source_row),.request_bank(request_bank),
  .line_write(line_write),.line_data(line_data),.line_address(line_address),.line_bank(line_bank),.line_done(line_done),.line_good(line_good),
  .debug_state(frame_debug),.front_valid(front_valid),.front_id(front_id),.front_session(line_session),.fault(fault),.published_frames(published_frames),.rejected_frames(rejected_frames),
  .awaddr(v_awaddr),.awvalid(v_awvalid),.awready(clients_awready[0]),.wdata(v_wdata),.wstrb(v_wstrb),
  .wvalid(v_wvalid),.wlast(v_wlast),.wready(clients_wready[0]),.araddr(v_araddr),.arvalid(v_arvalid),.arready(clients_arready[0]),
  .rdata(clients_rdata[127:0]),.rvalid(clients_rvalid[0]),.rlast(clients_rlast[0]),.rready(v_rready),.read_active(clients_read_active[0]));
 leased_ddr_frame_store #(.WIDTH(REMOTE_WIDTH),.HEIGHT(REMOTE_HEIGHT),.CAMERA_ID(1),.LINE_READ_GUARD((STEREO_ENABLED&&STEREO_AUTOMATIC)?0:128)) remote_frames(.clk(dhi_clk),.rst_n(reset_sync[1]),.memory_ready(ready_sync[1]&&video_active&&!arb_fault),
  .time_gray(time_gray),.front_start_ms(remote_line_start_ms),
  .owner_rearm(remote_rearm),.owner_quiesce(!stereo_initialized[1]||lease_quiesce),
  .owner_epoch(!stereo_initialized[1]?32'd1:lease_epoch),
  .external_pipeline_idle(lease_pipeline_idle),.lease_reader_idle(lease_reader_idle),
  .owner_initialized(stereo_initialized[1]),.owner_rearm_ack(stereo_rearm_ack[1]),.owner_rearm_rejected(stereo_rearm_rejected[1]),
  .lease_acquire(STEREO_ENABLED&&(STEREO_AUTOMATIC?auto_acquire[1]:stereo_acquire[1])),.lease_acquire_ready(stereo_acquire_ready[1]),.lease_acquire_ack(stereo_acquire_ack[1]),
  .lease_valid(stereo_lease_valid[1]),.lease_slot(stereo_lease_slots[2+:2]),.lease_meta(stereo_lease_meta[96+:96]),.lease_cookie(stereo_lease_cookies[48+:48]),
  .lease_release(STEREO_ENABLED&&(STEREO_AUTOMATIC?auto_release[1]:stereo_release[1])),.release_camera(8'd1),.release_cookie(STEREO_AUTOMATIC?auto_release_cookies[48+:48]:stereo_release_cookies[48+:48]),
  .lease_release_ack(stereo_release_ack[1]),.lease_release_rejected(stereo_release_rejected[1]),.configuration_error(store_config[1]),
  .token_empty(remote_token_empty),.token_read(remote_token_read),.token_valid(remote_token_valid),.token_data(remote_token_data),
  .line_req_valid(remote_line_req_valid),.line_req_ready(remote_line_req_ready),.new_frame(remote_new_frame),.source_row(remote_source_row),.request_bank(remote_request_bank),
  .line_write(remote_line_write),.line_data(remote_line_data),.line_address(remote_line_address),.line_bank(remote_line_bank),.line_done(remote_line_done),.line_good(remote_line_good),
  .debug_state(remote_frame_debug),.front_valid(remote_front_valid),.front_id(remote_front_id),.front_session(remote_front_session),.fault(remote_fault),.published_frames(remote_published),.rejected_frames(remote_rejected),
  .awaddr(remote_awaddr),.awvalid(remote_awvalid),.awready(clients_awready[1]),.wdata(remote_wdata),.wstrb(remote_wstrb),
  .wvalid(remote_wvalid),.wlast(remote_wlast),.wready(clients_wready[1]),.araddr(remote_araddr),.arvalid(remote_arvalid),.arready(clients_arready[1]),
  .rdata(clients_rdata[255:128]),.rvalid(clients_rvalid[1]),.rlast(clients_rlast[1]),.rready(remote_rready),.read_active(clients_read_active[1]));

 generate if(STEREO_ENABLED&&STEREO_AUTOMATIC)begin:depth_enabled
  stereo_depth_memory #(.CALIBRATION_READY(1),.MAP_ID(32'h4f87fe40),
   .NODE_FILE("hd720_20261028/calibration_v1/nodes.mem"),.MASK_FILE("hd720_20261028/calibration_v1/row_runs.mem"),.LUT_FILE("hd720_20261028/calibration_v1/depth720_Z16.mem"))depth(
   .clk(dhi_clk),.rst_n(reset_sync[1]),.memory_ready(ready_sync[1]),.video_active(video_active),.run_enable(stereo_run_enable&&(!METRIC_EXPORT_ENABLED||!metric_export_pause)),.export_quiescent(metric_export_quiescent),.job_sources(metric_job_sources),.time_gray(time_gray),
   .main_awaddr(awaddr),.main_awvalid(awvalid),.main_awready(awready),.main_wdata(wdata),.main_wstrb(wstrb),.main_wvalid(wvalid),.main_wlast(wlast),.main_wready(wready),
   .main_araddr(araddr),.main_arvalid(arvalid),.main_arready(arready),.main_rdata(rdata),.main_rvalid(rvalid),.main_rlast(rlast),.main_rready(rready),.main_read_epoch(main_read_epoch&&video_active),
   .awaddr(mc_awaddr),.awvalid(mc_awvalid),.awready(mc_awready),.wdata(mc_wdata),.wstrb(mc_wstrb),.wvalid(mc_wvalid),.wlast(mc_wlast),.wready(mc_wready),
   .araddr(mc_araddr),.arvalid(mc_arvalid),.arready(mc_arready),.rdata(mc_rdata),.rvalid(mc_rvalid),.rlast(mc_rlast),.rready(mc_rready),
   .camera_initialized(stereo_initialized),.camera_acquire_ready(stereo_acquire_ready),.camera_lease_valid(stereo_lease_valid),.camera_lease_slots(stereo_lease_slots),
   .camera_lease_cookies(stereo_lease_cookies),.camera_lease_meta(stereo_lease_meta),.camera_published_counts({remote_published,published_frames}),
   .camera_acquire(auto_acquire),.camera_release(auto_release),.camera_release_cookies(auto_release_cookies),.camera_pipeline_idle(auto_pipeline_idle),
   .feature_araddr(auto_araddr),.feature_arvalid(auto_arvalid),.feature_arready(clients_arready[2]),.feature_rdata(clients_rdata[383:256]),
   .feature_rvalid(clients_rvalid[2]),.feature_rlast(clients_rlast[2]),.feature_rready(auto_rready),.feature_read_active(auto_read_active),
   .line_req_valid(depth_line_req_valid),.line_req_ready(depth_line_req_ready),.new_frame(depth_new_frame),.source_row(depth_source_row),.request_bank(depth_request_bank),
   .line_write(depth_line_write),.line_data(depth_line_data),.line_address(depth_line_address),.line_bank(depth_line_bank),.line_done(depth_line_done),.line_good(depth_line_good),
   .line_frame_id(depth_line_frame_id),.line_session(depth_line_session),.line_start_ms(depth_line_start_ms),
   .jobs_started(stereo_jobs_started),.jobs_completed(stereo_jobs_completed),.jobs_skipped(stereo_jobs_skipped),.result_published(depth_result_published),
   .fault(stereo_system_fault),.configuration_error(stereo_configuration_error));
 end else begin:depth_disabled
  assign mc_awaddr=awaddr;assign mc_awvalid=awvalid;assign awready=mc_awready;
  assign mc_wdata=wdata;assign mc_wstrb=wstrb;assign mc_wvalid=wvalid;assign mc_wlast=wlast;assign wready=mc_wready;
  assign mc_araddr=araddr;assign mc_arvalid=arvalid;assign arready=mc_arready;
  assign rdata=mc_rdata;assign rvalid=mc_rvalid;assign rlast=mc_rlast;assign mc_rready=rready;
  assign metric_export_quiescent=0;assign metric_job_sources=0;
  assign stereo_jobs_started=0;assign stereo_jobs_completed=0;assign stereo_jobs_skipped=0;assign depth_result_published=0;
  assign stereo_system_fault=0;assign stereo_configuration_error=0;assign auto_pipeline_idle=1;
  assign auto_acquire=0;assign auto_release=0;assign auto_release_cookies=0;assign auto_araddr=0;assign auto_arvalid=0;assign auto_rready=0;assign auto_read_active=0;
  assign depth_line_req_ready=0;assign depth_line_write=0;assign depth_line_data=0;assign depth_line_address=0;assign depth_line_bank=0;
  assign depth_line_done=0;assign depth_line_good=0;assign depth_line_frame_id=0;assign depth_line_session=0;assign depth_line_start_ms=0;
 end endgenerate
 // 控制域请求翻转，DHI域冻结计数后应答；应答跨两拍后读取稳定多位数据。
 reg debug_request,debug_ack,debug_seen;reg [1:0] debug_req_sync,debug_ack_sync;
 reg [255:0] video_snapshot,remote_video_snapshot;reg [1:0] arb_fault_sync;
 always @(posedge dhi_clk or negedge rst_n) begin
  if(!rst_n) begin debug_req_sync<=0;debug_ack<=0;video_snapshot<=0;remote_video_snapshot<=0;end
  else begin
   debug_req_sync<={debug_req_sync[0],debug_request};
   if(debug_req_sync[1]!=debug_ack) begin
    video_snapshot<={frame_debug,30'b0,(fault||arb_fault),front_valid,front_id,rejected_frames,published_frames};
    remote_video_snapshot<={remote_frame_debug,30'b0,(remote_fault||arb_fault),remote_front_valid,remote_front_id,remote_rejected,remote_published};debug_ack<=debug_req_sync[1];
   end
  end
 end
 paxi_memory_test test(.clk(dhi_clk),.rst_n(reset_sync[1]),.ready(ready_sync[1]),
  .awaddr(t_awaddr),.awvalid(t_awvalid),.awready(t_awready),.wdata(t_wdata),.wstrb(t_wstrb),.wvalid(t_wvalid),.wlast(t_wlast),.wready(t_wready),
  .araddr(t_araddr),.arvalid(t_arvalid),.arready(t_arready),.rdata(rdata),.rvalid(rvalid && !video_active&&!alias_selected),.rlast(rlast),.rready(t_rready),
  .done(low_done),.failed(low_failed),.errors(low_errors),.first_address(low_first_address),.expected(low_expected),.observed(low_observed),.phase(phase));
 paxi_snapshot_address_probe address_probe(.clk(dhi_clk),.rst_n(reset_sync[1]&&low_done&&!low_failed),.ready(ready_sync[1]),
  .awaddr(a_awaddr),.awvalid(a_awvalid),.awready(a_awready),.wdata(a_wdata),.wstrb(a_wstrb),.wvalid(a_wvalid),.wlast(a_wlast),.wready(a_wready),
  .araddr(a_araddr),.arvalid(a_arvalid),.arready(a_arready),.rdata(rdata),.rvalid(rvalid&&!video_active&&alias_selected),.rlast(rlast),.rready(a_rready),
  .done(alias_done),.failed(alias_failed),.errors(alias_errors),.first_address(alias_first_address),.expected(alias_expected),.observed(alias_observed),.phase(alias_phase));
 clock_frequency_monitor meter(.reference_clk(clk_50),.source_clk(dhi_clk),.rst_n(rst_n),.edges(frequency),.fresh());
 reg [1:0] done_sync,phy_sync;
 always @(posedge clk_50) begin
  if(!rst_n) begin debug<=0;remote_debug<=0;arb_fault_sync<=0;done_sync<=0;phy_sync<=0;debug_request<=0;debug_ack_sync<=0;debug_seen<=0;end
  else begin
   debug_ack_sync<={debug_ack_sync[0],debug_ack};arb_fault_sync<={arb_fault_sync[0],arb_fault};
   if(debug_latch && debug_request==debug_ack_sync[1]) debug_request<=!debug_request;
   if(debug_ack_sync[1]!=debug_seen) begin debug_seen<=debug_ack_sync[1];debug[639:384]<=0;remote_debug<=0;end
   done_sync<={done_sync[0],done};phy_sync<={phy_sync[0],phy_ready};
   debug[31:0]<={29'b0,(done_sync[1]&&(failed||arb_fault_sync[1])),done_sync[1],phy_sync[1]};
   debug[383:352]<=frequency;
   if(done_sync[1]) begin debug[63:32]<=errors;debug[95:64]<={7'b0,first_address};
    debug[223:96]<=expected;debug[351:224]<=observed;end
  end
 end
endmodule
