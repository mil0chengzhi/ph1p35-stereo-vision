`timescale 1ns/1ps
// 功能：新生成MIS DDRMC IP的存储自检及三帧视频包装；内置DDR虚拟端口由IP专用约束定位。
// clk_50为参考/调试域，rst_n低有效；DDR输出dhi_clk由IP生成，禁止用普通逻辑分频。
// debug[511:384]冻结发布/拒绝帧数、显示帧ID和flags(front_valid/fault)；debug_latch为50MHz单拍。
// debug[383:0]为控制域结果：低32位flags，随后errors、first_address、expected128、observed128、频率计数。
// flags bit0=ready、1=done、2=failed；多位结果只在done同步完成后锁存，此后自检停止写入。
// DDR16位数据、2位DQS/DM、13位地址、2位bank；均是SiP内部专用信号，不要求用户接线。
module ph1p_video_memory #(parameter WIDTH=1280,HEIGHT=720,REMOTE_WIDTH=640,REMOTE_HEIGHT=480)(
 input wire clk_50,input wire rst_n,input wire debug_latch,output reg [639:0] debug,
 input wire debug_read_peer,input wire[4:0]debug_read_index,output wire[7:0]debug_read_byte,output wire debug_snapshot_led,
 output wire memory_clk,output wire memory_rst_n,output wire memory_verified,
 input wire token_empty,output wire token_read,input wire token_valid,input wire [129:0] token_data,
 input wire line_req_valid,output wire line_req_ready,input wire new_frame,input wire [15:0] source_row,input wire request_bank,
 output wire line_write,output wire [127:0] line_data,output wire [7:0] line_address,output wire line_bank,
 output wire line_done,output wire line_good,output wire [31:0] line_frame_id,output wire [31:0] line_session,
 input wire [31:0] time_gray,output wire [31:0] line_start_ms,
 input wire remote_token_empty,output wire remote_token_read,input wire remote_token_valid,input wire [129:0] remote_token_data,
 input wire remote_line_req_valid,output wire remote_line_req_ready,input wire remote_new_frame,input wire [15:0] remote_source_row,input wire remote_request_bank,
 output wire remote_line_write,output wire [127:0] remote_line_data,output wire [7:0] remote_line_address,output wire remote_line_bank,
 output wire remote_line_done,remote_line_good,output wire [31:0] remote_line_frame_id,remote_line_session,
 output wire remote_latest_valid,output wire[31:0]remote_latest_frame_id,remote_latest_session,
 output wire [31:0] remote_line_start_ms,output reg [255:0] remote_debug,
 input wire[24:0]cloud_awaddr,cloud_araddr,input wire[127:0]cloud_wdata,input wire[15:0]cloud_wstrb,
 input wire cloud_awvalid,cloud_wvalid,cloud_wlast,cloud_arvalid,cloud_rready,cloud_read_active,
 output wire cloud_awready,cloud_wready,cloud_arready,cloud_rvalid,cloud_rlast,output wire[127:0]cloud_rdata,
 output wire ddr_ck_p,output wire ddr_ck_n,output wire ddr_cke,output wire ddr_odt,
 output wire ddr_cs_n,output wire ddr_ras_n,output wire ddr_cas_n,output wire ddr_we_n,
 output wire [1:0] ddr_ba,output wire [12:0] ddr_addr,
 inout wire [1:0] ddr_dqs_p,inout wire [1:0] ddr_dqs_n,inout wire [15:0] ddr_dq,inout wire [1:0] ddr_dm
);
 wire dhi_clk,phy_reset,phy_ready;
 wire [24:0] awaddr,araddr,first_address;
 wire [127:0] wdata,rdata,expected,observed;
 wire [15:0] wstrb;
 wire awvalid,awready,wvalid,wready,wlast,arvalid,arready,rvalid,rlast,rready,done,failed;
 wire [31:0] errors,frequency;
 wire [3:0] phase;
 video_ddr memory(
  .ref_clk(clk_50),.sys_rst_n(rst_n),.apb_pclk(clk_50),.apb_prst_n(rst_n),
  .apb_psel(1'b0),.apb_penable(1'b0),.apb_pwrite(1'b0),.apb_paddr(16'b0),.apb_pwdata(32'b0),
  .apb_pready(),.apb_prdata(),.dhi_clk(dhi_clk),.cfg_clk(),.cfg_rst_n(),
  .ddrphy_rst_n(phy_reset),.ddrphy_rdy(phy_ready),
  .paxi_awaddr(awaddr),.paxi_awvalid(awvalid),.paxi_awready(awready),
  .paxi_wdata(wdata),.paxi_wstrb(wstrb),.paxi_wvalid(wvalid),.paxi_wlast(wlast),.paxi_wready(wready),
  .paxi_araddr(araddr),.paxi_arvalid(arvalid),.paxi_arready(arready),
  .paxi_rdata(rdata),.paxi_rlast(rlast),.paxi_rvalid(rvalid),.paxi_rready(rready),
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
 wire low_done,low_failed,alias_done,alias_failed;
 wire[31:0]low_errors,alias_errors;wire[24:0]low_first_address,alias_first_address;
 wire[127:0]low_expected,low_observed,alias_expected,alias_observed;
 wire[3:0]low_phase,alias_phase;
 wire[24:0]a_awaddr,a_araddr;wire[127:0]a_wdata;wire[15:0]a_wstrb;
 wire a_awvalid,a_awready,a_wvalid,a_wready,a_wlast,a_arvalid,a_arready,a_rready;
 wire alias_selected=low_done&&!low_failed;
 assign done=low_done&&(low_failed||alias_done);assign failed=low_failed||alias_failed;
 assign errors=low_failed?low_errors:alias_errors;
 assign first_address=low_failed?low_first_address:alias_first_address;
 assign expected=low_failed?low_expected:alias_expected;assign observed=low_failed?low_observed:alias_observed;
 assign phase=alias_selected?alias_phase:low_phase;
 wire video_active=done && !failed;
 assign memory_verified=video_active&&ready_sync[1];
 wire arb_fault;wire [31:0] arb_debug;
 wire [2:0] clients_awready,clients_wready,clients_arready,clients_rvalid,clients_rlast,clients_read_active;
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
 // Six non-overlapping slots, 2 Mi 16-bit words per slot, total 24 MiB.
 // Local owns slots 0..2; remote owns slots 3..5. Same physical DDR IP.
 hd_ddr_arbiter #(.CLIENTS(3),.PRIORITY_MASK(3'b101)) arbitration(.clk(dhi_clk),.rst_n(reset_sync[1]),.enable(video_active&&ready_sync[1]),
  .c_awaddr({cloud_awaddr,remote_awaddr+25'd6291456,v_awaddr}),.c_araddr({cloud_araddr,remote_araddr+25'd6291456,v_araddr}),
  .c_awvalid({cloud_awvalid,remote_awvalid,v_awvalid}),.c_wvalid({cloud_wvalid,remote_wvalid,v_wvalid}),.c_wlast({cloud_wlast,remote_wlast,v_wlast}),
  .c_arvalid({cloud_arvalid,remote_arvalid,v_arvalid}),.c_rready({cloud_rready,remote_rready,v_rready}),.c_read_active(clients_read_active),
  .c_wdata({cloud_wdata,remote_wdata,v_wdata}),.c_wstrb({cloud_wstrb,remote_wstrb,v_wstrb}),
  .c_awready(clients_awready),.c_wready(clients_wready),.c_arready(clients_arready),
  .c_rvalid(clients_rvalid),.c_rlast(clients_rlast),.c_rdata(clients_rdata),
  .awaddr(bus_awaddr),.araddr(bus_araddr),.wdata(bus_wdata),.wstrb(bus_wstrb),
  .awvalid(bus_awvalid),.wvalid(bus_wvalid),.wlast(bus_wlast),.arvalid(bus_arvalid),.rready(bus_rready),
  .awready(awready),.wready(wready),.arready(arready),.rdata(rdata),.rvalid(rvalid&&video_active),.rlast(rlast),
  .fault(arb_fault),.debug(arb_debug));
 
 
 
 
 
 assign clients_read_active[2]=cloud_read_active;
 assign cloud_awready=clients_awready[2];assign cloud_wready=clients_wready[2];assign cloud_arready=clients_arready[2];
 assign cloud_rvalid=clients_rvalid[2];assign cloud_rlast=clients_rlast[2];assign cloud_rdata=clients_rdata[383:256];
 wire [127:0] frame_debug;wire front_valid,fault;wire [31:0] front_id,published_frames,rejected_frames;
 // 与line_done一同交给行缓存邮箱；front_id在一次完整行响应内保持不变。
 assign line_frame_id=front_id;
 assign memory_clk=dhi_clk;assign memory_rst_n=reset_sync[1];
 assign awaddr=video_active?bus_awaddr:(alias_selected?a_awaddr:t_awaddr);assign awvalid=video_active?bus_awvalid:(alias_selected?a_awvalid:t_awvalid);
 assign wdata=video_active?bus_wdata:(alias_selected?a_wdata:t_wdata);assign wstrb=video_active?bus_wstrb:(alias_selected?a_wstrb:t_wstrb);
 assign wvalid=video_active?bus_wvalid:(alias_selected?a_wvalid:t_wvalid);assign wlast=video_active?bus_wlast:(alias_selected?a_wlast:t_wlast);
 assign araddr=video_active?bus_araddr:(alias_selected?a_araddr:t_araddr);assign arvalid=video_active?bus_arvalid:(alias_selected?a_arvalid:t_arvalid);
 assign rready=video_active?bus_rready:(alias_selected?a_rready:t_rready);
 assign t_awready=awready && !video_active&&!alias_selected;assign a_awready=awready && !video_active&&alias_selected;assign t_wready=wready && !video_active&&!alias_selected;assign a_wready=wready && !video_active&&alias_selected;assign t_arready=arready && !video_active&&!alias_selected;assign a_arready=arready && !video_active&&alias_selected;
 ddr_frame_store #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) frames(.clk(dhi_clk),.rst_n(reset_sync[1]),.memory_ready(ready_sync[1] && video_active && !arb_fault),
  .time_gray(time_gray),.front_start_ms(line_start_ms),
  .token_empty(token_empty),.token_read(token_read),.token_valid(token_valid),.token_data(token_data),
  .line_req_valid(line_req_valid),.line_req_ready(line_req_ready),.new_frame(new_frame),.source_row(source_row),.request_bank(request_bank),
  .line_write(line_write),.line_data(line_data),.line_address(line_address),.line_bank(line_bank),.line_done(line_done),.line_good(line_good),
  .debug_state(frame_debug),.front_valid(front_valid),.front_id(front_id),.front_session(line_session),.fault(fault),.published_frames(published_frames),.rejected_frames(rejected_frames),
  .awaddr(v_awaddr),.awvalid(v_awvalid),.awready(clients_awready[0]),.wdata(v_wdata),.wstrb(v_wstrb),
  .wvalid(v_wvalid),.wlast(v_wlast),.wready(clients_wready[0]),.araddr(v_araddr),.arvalid(v_arvalid),.arready(clients_arready[0]),
  .rdata(clients_rdata[127:0]),.rvalid(clients_rvalid[0]),.rlast(clients_rlast[0]),.rready(v_rready),.read_active(clients_read_active[0]));
 ddr_frame_store #(.WIDTH(REMOTE_WIDTH),.HEIGHT(REMOTE_HEIGHT)) remote_frames(.clk(dhi_clk),.rst_n(reset_sync[1]),.memory_ready(ready_sync[1]&&video_active&&!arb_fault),
  .latest_valid(remote_latest_valid),.latest_frame_id(remote_latest_frame_id),.latest_session(remote_latest_session),
  .time_gray(time_gray),.front_start_ms(remote_line_start_ms),
  .token_empty(remote_token_empty),.token_read(remote_token_read),.token_valid(remote_token_valid),.token_data(remote_token_data),
  .line_req_valid(remote_line_req_valid),.line_req_ready(remote_line_req_ready),.new_frame(remote_new_frame),.source_row(remote_source_row),.request_bank(remote_request_bank),
  .line_write(remote_line_write),.line_data(remote_line_data),.line_address(remote_line_address),.line_bank(remote_line_bank),.line_done(remote_line_done),.line_good(remote_line_good),
  .debug_state(remote_frame_debug),.front_valid(remote_front_valid),.front_id(remote_front_id),.front_session(remote_front_session),.fault(remote_fault),.published_frames(remote_published),.rejected_frames(remote_rejected),
  .awaddr(remote_awaddr),.awvalid(remote_awvalid),.awready(clients_awready[1]),.wdata(remote_wdata),.wstrb(remote_wstrb),
  .wvalid(remote_wvalid),.wlast(remote_wlast),.wready(clients_wready[1]),.araddr(remote_araddr),.arvalid(remote_arvalid),.arready(clients_arready[1]),
  .rdata(clients_rdata[255:128]),.rvalid(clients_rvalid[1]),.rlast(clients_rlast[1]),.rready(remote_rready),.read_active(clients_read_active[1]));
 // Isolated draft: source100/host50 PDPW snapshot ownership is unchanged.
 reg [1:0] arb_fault_sync;
 wire[255:0]debug_local_live={frame_debug,30'b0,(fault||arb_fault),front_valid,front_id,rejected_frames,published_frames};
 wire[255:0]debug_remote_live={remote_frame_debug,30'b0,(remote_fault||arb_fault),remote_front_valid,remote_front_id,remote_rejected,remote_published};
 snapshot_pdp_candidate debug_snapshot(.source_clk(dhi_clk),.host_clk(clk_50),.rst_n(rst_n),.take(debug_latch),
  .local_live(debug_local_live),.remote_live(debug_remote_live),.read_peer(debug_read_peer),.read_index(debug_read_index),
  .read_byte(debug_read_byte),.led(debug_snapshot_led),.busy(),.request_state(),.source_ack_state());
 paxi_memory_test test(.clk(dhi_clk),.rst_n(reset_sync[1]),.ready(ready_sync[1]),
  .awaddr(t_awaddr),.awvalid(t_awvalid),.awready(t_awready),.wdata(t_wdata),.wstrb(t_wstrb),.wvalid(t_wvalid),.wlast(t_wlast),.wready(t_wready),
  .araddr(t_araddr),.arvalid(t_arvalid),.arready(t_arready),.rdata(rdata),.rvalid(rvalid && !video_active&&!alias_selected),.rlast(rlast),.rready(t_rready),
  .done(low_done),.failed(low_failed),.errors(low_errors),.first_address(low_first_address),.expected(low_expected),.observed(low_observed),.phase(low_phase));
 paxi_snapshot_address_probe address_probe(.clk(dhi_clk),.rst_n(reset_sync[1]&&low_done&&!low_failed),.ready(ready_sync[1]),
 .awaddr(a_awaddr),.awvalid(a_awvalid),.awready(a_awready),.wdata(a_wdata),.wstrb(a_wstrb),.wvalid(a_wvalid),.wlast(a_wlast),.wready(a_wready),
 .araddr(a_araddr),.arvalid(a_arvalid),.arready(a_arready),.rdata(rdata),.rvalid(rvalid&&!video_active&&alias_selected),.rlast(rlast),.rready(a_rready),
 .done(alias_done),.failed(alias_failed),.errors(alias_errors),.first_address(alias_first_address),.expected(alias_expected),.observed(alias_observed),.phase(alias_phase));
 clock_frequency_monitor meter(.reference_clk(clk_50),.source_clk(dhi_clk),.rst_n(rst_n),.edges(frequency),.fresh());
 reg [1:0] done_sync,phy_sync;
 always @(posedge clk_50) begin
  if(!rst_n) begin debug<=0;remote_debug<=0;arb_fault_sync<=0;done_sync<=0;phy_sync<=0;end
  else begin
   // Upper debug/remote_debug remain reset-zero; snapshot bytes use the
   // new one-cycle CSR read port. All original low384 status writes follow.
   arb_fault_sync<={arb_fault_sync[0],arb_fault};
   done_sync<={done_sync[0],done};phy_sync<={phy_sync[0],phy_ready};
   debug[31:0]<={29'b0,(done_sync[1]&&(failed||arb_fault_sync[1])),done_sync[1],phy_sync[1]};
   debug[383:352]<=frequency;
   if(done_sync[1]) begin debug[63:32]<=errors;debug[95:64]<={7'b0,first_address};
    debug[223:96]<=expected;debug[351:224]<=observed;end
  end
 end
endmodule
