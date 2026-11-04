`timescale 1ns/1ps
// Single physical MC. Client0 is the existing complete camera-video fabric;
// client1 is the result store. Client2 is unused. The result store occupies a
// upper DDR region, with boot address probing before any owner starts.
// A result is1280x1440 words =1280x720{Z16,RGB565} points.
module stereo_depth_memory #(
 parameter TEXTURE_MIN=10,parameter CALIBRATION_READY=0,parameter[31:0]MAP_ID=0,
 parameter NODE_FILE="nodes.mem",MASK_FILE="masks.mem",LUT_FILE="real_depth_colour33.mem"
)(
 input wire clk,rst_n,memory_ready,video_active,run_enable,input wire[31:0]time_gray,
 input wire[24:0]main_awaddr,main_araddr,input wire[127:0]main_wdata,input wire[15:0]main_wstrb,
 input wire main_awvalid,main_wvalid,main_wlast,main_arvalid,main_rready,main_read_epoch,
 output wire main_awready,main_wready,main_arready,main_rvalid,main_rlast,output wire[127:0]main_rdata,
 output wire[24:0]awaddr,araddr,output wire[127:0]wdata,output wire[15:0]wstrb,
 output wire awvalid,wvalid,wlast,arvalid,rready,input wire awready,wready,arready,
 input wire[127:0]rdata,input wire rvalid,rlast,
 input wire[1:0]camera_initialized,camera_acquire_ready,camera_lease_valid,
 input wire[3:0]camera_lease_slots,input wire[95:0]camera_lease_cookies,
 input wire[191:0]camera_lease_meta,input wire[63:0]camera_published_counts,
 output wire[1:0]camera_acquire,camera_release,output wire[95:0]camera_release_cookies,
 output wire camera_pipeline_idle,
 output wire[24:0]feature_araddr,output wire feature_arvalid,feature_rready,feature_read_active,
 input wire feature_arready,feature_rvalid,feature_rlast,input wire[127:0]feature_rdata,
 input wire line_req_valid,output wire line_req_ready,input wire new_frame,input wire[15:0]source_row,input wire request_bank,
 output wire line_write,output wire[127:0]line_data,output wire[7:0]line_address,output wire line_bank,
 output wire line_done,line_good,output wire[31:0]line_frame_id,line_session,line_start_ms,
 output reg[31:0]jobs_started,jobs_completed,jobs_skipped,
 output wire export_quiescent,output wire[127:0]job_sources,
 output wire[31:0]result_published,output reg fault,output reg[3:0]fault_reason,output wire configuration_error
);
 localparam[24:0]RESULT_BASE=25'd16777216,RESULT_STRIDE=25'd2097152;
 // Upper address bits are not authorized by the legacy boot probe; the HD
 // candidate must extend and pass that probe before claiming physical support.
 // Four 4MiB result slots in the separately probed upper half of DDR.
 localparam LAYOUT_OK=RESULT_BASE==16777216&&RESULT_BASE%8==0&&RESULT_STRIDE>=1280*1440&&
                     RESULT_BASE+4*RESULT_STRIDE<=33554432;

 localparam WAITING=0,ACQUIRE=1,LAUNCH=2,WORK=3,RELEASE=4,FAILED=5,CHECK_PAIR=6;
 reg[2:0]phase;reg[63:0]attempted_counts;reg[63:0]previous_A,previous_B;
 reg pair_acceptable;reg previous_valid,stream_finished;reg run_was_enabled;reg[31:0]pair_tag,before_published;
 wire eng_ready,eng_busy,eng_done,eng_idle,eng_fault,eng_config;wire[3:0]eng_reason;
 wire result_empty,result_read,result_token_valid;wire[129:0]result_token;
 wire result_initialized,result_fault,result_config,result_read_active;
 wire outer_fault,outer_config;wire[31:0]outer_debug;
 wire[24:0]res_awaddr,res_araddr;wire[127:0]res_wdata;wire[15:0]res_wstrb;
 wire res_awvalid,res_wvalid,res_wlast,res_arvalid,res_rready;
 wire[2:0]c_awready,c_wready,c_arready,c_rvalid,c_rlast;wire[383:0]c_rdata;
 reg[5:0]main_pending;
 wire main_take_ar=main_arvalid&&main_arready,main_take_r=main_rvalid&&main_rready;
 wire parent_read_active=main_arvalid||main_pending!=0||main_read_epoch;
 always @(posedge clk)begin
  if(!rst_n)main_pending<=0;
  else case({main_take_ar,main_take_r})
   2'b10:main_pending<=main_pending+1'b1;
   2'b01:if(main_pending!=0)main_pending<=main_pending-1'b1;
   default:begin end
  endcase
 end
 assign main_awready=c_awready[0];assign main_wready=c_wready[0];assign main_arready=c_arready[0];
 assign main_rvalid=c_rvalid[0];assign main_rlast=c_rlast[0];assign main_rdata=c_rdata[127:0];
 hd_ddr_arbiter #(.CLIENTS(3),.PRIORITY_MASK(3'b000)) shared_physical_MC(.clk(clk),.rst_n(rst_n),.enable(memory_ready),
  .c_awaddr({25'd0,res_awaddr+RESULT_BASE,main_awaddr}),.c_araddr({25'd0,res_araddr+RESULT_BASE,main_araddr}),
  .c_awvalid({1'b0,res_awvalid,main_awvalid}),.c_wvalid({1'b0,res_wvalid,main_wvalid}),.c_wlast({1'b0,res_wlast,main_wlast}),
  .c_arvalid({1'b0,res_arvalid,main_arvalid}),.c_rready({1'b0,res_rready,main_rready}),.c_read_active({1'b0,result_read_active,parent_read_active}),
  .c_wdata({128'd0,res_wdata,main_wdata}),.c_wstrb({16'd0,res_wstrb,main_wstrb}),
  .c_awready(c_awready),.c_wready(c_wready),.c_arready(c_arready),.c_rvalid(c_rvalid),.c_rlast(c_rlast),.c_rdata(c_rdata),
  .awaddr(awaddr),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),.wvalid(wvalid),.wlast(wlast),.wready(wready),
  .araddr(araddr),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rvalid(rvalid),.rlast(rlast),.rready(rready),
  .fault(outer_fault),.configuration_error(outer_config),.debug(outer_debug));
 wire owners_ready=camera_initialized==3&&result_initialized;
 wire counts_new=camera_published_counts[31:0]!=0&&camera_published_counts[63:32]!=0&&
                 camera_published_counts[31:0]!=attempted_counts[31:0]&&camera_published_counts[63:32]!=attempted_counts[63:32];
 wire canonical=camera_lease_meta[191:184]==8'hA1&&camera_lease_meta[95:88]==8'hB1&&
                camera_lease_meta[181:160]!=0&&camera_lease_meta[85:64]!=0;
 wire fresh=!previous_valid||(camera_lease_meta[191:128]!=previous_A&&camera_lease_meta[95:32]!=previous_B);
 assign camera_acquire=phase==ACQUIRE&&!fault&&run_enable?~camera_lease_valid&camera_acquire_ready:2'd0;
 assign camera_release=phase==RELEASE&&!fault?camera_lease_valid:2'd0;
 assign camera_release_cookies=camera_lease_cookies;
 assign camera_pipeline_idle=eng_idle;
 assign job_sources={previous_A,previous_B};
 assign export_quiescent=phase==WAITING&&eng_idle&&camera_lease_valid==0&&jobs_completed==result_published&&memory_ready&&video_active&&!fault&&!configuration_error;
 wire launch=phase==LAUNCH&&!fault&&eng_ready&&run_enable;
 assign configuration_error=!LAYOUT_OK||outer_config||result_config||eng_config;
 stereo_dense_engine #(.TEXTURE_MIN(TEXTURE_MIN),.CALIBRATION_READY(CALIBRATION_READY),.MAP_ID(MAP_ID),.NODE_FILE(NODE_FILE),.MASK_FILE(MASK_FILE),.LUT_FILE(LUT_FILE))engine(
  .clk(clk),.rst(!rst_n),.static_mode(1'b1),.start_valid(launch),.start_ready(eng_ready),.pair_tag(pair_tag),.calibration_id(MAP_ID),.result_session(32'hD2800001),
  .lease_valid({camera_lease_valid[0],camera_lease_valid[1]}),.lease_cookie_A(camera_lease_cookies[95:48]),.lease_cookie_B(camera_lease_cookies[47:0]),
  .lease_slot_A(camera_lease_slots[3:2]),.lease_slot_B(camera_lease_slots[1:0]),.lease_meta_A(camera_lease_meta[191:96]),.lease_meta_B(camera_lease_meta[95:0]),
  .araddr(feature_araddr),.arvalid(feature_arvalid),.arready(feature_arready),.rdata(feature_rdata),.rvalid(feature_rvalid),.rlast(feature_rlast),.rready(feature_rready),.read_active(feature_read_active),
  .token_empty(result_empty),.token_read(result_read),.token_valid(result_token_valid),.token_data(result_token),.busy(eng_busy),.frame_stream_done(eng_done),.pipeline_idle(eng_idle),
  .fault(eng_fault),.fault_reason(eng_reason),.configuration_error(eng_config));
 wire store_ready=memory_ready&&video_active&&!outer_fault;
 leased_ddr_frame_store #(.WIDTH(1280),.HEIGHT(1440),.BUFFER_STRIDE(RESULT_STRIDE),.CAMERA_ID(1),.LINE_READ_GUARD(0))result_store(
  .clk(clk),.rst_n(rst_n),.memory_ready(store_ready),.time_gray(time_gray),
  .owner_rearm(store_ready&&!result_initialized),.owner_quiesce(!result_initialized),.owner_epoch(32'd1),
  .external_pipeline_idle(eng_idle),.lease_reader_idle(1'b1),.owner_initialized(result_initialized),
  .lease_acquire(1'b0),.lease_release(1'b0),.release_camera(8'd1),.release_cookie(48'd0),
  .token_empty(result_empty),.token_read(result_read),.token_valid(result_token_valid),.token_data(result_token),
  .line_req_valid(line_req_valid),.line_req_ready(line_req_ready),.new_frame(new_frame),.source_row(source_row),.request_bank(request_bank),
  .line_write(line_write),.line_data(line_data),.line_address(line_address),.line_bank(line_bank),.line_done(line_done),.line_good(line_good),
  .front_id(line_frame_id),.front_session(line_session),.front_start_ms(line_start_ms),
  .awaddr(res_awaddr),.awvalid(res_awvalid),.awready(c_awready[1]),.wdata(res_wdata),.wstrb(res_wstrb),.wvalid(res_wvalid),.wlast(res_wlast),.wready(c_wready[1]),
  .araddr(res_araddr),.arvalid(res_arvalid),.arready(c_arready[1]),.rdata(c_rdata[255:128]),.rvalid(c_rvalid[1]),.rlast(c_rlast[1]),.rready(res_rready),
  .fault(result_fault),.configuration_error(result_config),.published_frames(result_published),.read_active(result_read_active));
 always @(posedge clk)begin
  if(!rst_n)begin pair_acceptable<=0;phase<=WAITING;attempted_counts<=0;previous_A<=0;previous_B<=0;previous_valid<=0;
   stream_finished<=0;run_was_enabled<=0;pair_tag<=0;before_published<=0;jobs_started<=0;jobs_completed<=0;jobs_skipped<=0;fault<=0;fault_reason<=0;end
  else begin
   run_was_enabled<=run_enable;
   if(run_enable&&!run_was_enabled)attempted_counts<=camera_published_counts;
   if(outer_fault||result_fault||eng_fault||configuration_error)begin
    fault<=1;fault_reason<=outer_fault?1:result_fault?2:eng_fault?3:4;phase<=FAILED;
   end else case(phase)
    WAITING:if(run_enable&&run_was_enabled&&owners_ready&&counts_new&&jobs_started!=32'hffffffff)begin
     attempted_counts<=camera_published_counts;phase<=ACQUIRE;
    end
    ACQUIRE:if(!run_enable)begin
     // A missing second camera must not strand an already acquired lease.
     jobs_skipped<=jobs_skipped+1'b1;phase<=RELEASE;
    end else if(camera_lease_valid==3)begin
     // Both immutable leases remain owned during this extra validation cycle.
     pair_acceptable<=canonical&&fresh;phase<=CHECK_PAIR;
    end
    CHECK_PAIR:if(!run_enable||!pair_acceptable)begin
     jobs_skipped<=jobs_skipped+1'b1;phase<=RELEASE;
    end else begin pair_tag<=jobs_started+1'b1;phase<=LAUNCH;end
    LAUNCH:if(!run_enable)begin jobs_skipped<=jobs_skipped+1'b1;phase<=RELEASE;end
    else if(launch)begin
     jobs_started<=jobs_started+1'b1;before_published<=result_published;stream_finished<=0;
     previous_A<=camera_lease_meta[191:128];previous_B<=camera_lease_meta[95:32];previous_valid<=1;phase<=WORK;
    end
    WORK:begin
     if(eng_done)stream_finished<=1;
     if((stream_finished||eng_done)&&eng_idle&&result_published!=before_published)begin
      if(result_published!=before_published+1'b1)begin fault<=1;fault_reason<=5;phase<=FAILED;end
      else begin jobs_completed<=jobs_completed+1'b1;phase<=RELEASE;end
     end
    end
    RELEASE:if(camera_lease_valid==0)phase<=WAITING;
    FAILED:begin end
    default:begin fault<=1;fault_reason<=6;phase<=FAILED;end
   endcase
  end
 end
endmodule
