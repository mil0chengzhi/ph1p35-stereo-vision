`timescale 1ns/1ps
// Complete native720 memory-side receive/render/scan pipeline. The physical MC
// and camera fabric remain external, with a second arbitration boundary.
module hd_render_system(
 input wire memory_clk,pixel_clk,rst_n,enable,
 input wire packet_valid,input wire[191:0]packet_header,output wire packet_consume,
 output wire[10:0]payload_address,input wire[7:0]payload_data,
 output wire reply_valid,input wire reply_ready,output wire[7:0]reply_kind,
 output wire[31:0]reply_job,output wire[15:0]reply_sequence,
 input wire view_valid,input wire[47:0]view_data,output wire view_ready,output wire[15:0]view_applied_token,
 input wire[15:0]pixel_x,pixel_y,input wire[31:0]display_completed_job,output wire front_bank_pixel,output wire[31:0]pixel_job,published_job,
 output wire pixel_cloud_valid,output wire latest_valid,output wire[31:0]latest_job,latest_capture_job,
 input wire line_request,input wire[15:0]line_row,input wire line_cache_bank,input wire[32:0]line_publication,
 output wire line_ready,line_write,line_bank,line_done,line_good,output wire[127:0]line_data,output wire[7:0]line_address,output wire[31:0]line_frame,line_session,
 output wire[24:0]awaddr,araddr,output wire[127:0]wdata,output wire[15:0]wstrb,
 output wire awvalid,wvalid,wlast,arvalid,rready,read_active,input wire awready,wready,arready,input wire[127:0]rdata,input wire rvalid,rlast,
 output wire fault,output wire[31:0]rendered_frames,rejected_frames,projected_points,accepted_packets,
 output wire[5:0]state_debug
);
 wire[99:0]c_awaddr,c_araddr;wire[511:0]c_wdata,c_rdata;wire[63:0]c_wstrb;
 wire[3:0]c_awvalid,c_wvalid,c_wlast,c_arvalid,c_rready,c_active,c_awready,c_wready,c_arready,c_rvalid,c_rlast;
 wire receiver_fault,reader_fault,painter_fault,projector_fault,scan_fault,arbiter_fault,arbiter_config,scan_ready;
 wire[31:0]bank0_job,bank1_job;wire model_start,model_ready,model_busy,model_done,point_valid,point_ready;
 wire[31:0]requested_job,point_job,point_data;wire[19:0]point_index;
 wire frame_begin,frame_ready,frame_clear_done,draw_bank,draw_valid,draw_ready,flush,flush_done;
 wire[10:0]draw_x;wire[9:0]draw_y;wire[15:0]draw_rgb,draw_z;
 assign fault=receiver_fault||reader_fault||painter_fault||projector_fault||scan_fault||arbiter_fault||arbiter_config;
 hd_model_receiver #(.ALLOW_CANCEL(1))receive_model(.clk(memory_clk),.rst_n(rst_n),.enable(enable&&!fault),
  .packet_valid(packet_valid),.packet_header(packet_header),.packet_consume(packet_consume),.payload_address(payload_address),.payload_data(payload_data),
  .reply_valid(reply_valid),.reply_ready(reply_ready),.reply_kind(reply_kind),.reply_job(reply_job),.reply_sequence(reply_sequence),
  .reader_busy(model_busy),.reader_bank(requested_job==bank1_job),.published_job(display_completed_job),.rendered_job(published_job),
  .latest_valid(latest_valid),.latest_job(latest_job),.latest_capture_job(latest_capture_job),.bank0_job(bank0_job),.bank1_job(bank1_job),.busy(),.fault(receiver_fault),.accepted_packets(accepted_packets),.duplicate_packets(),
  .awaddr(c_awaddr[75+:25]),.araddr(c_araddr[75+:25]),.wdata(c_wdata[384+:128]),.wstrb(c_wstrb[48+:16]),
  .awvalid(c_awvalid[3]),.wvalid(c_wvalid[3]),.wlast(c_wlast[3]),.arvalid(c_arvalid[3]),.rready(c_rready[3]),.read_active(c_active[3]),
  .awready(c_awready[3]),.wready(c_wready[3]),.arready(c_arready[3]),.rdata(c_rdata[384+:128]),.rvalid(c_rvalid[3]),.rlast(c_rlast[3]));
 hd_model_reader models(.clk(memory_clk),.rst_n(rst_n),.start(model_start),.requested_job(requested_job),.bank0_job(bank0_job),.bank1_job(bank1_job),
  .start_ready(model_ready),.busy(model_busy),.done(model_done),.fault(reader_fault),.point_valid(point_valid),.point_ready(point_ready),.point_job(point_job),.point_data(point_data),.point_index(point_index),
  .araddr(c_araddr[50+:25]),.arvalid(c_arvalid[2]),.arready(c_arready[2]),.rdata(c_rdata[256+:128]),.rvalid(c_rvalid[2]),.rlast(c_rlast[2]),.rready(c_rready[2]),.read_active(c_active[2]),.cycles_elapsed());
 assign c_awaddr[50+:25]=0;assign c_wdata[256+:128]=0;assign c_wstrb[32+:16]=0;assign c_awvalid[2]=0;assign c_wvalid[2]=0;assign c_wlast[2]=0;
 hd_cloud_projector project(.memory_clk(memory_clk),.pixel_clk(pixel_clk),.rst_n(rst_n),.enable_memory(enable&&!fault),.enable_pixel(1'b1),
  .latest_valid(latest_valid),.latest_job(latest_job),.view_valid(view_valid),.view_data(view_data),.view_ready(view_ready),.view_applied_token(view_applied_token),
  .model_begin(model_start),.model_ready(model_ready),.requested_job(requested_job),.point_valid(point_valid),.point_ready(point_ready),.point_job(point_job),.point_data(point_data),.point_index(point_index),.model_done(model_done),.model_fault(reader_fault||painter_fault||receiver_fault||arbiter_fault||arbiter_config),
  .frame_begin(frame_begin),.frame_ready(frame_ready),.frame_clear_done(frame_clear_done),.draw_bank(draw_bank),.draw_valid(draw_valid),.draw_ready(draw_ready),.draw_x(draw_x),.draw_y(draw_y),.draw_rgb(draw_rgb),.draw_z(draw_z),.frame_flush(flush),.frame_flush_done(flush_done),
  .pixel_x(pixel_x),.pixel_y(pixel_y),.front_bank_pixel(front_bank_pixel),.pixel_job(pixel_job),.published_job_memory(published_job),.pixel_cloud_valid(pixel_cloud_valid),
  .rendered_frames(rendered_frames),.rejected_frames(rejected_frames),.projected_points(projected_points),.fault(projector_fault),.state_debug(state_debug));
 hd_tile_painter #(.CACHE_BITS(5))paint(.clk(memory_clk),.rst_n(rst_n),.frame_begin(frame_begin),.frame_bank(draw_bank),.frame_ready(frame_ready),.frame_clear_done(frame_clear_done),
  .draw_valid(draw_valid),.draw_ready(draw_ready),.draw_x(draw_x),.draw_y(draw_y),.draw_rgb(draw_rgb),.draw_z(draw_z),.frame_flush(flush),.frame_flush_done(flush_done),.fault(painter_fault),.cache_hits(),.cache_misses(),
  .awaddr(c_awaddr[25+:25]),.araddr(c_araddr[25+:25]),.wdata(c_wdata[128+:128]),.wstrb(c_wstrb[16+:16]),
  .awvalid(c_awvalid[1]),.wvalid(c_wvalid[1]),.wlast(c_wlast[1]),.arvalid(c_arvalid[1]),.rready(c_rready[1]),.read_active(c_active[1]),
  .awready(c_awready[1]),.wready(c_wready[1]),.arready(c_arready[1]),.rdata(c_rdata[128+:128]),.rvalid(c_rvalid[1]),.rlast(c_rlast[1]));
 hd_scan_reader scan(.clk(memory_clk),.rst_n(rst_n),.request(line_request&&enable),.row(line_row),.cache_bank(line_cache_bank),.publication(line_publication),
  .ready(scan_ready),.line_write(line_write),.line_data(line_data),.line_address(line_address),.line_bank(line_bank),.line_done(line_done),.line_good(line_good),.line_frame(line_frame),.line_session(line_session),
  .araddr(c_araddr[0+:25]),.arvalid(c_arvalid[0]),.arready(c_arready[0]),.rdata(c_rdata[0+:128]),.rvalid(c_rvalid[0]),.rlast(c_rlast[0]),.rready(c_rready[0]),.read_active(c_active[0]),.fault(scan_fault));
 assign line_ready=enable&&scan_ready;
 assign c_awaddr[0+:25]=0;assign c_wdata[0+:128]=0;assign c_wstrb[0+:16]=0;assign c_awvalid[0]=0;assign c_wvalid[0]=0;assign c_wlast[0]=0;
 hd_ddr_arbiter #(.CLIENTS(4),.WRITE_DRAIN(1),.PRIORITY_MASK(4'b0001))memory_clients(.clk(memory_clk),.rst_n(rst_n),.enable(enable),
  .c_awaddr(c_awaddr),.c_araddr(c_araddr),.c_wdata(c_wdata),.c_wstrb(c_wstrb),.c_awvalid(c_awvalid),.c_wvalid(c_wvalid),.c_wlast(c_wlast),.c_arvalid(c_arvalid),.c_rready(c_rready),.c_read_active(c_active),
  .c_awready(c_awready),.c_wready(c_wready),.c_arready(c_arready),.c_rvalid(c_rvalid),.c_rlast(c_rlast),.c_rdata(c_rdata),
  .awaddr(awaddr),.araddr(araddr),.wdata(wdata),.wstrb(wstrb),.awvalid(awvalid),.wvalid(wvalid),.wlast(wlast),.arvalid(arvalid),.rready(rready),
  .awready(awready),.wready(wready),.arready(arready),.rdata(rdata),.rvalid(rvalid),.rlast(rlast),.fault(arbiter_fault),.configuration_error(arbiter_config),.transaction_read_active(read_active),.debug());
endmodule
