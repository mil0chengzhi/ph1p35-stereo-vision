`timescale 1ns/1ps
// Actual composed computational engine. No camera PHY or memory arbitration.
// Only canonical A1/B1 source leases and explicit static mode are accepted.
// RGB565 input is pre-display ISP data, not raw Bayer. Static mode does not
// establish synchronized exposures. Keep leases until frame-stream drain AND
// the external result store's complete-frame publication or verified abort.
module stereo_dense_engine #(
 parameter WIDTH=1280,HEIGHT=720,X_BITS=11,Y_BITS=10,TEXTURE_MIN=10,
 parameter CALIBRATION_READY=0,parameter[31:0]MAP_ID=0,
 parameter NODE_FILE="nodes.mem",MASK_FILE="masks.mem",LUT_FILE="real_depth_colour33.mem"
)(
 input wire clk,rst,input wire static_mode,start_valid,output wire start_ready,
 input wire[31:0]pair_tag,calibration_id,result_session,
 input wire[1:0]lease_valid,input wire[47:0]lease_cookie_A,lease_cookie_B,
 input wire[1:0]lease_slot_A,lease_slot_B,input wire[95:0]lease_meta_A,lease_meta_B,
 output wire[24:0]araddr,output wire arvalid,input wire arready,
 input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,read_active,
 output wire token_empty,input wire token_read,output wire token_valid,output wire[129:0]token_data,
 output reg busy,frame_stream_done,output wire pipeline_idle,
 output reg fault,output reg[3:0]fault_reason,output wire configuration_error
);
 wire f_ready,f_busy,f_done,f_fault,f_config;wire[3:0]f_reason;
 wire lvalid,rvalid_feature,lready,rready_feature,lu,ru;wire[23:0]ld,rd;
 wire[31:0]lt,rt;wire[X_BITS-1:0]lx,rx;wire[Y_BITS-1:0]ly,ry;
 wire d_ready,d_busy,d_done,d_fault,d_config,d_valid,d_accept,d_usable;
 wire[3:0]d_reason;wire[31:0]d_tag;wire[X_BITS-1:0]d_x;wire[Y_BITS-1:0]d_y;wire[7:0]d_disparity;
 wire m_ready,m_valid,m_accept,m_usable,m_fault,m_config;wire[15:0]m_Z,m_colour;
 wire[31:0]m_tag;wire[X_BITS-1:0]m_x;wire[Y_BITS-1:0]m_y;wire[7:0]m_disparity;
 wire p_ready,p_busy,p_sent,p_good,p_fault,p_config;
 wire colour_valid,j_valid,j_ready,j_fault;wire[31:0]colour_index,j_tag;wire[15:0]colour_rgb,j_z,j_rgb;wire[X_BITS-1:0]j_x;wire[Y_BITS-1:0]j_y;
 wire s_start_ready,s_busy,s_done,s_fault,s_config,s_valid,s_ready;
 wire[3:0]s_reason;wire[7:0]j_disparity,s_disparity;wire[15:0]s_z,s_rgb;
 wire[31:0]s_tag;wire[X_BITS-1:0]s_x;wire[Y_BITS-1:0]s_y;
 reg[31:0]active_pair,active_result_session;reg[3:0]completed;
 reg[95:0]active_meta_A,active_meta_B;
 wire canonical=lease_valid==3&&lease_meta_A[95:88]==8'hA1&&lease_meta_B[95:88]==8'hB1&&
                lease_meta_A[85:64]!=0&&lease_meta_B[85:64]!=0;
 assign configuration_error=f_config||d_config||m_config||p_config||s_config;
 assign start_ready=!rst&&!busy&&!fault&&!configuration_error&&static_mode&&canonical&&
                    calibration_id==MAP_ID&&result_session[31:24]==8'hD2&&result_session[21:0]!=0&&f_ready&&d_ready&&p_ready&&s_start_ready;
 wire take_start=start_valid&&start_ready;
 wire any_fault=f_fault||d_fault||m_fault||j_fault||s_fault||(busy&&p_fault);
 assign pipeline_idle=!busy&&!f_busy&&!d_busy&&!p_busy&&!s_busy&&!s_valid&&!m_valid&&!j_valid&&!read_active&&token_empty&&!token_valid;
 stereo_leased_rectified_features #(.TEXTURE_MIN(TEXTURE_MIN),.WIDTH(WIDTH),.HEIGHT(HEIGHT),.X_BITS(X_BITS),.Y_BITS(Y_BITS),.LEFT_CAMERA(1),
  .CALIBRATION_READY(CALIBRATION_READY),.MAP_ID(MAP_ID),.NODE_FILE(NODE_FILE),.MASK_FILE(MASK_FILE))features(
  .clk(clk),.rst(rst),.start_valid(take_start),.start_ready(f_ready),.pair_tag(pair_tag),.calibration_id(calibration_id),
  .start_cookie_A(lease_cookie_A),.start_cookie_B(lease_cookie_B),.lease_valid(lease_valid),
  .lease_cookie_A(lease_cookie_A),.lease_cookie_B(lease_cookie_B),.lease_slot_A(lease_slot_A),.lease_slot_B(lease_slot_B),
  .araddr(araddr),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rvalid(rvalid),.rlast(rlast),.rready(rready),.read_active(read_active),
  .left_valid(lvalid),.left_ready(lready),.left_descriptor(ld),.left_usable(lu),.left_frame_tag(lt),.left_x(lx),.left_y(ly),
  .right_valid(rvalid_feature),.right_ready(rready_feature),.right_descriptor(rd),.right_usable(ru),.right_frame_tag(rt),.right_x(rx),.right_y(ry),
  .colour_valid(colour_valid),.colour_index(colour_index),.colour_rgb(colour_rgb),
  .frame_done(f_done),.busy(f_busy),.fault(f_fault),.fault_reason(f_reason),.configuration_error(f_config));
 hd_sgm1_matcher #(.P1(8),.P2(24),.WIDTH(WIDTH),.HEIGHT(HEIGHT),.X_BITS(X_BITS),.Y_BITS(Y_BITS))matching(
  .clk(clk),.rst(rst),.start_valid(take_start),.start_ready(d_ready),.pair_tag(pair_tag),
  .cost_limit(5'd24),.uniqueness_margin(5'd2),.tolerance(3'd1),
  .left_valid(lvalid),.left_ready(lready),.left_descriptor(ld),.left_usable(lu),.left_frame_tag(lt),.left_x(lx),.left_y(ly),
  .right_valid(rvalid_feature),.right_ready(rready_feature),.right_descriptor(rd),.right_usable(ru),.right_frame_tag(rt),.right_x(rx),.right_y(ry),
  .out_valid(d_valid),.out_ready(m_ready&&!fault),.out_frame_tag(d_tag),.out_x(d_x),.out_y(d_y),.out_disparity(d_disparity),.out_usable(d_usable),
  .busy(d_busy),.frame_done(d_done),.fault(d_fault),.fault_reason(d_reason),.configuration_error(d_config));
 stereo_metric_colour #(.CALIBRATION_READY(CALIBRATION_READY),.MAP_ID(MAP_ID),.X_BITS(X_BITS),.Y_BITS(Y_BITS),.LUT_FILE(LUT_FILE))metric(
  .clk(clk),.rst(rst),.calibration_id(calibration_id),.in_valid(d_valid&&!fault),.in_ready(m_ready),.in_disparity(d_disparity),.in_usable(d_usable),
  .in_frame_tag(d_tag),.in_x(d_x),.in_y(d_y),.out_valid(m_valid),.out_ready(m_accept&&!fault),.out_Z_mm(m_Z),.out_colour(m_colour),
  .out_usable(m_usable),.out_disparity(m_disparity),.out_frame_tag(m_tag),.out_x(m_x),.out_y(m_y),.fault(m_fault),.configuration_error(m_config));
 stereo_rgb_join #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.X_BITS(X_BITS),.Y_BITS(Y_BITS))rgb_join(
  .clk(clk),.rst(rst),.start(take_start),.frame_tag(pair_tag),
  .colour_valid(colour_valid),.colour_index(colour_index),.colour_rgb(colour_rgb),
  .in_valid(m_valid&&!fault),.in_ready(m_accept),.in_z(m_Z),.in_disparity(m_disparity),.in_tag(m_tag),.in_x(m_x),.in_y(m_y),
  .out_valid(j_valid),.out_ready(j_ready),.out_z(j_z),.out_rgb(j_rgb),.out_disparity(j_disparity),.out_tag(j_tag),.out_x(j_x),.out_y(j_y),.fault(j_fault));
 disparity_support3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.X_BITS(X_BITS),.Y_BITS(Y_BITS))support(
  .clk(clk),.rst(rst),.start_valid(take_start),.start_ready(s_start_ready),.frame_tag(pair_tag),
  .in_valid(j_valid&&!fault),.in_ready(j_ready),.in_disparity(j_disparity),.in_Z_mm(j_z),.in_RGB565(j_rgb),
  .in_frame_tag(j_tag),.in_x(j_x),.in_y(j_y),.out_valid(s_valid),.out_ready(s_ready&&!fault),
  .out_disparity(s_disparity),.out_Z_mm(s_z),.out_RGB565(s_rgb),.out_frame_tag(s_tag),.out_x(s_x),.out_y(s_y),
  .busy(s_busy),.frame_done(s_done),.fault(s_fault),.fault_reason(s_reason),.configuration_error(s_config));
 stereo_result_packer #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.X_BITS(X_BITS),.Y_BITS(Y_BITS))packer(
  .clk(clk),.rst(rst),.start_valid(take_start),.start_ready(p_ready),.start_frame(pair_tag),.start_session(result_session),.abort_frame(fault||any_fault),
  .in_valid(s_valid&&!fault),.in_ready(s_ready),.in_Z_mm(s_z),.in_colour(s_rgb),.in_frame_tag(s_tag),.in_x(s_x),.in_y(s_y),
  .token_empty(token_empty),.token_read(token_read),.token_valid(token_valid),.token_data(token_data),.busy(p_busy),
  .frame_sent(p_sent),.frame_good(p_good),.frame_fault(p_fault),.configuration_error(p_config));
 always @(posedge clk)begin
  if(rst)begin busy<=0;frame_stream_done<=0;fault<=0;fault_reason<=0;completed<=0;active_pair<=0;active_result_session<=0;active_meta_A<=0;active_meta_B<=0;end
  else begin
   frame_stream_done<=0;
   if(take_start)begin busy<=1;completed<=0;active_pair<=pair_tag;active_result_session<=result_session;active_meta_A<=lease_meta_A;active_meta_B<=lease_meta_B;end
   if(busy&&!fault)begin
    completed<=completed|{p_sent&&p_good,s_done,d_done,f_done};
    if(any_fault||calibration_id!=MAP_ID||pair_tag!=active_pair||result_session!=active_result_session||!static_mode||!canonical||lease_meta_A!=active_meta_A||lease_meta_B!=active_meta_B)begin
     fault<=1;fault_reason<=f_fault?1:d_fault?2:m_fault?3:p_fault?4:s_fault?6:5;
    end else if((completed|{p_sent&&p_good,s_done,d_done,f_done})==15&&!p_busy&&!s_busy&&!s_valid&&!m_valid&&!j_valid&&!read_active)begin
     busy<=0;frame_stream_done<=1;
    end
   end
  end
 end
endmodule
