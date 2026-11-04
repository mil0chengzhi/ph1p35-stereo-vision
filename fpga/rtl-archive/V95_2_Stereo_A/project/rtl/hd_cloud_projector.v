`timescale 1ns/1ps
// Native1280x720 points -> full1280x720 orthographic view.
// The input model and pose are immutable for each render. RGB/Z storage is
// external; this module contains no reduced-resolution framebuffer.
module hd_cloud_projector #(parameter ORBIT_CYCLES=12000000)(
 input wire memory_clk,pixel_clk,rst_n,enable_memory,enable_pixel,
 input wire latest_valid,input wire[31:0]latest_job,
 input wire view_valid,input wire[47:0]view_data,output wire view_ready,
 output reg[15:0]view_applied_token,
 output wire model_begin,input wire model_ready,output wire[31:0]requested_job,
 input wire point_valid,output wire point_ready,input wire[31:0]point_job,point_data,
 input wire[19:0]point_index,input wire model_done,model_fault,
 output wire frame_begin,input wire frame_ready,frame_clear_done,output wire draw_bank,
 output wire draw_valid,input wire draw_ready,output wire[10:0]draw_x,output wire[9:0]draw_y,
 output wire[15:0]draw_rgb,draw_z,output wire frame_flush,input wire frame_flush_done,
 input wire[15:0]pixel_x,pixel_y,
 output wire front_bank_pixel,output wire[31:0]pixel_job,published_job_memory,
 output wire pixel_cloud_valid,
 output reg[31:0]rendered_frames,rejected_frames,projected_points,
 output reg fault,output wire[5:0]state_debug
);
 localparam IDLE=0,BEGIN_FRAME=1,WAIT_CLEAR=2,START_SCAN=3,MEASURE=4,MEDIAN=5,GAIN_START=6,GAIN_DIV=7,
 WAIT_POINT=8,MX=9,MY=10,MZ=11,XYC=12,XZS=13,ZZC=14,ZXS=15,YYC=16,YZS=17,ZZP=18,ZYS=19,
 SX=20,SY=21,PROJECT=22,DRAW=23,FLUSH=24,ACK=25,FAILED=26,HIST_READ=27,HIST_WRITE=28,MEDIAN_READ=29;
 reg[5:0]state;assign state_debug=state;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]memory_reset;
 always @(posedge memory_clk or negedge rst_n)if(!rst_n)memory_reset<=0;else memory_reset<={memory_reset[0],1'b1};
 reg[47:0]desired_pose,active_pose;reg[31:0]source_job;reg back_bank,measure_pass,stream_finished;
 reg[19:0]point_count,valid_count;reg[10:0]scan_u,saved_u;reg[9:0]scan_v,saved_v;
 (* ram_style="bram" *)reg[19:0]histogram[0:15];reg[19:0]hist_q;
 reg[4:0]hist_clear_index;reg[3:0]hist_bin,hist_index;reg[20:0]hist_sum;
 reg[15:0]pivot_z,z,rgb;
 reg signed[26:0]ma,rotated_z;
 reg signed[24:0]xc,yc;
 reg signed[25:0]zc,rx,rz,ry,tmp;
 reg signed[17:0]mb;wire signed[45:0]product=ma*mb;
 wire signed[27:0]rot_product=product>>>14;
 // All four rotation updates are state-exclusive. Carry-in implements
 // subtraction without a separate full-width negation datapath.
 wire subtract_rotation=state==ZXS;
 wire[27:0]rotation_b=rot_product^{28{subtract_rotation}};
 wire[27:0]rotation_a={{2{tmp[25]}},tmp};
 wire[27:0]rotation_sum=rotation_a+rotation_b+subtract_rotation;
 wire signed[18:0]scaled_screen=product>>>28;
 wire signed[18:0]screen_sum=scaled_screen+(state==SX?19'sd640:19'sd360);

 reg signed[18:0]screen_x,screen_y;
 reg[17:0]gain;reg[31:0]div_numerator,div_quotient;reg[16:0]div_remainder;reg[5:0]div_count;
 wire[16:0]next_remainder={div_remainder[15:0],div_numerator[31]};
 wire quotient_bit=next_remainder>={1'b0,pivot_z};
 wire[31:0]next_quotient={div_quotient[30:0],quotient_bit};
 reg[(ORBIT_CYCLES>0?$clog2(ORBIT_CYCLES+1):1)-1:0]orbit_age;reg signed[7:0]auto_yaw;reg auto_up;reg seen_view;
 wire fields_valid=$signed(view_data[7:0])>=-24&&$signed(view_data[7:0])<=24&&view_data[1:0]==0&&
  $signed(view_data[15:8])>=-24&&$signed(view_data[15:8])<=24&&view_data[9:8]==0&&view_data[23:16]>=8&&view_data[23:16]<=24&&view_data[40:25]!=0;
 assign view_ready=1'b1;
 always @(posedge memory_clk)begin
  if(!memory_reset[1])begin desired_pose<={7'd0,16'd0,1'b0,8'd16,8'd16,8'd0};seen_view<=0;end
  else if(view_valid&&fields_valid)begin desired_pose<=view_data;seen_view<=1;end
 end
 function signed[15:0]trig(input signed[7:0]degrees,input want_sin);
  reg[7:0]magnitude;reg signed[15:0]c,s;
  begin magnitude=degrees<0?-degrees:degrees;c=14968;s=6664;
   case(magnitude>>2)
   0:begin c=16384;s=0;end
   1:begin c=16344;s=1143;end
   2:begin c=16225;s=2280;end
   3:begin c=16026;s=3406;end
   4:begin c=15749;s=4516;end
   5:begin c=15396;s=5604;end
   6:begin c=14968;s=6664;end
   endcase
   trig=want_sin?(degrees<0?-s:s):c;
  end
 endfunction
 wire signed[7:0]yaw=active_pose[24]?auto_yaw:$signed(active_pose[7:0]);
 wire signed[15:0]cyaw=trig(yaw,0),syaw=trig(yaw,1),cpitch=trig(active_pose[15:8],0),spitch=trig(active_pose[15:8],1);
 wire[15:0]incoming_z=point_data[31:16];wire z_valid=incoming_z>=500&&incoming_z<=1500;
 wire[3:0]bin=incoming_z>=1460?4'd15:(incoming_z-16'd500)>>6;
 wire[20:0]next_hist=hist_sum+{1'b0,hist_q};
 always @(posedge memory_clk)begin
  if(state==HIST_READ||state==MEDIAN_READ)hist_q<=histogram[state==HIST_READ?hist_bin:hist_index];
  if(!fault)begin
   if(state==WAIT_CLEAR&&hist_clear_index<16)histogram[hist_clear_index[3:0]]<=0;
   else if(state==HIST_WRITE)histogram[hist_bin]<=hist_q+1'b1;
  end
 end
 wire take_point=point_valid&&point_ready;
 wire sample_bad=point_job!=source_job||point_index!=point_count;
 wire source_is_new=latest_job!=source_job;
 wire pose_dirty=seen_view&&desired_pose!=active_pose;
 wire publish_busy;wire[32:0]publication,pub_ack;
 cdc_mode #(.WIDTH(33))hd_present(.src_clk(memory_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
  .write(state==FLUSH&&frame_flush_done&&!publish_busy),.mode({source_job,back_bank}),.busy(publish_busy),
  .commit(pixel_x==0&&pixel_y==720),.destination_mode(publication),.src_mode(pub_ack));
 assign front_bank_pixel=publication[0];assign pixel_job=publication[32:1];
 assign pixel_cloud_valid=enable_pixel&&publication[32:1]!=0;
 assign published_job_memory=pub_ack[32:1];
 assign model_begin=state==START_SCAN&&!fault;assign requested_job=source_job;
 assign point_ready=!fault&&(state==MEASURE||state==WAIT_POINT);
 assign frame_begin=state==BEGIN_FRAME&&!fault;assign draw_bank=back_bank;
 assign draw_valid=state==DRAW&&!fault;assign draw_x=screen_x[10:0];assign draw_y=screen_y[9:0];assign draw_rgb=rgb;
 wire signed[28:0]depth_key=$signed(rotated_z)+29'sd33554432;
 assign draw_z=depth_key<1024?16'd1:depth_key>=67107840?16'd65534:depth_key[25:10];
 assign frame_flush=state==FLUSH&&!fault;
 integer k;
 always @(posedge memory_clk)begin
  if(!memory_reset[1])begin
   state<=IDLE;active_pose<={7'd0,16'd0,1'b0,8'd16,8'd16,8'd0};source_job<=0;back_bank<=1;measure_pass<=1;
   point_count<=0;valid_count<=0;scan_u<=0;scan_v<=0;saved_u<=0;saved_v<=0;hist_index<=0;hist_sum<=0;pivot_z<=1000;z<=0;rgb<=0;stream_finished<=0;
   ma<=0;mb<=0;xc<=0;yc<=0;zc<=0;rx<=0;rz<=0;ry<=0;tmp<=0;rotated_z<=0;screen_x<=0;screen_y<=0;
   gain<=0;div_numerator<=0;div_quotient<=0;div_remainder<=0;div_count<=0;orbit_age<=0;auto_yaw<=0;auto_up<=1;
   rendered_frames<=0;rejected_frames<=0;projected_points<=0;fault<=0;view_applied_token<=0;
   hist_clear_index<=0;hist_bin<=0;
  end else if(!fault)begin
   if(model_done)stream_finished<=1;
   if(model_fault&&state!=IDLE)begin fault<=1;rejected_frames<=rejected_frames+1'b1;state<=FAILED;end
   else if(take_point&&sample_bad)begin fault<=1;rejected_frames<=rejected_frames+1'b1;state<=FAILED;end
   else if(state==MEASURE&&(model_done||stream_finished)&&point_count+(take_point?1:0)!=921600)begin fault<=1;state<=FAILED;end
   else begin
    if(take_point)begin
     point_count<=point_count+1'b1;
     if(scan_u==1279)begin scan_u<=0;scan_v<=scan_v+1'b1;end else scan_u<=scan_u+1'b1;
    end
    case(state)
     IDLE:if(enable_memory&&latest_valid&&latest_job!=0&&!publish_busy&&
       (source_is_new||pose_dirty||(active_pose[24]&&orbit_age>=ORBIT_CYCLES)))begin
      active_pose<=desired_pose;source_job<=latest_job;back_bank<=!pub_ack[0];measure_pass<=source_is_new;
      projected_points<=0;point_count<=0;valid_count<=0;scan_u<=0;scan_v<=0;hist_index<=0;hist_sum<=0;
      hist_clear_index<=0;hist_bin<=0;
      if(source_is_new||pose_dirty)begin auto_yaw<=0;auto_up<=1;end
      else if(auto_up)begin if(auto_yaw>=24)begin auto_up<=0;auto_yaw<=20;end else auto_yaw<=auto_yaw+4;end
      else if(auto_yaw<=-24)begin auto_up<=1;auto_yaw<=-20;end else auto_yaw<=auto_yaw-4;
      state<=BEGIN_FRAME;
     end else if(orbit_age<ORBIT_CYCLES)orbit_age<=orbit_age+1'b1;
     BEGIN_FRAME:if(frame_ready)state<=WAIT_CLEAR;
     WAIT_CLEAR:begin
      if(hist_clear_index<16)hist_clear_index<=hist_clear_index+1'b1;
      if(frame_clear_done&&hist_clear_index==16)begin
       point_count<=0;scan_u<=0;scan_v<=0;
       if(measure_pass)state<=START_SCAN;else state<=GAIN_START;
      end
     end
     START_SCAN:if(model_ready)begin stream_finished<=0;state<=measure_pass?MEASURE:WAIT_POINT;end
     MEASURE:begin
      if(take_point&&z_valid)begin hist_bin<=bin;valid_count<=valid_count+1'b1;state<=HIST_READ;end
      else if(model_done||stream_finished)begin
       if(point_count+(take_point?1:0)!=921600)begin fault<=1;state<=FAILED;end
       else begin hist_index<=0;hist_sum<=0;state<=MEDIAN_READ;end
      end
     end
     HIST_READ:state<=HIST_WRITE;
     HIST_WRITE:state<=MEASURE;
     MEDIAN_READ:state<=MEDIAN;
     MEDIAN:if(valid_count==0)begin pivot_z<=1000;state<=GAIN_START;end
      else if(next_hist>=(({1'b0,valid_count}+21'd1)>>1)||hist_index==15)begin pivot_z<=532+{hist_index,6'd0};state<=GAIN_START;end
      else begin hist_sum<=next_hist;hist_index<=hist_index+1'b1;state<=MEDIAN_READ;end
     GAIN_START:begin
      div_numerator<=({24'd0,active_pose[23:16]}<<20)+(pivot_z>>1);div_quotient<=0;div_remainder<=0;div_count<=32;state<=GAIN_DIV;
     end
     GAIN_DIV:begin
      div_numerator<=div_numerator<<1;div_remainder<=quotient_bit?next_remainder-{1'b0,pivot_z}:next_remainder;div_quotient<=next_quotient;div_count<=div_count-1'b1;
      if(div_count==1)begin gain<=next_quotient[17:0];point_count<=0;scan_u<=0;scan_v<=0;measure_pass<=0;state<=START_SCAN;end
     end
     WAIT_POINT:begin
      if(take_point&&z_valid)begin
       z<=incoming_z;rgb<=point_data[15:0];saved_u<=scan_u;saved_v<=scan_v;
       ma<=($signed({1'b0,scan_u,4'd0})-28'sd10215);mb<=$signed({1'b0,incoming_z});state<=MX;
      end
      if((model_done||stream_finished)&&!take_point)begin
       if(point_count!=921600)begin fault<=1;state<=FAILED;end else state<=FLUSH;
      end
     end
     MX:begin xc<=product[27:0];ma<=$signed({1'b0,saved_v,4'd0})-28'sd5620;mb<=$signed({1'b0,z});state<=MY;end
     MY:begin yc<=product[27:0];ma<=$signed({1'b0,z})-$signed({1'b0,pivot_z});mb<=18'sd29929;state<=MZ;end
     MZ:begin zc<=product[27:0];ma<=xc;mb<=cyaw;state<=XYC;end
     XYC:begin tmp<=rot_product;ma<=zc;mb<=syaw;state<=XZS;end
     XZS:begin rx<=$signed(rotation_sum);ma<=zc;mb<=cyaw;state<=ZZC;end
     ZZC:begin tmp<=rot_product;ma<=xc;mb<=syaw;state<=ZXS;end
     ZXS:begin rz<=$signed(rotation_sum);ma<=yc;mb<=cpitch;state<=YYC;end
     YYC:begin tmp<=rot_product;ma<=rz;mb<=-spitch;state<=YZS;end
     YZS:begin ry<=$signed(rotation_sum);ma<=rz;mb<=cpitch;state<=ZZP;end
     ZZP:begin tmp<=rot_product;ma<=yc;mb<=spitch;state<=ZYS;end
     ZYS:begin rotated_z<=$signed(rotation_sum);ma<=rx;mb<=$signed(gain);state<=SX;end
     SX:begin screen_x<=screen_sum;ma<=ry;mb<=$signed(gain);state<=SY;end
     SY:begin screen_y<=screen_sum;state<=PROJECT;end
     PROJECT:if(screen_x<0||screen_x>=1280||screen_y<0||screen_y>=720)state<=WAIT_POINT;else state<=DRAW;
     DRAW:if(draw_ready)begin projected_points<=projected_points+1'b1;state<=WAIT_POINT;end
     FLUSH:if(frame_flush_done&&!publish_busy)state<=ACK;
     ACK:if(!publish_busy&&pub_ack=={source_job,back_bank})begin
      rendered_frames<=rendered_frames+1'b1;view_applied_token<=active_pose[40:25];orbit_age<=0;state<=IDLE;
     end
     FAILED:begin end
     default:begin fault<=1;state<=FAILED;end
    endcase
   end
  end
 end
endmodule

