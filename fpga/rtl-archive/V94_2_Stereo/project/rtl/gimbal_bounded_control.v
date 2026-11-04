`timescale 1ns/1ps
// Relative, command-count-limited visual servo. No encoder/absolute-angle claim.
// UART sessions require a host heartbeat; K4 explicitly owns a local session.
// Local sessions require continuously advancing real source frames instead.
// Fault/stop finishes any existing
// STEP high/low phase, then sleeps both outputs. Both drivers hold during an explicitly warmed control session.
// Optional continuous K4 mode has no pulse/time quota. UART commissioning
// keeps its finite session bounds. Cumulative statistics
// and relative positions never reset on a key press. UART commissioning keeps
// its original MAX_TOTAL guard. K4 pause keeps both axes held; K3/STOP sleeps.
module gimbal_bounded_control #(
 parameter CLK_HZ=50000000,STEP_HZ=100,SETTLE_MS=200,MAX_BATCH=16,LOCAL_KEY_ENABLE=0,ENFORCE_POSITION_LIMIT=1,FAST_FEATURES=0,CONTINUOUS_KEY_FOLLOW=0,CONTINUOUS_LOCAL_ENABLE=0,RESPONSIVE_LOCAL_ENABLE=0,POSE_TOLERANT_ENABLE=0,
 parameter FAST_DEFAULT_PAN_GAIN=1,FAST_DEFAULT_RATE=0,FAST_DEFAULT_SETTLE=200,FAST_DEFAULT_BATCH=16,
 parameter LOCAL_SESSION_STEPS=96,LOCAL_SESSION_MS=30000,
 parameter WAKE_MS=300,MAX_WAKE_SHIFT=64,LEASE_MS=600,TARGET_MS=300,MAX_POSITION=96,MAX_TOTAL=384,
 parameter INITIAL_POSITION1=0,INITIAL_POSITION2=0,INITIAL_PULSES1=0,INITIAL_PULSES2=0,INITIAL_TOTAL=0,INITIAL_INVERT=0
)(
 input wire clk,rst_n,link_ok,stop_n,local_toggle,
 input wire write,input wire[15:0]address,input wire[7:0]data,
 input wire[199:0]target,
 output wire step1,dir1,en1,step2,dir2,en2,output wire[1:0]colour_select,
 output wire[511:0]status,output wire[31:0]session_status,output wire[31:0]tuning_status,
 output wire[287:0]extended_status,output wire statistics_saturated
);
 localparam MS_TICKS=CLK_HZ/1000;
 reg[31:0]ms_counter;wire millisecond=ms_counter==MS_TICKS-1;
 reg wake_pending;reg[15:0]wake_cx,wake_cy;
 reg armed,auto_run,holding,centered;reg hold_axis;reg enable1,enable2;
 reg local_mode,local_paused;reg[2:0]local_warm_frames;
 reg[15:0]session_steps,wake_elapsed;reg[31:0]session_milliseconds;
 wire[15:0]session_elapsed=|session_milliseconds[31:16]?16'hffff:session_milliseconds[15:0];
 wire continuous_local_capable=CONTINUOUS_LOCAL_ENABLE&&LOCAL_KEY_ENABLE&&FAST_FEATURES&&CONTINUOUS_KEY_FOLLOW;
 wire continuous_local_active=continuous_local_capable&&local_mode;
 reg[1:0]rate_profile,pan_gain_profile,tilt_gain_profile;reg[2:0]feedback_frames;reg[7:0]settle_cfg;reg[5:0]batch_limit;
 assign tuning_status={3'd0,pan_gain_profile,feedback_frames,tilt_gain_profile,batch_limit,settle_cfg,6'd0,rate_profile};
 assign session_status={session_elapsed,session_steps};
 reg[3:0]fault;reg[1:0]invert,selected_colour;reg[7:0]deadband;
 assign colour_select=selected_colour;
 reg[15:0]lease_age,target_age,cooldown;
 reg[15:0]last_sequence;reg[31:0]last_frame,last_epoch;
 reg[15:0]cx,cy;reg target_good;reg[2:0]acquired,center_frames;
 // Lifetime statistics never participate in continuous local authorization.
 // Wider counters saturate instead of wrapping. Legacy fields also saturate.
 reg signed[31:0]position1_full,position2_full;
 reg[63:0]total_steps_full,pulses1_full,pulses2_full;
 wire signed[15:0]position1=position1_full>32'sd32767 ? 16'h7fff:position1_full< -32'sd32768 ? 16'h8000:position1_full[15:0];
 wire signed[15:0]position2=position2_full>32'sd32767 ? 16'h7fff:position2_full< -32'sd32768 ? 16'h8000:position2_full[15:0];
 wire[15:0]total_steps=|total_steps_full[63:16]?16'hffff:total_steps_full[15:0];
 wire[31:0]pulses1=|pulses1_full[63:32]?32'hffffffff:pulses1_full[31:0];
 wire[31:0]pulses2=|pulses2_full[63:32]?32'hffffffff:pulses2_full[31:0];
 reg[31:0]accepted,rejected;
 assign extended_status={session_milliseconds,position2_full,position1_full,pulses2_full,pulses1_full,total_steps_full};
 assign statistics_saturated=(&total_steps_full)||(&pulses1_full)||(&pulses2_full)||
  position1_full==32'sh7fffffff||position1_full==32'sh80000000||position2_full==32'sh7fffffff||position2_full==32'sh80000000;
 reg start1,start2,direction1,direction2,old_step1,old_step2;
 wire busy1,busy2,done1,done2;
 reg[5:0]batch_remaining;reg batch_axis,batch_direction;
 // Only explicit local AUTO uses the ramp. Commissioning waveforms retain
 // their existing API. First/last4 pulses run at200Hz; an800Hz request also
 // traverses400Hz for4 pulses at either end. Short batches stay at200Hz.
 reg[5:0]ramp_total;
 wire[5:0]ramp_issued=ramp_total-batch_remaining;
 wire ramp_active=RESPONSIVE_LOCAL_ENABLE&&CONTINUOUS_KEY_FOLLOW&&FAST_FEATURES&&local_mode&&auto_run&&rate_profile>1;
 wire[1:0]motion_rate=!ramp_active?rate_profile:
  (ramp_issued<=4||batch_remaining<4)?2'd1:
  (rate_profile==3&&(ramp_issued<=8||batch_remaining<8))?2'd2:rate_profile;
 wire physical_busy=busy1||busy2||start1||start2;
 wire any_busy=physical_busy||(batch_remaining!=0);
 wire stop_write=write&&address==16'h0401&&data==0;
 wire[15:0]box_w=target[143:128]-target[127:112]+1'b1;
 wire[15:0]box_h=target[175:160]-target[159:144]+1'b1;
 // Permit a 16px peak-to-peak fluctuation around the acquired strict
 // size, frozen while armed. Acquisition stays strict; an unsupported cap,
 // partial sleeve occlusion, merged far region or source loss still stops now.
 reg[7:0]strict_width,strict_height;reg strict_shape_known;
 wire size_ok=target[143:128]>=target[127:112]&&target[175:160]>=target[159:144]&&
  box_w>=24&&box_h>=24&&box_w<=192&&box_h<=192;
 // A cube naturally exposes two faces while it moves. The measured104x64
 // box is valid, but the previous1.5:1 acquisition rejected it. This
 // optional2:1 envelope retains size, ROI, component, source and quality
 // gates; narrow caps/strips outside2:1 remain ineligible.
 wire strict_aspect=POSE_TOLERANT_ENABLE?
  ({1'b0,box_w}<=({1'b0,box_h}<<1)&&{1'b0,box_h}<=({1'b0,box_w}<<1)):
  (({1'b0,box_w}<<1)<=({1'b0,box_h}+(box_h<<1))&&
   ({1'b0,box_h}<<1)<=({1'b0,box_w}+(box_w<<1)));
 wire strict_box_ok=size_ok&&strict_aspect;
 wire[15:0]shape_dx=target[95:80]>cx?target[95:80]-cx:cx-target[95:80];
 wire[15:0]shape_dy=target[111:96]>cy?target[111:96]-cy:cy-target[111:96];
 wire near_strict_size=box_w+17'd16>={9'd0,strict_width}&&box_w<={9'd0,strict_width}+17'd16&&
  box_h+17'd16>={9'd0,strict_height}&&box_h<={9'd0,strict_height}+17'd16;
 wire quantized_aspect=({1'b0,box_w}<<1)<=({1'b0,box_h}+(box_h<<1)+17'd16)&&
  ({1'b0,box_h}<<1)<=({1'b0,box_w}+(box_w<<1)+17'd16);
 wire box_ok=strict_box_ok||(size_ok&&armed&&target_good&&acquired>=3&&strict_shape_known&&
  near_strict_size&&shape_dx<=8&&shape_dy<=8&&quantized_aspect);
 wire live_target=box_ok&&target[198:197]==selected_colour&&target[196]&&target[191]&&target[190]&&target[195:192]==0&&
                  target[95:80]<1280&&target[111:96]<720;
 wire[31:0]incoming_advance=target[47:16]-last_frame;
 wire incoming_new_frame=target[79:48]!=last_epoch||(incoming_advance!=0&&!incoming_advance[31]);
 wire heartbeat=write&&address==16'h0402&&data==8'hc3;
 wire key_event=LOCAL_KEY_ENABLE&&(local_toggle||(FAST_FEATURES&&write&&address==16'h0407&&data==8'h4b));
 wire key_stop=key_event&&armed;
 wire lease_expired=!local_mode&&lease_age>=LEASE_MS;
 wire local_timeout=!continuous_local_active&&(local_mode||FAST_FEATURES)&&armed&&session_elapsed>=LOCAL_SESSION_MS;
 wire local_exhausted=!continuous_local_active&&(local_mode||FAST_FEATURES)&&armed&&session_steps>=LOCAL_SESSION_STEPS;
 // Unreliable vision stops STEP immediately. Keep coil torque for at most
 //120ms while source-valid packets reacquire; three new valid frames are
 //needed before another batch. A source/link/epoch/K3 failure still stops.
 wire quality_pause=FAST_FEATURES&&armed&&!wake_pending&&target[196]&&target[190]&&
  target[198:197]==selected_colour&&target_age<120;
 wire halt=!armed||fault!=0||!link_ok||!stop_n||stop_write||key_stop||
           lease_expired||local_timeout||local_exhausted||target_age>=TARGET_MS||!live_target||!target_good;
 wire[15:0]ex=cx>640?cx-16'd640:16'd640-cx;
 wire[15:0]ey=cy>360?cy-16'd360:16'd360-cy;
 wire[16:0]error_sum={1'b0,ex}+{1'b0,ey};
 wire[15:0]incoming_ex=target[95:80]>640?target[95:80]-16'd640:16'd640-target[95:80];
 wire[15:0]incoming_ey=target[111:96]>360?target[111:96]-16'd360:16'd360-target[111:96];
 wire[16:0]incoming_error={1'b0,incoming_ex}+{1'b0,incoming_ey};
 // A moving object can increase image error despite correct motor motion.
 // Only explicit local follow omits the static monotonic-error assumption.
 // Host commissioning, source/target freshness, K3, pulse/session bounds
 // and cumulative ledgers keep their original behavior.
 wire continuous_key_follow=CONTINUOUS_KEY_FOLLOW&&FAST_FEATURES&&local_mode;
 reg[31:0]step_frame;reg waiting_response;reg[16:0]error_before_step;
 reg[5:0]progress_steps;reg[16:0]progress_error;
 wire legacy_request_axis=holding?(hold_axis?(ey>deadband):(ex<=deadband)):(ey>ex); // Finish the held axis before switching.
 wire request_axis=FAST_FEATURES?({2'd0,ey}+({2'd0,ey}<<1)>({2'd0,ex}<<2)):legacy_request_axis;
 wire[31:0]response_advance=target[47:16]-step_frame;
 wire response_ready=FAST_FEATURES?(response_advance>={29'd0,feedback_frames}&&!response_advance[31]):target[47:16]!=step_frame;
 wire request_direction=request_axis?((cy>360)^invert[1]):((cx>640)^invert[0]);
 wire cancel_batch=RESPONSIVE_LOCAL_ENABLE&&continuous_key_follow&&auto_run&&batch_remaining!=0&&
  target[15:0]!=last_sequence&&incoming_new_frame&&live_target&&
  ((batch_axis?incoming_ey:incoming_ex)<=deadband||
   (batch_axis?((target[111:96]>360)^invert[1]):((target[95:80]>640)^invert[0]))!=batch_direction);
 function can_step;
  input axis,positive;
  reg signed[15:0]p;
  begin p=axis?position2:position1;
   can_step=(continuous_local_active||((local_mode||FAST_FEATURES)?(total_steps<65535&&session_steps<LOCAL_SESSION_STEPS):total_steps<MAX_TOTAL))&&
            (!ENFORCE_POSITION_LIMIT||(positive?p<MAX_POSITION:p>-MAX_POSITION));
  end
 endfunction
 // Use the independently measured2.62/3.96px responses conservatively as
 //3/4px per pulse. Leave4px beyond the deadband, then cap each open-loop
 // batch. Constant comparisons use no runtime divider or additional DSP.
 function[5:0] batch_size;
  input axis;input[15:0]error;input[7:0]band;input[1:0]gain;input[5:0]limit;
  integer i;reg[16:0]margin;reg[16:0]threshold;
  begin
   batch_size=1; margin=0;threshold=0;
   if(FAST_FEATURES)begin
    if(error>(band>>1))margin={1'b0,error}-(band>>1);
    for(i=1;i<=MAX_BATCH;i=i+1)begin
     case(gain)
      0:threshold=axis?i*6:i*4;
      1:threshold=axis?i*7:i*5;
      2:threshold=axis?i*8:i*6;
      3:threshold=axis?i*9:i*7;
     endcase
     if(i<=limit&&(margin<<1)>=threshold)batch_size=i;
    end
   end else begin
    if({1'b0,error}>{9'd0,band}+17'd4)margin={1'b0,error}-{9'd0,band}-17'd4;
    for(i=1;i<=MAX_BATCH;i=i+1)if(margin>=(axis?i*4:i*3))batch_size=i;
   end
  end
 endfunction
 wire[5:0]planned_batch=batch_size(request_axis,request_axis?ey:ex,deadband,request_axis?tilt_gain_profile:pan_gain_profile,batch_limit);
 function can_batch;
  input axis,positive;input[5:0]count;
  reg signed[16:0]p,next_p;
  begin
   p=axis?{position2[15],position2}:{position1[15],position1};
   next_p=positive?p+$signed({11'd0,count}):p-$signed({11'd0,count});
   can_batch=(continuous_local_active||((local_mode||FAST_FEATURES)?(total_steps+17'd0+count<=65535&&session_steps+17'd0+count<=LOCAL_SESSION_STEPS):
             total_steps+17'd0+count<=MAX_TOTAL))&&(!ENFORCE_POSITION_LIMIT||(next_p<=MAX_POSITION&&next_p>=-MAX_POSITION));
  end
 endfunction
 // Physical DIR=1 increments relative pulse position; it is NOT camera angle.
 assign en1=enable1;assign en2=enable2;
 always @(posedge clk)begin
  if(!rst_n)begin enable1<=0;enable2<=0;end
  else begin
   enable1<=holding||busy1||start1;
   enable2<=holding||busy2||start2;
  end
 end
 d36a_step_burst #(.CLK_HZ(CLK_HZ),.STEP_HZ(STEP_HZ),.PULSES(1),.RUNTIME_RATE_ENABLE(FAST_FEATURES),.AWAKE_BYPASS_ENABLE(FAST_FEATURES)) axis1(
  .clk(clk),.rst_n(rst_n),.start(start1),.direction(direction1),.halt(halt),.rate_select(motion_rate),.awake_ready(holding&&!wake_pending&&enable1&&enable2),
  .step(step1),.dir(dir1),.en(),.busy(busy1),.done(done1));
 d36a_step_burst #(.CLK_HZ(CLK_HZ),.STEP_HZ(STEP_HZ),.PULSES(1),.RUNTIME_RATE_ENABLE(FAST_FEATURES),.AWAKE_BYPASS_ENABLE(FAST_FEATURES)) axis2(
  .clk(clk),.rst_n(rst_n),.start(start2),.direction(direction2),.halt(halt),.rate_select(motion_rate),.awake_ready(holding&&!wake_pending&&enable1&&enable2),
  .step(step2),.dir(dir2),.en(),.busy(busy2),.done(done2));
 always @(posedge clk)begin
  if(!rst_n)begin
   wake_pending<=0;wake_cx<=0;wake_cy<=0;wake_elapsed<=0;rate_profile<=FAST_DEFAULT_RATE;feedback_frames<=4;pan_gain_profile<=FAST_DEFAULT_PAN_GAIN;tilt_gain_profile<=2;settle_cfg<=FAST_FEATURES?FAST_DEFAULT_SETTLE:SETTLE_MS;batch_limit<=FAST_DEFAULT_BATCH;
   ms_counter<=0;armed<=0;auto_run<=0;holding<=0;hold_axis<=0;centered<=0;fault<=0;
   local_mode<=0;local_paused<=0;local_warm_frames<=0;session_steps<=0;session_milliseconds<=0;
   strict_width<=0;strict_height<=0;strict_shape_known<=0;
   selected_colour<=0;invert<=INITIAL_INVERT;deadband<=12;lease_age<=65535;target_age<=65535;cooldown<=0;
   last_sequence<=0;last_frame<=0;last_epoch<=0;cx<=0;cy<=0;target_good<=0;
   acquired<=0;center_frames<=0;position1_full<=INITIAL_POSITION1;position2_full<=INITIAL_POSITION2;total_steps_full<=INITIAL_TOTAL;
   pulses1_full<=INITIAL_PULSES1;pulses2_full<=INITIAL_PULSES2;accepted<=0;rejected<=0;
   start1<=0;start2<=0;direction1<=0;direction2<=0;old_step1<=0;old_step2<=0;
   step_frame<=0;waiting_response<=0;error_before_step<=0;progress_steps<=0;progress_error<=0;
   batch_remaining<=0;batch_axis<=0;batch_direction<=0;ramp_total<=0;
  end else begin
   start1<=0;start2<=0;old_step1<=step1;old_step2<=step2;
   if(wake_pending&&cooldown==0&&(!FAST_FEATURES||(target_good&&live_target&&acquired>=3&&local_warm_frames>=3&&target_age<100)))wake_pending<=0;
   if(!armed&&!any_busy&&!local_paused)begin holding<=0;local_mode<=0;end
   // Reaching a local allowance ends this session while retaining pose.
   // It can be explicitly resumed by K4 with a new allowance and fresh target.
   if(local_timeout||local_exhausted)begin
    fault<=local_timeout?12:11;armed<=0;auto_run<=0;local_paused<=local_mode;
    batch_remaining<=0;wake_pending<=0;waiting_response<=0;start1<=0;start2<=0;
   end
   if(millisecond)begin
    ms_counter<=0;
    if(lease_age!=65535)lease_age<=lease_age+1'b1;
    if(target_age!=65535)target_age<=target_age+1'b1;
    if(cooldown!=0)cooldown<=cooldown-1'b1;
    if(wake_pending&&wake_elapsed!=65535)wake_elapsed<=wake_elapsed+1'b1;
    if((local_mode||FAST_FEATURES)&&armed&&!(&session_milliseconds))session_milliseconds<=session_milliseconds+1'b1;
   end else ms_counter<=ms_counter+1'b1;
   if(heartbeat)lease_age<=0;
   if(step1&&!old_step1)begin
    if((local_mode||FAST_FEATURES)&&!(&session_steps))session_steps<=session_steps+1'b1;
    if(dir1&&position1_full!=32'sh7fffffff)position1_full<=position1_full+1'b1;
    else if(!dir1&&position1_full!=32'sh80000000)position1_full<=position1_full-1'b1;
    if(!(&pulses1_full))pulses1_full<=pulses1_full+1'b1;
    if(!(&total_steps_full))total_steps_full<=total_steps_full+1'b1;cooldown<=FAST_FEATURES?settle_cfg:SETTLE_MS;step_frame<=last_frame;
   end
   if(step2&&!old_step2)begin
    if((local_mode||FAST_FEATURES)&&!(&session_steps))session_steps<=session_steps+1'b1;
    if(dir2&&position2_full!=32'sh7fffffff)position2_full<=position2_full+1'b1;
    else if(!dir2&&position2_full!=32'sh80000000)position2_full<=position2_full-1'b1;
    if(!(&pulses2_full))pulses2_full<=pulses2_full+1'b1;
    if(!(&total_steps_full))total_steps_full<=total_steps_full+1'b1;cooldown<=FAST_FEATURES?settle_cfg:SETTLE_MS;step_frame<=last_frame;
   end
   if(FAST_FEATURES&&armed&&(!live_target||!target_good))begin
    centered<=0;center_frames<=0;waiting_response<=0;progress_steps<=0;progress_error<=error_sum;
   end
   if(!live_target)begin target_good<=0;acquired<=0;center_frames<=0;if(FAST_FEATURES&&wake_pending)local_warm_frames<=0;end
   if(target[15:0]!=last_sequence)begin
    last_sequence<=target[15:0];
    if(live_target&&incoming_new_frame)begin
     if(strict_box_ok&&!armed)begin strict_width<=box_w[7:0];strict_height<=box_h[7:0];strict_shape_known<=1;end
     if(armed&&target[79:48]!=last_epoch)begin fault<=5;armed<=0;auto_run<=0;end
     if(target[79:48]!=last_epoch)acquired<=1;
     else if(acquired<7)acquired<=acquired+1'b1;
     target_age<=0;target_good<=1;last_frame<=target[47:16];last_epoch<=target[79:48];
     cx<=target[95:80];cy<=target[111:96];
     if((local_mode||FAST_FEATURES)&&wake_pending&&local_warm_frames<7)local_warm_frames<=local_warm_frames+1'b1;
     if(auto_run&&cooldown==0&&!any_busy)begin
      if(incoming_ex<=deadband&&incoming_ey<=deadband)begin
       if(center_frames<7)center_frames<=center_frames+1'b1;
       if(center_frames>=2)begin
        centered<=1;if(!local_mode)auto_run<=0;waiting_response<=0;
        // A later moving target begins a new response observation window.
        if(local_mode)begin progress_steps<=0;progress_error<=incoming_error;end
       end
      end else begin
       center_frames<=0;centered<=0;
       if(local_mode&&centered)begin progress_steps<=0;progress_error<=incoming_error;end
      end
      if(waiting_response&&response_ready)begin
       waiting_response<=0;
       if(!continuous_key_follow&&incoming_error>error_before_step+17'd16)begin fault<=8;auto_run<=0;armed<=0;end
      end
     end
    end
   end
   // All jogs/autosteps pass the same hardware limits and freshness guards.
   if(write&&address==16'h0405)begin
    if(!armed&&!auto_run&&!any_busy&&data[7:2]==0)begin
     selected_colour<=data[1:0];strict_shape_known<=0;target_good<=0;acquired<=0;center_frames<=0;centered<=0;
     accepted<=accepted+1'b1;
    end else rejected<=rejected+1'b1;
   end
   if(write&&address==16'h0403)begin
    if(!local_mode&&!auto_run&&!any_busy&&data[7:2]==0)invert<=data[1:0];else rejected<=rejected+1'b1;
   end
   if(write&&address==16'h0406)begin
    if(!local_mode&&!auto_run&&!any_busy&&data>=8&&data<=32)deadband<=data;else rejected<=rejected+1'b1;
   end
   if(FAST_FEATURES&&write&&address>=16'h0408&&address<=16'h040d)begin
    if(armed||any_busy)rejected<=rejected+1'b1;
    else case(address)
     16'h0408:if(data<=3)rate_profile<=data[1:0];else rejected<=rejected+1'b1;
     16'h0409:if(data==34||data==67||data==100||data==200)settle_cfg<=data;else rejected<=rejected+1'b1;
     16'h040a:if(data==4||data==8||data==16||data==32)batch_limit<=data[5:0];else rejected<=rejected+1'b1;
     16'h040b:if(data>=2&&data<=6)feedback_frames<=data[2:0];else rejected<=rejected+1'b1;
     16'h040c:if(data<=3)pan_gain_profile<=data[1:0];else rejected<=rejected+1'b1;
     16'h040d:if(data<=3)tilt_gain_profile<=data[1:0];else rejected<=rejected+1'b1;
    endcase
   end
   // After physical warmup, only fresh video authorizes local automatic steps.
   // This is an explicit mode, not a synthetic UART heartbeat generator.
   if(local_mode&&armed&&!auto_run&&!halt&&holding&&!wake_pending&&
      cooldown==0&&local_warm_frames>=3&&acquired>=3&&!any_busy)begin
    auto_run<=1;centered<=0;center_frames<=0;waiting_response<=0;
    progress_steps<=0;progress_error<=error_sum;
   end
   if(write&&address==16'h0401)begin
    if(local_mode&&data!=0&&!(FAST_FEATURES&&local_paused&&(data==8'ha5||((fault==11||fault==12)&&data==8'h96))))rejected<=rejected+1'b1;
    else case(data)
     8'h00:begin local_mode<=0;local_paused<=0;batch_remaining<=0;wake_pending<=0;armed<=0;auto_run<=0;centered<=0;waiting_response<=0;accepted<=accepted+1'b1;end
     8'h96:if(!armed&&!any_busy)begin fault<=0;centered<=0;accepted<=accepted+1'b1;end else rejected<=rejected+1'b1;
     8'ha5:if(!armed&&fault==0&&!any_busy&&stop_n&&link_ok&&target_good&&live_target&&
              acquired>=3&&target_age<100&&lease_age<100&&ex<=256&&ey<=240&&(FAST_FEATURES?total_steps<65535:total_steps<MAX_TOTAL))begin
       armed<=1;auto_run<=0;centered<=0;cooldown<=20;accepted<=accepted+1'b1;
       if(FAST_FEATURES)begin local_mode<=0;local_paused<=0;session_steps<=0;session_milliseconds<=0;end
      end else rejected<=rejected+1'b1;
     // C1 wakes both SLEEP# pins without a STEP. The host must continue its
     // lease, wait for the warmup and inspect fresh, stable image measurements.
     8'hc1:if(armed&&!auto_run&&!any_busy&&!halt&&!holding)begin
       holding<=1;hold_axis<=0;cooldown<=WAKE_MS;wake_pending<=1;
       wake_cx<=cx;wake_cy<=cy;wake_elapsed<=0;local_warm_frames<=0;accepted<=accepted+1'b1;
      end else rejected<=rejected+1'b1;
     8'h5a:if(armed&&!halt&&!any_busy&&acquired>=3&&holding&&!wake_pending&&cooldown==0)begin
       auto_run<=1;centered<=0;center_frames<=0;waiting_response<=0;
       progress_steps<=0;progress_error<=error_sum;accepted<=accepted+1'b1;
      end else rejected<=rejected+1'b1;
     default:rejected<=rejected+1'b1;
    endcase
   end
   if(write&&address==16'h0404)begin
    if(data[7:2]!=0||!armed||auto_run||halt||any_busy||cooldown!=0||!holding||wake_pending)rejected<=rejected+1'b1;
    else if(!can_step(data[1],data[0]))begin fault<=6;armed<=0;rejected<=rejected+1'b1;end
    else begin
     holding<=1;hold_axis<=data[1];
     if(data[1])begin direction2<=data[0];start2<=1;end
     else begin direction1<=data[0];start1<=1;end
     cooldown<=FAST_FEATURES?settle_cfg:SETTLE_MS;step_frame<=last_frame;accepted<=accepted+1'b1;
    end
   end
   // Complete the bounded batch at the unchanged single-pulse waveform.
   // Every subsequent pulse still checks live target, lease and exact ledger.
   if(auto_run&&!halt&&(!FAST_FEATURES||acquired>=3)&&!physical_busy&&batch_remaining!=0&&!cancel_batch)begin
    if(!can_step(batch_axis,batch_direction))begin batch_remaining<=0;fault<=6;armed<=0;auto_run<=0;end
    else begin
     batch_remaining<=batch_remaining-1'b1;
     if(batch_axis)begin direction2<=batch_direction;start2<=1;end
     else begin direction1<=batch_direction;start1<=1;end
    end
   end
   if(auto_run&&!halt&&(!FAST_FEATURES||acquired>=3)&&!any_busy&&cooldown==0&&!waiting_response&&
      (ex>deadband||ey>deadband)&&last_frame!=step_frame)begin
    if(!can_batch(request_axis,request_direction,planned_batch))begin
     armed<=0;auto_run<=0;
     if((local_mode||FAST_FEATURES)&&total_steps+17'd0+planned_batch>65535)begin fault<=13;local_paused<=0;end
     else if((local_mode||FAST_FEATURES)&&session_steps+17'd0+planned_batch>LOCAL_SESSION_STEPS)begin
      fault<=11;local_paused<=local_mode;batch_remaining<=0;wake_pending<=0;waiting_response<=0;
     end else begin fault<=6;local_paused<=0;end
    end
    else if(!continuous_key_follow&&progress_steps>=8&&error_sum+17'd4>=progress_error)begin fault<=9;armed<=0;auto_run<=0;end
    else begin
     if(progress_steps>=8)begin progress_steps<=planned_batch;progress_error<=error_sum;end
     else progress_steps<=progress_steps+planned_batch;
     batch_remaining<=planned_batch-1'b1;batch_axis<=request_axis;batch_direction<=request_direction;ramp_total<=planned_batch;
     holding<=1;hold_axis<=request_axis;
     if(request_axis)begin direction2<=request_direction;start2<=1;end
     else begin direction1<=request_direction;start1<=1;end
     cooldown<=FAST_FEATURES?settle_cfg:SETTLE_MS;step_frame<=last_frame;waiting_response<=1;error_before_step<=error_sum;
    end
   end
   if(armed&&wake_pending&&incoming_new_frame&&(!FAST_FEATURES||(live_target&&target_good))&&
     ((target[95:80]>wake_cx?target[95:80]-wake_cx:wake_cx-target[95:80])>MAX_WAKE_SHIFT||
      (target[111:96]>wake_cy?target[111:96]-wake_cy:wake_cy-target[111:96])>MAX_WAKE_SHIFT))begin
    fault<=10;armed<=0;auto_run<=0;start1<=0;start2<=0;
   end
   // Physical stop, absent source, expiry and link loss override all starts.
   if(armed&&(!stop_n||!link_ok||lease_expired||(!(FAST_FEATURES&&wake_pending)&&(target_age>=TARGET_MS||((!live_target||!target_good)&&!quality_pause)))))begin
    armed<=0;auto_run<=0;local_paused<=0;start1<=0;start2<=0;waiting_response<=0;
    fault<=!stop_n?1:!link_ok?2:lease_expired?3:target_age>=TARGET_MS?4:7;
   end
   if(cancel_batch)begin
    batch_remaining<=0;start1<=0;start2<=0;waiting_response<=0;
    cooldown<=FAST_FEATURES?settle_cfg:SETTLE_MS;step_frame<=target[47:16];
   end
   if(halt)begin batch_remaining<=0;start1<=0;start2<=0;end
   // One debounced K4 press stops either a UART or a local session. Starting
   // requires a released emergency stop, strict reacquisition and valid source.
   // Recoverable source/stop faults may be acknowledged by this explicit press;
   // motion/response/limit faults remain latched. No key clears the ledger.
   if(key_event)begin
    if(armed||any_busy)begin
     if((local_mode||FAST_FEATURES)&&fault==0)begin local_mode<=1;local_paused<=1;end
     else begin local_mode<=0;local_paused<=0;end
     armed<=0;auto_run<=0;centered<=0;batch_remaining<=0;
     wake_pending<=0;waiting_response<=0;start1<=0;start2<=0;accepted<=accepted+1'b1;
    end else if(!stop_write&&stop_n&&link_ok&&target_good&&live_target&&strict_box_ok&&
      acquired>=3&&target_age<100&&ex<=512&&ey<=300&&(continuous_local_capable||total_steps<65535)&&
      (fault==0||fault==1||fault==2||fault==3||fault==4||fault==5||fault==7||fault==11||fault==12||(FAST_FEATURES&&fault==14)))begin
     local_mode<=1;local_paused<=0;local_warm_frames<=0;fault<=0;armed<=1;auto_run<=0;
     session_steps<=0;session_milliseconds<=0;
     centered<=0;center_frames<=0;holding<=1;hold_axis<=0;deadband<=8;
     // A normal K4 pause keeps both coils powered. Resuming that exact
     // held state needs no repeated cold-wake delay. Fresh acquisition above
     // remains mandatory. Cold start still waits WAKE_MS and new frames.
     if(continuous_local_capable&&local_paused&&holding&&enable1&&enable2&&fault==0)begin
      wake_pending<=0;cooldown<=0;local_warm_frames<=3;
     end else begin wake_pending<=1;cooldown<=WAKE_MS;end
     wake_cx<=cx;wake_cy<=cy;wake_elapsed<=0;
     progress_steps<=0;waiting_response<=0;accepted<=accepted+1'b1;
    end else rejected<=rejected+1'b1;
   end
   if(FAST_FEATURES&&armed&&wake_pending&&wake_elapsed>=1000)begin
    fault<=14;armed<=0;auto_run<=0;local_paused<=0;local_mode<=0;wake_pending<=0;
    batch_remaining<=0;waiting_response<=0;start1<=0;start2<=0;
   end
   // Emergency stop and link loss override K4 even during a holding pause.
   if((armed||local_paused)&&(!stop_n||!link_ok))begin
    fault<=!stop_n?1:2;local_paused<=0;local_mode<=0;armed<=0;auto_run<=0;
    batch_remaining<=0;wake_pending<=0;waiting_response<=0;start1<=0;start2<=0;
   end
   if(local_paused&&fault!=0&&fault!=11&&fault!=12)begin local_paused<=0;local_mode<=0;end
   // Explicit host/emergency STOP has priority over a simultaneous allowance
   // expiry and can never turn into a holding pause.
   if(stop_write)begin
    local_paused<=0;local_mode<=0;armed<=0;auto_run<=0;centered<=0;
    batch_remaining<=0;wake_pending<=0;waiting_response<=0;start1<=0;start2<=0;
   end
  end
 end
 // Little-endian host layout; top freezes this bus before block reads.
 assign status={MAX_TOTAL[15:0],8'd0,MAX_POSITION[7:0],
  24'd0,target[199:192],9'd0,progress_steps,error_before_step,
  rejected,accepted,last_sequence,8'd0,deadband,16'd0,cooldown,
  target_age,lease_age,last_epoch,last_frame,cy,cx,pulses2,pulses1,
  total_steps,12'd0,selected_colour,invert,position2,position1,
  8'd54,8'd0,4'd0,fault,ENFORCE_POSITION_LIMIT[0],local_paused,(local_mode&&armed),centered,target_good,any_busy,auto_run,armed};
 // synthesis translate_off
 initial if(CLK_HZ<1000||MS_TICKS<1||MAX_POSITION<1||MAX_TOTAL<1||MAX_BATCH<1||MAX_BATCH>32)$fatal(1,"gimbal parameters");
 always @(posedge clk)if(rst_n)begin
  if(total_steps_full<INITIAL_TOTAL||pulses1_full<INITIAL_PULSES1||pulses2_full<INITIAL_PULSES2)$fatal(1,"carry-in counter lost");
  if((step1||step2)&&(!en1||!en2))$fatal(1,"A step occurred without both axes held");
  if((step1&&!old_step1)&&(step2&&!old_step2))$fatal(1,"simultaneous motor steps");
  if((!LOCAL_KEY_ENABLE&&!FAST_FEATURES&&total_steps>MAX_TOTAL)||(armed&&!continuous_local_active&&session_steps>LOCAL_SESSION_STEPS)||(ENFORCE_POSITION_LIMIT&&(position1>MAX_POSITION||position1< -MAX_POSITION||position2>MAX_POSITION||position2< -MAX_POSITION)))$fatal(1,"motor budget exceeded");
 end
 // synthesis translate_on
endmodule
