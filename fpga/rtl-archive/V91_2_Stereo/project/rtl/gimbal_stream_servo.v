`timescale 1ns/1ps
// Integrated candidate; motors remain unarmed at boot.
// Frame-driven relative visual servo; no encoder or absolute-position claim.
// K4 toggles run/pause; pause holds. K3/link failure finishes a STEP phase,
// then releases both drivers. Invalid/stale vision stops STEP and holds.
// Continuous operation has no lifetime pulse quota. Statistics saturate.
module gimbal_stream_servo #(
 parameter CLK_HZ=50000000,DEADBAND=12,TARGET_MS=300,FEEDBACK_FRAMES=3,
 parameter INITIAL_INVERT=2,
 parameter signed [31:0]INITIAL_POSITION1=0,INITIAL_POSITION2=0,
 parameter [63:0]INITIAL_PULSES1=0,INITIAL_PULSES2=0,INITIAL_TOTAL=0
)(
 input wire clk,rst_n,link_ok,stop_n,local_toggle,
 input wire write,input wire[15:0]address,input wire[7:0]data,
 input wire[199:0]target,
 output wire step1,dir1,en1,step2,dir2,en2,
 output reg[1:0]colour_select,
 output wire[255:0]statistics,output wire[127:0]information
);
 localparam MS_TICKS=CLK_HZ/1000;
 reg[$clog2(MS_TICKS)-1:0]ms_count;
 wire tick=ms_count==MS_TICKS-1;
 reg armed,holding,good,seen,waiting;
 reg[2:0]warm;reg[1:0]awake_ms;
 reg[8:0]age;
 reg[11:0]cx,cy;
 reg[31:0]last_frame,last_epoch,step_frame;
 reg[15:0]last_sequence;
 reg[5:0]remaining,batch_total;
 reg axis,direction,start1,start2,old_step1,old_step2;
 reg[63:0]pulses1,pulses2,total;
 reg signed[31:0]position1,position2;
 wire busy1,busy2,done1,done2;
 wire[15:0]width=target[143:128]-target[127:112]+1'b1;
 wire[15:0]height=target[175:160]-target[159:144]+1'b1;
 wire box_good=target[143:128]>=target[127:112]&&target[175:160]>=target[159:144]&&
  width>=24&&height>=24&&width<=192&&height<=192&&
  {1'b0,width}<={height,1'b0}&&{1'b0,height}<={width,1'b0};
 wire incoming_good=target[196]&&target[191]&&target[190]&&target[195:192]==0&&
  target[198:197]==colour_select&&target[95:80]<1280&&target[111:96]<720&&box_good;
 // A held payload is not a new observation. Rebase only on a NEW message
 // after manual start or link recovery; require three fresh frames to move.
 wire incoming=target[15:0]!=last_sequence;
 wire[31:0]advance=target[47:16]-last_frame;
 wire frame_new=!seen||target[79:48]!=last_epoch||(advance!=0&&!advance[31]);
 wire toggle=local_toggle||(write&&address==16'h0407&&data==8'h4b);
 wire stop_write=write&&address==16'h0401&&data==0;
 wire colour_write=write&&address==16'h0405&&data<4;
 wire pause=toggle&&armed;
 wire halt=!armed||!good||age>=TARGET_MS||!link_ok||!stop_n||stop_write||pause||colour_write||
  (incoming&&(!incoming_good||!frame_new));
 wire[11:0]ex=cx>640?cx-12'd640:12'd640-cx;
 wire[11:0]ey=cy>360?cy-12'd360:12'd360-cy;
 wire chosen_axis=({2'd0,ey}+({2'd0,ey}<<1))>({2'd0,ex}<<2);
 wire chosen_direction=chosen_axis?((cy>360)^INITIAL_INVERT[1]):((cx>640)^INITIAL_INVERT[0]);
 wire[11:0]margin=(chosen_axis?ey:ex)-6;
 wire[5:0]planned=chosen_axis?(margin>=128?6'd32:margin[6:2]):(margin>=96?6'd32:margin[6:0]/3);
 wire[5:0]issued=batch_total-remaining;
 wire[1:0]rate=(issued<=4||remaining<=4)?2'd1:(issued<=8||remaining<=8)?2'd2:2'd3;
 wire[31:0]response_advance=last_frame-step_frame;
 wire feedback_ready=!waiting||(response_advance>=FEEDBACK_FRAMES&&!response_advance[31]);
 wire[11:0]new_ex=target[95:80]>640?target[95:80]-16'd640:16'd640-target[95:80];
 wire[11:0]new_ey=target[111:96]>360?target[111:96]-16'd360:16'd360-target[111:96];
 wire new_direction=axis?((target[111:96]>360)^INITIAL_INVERT[1]):((target[95:80]>640)^INITIAL_INVERT[0]);
 wire cancel=incoming&&incoming_good&&frame_new&&((axis?new_ey:new_ex)<=DEADBAND||new_direction!=direction);
 wire physical_busy=busy1||busy2||start1||start2;
 wire centered=ex<=DEADBAND&&ey<=DEADBAND;
 assign en1=holding||busy1||start1;
 assign en2=holding||busy2||start2;
 assign statistics={total,pulses2,pulses1,position2,position1};
 assign information={last_epoch,last_frame,last_sequence,4'd0,age,3'd0,warm,colour_select,
  1'b0,link_ok,stop_n,holding,armed,good,centered,waiting,remaining};
 d36a_step_burst #(.CLK_HZ(CLK_HZ),.STEP_HZ(100),.PULSES(1),.WAKE_US(2000),
  .RUNTIME_RATE_ENABLE(1),.AWAKE_BYPASS_ENABLE(1))axis1(
  .clk(clk),.rst_n(rst_n),.start(start1),.direction(direction),.halt(halt||cancel),
  .rate_select(rate),.awake_ready(awake_ms>=2),.step(step1),.dir(dir1),.en(),.busy(busy1),.done(done1));
 d36a_step_burst #(.CLK_HZ(CLK_HZ),.STEP_HZ(100),.PULSES(1),.WAKE_US(2000),
  .RUNTIME_RATE_ENABLE(1),.AWAKE_BYPASS_ENABLE(1))axis2(
  .clk(clk),.rst_n(rst_n),.start(start2),.direction(direction),.halt(halt||cancel),
  .rate_select(rate),.awake_ready(awake_ms>=2),.step(step2),.dir(dir2),.en(),.busy(busy2),.done(done2));
 always @(posedge clk)begin
  if(!rst_n)begin
   ms_count<=0;armed<=0;holding<=0;good<=0;seen<=0;waiting<=0;warm<=0;awake_ms<=0;age<=511;
   cx<=640;cy<=360;last_frame<=0;last_epoch<=0;step_frame<=0;last_sequence<=0;
   remaining<=0;batch_total<=0;axis<=0;direction<=0;start1<=0;start2<=0;
   old_step1<=0;old_step2<=0;colour_select<=0;
   pulses1<=INITIAL_PULSES1;pulses2<=INITIAL_PULSES2;total<=INITIAL_TOTAL;
   position1<=INITIAL_POSITION1;position2<=INITIAL_POSITION2;
  end else begin
   start1<=0;start2<=0;old_step1<=step1;old_step2<=step2;
   if(tick)begin ms_count<=0;if(age<511)age<=age+1'b1;if(holding&&awake_ms<3)awake_ms<=awake_ms+1'b1;end else ms_count<=ms_count+1'b1;
   if(!holding)awake_ms<=0;
   if((step1&&!old_step1)||(step2&&!old_step2))begin
    if(!(&total))total<=total+1'b1;step_frame<=last_frame;
   end
   if(step1&&!old_step1)begin
    if(!(&pulses1))pulses1<=pulses1+1'b1;
    if((dir1&&position1!=32'sh7fffffff)||(!dir1&&position1!=32'sh80000000))
     position1<=position1+(dir1?32'sd1:-32'sd1);
   end
   if(step2&&!old_step2)begin
    if(!(&pulses2))pulses2<=pulses2+1'b1;
    if((dir2&&position2!=32'sh7fffffff)||(!dir2&&position2!=32'sh80000000))
     position2<=position2+(dir2?32'sd1:-32'sd1);
   end
   if(incoming)begin
    seen<=1;last_sequence<=target[15:0];
    if(frame_new)begin
     last_frame<=target[47:16];last_epoch<=target[79:48];
     if(incoming_good)begin
      cx<=target[91:80];cy<=target[107:96];good<=1;age<=0;
      if(target[79:48]!=last_epoch)begin warm<=1;remaining<=0;waiting<=0;end
      else if(warm<3)warm<=warm+1'b1;
     end else begin good<=0;warm<=0;remaining<=0;end
    end else begin good<=0;warm<=0;remaining<=0;end
   end
   if(age>=TARGET_MS)begin good<=0;warm<=0;remaining<=0;end
   if(cancel)begin remaining<=0;waiting<=1;end
   if((done1||done2)&&remaining==0)waiting<=1;
   if(!halt&&!cancel&&warm>=3&&!physical_busy&&!done1&&!done2)begin
    if(remaining!=0)begin
     if(axis)start2<=1;else start1<=1;
     remaining<=remaining-1'b1;
    end else if(feedback_ready&&!centered)begin
     axis<=chosen_axis;direction<=chosen_direction;waiting<=0;
     remaining<=planned==0?6'd1:planned;batch_total<=planned==0?6'd1:planned;
    end
   end
   if(toggle)begin
    armed<=!armed;holding<=1;remaining<=0;waiting<=0;warm<=0;
    if(!armed)begin seen<=0;good<=0;age<=511;end
   end
   if(colour_write)begin colour_select<=data[1:0];good<=0;warm<=0;remaining<=0;waiting<=0;end
   if(!link_ok||!stop_n||stop_write)begin
    armed<=0;holding<=0;good<=0;warm<=0;remaining<=0;waiting<=0;
    seen<=0;last_sequence<=target[15:0];
   end
  end
 end
 // synthesis translate_off
 always @(posedge clk)if(rst_n&&step1&&step2)$fatal(1,"Both axes stepped simultaneously");
 // synthesis translate_on
endmodule
