`timescale 1ns/1ps
// K4 requests run/pause. Starting first drains stereo acquisition; ordinary
// pause uses the servo's hold-orientation transition. K3/link loss releases
// through the servo's independent stop input and clears the requested run.
module follow_mode_gate #(parameter CLK_HZ=50000000)(
 input wire clk,rst_n,toggle,stop_n,explicit_stop,link_ok,
 input wire memory_quiet,native_mode_applied,servo_armed,
 output reg requested_run,output reg servo_toggle
);
 localparam TICKS=CLK_HZ/1000;
 reg[$clog2(TICKS)-1:0]ms_count;
 reg[3:0]quiet_ms;reg command_pending,command_value;
 wire tick=ms_count==TICKS-1;
 wire ready=memory_quiet&&native_mode_applied;
 always @(posedge clk)begin
  if(!rst_n)begin ms_count<=0;quiet_ms<=0;command_pending<=0;command_value<=0;requested_run<=0;servo_toggle<=0;end
  else begin
   servo_toggle<=0;
   if(tick)ms_count<=0;else ms_count<=ms_count+1'b1;
   if(!requested_run||!ready)quiet_ms<=0;
   else if(tick&&quiet_ms<10)quiet_ms<=quiet_ms+1'b1;
   if(command_pending&&servo_armed==command_value)command_pending<=0;
   if(!command_pending&&servo_armed!=requested_run&&(!requested_run||quiet_ms==10))begin
    servo_toggle<=1;command_pending<=1;command_value<=requested_run;
   end
   if(toggle)begin requested_run<=!requested_run;quiet_ms<=0;end
   if(!stop_n||explicit_stop||!link_ok)begin requested_run<=0;quiet_ms<=0;command_pending<=0;servo_toggle<=0;end
  end
 end
endmodule
