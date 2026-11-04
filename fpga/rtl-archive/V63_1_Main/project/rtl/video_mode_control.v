`timescale 1ns/1ps
// Board50-domain UI only. KEY3 is synchronized/debounced. No motor outputs.
// Latest requested mode survives a busy CDC; dispatch happens when available.
// applied_mode is the acknowledged source-domain value from cdc_mode WIDTH3.
module video_mode_control #(parameter DEBOUNCE_CYCLES=500000,WIDTH=3)(
 input wire clk,rst_n,key_n,host_write,
 input wire [WIDTH-1:0] host_mode,applied_mode,input wire busy,
 output reg mode_write,output reg [WIDTH-1:0] mode_value,requested_mode,
 output reg [31:0] key_count
);
 localparam DBITS=DEBOUNCE_CYCLES>1?$clog2(DEBOUNCE_CYCLES):1;
 (* async_reg="true" *) reg [1:0] key_sync;
 reg stable_n;reg [DBITS-1:0] debounce;
 wire key_event=key_sync[1]!=stable_n&&debounce==DEBOUNCE_CYCLES-1&&!key_sync[1];
 always @(posedge clk)begin
  if(!rst_n)begin key_sync<=2'b11;stable_n<=1;debounce<=0;key_count<=0;
   mode_write<=0;mode_value<=0;requested_mode<=0;end
  else begin
   key_sync<={key_sync[0],key_n};mode_write<=0;
   if(key_sync[1]==stable_n)debounce<=0;
   else if(debounce==DEBOUNCE_CYCLES-1)begin stable_n<=key_sync[1];debounce<=0;end
   else debounce<=debounce+1'b1;
   if(key_event)begin key_count<=key_count+1'b1;requested_mode<=requested_mode+1'b1;end
   if(host_write)requested_mode<=host_mode;
   if(!busy&&!mode_write&&requested_mode!=applied_mode)begin mode_write<=1;mode_value<=requested_mode;end
  end
 end
 // synthesis translate_off
 initial if(DEBOUNCE_CYCLES<1)$fatal(1,"Invalid debounce interval");
 // synthesis translate_on
endmodule
