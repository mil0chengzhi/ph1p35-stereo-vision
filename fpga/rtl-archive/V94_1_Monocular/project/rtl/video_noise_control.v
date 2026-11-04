`timescale 1ns/1ps
// Single outstanding NR mode request, for cdc_mode WIDTH=1.
// At 50 MHz, the default consecutive-sample debounce is approximately 20 ms.
// A key held through reset must first be released for DEBOUNCE_CYCLES samples.
// Busy/cooldown events are discarded, not queued. key_count counts accepted
// debounced key requests only; valid host requests take priority over the key.
module video_noise_control #(
 parameter integer DEBOUNCE_CYCLES=1000000
)(
 input wire clk,rst_n,key_n,
 input wire host_write,input wire [7:0] host_data,
 input wire busy,applied_mode,
 output reg mode_write,mode_value,requested_mode,
 output reg [31:0] key_count
);
 localparam integer COUNT_BITS=(DEBOUNCE_CYCLES>1)?$clog2(DEBOUNCE_CYCLES):1;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] key_sync;
 reg [COUNT_BITS-1:0] stable_count;
 reg key_armed,startup_pending;
 wire key_event=key_armed&&!key_sync[1]&&(stable_count==DEBOUNCE_CYCLES-1);
 wire valid_host=host_write&&((host_data==8'd0)||(host_data==8'd1));

 always @(posedge clk or negedge rst_n)
  if(!rst_n) key_sync<=2'b00;
  else key_sync<={key_sync[0],key_n};

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin stable_count<=0;key_armed<=0;end
  else if(!key_armed) begin
   if(!key_sync[1]) stable_count<=0;
   else if(stable_count==DEBOUNCE_CYCLES-1) begin stable_count<=0;key_armed<=1;end
   else stable_count<=stable_count+1'b1;
  end else begin
   if(key_sync[1]) stable_count<=0;
   else if(stable_count==DEBOUNCE_CYCLES-1) begin stable_count<=0;key_armed<=0;end
   else stable_count<=stable_count+1'b1;
  end
 end

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   mode_write<=0;mode_value<=1;requested_mode<=1;key_count<=0;startup_pending<=1;
  end else begin
   mode_write<=0;
   // The old mode_write is the request that cdc_mode samples on THIS edge.
   // Keep this entire edge quiet so its busy update can become observable.
   if(!busy&&!mode_write) begin
    if(valid_host) begin
     mode_write<=1;mode_value<=host_data[0];requested_mode<=host_data[0];startup_pending<=0;
    end else if(key_event) begin
     mode_write<=1;mode_value<=!applied_mode;requested_mode<=!applied_mode;
     key_count<=key_count+1'b1;startup_pending<=0;
    end else if(startup_pending) begin
     mode_write<=1;mode_value<=1;requested_mode<=1;startup_pending<=0;
    end
   end
  end
 end
endmodule
