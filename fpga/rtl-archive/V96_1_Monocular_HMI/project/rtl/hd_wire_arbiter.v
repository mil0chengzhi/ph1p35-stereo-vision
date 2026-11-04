`timescale 1ns/1ps
// Gate both legacy admissions before granting HD. Existing legacy ownership
// covers each complete native/control frame and its gap. No preemption.
module hd_wire_arbiter(
 input wire clk,rst_n,enabled,legacy_busy,input wire[7:0]legacy_data,input wire legacy_enable,output wire legacy_gate,
 input wire hd_offer,hd_busy,input wire[7:0]hd_data,input wire hd_enable,output wire hd_accept,
 output wire[7:0]gmii_data,output wire gmii_enable
);
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]reset_pipe,ready_sync;
 wire local_reset_n=reset_pipe[1];
 always @(posedge clk or negedge rst_n)
  if(!rst_n)reset_pipe<=0;else reset_pipe<={reset_pipe[0],1'b1};
 always @(posedge clk or negedge local_reset_n)
  if(!local_reset_n)ready_sync<=0;else ready_sync<={ready_sync[0],enabled};
 reg hd_active;
 assign legacy_gate=local_reset_n&&ready_sync[1]&&!hd_active&&!hd_busy&&!hd_offer;
 assign hd_accept=local_reset_n&&ready_sync[1]&&!hd_active&&!hd_busy&&!legacy_busy&&hd_offer;
 assign gmii_data=hd_active?hd_data:legacy_data;
 assign gmii_enable=local_reset_n&&ready_sync[1]&&(hd_active?hd_enable:legacy_enable);
 always @(posedge clk or negedge local_reset_n)begin
  if(!local_reset_n)hd_active<=0;
  else if(hd_accept)hd_active<=1;
  else if(hd_active&&!hd_busy)hd_active<=0;
 end
 // synthesis translate_off
 always @(posedge clk)if(local_reset_n)begin
  if(hd_busy&&legacy_busy)$fatal(1,"HD and native/control MACs simultaneously own wire");
  if(hd_enable&&!hd_active)$fatal(1,"HD output without ownership");
 end
 // synthesis translate_on
endmodule
