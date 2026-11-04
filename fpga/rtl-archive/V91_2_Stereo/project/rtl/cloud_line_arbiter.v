`timescale 1ns/1ps
// Two complete-line clients, one existing DDR reader. An accepted line keeps
// its owner through DONE, including disable and faults. No DDR command changes.
module cloud_line_arbiter(
 input wire clk,rst_n,
 input wire d_request,d_new,d_bank,input wire[15:0]d_row,output wire d_ready,d_write,d_done,
 input wire c_request,c_new,c_bank,input wire[15:0]c_row,output wire c_ready,c_write,c_done,
 output wire m_request,m_new,m_bank,output wire[15:0]m_row,
 input wire m_ready,m_write,m_done,output reg active,owner
);
 reg last_owner;
 wire choose_cloud=c_request&&(!d_request||!last_owner);
 assign m_request=!active&&(d_request||c_request);
 assign m_new=choose_cloud?c_new:d_new;
 assign m_row=choose_cloud?c_row:d_row;
 assign m_bank=choose_cloud?c_bank:d_bank;
 assign d_ready=!active&&!choose_cloud&&m_ready;
 assign c_ready=!active&&choose_cloud&&m_ready;
 assign d_write=active&&!owner&&m_write;assign c_write=active&&owner&&m_write;
 assign d_done=active&&!owner&&m_done;assign c_done=active&&owner&&m_done;
 always @(posedge clk)begin
  if(!rst_n)begin active<=0;owner<=0;last_owner<=0;end
  else begin
   if(m_request&&m_ready)begin active<=1;owner<=choose_cloud;last_owner<=choose_cloud;end
   if(active&&m_done)active<=0;
  end
 end
endmodule
