`timescale 1ns/1ps
// Exact160-beat RGB565 lines, up to16 ordered reads; no scaling.
module hd_scan_reader #(parameter BASE=20971520,STRIDE=1048576,WINDOW=16,TIMEOUT=200000)(
 input wire clk,rst_n,request,input wire[15:0]row,input wire cache_bank,input wire[32:0]publication,
 output wire ready,output wire line_write,output wire[127:0]line_data,output wire[7:0]line_address,
 output reg line_bank,line_done,line_good,output reg[31:0]line_frame,line_session,
 output wire[24:0]araddr,output wire arvalid,input wire arready,
 input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,read_active,output reg fault
);
 reg busy,command_live;reg[8:0]issued,returned;reg[24:0]base_address,command_address;reg[(TIMEOUT>1?$clog2(TIMEOUT):1)-1:0]age;
 wire command_take=arvalid&&arready,return_take=rvalid&&rready;
 assign ready=!busy&&!fault&&!command_live&&issued==returned;
 assign araddr=command_address;assign arvalid=rst_n&&command_live;
 assign rready=rst_n&&(busy||fault||issued!=returned);
 assign read_active=busy||command_live||issued!=returned;
 assign line_write=return_take&&busy&&!fault&&rlast&&issued!=returned;
 assign line_data=rdata;assign line_address=returned[7:0];
 always @(posedge clk)begin
  if(!rst_n)begin busy<=0;command_live<=0;issued<=0;returned<=0;base_address<=0;command_address<=0;age<=0;line_bank<=0;line_done<=0;line_good<=0;line_frame<=0;line_session<=0;fault<=0;end
  else begin
   line_done<=0;
   if(command_take)begin command_live<=0;issued<=issued+1'b1;end
   if(return_take)begin
    if(issued==returned||!rlast)fault<=1;
    else begin
     returned<=returned+1'b1;
     if(returned==159)begin if(issued!=160||command_live)fault<=1;else begin busy<=0;line_done<=1;line_good<=!fault;end end
    end
   end
   if(read_active)begin age<=age+1'b1;if(age==TIMEOUT-1)fault<=1;end else age<=0;
   if(command_take||return_take)age<=0;
   if(!fault)begin
    if(request&&ready)begin
     line_bank<=cache_bank;line_good<=0;line_frame<=publication[32:1];line_session<={31'd0,publication[0]};
     if(row>=720||publication[32:1]==0)line_done<=1;
     else begin busy<=1;issued<=0;returned<=0;base_address<=BASE+(publication[0]?STRIDE:0)+({9'd0,row}<<10)+({9'd0,row}<<8);age<=0;end
    end
    if(busy&&!command_live&&issued<160&&issued-returned<WINDOW)begin command_address<=base_address+{issued,3'b0};command_live<=1;end
   end
  end
 end
endmodule
