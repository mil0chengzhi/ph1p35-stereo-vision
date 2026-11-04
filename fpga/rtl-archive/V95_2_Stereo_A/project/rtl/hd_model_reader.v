`timescale 1ns/1ps
// Ordered 128-bit DDR prefetch for immutable1280x720 {Z16,RGB565} models.
// Bus ownership is released while the local FIFO feeds a slow projector.
module hd_model_reader #(parameter BASE=16777216,STRIDE=2097152,FIFO_BITS=5,READ_WINDOW=16,TIMEOUT=200000)(
 input wire clk,rst_n,start,input wire[31:0]requested_job,bank0_job,bank1_job,
 output wire start_ready,output reg busy,done,fault,
 output wire point_valid,input wire point_ready,output wire[31:0]point_job,point_data,
 output reg[19:0]point_index,
 output wire[24:0]araddr,output wire arvalid,input wire arready,
 input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,read_active,
 output reg[31:0]cycles_elapsed
);
 localparam BEATS=230400,DEPTH=1<<FIFO_BITS;
 reg[127:0]fifo[0:DEPTH-1];reg[127:0]read_q; // Held through all four consumed points.
 reg[FIFO_BITS-1:0]head,tail;reg[1:0]lane,word_state;
 reg[18:0]issued,received,consumed;reg[31:0]source_job;reg[(TIMEOUT>1?$clog2(TIMEOUT):1)-1:0]age;reg bank;
 reg command_live;reg[24:0]command_address;
 wire bank_valid=requested_job!=0&&(requested_job==bank0_job||requested_job==bank1_job);
 wire source_unchanged=(bank?bank1_job:bank0_job)==source_job;
 wire take_point=point_valid&&point_ready;
 assign start_ready=!busy&&!fault&&!command_live&&issued==received&&bank_valid;
 assign araddr=command_address;assign arvalid=rst_n&&command_live;
 assign rready=rst_n&&(busy||fault||issued!=received);
 assign read_active=command_live||issued!=received;
 assign point_valid=rst_n&&busy&&!fault&&word_state==2;
 assign point_data=read_q>>(lane*32);assign point_job=source_job;
 always @(posedge clk)begin
  if(rvalid&&rready&&!fault&&issued!=received&&rlast)fifo[tail]<=rdata;
  if(busy&&!fault&&word_state==0&&consumed!=received)read_q<=fifo[head];
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   head<=0;tail<=0;lane<=0;word_state<=0;issued<=0;received<=0;consumed<=0;point_index<=0;
   source_job<=0;bank<=0;busy<=0;done<=0;fault<=0;command_live<=0;command_address<=0;age<=0;cycles_elapsed<=0;
  end else begin
   done<=0;
   if(arvalid&&arready)begin command_live<=0;issued<=issued+1'b1;end
   if(rvalid&&rready)begin
    if(issued==received||!rlast)fault<=1;
    else begin received<=received+1'b1;tail<=tail+1'b1;end
   end
   if(read_active)begin age<=age+1'b1;if(age==TIMEOUT-1)fault<=1;end else age<=0;
   if((arvalid&&arready)||(rvalid&&rready))age<=0;
   if(busy&&cycles_elapsed!=32'hffffffff)cycles_elapsed<=cycles_elapsed+1'b1;
   if(busy&&!source_unchanged)fault<=1;
   if(!fault)begin
    if(start&&start_ready)begin
     source_job<=requested_job;bank<=requested_job==bank1_job;busy<=1;point_index<=0;issued<=0;received<=0;consumed<=0;
     head<=0;tail<=0;lane<=0;word_state<=0;age<=0;cycles_elapsed<=0;
    end
    if(busy&&!command_live&&issued<BEATS&&issued-received<READ_WINDOW&&issued-consumed<DEPTH-1)begin
     command_address<=BASE+(bank?STRIDE:0)+{issued,3'b0};command_live<=1;
    end
    if(busy)case(word_state)
     0:if(consumed!=received)word_state<=1;
     1:begin lane<=0;word_state<=2;end
     2:if(take_point)begin
      point_index<=point_index+1'b1;
      if(lane==3)begin consumed<=consumed+1'b1;head<=head+1'b1;word_state<=0;end else lane<=lane+1'b1;
      if(point_index==921599)begin
       if(issued!=BEATS||received!=BEATS||command_live)fault<=1;
       else begin busy<=0;done<=1;end
      end
     end
     default: fault<=1;
    endcase
   end
  end
 end
endmodule
