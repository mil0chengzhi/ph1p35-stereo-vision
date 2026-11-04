`timescale 1ns/1ps
// Autonomous SC520CS-03 boot at 50 MHz. ID EE4B (cold) / 9E4B (configured).
// Read ID first; write the complete 167-entry full-mode ROM in order.
// POWER_WAIT=100 ms, RESET_WAIT=20 ms, TIMEOUT=1 s per state/transaction.
// stream_start arms the receiver when the final stream-on command is accepted;
// done/failed still report the acknowledgement of that command separately.
module sensor_boot #(parameter POWER_WAIT=5000000,RESET_WAIT=1000000,TIMEOUT=50000000)(
 input wire clk,input wire rst_n,input wire power_ready,
 output wire cmd_valid,input wire cmd_ready,output wire read,output wire [15:0] address,output wire [7:0] value,
 input wire cmd_done,input wire [1:0] cmd_error,input wire [7:0] read_data,
 output wire active,output reg done,output reg failed,output reg [2:0] code,
 output reg stream_start,output wire [7:0] step,output reg [15:0] chip_id
);
 localparam POWER=0,ISSUE=1,WAIT_DONE=2,RESET_DELAY=3,FINISHED=4;
 reg [2:0] state;reg [1:0] id_phase;reg [7:0] index;localparam AGE_BITS=TIMEOUT>1?$clog2(TIMEOUT):1,POWER_BITS=POWER_WAIT>1?$clog2(POWER_WAIT):1;reg[AGE_BITS-1:0]age;reg[POWER_BITS-1:0]power_age;
 wire [15:0] rom_address;wire [7:0] rom_value,rom_size;wire rom_valid;
 sc520cs_boot_rom rom(.index(index),.address(rom_address),.value(rom_value),.valid(rom_valid),.size(rom_size));
 assign active=(state!=FINISHED);
 assign cmd_valid=(state==ISSUE)&&((id_phase!=2)||rom_valid);
 assign read=(id_phase!=2);
 assign address=(id_phase==0)?16'h3107:(id_phase==1)?16'h3108:rom_address;
 assign value=(id_phase==2)?rom_value:8'b0;assign step=index;
 always @(posedge clk) begin
  if(!rst_n) begin state<=POWER;id_phase<=0;index<=0;age<=0;power_age<=0;done<=0;failed<=0;code<=0;stream_start<=0;chip_id<=0;end
  else begin
   stream_start<=0;
   if(state!=FINISHED) begin
    if(age==TIMEOUT-1) begin state<=FINISHED;done<=1;failed<=1;code<=4;end
    else begin
     age<=age+1'b1;
     case(state)
      POWER: begin
       if(!power_ready) power_age<=0;
       else if(power_age==POWER_WAIT-1) begin state<=ISSUE;age<=0;end
       else power_age<=power_age+1'b1;
      end
      ISSUE: begin
       if(id_phase==2&&!rom_valid) begin state<=FINISHED;done<=1;failed<=1;code<=5;end
       else if(cmd_ready) begin
        state<=WAIT_DONE;age<=0;
        if(id_phase==2 && rom_address==16'h0100 && rom_value==1) stream_start<=1;
       end
      end
      WAIT_DONE: if(cmd_done) begin
       age<=0;
       if(cmd_error!=0) begin done<=1;failed<=1;code<={1'b0,cmd_error};state<=FINISHED;end
       else if(id_phase==0) begin chip_id[15:8]<=read_data;id_phase<=1;state<=ISSUE;end
       else if(id_phase==1) begin
        chip_id[7:0]<=read_data;
        if({chip_id[15:8],read_data}!=16'hee4b && {chip_id[15:8],read_data}!=16'h9e4b) begin done<=1;failed<=1;code<=3;state<=FINISHED;end
        else begin id_phase<=2;state<=ISSUE;end
       end else if(index==rom_size-1'b1) begin done<=1;state<=FINISHED;end
       else begin index<=index+1'b1;state<=(rom_address==16'h0103)?RESET_DELAY:ISSUE;end
      end
      RESET_DELAY: if(age==RESET_WAIT-1) begin age<=0;state<=ISSUE;end
      default: begin state<=FINISHED;done<=1;failed<=1;code<=4;end
     endcase
    end
   end
  end
 end
endmodule
