`timescale 1ns/1ps
// Board50MHz measurement from the accepted K2/host/BT capture event to the
// returned A full-frame scanout completion. Depth-core cycles alone do not pass.
module hd_capture_meter #(parameter DISTINGUISH_DRAIN=0)(
 input wire board_clk,memory_clk,rst_n,capture,cancel,
 input wire model_displayed,displayed_successful,input wire[31:0]displayed_job,
 output reg active,output reg[31:0]last_job,last_ticks,
 output reg successful,canceled,output wire notice_overrun,output wire within_5s,
 output reg[31:0]confirmed_job_memory
);
 wire mb_busy,notice_valid,seen_busy,seen_valid;wire[31:0]notice_job,seen_job;wire[32:0]notice_payload;reg overrun;
 assign notice_job=notice_payload[31:0];
 reg[31:0]ticks,previous_job;
 assign notice_overrun=overrun;assign within_5s=successful&&last_ticks<=32'd250000000;
 follow_payload_mailbox #(.WIDTH(33))completion_notice(.source_clk(memory_clk),.destination_clk(board_clk),.rst_n(rst_n),
  .source_write(model_displayed&&!mb_busy),.source_data({(!DISTINGUISH_DRAIN||displayed_successful),displayed_job}),.source_busy(mb_busy),
  .valid(notice_valid),.data(notice_payload),.consume(notice_valid));
 follow_payload_mailbox #(.WIDTH(32))completion_seen(.source_clk(board_clk),.destination_clk(memory_clk),.rst_n(rst_n),
  .source_write(notice_valid&&!seen_busy),.source_data(notice_job),.source_busy(seen_busy),
  .valid(seen_valid),.data(seen_job),.consume(seen_valid));
 always @(posedge memory_clk)begin
  if(!rst_n)begin overrun<=0;confirmed_job_memory<=0;end
  else begin
   if(model_displayed&&mb_busy)overrun<=1;
   if(seen_valid)confirmed_job_memory<=seen_job;
  end
 end
 always @(posedge board_clk)begin
  if(!rst_n)begin active<=0;last_job<=0;last_ticks<=0;successful<=0;canceled<=0;ticks<=0;previous_job<=0;end
  else begin
   if(active&&ticks!=32'hffffffff)ticks<=ticks+1'b1;
   if(capture&&!active)begin active<=1;successful<=0;canceled<=0;ticks<=0;end
   if(notice_valid)begin
    previous_job<=notice_job;
    if(active&&notice_job!=0&&notice_job!=previous_job)begin last_job<=notice_job;last_ticks<=ticks;successful<=notice_payload[32];active<=0;canceled<=!notice_payload[32];end
   end
   if(cancel&&active)begin active<=0;successful<=0;canceled<=1;last_ticks<=ticks;end
  end
 end
endmodule
