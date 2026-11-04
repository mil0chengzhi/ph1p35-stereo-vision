`timescale 1ns/1ps
// A completed view requires every one of921600 raster pixels, in order, from
// one published job. The line buffer has one pixel clock of latency.
module hd_scanout_completion #(parameter PIXEL_LATENCY=1)(
 input wire pixel_clk,memory_clk,rst_n,input wire[15:0]x,y,input wire de,
 input wire pixel_valid,source_valid,input wire[31:0]source_job,
 output wire[31:0]completed_job_memory,output reg[31:0]complete_frames,incomplete_frames
);
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]pixel_reset;
 always @(posedge pixel_clk or negedge rst_n)if(!rst_n)pixel_reset<=0;else pixel_reset<={pixel_reset[0],1'b1};
 reg[15:0]x_q,y_q;reg de_q,bad;reg[19:0]count;reg[31:0]job,complete_job;
 wire busy;wire[31:0]ack;
 wire[15:0]sample_x=PIXEL_LATENCY?x_q:x,sample_y=PIXEL_LATENCY?y_q:y;
 wire sample_de=PIXEL_LATENCY?de_q:de;
 cdc_mode #(.WIDTH(32))complete_view(.src_clk(pixel_clk),.dst_clk(memory_clk),.rst_n(rst_n),
  .write(!busy&&complete_job!=0&&complete_job!=ack),.mode(complete_job),.busy(busy),
  .commit(1'b1),.destination_mode(completed_job_memory),.src_mode(ack));
 always @(posedge pixel_clk)begin
  if(!pixel_reset[1])begin x_q<=0;y_q<=0;de_q<=0;bad<=1;count<=0;job<=0;complete_job<=0;complete_frames<=0;incomplete_frames<=0;end
  else begin
   x_q<=x;y_q<=y;de_q<=de;
   if(sample_de&&sample_x<1280&&sample_y<720)begin
    if(sample_x==0&&sample_y==0)begin count<=1;job<=source_job;bad<=!pixel_valid||!source_valid||source_job==0;end
    else begin
     count<=count+1'b1;
     if(!pixel_valid||!source_valid||source_job!=job||count!=sample_y*1280+sample_x)bad<=1;
     if(sample_x==1279&&sample_y==719)begin
      if(!bad&&pixel_valid&&source_valid&&source_job==job&&count==921599)begin complete_job<=job;complete_frames<=complete_frames+1'b1;end
      else incomplete_frames<=incomplete_frames+1'b1;
     end
    end
   end
  end
 end
endmodule
