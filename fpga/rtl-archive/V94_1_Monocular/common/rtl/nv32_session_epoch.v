`timescale 1ns/1ps
// Context held at display origin, before the delayed first queue pixel arrives.
// Never wrap a session epoch; block transmission on exhaustion until a verified
// complete-stream reset/restart. Fixed22-bit wire namespace; no persistent boot ID.
module nv32_session_epoch #(parameter EPOCH_BITS=22)(
 input wire clk,rst_n,frame_origin,input wire[31:0]context_word,
 output reg[EPOCH_BITS-1:0]epoch,output reg exhausted
);
 reg[31:0]previous_context;
 always @(posedge clk)begin
  if(!rst_n)begin epoch<=1;previous_context<=0;exhausted<=0;end
  else if(frame_origin&&context_word!=previous_context&&!exhausted)begin
   if(&epoch)exhausted<=1;
   else begin epoch<=epoch+1'b1;previous_context<=context_word;end
  end
 end
endmodule
