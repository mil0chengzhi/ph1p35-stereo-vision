`timescale 1ns/1ps
// One-entry held-data mailbox. The destination consumer samples data only
// while valid and acknowledges on consume. There is no redundant destination
// payload bank: the consuming MAC already latches its complete packet.
// Constrain held_data -> consumer latch below two destination clock periods;
// request first-stage and ACK first-stage cuts must be source-qualified.
module follow_payload_mailbox #(parameter WIDTH=256,SYNC_STAGES=2)(
 input wire source_clk,destination_clk,rst_n,
 input wire source_write,input wire[WIDTH-1:0]source_data,
 output wire source_busy,
 output wire valid,output wire[WIDTH-1:0]data,input wire consume
);
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]source_reset,destination_reset;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg request,ack;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[SYNC_STAGES-1:0]request_sync,ack_sync;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[WIDTH-1:0]held_data;
 always @(posedge source_clk or negedge rst_n)
  if(!rst_n)source_reset<=0;else source_reset<={source_reset[0],1'b1};
 always @(posedge destination_clk or negedge rst_n)
  if(!rst_n)destination_reset<=0;else destination_reset<={destination_reset[0],1'b1};
 assign source_busy=!source_reset[1]||request!=ack_sync[SYNC_STAGES-1];
 assign valid=destination_reset[1]&&request_sync[SYNC_STAGES-1]!=ack;
 assign data=held_data;
 always @(posedge source_clk or negedge source_reset[1])begin
  if(!source_reset[1])begin request<=0;ack_sync<=0;held_data<=0;end
  else begin
   ack_sync<={ack_sync[SYNC_STAGES-2:0],ack};
   if(source_write&&!source_busy)begin held_data<=source_data;request<=!request;end
  end
 end
 always @(posedge destination_clk or negedge destination_reset[1])begin
  if(!destination_reset[1])begin request_sync<=0;ack<=0;end
  else begin request_sync<={request_sync[SYNC_STAGES-2:0],request};if(valid&&consume)ack<=request_sync[SYNC_STAGES-1];end
 end
 // synthesis translate_off
 initial if(SYNC_STAGES<2||SYNC_STAGES>3)$fatal(1,"unsupported mailbox synchronizer length");
 // synthesis translate_on
endmodule
