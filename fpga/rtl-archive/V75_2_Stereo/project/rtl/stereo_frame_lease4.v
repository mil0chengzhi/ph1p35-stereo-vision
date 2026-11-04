`timescale 1ns/1ps
// One camera, four DDR slots; ownership only, no DDR commands or CDC here.
// Outer controller must verify complete frame + write fence before commit;
// *_pipeline_idle includes every external request, response and consumer stage.
// rst_n requires external drain or verified whole-memory reset. Rearm epochs
// must never be reused across such resets: that guarantee belongs to the owner.
module stereo_frame_lease4 #(parameter CAMERA_ID=2,parameter SEQ_BITS=16)(
 input wire clk,input wire rst_n,input wire enable,input wire quiesce,
 input wire all_pipeline_idle,input wire write_pipeline_idle,input wire read_pipeline_idle,
 input wire rearm_valid,input wire [31:0] rearm_epoch,
 output reg initialized,output reg [31:0] epoch,output reg rearm_ack,output reg rearm_rejected,output reg fault,
 input wire write_begin_valid,input wire [95:0] write_begin_meta,
 output wire write_begin_ready,output reg write_begin_ack,
 input wire write_commit_valid,input wire write_abort_valid,
 output reg write_commit_ack,output reg write_abort_ack,
 output reg writer_valid,output reg [1:0] writer_slot,output reg [95:0] writer_meta,
 input wire present_valid,output wire present_ready,output reg present_ack,
 output reg front_valid,output reg [1:0] front_slot,output reg [95:0] front_meta,
 output reg ready_valid,output reg [1:0] ready_slot,output reg [95:0] ready_meta,
 input wire acquire_valid,output wire acquire_ready,output reg acquire_ack,
 output reg lease_valid,output reg [1:0] lease_slot,output reg [95:0] lease_meta,
 output reg [31+SEQ_BITS:0] lease_cookie,
 input wire release_valid,input wire [7:0] release_camera,input wire [31+SEQ_BITS:0] release_cookie,
 output reg release_ack,output reg release_rejected,
 output wire [3:0] occupied_mask,output reg sequence_exhausted
);
 reg [SEQ_BITS-1:0] next_sequence;
 wire conflict=write_commit_valid&&write_abort_valid;
 wire run=initialized&&enable&&!quiesce&&!fault&&!conflict&&!rearm_valid;
 genvar i;
 generate for(i=0;i<4;i=i+1)begin:slots
  assign occupied_mask[i]=(front_valid&&front_slot==i)||(ready_valid&&ready_slot==i)||
                           (writer_valid&&writer_slot==i)||(lease_valid&&lease_slot==i);
 end endgenerate
 wire has_free=!(&occupied_mask);
 wire [1:0] free_slot=!occupied_mask[0]?0:!occupied_mask[1]?1:!occupied_mask[2]?2:3;
 assign write_begin_ready=run&&!writer_valid&&write_pipeline_idle&&has_free;
 assign present_ready=run&&ready_valid;
 assign acquire_ready=run&&!lease_valid&&!sequence_exhausted&&(ready_valid||front_valid);
 always @(posedge clk)begin
  if(!rst_n)begin
   initialized<=0;epoch<=0;rearm_ack<=0;rearm_rejected<=0;fault<=0;
   writer_valid<=0;writer_slot<=0;writer_meta<=0;write_begin_ack<=0;write_commit_ack<=0;write_abort_ack<=0;
   front_valid<=0;front_slot<=0;front_meta<=0;ready_valid<=0;ready_slot<=0;ready_meta<=0;present_ack<=0;
   lease_valid<=0;lease_slot<=0;lease_meta<=0;lease_cookie<=0;acquire_ack<=0;
   release_ack<=0;release_rejected<=0;sequence_exhausted<=0;next_sequence<=1;
  end else begin
   rearm_ack<=0;rearm_rejected<=0;write_begin_ack<=0;write_commit_ack<=0;write_abort_ack<=0;
   present_ack<=0;acquire_ack<=0;release_ack<=0;release_rejected<=0;
   if(rearm_valid)begin
    if(quiesce&&all_pipeline_idle&&write_pipeline_idle&&read_pipeline_idle&&
       !writer_valid&&!lease_valid&&rearm_epoch!=0&&rearm_epoch>epoch)begin
     initialized<=1;epoch<=rearm_epoch;rearm_ack<=1;fault<=0;
     front_valid<=0;ready_valid<=0;writer_valid<=0;lease_valid<=0;
     sequence_exhausted<=0;next_sequence<=1;
    end else rearm_rejected<=1;
   end else begin
    if(conflict)fault<=1;
    // Release can drain an old lease even while quiescing or after a fault.
    if(release_valid)begin
     if(lease_valid&&read_pipeline_idle&&release_camera==CAMERA_ID&&release_cookie==lease_cookie)begin
      lease_valid<=0;release_ack<=1;
     end else release_rejected<=1;
    end
    if(write_begin_valid&&write_begin_ready)begin
     writer_valid<=1;writer_slot<=free_slot;writer_meta<=write_begin_meta;write_begin_ack<=1;
    end
    // All selections use pre-edge completed roles, never a new commit/writer.
    if(present_valid&&present_ready)begin
     front_valid<=1;front_slot<=ready_slot;front_meta<=ready_meta;ready_valid<=0;present_ack<=1;
    end
    if(acquire_valid&&acquire_ready)begin
     lease_valid<=1;lease_slot<=ready_valid?ready_slot:front_slot;
     lease_meta<=ready_valid?ready_meta:front_meta;lease_cookie<={epoch,next_sequence};acquire_ack<=1;
     if(&next_sequence)sequence_exhausted<=1;
     else next_sequence<=next_sequence+1'b1;
    end
    if(!conflict&&writer_valid&&write_pipeline_idle)begin
     if(write_abort_valid)begin writer_valid<=0;write_abort_ack<=1;end
     else if(write_commit_valid&&initialized&&!fault)begin
      ready_valid<=1;ready_slot<=writer_slot;ready_meta<=writer_meta;writer_valid<=0;write_commit_ack<=1;
     end
    end
   end
  end
 end
endmodule
