`timescale 1ns/1ps
// Isolated four-slot DDR frame store. No board activation or CDC here.
// A verified last-write readback orders prior writes only under the outer
// controller's ordered-write/read service contract. Physical MC proof is external.
// Partial/aborted frames fence their last accepted write before slot release.
// Posted AR and split AW/W are retained on a local fault; reset requires outer
// drain or a verified whole-MC reset including every producer/consumer.
module leased_ddr_frame_store #(
 parameter WIDTH=1280,HEIGHT=720,BUFFER_STRIDE=2097152,
 parameter FRAME_WORDS=WIDTH*HEIGHT/8,LINE_WORDS=WIDTH/8,
 parameter TIMEOUT=100000,READ_WINDOW=16,WRITE_DRAIN=128,CAMERA_ID=2,LINE_READ_GUARD=WRITE_DRAIN
)(
 input wire clk,rst_n,memory_ready,input wire[31:0]time_gray,
 input wire owner_rearm,owner_quiesce,input wire[31:0]owner_epoch,
 input wire external_pipeline_idle,lease_reader_idle,
 output wire owner_initialized,owner_rearm_ack,owner_rearm_rejected,
 input wire lease_acquire,output wire lease_acquire_ready,lease_acquire_ack,
 output wire lease_valid,output wire[1:0]lease_slot,output wire[95:0]lease_meta,
 output wire[47:0]lease_cookie,
 input wire lease_release,input wire[7:0]release_camera,input wire[47:0]release_cookie,
 output wire lease_release_ack,lease_release_rejected,
 input wire token_empty,output wire token_read,input wire token_valid,input wire[129:0]token_data,
 input wire line_req_valid,output wire line_req_ready,input wire new_frame,input wire[15:0]source_row,input wire request_bank,
 output wire line_write,output wire[127:0]line_data,output wire[7:0]line_address,output wire line_bank,
 output reg line_done,line_good,output wire front_valid,
 output wire[31:0]front_id,front_session,front_start_ms,
 output wire[24:0]awaddr,output wire awvalid,input wire awready,
 output wire[127:0]wdata,output wire[15:0]wstrb,output wire wvalid,wlast,input wire wready,
 output wire[24:0]araddr,output wire arvalid,input wire arready,
 input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,
 output reg fault,output reg[31:0]published_frames,rejected_frames,
 output wire read_active,configuration_error,output wire[127:0]debug_state
);
 localparam CONFIG_OK=WIDTH>=8&&WIDTH%8==0&&HEIGHT>0&&FRAME_WORDS==WIDTH*HEIGHT/8&&
  LINE_WORDS==WIDTH/8&&LINE_WORDS<=256&&FRAME_WORDS*8<=BUFFER_STRIDE&&
  BUFFER_STRIDE%8==0&&4*BUFFER_STRIDE<=33554432&&READ_WINDOW>=1&&READ_WINDOW<=16&&
  WRITE_DRAIN>=1&&WRITE_DRAIN<=255&&LINE_READ_GUARD>=0&&LINE_READ_GUARD<=255&&TIMEOUT>=8&&(CAMERA_ID==1||CAMERA_ID==2);
 assign configuration_error=!CONFIG_OK;
 localparam FETCH=0,WAIT_TOKEN=1,ALLOCATE=2,ALLOC_WAIT=3,WRITE=4,
  FENCE_REQ=5,FENCE_WAIT=6,COMMIT_WAIT=7,ABORT_FENCE_REQ=8,ABORT_FENCE_WAIT=9,ABORT_WAIT=10;
 localparam IDLE=0,LINE=1,FENCE_ADDR=2,FENCE_DATA=3;
 reg[3:0]writer;reg[1:0]reader;
 reg[95:0]pending_meta;reg pending_start;
 reg[31:0]write_count,write_age,read_age;
 reg[24:0]write_address,line_base,read_address;
 reg[127:0]write_word;reg aw_sent,w_sent,write_unfenced;
 reg[8:0]issued,returned;reg[5:0]read_pending;reg ar_live;
 reg task_bank;reg[7:0]read_guard;
 reg fence_done,fence_good;
 reg[31:0]time_sync0,time_sync1;
 wire owner_fault,owner_writer_valid,owner_ready_valid,begin_ready,begin_ack,commit_ack,abort_ack;
 wire present_ready,present_ack,sequence_exhausted;wire[31:0]active_epoch;
 wire[1:0]writer_slot,front_slot,ready_slot;wire[95:0]writer_meta,front_meta,ready_meta;
 wire[3:0]occupied;
 function[31:0]ungray(input[31:0]g);
  integer n;begin ungray[31]=g[31];for(n=30;n>=0;n=n-1)ungray[n]=ungray[n+1]^g[n];end
 endfunction
 always @(posedge clk)begin
  if(!rst_n)begin time_sync0<=0;time_sync1<=0;end
  else begin time_sync0<=time_gray;time_sync1<=time_sync0;end
 end
 wire write_idle=writer!=WRITE&&!write_unfenced;
 wire writer_waiting=writer!=FETCH&&writer!=FENCE_REQ&&writer!=ABORT_FENCE_REQ;
 wire reader_waiting=reader!=IDLE||ar_live||read_pending!=0;
 wire bad_response=rvalid&&(read_pending==0||!rlast);
 wire local_abort_now=!CONFIG_OK||owner_fault||bad_response||(writer_waiting&&write_age==TIMEOUT-1)||
                      (reader_waiting&&read_age==TIMEOUT-1);
 wire all_idle=external_pipeline_idle&&lease_reader_idle&&reader==IDLE&&!ar_live&&read_pending==0&&
               writer==FETCH&&!token_valid&&write_idle&&!rvalid;
 wire fence_request=writer==FENCE_REQ||writer==ABORT_FENCE_REQ;
 wire fence_accept=reader==IDLE&&!ar_live&&read_pending==0&&fence_request&&memory_ready&&!fault;
 wire take_line=line_req_valid&&line_req_ready;
 wire take_present=take_line&&new_frame&&present_ready;
 wire[1:0]selected_front=take_present?ready_slot:front_slot;
 wire selected_valid=front_valid||take_present;
 stereo_frame_lease4 #(.CAMERA_ID(CAMERA_ID),.SEQ_BITS(16))ownership(
  .clk(clk),.rst_n(rst_n),.enable(memory_ready&&!fault&&!local_abort_now),.quiesce(owner_quiesce),
  .all_pipeline_idle(all_idle),.write_pipeline_idle(write_idle),
  .read_pipeline_idle(lease_reader_idle&&reader==IDLE&&!ar_live&&read_pending==0),
  .rearm_valid(owner_rearm&&CONFIG_OK),.rearm_epoch(owner_epoch),.initialized(owner_initialized),.epoch(active_epoch),
  .rearm_ack(owner_rearm_ack),.rearm_rejected(owner_rearm_rejected),.fault(owner_fault),
  .write_begin_valid(writer==ALLOCATE&&!fault),.write_begin_meta(pending_meta),
  .write_begin_ready(begin_ready),.write_begin_ack(begin_ack),
  .write_commit_valid(writer==COMMIT_WAIT&&!fault&&!local_abort_now),.write_abort_valid(writer==ABORT_WAIT&&!fault&&!local_abort_now),
  .write_commit_ack(commit_ack),.write_abort_ack(abort_ack),
  .writer_valid(owner_writer_valid),.writer_slot(writer_slot),.writer_meta(writer_meta),
  .present_valid(take_line&&new_frame),.present_ready(present_ready),.present_ack(present_ack),
  .front_valid(front_valid),.front_slot(front_slot),.front_meta(front_meta),
  .ready_valid(owner_ready_valid),.ready_slot(ready_slot),.ready_meta(ready_meta),
  .acquire_valid(lease_acquire),.acquire_ready(lease_acquire_ready),.acquire_ack(lease_acquire_ack),
  .lease_valid(lease_valid),.lease_slot(lease_slot),.lease_meta(lease_meta),.lease_cookie(lease_cookie),
  .release_valid(lease_release),.release_camera(release_camera),.release_cookie(release_cookie),
  .release_ack(lease_release_ack),.release_rejected(lease_release_rejected),
  .occupied_mask(occupied),.sequence_exhausted(sequence_exhausted));
 assign front_session=front_meta[95:64];assign front_id=front_meta[63:32];assign front_start_ms=front_meta[31:0];
 assign token_read=memory_ready&&!fault&&!local_abort_now&&owner_initialized&&!owner_quiesce&&writer==FETCH&&!token_empty;
 assign awaddr=write_address;assign wdata=write_word;assign wstrb=0;assign wlast=1;
 // Retain both halves of a posted write on timeout; never reuse its slot.
 assign awvalid=rst_n&&writer==WRITE&&!aw_sent&&reader!=LINE;
 assign wvalid=rst_n&&writer==WRITE&&!w_sent&&reader!=LINE;
 assign line_req_ready=memory_ready&&!fault&&!local_abort_now&&owner_initialized&&!owner_quiesce&&reader==IDLE&&!ar_live&&read_pending==0&&!fence_request&&writer!=WRITE;
 assign araddr=read_address;assign arvalid=rst_n&&ar_live;
 assign rready=rst_n&&(reader!=IDLE||read_pending!=0||fault||bad_response);
 assign read_active=reader!=IDLE||ar_live||read_pending!=0;
 assign line_write=reader==LINE&&rvalid&&rready&&rlast&&!fault;
 assign line_data=rdata;assign line_address=returned[7:0];assign line_bank=task_bank;
 assign debug_state={16'b0,writer,reader,fault,owner_fault,occupied,ar_live,write_unfenced,
                     aw_sent,w_sent,read_pending,issued,returned,write_count,write_address,15'd0};
 wire take_ar=arvalid&&arready,take_r=rvalid&&rready&&rlast;
 wire write_complete=writer==WRITE&&(aw_sent||(awvalid&&awready))&&(w_sent||(wvalid&&wready));
 always @(posedge clk)begin
  if(!rst_n)begin
   writer<=FETCH;reader<=IDLE;pending_meta<=0;pending_start<=0;write_count<=0;
   write_age<=0;read_age<=0;write_address<=0;line_base<=0;read_address<=0;write_word<=0;
   aw_sent<=0;w_sent<=0;write_unfenced<=0;issued<=0;returned<=0;read_pending<=0;ar_live<=0;
   task_bank<=0;read_guard<=0;fence_done<=0;fence_good<=0;line_done<=0;line_good<=0;
   fault<=0;published_frames<=0;rejected_frames<=0;
  end else begin
   line_done<=0;fence_done<=0;
   if(local_abort_now)begin fault<=1;line_done<=1;line_good<=0;end
   if(awvalid&&awready)begin aw_sent<=1;write_age<=0;end
   if(wvalid&&wready)begin w_sent<=1;write_age<=0;end
   if(write_complete)write_unfenced<=1;
   // Address requests are registered before presentation, and remain posted
   // until accepted even if fault is raised while ARREADY is low.
   if(take_ar)begin
    ar_live<=0;read_age<=0;
    if(reader==LINE)issued<=issued+1'b1;
    else if(reader==FENCE_ADDR)reader<=FENCE_DATA;
   end
   case({take_ar,take_r})
    2'b10:read_pending<=read_pending+1'b1;
    2'b01:begin if(read_pending==0)fault<=1;else read_pending<=read_pending-1'b1;end
    default:begin end
   endcase
   if(rvalid&&rready&&!rlast)begin fault<=1;line_done<=1;line_good<=0;end
   if(fault||local_abort_now)begin
    // No new read is scheduled; every already-posted request is drained.
    if(!ar_live&&read_pending==0)reader<=IDLE;
   end else if(memory_ready)begin
    if(writer!=FETCH&&writer!=FENCE_REQ&&writer!=ABORT_FENCE_REQ)begin
     if(write_age==TIMEOUT-1)begin fault<=1;line_done<=1;line_good<=0;end
     else write_age<=write_age+1'b1;
    end else write_age<=0;
    case(writer)
     FETCH:begin
      if(owner_quiesce&&owner_writer_valid)begin pending_start<=0;writer<=write_unfenced?ABORT_FENCE_REQ:ABORT_WAIT;end
      else if(token_read)writer<=WAIT_TOKEN;
     end
     WAIT_TOKEN:if(token_valid)begin
      write_age<=0;writer<=FETCH;
      if(owner_quiesce)begin
       pending_start<=0;
       if(owner_writer_valid)writer<=write_unfenced?ABORT_FENCE_REQ:ABORT_WAIT;
      end else case(token_data[129:128])
       1:begin
        pending_meta<={token_data[63:32],token_data[31:0],ungray(time_sync1)};pending_start<=1;
        if(owner_writer_valid)begin
         rejected_frames<=rejected_frames+1'b1;writer<=write_unfenced?ABORT_FENCE_REQ:ABORT_WAIT;
        end else writer<=ALLOCATE;
       end
       0:if(owner_writer_valid)begin
        if(write_count<FRAME_WORDS)begin
         write_address<=writer_slot*BUFFER_STRIDE+write_count*8;write_word<=token_data[127:0];
         aw_sent<=0;w_sent<=0;writer<=WRITE;
        end else begin pending_start<=0;rejected_frames<=rejected_frames+1'b1;writer<=write_unfenced?ABORT_FENCE_REQ:ABORT_WAIT;end
       end
       2:begin
        pending_start<=0;
        if(owner_writer_valid&&token_data[0]&&token_data[32:1]==writer_meta[63:32]&&token_data[64:33]==writer_meta[95:64]&&write_count==FRAME_WORDS)
         writer<=FENCE_REQ;
        else begin
         rejected_frames<=rejected_frames+1'b1;
         if(owner_writer_valid)writer<=write_unfenced?ABORT_FENCE_REQ:ABORT_WAIT;
        end
       end
       default:begin
        pending_start<=0;rejected_frames<=rejected_frames+1'b1;
        if(owner_writer_valid)writer<=write_unfenced?ABORT_FENCE_REQ:ABORT_WAIT;
       end
      endcase
     end
     ALLOCATE:begin
      if(owner_quiesce)begin pending_start<=0;writer<=FETCH;end
      else if(begin_ready)writer<=ALLOC_WAIT;
     end
     ALLOC_WAIT:if(begin_ack)begin write_count<=0;pending_start<=0;writer<=owner_quiesce?ABORT_WAIT:FETCH;end
     WRITE:if(write_complete)begin
      writer<=owner_quiesce?ABORT_FENCE_REQ:FETCH;write_count<=write_count+1'b1;
      if(owner_quiesce)pending_start<=0;
     end
     FENCE_REQ:if(fence_accept)begin writer<=FENCE_WAIT;write_age<=0;end
     ABORT_FENCE_REQ:if(fence_accept)begin writer<=ABORT_FENCE_WAIT;write_age<=0;end
     FENCE_WAIT,ABORT_FENCE_WAIT:if(fence_done)begin
      if(!fence_good)begin fault<=1;rejected_frames<=rejected_frames+1'b1;end
      else begin write_unfenced<=0;writer<=writer==FENCE_WAIT?COMMIT_WAIT:ABORT_WAIT;end
     end
     COMMIT_WAIT:if(commit_ack)begin published_frames<=published_frames+1'b1;writer<=FETCH;end
     ABORT_WAIT:if(abort_ack)writer<=pending_start?ALLOCATE:FETCH;
     default:begin fault<=1;end
    endcase
    if(reader!=IDLE||ar_live||read_pending!=0)begin
     if(read_age==TIMEOUT-1)begin fault<=1;line_done<=1;line_good<=0;end
     else read_age<=read_age+1'b1;
    end else read_age<=0;
    if(rvalid&&rready)read_age<=0;
    case(reader)
     IDLE:begin
      if(fence_accept)begin reader<=FENCE_ADDR;read_age<=0;read_guard<=WRITE_DRAIN;end
      else if(take_line)begin
       if(selected_valid&&source_row<HEIGHT)begin
        line_base<=selected_front*BUFFER_STRIDE+source_row*WIDTH;task_bank<=request_bank;
        issued<=0;returned<=0;reader<=LINE;read_age<=0;read_guard<=LINE_READ_GUARD;
       end else begin line_done<=1;line_good<=0;end
      end
     end
     LINE:begin
      if(read_guard!=0)read_guard<=read_guard-1'b1;
      if(!ar_live&&read_guard==0&&issued<LINE_WORDS&&read_pending<READ_WINDOW)begin
       read_address<=line_base+{issued,3'b0};ar_live<=1;
      end
      if(take_r)begin
       returned<=returned+1'b1;
       if(returned==LINE_WORDS-1)begin reader<=IDLE;line_done<=1;line_good<=1;end
      end
     end
     FENCE_ADDR:begin
      if(read_guard!=0)read_guard<=read_guard-1'b1;
      else if(!ar_live)begin read_address<=write_address;ar_live<=1;end
     end
     FENCE_DATA:if(take_r)begin
      reader<=IDLE;fence_done<=1;fence_good<=rdata==write_word;read_age<=0;
     end
    endcase
   end
  end
 end
endmodule
