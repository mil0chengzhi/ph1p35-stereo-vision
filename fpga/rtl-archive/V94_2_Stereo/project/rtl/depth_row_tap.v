`timescale 1ns/1ps
// B26 metric row interface, pending board qualification.
// Tap one existing immutable-front display row. No extra DDR requests or stalls.
// Caller must HOLD new depth publication and prove hold_confirmed only after
// the current job has completed/published. Each row must match expected_job.
// The caller retains HOLD until all180 host rows and descriptor checks finish.
// Payload is320 little-endian uint16 Z values; zero remains invalid unchanged.
// Result store128-bit beats contain4 {Z16,colour16} points, low point first.
module depth_row_tap #(parameter WIDTH=320,HEIGHT=180,TIMEOUT=50000000)(
 input wire host_clk,memory_clk,rst_n,
 input wire host_request,input wire[7:0]host_row,input wire[31:0]host_expected_job,
 input wire[9:0]host_read_address,output wire[7:0]host_read_data,
 output wire host_busy,output reg host_valid,output reg[3:0]host_error,
 output reg[31:0]host_job,output reg[7:0]host_returned_row,output reg host_overrun,
 input wire hold_confirmed,
 input wire line_request,line_ready,input wire[15:0]line_row,
 input wire line_write,input wire[127:0]line_data,input wire[7:0]line_address,
 input wire line_done,line_good,input wire[31:0]line_frame_id,line_session
);
 // Both domains share asynchronous assertion and release locally after two clocks.
 // No independent domain reset; RAM contents are exposed only after a new ACK.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]host_reset,memory_reset;
 wire host_rst_n=host_reset[1],memory_rst_n=memory_reset[1];
 always @(posedge host_clk or negedge rst_n)
  if(!rst_n)host_reset<=0;else host_reset<={host_reset[0],1'b1};
 always @(posedge memory_clk or negedge rst_n)
  if(!rst_n)memory_reset<=0;else memory_reset<={memory_reset[0],1'b1};


 // Zero-cycle IA->OFA identity, same vendor primitive as the existing DDR
 // hold chain. Only diagnostic payload bit120 is delayed; all other bits,
 // address, write-enable and handshakes retain their original timing.
 (* keep="true" *)wire diagnostic_held_bit;
 wire[127:0]diagnostic_line_data={line_data[127:121],diagnostic_held_bit,line_data[119:0]};
 (* keep_hierarchy="true" *)PH1P_LOGIC_SLICE #(

          .ADDER_EN(1'b0),
          .INIT_LUT5A(32'hAAAAAAAA), .INIT_LUT5B(32'h00000000),
          .LUTMODE("DLUT5"), .LUT5A_SD0("IC"), .LUT5A_SD1("IE"), .LUT5B_SD2("ID"),
          .OFA_SD("LUT5A"), .OFB_SD("LUTX"), .OQAMUX("QA"), .OQBMUX("QB"),
          .ADDERMUX("0"), .LOC_F7CY("CY"), .CIMUX("FIXED"), .DELAY_EN("DISABLE"),
          .GSR("DISABLE"), .CEAMUX("0"), .CEBMUX("0"), .SSRMUX("0"), .ASRMUX("0"),
          .CLKMUX("0"), .REGA_SD("IMA"), .REGB_SD("IMB"),
          .REGA_REGSET_ASYNC("RESET"), .REGB_REGSET_ASYNC("RESET"),
          .REGA_REGSET_SYNC("RESET"), .REGB_REGSET_SYNC("RESET"),
          .SSROVERCE("DISABLE"), .LATCHMODE("DFF"), .TESTMODE("OFF"),
          .DISRAM("DISABLE"), .RAMMODE("SHIFTER"), .DISREADBACK("READBACK"), .FCIMUX("0")
        
 ) diagnostic_input_hold(
 .ia(line_data[120]),.ib(1'b0),.ic(1'b0),.id(1'b0),.ie(1'b0),.imf(1'b0),.ima(1'b0),.imb(1'b0),
 .cea(1'b0),.ceb(1'b0),.clk(1'b0),.ssr(1'b0),.asr(1'b0),.fci(),
 .ofa(diagnostic_held_bit),.ofb(),.oqa(),.oqb(),.fco());
 localparam WORDS=WIDTH/4,ABITS=$clog2(WORDS),BYTES=WIDTH*2;
 localparam E_HOLD=1,E_ROW=2,E_ORDER=3,E_JOB=4,E_SESSION=5,E_LINE=6,E_TIMEOUT=7;
 reg[63:0]payload[0:WORDS-1];reg[63:0]host_word;
 reg request_toggle,ack_toggle,ack_seen,request_seen;
 (* keep=1,preserve=1 *)reg[1:0]ack_sync,request_sync;
 reg[7:0]row_mailbox,target_row;
 reg[31:0]job_mailbox,target_job,return_job,age;
 reg[7:0]return_row;
 reg return_good;reg[3:0]return_error;
 reg pending,capturing,bad_order;reg[8:0]beats;
 assign host_busy=request_toggle!=ack_sync[1]||ack_seen!=ack_sync[1];
 always @(posedge host_clk)host_word<=payload[host_read_address[ABITS+2:3]];
 assign host_read_data=host_valid&&!host_busy&&host_read_address<BYTES?(host_word>>(host_read_address[2:0]*8)):8'd0;

 always @(posedge host_clk or negedge host_rst_n)begin
  if(!host_rst_n)begin request_toggle<=0;ack_sync<=0;ack_seen<=0;row_mailbox<=0;job_mailbox<=0;
   host_valid<=0;host_error<=0;host_job<=0;host_returned_row<=0;host_overrun<=0;end
  else begin
   ack_sync<={ack_sync[0],ack_toggle};
   if(host_request)begin
    if(host_busy)host_overrun<=1;
    else begin row_mailbox<=host_row;job_mailbox<=host_expected_job;request_toggle<=!request_toggle;host_valid<=0;host_error<=0;host_overrun<=0;end
   end
   if(ack_sync[1]!=ack_seen)begin
    ack_seen<=ack_sync[1];host_valid<=return_good;host_error<=return_error;
    host_job<=return_job;host_returned_row<=return_row;
   end
  end
 end

 task complete;
  input good;input[3:0]reason;input[31:0]job;
  begin
   pending<=0;capturing<=0;return_good<=good;return_error<=reason;
   return_job<=job;return_row<=target_row;ack_toggle<=request_seen;
  end
 endtask
 always @(posedge memory_clk or negedge memory_rst_n)begin
  if(!memory_rst_n)begin request_sync<=0;request_seen<=0;ack_toggle<=0;
   target_row<=0;target_job<=0;return_good<=0;return_error<=0;return_job<=0;return_row<=0;
   pending<=0;capturing<=0;bad_order<=0;beats<=0;age<=0;end
  else begin
   request_sync<={request_sync[0],request_toggle};
   if(request_sync[1]!=request_seen)begin
    request_seen<=request_sync[1];target_row<=row_mailbox;target_job<=job_mailbox;
    pending<=1;capturing<=0;bad_order<=0;beats<=0;age<=0;
   end else if(pending)begin
    age<=age+1'b1;
    if(!hold_confirmed)complete(0,E_HOLD,0);
    else if(target_row>=HEIGHT)complete(0,E_ROW,0);
    else if(target_job==0)complete(0,E_JOB,0);
    else if(age==TIMEOUT-1)complete(0,E_TIMEOUT,0);
    else if(!capturing)begin
     if(line_request&&line_ready&&line_row==target_row)begin capturing<=1;beats<=0;bad_order<=0;end
    end else begin
     if(line_write)begin
      if(beats>=WORDS||line_address!=beats)bad_order<=1;
      else payload[line_address[ABITS-1:0]]<={diagnostic_line_data[127:112],diagnostic_line_data[95:80],diagnostic_line_data[63:48],diagnostic_line_data[31:16]};
      if(beats<=WORDS)beats<=beats+1'b1;
     end
     if(line_done)begin
      if(bad_order||(line_write&&(beats>=WORDS||line_address!=beats)))complete(0,E_ORDER,line_frame_id);
      else if(!line_good||beats+(line_write?1:0)!=WORDS)complete(0,E_LINE,line_frame_id);
      else if(line_frame_id!=target_job)complete(0,E_JOB,line_frame_id);
      else if(line_session!=32'hD2800001)complete(0,E_SESSION,line_frame_id);
      else complete(1,0,line_frame_id);
     end
    end
   end
  end
 end
 // synthesis translate_off
 initial if(WIDTH!=320||HEIGHT!=180||TIMEOUT<100)$fatal(1,"Unsupported metric row capture configuration");
 // synthesis translate_on
endmodule
