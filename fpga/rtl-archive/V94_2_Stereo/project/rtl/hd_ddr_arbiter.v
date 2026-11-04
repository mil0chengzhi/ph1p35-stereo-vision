`timescale 1ns/1ps
// Ordered single-beat MC arbitration, bounded read/write epochs and full
// physical write-to-read drain. Priority scan clients alternate with normal
// traffic; no active model producer can indefinitely starve camera/HDMI lines.
module hd_ddr_arbiter #(
 parameter CLIENTS=3,OWNER_BITS=$clog2(CLIENTS),WRITE_DRAIN=128,TIMEOUT=200000,
 parameter MAX_OUTSTANDING=16,READ_QUANTUM=32,PRIORITY_QUANTUM=160,
 parameter[CLIENTS-1:0]PRIORITY_MASK=1
)(
 input wire clk,rst_n,enable,
 input wire[CLIENTS*25-1:0]c_awaddr,c_araddr,input wire[CLIENTS*128-1:0]c_wdata,input wire[CLIENTS*16-1:0]c_wstrb,
 input wire[CLIENTS-1:0]c_awvalid,c_wvalid,c_wlast,c_arvalid,c_rready,c_read_active,
 output reg[CLIENTS-1:0]c_awready,c_wready,c_arready,c_rvalid,c_rlast,output wire[CLIENTS*128-1:0]c_rdata,
 output reg[24:0]awaddr,araddr,output reg[127:0]wdata,output reg[15:0]wstrb,
 output reg awvalid,wvalid,wlast,arvalid,rready,input wire awready,wready,arready,
 input wire[127:0]rdata,input wire rvalid,rlast,
 output reg fault,output wire configuration_error,output wire transaction_read_active,output wire[31:0]debug
);
 localparam IDLE=0,WRITE=1,DRAIN=2,READ=3;
 localparam CONFIG_OK=CLIENTS>=2&&CLIENTS<=8&&WRITE_DRAIN>=1&&WRITE_DRAIN<=65535&&
  MAX_OUTSTANDING>=1&&MAX_OUTSTANDING<=16&&READ_QUANTUM>=1&&READ_QUANTUM<=256&&
  PRIORITY_QUANTUM>=1&&PRIORITY_QUANTUM<=256;
 reg[1:0]state;reg[OWNER_BITS-1:0]owner,priority_robin,normal_robin;
 reg last_priority,prefer_write,aw_done,w_done,command_live;
 reg[24:0]command_address;reg[5:0]outstanding,memory_outstanding;
 reg[8:0]epoch_reads;reg[15:0]write_guard;reg[31:0]age;
 wire[CLIENTS-1:0]pending=c_arvalid|c_awvalid|c_wvalid;
 wire[CLIENTS-1:0]normal_pending=pending&~PRIORITY_MASK,priority_pending=pending&PRIORITY_MASK;
 reg choose_valid,choose_priority;reg[OWNER_BITS-1:0]choice,round_start;reg[CLIENTS-1:0]candidates;integer j,index;
 wire[CLIENTS-1:0]owner_mask={{(CLIENTS-1){1'b0}},1'b1}<<owner;
 wire other_pending=|(pending&~owner_mask);
 wire read_limit=epoch_reads>=(PRIORITY_MASK[owner]?PRIORITY_QUANTUM:READ_QUANTUM);
 wire client_take=c_arvalid[owner]&&c_arready[owner];
 wire memory_take=arvalid&&arready,return_take=rvalid&&rready;
 wire write_pair=(aw_done||(awvalid&&awready))&&(w_done||(wvalid&&wready));
 assign c_rdata={CLIENTS{rdata}};assign configuration_error=!CONFIG_OK;
 assign transaction_read_active=state==DRAIN||state==READ||command_live||outstanding!=0;
 wire[3:0]owner_debug=owner;
 assign debug={fault,command_live,2'b0,owner_debug,state,outstanding,memory_outstanding,epoch_reads,1'b0};
 always @*begin
  candidates=pending;choose_priority=0;
  if(|priority_pending&&(!last_priority||!(|normal_pending)))begin candidates=priority_pending;choose_priority=1;end
  else if(|normal_pending)candidates=normal_pending;
  round_start=choose_priority?priority_robin:normal_robin;
  choose_valid=0;choice=round_start;
  for(j=0;j<CLIENTS;j=j+1)begin
   index=round_start+j;if(index>=CLIENTS)index=index-CLIENTS;
   if(!choose_valid&&candidates[index])begin choose_valid=1;choice=index;end
  end
 end
 always @*begin
  c_awready=0;c_wready=0;c_arready=0;c_rvalid=0;c_rlast=0;
  awaddr=c_awaddr[owner*25+:25];wdata=c_wdata[owner*128+:128];wstrb=c_wstrb[owner*16+:16];wlast=c_wlast[owner];
  awvalid=0;wvalid=0;araddr=command_address;arvalid=0;rready=0;
  if(rst_n&&CONFIG_OK)begin
   // Fault never retracts an MC command which has already been posted.
   if(state==WRITE)begin
    awvalid=c_awvalid[owner]&&!aw_done;wvalid=c_wvalid[owner]&&!w_done;
    c_awready[owner]=awready&&!aw_done;c_wready[owner]=wready&&!w_done;
   end
   if(state==READ)begin
    arvalid=command_live;
    if(enable&&!fault&&!read_limit&&outstanding<MAX_OUTSTANDING&&(!command_live||arready))c_arready[owner]=1;
    if(memory_outstanding!=0)begin
     rready=fault?1'b1:c_rready[owner];
     if(!fault)begin c_rvalid[owner]=rvalid;c_rlast[owner]=rlast;end
    end
   end
  end
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;owner<=0;priority_robin<=0;normal_robin<=0;last_priority<=0;prefer_write<=0;aw_done<=0;w_done<=0;
   command_live<=0;command_address<=0;outstanding<=0;memory_outstanding<=0;epoch_reads<=0;write_guard<=0;age<=0;fault<=0;
  end else begin
   if(write_guard!=0)write_guard<=write_guard-1'b1;
   if(state==IDLE||memory_take||return_take||(awvalid&&awready)||(wvalid&&wready))age<=0;
   else if(age==TIMEOUT-1)fault<=1;else age<=age+1'b1;
   if(!enable&&(state!=IDLE||command_live||outstanding!=0))fault<=1;
   if(rvalid&&(state!=READ||memory_outstanding==0))fault<=1;
   if(state==WRITE)begin
    if(awvalid&&awready)aw_done<=1;
    if(wvalid&&wready)begin w_done<=1;if(!wlast)fault<=1;end
    if(write_pair)begin
     write_guard<=WRITE_DRAIN;aw_done<=0;w_done<=0;prefer_write<=0;
     // Pending client inputs describe the just-accepted beat until the next
     // clock. Return through IDLE so a new beat cannot be mistaken for it.
     state<=IDLE;
    end
   end else if(state==READ)begin
    if(client_take)begin command_address<=c_araddr[owner*25+:25];command_live<=1;epoch_reads<=epoch_reads+1'b1;end
    else if(memory_take)command_live<=0;
    case({client_take,return_take})
     2'b10:outstanding<=outstanding+1'b1;
     2'b01:if(outstanding==0)fault<=1;else outstanding<=outstanding-1'b1;
     default:begin end
    endcase
    case({memory_take,return_take})
     2'b10:memory_outstanding<=memory_outstanding+1'b1;
     2'b01:if(memory_outstanding==0)fault<=1;else memory_outstanding<=memory_outstanding-1'b1;
     default:begin end
    endcase
    if(return_take&&!rlast)fault<=1;
    if(!fault&&!c_read_active[owner]&&outstanding!=0)fault<=1;
    if(outstanding==0&&!command_live&&!client_take&&
      (fault||read_limit||!c_read_active[owner]||(!c_arvalid[owner]&&other_pending)))begin state<=IDLE;prefer_write<=1;end
   end else if(!fault&&enable&&CONFIG_OK)case(state)
    IDLE:if(choose_valid)begin
     owner<=choice;last_priority<=PRIORITY_MASK[choice];
     if(choose_priority)priority_robin<=choice==CLIENTS-1?0:choice+1'b1;
     else normal_robin<=choice==CLIENTS-1?0:choice+1'b1;
     aw_done<=0;w_done<=0;epoch_reads<=0;
     if(c_arvalid[choice]&&(!prefer_write||!(c_awvalid[choice]||c_wvalid[choice])))begin
      state<=DRAIN;
     end else state<=WRITE;
    end
    DRAIN:if(write_guard==0)state<=READ;
    default:begin fault<=1;state<=IDLE;end
   endcase
  end
 end
endmodule
