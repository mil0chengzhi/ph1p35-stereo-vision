`timescale 1ns/1ps
// Two fixed pAXI clients, one physical DDR controller. Never overlap a read
// epoch with writes: the board's existing MC requires a write-drain guard.
// AW and W can accept in either order. Ownership stays fixed until both do.
// Read ownership covers a complete client line/fence, including every return.
module stereo_ddr_arbiter #(parameter WRITE_DRAIN=128,TIMEOUT=200000)(
 input wire clk,rst_n,enable,
 input wire [49:0] c_awaddr,c_araddr,
 input wire [1:0] c_awvalid,c_wvalid,c_wlast,c_arvalid,c_rready,c_read_active,
 input wire [255:0] c_wdata,input wire [31:0] c_wstrb,
 output reg [1:0] c_awready,c_wready,c_arready,c_rvalid,c_rlast,
 output wire [255:0] c_rdata,
 output reg [24:0] awaddr,araddr,output reg [127:0] wdata,
 output reg [15:0] wstrb,output reg awvalid,wvalid,wlast,arvalid,rready,
 input wire awready,wready,arready,input wire [127:0] rdata,input wire rvalid,rlast,
 output reg fault,output wire [31:0] debug
);
 localparam IDLE=0,WRITE=1,DRAIN=2,READ=3;
 reg [1:0] state;reg owner,prefer_write,prefer_read,aw_done,w_done;
 reg [7:0] drain;reg [31:0] age;
 assign c_rdata={rdata,rdata};
 assign debug={22'd0,fault,owner,state,aw_done,w_done,prefer_read,prefer_write,2'd0};
 always @* begin
  c_awready=0;c_wready=0;c_arready=0;c_rvalid=0;c_rlast=0;
  awaddr=c_awaddr[owner*25+:25];araddr=c_araddr[owner*25+:25];
  wdata=c_wdata[owner*128+:128];wstrb=c_wstrb[owner*16+:16];
  awvalid=0;wvalid=0;wlast=0;arvalid=0;rready=fault;
  if(enable&&!fault)begin
   if(state==WRITE)begin
    awvalid=c_awvalid[owner]&&!aw_done;wvalid=c_wvalid[owner]&&!w_done;
    wlast=c_wlast[owner];c_awready[owner]=awready&&!aw_done;
    c_wready[owner]=wready&&!w_done;
   end
   if(state==READ)begin
    arvalid=c_arvalid[owner];rready=c_rready[owner];
    c_arready[owner]=arready;c_rvalid[owner]=rvalid;c_rlast[owner]=rlast;
   end
  end
 end
 always @(posedge clk)begin
  if(!rst_n)begin state<=IDLE;owner<=0;prefer_write<=0;prefer_read<=0;
   aw_done<=0;w_done<=0;drain<=0;age<=0;fault<=0;end
  else if(enable&&!fault)begin
   if(state!=IDLE)begin
    if(age==TIMEOUT-1)fault<=1;else age<=age+1'b1;
   end else age<=0;
   case(state)
    IDLE:begin
     aw_done<=0;w_done<=0;
     if(|c_arvalid)begin
      owner<=c_arvalid[prefer_read]?prefer_read:!prefer_read;
      drain<=WRITE_DRAIN-1;state<=DRAIN;
     end else if(|(c_awvalid|c_wvalid))begin
      owner<=(c_awvalid[prefer_write]||c_wvalid[prefer_write])?prefer_write:!prefer_write;
      state<=WRITE;
     end
    end
    WRITE:begin
     if(awvalid&&awready)aw_done<=1;
     if(wvalid&&wready)w_done<=1;
     if((aw_done||(awvalid&&awready))&&(w_done||(wvalid&&wready)))begin
      state<=IDLE;prefer_write<=!owner;
     end
    end
    DRAIN:if(drain==0)state<=READ;else drain<=drain-1'b1;
    READ:if(!c_read_active[owner])begin state<=IDLE;prefer_read<=!owner;end
    default:begin fault<=1;state<=IDLE;end
   endcase
  end
 end
endmodule
