`timescale 1ns/1ps
// Capability candidate: NOREG output hold + synchronous validity epochs.
// RAM outputs are never reset. Independent domain-reset semantics use two
// one-bit FFs. PortB captures raw oldA plus old live-valid, avoiding reset-race
// forwarding and preserving simultaneousA-reset/B-capture old-register data.
module dual_eram_registers_validmask_v2 #(parameter WIDTH=146,SLICE=20)(
 input wire clk,live_reset_n,host_reset_n,live_write,host_write,
 input wire[WIDTH-1:0]live_input,
 output wire[WIDTH-1:0]live_output,host_output
);
 localparam N=(WIDTH+SLICE-1)/SLICE;
 wire[N*SLICE-1:0]input_padded={{(N*SLICE-WIDTH){1'b0}},live_input};
 wire[N*SLICE-1:0]live_raw,host_raw;
 reg live_valid,host_valid;
 always @(posedge clk)begin
  if(!live_reset_n)live_valid<=0;else if(live_write)live_valid<=1;
  if(!host_reset_n)host_valid<=0;else if(host_write)host_valid<=live_valid;
 end
 assign live_output=live_valid?live_raw[WIDTH-1:0]:{WIDTH{1'b0}};
 assign host_output=host_valid?host_raw[WIDTH-1:0]:{WIDTH{1'b0}};
 genvar g;generate for(g=0;g<N;g=g+1)begin:parts
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(SLICE),.DATA_WIDTH_B(SLICE),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
  .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.BYTE_ENABLE(0),.MODE("DP"),
  .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.WRITEMODE_A("WRITETHROUGH"),.WRITEMODE_B("WRITETHROUGH"),
  .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),.OREGSET_A("RESET"),.OREGSET_B("RESET"),
  .SSROVERCE("ENABLE"),.CLKMODE("SYNC"),.ASYNC_RESET_RELEASE_A("SYNC"),.ASYNC_RESET_RELEASE_B("SYNC"),.FILL_ALL("00000000000000000000"))ram(
  .clka(clk),.dia(input_padded[g*SLICE+:SLICE]),.addra(10'd0),.cea(live_write&&live_reset_n),.ocea(1'b1),.wea(live_write&&live_reset_n),.rsta(1'b0),.bea(1'b1),
  .clkb(clk),.dib(live_raw[g*SLICE+:SLICE]),.addrb(10'd1),.ceb(host_write&&host_reset_n),.oceb(1'b1),.web(host_write&&host_reset_n),.rstb(1'b0),.beb(1'b1),
  .doa(live_raw[g*SLICE+:SLICE]),.dob(host_raw[g*SLICE+:SLICE]),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr());
 end endgenerate
endmodule
