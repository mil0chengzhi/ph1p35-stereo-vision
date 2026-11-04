// Candidate subsystem only. No board top, physical clocks, pins or constraints.
module stereo_snapshot_uart_diagnostic(
    input wire csr_clk,csr_rst,input wire[15:0]csr_address,
    input wire csr_write,input wire[7:0]csr_write_data,
    output wire[7:0]csr_read_data,output wire csr_selected,
    input wire mem_clk,mem_rst,input wire[1:0]tap_valid,input wire[259:0]tap_data,
    input wire external_fault,output wire fault,configuration_error,
    output wire[24:0]awaddr,araddr,output wire[127:0]wdata,output wire[15:0]wstrb,
    output wire awvalid,wvalid,wlast,arvalid,rready,read_active,
    input wire awready,wready,arready,rvalid,rlast,input wire[127:0]rdata
);
    wire command_valid,command_ready,done,ram_write;
    wire[159:0]command;wire[255:0]response;
    wire[7:0]ram_address;wire[127:0]ram_data;
    snapshot_uart_mailbox mailbox(
        .csr_clk(csr_clk),.csr_rst(csr_rst),.csr_address(csr_address),.csr_write(csr_write),
        .csr_write_data(csr_write_data),.csr_read_data(csr_read_data),.csr_selected(csr_selected),
        .mem_clk(mem_clk),.mem_rst(mem_rst),.mem_command_valid(command_valid),.mem_command_ready(command_ready),
        .mem_command(command),.mem_done(done),.mem_response(response),.mem_fault(fault),
        .mem_ram_write(ram_write),.mem_ram_address(ram_address),.mem_ram_data(ram_data));
    snapshot_diagnostic_engine engine(
        .clk(mem_clk),.rst(mem_rst),.tap_valid(tap_valid),.tap_data(tap_data),
        .command_valid(command_valid),.command_ready(command_ready),.command(command),.done(done),.response(response),
        .external_fault(external_fault),.fault(fault),.configuration_error(configuration_error),
        .ram_write(ram_write),.ram_address(ram_address),.ram_data(ram_data),
        .awaddr(awaddr),.araddr(araddr),.wdata(wdata),.wstrb(wstrb),.awvalid(awvalid),.wvalid(wvalid),.wlast(wlast),
        .arvalid(arvalid),.rready(rready),.read_active(read_active),.awready(awready),.wready(wready),.arready(arready),
        .rvalid(rvalid),.rlast(rlast),.rdata(rdata));
endmodule
