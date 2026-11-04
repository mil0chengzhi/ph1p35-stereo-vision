`timescale 1ns/1ps
// Boot > queued UART > new AE batch. A granted AE group remains exclusive through
// launch and readback. One UART request may queue while AE owns the bus.
module camera_ae_mux(
 input wire clk,rst_n,boot_active,boot_valid,output wire boot_ready,
 input wire boot_read,input wire [15:0] boot_address,input wire [7:0] boot_value,output wire boot_done,
 input wire host_valid,host_read,input wire [6:0] host_device,input wire [15:0] host_address,input wire [7:0] host_value,output wire host_done,
 input wire ae_request,output reg ae_grant,input wire ae_valid,ae_read,
 input wire [15:0] ae_address,input wire [7:0] ae_value,output wire ae_ready,ae_done,
 output wire bus_valid,input wire bus_ready,output wire bus_read,output wire [6:0] bus_device,
 output wire [15:0] bus_address,output wire [7:0] bus_value,input wire bus_done
);
 reg pending,inflight,saved_read;reg [6:0] saved_device;reg [15:0] saved_address;reg [7:0] saved_value;
 assign bus_valid=boot_active?boot_valid:ae_grant?ae_valid:(pending&&!inflight);
 assign bus_read=boot_active?boot_read:ae_grant?ae_read:saved_read;
 assign bus_device=(boot_active||ae_grant)?7'h36:saved_device;
 assign bus_address=boot_active?boot_address:ae_grant?ae_address:saved_address;
 assign bus_value=boot_active?boot_value:ae_grant?ae_value:saved_value;
 assign boot_ready=boot_active&&bus_ready;
 assign boot_done=bus_done&&boot_active;
 assign host_done=bus_done&&inflight;
 assign ae_ready=ae_grant&&bus_ready&&!boot_active;
 assign ae_done=ae_grant&&bus_done&&!boot_active;
 always @(posedge clk) begin
  if(!rst_n) begin
   pending<=0;inflight<=0;ae_grant<=0;saved_read<=0;saved_device<=0;saved_address<=0;saved_value<=0;
  end else begin
   if(host_valid) begin pending<=1;saved_read<=host_read;saved_device<=host_device;saved_address<=host_address;saved_value<=host_value;end
   if(!boot_active&&!ae_grant&&pending&&!inflight&&bus_ready) begin pending<=0;inflight<=1;end
   if(bus_done&&inflight) inflight<=0;
   if(!boot_active&&!inflight&&!pending&&!host_valid&&ae_request) ae_grant<=1;
   if(!ae_request) ae_grant<=0;
  end
 end
endmodule
