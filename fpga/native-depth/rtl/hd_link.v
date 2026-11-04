`timescale 1ns/1ps
// Persistent one-packet RAMs with held-data metadata mailboxes. All endpoints
// share the system/memory reset: PHY restart must not release an owned RX RAM.
// TX metadata is consumed after the complete MAC frame and interframe gap.
module hd_link #(
 parameter[47:0]LOCAL_MAC=48'h020000002502,PEER_MAC=48'h020000002501,
 parameter[31:0]CALIBRATION_ID=32'hc633449a
)(
 input wire memory_clk,tx_clk,rx_clk,rst_n,
 input wire payload_write,input wire[10:0]payload_address,input wire[7:0]payload_data,
 input wire send_valid,output wire send_ready,input wire[7:0]send_kind,input wire[31:0]send_job,input wire[15:0]send_sequence,
 output wire packet_valid,output wire[191:0]packet_header,input wire packet_consume,
 input wire[10:0]read_address,output reg[7:0]read_data,
 output wire tx_offer,input wire tx_accept,output wire tx_busy,output wire[7:0]gmii_txd,output wire gmii_tx_en,
 input wire[7:0]gmii_rxd,input wire gmii_rx_dv,gmii_rx_er,
 output wire[31:0]rx_good,rx_bad_crc,rx_rejected
);
 reg[7:0]tx_ram[0:1279],rx_ram[0:1279];reg[7:0]tx_byte;
 wire[10:0]tx_address,rx_address;wire rx_write;wire[7:0]rx_byte;
 wire tx_mb_busy,tx_valid,tx_mac_ready,rx_mb_busy,rx_valid;
 wire[55:0]tx_descriptor;wire[191:0]rx_descriptor;
 // MAC admits only HD72/v1, known length/CID/flags, 22-bit jobs and
 // sequences <=2880. Keep all dynamic bits; reconstruct checked constants.
 wire[36:0]rx_compact;
 wire[7:0]received_kind={5'd0,rx_compact[36:34]};
 assign packet_header={32'h48443732,8'd1,received_kind,
  (received_kind==1?16'd1280:16'd0),10'd0,rx_compact[33:12],
  4'd0,rx_compact[11:0],16'd2880,CALIBRATION_ID,16'd1,16'd0};
 // All private RAM/mailboxes share the common reset. PHY-only restart
 // never releases ownership. Deassert MAC and active state locally in the
 // real TX/RX domains; a memory-clock reset must not directly release them.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]tx_reset,rx_reset;
 always @(posedge tx_clk or negedge rst_n)if(!rst_n)tx_reset<=0;else tx_reset<={tx_reset[0],1'b1};
 always @(posedge rx_clk or negedge rst_n)if(!rst_n)rx_reset<=0;else rx_reset<={rx_reset[0],1'b1};
 wire tx_rst_n=tx_reset[1],rx_rst_n=rx_reset[1];
 reg tx_active;wire tx_finished=tx_active&&!tx_busy;
 assign send_ready=!tx_mb_busy;
 assign tx_offer=tx_valid&&!tx_active&&tx_mac_ready;
 always @(posedge memory_clk)if(payload_write&&payload_address<1280)tx_ram[payload_address]<=payload_data;
 always @(posedge tx_clk)tx_byte<=tx_ram[tx_address];
 always @(posedge rx_clk)if(rx_write&&rx_address<1280)rx_ram[rx_address]<=rx_byte;
 always @(posedge memory_clk)read_data<=rx_ram[read_address];
 always @(posedge tx_clk or negedge tx_rst_n)begin
  if(!tx_rst_n)tx_active<=0;
  else if(tx_accept&&tx_offer)tx_active<=1;
  else if(tx_finished)tx_active<=0;
 end
 follow_payload_mailbox #(.WIDTH(56))tx_mailbox(.source_clk(memory_clk),.destination_clk(tx_clk),.rst_n(rst_n),
  .source_write(send_valid&&send_ready),.source_data({send_kind,send_job,send_sequence}),.source_busy(tx_mb_busy),
  .valid(tx_valid),.data(tx_descriptor),.consume(tx_finished));
 follow_payload_mailbox #(.WIDTH(37))rx_mailbox(.source_clk(rx_clk),.destination_clk(memory_clk),.rst_n(rst_n),
  .source_write(rx_valid&&!rx_mb_busy),.source_data({rx_descriptor[146:144],rx_descriptor[117:96],rx_descriptor[91:80]}),.source_busy(rx_mb_busy),
  .valid(packet_valid),.data(rx_compact),.consume(packet_consume));
 hd_packet_mac #(.LOCAL_MAC(LOCAL_MAC),.PEER_MAC(PEER_MAC),.CALIBRATION_ID(CALIBRATION_ID))mac(
  .tx_clk(tx_clk),.tx_rst_n(tx_rst_n),.tx_start(tx_accept&&tx_offer),.tx_ready(tx_mac_ready),.tx_busy(tx_busy),
  .tx_kind(tx_descriptor[55:48]),.tx_job(tx_descriptor[47:16]),.tx_sequence(tx_descriptor[15:0]),
  .tx_payload_address(tx_address),.tx_payload_data(tx_byte),.gmii_txd(gmii_txd),.gmii_tx_en(gmii_tx_en),
  .rx_clk(rx_clk),.rx_rst_n(rx_rst_n),.gmii_rxd(gmii_rxd),.gmii_rx_dv(gmii_rx_dv),.gmii_rx_er(gmii_rx_er),
  .packet_ready(!rx_mb_busy),.packet_valid(rx_valid),.packet_header(rx_descriptor),
  .payload_write(rx_write),.payload_address(rx_address),.payload_data(rx_byte),.rx_good(rx_good),.rx_bad_crc(rx_bad_crc),.rx_rejected(rx_rejected));
 // synthesis translate_off
 always @(posedge tx_clk)if(rst_n&&tx_accept&&!tx_offer)$fatal(1,"HD MAC admitted without descriptor");
 always @(posedge memory_clk)if(rst_n&&payload_write&&tx_mb_busy)$fatal(1,"HD TX packet RAM changed while owned");
 // synthesis translate_on
endmodule
