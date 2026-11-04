`timescale 1ns/1ps
// NV31 Ethernet II video transmitter. Each descriptor owns a complete private
// RAM slot until packet_done. packet_word_data MUST be the one-cycle synchronous
// read of packet_word_addr; no additional output register or backpressure.
// RGB565 words and all NV31 multibyte fields are little endian. MAC addresses
// and EtherType are network byte order. Flags: bit0 first packet, bit1 last.
// One GMII byte per clk; packet_done is after the twelve-byte interframe gap.
module nv_video_tx #(
 parameter [47:0] LOCAL_MAC=48'h020000002501, PEER_MAC=48'h020000002502,
 parameter integer WIDTH=1280, HEIGHT=720, PACKET_PIXELS=640,
 parameter [7:0] STREAM_ID=8'd1
)(
 input wire clk,rst_n,
 input wire packet_valid, output wire packet_ready,
 input wire [31:0] packet_session,packet_frame,
 input wire [15:0] packet_index,
 output reg [9:0] packet_word_addr,
 input wire [15:0] packet_word_data,
 output wire busy, output reg packet_done,packet_good,
 output reg [31:0] tx_packets,tx_dropped,
 output reg [7:0] gmii_txd,output reg gmii_tx_en
);
 localparam integer PACKETS=WIDTH*HEIGHT/PACKET_PIXELS;
 localparam integer PARAM_OK=WIDTH>0 && HEIGHT>0 && WIDTH<=16384 && HEIGHT<=16384 &&
   PACKET_PIXELS>0 && PACKET_PIXELS<=1024 && WIDTH%PACKET_PIXELS==0 &&
   PACKETS>0 && PACKETS<=65535;
 localparam IDLE=0,PREAMBLE=1,HEADER=2,PIXELS=3,FCS=4,GAP=5;
 reg [2:0] state;
 reg [5:0] index;
 reg [10:0] payload_byte;
 reg [31:0] session_latched,frame_latched,crc,fcs;
 reg [15:0] packet_latched;

 function [31:0] crc32_byte;
  input [31:0] previous;input [7:0] data;
  reg [31:0] value;integer bit_number;
  begin value=previous^{24'b0,data};
   for(bit_number=0;bit_number<8;bit_number=bit_number+1)
    if(value[0]) value=(value>>1)^32'hEDB88320;else value=value>>1;
   crc32_byte=value;
  end
 endfunction
 function [7:0] header_byte;
  input [5:0] n;
  begin
   case(n)
    0:header_byte=PEER_MAC[47:40];1:header_byte=PEER_MAC[39:32];
    2:header_byte=PEER_MAC[31:24];3:header_byte=PEER_MAC[23:16];
    4:header_byte=PEER_MAC[15:8];5:header_byte=PEER_MAC[7:0];
    6:header_byte=LOCAL_MAC[47:40];7:header_byte=LOCAL_MAC[39:32];
    8:header_byte=LOCAL_MAC[31:24];9:header_byte=LOCAL_MAC[23:16];
    10:header_byte=LOCAL_MAC[15:8];11:header_byte=LOCAL_MAC[7:0];
    12:header_byte=8'h88;13:header_byte=8'hB5;
    14:header_byte=8'h4E;15:header_byte=8'h56;16:header_byte=8'h33;17:header_byte=8'h31;
    18:header_byte=8'd1;19:header_byte=STREAM_ID;20:header_byte=8'd1;
    21:header_byte={6'd0,(packet_latched==PACKETS-1),(packet_latched==0)};
    22:header_byte=session_latched[7:0];23:header_byte=session_latched[15:8];
    24:header_byte=session_latched[23:16];25:header_byte=session_latched[31:24];
    26:header_byte=frame_latched[7:0];27:header_byte=frame_latched[15:8];
    28:header_byte=frame_latched[23:16];29:header_byte=frame_latched[31:24];
    30:header_byte=packet_latched[7:0];31:header_byte=packet_latched[15:8];
    32:header_byte=PACKETS&255;33:header_byte=(PACKETS>>8)&255;
    34:header_byte=WIDTH&255;35:header_byte=(WIDTH>>8)&255;
    36:header_byte=HEIGHT&255;37:header_byte=(HEIGHT>>8)&255;
    default:header_byte=0;
   endcase
  end
 endfunction
 wire [7:0] next_header=header_byte(index);
 wire [7:0] next_pixel=payload_byte[0]?packet_word_data[15:8]:packet_word_data[7:0];
 assign busy=state!=IDLE;
 assign packet_ready=rst_n && PARAM_OK && state==IDLE;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;index<=0;payload_byte<=0;session_latched<=0;frame_latched<=0;
   packet_latched<=0;packet_word_addr<=0;crc<=32'hFFFFFFFF;fcs<=0;
   gmii_txd<=0;gmii_tx_en<=0;packet_done<=0;packet_good<=0;tx_packets<=0;tx_dropped<=0;
  end else begin
   packet_done<=0;packet_good<=0;gmii_tx_en<=0;
   if(!PARAM_OK) begin state<=IDLE;gmii_txd<=0;end
   else case(state)
    IDLE: if(packet_valid) begin
     packet_word_addr<=0;
     if(packet_index>=PACKETS) begin
      packet_done<=1;if(!(&tx_dropped))tx_dropped<=tx_dropped+1'b1;
     end else begin
      session_latched<=packet_session;frame_latched<=packet_frame;packet_latched<=packet_index;
      crc<=32'hFFFFFFFF;index<=0;state<=PREAMBLE;
     end
    end
    PREAMBLE: begin
     gmii_tx_en<=1;gmii_txd<=index==7?8'hD5:8'h55;
     if(index==7)begin index<=0;state<=HEADER;end else index<=index+1'b1;
    end
    HEADER: begin
     gmii_tx_en<=1;gmii_txd<=next_header;crc<=crc32_byte(crc,next_header);
     if(index==37)begin payload_byte<=0;state<=PIXELS;end else index<=index+1'b1;
    end
    PIXELS: begin
     gmii_tx_en<=1;gmii_txd<=next_pixel;crc<=crc32_byte(crc,next_pixel);
     // Updating address on LOW makes the external one-cycle RAM update at
     // HIGH, after this module sampled the HIGH byte of the previous word.
     if(!payload_byte[0] && packet_word_addr<PACKET_PIXELS-1)
      packet_word_addr<=packet_word_addr+1'b1;
     if(payload_byte==PACKET_PIXELS*2-1)begin
      fcs<=~crc32_byte(crc,next_pixel);index<=0;state<=FCS;
     end else payload_byte<=payload_byte+1'b1;
    end
    FCS: begin
     gmii_tx_en<=1;gmii_txd<=fcs[7:0];fcs<={8'd0,fcs[31:8]};
     if(index==3)begin index<=0;state<=GAP;end else index<=index+1'b1;
    end
    GAP: begin
     if(index==11)begin
      state<=IDLE;packet_done<=1;packet_good<=1;
      if(!(&tx_packets))tx_packets<=tx_packets+1'b1;
     end else index<=index+1'b1;
    end
    default:state<=IDLE;
   endcase
  end
 end
 // synthesis translate_off
 initial if(!PARAM_OK)$fatal(1,"nv_video_tx: invalid geometry/packet parameters");
 // synthesis translate_on
endmodule
