`timescale 1ns/1ps
// NV32 GMII receiver. Reserve an EXTERNAL PRIVATE slot after EtherType 88B5.
// 88B6 control and other EtherTypes never emit video start/done events.
// packet_start only pulses for a reserved slot. Writes before FCS are speculative.
// packet_done pulses at the DV falling edge for each recognized video type, including
// packets refused for no slot. Publish a slot ONLY on packet_done&&packet_good.
// Metadata is frozen for that packet at done; it is not trusted when good=0.
// Packet/frame/session continuity and DDR frame publication belong to the caller.
module nv32_video_rx #(
 parameter [47:0] LOCAL_MAC=48'h020000002502, PEER_MAC=48'h020000002501,
 parameter integer WIDTH=1280,HEIGHT=720,PACKET_PIXELS=640,
 parameter [7:0] STREAM_ID=8'd1
)(
 input wire clk,rst_n,
 input wire [7:0] gmii_rxd,input wire gmii_rx_dv,gmii_rx_er,packet_accept,
 output reg packet_start,packet_word_write,
 output reg [9:0] packet_word_addr,output reg [15:0] packet_word_data,
 output reg packet_done,packet_good,
 output reg [31:0] packet_session,packet_frame,output reg [15:0] packet_index,
 output reg [31:0] rx_packets,rx_bad_crc,rx_rejected,rx_no_slot
);
 localparam integer PACKETS=WIDTH*HEIGHT/PACKET_PIXELS;
 localparam integer BODY_FCS_BYTES=38+PACKET_PIXELS*2+4;
 localparam integer PARAM_OK=WIDTH>0 && HEIGHT>0 && WIDTH<=16384 && HEIGHT<=16384 &&
   PACKET_PIXELS>0 && PACKET_PIXELS<=1024 && WIDTH%PACKET_PIXELS==0 &&
   PACKETS>0 && PACKETS<=65535 && (STREAM_ID==1 || STREAM_ID==2);

 // Session namespace: kind[31:24], stereo/camera-control[23:22], epoch[21:0].
 // Every pixel is still RGB565. A1 means pre-display ISP RGB565 from A;
 // D1 means final display relay, B2 processed B-camera display, D2 depth display.
 function kind_allowed;
  input[7:0] kind;
  begin kind_allowed=(STREAM_ID==1)?(kind==8'hA1||kind==8'hD1):
                                        (kind==8'hB2||kind==8'hD2);end
 endfunction
 localparam IDLE=0,PREAMBLE=1,BODY=2,DISCARD=3;
 reg [1:0] state;reg [3:0] preamble_count;
 reg [15:0] byte_count;reg [31:0] crc;
 reg reserved,header_bad,header_good,receive_error;reg ethertype_high_ok,video_candidate;
 reg [7:0] flags,low_byte,frame_kind;
 function [31:0] crc32_byte;
  input [31:0] previous;input [7:0] data;
  reg [31:0] value;integer bit_number;
  begin value=previous^{24'b0,data};
   for(bit_number=0;bit_number<8;bit_number=bit_number+1)
    if(value[0])value=(value>>1)^32'hEDB88320;else value=value>>1;
   crc32_byte=value;
  end
 endfunction
 // Bit8 means this byte is invariant and must be checked. All others are
 // session/frame/index/flags and are latched or validated separately.
 function [8:0] expected_byte;
  input [15:0] n;
  begin
   case(n)
    0:expected_byte={1'b1,LOCAL_MAC[47:40]};1:expected_byte={1'b1,LOCAL_MAC[39:32]};
    2:expected_byte={1'b1,LOCAL_MAC[31:24]};3:expected_byte={1'b1,LOCAL_MAC[23:16]};
    4:expected_byte={1'b1,LOCAL_MAC[15:8]};5:expected_byte={1'b1,LOCAL_MAC[7:0]};
    6:expected_byte={1'b1,PEER_MAC[47:40]};7:expected_byte={1'b1,PEER_MAC[39:32]};
    8:expected_byte={1'b1,PEER_MAC[31:24]};9:expected_byte={1'b1,PEER_MAC[23:16]};
    10:expected_byte={1'b1,PEER_MAC[15:8]};11:expected_byte={1'b1,PEER_MAC[7:0]};
    12:expected_byte=9'h188;13:expected_byte=9'h1B5;
    14:expected_byte=9'h14E;15:expected_byte=9'h156;
    16:expected_byte=9'h133;17:expected_byte=9'h132;
    18:expected_byte=9'h102;19:expected_byte={1'b1,STREAM_ID};
    32:expected_byte=9'h100|(PACKETS&255);33:expected_byte=9'h100|((PACKETS>>8)&255);
    34:expected_byte=9'h100|(WIDTH&255);35:expected_byte=9'h100|((WIDTH>>8)&255);
    36:expected_byte=9'h100|(HEIGHT&255);37:expected_byte=9'h100|((HEIGHT>>8)&255);
    default:expected_byte=0;
   endcase
  end
 endfunction
 wire [8:0] expected=expected_byte(byte_count);
 wire mismatch=expected[8] && gmii_rxd!=expected[7:0];
 wire [7:0] expected_flags={4'd0,packet_session[23:22],(packet_index==PACKETS-1),(packet_index==0)};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   state<=IDLE;preamble_count<=0;byte_count<=0;crc<=32'hFFFFFFFF;
   ethertype_high_ok<=0;video_candidate<=0;reserved<=0;header_bad<=0;header_good<=0;receive_error<=0;flags<=0;low_byte<=0;frame_kind<=0;
   packet_start<=0;packet_word_write<=0;packet_word_addr<=0;packet_word_data<=0;
   packet_done<=0;packet_good<=0;packet_session<=0;packet_frame<=0;packet_index<=0;
   rx_packets<=0;rx_bad_crc<=0;rx_rejected<=0;rx_no_slot<=0;
  end else begin
   packet_start<=0;packet_word_write<=0;packet_done<=0;packet_good<=0;
   if(!PARAM_OK)state<=IDLE;
   else case(state)
    IDLE:if(gmii_rx_dv)begin
     if(gmii_rxd==8'h55 && !gmii_rx_er)begin state<=PREAMBLE;preamble_count<=1;end
     else state<=DISCARD;
    end
    PREAMBLE:begin
     if(!gmii_rx_dv)begin state<=IDLE;if(!(&rx_rejected))rx_rejected<=rx_rejected+1'b1;end
     else if(gmii_rx_er)state<=DISCARD;
     else if(gmii_rxd==8'h55 && preamble_count<7)preamble_count<=preamble_count+1'b1;
     else if(gmii_rxd==8'hD5 && preamble_count==7)begin
      state<=BODY;byte_count<=0;crc<=32'hFFFFFFFF;reserved<=0;
      ethertype_high_ok<=0;video_candidate<=0;header_bad<=0;header_good<=0;receive_error<=0;
      packet_session<=0;packet_frame<=0;packet_index<=0;flags<=0;frame_kind<=0;
     end else state<=DISCARD;
    end
    BODY:begin
     if(gmii_rx_dv)begin
      crc<=crc32_byte(crc,gmii_rxd);
      if(byte_count!=16'hFFFF)byte_count<=byte_count+1'b1;
      if(gmii_rx_er)receive_error<=1;
      if(mismatch)header_bad<=1;
      case(byte_count)
       12:ethertype_high_ok<=gmii_rxd==8'h88;
       13:if(ethertype_high_ok&&gmii_rxd==8'hb5)begin
        video_candidate<=1;reserved<=packet_accept;packet_start<=packet_accept;
       end
       20:frame_kind<=gmii_rxd;
       21:flags<=gmii_rxd;
       22:packet_session[7:0]<=gmii_rxd;23:packet_session[15:8]<=gmii_rxd;
       24:packet_session[23:16]<=gmii_rxd;25:packet_session[31:24]<=gmii_rxd;
       26:packet_frame[7:0]<=gmii_rxd;27:packet_frame[15:8]<=gmii_rxd;
       28:packet_frame[23:16]<=gmii_rxd;29:packet_frame[31:24]<=gmii_rxd;
       30:packet_index[7:0]<=gmii_rxd;31:packet_index[15:8]<=gmii_rxd;
       37:header_good<=!header_bad && !mismatch && packet_index<PACKETS && flags==expected_flags && kind_allowed(frame_kind) &&
                       frame_kind==packet_session[31:24] && packet_session[21:0]!=0;
      endcase
      if(reserved && header_good && byte_count>=38 && byte_count<38+PACKET_PIXELS*2)begin
       if(!byte_count[0])low_byte<=gmii_rxd;
       else begin
        packet_word_write<=1;packet_word_addr<=(byte_count-38)>>1;
        packet_word_data<={gmii_rxd,low_byte};
       end
      end
     end else begin
      state<=IDLE;packet_done<=video_candidate;
      if(video_candidate)begin
      if(!reserved)begin if(!(&rx_no_slot))rx_no_slot<=rx_no_slot+1'b1;end
      else if(!header_good || receive_error || byte_count!=BODY_FCS_BYTES)begin
       if(!(&rx_rejected))rx_rejected<=rx_rejected+1'b1;
      end else if(crc!=32'hDEBB20E3)begin
       if(!(&rx_bad_crc))rx_bad_crc<=rx_bad_crc+1'b1;
      end else begin packet_good<=1;if(!(&rx_packets))rx_packets<=rx_packets+1'b1;end
      end
     end
    end
    DISCARD:if(!gmii_rx_dv)begin state<=IDLE;if(!(&rx_rejected))rx_rejected<=rx_rejected+1'b1;end
    default:state<=IDLE;
   endcase
  end
 end
 // synthesis translate_off
 initial if(!PARAM_OK)$fatal(1,"nv32_video_rx: invalid geometry/packet parameters");
 // synthesis translate_on
endmodule
