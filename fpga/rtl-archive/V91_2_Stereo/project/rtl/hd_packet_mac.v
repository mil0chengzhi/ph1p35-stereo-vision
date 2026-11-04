`timescale 1ns/1ps
// HD72 Ethernet channel. CRC covers MAC header, HD header, payload and padding.
// A DATA packet owns its1280-byte external RAM until the consumer acknowledges.
module hd_packet_mac #(
 parameter[47:0]LOCAL_MAC=48'h020000000002,PEER_MAC=48'h020000000001,
 parameter[31:0]CALIBRATION_ID=32'h4f87fe40
)(
 input wire tx_clk,tx_rst_n,tx_start,output wire tx_ready,tx_busy,
 input wire[7:0]tx_kind,input wire[31:0]tx_job,input wire[15:0]tx_sequence,
 output wire[10:0]tx_payload_address,input wire[7:0]tx_payload_data,
 output wire[7:0]gmii_txd,output wire gmii_tx_en,
 input wire rx_clk,rx_rst_n,input wire[7:0]gmii_rxd,input wire gmii_rx_dv,gmii_rx_er,
 input wire packet_ready,output reg packet_valid,output wire[191:0]packet_header,
 output wire payload_write,output wire[10:0]payload_address,output wire[7:0]payload_data,
 output reg[31:0]rx_good,rx_bad_crc,rx_rejected
);
 function[31:0]crc_byte(input[31:0]previous,input[7:0]value);
  integer i;reg[31:0]c;begin c=previous;for(i=0;i<8;i=i+1)begin if(c[0]^value[i])c=(c>>1)^32'hedb88320;else c=c>>1;end crc_byte=c;end
 endfunction
 localparam T_IDLE=0,T_PRE=1,T_BODY=2,T_FCS=3,T_GAP=4;
 reg[2:0]ts;reg[10:0]ti,body_length,payload_length;reg[7:0]kind_q;reg[31:0]job_q,tx_crc;reg[15:0]sequence_q;reg[3:0]gap;
 function[7:0]header_byte(input[15:0]index);
  reg[15:0]h;begin
   if(index<6)header_byte=PEER_MAC[(5-index)*8+:8];
   else if(index<12)header_byte=LOCAL_MAC[(11-index)*8+:8];
   else if(index==12)header_byte=8'h88;else if(index==13)header_byte=8'hb7;
   else begin h=index-14;
    case(h)
     0:header_byte=8'h48;1:header_byte=8'h44;2:header_byte=8'h37;3:header_byte=8'h32;
     4:header_byte=1;5:header_byte=kind_q;6:header_byte={5'd0,payload_length[10:8]};7:header_byte=payload_length[7:0];
     8:header_byte=job_q[31:24];9:header_byte=job_q[23:16];10:header_byte=job_q[15:8];11:header_byte=job_q[7:0];
     12:header_byte=sequence_q[15:8];13:header_byte=sequence_q[7:0];14:header_byte=8'h0b;15:header_byte=8'h40;
     16:header_byte=CALIBRATION_ID[31:24];17:header_byte=CALIBRATION_ID[23:16];18:header_byte=CALIBRATION_ID[15:8];19:header_byte=CALIBRATION_ID[7:0];
     20:header_byte=0;21:header_byte=1;22:header_byte=0;23:header_byte=0;default:header_byte=0;
    endcase
   end
  end
 endfunction
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg[7:0]body_byte;
 wire[10:0]next_body_index=ti+11'd1;
 wire[7:0]next_body_byte=next_body_index<38?header_byte({5'd0,next_body_index}):
     (next_body_index<38+payload_length?tx_payload_data:8'd0);
 wire[31:0]fcs=~tx_crc;
 assign tx_ready=tx_rst_n&&ts==T_IDLE;
 assign tx_busy=ts!=T_IDLE;
 // Two-byte address lookahead includes the new body-byte register.
 assign tx_payload_address=ti>=36&&ti<36+payload_length?ti-36:0;
 assign gmii_tx_en=tx_rst_n&&(ts==T_PRE||ts==T_BODY||ts==T_FCS);
 assign gmii_txd=ts==T_PRE?(ti==7?8'hd5:8'h55):ts==T_BODY?body_byte:ts==T_FCS?(fcs>>(ti*8)):8'd0;
 always @(posedge tx_clk)begin
  if(!tx_rst_n)begin body_byte<=0;ts<=T_IDLE;ti<=0;body_length<=0;payload_length<=0;kind_q<=0;job_q<=0;sequence_q<=0;tx_crc<=32'hffffffff;gap<=0;end
  else case(ts)
   T_IDLE:if(tx_start&&tx_ready&&tx_job!=0&&tx_job[31:22]==0&&tx_kind>=1&&tx_kind<=4&&
    ((tx_kind==1&&tx_sequence<2880)||(tx_kind!=1&&tx_sequence<=2880)))begin
    kind_q<=tx_kind;job_q<=tx_job;sequence_q<=tx_sequence;payload_length<=tx_kind==1?16'd1280:16'd0;
    body_length<=tx_kind==1?16'd1318:16'd60;ti<=0;tx_crc<=32'hffffffff;ts<=T_PRE;
   end
   T_PRE:if(ti==7)begin body_byte<=header_byte(0);ti<=0;ts<=T_BODY;end else ti<=ti+1'b1;
   T_BODY:begin body_byte<=next_body_byte;tx_crc<=crc_byte(tx_crc,body_byte);if(ti==body_length-1)begin ti<=0;ts<=T_FCS;end else ti<=ti+1'b1;end
   T_FCS:if(ti==3)begin gap<=0;ts<=T_GAP;end else ti<=ti+1'b1;
   T_GAP:if(gap==11)ts<=T_IDLE;else gap<=gap+1'b1;
   default:ts<=T_IDLE;
  endcase
 end
 localparam R_IDLE=0,R_PRE=1,R_BODY=2,R_DROP=3;
 reg[1:0]rs;reg[3:0]preamble_count;reg[10:0]count;
 reg[2:0]rx_kind;reg[21:0]job;reg[11:0]sequence_number;
 reg[31:0]rx_crc;reg bad,capture;
 wire[10:0]expected_body=rx_kind==1?11'd1318:11'd60;
 assign packet_header={32'h48443732,8'd1,5'd0,rx_kind,(rx_kind==1?16'd1280:16'd0),10'd0,job,4'd0,sequence_number,16'd2880,CALIBRATION_ID,16'd1,16'd0};
 reg[7:0]expected_byte;reg constant_byte;
 always @*begin
  expected_byte=0;constant_byte=1;
  case(count)
   11'd0:expected_byte=LOCAL_MAC[47-:8];
   11'd1:expected_byte=LOCAL_MAC[39-:8];
   11'd2:expected_byte=LOCAL_MAC[31-:8];
   11'd3:expected_byte=LOCAL_MAC[23-:8];
   11'd4:expected_byte=LOCAL_MAC[15-:8];
   11'd5:expected_byte=LOCAL_MAC[7-:8];
   11'd6:expected_byte=PEER_MAC[47-:8];
   11'd7:expected_byte=PEER_MAC[39-:8];
   11'd8:expected_byte=PEER_MAC[31-:8];
   11'd9:expected_byte=PEER_MAC[23-:8];
   11'd10:expected_byte=PEER_MAC[15-:8];
   11'd11:expected_byte=PEER_MAC[7-:8];
   11'd12:expected_byte=8'h88;
   11'd13:expected_byte=8'hb7;
   11'd14:expected_byte=8'h48;
   11'd15:expected_byte=8'h44;
   11'd16:expected_byte=8'h37;
   11'd17:expected_byte=8'h32;
   11'd18:expected_byte=8'd1;
   11'd20:expected_byte=(rx_kind==1?8'h05:8'h00);
   11'd21:expected_byte=8'd0;
   11'd22:expected_byte=8'd0;
   11'd28:expected_byte=8'h0b;
   11'd29:expected_byte=8'h40;
   11'd30:expected_byte=CALIBRATION_ID[31-:8];
   11'd31:expected_byte=CALIBRATION_ID[23-:8];
   11'd32:expected_byte=CALIBRATION_ID[15-:8];
   11'd33:expected_byte=CALIBRATION_ID[7-:8];
   11'd34:expected_byte=8'd0;
   11'd35:expected_byte=8'd1;
   11'd36:expected_byte=8'd0;
   11'd37:expected_byte=8'd0;
   default:constant_byte=0;
  endcase
 end
 wire header_ok=rx_kind>=1&&rx_kind<=4&&job!=0&&(rx_kind==1?sequence_number<2880:sequence_number<=2880);
 assign payload_write=rx_rst_n&&rs==R_BODY&&gmii_rx_dv&&capture&&!bad&&count>=38&&count<1318&&rx_kind==1;
 assign payload_address=count-38;assign payload_data=gmii_rxd;
 always @(posedge rx_clk)begin
  if(!rx_rst_n)begin rs<=R_IDLE;preamble_count<=0;count<=0;rx_kind<=0;job<=0;sequence_number<=0;rx_crc<=32'hffffffff;bad<=0;capture<=0;packet_valid<=0;rx_good<=0;rx_bad_crc<=0;rx_rejected<=0;end
  else begin
   packet_valid<=0;
   case(rs)
    R_IDLE:if(gmii_rx_dv)begin
     if(gmii_rxd==8'h55&&!gmii_rx_er)begin preamble_count<=1;rs<=R_PRE;end else rs<=R_DROP;
    end
    R_PRE:if(!gmii_rx_dv||gmii_rx_er)begin rs<=R_IDLE;rx_rejected<=rx_rejected+1'b1;end
     else if(preamble_count<7&&gmii_rxd==8'h55)preamble_count<=preamble_count+1'b1;
     else if(preamble_count==7&&gmii_rxd==8'hd5)begin rs<=R_BODY;count<=0;rx_crc<=32'hffffffff;bad<=0;capture<=packet_ready;rx_kind<=0;job<=0;sequence_number<=0;end
     else rs<=R_DROP;
    R_BODY:if(gmii_rx_dv)begin
     if(count<2047)count<=count+1'b1;else bad<=1;
     if(gmii_rx_er||(constant_byte&&gmii_rxd!=expected_byte))bad<=1;
     rx_crc<=crc_byte(rx_crc,gmii_rxd);
     case(count)
      19:begin rx_kind<=gmii_rxd[2:0];if(gmii_rxd<1||gmii_rxd>4)bad<=1;end
      23:begin job[21:16]<=gmii_rxd[5:0];if(gmii_rxd[7:6]!=0)bad<=1;end
      24:job[15:8]<=gmii_rxd;
      25:job[7:0]<=gmii_rxd;
      26:begin sequence_number[11:8]<=gmii_rxd[3:0];if(gmii_rxd[7:4]!=0)bad<=1;end
      27:sequence_number[7:0]<=gmii_rxd;
     endcase
    end else begin
     rs<=R_IDLE;
     if(rx_crc!=32'hdebb20e3)rx_bad_crc<=rx_bad_crc+1'b1;
     else if(bad||!capture||!header_ok||count!=expected_body+4)rx_rejected<=rx_rejected+1'b1;
     else begin packet_valid<=1;rx_good<=rx_good+1'b1;end
    end
    R_DROP:if(!gmii_rx_dv)begin rs<=R_IDLE;rx_rejected<=rx_rejected+1'b1;end
    default:rs<=R_IDLE;
   endcase
  end
 end
endmodule
