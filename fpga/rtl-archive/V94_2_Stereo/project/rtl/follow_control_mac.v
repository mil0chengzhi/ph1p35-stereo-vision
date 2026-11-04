`timescale 1ns/1ps
// Fixed, point-to-point Ethernet II control channel. No RAM, IP/UDP or ARP.
// Byte order: MAC/header and payload MSB first; Ethernet FCS LSB first.
// tx_request is a pulse in tx_clk, ignored while tx_busy. The caller must
// cross clock domains before using this interface. RX commits only after DV
// falls following an exact 64-byte body+FCS and all checks pass.
module follow_control_mac #(
    parameter [47:0] LOCAL_MAC = 48'h020000000001,
    parameter [47:0] PEER_MAC = 48'h020000000002,
    parameter [7:0] LOCAL_ROLE = 8'h01,
    parameter [7:0] PEER_ROLE = 8'h02,
    parameter VIEW_TX_ONLY = 0,HELD_TX_PAYLOAD=0,DIRECT_RX_PAYLOAD=0
) (
    input wire tx_clk,
    input wire tx_rst_n,
    input wire tx_request,
    input wire [255:0] tx_payload,
    output wire tx_busy,
    output reg tx_done,
    output reg [7:0] gmii_txd,
    output reg gmii_tx_en,
    input wire rx_clk,
    input wire rx_rst_n,
    input wire [7:0] gmii_rxd,
    input wire gmii_rx_dv,
    input wire gmii_rx_er,
    output reg rx_valid,
    output wire [255:0] rx_payload,
    output reg [31:0] rx_good,
    output reg [31:0] rx_bad_crc,
    output reg [31:0] rx_rejected
);
    function [31:0] crc32_byte;
        input [31:0] previous;
        input [7:0] data;
        reg [31:0] value;
        integer bit_number;
        begin
            value = previous ^ {24'b0, data};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
                if (value[0]) value = (value >> 1) ^ 32'hEDB88320;
                else value = value >> 1;
            crc32_byte = value;
        end
    endfunction

    function [7:0] header_byte;
        input [6:0] index;
        input [47:0] destination;
        input [47:0] source;
        input [7:0] role;
        begin
            case (index)
                0: header_byte = destination[47:40];
                1: header_byte = destination[39:32];
                2: header_byte = destination[31:24];
                3: header_byte = destination[23:16];
                4: header_byte = destination[15:8];
                5: header_byte = destination[7:0];
                6: header_byte = source[47:40];
                7: header_byte = source[39:32];
                8: header_byte = source[31:24];
                9: header_byte = source[23:16];
                10: header_byte = source[15:8];
                11: header_byte = source[7:0];
                12: header_byte = 8'h88;
                13: header_byte = 8'hB6;
                14: header_byte = 8'h54; // T
                15: header_byte = 8'h47; // G
                16: header_byte = 8'h32; // 2
                17: header_byte = 8'h36; // 6
                18: header_byte = 8'h01; // protocol version
                19: header_byte = role;
                20: header_byte = 8'h00;
                21: header_byte = 8'h20; // 32 payload bytes
                default: header_byte = 8'h00;
            endcase
        end
    endfunction

    localparam [2:0] TX_IDLE=0, TX_PREAMBLE=1, TX_BODY=2, TX_FCS=3, TX_GAP=4;
    reg [2:0] tx_state;
    reg [6:0] tx_index;
    reg [255:0] tx_shift;
    reg[40:0]tx_view;
    function[7:0]view_byte(input[6:0]index,input[40:0]record);
     begin case(index)
      22:view_byte=8'h56;23:view_byte=8'h49;24:view_byte=8'h45;25:view_byte=8'h57;
      26:view_byte={7'd0,record[40]};27:view_byte=record[39:32];28:view_byte=record[31:24];
      29:view_byte=record[23:16];30:view_byte=record[15:8];31:view_byte=record[7:0];
      32:view_byte=8'hc0;default:view_byte=0;
     endcase end
    endfunction
    reg [31:0] tx_crc;
    reg [31:0] tx_fcs;
    wire[4:0]held_byte=5'd21-tx_index[4:0];
    wire[7:0]serial_payload=HELD_TX_PAYLOAD?tx_payload[{held_byte,3'b000}+:8]:tx_shift[255:248];
    // Next-byte selection runs in parallel with the current-byte CRC.
    (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg[7:0]tx_body_byte;
    wire[6:0]next_body_index=tx_index+7'd1;
    wire[4:0]next_held_byte=5'd21-next_body_index[4:0];
    wire[7:0]next_serial=HELD_TX_PAYLOAD?tx_payload[{next_held_byte,3'b000}+:8]:
        (tx_index<22?tx_shift[255:248]:tx_shift[247:240]);
    wire[7:0]next_body_byte=next_body_index<22?header_byte(next_body_index,PEER_MAC,LOCAL_MAC,LOCAL_ROLE):
        (VIEW_TX_ONLY?view_byte(next_body_index,tx_view):(next_body_index<54?next_serial:8'd0));
    wire[31:0]tx_crc_next=crc32_byte(tx_crc,tx_body_byte);
    assign tx_busy = (tx_state != TX_IDLE);

    always @(posedge tx_clk or negedge tx_rst_n) begin
        if (!tx_rst_n) begin
            tx_body_byte<=0;
            tx_state <= TX_IDLE;
            tx_index <= 0;
            tx_shift <= 0;tx_view<=0;
            tx_crc <= 32'hFFFFFFFF;
            tx_fcs <= 0;
            gmii_txd <= 0;
            gmii_tx_en <= 0;
            tx_done <= 0;
        end else begin
            tx_done <= 0;
            case (tx_state)
                TX_IDLE: begin
                    gmii_tx_en <= 0;
                    gmii_txd <= 0;
                    if (tx_request) begin
                        if(VIEW_TX_ONLY)tx_view<=tx_payload[216:176];else if(!HELD_TX_PAYLOAD)tx_shift <= tx_payload;
                        tx_index <= 0;
                        tx_crc <= 32'hFFFFFFFF;
                        tx_state <= TX_PREAMBLE;
                    end
                end
                TX_PREAMBLE: begin
                    gmii_tx_en <= 1;
                    gmii_txd <= (tx_index == 7) ? 8'hD5 : 8'h55;
                    if (tx_index == 7) begin
                        tx_body_byte<=header_byte(0,PEER_MAC,LOCAL_MAC,LOCAL_ROLE);
                        tx_index <= 0;
                        tx_state <= TX_BODY;
                    end else tx_index <= tx_index + 1'b1;
                end
                TX_BODY: begin
                    tx_body_byte<=next_body_byte;
                    gmii_tx_en <= 1;
                    gmii_txd <= tx_body_byte;
                    tx_crc <= tx_crc_next;
                    if (!VIEW_TX_ONLY && !HELD_TX_PAYLOAD && (tx_index >= 22) && (tx_index < 54))
                        tx_shift <= {tx_shift[247:0], 8'h00};
                    if (tx_index == 59) begin
                        tx_fcs <= ~tx_crc_next;
                        tx_index <= 0;
                        tx_state <= TX_FCS;
                    end else tx_index <= tx_index + 1'b1;
                end
                TX_FCS: begin
                    gmii_tx_en <= 1;
                    gmii_txd <= tx_fcs[7:0];
                    tx_fcs <= {8'h00, tx_fcs[31:8]};
                    if (tx_index == 3) begin
                        tx_index <= 0;
                        tx_state <= TX_GAP;
                    end else tx_index <= tx_index + 1'b1;
                end
                TX_GAP: begin
                    gmii_tx_en <= 0;
                    gmii_txd <= 0;
                    if (tx_index == 0) tx_done <= 1;
                    // Twelve idle bytes are mandatory even if requested sooner.
                    if (tx_index == 11) begin
                        tx_index <= 0;
                        tx_state <= TX_IDLE;
                    end else tx_index <= tx_index + 1'b1;
                end
                default: begin
                    tx_state <= TX_GAP;
                    tx_index <= 0;
                    gmii_tx_en <= 0;
                    gmii_txd <= 0;
                end
            endcase
        end
    end

    localparam [2:0] RX_IDLE=0, RX_PREAMBLE=1, RX_BODY=2, RX_END=3, RX_DROP=4;
    reg [2:0] rx_state;
    reg [6:0] rx_index;
    reg [31:0] rx_crc;
    reg [255:0] rx_shift;
    reg[255:0]rx_committed;
    // Direct mode is valid only at rx_valid; a source mailbox captures this
    // same-clock record before the following packet can reuse rx_shift.
    assign rx_payload=DIRECT_RX_PAYLOAD?rx_shift:rx_committed;
    reg rx_frame_bad;
    wire [7:0] rx_expected_header =
        header_byte(rx_index, LOCAL_MAC, PEER_MAC, PEER_ROLE);

    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            rx_state <= RX_IDLE;
            rx_index <= 0;
            rx_crc <= 32'hFFFFFFFF;
            rx_shift <= 0;
            rx_frame_bad <= 0;
            rx_valid <= 0;
            rx_committed <= 0;
            rx_good <= 0;
            rx_bad_crc <= 0;
            rx_rejected <= 0;
        end else begin
            rx_valid <= 0;
            case (rx_state)
                RX_IDLE: begin
                    if (gmii_rx_dv) begin
                        rx_index <= 1;
                        rx_crc <= 32'hFFFFFFFF;
                        rx_frame_bad <= 0;
                        if ((gmii_rxd == 8'h55) && !gmii_rx_er)
                            rx_state <= RX_PREAMBLE;
                        else rx_state <= RX_DROP;
                    end
                end
                RX_PREAMBLE: begin
                    if (!gmii_rx_dv) begin
                        rx_rejected <= rx_rejected + 1'b1;
                        rx_state <= RX_IDLE;
                    end else if (gmii_rx_er ||
                        (gmii_rxd != ((rx_index == 7) ? 8'hD5 : 8'h55))) begin
                        rx_state <= RX_DROP;
                    end else if (rx_index == 7) begin
                        rx_index <= 0;
                        rx_shift <= 0;
                        rx_state <= RX_BODY;
                    end else rx_index <= rx_index + 1'b1;
                end
                RX_BODY: begin
                    if (!gmii_rx_dv) begin
                        rx_rejected <= rx_rejected + 1'b1;
                        rx_state <= RX_IDLE;
                    end else begin
                        rx_crc <= crc32_byte(rx_crc, gmii_rxd);
                        if (gmii_rx_er ||
                            ((rx_index < 22) && (gmii_rxd != rx_expected_header)) ||
                            ((rx_index >= 54) && (rx_index < 60) && (gmii_rxd != 0)))
                            rx_frame_bad <= 1;
                        if ((rx_index >= 22) && (rx_index < 54))
                            rx_shift <= {rx_shift[247:0], gmii_rxd};
                        if (rx_index == 63) rx_state <= RX_END;
                        else rx_index <= rx_index + 1'b1;
                    end
                end
                RX_END: begin
                    if (gmii_rx_dv) begin
                        // Never resynchronize inside a malformed/overlong frame.
                        rx_state <= RX_DROP;
                    end else begin
                        // Including the four FCS bytes leaves this raw residue.
                        if (!rx_frame_bad && (rx_crc == 32'hDEBB20E3)) begin
                            rx_committed <= rx_shift;
                            rx_valid <= 1;
                            rx_good <= rx_good + 1'b1;
                        end else begin
                            rx_rejected <= rx_rejected + 1'b1;
                            if (rx_crc != 32'hDEBB20E3)
                                rx_bad_crc <= rx_bad_crc + 1'b1;
                        end
                        rx_state <= RX_IDLE;
                    end
                end
                RX_DROP: begin
                    if (!gmii_rx_dv) begin
                        rx_rejected <= rx_rejected + 1'b1;
                        rx_state <= RX_IDLE;
                    end
                end
                default: rx_state <= RX_DROP;
            endcase
        end
    end
endmodule
