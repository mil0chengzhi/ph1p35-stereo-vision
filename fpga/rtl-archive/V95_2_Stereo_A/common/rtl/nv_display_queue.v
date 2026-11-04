`timescale 1ns/1ps
// Four private packet slots. Pixels never wait for Ethernet. A full slot at a
// packet boundary cancels the rest of that display frame; the receiver rejects
// its incomplete predecessor when the next index-0 packet arrives.
// Slot payload/metadata stay unchanged from publish until MAC completion ACK.
module nv_display_queue #(
    parameter integer WIDTH=1280, HEIGHT=720, PACKET_PIXELS=640, DYNAMIC_META=0
)(
    input wire pixel_clk,tx_clk,rst_n,output wire tx_domain_rst_n,
    input wire [23:0] rgb,input wire [15:0] x,y,input wire de,
    input wire [31:0] source_frame_id,source_session,
    output reg packet_valid,input wire packet_ready,
    output reg [31:0] packet_session,
    output reg [31:0] packet_frame,output reg [15:0] packet_index,
    input wire [9:0] packet_word_addr,output reg [15:0] packet_word_data,
    input wire packet_done,packet_good,
    output reg [31:0] frames_queued,frames_dropped,frame_counter,
    output reg [31:0] packets_sent,frames_sent,last_frame_sent,
    output reg [2:0] high_water
);
    // This compact packetizer is intentionally two equal packets per line.
    localparam integer PARAM_OK=PACKET_PIXELS>=2&&PACKET_PIXELS<=1024&&WIDTH==2*PACKET_PIXELS&&HEIGHT>0&&HEIGHT<=32767;
    localparam integer PACKETS=WIDTH*HEIGHT/PACKET_PIXELS;
    (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [1:0] pixel_reset,tx_reset;
    wire pixel_rst_n=pixel_reset[1],tx_rst_n=tx_reset[1];
    assign tx_domain_rst_n=tx_rst_n;
    always @(posedge pixel_clk or negedge rst_n)
        if(!rst_n) pixel_reset<=0;else pixel_reset<={pixel_reset[0],1'b1};
    always @(posedge tx_clk or negedge rst_n)
        if(!rst_n) tx_reset<=0;else tx_reset<={tx_reset[0],1'b1};
    localparam SLOT_BITS=$clog2(PACKET_PIXELS);
    (* ram_style="bram" *) reg [15:0] payload[0:(4<<SLOT_BITS)-1];
    (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [31:0] frame_meta[0:3];
    (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [15:0] index_meta[0:3];
    (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [3:0] published,released;
    (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [3:0] release_sync0,release_sync1,publish_sync0,publish_sync1;
    reg [1:0] write_bank,read_bank;
    reg capture_frame,alternate,writing,reading;
    reg [31:0] active_frame,active_session;
    (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg [31:0] session_meta[0:3];
    reg [9:0] write_word;
    wire first_pixel=de&&x==0&&y==0;
    wire boundary=de&&(x==0||x==PACKET_PIXELS);
    wire selected=PARAM_OK&&(first_pixel?!alternate:capture_frame);
    wire slot_free=published[write_bank]==release_sync1[write_bank];
    wire [3:0] occupied=published^release_sync1;
    wire [2:0] used={2'd0,occupied[0]}+{2'd0,occupied[1]}+{2'd0,occupied[2]}+{2'd0,occupied[3]};
    wire take_pixel=pixel_rst_n&&de&&y<HEIGHT&&x<WIDTH&&selected&&
        (boundary?slot_free:writing);
    wire [9:0] payload_write_address=boundary?10'd0:write_word;
    // Constant stream incarnation for this firmware. A reset restarts frame IDs;
    // B accepts only an index-0 boundary and never splices an incomplete frame.
    // Session is latched per source frame and per published packet slot.
    always @(posedge pixel_clk) begin
        if(take_pixel) payload[{write_bank,payload_write_address[SLOT_BITS-1:0]}]<={rgb[23:19],rgb[15:10],rgb[7:3]};
    end
    always @(posedge tx_clk) packet_word_data<=payload[{read_bank,packet_word_addr[SLOT_BITS-1:0]}];
    integer i;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            published<=0;release_sync0<=0;release_sync1<=0;write_bank<=0;
            capture_frame<=0;alternate<=0;writing<=0;active_frame<=0;active_session<=32'h4e563101;write_word<=0;
            frames_queued<=0;frames_dropped<=0;frame_counter<=0;high_water<=0;
            for(i=0;i<4;i=i+1)begin frame_meta[i]<=0;index_meta[i]<=0;session_meta[i]<=32'h4e563101;end
        end else begin
            release_sync0<=released;release_sync1<=release_sync0;
            if(used>high_water)high_water<=used;
            if(first_pixel)begin
                alternate<=!alternate;capture_frame<=!alternate;
                active_frame<=DYNAMIC_META?source_frame_id:frame_counter;
                active_session<=DYNAMIC_META?source_session:32'h4e563101;frame_counter<=frame_counter+1'b1;
                writing<=0;
            end
            if(boundary&&selected)begin
                if(!slot_free)begin
                    capture_frame<=0;writing<=0;frames_dropped<=frames_dropped+1'b1;
                end else begin writing<=1;write_word<=1;end
            end else if(take_pixel)begin
                if(write_word==PACKET_PIXELS-1)begin
                    writing<=0;write_word<=0;
                    frame_meta[write_bank]<=active_frame;session_meta[write_bank]<=active_session;
                    index_meta[write_bank]<=(y<<1)+(x>=PACKET_PIXELS);
                    published[write_bank]<=!published[write_bank];
                    write_bank<=write_bank+1'b1;
                    if(y==HEIGHT-1&&x==WIDTH-1)begin
                        frames_queued<=frames_queued+1'b1;capture_frame<=0;
                    end
                end else write_word<=write_word+1'b1;
            end
        end
    end
    always @(posedge tx_clk or negedge tx_rst_n)begin
        if(!tx_rst_n)begin
            publish_sync0<=0;publish_sync1<=0;released<=0;read_bank<=0;
            packet_valid<=0;packet_frame<=0;packet_index<=0;packet_session<=32'h4e563101;reading<=0;
            packets_sent<=0;frames_sent<=0;last_frame_sent<=0;
        end else begin
            publish_sync0<=published;publish_sync1<=publish_sync0;
            if(!reading&&!packet_valid&&publish_sync1[read_bank]!=released[read_bank])begin
                packet_frame<=frame_meta[read_bank];packet_index<=index_meta[read_bank];packet_session<=session_meta[read_bank];
                packet_valid<=1;
            end
            if(packet_valid&&packet_ready)begin packet_valid<=0;reading<=1;end
            if(reading&&packet_done)begin
                reading<=0;released[read_bank]<=publish_sync1[read_bank];read_bank<=read_bank+1'b1;
                if(packet_good)begin
                    packets_sent<=packets_sent+1'b1;
                    if(packet_index==PACKETS-1)begin frames_sent<=frames_sent+1'b1;last_frame_sent<=packet_frame;end
                end
            end
        end
    end
endmodule
