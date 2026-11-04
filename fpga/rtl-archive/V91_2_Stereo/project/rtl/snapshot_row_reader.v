// Read one immutable native720 RGB565 row, verify lease and produce CRC32.
// RAM writes are internal 128-bit words; no UART or clock-domain crossing here.
module snapshot_row_reader #(
    parameter integer TIMEOUT=1000000,
    parameter integer SNAPSHOT_BASE=12582912,SLOT_WORDS=2097152,
    parameter integer PROTECTED_END=12582912,ADDRESS_WORDS=33554432
)(
    input wire clk,rst,input wire request_valid,output wire request_ready,
    input wire request_camera,input wire[9:0]request_row,
    input wire[31:0]request_epoch,request_tag,
    input wire[1:0]lease_valid,input wire[31:0]lease_epoch_A,lease_epoch_B,
    output wire[24:0]araddr,output wire arvalid,input wire arready,
    input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,read_active,
    output reg ram_write,output reg[7:0]ram_address,output reg[127:0]ram_data,
    output reg result_done,result_good,result_camera,output reg[9:0]result_row,
    output reg[31:0]result_epoch,result_tag,result_crc32,
    output reg[3:0]result_error,output reg fault,output wire configuration_error
);
    localparam CONFIG_OK=TIMEOUT>=32&&SNAPSHOT_BASE>=PROTECTED_END&&SNAPSHOT_BASE%8==0&&
        SLOT_WORDS>=1280*720&&SLOT_WORDS%8==0&&SNAPSHOT_BASE+2*SLOT_WORDS<=ADDRESS_WORDS&&ADDRESS_WORDS<=33554432;
    localparam IDLE=0,CMD=1,DATA=2,CRC=3,EMIT=4,DRAIN=5,FAILED=6;
    reg[2:0]state;
    reg camera,pending,cancelled;
    reg[31:0]epoch,tag,age,crc;
    reg[9:0]row;
    reg[7:0]index;
    reg[3:0]byte_index;
    reg[24:0]row_address;
    reg[127:0]crc_word;
    wire lease_matches=lease_valid[camera]&&(camera?lease_epoch_B:lease_epoch_A)==epoch;
    wire request_matches=lease_valid[request_camera]&&(request_camera?lease_epoch_B:lease_epoch_A)==request_epoch;
    wire[24:0]request_y={15'd0,request_row};
    function[31:0]crc_byte;
        input[31:0]c;input[7:0]b;reg[31:0]v;integer n;
        begin v=c^{24'd0,b};for(n=0;n<8;n=n+1)v=(v>>1)^(v[0]?32'hedb88320:32'd0);crc_byte=v;end
    endfunction
    wire[31:0]crc_next=crc_byte(crc,crc_word[byte_index*8+:8]);
    assign configuration_error=!CONFIG_OK;
    assign request_ready=CONFIG_OK&&!rst&&!fault&&state==IDLE;
    assign araddr=row_address+{14'd0,index,3'b000};
    // Once presented, an AR remains valid and stable until accepted, even if
    // the lease disappears or the watchdog expires while the bus is stalled.
    assign arvalid=!rst&&state==CMD;
    assign rready=!rst&&(pending||fault);
    assign read_active=!rst&&(state==CMD||state==DATA||state==CRC||pending);
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;camera<=0;pending<=0;cancelled<=0;epoch<=0;tag<=0;age<=0;crc<=32'hffffffff;
            row<=0;index<=0;byte_index<=0;row_address<=0;crc_word<=0;
            ram_write<=0;ram_address<=0;ram_data<=0;result_done<=0;result_good<=0;
            result_camera<=0;result_row<=0;result_epoch<=0;result_tag<=0;result_crc32<=0;result_error<=0;fault<=0;
        end else begin
            ram_write<=0;result_done<=0;
            if(fault)begin
                if(arvalid&&arready)begin pending<=1;state<=FAILED;end
                if(rvalid&&rready&&rlast)pending<=0;
            end else if(state!=IDLE&&state!=EMIT&&age==TIMEOUT-1)begin
                fault<=1;result_good<=0;result_error<=4;state<=arvalid&&!arready?CMD:FAILED;
                if(arvalid&&arready)pending<=1;
                if(rvalid&&rready&&rlast)pending<=0;
            end else if(rvalid&&state!=DATA&&state!=DRAIN)begin
                fault<=1;result_good<=0;result_error<=5;state<=arvalid&&!arready?CMD:FAILED;
                if(arvalid&&arready)pending<=1;
                if(rready&&rlast)pending<=0;
            end else if((state==CMD||state==DATA||state==CRC)&&!lease_matches)begin
                result_good<=0;result_error<=3;cancelled<=1;age<=age+1'b1;
                if(state==CMD)begin
                    if(arready)begin pending<=1;state<=DRAIN;end
                end else if(pending)begin
                    if(rvalid&&rready&&rlast)begin pending<=0;state<=EMIT;end
                    else state<=DRAIN;
                end else state<=EMIT;
            end else begin
                if(state==IDLE)age<=0;else age<=age+1'b1;
                case(state)
                    IDLE:if(request_valid&&request_ready)begin
                        camera<=request_camera;row<=request_row;epoch<=request_epoch;tag<=request_tag;
                        result_camera<=request_camera;result_row<=request_row;result_epoch<=request_epoch;result_tag<=request_tag;
                        result_good<=0;result_error<=0;result_crc32<=0;index<=0;byte_index<=0;crc<=32'hffffffff;cancelled<=0;
                        row_address<=SNAPSHOT_BASE+(request_camera?SLOT_WORDS:0)+(request_y<<10)+(request_y<<8);
                        if(request_row>=720)begin result_error<=1;state<=EMIT;end
                        else if(!request_matches)begin result_error<=2;state<=EMIT;end
                        else state<=CMD;
                    end
                    CMD:if(arvalid&&arready)begin pending<=1;state<=cancelled?DRAIN:DATA;end
                    DATA:if(rvalid&&rready)begin
                        if(!rlast)begin fault<=1;result_error<=6;result_good<=0;state<=FAILED;end
                        else begin
                            pending<=0;ram_write<=1;ram_address<=index;ram_data<=rdata;
                            crc_word<=rdata;byte_index<=0;state<=CRC;
                        end
                    end
                    CRC:begin
                        crc<=crc_next;
                        if(byte_index==15)begin
                            if(index==159)begin result_crc32<=~crc_next;result_good<=1;state<=EMIT;end
                            else begin index<=index+1'b1;state<=CMD;end
                        end else byte_index<=byte_index+1'b1;
                    end
                    DRAIN:if(rvalid&&rready&&rlast)begin pending<=0;state<=EMIT;end
                    EMIT:begin
                        if(result_good&&!lease_matches)begin result_good<=0;result_error<=3;end
                        result_done<=1;state<=IDLE;
                    end
                    default:begin fault<=1;result_error<=7;state<=FAILED;end
                endcase
            end
        end
    end
endmodule
