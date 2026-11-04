// Two-clock command/result mailbox and one immutable native720 RGB565 row cache.
// Both resets MUST be coordinated with the memory controller and all bus clients.
// Bundled payload registers remain frozen until the round-trip toggle completes.
// Integration must constrain these CDC paths; two-flop toggles alone are not STA.
module snapshot_uart_mailbox(
    input wire csr_clk,csr_rst,input wire[15:0]csr_address,
    input wire csr_write,input wire[7:0]csr_write_data,
    output reg[7:0]csr_read_data,output wire csr_selected,
    input wire mem_clk,mem_rst,
    output reg mem_command_valid,input wire mem_command_ready,
    output reg[159:0]mem_command,
    input wire mem_done,input wire[255:0]mem_response,input wire mem_fault,
    input wire mem_ram_write,input wire[7:0]mem_ram_address,input wire[127:0]mem_ram_data
);
    reg camera;
    reg[15:0]row;
    reg[31:0]session,epoch_A,epoch_B,tag,accepted,rejected,completed;
    reg busy,row_valid;
    reg[159:0]command_frozen;
    reg[255:0]response_frozen,response_csr;
    reg request_toggle,ack_toggle;
    (* async_reg="true" *)reg request_m1,request_m2,ack_c1,ack_c2,fault_c1,fault_c2;
    reg[1:0]mem_state;
    localparam M_IDLE=0,M_COMMAND=1,M_WAIT=2;
    wire command_write=csr_write&&csr_address==16'h0301;
    wire valid_opcode=csr_write_data>=1&&csr_write_data<=4;
    wire pixel_address=csr_address>=16'h4000&&csr_address<16'h4a00;
    wire[7:0]response_offset=csr_address[7:0]-8'h20;
    wire[7:0]word_offset=csr_address[7:0];
    reg[127:0]row_ram[0:159];
    reg[127:0]row_read_word;
    reg[3:0]row_read_lane;
    assign csr_selected=(csr_address>=16'h0300&&csr_address<16'h0360)||pixel_address;
    // 2560 bytes = 160 x128 bits. Never clear the RAM with a reset loop.
    always @(posedge mem_clk)begin
        if(!mem_rst&&mem_state==M_WAIT&&mem_command[159:152]==4&&mem_ram_write&&mem_ram_address<160)
            row_ram[mem_ram_address]<=mem_ram_data;
    end
    always @(posedge csr_clk)begin
        if(pixel_address)row_read_word<=row_ram[csr_address[11:4]];
        row_read_lane<=csr_address[3:0];
    end
    always @(posedge mem_clk)begin
        if(mem_rst)begin
            request_m1<=0;request_m2<=0;ack_toggle<=0;mem_state<=M_IDLE;
            mem_command_valid<=0;mem_command<=0;response_frozen<=0;
        end else begin
            request_m1<=request_toggle;request_m2<=request_m1;
            case(mem_state)
                M_IDLE:if(request_m2!=ack_toggle)begin
                    mem_command<=command_frozen;mem_command_valid<=1;mem_state<=M_COMMAND;
                end
                M_COMMAND:if(mem_command_valid&&mem_command_ready)begin
                    mem_command_valid<=0;mem_state<=M_WAIT;
                end
                M_WAIT:if(mem_done)begin
                    response_frozen<=mem_response;ack_toggle<=request_m2;mem_state<=M_IDLE;
                end
                default:begin mem_command_valid<=0;mem_state<=M_IDLE;end
            endcase
        end
    end
    always @(posedge csr_clk)begin
        if(csr_rst)begin
            camera<=0;row<=0;session<=0;epoch_A<=0;epoch_B<=0;tag<=0;
            accepted<=0;rejected<=0;completed<=0;busy<=0;row_valid<=0;
            command_frozen<=0;response_csr<=0;request_toggle<=0;
            ack_c1<=0;ack_c2<=0;fault_c1<=0;fault_c2<=0;
        end else begin
            ack_c1<=ack_toggle;ack_c2<=ack_c1;fault_c1<=mem_fault;fault_c2<=fault_c1;
            if(fault_c2)row_valid<=0;
            if(busy&&ack_c2==request_toggle)begin
                response_csr<=response_frozen;completed<=completed+1'b1;busy<=0;
                row_valid<=response_frozen[7:0]==4&&response_frozen[15:8]==0&&!fault_c2;
            end
            if(command_write)begin
                if(!busy&&!fault_c2&&valid_opcode)begin
                    // [159:152]opcode [151]camera [150:135]row [134:103]session
                    // [102:71]epochA [70:39]epochB [38:7]tag [6:0]reserved.
                    command_frozen<={csr_write_data,camera,row,session,epoch_A,epoch_B,tag,7'd0};
                    request_toggle<=~request_toggle;busy<=1;row_valid<=0;accepted<=accepted+1'b1;
                end else rejected<=rejected+1'b1;
            end
            if(csr_write)begin
                if(csr_address==16'h0302)camera<=csr_write_data[0];
                if(csr_address>=16'h0303&&csr_address<=16'h0304)row[(csr_address-16'h0303)*8+:8]<=csr_write_data;
                if(csr_address>=16'h0305&&csr_address<=16'h0308)session[(csr_address-16'h0305)*8+:8]<=csr_write_data;
                if(csr_address>=16'h0309&&csr_address<=16'h030c)epoch_A[(csr_address-16'h0309)*8+:8]<=csr_write_data;
                if(csr_address>=16'h030d&&csr_address<=16'h0310)epoch_B[(csr_address-16'h030d)*8+:8]<=csr_write_data;
                if(csr_address>=16'h0311&&csr_address<=16'h0314)tag[(csr_address-16'h0311)*8+:8]<=csr_write_data;
            end
        end
    end
    always @*begin
        csr_read_data=0;
        if(pixel_address)begin
            if(row_valid&&!busy&&!fault_c2)csr_read_data=row_read_word[row_read_lane*8+:8];
        end else if(csr_address>=16'h0320&&csr_address<=16'h033f)
            csr_read_data=response_csr[response_offset*8+:8];
        else case(csr_address)
            16'h0300:csr_read_data={4'd0,fault_c2,response_csr[15:8]!=0,row_valid,busy};
            16'h0302:csr_read_data={7'd0,camera};
            16'h0340:csr_read_data="S";16'h0341:csr_read_data="D";
            16'h0342:csr_read_data="G";16'h0343:csr_read_data="1";
            16'h0344:csr_read_data=8'h00;16'h0345:csr_read_data=8'h05; // width1280 LE
            16'h0346:csr_read_data=8'hd0;16'h0347:csr_read_data=8'h02; // height720 LE
            16'h0348:csr_read_data=8'h00;16'h0349:csr_read_data=8'h0a; // row2560 bytes
            default:begin
                if(csr_address>=16'h0303&&csr_address<=16'h0304)csr_read_data=row[(csr_address-16'h0303)*8+:8];
                if(csr_address>=16'h0305&&csr_address<=16'h0308)csr_read_data=session[(csr_address-16'h0305)*8+:8];
                if(csr_address>=16'h0309&&csr_address<=16'h030c)csr_read_data=epoch_A[(csr_address-16'h0309)*8+:8];
                if(csr_address>=16'h030d&&csr_address<=16'h0310)csr_read_data=epoch_B[(csr_address-16'h030d)*8+:8];
                if(csr_address>=16'h0311&&csr_address<=16'h0314)csr_read_data=tag[(csr_address-16'h0311)*8+:8];
                if(csr_address>=16'h0315&&csr_address<=16'h0318)csr_read_data=accepted[(csr_address-16'h0315)*8+:8];
                if(csr_address>=16'h0319&&csr_address<=16'h031c)csr_read_data=rejected[(csr_address-16'h0319)*8+:8];
                if(csr_address>=16'h034c&&csr_address<=16'h034f)csr_read_data=completed[(csr_address-16'h034c)*8+:8];
            end
        endcase
    end
endmodule
