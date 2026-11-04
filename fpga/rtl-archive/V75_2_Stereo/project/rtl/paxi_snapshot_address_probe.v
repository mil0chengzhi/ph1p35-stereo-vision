// Boot-only targeted alias probe. Run before any video client writes DDR.
// Exercises address bits3..23 and all six video/two snapshot slot boundaries.
// Does not exhaustively test capacity, timing margin, retention or every cell.
module paxi_snapshot_address_probe #(
    parameter integer TIMEOUT=1000000,WRITE_DRAIN=1024
)(
    input wire clk,rst_n,ready,
    output wire[24:0]awaddr,output wire awvalid,input wire awready,
    output wire[127:0]wdata,output wire[15:0]wstrb,output wire wvalid,wlast,input wire wready,
    output wire[24:0]araddr,output wire arvalid,input wire arready,
    input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,
    output reg done,failed,output reg[31:0]errors,output reg[24:0]first_address,
    output reg[127:0]expected,observed,output wire[3:0]phase
);
    localparam WAIT=0,WRITE=1,DRAIN=2,AR=3,DATA=4,FINISH=5,FAILED=6;
    reg[2:0]state;reg[5:0]index;reg pass,aw_sent,w_sent,pending,fatal;
    reg[31:0]age;
    function[24:0]address;
        input[5:0]i;
        begin case(i)
             0:address=25'h0000000; 1:address=25'h0000008; 2:address=25'h0000010;
             3:address=25'h0000020; 4:address=25'h0000040; 5:address=25'h0000080;
             6:address=25'h0000100; 7:address=25'h0000200; 8:address=25'h0000400;
             9:address=25'h0000800;10:address=25'h0001000;11:address=25'h0002000;
            12:address=25'h0004000;13:address=25'h0008000;14:address=25'h0010000;
            15:address=25'h0020000;16:address=25'h0040000;17:address=25'h0080000;
            18:address=25'h0100000;19:address=25'h012bff8;20:address=25'h01ffff8;
            21:address=25'h0200000;22:address=25'h0200008;23:address=25'h032bff8;
            24:address=25'h03ffff8;25:address=25'h0400000;26:address=25'h0400008;
            27:address=25'h052bff8;28:address=25'h05ffff8;29:address=25'h0600000;
            30:address=25'h0600008;31:address=25'h06e0ff8;32:address=25'h07ffff8;
            33:address=25'h0800000;34:address=25'h0800008;35:address=25'h08e0ff8;
            36:address=25'h09ffff8;37:address=25'h0a00000;38:address=25'h0a00008;
            39:address=25'h0ae0ff8;40:address=25'h0bffff8;41:address=25'h0c00000;
            42:address=25'h0c00008;43:address=25'h0ce0ff8;44:address=25'h0dffff8;
            45:address=25'h0e00000;46:address=25'h0e00008;47:address=25'h0ee0ff8;
            48:address=25'h0fffff8;default:address=0;
        endcase end
    endfunction
    function[127:0]pattern;
        input[24:0]a;input p;reg[31:0]v;
        begin v={7'd0,a};pattern={v^32'hf00f55aa,~v^32'h13579bdf,v^32'h2468ace0,~v^32'hdeadbeef}^{128{p}};end
    endfunction
    wire[24:0]current_address=address(index);
    wire pair_done=(aw_sent||(awvalid&&awready))&&(w_sent||(wvalid&&wready));
    assign awaddr=current_address;assign araddr=current_address;
    assign awvalid=rst_n&&state==WRITE&&!aw_sent;
    assign wvalid=rst_n&&state==WRITE&&!w_sent;
    assign wdata=pattern(current_address,pass);assign wstrb=0;assign wlast=1;
    // An already posted command remains held during timeout quarantine.
    assign arvalid=rst_n&&state==AR;
    assign rready=rst_n&&(pending||fatal);
    assign phase={pass,state};
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=WAIT;index<=0;pass<=0;aw_sent<=0;w_sent<=0;pending<=0;fatal<=0;age<=0;
            done<=0;failed<=0;errors<=0;first_address<=0;expected<=0;observed<=0;
        end else begin
            if(awvalid&&awready)aw_sent<=1;
            if(wvalid&&wready)w_sent<=1;
            if(fatal)begin
                if(state==WRITE&&pair_done)state<=FAILED;
                if(state==AR&&arready)begin pending<=1;state<=FAILED;end
                if(rvalid&&rready&&rlast)pending<=0;
            end else if((state!=FINISH&&state!=DRAIN&&age==TIMEOUT-1)||
                        (rvalid&&state!=DATA)||(state==DATA&&rvalid&&rready&&!rlast))begin
                fatal<=1;done<=1;failed<=1;
                if(!(&errors))errors<=errors+1'b1;
                if(errors==0)begin first_address<=current_address;expected<=pattern(current_address,pass);observed<=rdata;end
                if(state==WRITE&&!pair_done)state<=WRITE;
                else if(state==AR&&!arready)state<=AR;
                else state<=FAILED;
                if(arvalid&&arready)pending<=1;
                if(rvalid&&rready&&rlast)pending<=0;
            end else begin
                age<=age+1'b1;
                case(state)
                    WAIT:if(ready)begin state<=WRITE;age<=0;end
                    WRITE:if(pair_done)begin
                        aw_sent<=0;w_sent<=0;age<=0;
                        if(index==48)begin index<=0;state<=DRAIN;end else index<=index+1'b1;
                    end
                    DRAIN:if(age==WRITE_DRAIN-1)begin state<=AR;age<=0;end
                    AR:if(arready)begin pending<=1;state<=DATA;age<=0;end
                    DATA:if(rvalid&&rready)begin
                        pending<=0;age<=0;
                        if(rdata!==pattern(current_address,pass))begin
                            failed<=1;if(!(&errors))errors<=errors+1'b1;
                            if(errors==0)begin first_address<=current_address;expected<=pattern(current_address,pass);observed<=rdata;end
                        end
                        if(index==48)begin
                            index<=0;
                            if(pass)begin done<=1;state<=FINISH;end
                            else begin pass<=1;state<=WRITE;aw_sent<=0;w_sent<=0;end
                        end else begin index<=index+1'b1;state<=AR;end
                    end
                    FINISH:age<=0;
                    default:begin fatal<=1;done<=1;failed<=1;state<=FAILED;end
                endcase
            end
        end
    end
endmodule
