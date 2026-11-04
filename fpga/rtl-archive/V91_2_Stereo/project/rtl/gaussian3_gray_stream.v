// Candidate only. One single-port synchronous RAM, three padded rows.
// Emits every original coordinate, including invalid boundary samples.
// Exact integer kernel [1 2 1]^T*[1 2 1], rounding (sum+8)>>4.
module gaussian3_gray_stream #(
    parameter integer WIDTH=320,HEIGHT=180,SPLIT_COLUMN=0,
    parameter integer X_BITS=9,Y_BITS=8,MEM_BITS=10
)(
    input wire clk,rst,
    input wire start_valid,output wire start_ready,
    input wire [31:0]frame_tag,
    input wire in_valid,output wire in_ready,
    input wire [7:0]gray,input wire pixel_usable,
    output wire out_valid,input wire out_ready,
    output reg [7:0]out_gray,output reg out_usable,
    output reg [31:0]out_frame_tag,
    output reg [X_BITS-1:0]out_x,
    output reg [Y_BITS-1:0]out_y,
    output reg frame_done
);
    localparam STRIDE=WIDTH+2,ROWS=HEIGHT+2;
    localparam IDLE=0,GET=1,WRITE=2,READ0=3,READ1=4,ASSEMBLE=5,SUM=6,EMIT=7;
    reg [2:0]state;
    reg [1:0]bank;
    reg [X_BITS-1:0]px;
    reg [Y_BITS-1:0]py;
    reg [8:0]current_pixel,ram_q;
    reg [8:0]memory[0:3*STRIDE-1];
    reg [23:0]history0,history1;
    reg [2:0]valid0,valid1;
    reg [9:0]row0,row1,row2;
    reg pending_usable;
    wire inside_image=px>=1&&px<WIDTH+1&&py>=1&&py<HEIGHT+1;
    wire emit_sample=px>=2&&py>=2;
    wire [1:0]next_bank=bank==2 ? 0 : bank+1'b1;
    wire [1:0]last_bank=bank==0 ? 2 : bank-1'b1;
    wire [MEM_BITS-1:0]write_address=bank*STRIDE+px;
    wire [MEM_BITS-1:0]read_address=(state==READ0 ? next_bank : last_bank)*STRIDE+px;
    reg [8:0]top_pixel;
    wire [23:0]column={current_pixel[7:0],ram_q[7:0],top_pixel[7:0]};
    wire [2:0]column_valid={current_pixel[8],ram_q[8],top_pixel[8]};
    wire complete_window=&{valid0,valid1,column_valid};
    wire [11:0]weighted_sum={2'b0,row0}+{1'b0,row1,1'b0}+{2'b0,row2}+12'd8;
    assign start_ready=state==IDLE;
    assign in_ready=state==GET&&inside_image;
    assign out_valid=state==EMIT;

    // Read and write are mutually exclusive. No reset is applied to the RAM.
    always @(posedge clk)if(!rst)begin
        if(state==WRITE)memory[write_address]<=current_pixel;
        else if(state==READ0||state==READ1)ram_q<=memory[read_address];
    end
    task advance;
    begin
        if(px==STRIDE-1)begin
            px<=0;bank<=next_bank;
            if(py==ROWS-1)begin state<=IDLE;frame_done<=1;end
            else begin py<=py+1'b1;state<=GET;end
        end else begin px<=px+1'b1;state<=GET;end
    end endtask
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;bank<=0;px<=0;py<=0;current_pixel<=0;top_pixel<=0;
            history0<=0;history1<=0;valid0<=0;valid1<=0;
            row0<=0;row1<=0;row2<=0;pending_usable<=0;
            out_gray<=0;out_usable<=0;out_frame_tag<=0;out_x<=0;out_y<=0;frame_done<=0;
        end else begin
            frame_done<=0;
            case(state)
                IDLE:if(start_valid)begin
                    out_frame_tag<=frame_tag;px<=0;py<=0;bank<=0;state<=GET;
                end
                GET:if(!inside_image||in_valid)begin
                    current_pixel<=inside_image ? {pixel_usable,gray} : 9'd0;state<=WRITE;
                end
                WRITE:begin
                    if(px==0)begin history0<=0;history1<=0;valid0<=0;valid1<=0;end
                    state<=READ0;
                end
                READ0:state<=READ1;
                READ1:begin top_pixel<=ram_q;state<=ASSEMBLE;end
                ASSEMBLE:begin
                    history0<=history1;history1<=column;valid0<=valid1;valid1<=column_valid;
                    if(emit_sample)begin
                        row0<={2'b0,history0[7:0]}+{1'b0,history1[7:0],1'b0}+{2'b0,column[7:0]};
                        row1<={2'b0,history0[15:8]}+{1'b0,history1[15:8],1'b0}+{2'b0,column[15:8]};
                        row2<={2'b0,history0[23:16]}+{1'b0,history1[23:16],1'b0}+{2'b0,column[23:16]};
                        // Internal seam represents two separately padded source images.
                        pending_usable<=complete_window&&(SPLIT_COLUMN==0||
                            (px!=SPLIT_COLUMN+1&&px!=SPLIT_COLUMN+2));out_x<=px-2;out_y<=py-2;state<=SUM;
                    end else advance;
                end
                SUM:begin
                    out_gray<=pending_usable ? weighted_sum[11:4] : 8'd0;
                    out_usable<=pending_usable;state<=EMIT;
                end
                EMIT:if(out_ready)advance;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
