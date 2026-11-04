// Experimental 5x5 window builder. One synchronous, time-multiplexed RAM stores
// five padded rows. Input is exactly WIDTH*HEIGHT gray/mask samples per frame.
// Padding is inserted internally; all WIDTH*HEIGHT output positions are emitted.
module census_window_cache #(
    parameter integer WIDTH=320,HEIGHT=180,
    parameter integer X_BITS=9,Y_BITS=8,MEM_BITS=11
)(
    input wire clk,rst,
    input wire start_valid,output wire start_ready,
    input wire [31:0]frame_tag,
    input wire in_valid,output wire in_ready,
    input wire [7:0]gray,
    input wire pixel_usable,
    output wire out_valid,input wire out_ready,
    output reg [199:0]window_gray,
    output reg window_usable,
    output reg [31:0]out_frame_tag,
    output reg [X_BITS-1:0]out_x,
    output reg [Y_BITS-1:0]out_y,
    output reg frame_done
);
    localparam STRIDE=WIDTH+4,ROWS=HEIGHT+4;
    localparam IDLE=0,GET=1,WRITE=2,READ=3,ASSEMBLE=4,EMIT=5;
    reg[2:0]state,current_bank;
    reg[X_BITS-1:0]px;
    reg[Y_BITS-1:0]py;
    reg[1:0]read_index;
    reg[8:0]current_pixel,ram_q,col0,col1,col2;
    reg[8:0]memory[0:5*STRIDE-1];
    reg[39:0]history[0:3];
    reg[4:0]history_valid[0:3];
    wire inside_image=px>=2 && px<WIDTH+2 && py>=2 && py<HEIGHT+2;
    wire emit_window=px>=4 && py>=4;
    wire[39:0]column={current_pixel[7:0],ram_q[7:0],col2[7:0],col1[7:0],col0[7:0]};
    wire[4:0]column_valid={current_pixel[8],ram_q[8],col2[8],col1[8],col0[8]};
    wire complete_window=&{history_valid[0],history_valid[1],history_valid[2],history_valid[3],column_valid};
    function[2:0]wrapped_bank;
        input[2:0]base;
        input[2:0]offset;
        reg[3:0]sum;
        begin sum={1'b0,base}+{1'b0,offset};wrapped_bank=sum>=5 ? sum-5 : sum;end
    endfunction
    wire[2:0]read_bank=wrapped_bank(current_bank,{1'b0,read_index}+3'd1);
    wire[MEM_BITS-1:0]write_address=current_bank*STRIDE+px;
    wire[MEM_BITS-1:0]read_address=read_bank*STRIDE+px;
    assign start_ready=state==IDLE;
    assign in_ready=state==GET && inside_image;
    assign out_valid=state==EMIT;
    always @(posedge clk)if(!rst)begin
        if(state==WRITE)memory[write_address]<=current_pixel;
        else if(state==READ)ram_q<=memory[read_address];
    end
    task advance_pixel;
    begin
        if(px==STRIDE-1)begin
            px<=0;current_bank<=current_bank==4 ? 0 : current_bank+1'b1;
            if(py==ROWS-1)begin state<=IDLE;frame_done<=1;end
            else begin py<=py+1'b1;state<=GET;end
        end else begin px<=px+1'b1;state<=GET;end
    end endtask
    integer i,r,c;
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;current_bank<=0;px<=0;py<=0;read_index<=0;current_pixel<=0;
            col0<=0;col1<=0;col2<=0;window_gray<=0;window_usable<=0;
            out_frame_tag<=0;out_x<=0;out_y<=0;frame_done<=0;
            for(i=0;i<4;i=i+1)begin history[i]<=0;history_valid[i]<=0;end
        end else begin
            frame_done<=0;
            case(state)
                IDLE:if(start_valid)begin
                    out_frame_tag<=frame_tag;px<=0;py<=0;current_bank<=0;state<=GET;
                end
                GET:if(!inside_image||in_valid)begin
                    current_pixel<=inside_image ? {pixel_usable,gray} : 9'd0;state<=WRITE;
                end
                WRITE:begin
                    if(px==0)for(i=0;i<4;i=i+1)begin history[i]<=0;history_valid[i]<=0;end
                    read_index<=0;state<=READ;
                end
                READ:begin
                    if(read_index==1)col0<=ram_q;
                    if(read_index==2)col1<=ram_q;
                    if(read_index==3)begin col2<=ram_q;state<=ASSEMBLE;end
                    else read_index<=read_index+1'b1;
                end
                ASSEMBLE:begin
                    for(i=0;i<3;i=i+1)begin history[i]<=history[i+1];history_valid[i]<=history_valid[i+1];end
                    history[3]<=column;history_valid[3]<=column_valid;
                    if(emit_window)begin
                        for(r=0;r<5;r=r+1)begin
                            for(c=0;c<4;c=c+1)window_gray[(r*5+c)*8 +: 8]<=history[c][r*8 +: 8];
                            window_gray[(r*5+4)*8 +: 8]<=column[r*8 +: 8];
                        end
                        window_usable<=complete_window;out_x<=px-4;out_y<=py-4;state<=EMIT;
                    end else advance_pixel;
                end
                EMIT:if(out_ready)advance_pixel;
                default:state<=IDLE;
            endcase
        end
    end
endmodule

// Test-only byte inspection interface, keeping the 200-bit internal window
// below package IO count while allowing exact gate-level byte reconstruction.
module census_window_cache_shell(
    input wire clk,rst,start_valid,
    output wire start_ready,
    input wire[31:0]frame_tag,
    input wire in_valid,
    output wire in_ready,
    input wire[7:0]gray,
    input wire pixel_usable,
    output wire out_valid,
    input wire out_ready,
    input wire[4:0]byte_index,
    output wire[7:0]window_byte,
    output wire window_usable,
    output wire[31:0]out_frame_tag,
    output wire[8:0]out_x,
    output wire[7:0]out_y,
    output wire frame_done
);
    wire[199:0]window_gray;
    census_window_cache engine(.clk(clk),.rst(rst),.start_valid(start_valid),.start_ready(start_ready),
        .frame_tag(frame_tag),.in_valid(in_valid),.in_ready(in_ready),.gray(gray),.pixel_usable(pixel_usable),
        .out_valid(out_valid),.out_ready(out_ready),.window_gray(window_gray),.window_usable(window_usable),
        .out_frame_tag(out_frame_tag),.out_x(out_x),.out_y(out_y),.frame_done(frame_done));
    assign window_byte=byte_index<25 ? window_gray[byte_index*8 +: 8] : 8'd0;
endmodule
