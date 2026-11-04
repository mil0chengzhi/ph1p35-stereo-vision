// Experimental building block, not connected to either camera/DDR/display.
// Caller supplies an already aligned 5x5 grayscale window, row-major,
// window_gray[7:0] = upper-left. Center (index12) is excluded.
module census5x5_window #(
    parameter integer TEXTURE_MIN = 6,
    parameter integer TAG_WIDTH = 32
)(
    input wire clk, input wire rst,
    input wire in_valid, output wire in_ready,
    input wire [199:0] window_gray,
    input wire window_usable,
    input wire [TAG_WIDTH-1:0] in_tag,
    output reg out_valid, input wire out_ready,
    output reg [23:0] descriptor,
    output reg descriptor_usable,
    output reg [TAG_WIDTH-1:0] out_tag
);
    wire [23:0] census;
    wire [7:0] pixel_min [0:79];
    wire [7:0] pixel_max [0:79];
    wire [7:0] global_min [0:15];
    wire [7:0] global_max [0:15];
    reg stage_valid, stage_usable;
    reg [23:0] stage_descriptor;
    reg [TAG_WIDTH-1:0] stage_tag;
    reg [39:0] row_min, row_max;
    wire output_ready = !out_valid || out_ready;
    assign in_ready = !stage_valid || output_ready;
    genvar r,j,k;
    generate
        for(k=0;k<25;k=k+1)begin : comparisons
            if(k<12)assign census[k]=window_gray[k*8 +: 8] < window_gray[96 +: 8];
            else if(k>12)assign census[k-1]=window_gray[k*8 +: 8] < window_gray[96 +: 8];
        end
        for(r=0;r<5;r=r+1)begin : rows
            for(j=0;j<8;j=j+1)begin : leaves
                if(j<5)begin
                    assign pixel_min[r*16+8+j]=window_gray[(r*5+j)*8 +: 8];
                    assign pixel_max[r*16+8+j]=window_gray[(r*5+j)*8 +: 8];
                end else begin
                    assign pixel_min[r*16+8+j]=8'hff;
                    assign pixel_max[r*16+8+j]=8'h00;
                end
            end
            for(j=1;j<8;j=j+1)begin : tree
                assign pixel_min[r*16+j]=pixel_min[r*16+j*2]<pixel_min[r*16+j*2+1] ? pixel_min[r*16+j*2] : pixel_min[r*16+j*2+1];
                assign pixel_max[r*16+j]=pixel_max[r*16+j*2]>pixel_max[r*16+j*2+1] ? pixel_max[r*16+j*2] : pixel_max[r*16+j*2+1];
            end
        end
        for(j=0;j<8;j=j+1)begin : global_leaves
            if(j<5)begin
                assign global_min[8+j]=row_min[j*8 +: 8];
                assign global_max[8+j]=row_max[j*8 +: 8];
            end else begin
                assign global_min[8+j]=8'hff;
                assign global_max[8+j]=8'h00;
            end
        end
        for(j=1;j<8;j=j+1)begin : global_tree
            assign global_min[j]=global_min[j*2]<global_min[j*2+1] ? global_min[j*2] : global_min[j*2+1];
            assign global_max[j]=global_max[j*2]>global_max[j*2+1] ? global_max[j*2] : global_max[j*2+1];
        end
    endgenerate
    integer row;
    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0;
            descriptor <= 24'd0;
            descriptor_usable <= 1'b0;
            out_tag <= {TAG_WIDTH{1'b0}};
            stage_valid<=0; stage_usable<=0; stage_descriptor<=0;stage_tag<=0;
            row_min<=0;row_max<=0;
        end else begin
            if(in_ready)begin
                stage_valid<=in_valid;
                if(in_valid)begin
                    stage_descriptor<=census;stage_usable<=window_usable;stage_tag<=in_tag;
                    for(row=0;row<5;row=row+1)begin
                        row_min[row*8 +: 8]<=pixel_min[row*16+1];
                        row_max[row*8 +: 8]<=pixel_max[row*16+1];
                    end
                end
            end
            if(output_ready)begin
                out_valid<=stage_valid;
                if(stage_valid)begin
                    descriptor<=stage_descriptor;
                    descriptor_usable<=stage_usable && ({1'b0,global_max[1]}-{1'b0,global_min[1]} >= TEXTURE_MIN);
                    out_tag<=stage_tag;
                end
            end
        end
    end
endmodule
