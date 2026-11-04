// Isolated arithmetic kernel: caller supplies the four valid source texels.
// No DDR address generation, calibration selection or camera synchronization.
module rectify_bilinear_q5 #(
    parameter integer TAG_WIDTH=32
)(
    input wire clk,rst,
    input wire in_valid,output wire in_ready,
    input wire [7:0] p00,p10,p01,p11,
    input wire [4:0] frac_x,frac_y,
    input wire sample_usable,
    input wire [TAG_WIDTH-1:0] in_tag,
    output reg out_valid,input wire out_ready,
    output reg [7:0] gray,
    output reg out_usable,
    output reg [TAG_WIDTH-1:0] out_tag
);
    reg v1,v2,usable1,usable2;
    reg [13:0] top1,bottom1;
    reg [4:0] fy1;
    reg [17:0] sum2;
    reg [TAG_WIDTH-1:0] tag1,tag2;
    wire ready3=!out_valid||out_ready;
    wire ready2=!v2||ready3;
    assign in_ready=!v1||ready2;
    wire signed [8:0] delta_top=$signed({1'b0,p10})-$signed({1'b0,p00});
    wire signed [8:0] delta_bottom=$signed({1'b0,p11})-$signed({1'b0,p01});
    wire signed [14:0] product_top=delta_top*$signed({1'b0,frac_x});
    wire signed [14:0] product_bottom=delta_bottom*$signed({1'b0,frac_x});
    wire signed [14:0] horizontal_top=$signed({2'b00,p00,5'b0})+product_top;
    wire signed [14:0] horizontal_bottom=$signed({2'b00,p01,5'b0})+product_bottom;
    wire signed [14:0] delta_vertical=$signed({1'b0,bottom1})-$signed({1'b0,top1});
    wire signed [20:0] product_vertical=delta_vertical*$signed({1'b0,fy1});
    wire signed [20:0] vertical_sum=$signed({2'b00,top1,5'b0})+product_vertical;
    always @(posedge clk)begin
        if(rst)begin
            v1<=0;v2<=0;out_valid<=0;usable1<=0;usable2<=0;out_usable<=0;
            top1<=0;bottom1<=0;fy1<=0;sum2<=0;tag1<=0;tag2<=0;out_tag<=0;gray<=0;
        end else begin
            if(in_ready)begin
                v1<=in_valid;
                if(in_valid)begin
                    top1<=horizontal_top[13:0];bottom1<=horizontal_bottom[13:0];
                    fy1<=frac_y;usable1<=sample_usable;tag1<=in_tag;
                end
            end
            if(ready2)begin
                v2<=v1;
                if(v1)begin sum2<=vertical_sum[17:0];usable2<=usable1;tag2<=tag1;end
            end
            if(ready3)begin
                out_valid<=v2;
                if(v2)begin
                    gray<=usable2 ? ((sum2+18'd512)>>10) : 8'd0;
                    out_usable<=usable2;out_tag<=tag2;
                end
            end
        end
    end
endmodule
