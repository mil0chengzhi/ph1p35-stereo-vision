// Isolated dual-input LR storage; not a board design or exposure synchronizer.
// Experimental left-right consistency filter for finite positive disparity.
// Both loaded maps use their ORIGINAL, unmirrored x coordinates.
// A mirrored reverse matcher must reverse its output addresses before loading.
// Pair readiness is supplied by the future frame-pair controller, not inferred
// from frame numbers. This block does not create synchronized exposure.
module stereo_lr_dual_row_filter #(
    parameter integer WIDTH=320,
    parameter integer X_BITS=9
)(
    input wire clk,rst,
    input wire row_valid,output wire row_ready,
    input wire row_pair_usable,
    input wire [31:0] pair_id,
    input wire [15:0] row_y,
    input wire [2:0] tolerance,
    output reg row_rejected,row_done,
    input wire left_valid,output wire left_ready,input wire right_valid,output wire right_ready,
    input wire[31:0]left_pair_id,right_pair_id,input wire[15:0]left_y,right_y,
    input wire[X_BITS-1:0]left_x,right_x,output reg fault,output reg[1:0]fault_reason,
    input wire [7:0] left_disparity,right_disparity,
    input wire left_usable,right_usable,
    output wire out_valid,input wire out_ready,
    output wire [31:0] out_pair_id,
    output wire [15:0] out_y,
    output wire [X_BITS-1:0] out_x,
    output wire [7:0] out_disparity,
    output wire out_match_usable
);
    localparam IDLE=0,LOAD=1,FETCH=2,READ_RIGHT=3,CHECK=4,EMIT=5;
    reg[2:0]state;
    reg[8:0]left_ram[0:WIDTH-1],right_ram[0:WIDTH-1];
    reg[8:0]left_q,right_q;
    reg[X_BITS:0]left_count,right_count;reg[X_BITS-1:0]x;
    reg[31:0]saved_pair;
    reg[15:0]saved_y;
    reg[2:0]saved_tolerance;
    reg forward_ok,match_ok;
    wire can_read_right=left_q[8] && left_q[7:0]!=0 && left_q[7:0]<=x;
    wire[7:0]difference=left_q[7:0]>right_q[7:0] ? left_q[7:0]-right_q[7:0] : right_q[7:0]-left_q[7:0];
    assign row_ready=state==IDLE&&!fault;
    assign left_ready=state==LOAD&&!fault&&left_count<WIDTH;
    assign right_ready=state==LOAD&&!fault&&right_count<WIDTH;
    wire take_left=left_valid&&left_ready,take_right=right_valid&&right_ready;
    wire good_left=left_pair_id==saved_pair&&left_y==saved_y&&{1'b0,left_x}==left_count;
    wire good_right=right_pair_id==saved_pair&&right_y==saved_y&&right_x==WIDTH-1-right_count;
    wire bad_load=(take_left&&!good_left)||(take_right&&!good_right);
    wire left_complete=left_count==WIDTH||(take_left&&good_left&&left_count==WIDTH-1);
    wire right_complete=right_count==WIDTH||(take_right&&good_right&&right_count==WIDTH-1);
    // Left arrives in ascending x, reverse/right in descending original x.
    assign out_valid=state==EMIT&&!fault;
    assign out_pair_id=saved_pair;
    assign out_y=saved_y;
    assign out_x=x;
    assign out_disparity=left_q[7:0];
    assign out_match_usable=match_ok;
    always @(posedge clk)if(!rst)begin
        if(take_left&&good_left)left_ram[left_x]<={left_usable,left_disparity};
        if(take_right&&good_right)right_ram[right_x]<={right_usable,right_disparity};
        if(state==FETCH)left_q<=left_ram[x];
        if(state==READ_RIGHT&&can_read_right)right_q<=right_ram[x-left_q[7:0]];
    end
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;left_count<=0;right_count<=0;x<=0;fault<=0;fault_reason<=0;saved_pair<=0;saved_y<=0;saved_tolerance<=0;
            forward_ok<=0;match_ok<=0;row_rejected<=0;row_done<=0;
        end else begin
            row_rejected<=0;row_done<=0;
            if(bad_load)begin fault<=1;fault_reason<=take_left&&!good_left?1:2;state<=6;end
            else if(!fault)case(state)
                IDLE:if(row_valid)begin
                    if(WIDTH<2||WIDTH>1280||X_BITS<$clog2(WIDTH)||!row_pair_usable||tolerance>2)row_rejected<=1;
                    else begin
                        saved_pair<=pair_id;saved_y<=row_y;saved_tolerance<=tolerance;
                        left_count<=0;right_count<=0;state<=LOAD;
                    end
                end
                LOAD:begin
                    if(take_left)left_count<=left_count+1'b1;
                    if(take_right)right_count<=right_count+1'b1;
                    if(left_complete&&right_complete)begin x<=0;state<=FETCH;end
                end
                FETCH:state<=READ_RIGHT;
                READ_RIGHT:begin forward_ok<=can_read_right;state<=CHECK;end
                CHECK:begin match_ok<=forward_ok&&right_q[8]&&difference<=saved_tolerance;state<=EMIT;end
                EMIT:if(out_ready)begin
                    if(x==WIDTH-1)begin state<=IDLE;row_done<=1;end
                    else begin x<=x+1'b1;state<=FETCH;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
