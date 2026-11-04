// Isolated dual-stream prototype; no calibration, DDR or board integration.
// Accept logical-left/right rows independently in ascending original x.
// REVERSE=1 swaps cameras and mirrors RAM reads; output x is original right x.
// A malformed accepted input latches fault until reset and invalidates the row.
// Experimental serial Hamming/WTA engine. Positive horizontal disparity only:
// left(x) corresponds to right(x-d). No calibration, rectification or metric Z.
// Load a complete descriptor row, then search one candidate per clock.
// Two row RAMs are fully overwritten before use; reset does not clear RAM.
module stereo_census_dual_row_matcher #(
    parameter integer WIDTH = 320,
    parameter integer MAX_DISPARITIES = 128,
    parameter integer X_BITS = 9,
    parameter integer REVERSE = 0
)(
    input wire clk, input wire rst,
    input wire row_valid, output wire row_ready,
    input wire row_pair_usable,
    input wire [31:0] pair_id,
    input wire [15:0] row_y,
    input wire [7:0] disparity_min, disparity_max,
    input wire [4:0] cost_limit, uniqueness_margin,
    output reg row_rejected, output reg row_done,
    input wire left_valid,output wire left_ready,input wire right_valid,output wire right_ready,
    input wire[31:0]left_pair_id,right_pair_id,input wire[15:0]left_y,right_y,
    input wire[X_BITS-1:0]left_x,right_x,output reg fault,output reg[1:0]fault_reason,
    input wire [23:0] left_descriptor, right_descriptor,
    input wire left_usable, right_usable,
    output wire out_valid, input wire out_ready,
    output wire [31:0] out_pair_id,
    output wire [15:0] out_y,
    output wire [X_BITS-1:0] out_x,
    output wire [7:0] out_disparity,
    output wire [4:0] out_best_cost, out_second_cost,
    output wire [8:0] out_candidates,
    output wire out_match_usable
);
    localparam IDLE=0, LOAD=1, FETCH=2, INIT=3, SCAN=4, EMIT=5;
    reg [2:0] state;
    reg [24:0] left_ram [0:WIDTH-1];
    reg [24:0] right_ram [0:WIDTH-1];
    reg [24:0] left_q, right_q;
    reg [X_BITS:0] left_count,right_count;
    reg [X_BITS-1:0] x;
    reg [31:0] saved_pair;
    reg [15:0] saved_y;
    reg [7:0] saved_min, saved_max, last_d, response_d;
    reg [8:0] issue_d;
    reg response_valid;
    reg [4:0] saved_cost, saved_margin;
    reg [23:0] left_hold;
    reg left_ok;
    reg [4:0] best_cost, second_cost;
    reg [7:0] best_d;
    reg [8:0] candidates;

    function [4:0] hamming24;
        input [23:0] bits;
        reg [2:0] a,b,c,d,e,f;
        reg [3:0] ab,cd,ef;
        begin
            a={2'b0,bits[0]}+bits[1]+bits[2]+bits[3];
            b={2'b0,bits[4]}+bits[5]+bits[6]+bits[7];
            c={2'b0,bits[8]}+bits[9]+bits[10]+bits[11];
            d={2'b0,bits[12]}+bits[13]+bits[14]+bits[15];
            e={2'b0,bits[16]}+bits[17]+bits[18]+bits[19];
            f={2'b0,bits[20]}+bits[21]+bits[22]+bits[23];
            ab={1'b0,a}+b; cd={1'b0,c}+d; ef={1'b0,e}+f;
            hamming24={1'b0,ab}+cd+ef;
        end
    endfunction
    wire [4:0] candidate_cost = hamming24(left_hold ^ right_q[23:0]);
    wire issue = state==SCAN && issue_d <= {1'b0,last_d};
    wire configuration_ok=WIDTH>=2&&WIDTH<=1280&&X_BITS>=$clog2(WIDTH)&&MAX_DISPARITIES>=1&&MAX_DISPARITIES<=256&&(REVERSE==0||REVERSE==1);
    assign row_ready = state==IDLE&&!fault;
    assign left_ready=state==LOAD&&!fault&&left_count<WIDTH;
    assign right_ready=state==LOAD&&!fault&&right_count<WIDTH;
    wire take_left=left_valid&&left_ready,take_right=right_valid&&right_ready;
    wire good_left=left_pair_id==saved_pair&&left_y==saved_y&&{1'b0,left_x}==left_count;
    wire good_right=right_pair_id==saved_pair&&right_y==saved_y&&{1'b0,right_x}==right_count;
    wire bad_load=(take_left&&!good_left)||(take_right&&!good_right);
    wire left_complete=left_count==WIDTH||(take_left&&good_left&&left_count==WIDTH-1);
    wire right_complete=right_count==WIDTH||(take_right&&good_right&&right_count==WIDTH-1);
    // Both rows may arrive entirely separately; each RAM is written once in natural x order.
    assign out_valid = state==EMIT&&!fault;
    assign out_pair_id = saved_pair;
    assign out_y = saved_y;
    assign out_x = REVERSE ? WIDTH-1-x : x;
    assign out_disparity = best_d;
    assign out_best_cost = best_cost;
    assign out_second_cost = second_cost;
    assign out_candidates = candidates;
    assign out_match_usable = left_ok && candidates >= 2 && best_cost <= saved_cost &&
                              ({1'b0,second_cost}-{1'b0,best_cost} >= {1'b0,saved_margin});

    // Read indices are explicitly bounded to the physical row-RAM width.
    // Valid FETCH/SCAN addresses are0..WIDTH-1; inactive wrapped values are unused.
    wire[X_BITS-1:0]centre_address=REVERSE ? WIDTH-1-x : x;
    wire[X_BITS-1:0]candidate_address=REVERSE ? WIDTH-1-(x-issue_d) : x-issue_d;
    always @(posedge clk) begin
        if (!rst) begin
            if(take_left&&good_left)left_ram[left_count[X_BITS-1:0]]<={left_usable,left_descriptor};
            if(take_right&&good_right)right_ram[right_count[X_BITS-1:0]]<={right_usable,right_descriptor};
            if(state==FETCH)begin
                if(REVERSE)left_q<=right_ram[centre_address];else left_q<=left_ram[centre_address];
            end
            if(issue)begin
                if(REVERSE)right_q<=left_ram[candidate_address];else right_q<=right_ram[candidate_address];
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state<=IDLE; left_count<=0;right_count<=0;x<=0;fault<=0;fault_reason<=0;
            saved_pair<=0; saved_y<=0; saved_min<=0; saved_max<=0;
            last_d<=0; issue_d<=0; response_d<=0; response_valid<=0;
            saved_cost<=0; saved_margin<=0; left_hold<=0; left_ok<=0;
            best_cost<=31; second_cost<=31; best_d<=0; candidates<=0;
            row_rejected<=0; row_done<=0;
        end else begin
            row_rejected<=0; row_done<=0;
            if(bad_load)begin
                fault<=1;fault_reason<=take_left&&!good_left?1:2;state<=6;
            end else if(!fault)case (state)
                IDLE: if (row_valid) begin
                    if (!configuration_ok || !row_pair_usable || disparity_min > disparity_max ||
                        disparity_max >= MAX_DISPARITIES || disparity_max >= WIDTH ||
                        cost_limit > 24 || uniqueness_margin==0 || uniqueness_margin > 24) begin
                        row_rejected<=1;
                    end else begin
                        saved_pair<=pair_id; saved_y<=row_y;
                        saved_min<=disparity_min; saved_max<=disparity_max;
                        saved_cost<=cost_limit; saved_margin<=uniqueness_margin;
                        left_count<=0;right_count<=0;state<=LOAD;
                    end
                end
                LOAD:begin
                    if(take_left)left_count<=left_count+1'b1;
                    if(take_right)right_count<=right_count+1'b1;
                    if(left_complete&&right_complete)begin x<=0;state<=FETCH;end
                end
                FETCH: state<=INIT;
                INIT: begin
                    left_hold<=left_q[23:0]; left_ok<=left_q[24];
                    best_cost<=31; second_cost<=31; best_d<=0; candidates<=0;
                    issue_d<={1'b0,saved_min};
                    last_d <= x < saved_max ? x : saved_max;
                    response_valid<=0; state<=SCAN;
                end
                SCAN: begin
                    response_valid<=issue;
                    if (issue) begin response_d<=issue_d[7:0]; issue_d<=issue_d+1'b1; end
                    if (response_valid && right_q[24]) begin
                        candidates<=candidates+1'b1;
                        if (candidate_cost < best_cost) begin
                            second_cost<=best_cost; best_cost<=candidate_cost; best_d<=response_d;
                        end else if (candidate_cost < second_cost) second_cost<=candidate_cost;
                    end
                    if (!issue && !response_valid) state<=EMIT;
                end
                EMIT: if (out_ready) begin
                    if (x==WIDTH-1) begin state<=IDLE; row_done<=1; end
                    else begin x<=x+1'b1; state<=FETCH; end
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
