// Isolated descriptor-row pipeline. No camera, calibration, DDR or metric Z.
// Callers must supply one immutable, rectified, correctly identified row pair.
// Input fanout is atomic into both directions. All partial frame results must
// be discarded if fault is reported; no new row is accepted until reset.
module stereo_dual_row_pipeline #(
    parameter integer WIDTH=320,MAX_DISPARITIES=128,X_BITS=9
)(
    input wire clk,rst,row_valid,output wire row_ready,
    input wire row_pair_usable,input wire[31:0]pair_id,input wire[15:0]row_y,
    input wire[7:0]disparity_min,disparity_max,
    input wire[4:0]cost_limit,uniqueness_margin,input wire[2:0]tolerance,
    output reg row_rejected,row_done,
    input wire left_valid,output wire left_ready,input wire right_valid,output wire right_ready,
    input wire[31:0]left_pair_id,right_pair_id,input wire[15:0]left_y,right_y,
    input wire[X_BITS-1:0]left_x,right_x,
    input wire[23:0]left_descriptor,right_descriptor,input wire left_usable,right_usable,
    output wire out_valid,input wire out_ready,
    output wire[31:0]out_pair_id,output wire[15:0]out_y,
    output wire[X_BITS-1:0]out_x,output wire[7:0]out_disparity,
    output wire out_match_usable,output wire fault,output wire[1:0]fault_reason
);
    reg busy,fault_latched;reg[1:0]reason_latched;
    wire f_row_ready,r_row_ready,c_row_ready,f_rejected,r_rejected,c_rejected;
    wire f_done,r_done,c_done,f_fault,r_fault,c_fault;
    wire[1:0]f_reason,r_reason,c_reason;
    wire f_left_ready,f_right_ready,r_left_ready,r_right_ready;
    wire f_valid,r_valid,c_valid,c_out_ready;
    wire f_ready,r_ready;
    wire[31:0]f_pair,r_pair;wire[15:0]f_y,r_y;
    wire[X_BITS-1:0]f_x,r_x;wire[7:0]f_disparity,r_disparity;wire f_usable,r_usable;
    wire[4:0]f_best,f_second,r_best,r_second;wire[8:0]f_candidates,r_candidates;
    wire core_fault=f_fault||r_fault||c_fault||f_rejected||r_rejected||c_rejected;
    wire[1:0]core_reason=f_fault?f_reason:r_fault?r_reason:c_fault?c_reason:2'd3;
    assign fault=fault_latched||core_fault;
    assign fault_reason=fault_latched?reason_latched:core_reason;
    assign row_ready=!rst&&!busy&&!fault&&f_row_ready&&r_row_ready&&c_row_ready;
    wire take_row=row_valid&&row_ready;
    wire config_good=WIDTH>=2&&WIDTH<=1280&&X_BITS>=$clog2(WIDTH)&&MAX_DISPARITIES>=1&&MAX_DISPARITIES<=256&&
        row_pair_usable&&disparity_min<=disparity_max&&disparity_max<MAX_DISPARITIES&&disparity_max<WIDTH&&
        cost_limit<=24&&uniqueness_margin>=1&&uniqueness_margin<=24&&tolerance<=2;
    wire start_cores=take_row&&config_good;
    assign left_ready=!rst&&busy&&!fault&&f_left_ready&&r_left_ready;
    assign right_ready=!rst&&busy&&!fault&&f_right_ready&&r_right_ready;
    wire accept_left=left_valid&&left_ready,accept_right=right_valid&&right_ready;
    wire c_left_ready,c_right_ready;
    assign f_ready=busy&&!fault&&c_left_ready;
    assign r_ready=busy&&!fault&&c_right_ready;
    assign out_valid=busy&&!fault&&c_valid;
    assign c_out_ready=busy&&!fault&&out_ready;

    stereo_census_dual_row_matcher #(.WIDTH(WIDTH),.MAX_DISPARITIES(MAX_DISPARITIES),.X_BITS(X_BITS),.REVERSE(0))forward_match(
        .clk(clk),.rst(rst),.row_valid(start_cores),.row_ready(f_row_ready),.row_pair_usable(1'b1),.pair_id(pair_id),.row_y(row_y),
        .disparity_min(disparity_min),.disparity_max(disparity_max),.cost_limit(cost_limit),.uniqueness_margin(uniqueness_margin),
        .row_rejected(f_rejected),.row_done(f_done),.left_valid(accept_left),.left_ready(f_left_ready),.right_valid(accept_right),.right_ready(f_right_ready),
        .left_pair_id(left_pair_id),.right_pair_id(right_pair_id),.left_y(left_y),.right_y(right_y),.left_x(left_x),.right_x(right_x),
        .left_descriptor(left_descriptor),.right_descriptor(right_descriptor),.left_usable(left_usable),.right_usable(right_usable),
        .fault(f_fault),.fault_reason(f_reason),.out_valid(f_valid),.out_ready(f_ready),.out_pair_id(f_pair),.out_y(f_y),.out_x(f_x),
        .out_disparity(f_disparity),.out_best_cost(f_best),.out_second_cost(f_second),.out_candidates(f_candidates),.out_match_usable(f_usable));
    stereo_census_dual_row_matcher #(.WIDTH(WIDTH),.MAX_DISPARITIES(MAX_DISPARITIES),.X_BITS(X_BITS),.REVERSE(1))reverse_match(
        .clk(clk),.rst(rst),.row_valid(start_cores),.row_ready(r_row_ready),.row_pair_usable(1'b1),.pair_id(pair_id),.row_y(row_y),
        .disparity_min(disparity_min),.disparity_max(disparity_max),.cost_limit(cost_limit),.uniqueness_margin(uniqueness_margin),
        .row_rejected(r_rejected),.row_done(r_done),.left_valid(accept_left),.left_ready(r_left_ready),.right_valid(accept_right),.right_ready(r_right_ready),
        .left_pair_id(left_pair_id),.right_pair_id(right_pair_id),.left_y(left_y),.right_y(right_y),.left_x(left_x),.right_x(right_x),
        .left_descriptor(left_descriptor),.right_descriptor(right_descriptor),.left_usable(left_usable),.right_usable(right_usable),
        .fault(r_fault),.fault_reason(r_reason),.out_valid(r_valid),.out_ready(r_ready),.out_pair_id(r_pair),.out_y(r_y),.out_x(r_x),
        .out_disparity(r_disparity),.out_best_cost(r_best),.out_second_cost(r_second),.out_candidates(r_candidates),.out_match_usable(r_usable));
    stereo_lr_dual_row_filter #(.WIDTH(WIDTH),.X_BITS(X_BITS))consistency(
        .clk(clk),.rst(rst),.row_valid(start_cores),.row_ready(c_row_ready),.row_pair_usable(1'b1),.pair_id(pair_id),.row_y(row_y),.tolerance(tolerance),
        .row_rejected(c_rejected),.row_done(c_done),.left_valid(f_valid&&busy&&!fault),.left_ready(c_left_ready),
        .right_valid(r_valid&&busy&&!fault),.right_ready(c_right_ready),
        .left_pair_id(f_pair),.right_pair_id(r_pair),.left_y(f_y),.right_y(r_y),.left_x(f_x),.right_x(r_x),
        .left_disparity(f_disparity),.right_disparity(r_disparity),.left_usable(f_usable),.right_usable(r_usable),
        .fault(c_fault),.fault_reason(c_reason),.out_valid(c_valid),.out_ready(c_out_ready),
        .out_pair_id(out_pair_id),.out_y(out_y),.out_x(out_x),.out_disparity(out_disparity),.out_match_usable(out_match_usable));
    always @(posedge clk)begin
        if(rst)begin busy<=0;fault_latched<=0;reason_latched<=0;row_rejected<=0;row_done<=0;end
        else begin
            row_rejected<=0;row_done<=0;
            if(core_fault&&!fault_latched)begin fault_latched<=1;reason_latched<=core_reason;end
            if(take_row)begin
                if(config_good)busy<=1;else row_rejected<=1;
            end
            if(c_done&&busy&&!fault)begin busy<=0;row_done<=1;end
        end
    end
endmodule
