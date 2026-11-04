// Candidate arithmetic -> Gaussian3 -> Census pipeline; not board installed.
module rectification_feature_gaussian_stream #(
    parameter integer WIDTH=320,HEIGHT=180,SPLIT_COLUMN=0,TEXTURE_MIN=10,
    parameter integer X_BITS=9,Y_BITS=8,MEM_BITS=11
)(
    input wire clk,rst,
    input wire start_valid,output wire start_ready,
    input wire [31:0]frame_tag,
    input wire in_valid,output wire in_ready,
    input wire [7:0]p00,p10,p01,p11,
    input wire [4:0]frac_x,frac_y,
    input wire sample_usable,
    output wire out_valid,input wire out_ready,
    output wire [23:0]descriptor,
    output wire descriptor_usable,
    output wire [31:0]out_frame_tag,
    output wire [X_BITS-1:0]out_x,
    output wire [Y_BITS-1:0]out_y,
    output reg frame_done,
    output reg fault
);
    localparam TOTAL=WIDTH*HEIGHT;
    reg busy;
    reg[31:0]active_frame,input_count,received_count,output_count;
    wire cache_start_ready,sampler_ready,sample_valid,cache_ready;
    wire filter_start_ready,filter_ready,filter_valid,filter_usable;
    wire [7:0]filter_gray;wire [31:0]filter_frame;
    wire [X_BITS-1:0]filter_x;wire [Y_BITS-1:0]filter_y;
    reg [X_BITS-1:0]expected_filter_x;
    reg [Y_BITS-1:0]expected_filter_y;
    wire filter_matches=filter_frame==active_frame && filter_x==expected_filter_x && filter_y==expected_filter_y;
    wire[7:0]sample_gray;
    wire sample_ok;
    wire[63:0]sample_tag;
    wire input_open=busy&&input_count<TOTAL&&!fault;
    assign start_ready=!busy&&cache_start_ready&&filter_start_ready&&!fault;
    assign in_ready=input_open&&sampler_ready;
    wire sample_matches=sample_tag=={active_frame,received_count};
    rectify_bilinear_q5 #(.TAG_WIDTH(64))sampler(
        .clk(clk),.rst(rst),.in_valid(in_valid&&input_open),.in_ready(sampler_ready),
        .p00(p00),.p10(p10),.p01(p01),.p11(p11),.frac_x(frac_x),.frac_y(frac_y),
        .sample_usable(sample_usable),.in_tag({active_frame,input_count}),
        .out_valid(sample_valid),.out_ready(filter_ready&&sample_matches&&!fault),
        .gray(sample_gray),.out_usable(sample_ok),.out_tag(sample_tag));
    gaussian3_gray_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SPLIT_COLUMN(SPLIT_COLUMN),.X_BITS(X_BITS),.Y_BITS(Y_BITS),.MEM_BITS($clog2(3*(WIDTH+2))))filter(
        .clk(clk),.rst(rst),.start_valid(start_valid&&start_ready),.start_ready(filter_start_ready),.frame_tag(frame_tag),
        .in_valid(sample_valid&&sample_matches&&!fault),.in_ready(filter_ready),.gray(sample_gray),.pixel_usable(sample_ok),
        .out_valid(filter_valid),.out_ready(cache_ready&&filter_matches&&!fault),.out_gray(filter_gray),.out_usable(filter_usable),
        .out_frame_tag(filter_frame),.out_x(filter_x),.out_y(filter_y),.frame_done());
    wire window_valid,window_ready,window_ok,cache_done;
    wire[199:0]window_gray;
    wire[31:0]window_frame;
    wire[X_BITS-1:0]window_x;
    wire[Y_BITS-1:0]window_y;
    census_window_cache #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.X_BITS(X_BITS),.Y_BITS(Y_BITS),.MEM_BITS(MEM_BITS))cache(
        .clk(clk),.rst(rst),.start_valid(start_valid&&start_ready),.start_ready(cache_start_ready),.frame_tag(frame_tag),
        .in_valid(filter_valid&&filter_matches&&!fault),.in_ready(cache_ready),.gray(filter_gray),.pixel_usable(filter_usable),
        .out_valid(window_valid),.out_ready(window_ready&&!fault),.window_gray(window_gray),.window_usable(window_ok),
        .out_frame_tag(window_frame),.out_x(window_x),.out_y(window_y),.frame_done(cache_done));
    // Reproduce zero padding at the two independent Census image edges.
    // The five per-column masks are shared across the five window rows.
    wire[199:0]census_window;wire[4:0]seam_keep;
    genvar seam_c,seam_r;
    generate for(seam_c=0;seam_c<5;seam_c=seam_c+1)begin:seam_columns
      assign seam_keep[seam_c]=SPLIT_COLUMN==0||
        (window_x<SPLIT_COLUMN ? window_x+seam_c<SPLIT_COLUMN+2 :
                                window_x+seam_c>=SPLIT_COLUMN+2);
      for(seam_r=0;seam_r<5;seam_r=seam_r+1)begin:seam_rows
        assign census_window[(seam_r*5+seam_c)*8+:8]=
          seam_keep[seam_c]?window_gray[(seam_r*5+seam_c)*8+:8]:8'd0;
      end
    end endgenerate
    wire census_valid;
    wire[63:0]census_tag;
    wire[15:0]x16=window_x,y16=window_y;
    census5x5_window #(.TEXTURE_MIN(TEXTURE_MIN),.TAG_WIDTH(64))census(
        .clk(clk),.rst(rst),.in_valid(window_valid&&!fault),.in_ready(window_ready),
        .window_gray(census_window),.window_usable(window_ok),.in_tag({window_frame,y16,x16}),
        .out_valid(census_valid),.out_ready(out_ready&&!fault),.descriptor(descriptor),
        .descriptor_usable(descriptor_usable),.out_tag(census_tag));
    assign out_valid=census_valid&&!fault;
    assign out_frame_tag=census_tag[63:32];
    assign out_x=census_tag[15:0];
    assign out_y=census_tag[31:16];
    always @(posedge clk)begin
        if(rst)begin
            busy<=0;active_frame<=0;input_count<=0;received_count<=0;output_count<=0;frame_done<=0;fault<=0;
            expected_filter_x<=0;expected_filter_y<=0;
        end else begin
            frame_done<=0;
            if(start_valid&&start_ready)begin
                busy<=1;active_frame<=frame_tag;input_count<=0;received_count<=0;output_count<=0;
                expected_filter_x<=0;expected_filter_y<=0;
            end
            if(in_valid&&in_ready)input_count<=input_count+1'b1;
            if(sample_valid&&filter_ready&&!fault)begin
                if(!sample_matches)fault<=1;
                else received_count<=received_count+1'b1;
            end
            if(filter_valid&&!filter_matches)fault<=1;
            if(filter_valid&&cache_ready&&filter_matches&&!fault)begin
                if(expected_filter_x==WIDTH-1)begin expected_filter_x<=0;expected_filter_y<=expected_filter_y+1'b1;end
                else expected_filter_x<=expected_filter_x+1'b1;
            end
            if(out_valid&&out_ready)begin
                if(out_frame_tag!=active_frame)fault<=1;
                output_count<=output_count+1'b1;
                if(output_count==TOTAL-1)begin busy<=0;frame_done<=1;end
            end
        end
    end
endmodule
