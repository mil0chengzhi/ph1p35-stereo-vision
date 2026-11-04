// One configuration permits one bounded, independent two-axis sequence.
// start/stop must be synchronous. Stop is sticky, including during the gap.
// Counts observe actual STEP rising edges, one clk after the physical edge.
// Reset/reconfiguration releases EN and loses relative pulse counts; it does
// NOT establish a mechanical origin or preserve a motor's electrical phase.
module seq_control #(
    parameter integer CLK_HZ=50000000,
    parameter integer STEP_HZ=200,
    parameter integer PULSES=44,
    parameter integer GAP_TICKS=5*CLK_HZ
)(
    input wire clk,rst_n,start,stop,
    output wire step1,dir1,en1,step2,dir2,en2,
    output reg [3:0] phase,
    output reg [15:0] count1,count2,
    output reg consumed,
    output reg [31:0] gap_ticks,
    output wire busy1,busy2
);
    localparam [3:0] IDLE=0,CN1_START=1,CN1_RUN=2,GAP=3,
        CN2_START=4,CN2_RUN=5,DONE=6,CANCELLED=7;
    localparam [63:0] HALF_WIDE=(STEP_HZ>0)?
        (64'd1*CLK_HZ+64'd2*STEP_HZ-1)/(64'd2*STEP_HZ):64'd0;
    localparam [63:0] WAKE_WIDE=(64'd1*CLK_HZ*2000+999999)/1000000;
    localparam [63:0] MIN_PHASE_WIDE=(64'd1*CLK_HZ+999999)/1000000;
    localparam PARAM_VALID=CLK_HZ>0 && STEP_HZ>0 && PULSES>0 &&
        PULSES<=65535 && GAP_TICKS>0 && HALF_WIDE>=MIN_PHASE_WIDE &&
        HALF_WIDE>0 && HALF_WIDE<=2147483647 &&
        WAKE_WIDE>0 && WAKE_WIDE<=2147483647;
    wire done1,done2;
    wire halt=stop || phase==CANCELLED;
    reg step1_d,step2_d;

    generate if(PARAM_VALID)begin: valid_parameters
        d36a_step_burst #(.CLK_HZ(CLK_HZ),.STEP_HZ(STEP_HZ),
            .PULSES(PULSES),.WAKE_US(2000)) cn1(
            .clk(clk),.rst_n(rst_n),.start(phase==CN1_START && !stop),
            .direction(1'b0),.halt(halt),.step(step1),.dir(dir1),
            .en(en1),.busy(busy1),.done(done1));
        d36a_step_burst #(.CLK_HZ(CLK_HZ),.STEP_HZ(STEP_HZ),
            .PULSES(PULSES),.WAKE_US(2000)) cn2(
            .clk(clk),.rst_n(rst_n),.start(phase==CN2_START && !stop),
            .direction(1'b0),.halt(halt),.step(step2),.dir(dir2),
            .en(en2),.busy(busy2),.done(done2));
    end else begin: invalid_parameters
        assign {step1,dir1,en1,step2,dir2,en2,busy1,busy2,done1,done2}=10'b0;
    end endgenerate

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            phase<=IDLE;count1<=0;count2<=0;consumed<=0;gap_ticks<=0;
            step1_d<=0;step2_d<=0;
        end else begin
            step1_d<=step1;step2_d<=step2;
            if(step1 && !step1_d)count1<=count1+1'b1;
            if(step2 && !step2_d)count2<=count2+1'b1;
            if(!PARAM_VALID || stop)begin
                phase<=CANCELLED;
                consumed<=1'b1;
            end else case(phase)
                IDLE: if(start && !consumed)begin
                    consumed<=1'b1;
                    phase<=CN1_START;
                end
                CN1_START: phase<=CN1_RUN;
                CN1_RUN: if(done1)begin gap_ticks<=0;phase<=GAP;end
                GAP: begin
                    // Entry has gap_ticks=0; exactly GAP_TICKS full clk
                    // intervals elapse before the CN2_START state appears.
                    if(gap_ticks==GAP_TICKS-1)begin
                        gap_ticks<=GAP_TICKS;
                        phase<=CN2_START;
                    end else gap_ticks<=gap_ticks+1'b1;
                end
                CN2_START: phase<=CN2_RUN;
                CN2_RUN: if(done2)phase<=DONE;
                DONE: begin end
                CANCELLED: begin end
                default: begin phase<=CANCELLED;consumed<=1'b1;end
            endcase
        end
    end
    // synthesis translate_off
    initial if(!PARAM_VALID)$fatal(1,"seq_control invalid parameters");
    // synthesis translate_on
endmodule
