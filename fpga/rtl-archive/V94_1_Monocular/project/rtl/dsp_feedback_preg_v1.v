`timescale 1ns/1ps
// Future isolated candidate. Not integrated or native qualified.
// A={XA,XB}=zero-extended registered P low FEEDBACK_WIDTH, B=Z, C=0.
// Explicit fabric feedback exposes the independent state to native optimization.
// Explicit hard SIOU/TADD avoids relying on constant-multiply inference.
(* keep_hierarchy="true" *)
module dsp_feedback_preg_v1 #(parameter FEEDBACK_WIDTH=54)(
 input wire clk,ce,sync_clear,zero_base,
 input wire [53:0] addend,
 output wire [53:0] p
);
 wire [53:0] ta,tb,tc,ts;
 wire [53:0] p_feedback;
 generate if(FEEDBACK_WIDTH==54)begin:full_feedback
  assign p_feedback=p;
 end else begin:low_feedback
  assign p_feedback={{(54-FEEDBACK_WIDTH){1'b0}},p[FEEDBACK_WIDTH-1:0]};
 end endgenerate

 wire ci,co,over,under;
 wire [6:0] operation;
 wire [5:0] oversel;
 PH1P_PHY_DSPSIOU_V2 #(
  .CLKMUX("SIG"),.CE0("SIG"),.CE1("0"),.CE2("0"),.CE3("0"),
  .RST0("SIG"),.RST1("0"),.RST2("0"),.RST3("0"),
  .XAREG("0"),.XBREG("0"),.YREG("0"),.ZREG("0"),
  .INCTRLREG("0"),.MUXCTRLREG("0"),.OPCTRLREG("0"),.CINCTRLREG("0"),
  .CXAREG("0"),.CXBREG("0"),.CYREG("0"),.XABREG("0"),.MREG("0"),
  .POUWREG("1"),.CCOREG("0"),.SR0MODE("SYNC"),.SR1MODE("SYNC"),
  .SR2MODE("SYNC"),.SR3MODE("SYNC"),.CEPOU("CE0"),.RSTPOU("RST0"),
  .USER_GSRN("ENABLE"),.RND(54'd0),.USE_OVERFLOW("P53")
 ) siou(
  .clk(clk),.ce({3'b0,ce}),.rst({3'b0,sync_clear}),
  .y(18'd0),.xa(p_feedback[53:27]),.xb(p_feedback[26:0]),.z(addend),
  // zero_base bypasses P for same-edge clear+add (generic AE OFFSET_X=0).
  .muxctrl(zero_base?7'b0001100:7'b0001101),
  .inctrl(7'b0010111),.cinctrl(3'b000),.opctrl(11'h408),
  .cxai(27'd0),.cxbi(27'd0),.cxabi(27'd0),.cyi(18'd0),.cpi(54'd0),.cci(1'b0),
  .add_sum(),.mult_p(),.logic_p(),.logic_wxor(),
  .tadd_a(ta),.tadd_b(tb),.tadd_c(tc),.tadd_ci(ci),
  .tadd_opctrl(operation),.tadd_oversel(oversel),
  .tadd_sum(ts),.tadd_co(co),.tadd_overflow(over),.tadd_underflow(under),
  .p(p)
 );
 PH1P_PHY_DSPTADD tu(
  .a(ta),.b(tb),.c(tc),.ci(ci),.opctrl(operation),.oversel(oversel),
  .sum(ts),.co(co),.overflow(over),.underflow(under)
 );
endmodule

// Exact intended rule: rst dominates; zero_base+add_enable loads addend;
// zero_base alone clears; add_enable alone adds modulo WIDTH; else hold.
// All accumulators use their original source clock, with no new CDC.
module working_dsp_accumulator_v1 #(
 parameter WIDTH=20,INPUT_WIDTH=8
)(
 input wire clk,rst_n,zero_base,add_enable,
 input wire [INPUT_WIDTH-1:0] addend,
 output wire [WIDTH-1:0] value
);
 wire [53:0] full_p;
 wire [53:0] increment=add_enable?{{(54-INPUT_WIDTH){1'b0}},addend}:54'd0;
 dsp_feedback_preg_v1 #(.FEEDBACK_WIDTH(WIDTH)) hard_state(
  .clk(clk),.ce(!rst_n||zero_base||add_enable),.sync_clear(!rst_n),
  .zero_base(zero_base),.addend(increment),.p(full_p)
 );
 // Carries above WIDTH cannot affect lower WIDTH under plain addition.
 // No packing of independent channels or dependence on geometric bounds.
 assign value=full_p[WIDTH-1:0];
endmodule
