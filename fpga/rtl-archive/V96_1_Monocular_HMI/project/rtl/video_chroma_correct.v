`timescale 1ns/1ps
// Bounded neutral-preserving display chroma enhancement on gamma-encoded RGB888.
// Use AFTER detection/analysis and BEFORE display tone/OSD. This is not sensor CCM.
// mode 0/3: identity; 1: soft; 2: medium. No gamma conversion is performed here.
// Soft /16: [18,-2,0; -1,18,-1; 0,-2,18]. Medium /16: [20,-4,0; -2,20,-2; 0,-4,20].
// Rows sum to 16; exact neutral RGB stays unchanged. Integer rounding is nearest,
// ties toward +infinity, followed by clipping to [0,255]. No multiply or RAM.
// Two sampled stages: input at edge N -> output after edge N+1, one RGB per clk.
// rst_n and flush are synchronous; either clears both stages and discards in-flight pixels.
// mode is tagged with each pixel; the caller should keep it stable for a whole frame.
module video_chroma_correct(
 input wire clk,input wire rst_n,input wire flush,input wire [1:0] mode,
 input wire [23:0] rgb_in,input wire valid_in,
 output reg [23:0] rgb_out,output reg valid_out
);
 reg [23:0] base1;
 reg [1:0] mode1;
 reg valid1;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *) reg signed [10:0] dr1,dg1,db1;
 wire signed [10:0] r_ext=$signed({3'b0,rgb_in[23:16]});
 wire signed [10:0] g_ext=$signed({3'b0,rgb_in[15:8]});
 wire signed [10:0] b_ext=$signed({3'b0,rgb_in[7:0]});
 reg signed [11:0] cr,cg,cb;
 reg signed [11:0] sum_r,sum_g,sum_b;
 function [7:0] clip8;
  input signed [11:0] value;
  begin if(value<0) clip8=0;else if(value>255) clip8=255;else clip8=value[7:0];end
 endfunction
 always @* begin
  cr=0;cg=0;cb=0;
  case(mode1)
   2'd1: begin
    cr=($signed({dr1[10],dr1})+12'sd4)>>>3;
    cg=($signed({dg1[10],dg1})+12'sd8)>>>4;
    cb=($signed({db1[10],db1})+12'sd4)>>>3;
   end
   2'd2: begin
    cr=($signed({dr1[10],dr1})+12'sd2)>>>2;
    cg=($signed({dg1[10],dg1})+12'sd4)>>>3;
    cb=($signed({db1[10],db1})+12'sd2)>>>2;
   end
   default: begin cr=0;cg=0;cb=0;end
  endcase
  sum_r=$signed({4'b0,base1[23:16]})+cr;
  sum_g=$signed({4'b0,base1[15:8]})+cg;
  sum_b=$signed({4'b0,base1[7:0]})+cb;
 end
 always @(posedge clk) begin
  if(!rst_n || flush) begin
   base1<=0;mode1<=0;valid1<=0;dr1<=0;dg1<=0;db1<=0;rgb_out<=0;valid_out<=0;
  end else begin
   base1<=rgb_in;mode1<=mode;valid1<=valid_in;
   dr1<=r_ext-g_ext;dg1<=(g_ext<<<1)-r_ext-b_ext;db1<=b_ext-g_ext;
   valid_out<=valid1;
   rgb_out<=valid1?{clip8(sum_r),clip8(sum_g),clip8(sum_b)}:24'b0;
  end
 end
endmodule
