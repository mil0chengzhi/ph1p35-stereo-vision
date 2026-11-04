// Exact 5x5 Census and max-minus-min texture test, using shared comparators.
// One accepted window is retained until all 25 samples have been examined.
// All 24 descriptor bits, usability and tags are held under output stalls.
module census5x5_window #(
 parameter integer TEXTURE_MIN=6,TAG_WIDTH=32
)(
 input wire clk,rst,in_valid,output wire in_ready,
 input wire[199:0]window_gray,input wire window_usable,input wire[TAG_WIDTH-1:0]in_tag,
 output reg out_valid,input wire out_ready,output reg[23:0]descriptor,
 output reg descriptor_usable,output reg[TAG_WIDTH-1:0]out_tag
);
 reg scanning;reg[4:0]index_q;reg[199:0]window_q;
 reg[7:0]minimum,maximum;reg usable_q;
 wire[7:0]sample_q=window_q[index_q*8+:8];
 wire[7:0]next_min=sample_q<minimum?sample_q:minimum;
 wire[7:0]next_max=sample_q>maximum?sample_q:maximum;
 assign in_ready=!scanning&&(!out_valid||out_ready);
 always @(posedge clk)begin
  if(rst)begin scanning<=0;index_q<=0;window_q<=0;minimum<=255;maximum<=0;usable_q<=0;
   out_valid<=0;descriptor<=0;descriptor_usable<=0;out_tag<=0;
  end else begin
   if(out_valid&&out_ready)out_valid<=0;
   if(in_valid&&in_ready)begin
    window_q<=window_gray;usable_q<=window_usable;out_tag<=in_tag;
    minimum<=255;maximum<=0;index_q<=0;scanning<=1;descriptor<=0;
   end else if(scanning)begin
    minimum<=next_min;maximum<=next_max;
    if(index_q<12)descriptor[index_q]<=sample_q<window_q[103:96];
    else if(index_q>12)descriptor[index_q-1'b1]<=sample_q<window_q[103:96];
    if(index_q==24)begin
     descriptor_usable<=usable_q&&({1'b0,next_max}-{1'b0,next_min}>=TEXTURE_MIN);
     scanning<=0;out_valid<=1;
    end else index_q<=index_q+1'b1;
   end
  end
 end
endmodule
