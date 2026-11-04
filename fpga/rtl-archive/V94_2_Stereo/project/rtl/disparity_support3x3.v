`timescale 1ns/1ps
// ISOLATED SOURCE CANDIDATE, NOT INSTALLED OR SIMULATED.
// Insert AFTER RGB join: payload={d8,Z16,original_RGB56516}, valid=(Z!=0).
// One simultaneous pass over ORIGINAL input, delta<=1, at least2 of8 neighbors.
// Every original coordinate is emitted; only centre Z may change to0.
// Three padded rows share one synchronous, read/write time-multiplexed RAM.
module disparity_support3x3 #(
 parameter integer WIDTH=1280,HEIGHT=720,
 parameter integer X_BITS=11,Y_BITS=10,MEM_BITS=$clog2(3*(WIDTH+2))
)(
 input wire clk,rst,
 input wire start_valid,output wire start_ready,input wire[31:0]frame_tag,
 input wire in_valid,output wire in_ready,
 input wire[7:0]in_disparity,input wire[15:0]in_Z_mm,in_RGB565,
 input wire[31:0]in_frame_tag,input wire[X_BITS-1:0]in_x,input wire[Y_BITS-1:0]in_y,
 output wire out_valid,input wire out_ready,
 output reg[7:0]out_disparity,output reg[15:0]out_Z_mm,out_RGB565,
 output reg[31:0]out_frame_tag,output reg[X_BITS-1:0]out_x,output reg[Y_BITS-1:0]out_y,
 output wire busy,output reg frame_done,fault,output reg[3:0]fault_reason,
 output wire configuration_error
);
 localparam integer STRIDE=WIDTH+2,ROWS=HEIGHT+2,TOTAL=WIDTH*HEIGHT;
 localparam integer OK=WIDTH>=1&&WIDTH<=1280&&HEIGHT>=1&&HEIGHT<=720&&
  X_BITS>=$clog2(WIDTH+2)&&Y_BITS>=$clog2(HEIGHT+2)&&MEM_BITS>=$clog2(3*(WIDTH+2));
 localparam IDLE=0,GET=1,WRITE=2,READ0=3,READ1=4,ASSEMBLE=5,CHECK=6,EMIT=7;
 reg[2:0]state;reg[1:0]bank;
 reg[X_BITS-1:0]px;reg[Y_BITS-1:0]py;
 reg[19:0]input_count,output_count;reg[31:0]active_tag;
 reg[39:0]current_pixel,ram_q,top_pixel,centre_q;
 reg[119:0]history0,history1;reg[7:0]neighbor_q;
 (* ram_style="bram" *)reg[39:0]memory[0:3*STRIDE-1];
 wire inside_image=px>=1&&px<WIDTH+1&&py>=1&&py<HEIGHT+1;
 wire emit_sample=px>=2&&py>=2;
 wire[1:0]next_bank=bank==2?0:bank+1'b1;
 wire[1:0]last_bank=bank==0?2:bank-1'b1;
 // Constant bank-base muxes avoid any runtime address multiplication.
 function[MEM_BITS-1:0]bank_base(input[1:0]which);
  begin case(which)0:bank_base=0;1:bank_base=STRIDE;default:bank_base=2*STRIDE;endcase end
 endfunction
 wire[MEM_BITS-1:0]write_address=bank_base(bank)+px;
 wire[MEM_BITS-1:0]read_address=bank_base(state==READ0?next_bank:last_bank)+px;
 wire[119:0]column={current_pixel,ram_q,top_pixel};
 wire[39:0]centre=history1[79:40];
 function neighbor_matches(input[39:0]neighbour,input[39:0]middle);
  reg[8:0]nd,cd;
  begin
   nd={1'b0,neighbour[39:32]};cd={1'b0,middle[39:32]};
   neighbor_matches=neighbour[31:16]!=0&&(nd==cd||nd+9'd1==cd||cd+9'd1==nd);
  end
 endfunction
 wire[7:0]hits={neighbor_matches(column[119:80],centre),
  neighbor_matches(history1[119:80],centre),neighbor_matches(history0[119:80],centre),
  neighbor_matches(column[79:40],centre),neighbor_matches(history0[79:40],centre),
  neighbor_matches(column[39:0],centre),neighbor_matches(history1[39:0],centre),
  neighbor_matches(history0[39:0],centre)};
 wire[3:0]support_count={3'd0,neighbor_q[0]}+{3'd0,neighbor_q[1]}+
  {3'd0,neighbor_q[2]}+{3'd0,neighbor_q[3]}+{3'd0,neighbor_q[4]}+
  {3'd0,neighbor_q[5]}+{3'd0,neighbor_q[6]}+{3'd0,neighbor_q[7]};
 assign configuration_error=!OK;
 assign start_ready=!rst&&!fault&&OK&&state==IDLE;
 assign in_ready=!rst&&!fault&&OK&&state==GET&&inside_image;
 assign out_valid=!rst&&!fault&&OK&&state==EMIT;
 assign busy=state!=IDLE||fault;

 // No RAM reset, and read/write are mutually exclusive. Padding overwrites
 // complete old rows before the first output of every new frame is possible.
 always @(posedge clk)if(!rst&&!fault&&OK)begin
  if(state==WRITE)memory[write_address]<=current_pixel;
  else if(state==READ0||state==READ1)ram_q<=memory[read_address];
 end
 task advance_pixel;
  begin
   if(px==STRIDE-1)begin
    px<=0;bank<=next_bank;
    if(py==ROWS-1)begin
     if(state==EMIT&&input_count==TOTAL&&output_count==TOTAL-1)begin state<=IDLE;frame_done<=1;end
     else begin fault<=1;fault_reason<=3;end
    end else begin py<=py+1'b1;state<=GET;end
   end else begin px<=px+1'b1;state<=GET;end
  end
 endtask
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;bank<=0;px<=0;py<=0;input_count<=0;output_count<=0;active_tag<=0;
   current_pixel<=0;top_pixel<=0;centre_q<=0;history0<=0;history1<=0;neighbor_q<=0;
   out_disparity<=0;out_Z_mm<=0;out_RGB565<=0;out_frame_tag<=0;out_x<=0;out_y<=0;
   frame_done<=0;fault<=0;fault_reason<=0;
  end else begin
   frame_done<=0;
   if(!fault&&OK)case(state)
    IDLE:if(start_valid&&start_ready)begin
     active_tag<=frame_tag;px<=0;py<=0;bank<=0;input_count<=0;output_count<=0;
     history0<=0;history1<=0;neighbor_q<=0;state<=GET;
    end
    GET:begin
     if(!inside_image)begin current_pixel<=0;state<=WRITE;end
     else if(in_valid&&in_ready)begin
      if(in_frame_tag!=active_tag)begin fault<=1;fault_reason<=1;end
      else if(in_x!=px-1||in_y!=py-1||input_count>=TOTAL)begin fault<=1;fault_reason<=2;end
      else begin
       current_pixel<={in_disparity,in_Z_mm,in_RGB565};input_count<=input_count+1'b1;state<=WRITE;
      end
     end
    end
    WRITE:begin if(px==0)begin history0<=0;history1<=0;end state<=READ0;end
    READ0:state<=READ1;
    READ1:begin top_pixel<=ram_q;state<=ASSEMBLE;end
    ASSEMBLE:begin
     // Capture centre/support BEFORE advancing the horizontal history.
     history0<=history1;history1<=column;
     if(emit_sample)begin
      centre_q<=centre;neighbor_q<=hits;out_frame_tag<=active_tag;out_x<=px-2;out_y<=py-2;state<=CHECK;
     end else advance_pixel;
    end
    CHECK:begin
     out_disparity<=centre_q[39:32];out_RGB565<=centre_q[15:0];
     out_Z_mm<=centre_q[31:16]!=0&&support_count>=2?centre_q[31:16]:16'd0;
     state<=EMIT;
    end
    EMIT:if(out_ready)begin
     if(output_count>=TOTAL)begin fault<=1;fault_reason<=3;end
     else begin output_count<=output_count+1'b1;advance_pixel;end
    end
    default:begin fault<=1;fault_reason<=4;end
   endcase
  end
 end
endmodule
