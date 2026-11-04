`timescale 1ns/1ps
// Isolated, unqualified 5x5 Census-cost aggregation. No fabricated hole filling.
// Every one of25 supports must be usable; stable lower-disparity tie break.
// Forward/reverse costs are the SAME support sum at xL,xR=xL-d.
// Four raw-cost line banks replace the oldest row only after its read completes.
// Frame/row starts gate old RAM contents; RAM is never asynchronously reset.
module stereo_streamed_row_matcher #(
 parameter WIDTH=320,HEIGHT=180,DMIN=16,DMAX=59,MAX_DISPARITIES=128,X_BITS=9,Y_BITS=8
)(
 input wire clk,rst,start_valid,output wire start_ready,
 input wire[31:0]pair_tag,input wire[7:0]disparity_min,disparity_max,
 input wire[4:0]cost_limit,uniqueness_margin,input wire[2:0]tolerance,
 input wire left_valid,output wire left_ready,input wire[23:0]left_descriptor,
 input wire left_usable,input wire[31:0]left_frame_tag,input wire[X_BITS-1:0]left_x,input wire[Y_BITS-1:0]left_y,
 input wire right_valid,output wire right_ready,input wire[23:0]right_descriptor,
 input wire right_usable,input wire[31:0]right_frame_tag,input wire[X_BITS-1:0]right_x,input wire[Y_BITS-1:0]right_y,
 output wire out_valid,input wire out_ready,output wire[31:0]out_frame_tag,
 output wire[X_BITS-1:0]out_x,output wire[Y_BITS-1:0]out_y,
 output wire[7:0]out_disparity,output wire out_usable,
 output wire busy,output reg frame_done,fault,output reg[3:0]fault_reason,
 output wire configuration_error
);
 localparam ND=DMAX-DMIN+1,AB=$clog2(WIDTH*ND),DEPTH=1<<AB;
 localparam OK=WIDTH>=9&&HEIGHT>=9&&WIDTH<=1280&&HEIGHT<=720&&DMIN>=1&&DMAX<WIDTH&&
  DMAX<MAX_DISPARITIES&&ND>=2&&ND<=64&&X_BITS>=$clog2(WIDTH)&&Y_BITS>=$clog2(HEIGHT);
 localparam IDLE=0,LOAD=1,CLEAR=2,FETCH=3,VERTICAL=4,COMMIT=5,LREAD=6,RREAD=7,
  CHECK=8,EMIT=9,PAD=10;
 reg[3:0]state;
 reg[31:0]saved_pair;reg[9:0]saved_cost;reg[4:0]saved_margin;reg[2:0]saved_tol;
 reg[Y_BITS:0]raw_y;reg[X_BITS:0]raw_x,lcount,rcount,clear_x,emit_x;
 reg[Y_BITS-1:0]emit_y;reg[7:0]di;reg[AB-1:0]address;
 reg[24:0]left_rows[0:WIDTH-1],right_rows[0:WIDTH-1];reg[24:0]lq,rq;
 reg[5:0]line0[0:DEPTH-1],line1[0:DEPTH-1],line2[0:DEPTH-1],line3[0:DEPTH-1];
 reg[5:0]q0,q1,q2,q3;
 // Each10-bit vertical support contains {bad_count3,sum7}; four predecessors.
 reg[39:0]horizontal[0:ND-1];reg[39:0]hq,hist;
 reg[34:0]reverse_best[0:WIDTH-1];reg[34:0]reverse_q;
 reg[8:0]forward_best[0:WIDTH-1];reg[8:0]forward_q;
 reg[9:0]best,second;reg[7:0]best_d;reg[6:0]count;
 reg[6:0]vertical_sum;reg[2:0]vertical_bad;reg[5:0]raw_cost;
 reg[7:0]out_d;reg out_ok;
 wire[7:0]d=DMIN+di;
 wire cx_valid=raw_x>=2&&raw_x<WIDTH+2;
 wire[X_BITS:0]cx=raw_x-2;
 wire reverse_address_valid=cx_valid&&cx>=d&&cx-d<WIDTH;
 wire[X_BITS-1:0]reverse_address=cx-d;
 wire[X_BITS-1:0]right_fetch_address=raw_x-d;
 wire[X_BITS-1:0]LR_address=emit_x-forward_q[7:0];
 wire reverse_read_enable=state==FETCH||state==RREAD;
 wire[X_BITS-1:0]reverse_read_address=state==FETCH?(reverse_address_valid?reverse_address:{X_BITS{1'b0}}):
  (forward_q[8]&&emit_x>=forward_q[7:0]?LR_address:{X_BITS{1'b0}});
 assign configuration_error=!OK;
 wire job_ok=disparity_min==DMIN&&disparity_max==DMAX&&cost_limit<=24&&
  uniqueness_margin>=1&&uniqueness_margin<=24&&tolerance<=2;
 assign busy=state!=IDLE;
 assign start_ready=!rst&&!fault&&OK&&job_ok&&state==IDLE;
 assign left_ready=!rst&&!fault&&state==LOAD&&lcount<WIDTH;
 assign right_ready=!rst&&!fault&&state==LOAD&&rcount<WIDTH;
 wire take_l=left_valid&&left_ready,take_r=right_valid&&right_ready;
 wire bad_l=take_l&&(left_frame_tag!=saved_pair||left_y!=raw_y||left_x!=lcount);
 wire bad_r=take_r&&(right_frame_tag!=saved_pair||right_y!=raw_y||right_x!=rcount);
 wire lcomplete=lcount==WIDTH||(take_l&&!bad_l&&lcount==WIDTH-1);
 wire rcomplete=rcount==WIDTH||(take_r&&!bad_r&&rcount==WIDTH-1);
 assign out_valid=!rst&&!fault&&(state==EMIT||state==PAD);
 assign out_frame_tag=saved_pair;assign out_x=emit_x[X_BITS-1:0];assign out_y=emit_y;
 assign out_disparity=state==PAD?8'd0:out_d;assign out_usable=state==PAD?1'b0:out_ok;
 function[4:0]hamming24;
  input[23:0]bits;reg[2:0]a,b,c,e,f,g;reg[3:0]ab,ce,fg;
  begin
   a={2'd0,bits[0]}+bits[1]+bits[2]+bits[3];b={2'd0,bits[4]}+bits[5]+bits[6]+bits[7];
   c={2'd0,bits[8]}+bits[9]+bits[10]+bits[11];e={2'd0,bits[12]}+bits[13]+bits[14]+bits[15];
   f={2'd0,bits[16]}+bits[17]+bits[18]+bits[19];g={2'd0,bits[20]}+bits[21]+bits[22]+bits[23];
   ab={1'b0,a}+b;ce={1'b0,c}+e;fg={1'b0,f}+g;hamming24={1'b0,ab}+ce+fg;
  end
 endfunction
 wire raw_good=lq[24]&&rq[24]&&raw_x<WIDTH&&raw_x>=d&&raw_y<HEIGHT;
 wire[5:0]new_raw={raw_good,hamming24(lq[23:0]^rq[23:0])};
 // All missing rows are explicitly unusable, independent of stale RAM bits.
 // Before row4 only raw_y previous rows exist, whichever bank received them.
 wire[5:0]v0=(raw_y<4&&raw_y<=0)?6'd0:q0;
 wire[5:0]v1=(raw_y<4&&raw_y<=1)?6'd0:q1;
 wire[5:0]v2=(raw_y<4&&raw_y<=2)?6'd0:q2;
 wire[5:0]v3=(raw_y<4&&raw_y<=3)?6'd0:q3;
 wire[39:0]fresh_hist=raw_x==0?{4{10'b1010000000}}:hq;
 wire[9:0]cost_sum={3'd0,vertical_sum}+{3'd0,hist[6:0]}+{3'd0,hist[16:10]}+
                  {3'd0,hist[26:20]}+{3'd0,hist[36:30]};
 wire[5:0]bad_sum={3'd0,vertical_bad}+{3'd0,hist[9:7]}+{3'd0,hist[19:17]}+
                 {3'd0,hist[29:27]}+{3'd0,hist[39:37]};
 wire support_good=raw_y>=6&&raw_y<HEIGHT-2&&raw_x>=6&&raw_x<WIDTH-2&&
                   cx>=d+4&&bad_sum==0;
 // Per-pixel WTA next values include the current candidate, including the last.
 reg[9:0]next_best,next_second;reg[7:0]next_d;reg[6:0]next_count;
 reg[9:0]rbest,rsecond,rnext_best,rnext_second;reg[7:0]rd,rnext_d;reg[6:0]rn,rnext_count;
 wire reverse_write_enable=state==CLEAR||(state==COMMIT&&raw_y>=2&&support_good&&reverse_address_valid);
 wire[X_BITS-1:0]reverse_write_address=state==CLEAR?clear_x[X_BITS-1:0]:reverse_address;
 wire[34:0]reverse_write_data=state==CLEAR?{7'd0,8'd0,10'd1023,10'd1023}:{rnext_count,rnext_d,rnext_second,rnext_best};
 always @* begin
  next_best=di==0?10'd1023:best;next_second=di==0?10'd1023:second;
  next_d=di==0?8'd0:best_d;next_count=di==0?7'd0:count;
  if(support_good)begin
   next_count=next_count+1'b1;
   if(cost_sum<next_best)begin next_second=next_best;next_best=cost_sum;next_d=d;end
   else if(cost_sum<next_second)next_second=cost_sum;
  end
  rbest=reverse_q[9:0];rsecond=reverse_q[19:10];rd=reverse_q[27:20];rn=reverse_q[34:28];
  rnext_best=rbest;rnext_second=rsecond;rnext_d=rd;rnext_count=rn;
  if(support_good&&reverse_address_valid)begin
   rnext_count=rn+1'b1;
   if(cost_sum<rbest)begin rnext_second=rbest;rnext_best=cost_sum;rnext_d=d;end
   else if(cost_sum<rsecond)rnext_second=cost_sum;
  end
 end
 wire forward_good=next_count>=2&&next_best<=saved_cost&&
                   ({1'b0,next_second}-{1'b0,next_best}>=saved_margin);
 wire reverse_good=rn>=2&&rbest<=saved_cost&&({1'b0,rsecond}-{1'b0,rbest}>=saved_margin);
 wire[7:0]difference=forward_q[7:0]>=rd?forward_q[7:0]-rd:rd-forward_q[7:0];
 // Synchronous RAM ports. No RAM array is reset or read combinationally.
 always @(posedge clk)begin
  if(take_l)left_rows[left_x]<={left_usable,left_descriptor};
  if(take_r)right_rows[right_x]<={right_usable,right_descriptor};
  if(!rst&&!fault&&state==FETCH)begin
   lq<=raw_x<WIDTH?left_rows[raw_x[X_BITS-1:0]]:25'd0;
   rq<=raw_x>=d&&raw_x-d<WIDTH?right_rows[right_fetch_address]:25'd0;
   q0<=line0[address];q1<=line1[address];q2<=line2[address];q3<=line3[address];
   hq<=horizontal[di];

  end
  if(!rst&&!fault&&reverse_read_enable)reverse_q<=reverse_best[reverse_read_address];
  if(!rst&&!fault&&reverse_write_enable)reverse_best[reverse_write_address]<=reverse_write_data;
  if(!rst&&!fault&&state==COMMIT)begin
   if(raw_x<WIDTH)case(raw_y[1:0])
    0:line0[address]<=raw_cost;1:line1[address]<=raw_cost;
    2:line2[address]<=raw_cost;3:line3[address]<=raw_cost;
   endcase
   horizontal[di]<={hist[29:0],vertical_bad,vertical_sum};
   if(raw_y>=2&&cx_valid&&di==ND-1)forward_best[cx[X_BITS-1:0]]<={forward_good,next_d};

  end
  if(!rst&&!fault&&state==LREAD)forward_q<=forward_best[emit_x[X_BITS-1:0]];

 end
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;saved_pair<=0;saved_cost<=0;saved_margin<=0;saved_tol<=0;
   raw_y<=0;raw_x<=0;lcount<=0;rcount<=0;clear_x<=0;emit_x<=0;emit_y<=0;di<=0;address<=0;
   vertical_sum<=0;vertical_bad<=0;raw_cost<=0;hist<=0;best<=1023;second<=1023;best_d<=0;count<=0;
   out_d<=0;out_ok<=0;frame_done<=0;fault<=0;fault_reason<=0;
  end else begin
   frame_done<=0;
   if(bad_l||bad_r)begin fault<=1;fault_reason<=bad_l?1:2;end
   else if(!fault)case(state)
    IDLE:if(start_valid&&start_ready)begin
     saved_pair<=pair_tag;saved_cost<=({5'd0,cost_limit}<<4)+({5'd0,cost_limit}<<3)+{5'd0,cost_limit};
     saved_margin<=uniqueness_margin;saved_tol<=tolerance;
     raw_y<=0;raw_x<=0;lcount<=0;rcount<=0;di<=0;address<=0;state<=LOAD;
    end
    LOAD:begin
     if(take_l)lcount<=lcount+1'b1;if(take_r)rcount<=rcount+1'b1;
     if(lcomplete&&rcomplete)begin clear_x<=0;raw_x<=0;di<=0;address<=0;state<=CLEAR;end
    end
    CLEAR:if(clear_x==WIDTH-1)state<=FETCH;else clear_x<=clear_x+1'b1;
    FETCH:state<=VERTICAL;
    VERTICAL:begin
     raw_cost<=new_raw;hist<=fresh_hist;
     vertical_sum<={2'd0,new_raw[4:0]}+{2'd0,v0[4:0]}+{2'd0,v1[4:0]}+{2'd0,v2[4:0]}+{2'd0,v3[4:0]};
     vertical_bad<={2'd0,!new_raw[5]}+{2'd0,!v0[5]}+{2'd0,!v1[5]}+{2'd0,!v2[5]}+{2'd0,!v3[5]};
     state<=COMMIT;
    end
    COMMIT:begin
     best<=next_best;second<=next_second;best_d<=next_d;count<=next_count;
     address<=address+1'b1;
     if(di==ND-1)begin
      di<=0;
      if(raw_x==WIDTH+1)begin
       if(raw_y>=2)begin emit_x<=0;emit_y<=raw_y-2;state<=LREAD;end
       else begin raw_y<=raw_y+1'b1;lcount<=0;rcount<=0;state<=LOAD;end
      end else begin raw_x<=raw_x+1'b1;state<=FETCH;end
     end else begin di<=di+1'b1;state<=FETCH;end
    end
    LREAD:state<=RREAD;
    RREAD:state<=CHECK;
    CHECK:begin
     out_ok<=forward_q[8]&&emit_x>=forward_q[7:0]&&reverse_good&&difference<=saved_tol;
     out_d<=forward_q[8]&&emit_x>=forward_q[7:0]&&reverse_good&&difference<=saved_tol?forward_q[7:0]:0;
     state<=EMIT;
    end
    EMIT:if(out_ready)begin
     if(emit_x==WIDTH-1)begin
      if(raw_y==HEIGHT-1)begin emit_y<=HEIGHT-2;emit_x<=0;state<=PAD;end
      else begin raw_y<=raw_y+1'b1;lcount<=0;rcount<=0;state<=LOAD;end
     end else begin emit_x<=emit_x+1'b1;state<=LREAD;end
    end
    PAD:if(out_ready)begin
     if(emit_x==WIDTH-1)begin
      if(emit_y==HEIGHT-1)begin state<=IDLE;frame_done<=1;end
      else begin emit_y<=emit_y+1'b1;emit_x<=0;end
     end else emit_x<=emit_x+1'b1;
    end
    default:begin fault<=1;fault_reason<=3;end
   endcase
  end
 end
endmodule
