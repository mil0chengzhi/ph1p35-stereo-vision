`timescale 1ns/1ps
// An unusable left descriptor produces the same invalid record and minimum63
// without redundant candidate searches. Every native pixel is still emitted.
// Full native-pixel search, two consecutive disparities per three clocks (registered cost/penalty).
// One LEFT SGM path, followed by forward/reverse WTA reciprocal checking.
// This is deliberately not labelled four-path or independent reverse SGM.
// Every source pixel is processed; only bounds/descriptor/quality reject it.
module hd_sgm1_matcher #(
 parameter WIDTH=1280,HEIGHT=720,DMIN=72,DMAX=232,X_BITS=11,Y_BITS=10,P1=2,P2=16
)(
 input wire clk,rst,start_valid,output wire start_ready,
 input wire[31:0]pair_tag,input wire[4:0]cost_limit,uniqueness_margin,input wire[2:0]tolerance,
 input wire left_valid,output wire left_ready,input wire[23:0]left_descriptor,input wire left_usable,
 input wire[31:0]left_frame_tag,input wire[X_BITS-1:0]left_x,input wire[Y_BITS-1:0]left_y,
 input wire right_valid,output wire right_ready,input wire[23:0]right_descriptor,input wire right_usable,
 input wire[31:0]right_frame_tag,input wire[X_BITS-1:0]right_x,input wire[Y_BITS-1:0]right_y,
 output wire out_valid,input wire out_ready,output wire[31:0]out_frame_tag,
 output wire[X_BITS-1:0]out_x,output wire[Y_BITS-1:0]out_y,
 output wire[7:0]out_disparity,output wire out_usable,
 output wire busy,output reg frame_done,fault,output reg[3:0]fault_reason,output wire configuration_error,
 output reg[31:0]cycles_elapsed
);
 localparam ND=DMAX-DMIN+1,PAIRS=(ND+1)/2,PB=$clog2(PAIRS+1),HALF=WIDTH/2;
 localparam IDLE=0,LOAD=1,CLEAR=2,PIXEL=3,INIT=4,FETCH=5,CALC=6,FINISH=7,OUT0=8,OUT1=9,OUT2=10,EMIT=11,SCORE=12;
 localparam OK=WIDTH>=8&&WIDTH%2==0&&HEIGHT>=1&&WIDTH<=1280&&HEIGHT<=720&&DMIN>=1&&DMAX<WIDTH&&DMAX<=255&&ND>=2&&ND<=256&&P1>=1&&P2>P1&&P2<=24;
 reg[3:0]state;reg[31:0]tag;reg[4:0]limit_q,margin_q;reg[2:0]tolerance_q;
 reg[X_BITS:0]lc,rc;reg[X_BITS-1:0]x,emit_x,clear_pair;reg[Y_BITS-1:0]y;
 reg[PB-1:0]pair_index;
 reg[24:0]left_rows[0:WIDTH-1],right_even[0:HALF-1],right_odd[0:HALF-1];
 (* ram_style="bram" *)reg[19:0]reverse_even[0:HALF-1],reverse_odd[0:HALF-1];reg[8:0]forward[0:WIDTH-1];
 (* ram_style="bram" *)reg[11:0]horizontal[0:PAIRS-1];
 reg[24:0]left_q,right_even_q,right_odd_q;
 reg[19:0]reverse_even_hold,reverse_odd_hold;
 reg[19:0]reverse_even_q,reverse_odd_q;reg[8:0]forward_q;reg[11:0]horizontal_q;
 reg[23:0]left_hold;reg left_ok;reg[11:0]h_current;reg[5:0]h_lower,h_min,previous_min;
 // Parallel census cost and SGM predecessor penalty are registered before
 // the horizontal RAM write and forward/reverse WTA accumulation.
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[4:0]cost0_q,cost1_q;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[5:0]penalty0_q,penalty1_q;
 reg[19:0]best_record;reg[7:0]out_d;reg out_ok;
 wire[8:0]d0=DMIN+{pair_index,1'b0},d1=d0+1'b1;
 wire[X_BITS-1:0]r0=x>=d0?x-d0:0,r1=x>=d1?x-d1:0;
 wire[X_BITS-2:0]even_address=r0[0]?r1[X_BITS-1:1]:r0[X_BITS-1:1];
 wire[X_BITS-2:0]odd_address=r0[0]?r0[X_BITS-1:1]:r1[X_BITS-1:1];
 wire[24:0]right0=r0[0]?right_odd_q:right_even_q,right1=r1[0]?right_odd_q:right_even_q;
 wire[19:0]reverse0=r0[0]?reverse_odd_hold:reverse_even_hold,reverse1=r1[0]?reverse_odd_hold:reverse_even_hold;
 wire good0=left_ok&&right0[24]&&x>=d0&&d0<=DMAX;
 wire good1=left_ok&&right1[24]&&x>=d1&&d1<=DMAX;
 wire take_l=left_valid&&left_ready,take_r=right_valid&&right_ready;
 wire bad_l=take_l&&(left_frame_tag!=tag||left_y!=y||left_x!=lc);
 wire bad_r=take_r&&(right_frame_tag!=tag||right_y!=y||right_x!=rc);
 wire loaded_l=lc==WIDTH||(take_l&&!bad_l&&lc==WIDTH-1);
 wire loaded_r=rc==WIDTH||(take_r&&!bad_r&&right_x==WIDTH-1);
 function[4:0]hamming24(input[23:0]v);
  reg[2:0]a,b,c,d,e,f;reg[3:0]ab,cd,ef;
  begin
   a={2'd0,v[0]}+v[1]+v[2]+v[3];b={2'd0,v[4]}+v[5]+v[6]+v[7];
   c={2'd0,v[8]}+v[9]+v[10]+v[11];d={2'd0,v[12]}+v[13]+v[14]+v[15];
   e={2'd0,v[16]}+v[17]+v[18]+v[19];f={2'd0,v[20]}+v[21]+v[22]+v[23];
   ab={1'b0,a}+b;cd={1'b0,c}+d;ef={1'b0,e}+f;hamming24={1'b0,ab}+cd+ef;
  end
 endfunction
 // The arithmetic is modulo64, exactly as the original path()[5:0].
 function[5:0]penalty(input[5:0]same,lower,upper,minimum,input valid_previous);
  reg[7:0]a,b,c,m0,m1,m,v;
  begin
   if(!valid_previous||minimum==63)penalty=0;
   else begin
    a={2'd0,lower}+P1;b={2'd0,upper}+P1;c={2'd0,minimum}+P2;
    m0=same<a?same:a;m1=b<c?b:c;m=m0<m1?m0:m1;
    v=m-minimum;penalty=v[5:0];
   end
  end
 endfunction
 // {best6,second6,disparity8}; ties still keep the first d.
 // A usable score is <=24+P2<=48. Thus second==63 means fewer than
 // two usable candidates, without an explicit redundant count field.
 function[19:0]accumulate(input[19:0]old,input[5:0]score,input[7:0]disparity);
  begin
   if(score<old[19:14])accumulate={score,old[19:14],disparity};
   else if(score<old[13:8])accumulate={old[19:14],score,old[7:0]};
   else accumulate=old;
  end
 endfunction
 wire[5:0]new0=good0?({1'b0,cost0_q}+penalty0_q):6'd63;
 wire[5:0]new1=good1?({1'b0,cost1_q}+penalty1_q):6'd63;
 wire[19:0]after0=good0?accumulate(best_record,new0,d0[7:0]):best_record;
 wire[19:0]after1=good1?accumulate(after0,new1,d1[7:0]):after0;
 wire[19:0]rev_new0=accumulate(reverse0,new0,d0[7:0]),rev_new1=accumulate(reverse1,new1,d1[7:0]);
 wire[5:0]new_min01=new0<new1?new0:new1;
 wire[19:0]empty_record={6'd63,6'd63,8'd0};
 wire forward_good=best_record[13:8]!=6'd63&&best_record[19:14]<=limit_q&&({1'b0,best_record[13:8]}-{1'b0,best_record[19:14]})>=margin_q;
 wire[X_BITS-1:0]reciprocal=emit_x>=forward_q[7:0]?emit_x-forward_q[7:0]:0;
 wire[19:0]reverse_emit=reciprocal[0]?reverse_odd_q:reverse_even_q;
 wire reverse_good=reverse_emit[13:8]!=6'd63&&reverse_emit[19:14]<=limit_q&&({1'b0,reverse_emit[13:8]}-{1'b0,reverse_emit[19:14]})>=margin_q;
 wire[7:0]difference=forward_q[7:0]>reverse_emit[7:0]?forward_q[7:0]-reverse_emit[7:0]:reverse_emit[7:0]-forward_q[7:0];
 assign configuration_error=!OK;
 assign start_ready=!rst&&!fault&&OK&&state==IDLE&&cost_limit<=24&&uniqueness_margin>=1&&uniqueness_margin<=24&&tolerance<=2;
 assign busy=state!=IDLE;
 assign left_ready=!rst&&!fault&&state==LOAD&&lc<WIDTH;
 assign right_ready=!rst&&!fault&&state==LOAD&&rc<WIDTH;
 assign out_valid=!rst&&!fault&&state==EMIT;assign out_frame_tag=tag;assign out_x=emit_x;assign out_y=y;
 assign out_disparity=out_ok?out_d:8'd0;assign out_usable=out_ok;
 // One synchronous read and one synchronous write port per physical bank.
 wire reverse_read=state==FETCH||state==OUT1;
 wire reverse_even_write=state==CLEAR||(state==CALC&&((good0&&!r0[0])||(good1&&!r1[0])));
 wire reverse_odd_write=state==CLEAR||(state==CALC&&((good0&&r0[0])||(good1&&r1[0])));
 wire[X_BITS-2:0]reverse_even_ra=state==OUT1?reciprocal[X_BITS-1:1]:even_address;
 wire[X_BITS-2:0]reverse_odd_ra=state==OUT1?reciprocal[X_BITS-1:1]:odd_address;
 wire[X_BITS-2:0]reverse_even_wa=state==CLEAR?clear_pair[X_BITS-2:0]:even_address;
 wire[X_BITS-2:0]reverse_odd_wa=state==CLEAR?clear_pair[X_BITS-2:0]:odd_address;
 wire[19:0]reverse_even_wd=state==CLEAR?empty_record:(good0&&!r0[0])?rev_new0:rev_new1;
 wire[19:0]reverse_odd_wd=state==CLEAR?empty_record:(good0&&r0[0])?rev_new0:rev_new1;
 always @(posedge clk)begin
  if(reverse_read)reverse_even_q<=reverse_even[reverse_even_ra];
  if(reverse_even_write)reverse_even[reverse_even_wa]<=reverse_even_wd;
 end
 always @(posedge clk)begin
  if(reverse_read)reverse_odd_q<=reverse_odd[reverse_odd_ra];
  if(reverse_odd_write)reverse_odd[reverse_odd_wa]<=reverse_odd_wd;
 end
 always @(posedge clk)begin
  if(take_l&&!bad_l)left_rows[left_x]<={left_usable,left_descriptor};
  if(take_r&&!bad_r)begin
   if(right_x[0])right_odd[right_x[X_BITS-1:1]]<={right_usable,right_descriptor};
   else right_even[right_x[X_BITS-1:1]]<={right_usable,right_descriptor};
  end
  if(state==PIXEL)left_q<=left_rows[x];
  // Last-pair lookahead is unused; clamp it to zero rather than reading OOB.
  if(state==PIXEL||state==FETCH)horizontal_q<=horizontal[(state==PIXEL||pair_index==PAIRS-1)?0:pair_index+1'b1];
  if(state==FETCH)begin
   right_even_q<=right_even[even_address];right_odd_q<=right_odd[odd_address];
  end
  if(state==SCORE)begin
   reverse_even_hold<=reverse_even_q;reverse_odd_hold<=reverse_odd_q;
   cost0_q<=hamming24(left_hold^right0[23:0]);cost1_q<=hamming24(left_hold^right1[23:0]);
   penalty0_q<=penalty(h_current[5:0],h_lower,h_current[11:6],previous_min,x!=0);
   penalty1_q<=penalty(h_current[11:6],h_current[5:0],pair_index==PAIRS-1?6'd63:horizontal_q[5:0],previous_min,x!=0);
  end
  if(state==CALC)begin
   horizontal[pair_index]<={new1,new0};
  end
  if(state==FINISH)forward[x]<={forward_good,best_record[7:0]};
  if(state==OUT0)forward_q<=forward[emit_x];
 end
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;tag<=0;limit_q<=0;margin_q<=1;tolerance_q<=0;lc<=0;rc<=0;x<=0;y<=0;emit_x<=0;clear_pair<=0;pair_index<=0;
   left_hold<=0;left_ok<=0;h_current<=12'hfff;h_lower<=63;h_min<=63;previous_min<=63;best_record<=0;out_d<=0;out_ok<=0;
   frame_done<=0;fault<=0;fault_reason<=0;cycles_elapsed<=0;
  end else begin
   frame_done<=0;if(busy&&cycles_elapsed!=32'hffffffff)cycles_elapsed<=cycles_elapsed+1'b1;
   if(bad_l||bad_r)begin fault<=1;fault_reason<=bad_l?1:2;end
   else if(!fault)case(state)
    IDLE:if(start_valid&&start_ready)begin tag<=pair_tag;limit_q<=cost_limit;margin_q<=uniqueness_margin;tolerance_q<=tolerance;lc<=0;rc<=0;y<=0;cycles_elapsed<=0;state<=LOAD;end
    LOAD:begin if(take_l)lc<=lc+1'b1;if(take_r)rc<=rc+1'b1;if(loaded_l&&loaded_r)begin clear_pair<=0;state<=CLEAR;end end
    CLEAR:if(clear_pair==HALF-1)begin x<=0;previous_min<=63;state<=PIXEL;end else clear_pair<=clear_pair+1'b1;
    PIXEL:state<=INIT;
    INIT:begin left_hold<=left_q[23:0];left_ok<=left_q[24];h_current<=horizontal_q;h_lower<=63;h_min<=63;pair_index<=0;best_record<=empty_record;state<=left_q[24]?FETCH:FINISH;end
    FETCH:state<=SCORE;
    SCORE:state<=CALC;
    CALC:begin
     best_record<=after1;if(new_min01<h_min)h_min<=new_min01;
     h_lower<=h_current[11:6];h_current<=horizontal_q;
     if(pair_index==PAIRS-1)state<=FINISH;else begin pair_index<=pair_index+1'b1;state<=FETCH;end
    end
    FINISH:begin previous_min<=h_min;if(x==WIDTH-1)begin emit_x<=0;state<=OUT0;end else begin x<=x+1'b1;state<=PIXEL;end end
    OUT0:state<=OUT1;
    OUT1:state<=OUT2;
    OUT2:begin out_d<=forward_q[7:0];out_ok<=forward_q[8]&&emit_x>=forward_q[7:0]&&reverse_good&&difference<=tolerance_q;state<=EMIT;end
    EMIT:if(out_ready)begin if(emit_x!=WIDTH-1)begin emit_x<=emit_x+1'b1;state<=OUT0;end else if(y!=HEIGHT-1)begin y<=y+1'b1;lc<=0;rc<=0;state<=LOAD;end else begin state<=IDLE;frame_done<=1;end end
    default:begin fault<=1;fault_reason<=3;end
   endcase
  end
 end
 // synthesis translate_off
 always @(posedge clk)if(!rst&&!fault&&state==CALC)begin
  if(good0&&new0>48)$fatal(1,"Usable score reached sentinel range");
  if(good1&&new1>48)$fatal(1,"Usable score reached sentinel range");
 end
 // synthesis translate_on
endmodule
