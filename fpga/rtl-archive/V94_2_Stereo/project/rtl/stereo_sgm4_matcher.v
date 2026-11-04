`timescale 1ns/1ps
// Five-row vertical-band serial four-path Census SGM. LEFT, UP-LEFT, UP and UP-RIGHT paths;
// both WTA maps use the same aggregated cost volume, then reciprocal checking.
// No learned weights, hole filling or fabricated geometry. Every frame/row
// input is tagged and ordered. Synchronous RAM, no bulk RAM reset.
module stereo_sgm4_band_matcher #(
 parameter WIDTH=320,HEIGHT=180,DMIN=16,DMAX=59,MAX_DISPARITIES=128,
 parameter X_BITS=9,Y_BITS=8,P1=2,P2=16
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
 localparam IDLE=0,LOAD=1,CLEAR=2,PIXEL=3,INIT=4,FETCH=5,COST=6,PATH=7,
  COMMIT=8,FINISH_PIXEL=9,OFETCH=10,OREVERSE=11,OCHECK=12,EMIT=13,SCORE=14;
 localparam OK=WIDTH>=8&&WIDTH<=1280&&HEIGHT>=5&&HEIGHT<=720&&DMIN>=1&&DMAX<WIDTH&&
  DMAX<MAX_DISPARITIES&&ND>=2&&ND<=64&&X_BITS>=$clog2(WIDTH)&&Y_BITS>=$clog2(HEIGHT)&&
  P1>=1&&P1<=8&&P2>P1&&P2<=24; // C+4*P2<=120 fits the seven-bit aggregate.
 reg[3:0]state;
 reg[31:0]saved_pair;reg[5:0]saved_limit;reg[4:0]saved_margin;reg[2:0]saved_tolerance;
 reg[X_BITS:0]lc,rc;reg[X_BITS-1:0]x,clear_x,emit_x;reg[Y_BITS-1:0]y;
 reg[5:0]di;reg[AB-1:0]base;
 // Three left rows and five right rows provide a bounded two-row lookahead.
 reg[24:0]left_rows[0:3*WIDTH-1],right_rows[0:5*WIDTH-1];reg[24:0]left_q,right_q;
 reg[Y_BITS:0]loaded_rows;
 reg[1:0]left_load_bank,left_center_bank;
 reg[2:0]right_load_bank,right_center_bank,band_bank,band_index;
 reg[4:0]band_best;reg band_found;
 localparam LBITS=$clog2(3*WIDTH),RBITS=$clog2(5*WIDTH);
 wire[LBITS-1:0]left_write_address=left_load_bank*WIDTH+left_x;
 wire[RBITS-1:0]right_write_address=right_load_bank*WIDTH+right_x;
 wire[LBITS-1:0]left_read_address=left_center_bank*WIDTH+x;
 wire[2:0]first_band_bank=right_center_bank>=2?right_center_bank-3'd2:right_center_bank+3'd3;
 wire[2:0]next_band_bank=band_bank==4?3'd0:band_bank+3'd1;
 wire[2:0]right_read_bank=state==FETCH?first_band_bank:next_band_bank;
 wire signed[Y_BITS+2:0]candidate_y=$signed({1'b0,y})+$signed({1'b0,band_index})-2;
 wire band_inside=candidate_y>=0&&candidate_y<HEIGHT;
 wire[Y_BITS:0]goal_row=y+2<HEIGHT?y+2:HEIGHT-1;
 wire[Y_BITS:0]next_goal_row=y+3<HEIGHT?y+3:HEIGHT-1;
 reg[5:0]vertical[0:DEPTH-1],horizontal[0:ND-1],vertical_min[0:WIDTH-1];
 reg[5:0]h0,v0,hread,vread,hleft,vleft,vmin_q,previous_hmin,new_hmin,new_vmin;
 reg[23:0]left_hold;reg left_ok,candidate_ok;
 reg[4:0]raw_cost;reg[5:0]new_h,new_v,new_nw,new_ne;wire[6:0]score;
 // The NW old-row column is saved before overwrite. This avoids double
 // buffering an entire cost image; every large array has one read port.
 reg[5:0]vertical_nw[0:DEPTH-1],nw_cache[0:ND-1];
 (* ram_style="bram" *) reg[5:0]vertical_ne[0:DEPTH-1];
 reg[5:0]nw_min[0:WIDTH-1],ne_min[0:WIDTH-1];
 reg[5:0]nw_read,ne_read,nw_ram_read,nw_current,nw0,ne0,nwleft,neleft;
 reg[5:0]nw_min_q,ne_min_q,previous_nw_min,new_nw_min,new_ne_min;
 reg[6:0]score_ab,score_cd;
 wire[AB-1:0]ne_base=x==WIDTH-1?0:base+ND;
 wire[AB-1:0]ne_address=ne_base+di;
 wire[AB-1:0]ne_read_address=state==FETCH?ne_address:(di==ND-1?{AB{1'b0}}:ne_address+1'b1);
 // best7, second7, best disparity8, usable candidate count7.
 (* ram_style="bram" *) reg[28:0]reverse_best[0:WIDTH-1];reg[28:0]reverse_q;
 reg[8:0]forward_best[0:WIDTH-1];reg[8:0]forward_q;
 reg[6:0]best,second,count;reg[7:0]best_d,out_d;reg out_ok;
 wire[7:0]d=DMIN+di;
 wire[AB-1:0]address=base+di;
 wire[AB-1:0]vread_address=state==FETCH?address:(di==ND-1?{AB{1'b0}}:address+1'b1);
 wire[5:0]hread_address=state==FETCH?di:(di==ND-1?6'd0:di+1'b1);
 wire candidate_inside=x>=d;
 wire[X_BITS-1:0]right_address=candidate_inside?x-d:0;
 wire[RBITS-1:0]right_band_address=right_read_bank*WIDTH+right_address;
 wire current_band_ok=left_ok&&right_q[24]&&candidate_inside&&band_inside;
 wire[4:0]current_band_cost=hamming24(left_hold^right_q[23:0]);
 wire[4:0]updated_band_cost=current_band_ok&&(!band_found||current_band_cost<band_best)?current_band_cost:band_best;
 wire[X_BITS-1:0]reciprocal_address=forward_q[8]&&emit_x>=forward_q[7:0]?emit_x-forward_q[7:0]:0;
 wire take_l=left_valid&&left_ready,take_r=right_valid&&right_ready;
 wire bad_l=take_l&&(left_frame_tag!=saved_pair||left_y!=loaded_rows||left_x!=lc);
 wire bad_r=take_r&&(right_frame_tag!=saved_pair||right_y!=loaded_rows||right_x!=rc);
 wire ldone=lc==WIDTH||(take_l&&!bad_l&&lc==WIDTH-1);
 wire rdone=rc==WIDTH||(take_r&&!bad_r&&rc==WIDTH-1);
 wire[6:0]reverse_cost=reverse_q[28:22],reverse_second=reverse_q[21:15];
 wire[7:0]reverse_d=reverse_q[14:7];wire[6:0]reverse_count=reverse_q[6:0];
 wire reverse_usable=reverse_count>=2&&reverse_cost<=saved_limit&&({1'b0,reverse_second}-{1'b0,reverse_cost})>=saved_margin;
 wire[7:0]difference=forward_q[7:0]>reverse_d?forward_q[7:0]-reverse_d:reverse_d-forward_q[7:0];
 wire reverse_we=state==CLEAR||(state==COMMIT&&candidate_ok);
 wire[X_BITS-1:0]reverse_write_address=state==CLEAR?clear_x:right_address;
 wire[28:0]reverse_write_data=state==CLEAR?{7'd127,7'd127,8'd0,7'd0}:
  score<reverse_cost?{score,reverse_cost,d,reverse_count+7'd1}:
  score<reverse_second?{reverse_cost,score,reverse_d,reverse_count+7'd1}:
  {reverse_cost,reverse_second,reverse_d,reverse_count+7'd1};
 wire reverse_re=state==FETCH||state==OREVERSE;
 wire[X_BITS-1:0]reverse_read_address=state==FETCH?right_address:reciprocal_address;
 assign configuration_error=!OK;
 wire parameters_ok=disparity_min==DMIN&&disparity_max==DMAX&&cost_limit<=24&&
  uniqueness_margin>=1&&uniqueness_margin<=24&&tolerance<=2;
 assign start_ready=!rst&&!fault&&OK&&parameters_ok&&state==IDLE;
 assign busy=state!=IDLE;
 assign left_ready=!rst&&!fault&&state==LOAD&&lc<WIDTH;
 assign right_ready=!rst&&!fault&&state==LOAD&&rc<WIDTH;
 assign out_valid=!rst&&!fault&&state==EMIT;
 assign out_frame_tag=saved_pair;assign out_x=emit_x;assign out_y=y;
 assign out_disparity=out_ok?out_d:8'd0;assign out_usable=out_ok;
 function[4:0]hamming24;
  input[23:0]v;integer j;reg[4:0]sum;
  begin sum=0;for(j=0;j<24;j=j+1)sum=sum+v[j];hamming24=sum;end
 endfunction
 function[5:0]path_cost;
  input[4:0]c;input[5:0]same,lower,upper,minimum;input has_previous;
  reg[7:0]a,b,e,m,value;
  begin
   if(!has_previous||minimum==63)path_cost={1'b0,c};
   else begin
    a={2'd0,lower}+P1;b={2'd0,upper}+P1;e={2'd0,minimum}+P2;
    m=same;if(a<m)m=a;if(b<m)m=b;if(e<m)m=e;
    value={3'd0,c}+m-minimum;path_cost=value[5:0];
   end
  end
 endfunction
 // Time-share ONE read port over FETCH/COST, then write in COMMIT. Keeping
 // a single physical read register prevents TD mapping a second port into
 // thousands of distributed-RAM LUTs. No new pipeline cycles are needed.
 always @(posedge clk)begin
  if(!rst&&!fault)begin
   if(take_l&&!bad_l)left_rows[left_write_address]<={left_usable,left_descriptor};
   if(take_r&&!bad_r)right_rows[right_write_address]<={right_usable,right_descriptor};
   if(state==PIXEL)begin left_q<=left_rows[left_read_address];vmin_q<=vertical_min[x];
    nw_min_q<=nw_min[x];ne_min_q<=x==WIDTH-1?6'd63:ne_min[x+1'b1];end
   if(state==FETCH||(state==COST&&band_index<4))begin
    right_q<=right_rows[right_band_address];
   end
   if(state==FETCH||state==COST)begin hread<=horizontal[hread_address];vread<=vertical[vread_address];
    nw_read<=nw_cache[hread_address];ne_read<=vertical_ne[ne_read_address];nw_ram_read<=vertical_nw[vread_address];end
   if(state==COMMIT)begin vertical[address]<=new_v;horizontal[di]<=new_h;
    vertical_nw[address]<=new_nw;vertical_ne[address]<=new_ne;nw_cache[di]<=nw_current;end
   if(state==FINISH_PIXEL)begin
    vertical_min[x]<=new_vmin;nw_min[x]<=new_nw_min;ne_min[x]<=new_ne_min;
    forward_best[x]<={count>=2&&best<=saved_limit&&({1'b0,second}-{1'b0,best})>=saved_margin,best_d};
   end
   if(reverse_we)reverse_best[reverse_write_address]<=reverse_write_data;
   if(reverse_re)reverse_q<=reverse_best[reverse_read_address];
   if(state==OFETCH)forward_q<=forward_best[emit_x];
  end
 end
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;saved_pair<=0;saved_limit<=0;saved_margin<=0;saved_tolerance<=0;
   lc<=0;rc<=0;x<=0;y<=0;clear_x<=0;emit_x<=0;di<=0;base<=0;
   loaded_rows<=0;left_load_bank<=0;left_center_bank<=0;right_load_bank<=0;right_center_bank<=0;
   band_bank<=0;band_index<=0;band_best<=24;band_found<=0;
   hleft<=63;vleft<=63;previous_hmin<=63;new_hmin<=63;new_vmin<=63;
   left_hold<=0;left_ok<=0;candidate_ok<=0;raw_cost<=0;new_h<=63;new_v<=63;new_nw<=63;new_ne<=63;
   nw0<=63;ne0<=63;nwleft<=63;neleft<=63;previous_nw_min<=63;new_nw_min<=63;new_ne_min<=63;score_ab<=0;score_cd<=0;
   best<=127;second<=127;count<=0;best_d<=0;out_d<=0;out_ok<=0;
   frame_done<=0;fault<=0;fault_reason<=0;
  end else begin
   frame_done<=0;
   if(bad_l||bad_r)begin fault<=1;fault_reason<=bad_l?1:2;end
   else if(!fault)case(state)
    IDLE:if(start_valid&&start_ready)begin
     saved_pair<=pair_tag;saved_limit<={cost_limit,1'b0};saved_margin<=uniqueness_margin;saved_tolerance<=tolerance;
     lc<=0;rc<=0;y<=0;state<=LOAD;loaded_rows<=0;
     left_load_bank<=0;left_center_bank<=0;right_load_bank<=0;right_center_bank<=0;
    end
    LOAD:begin
     if(take_l)lc<=lc+1'b1;if(take_r)rc<=rc+1'b1;
     if(ldone&&rdone)begin
      loaded_rows<=loaded_rows+1'b1;lc<=0;rc<=0;
      left_load_bank<=left_load_bank==2?0:left_load_bank+1'b1;
      right_load_bank<=right_load_bank==4?0:right_load_bank+1'b1;
      if(loaded_rows>=goal_row)begin clear_x<=0;state<=CLEAR;end
     end
    end
    CLEAR:if(clear_x==WIDTH-1)begin x<=0;base<=0;previous_hmin<=63;previous_nw_min<=63;state<=PIXEL;end else clear_x<=clear_x+1'b1;
    PIXEL:state<=INIT;
    INIT:begin
     left_hold<=left_q[23:0];left_ok<=left_q[24];di<=0;hleft<=63;vleft<=63;nwleft<=63;neleft<=63;
     new_hmin<=63;new_vmin<=63;new_nw_min<=63;new_ne_min<=63;best<=127;second<=127;best_d<=0;count<=0;state<=FETCH;
    end
    FETCH:begin
     band_bank<=first_band_bank;band_index<=0;band_best<=24;band_found<=0;state<=COST;
    end
    COST:begin
     if(band_index==0)begin
      h0<=hread;v0<=vread;nw0<=nw_read;ne0<=ne_read;nw_current<=nw_ram_read;
     end
     band_best<=updated_band_cost;band_found<=band_found||current_band_ok;
     if(band_index==4)begin
      raw_cost<=updated_band_cost;candidate_ok<=band_found||current_band_ok;state<=PATH;
     end else begin band_index<=band_index+1'b1;band_bank<=next_band_bank;end
    end
    PATH:begin
     new_h<=candidate_ok?path_cost(raw_cost,h0,hleft,di==ND-1?6'd63:hread,previous_hmin,x!=0):6'd63;
     new_v<=candidate_ok?path_cost(raw_cost,v0,vleft,di==ND-1?6'd63:vread,vmin_q,y!=0):6'd63;
     new_nw<=candidate_ok?path_cost(raw_cost,nw0,nwleft,di==ND-1?6'd63:nw_read,previous_nw_min,y!=0&&x!=0):6'd63;
     new_ne<=candidate_ok?path_cost(raw_cost,ne0,neleft,di==ND-1?6'd63:ne_read,ne_min_q,y!=0&&x!=WIDTH-1):6'd63;
     state<=SCORE;
    end
    SCORE:begin score_ab<={1'b0,new_h}+{1'b0,new_v};score_cd<={1'b0,new_nw}+{1'b0,new_ne};state<=COMMIT;end
    COMMIT:begin
     // Combined score is evaluated from registered normalized paths.
     hleft<=h0;vleft<=v0;nwleft<=nw0;neleft<=ne0;
     if(new_h<new_hmin)new_hmin<=new_h;if(new_v<new_vmin)new_vmin<=new_v;
     if(new_nw<new_nw_min)new_nw_min<=new_nw;if(new_ne<new_ne_min)new_ne_min<=new_ne;
     if(candidate_ok)begin
      count<=count+1'b1;
      if(score<best)begin second<=best;best<=score;best_d<=d;end
      else if(score<second)second<=score;
     end
     if(di==ND-1)state<=FINISH_PIXEL;else begin di<=di+1'b1;state<=FETCH;end
    end
    FINISH_PIXEL:begin
     previous_hmin<=new_hmin;previous_nw_min<=nw_min_q;
     if(x==WIDTH-1)begin emit_x<=0;state<=OFETCH;end
     else begin x<=x+1'b1;base<=base+ND;state<=PIXEL;end
    end
    OFETCH:state<=OREVERSE;
    OREVERSE:state<=OCHECK;
    OCHECK:begin out_d<=forward_q[7:0];out_ok<=forward_q[8]&&emit_x>=forward_q[7:0]&&reverse_usable&&difference<=saved_tolerance;state<=EMIT;end
    EMIT:if(out_ready)begin
     if(emit_x==WIDTH-1)begin
      if(y==HEIGHT-1)begin state<=IDLE;frame_done<=1;end
      else begin
       y<=y+1'b1;lc<=0;rc<=0;
       left_center_bank<=left_center_bank==2?0:left_center_bank+1'b1;
       right_center_bank<=right_center_bank==4?0:right_center_bank+1'b1;
       if(loaded_rows>next_goal_row)begin clear_x<=0;state<=CLEAR;end else state<=LOAD;
      end
     end else begin emit_x<=emit_x+1'b1;state<=OFETCH;end
    end
    default:begin fault<=1;fault_reason<=3;end
   endcase
  end
 end
 // Combinational score from registered path results, stable in COMMIT.
 assign score={2'd0,score_ab}+{2'd0,score_cd}-{4'd0,raw_cost}-({4'd0,raw_cost}<<1);
endmodule
