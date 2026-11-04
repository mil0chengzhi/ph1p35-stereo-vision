`timescale 1ns/1ps
// One-bit sampled mask, synchronous RAM and bounded four-neighbour flood fill.
// Capturer must overwrite EVERY interior grid cell (including zeroes) before
// start. Border cells are never read. No RAM clear/reset and no RGB frame RAM.
// At native 160x90 grid, <=1536 foreground cells fit in 47500 pixel clocks;
// 720p has 49500 vertical-blank clocks. Watchdog rejects overload/overrun.
module sparse_component_selector #(
 parameter COLUMNS=160,ROWS=90,SHIFT=3,MAX_FOREGROUND=1536,
 parameter MIN_COUNT=8,MAX_WIDTH=384,MAX_HEIGHT=320,DEADLINE=47500
)(
 input wire clk,rst_n,abort,start,overload,
 input wire write_mask,mask_value,input wire[7:0]mask_x,input wire[6:0]mask_y,
 output wire busy,output reg done,found,failed,
 output reg[13:0]count,output reg[24:0]sum_x,output reg[23:0]sum_y,
 output reg[15:0]min_x,max_x,min_y,max_y
);
 localparam N=COLUMNS*ROWS,AW=$clog2(N),QW=$clog2(MAX_FOREGROUND+1);
 localparam IDLE=0,SCAN_READ=1,SCAN_CHECK=2,POP_READ=3,POP_CHECK=4,
            NEIGH_READ=5,NEIGH_CHECK=6,COMPLETE=7,FINISH=8;
 reg[3:0]state;reg[15:0]cycles;
 reg mask[0:N-1],mask_q;reg[14:0]queue[0:MAX_FOREGROUND-1],queue_q;
 reg[7:0]scan_x,node_x;reg[6:0]scan_y,node_y;
 reg[1:0]neighbour;reg[QW-1:0]head,tail;
 reg[13:0]component_count,best_count,second_count;
 reg[24:0]component_sum_x,best_sum_x;reg[23:0]component_sum_y,best_sum_y;
 reg[7:0]cx0,cx1,bx0,bx1;reg[6:0]cy0,cy1,by0,by1;
 reg scan_finished;
 function[AW-1:0]address;
  input[7:0]xx;input[6:0]yy;
  begin address=yy*COLUMNS+xx;end
 endfunction
 wire[7:0]nx=neighbour==0?node_x-1'b1:neighbour==1?node_x+1'b1:node_x;
 wire[6:0]ny=neighbour==2?node_y-1'b1:neighbour==3?node_y+1'b1:node_y;
 wire neighbour_valid=nx>=1&&nx<COLUMNS-1&&ny>=1&&ny<ROWS-1;
 wire read_mask=state==SCAN_READ||(state==NEIGH_READ&&neighbour_valid);
 wire[AW-1:0]read_address=state==SCAN_READ?address(scan_x,scan_y):address(nx,ny);
 wire clear_root=state==SCAN_CHECK&&mask_q;
 wire clear_neighbour=state==NEIGH_CHECK&&neighbour_valid&&mask_q;
 wire clear_mask=clear_root||clear_neighbour;
 wire[AW-1:0]clear_address=clear_root?address(scan_x,scan_y):address(nx,ny);
 wire queue_root=clear_root;
 wire queue_neighbour=clear_neighbour&&tail<MAX_FOREGROUND;
 // These RAM operations have no asynchronous reset, enabling ERAM inference.
 always @(posedge clk)begin
  if(read_mask)mask_q<=mask[read_address];
  if(write_mask&&state==IDLE)mask[address(mask_x,mask_y)]<=mask_value;
  else if(clear_mask)mask[clear_address]<=1'b0;
  if(state==POP_READ&&head<tail)queue_q<=queue[head];
  if(queue_root||queue_neighbour)queue[queue_root?{QW{1'b0}}:tail]<=queue_root?{scan_x,scan_y}:{nx,ny};
 end
 wire[15:0]cw=(({8'd0,cx1}-{8'd0,cx0})<<SHIFT)+1'b1;
 wire[15:0]ch=(({9'd0,cy1}-{9'd0,cy0})<<SHIFT)+1'b1;
 wire[14:0]grid_area=({7'd0,cx1}-{7'd0,cx0}+15'd1)*({8'd0,cy1}-{8'd0,cy0}+15'd1);
 wire component_valid=component_count>=MIN_COUNT&&cw>=2*(1<<SHIFT)&&ch>=2*(1<<SHIFT)&&
  cw<=MAX_WIDTH&&ch<=MAX_HEIGHT&&({2'd0,component_count}<<2)>=grid_area;
 assign busy=state!=IDLE;
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;cycles<=0;done<=0;found<=0;failed<=0;count<=0;sum_x<=0;sum_y<=0;
   min_x<=0;max_x<=0;min_y<=0;max_y<=0;scan_x<=1;scan_y<=1;node_x<=0;node_y<=0;
   neighbour<=0;head<=0;tail<=0;component_count<=0;component_sum_x<=0;component_sum_y<=0;
   best_count<=0;second_count<=0;best_sum_x<=0;best_sum_y<=0;cx0<=0;cx1<=0;cy0<=0;cy1<=0;
   bx0<=0;bx1<=0;by0<=0;by1<=0;scan_finished<=0;
  end else begin
   done<=0;
   if(state!=IDLE)cycles<=cycles+1'b1;
   case(state)
    IDLE:if(start)begin
     cycles<=0;best_count<=0;second_count<=0;best_sum_x<=0;best_sum_y<=0;
     scan_x<=1;scan_y<=1;scan_finished<=0;found<=0;failed<=overload;count<=0;
     state<=overload?FINISH:SCAN_READ;
    end
    SCAN_READ:state<=SCAN_CHECK;
    SCAN_CHECK:begin
     if(scan_x==COLUMNS-2)begin
      scan_x<=1;if(scan_y==ROWS-2)scan_finished<=1;else scan_y<=scan_y+1'b1;
     end else scan_x<=scan_x+1'b1;
     if(mask_q)begin
      component_count<=0;component_sum_x<=0;component_sum_y<=0;
      cx0<=255;cy0<=127;cx1<=0;cy1<=0;head<=0;tail<=1;state<=POP_READ;
     end else state<=(scan_x==COLUMNS-2&&scan_y==ROWS-2)?FINISH:SCAN_READ;
    end
    POP_READ:state<=head<tail?POP_CHECK:COMPLETE;
    POP_CHECK:begin
     node_x<=queue_q[14:7];node_y<=queue_q[6:0];head<=head+1'b1;
     component_count<=component_count+1'b1;
     component_sum_x<=component_sum_x+({17'd0,queue_q[14:7]}<<SHIFT);
     component_sum_y<=component_sum_y+({17'd0,queue_q[6:0]}<<SHIFT);
     if(queue_q[14:7]<cx0)cx0<=queue_q[14:7];if(queue_q[14:7]>cx1)cx1<=queue_q[14:7];
     if(queue_q[6:0]<cy0)cy0<=queue_q[6:0];if(queue_q[6:0]>cy1)cy1<=queue_q[6:0];
     neighbour<=0;state<=NEIGH_READ;
    end
    NEIGH_READ:state<=NEIGH_CHECK;
    NEIGH_CHECK:begin
     if(neighbour_valid&&mask_q)begin
      if(tail>=MAX_FOREGROUND)begin failed<=1;state<=FINISH;end
      else tail<=tail+1'b1;
     end
     if(!(neighbour_valid&&mask_q&&tail>=MAX_FOREGROUND))begin
      if(neighbour==3)state<=POP_READ;
      else begin neighbour<=neighbour+1'b1;state<=NEIGH_READ;end
     end
    end
    COMPLETE:begin
     if(component_valid)begin
      if(component_count>best_count)begin
       second_count<=best_count;best_count<=component_count;best_sum_x<=component_sum_x;best_sum_y<=component_sum_y;
       bx0<=cx0;bx1<=cx1;by0<=cy0;by1<=cy1;
      end else if(component_count>second_count)second_count<=component_count;
     end
     state<=scan_finished?FINISH:SCAN_READ;
    end
    FINISH:begin
     // Two similarly sized candidates are ambiguous; do not silently switch.
     found<=!failed&&best_count>=MIN_COUNT&&({1'b0,best_count}>=({1'b0,second_count}+({1'b0,second_count}>>1)));
     count<=best_count;sum_x<=best_sum_x;sum_y<=best_sum_y;
     min_x<={8'd0,bx0}<<SHIFT;max_x<={8'd0,bx1}<<SHIFT;
     min_y<={9'd0,by0}<<SHIFT;max_y<={9'd0,by1}<<SHIFT;
     done<=1;state<=IDLE;
    end
    default:begin failed<=1;state<=FINISH;end
   endcase
   if(state!=IDLE&&state!=FINISH&&cycles>=DEADLINE-1)begin failed<=1;state<=FINISH;end
   if(abort)begin state<=IDLE;done<=0;found<=0;failed<=1;end
  end
 end
endmodule
