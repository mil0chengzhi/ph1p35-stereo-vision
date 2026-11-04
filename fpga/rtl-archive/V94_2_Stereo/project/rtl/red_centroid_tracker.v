`timescale 1ns/1ps
// Lightweight B-local visual-control measurement, before any OSD. No RGB frame RAM.
// Complete new native rasters only; sparse colour samples produce a centroid
// and bounds of the dominant four-connected colour component. It identifies
// a selected colour region, not a semantic object. Similar-size candidates and
// overload are rejected. The A-board keeps the full multi-target OSD.
module red_centroid_tracker #(
 parameter WIDTH=1280,HEIGHT=720,SAMPLE_SHIFT=3,
 parameter ROI_X0=128,ROI_X1=1151,ROI_Y0=64,ROI_Y1=655,
 parameter MIN_RED=8,MAX_BOX_WIDTH=384,MAX_BOX_HEIGHT=320
)(
 input wire clk,rst_n,input wire [1:0]colour_select,input wire [15:0]x,y,input wire [23:0]rgb,
 input wire de,pixel_valid,source_valid,input wire[31:0]source_frame,
 output reg result_commit,result_valid,found,output reg[1:0]result_colour,
 output reg[31:0]result_frame,output reg[15:0]result_sequence,
 output reg[15:0]center_x,center_y,min_x,max_x,min_y,max_y,
 output reg[13:0]red_samples,output reg[3:0]reason,output wire busy
);
 localparam N=((WIDTH+(1<<SAMPLE_SHIFT)-1)>>SAMPLE_SHIFT)*((HEIGHT+(1<<SAMPLE_SHIFT)-1)>>SAMPLE_SHIFT);
 localparam IDLE=0,DIVX=1,DIVY=2,PUBLISH=3,COMPONENT=4;
 reg[2:0]state;
 reg active,seen,bad,complete;
 reg[31:0]last_id,working_id,held_frame;
 reg[15:0]expected_x,expected_y;
 reg[14:0]all_samples;
 reg[13:0]red_count,denominator;
 reg[24:0]x_total,dividend,quotient;
 reg[23:0]y_total,held_y;
 reg[15:0]x0,x1,y0,y1;
 reg[14:0]remainder;
 reg[4:0]division_bit;
 wire origin=de&&x==0&&y==0;
 wire fresh=source_valid&&(!seen||last_id!=source_frame);
 wire taking=(origin?fresh:active)&&source_valid;
 wire take=x[SAMPLE_SHIFT-1:0]==0&&y[SAMPLE_SHIFT-1:0]==0;
 wire[7:0]r=rgb[23:16],g=rgb[15:8],b=rgb[7:0];
 wire[7:0]gb_difference=g>b?g-b:b-g;
 wire colour_red=r>=48&&{1'b0,r}>{1'b0,g}+9'd16&&{1'b0,r}>{1'b0,b}+9'd20&&
                 ({3'd0,r}<<2)>(({3'd0,g}<<2)+{3'd0,g})&&gb_difference<=64;
 // One selected colour shares the original sparse rows/divider/centroid.
 // Selection is stamped into each result; mixed-colour frames are invalid.
 reg[1:0]working_colour,held_colour;
 wire[1:0]current_colour=origin?colour_select:working_colour;
 wire[7:0]max_rb=r>b?r:b,min_rb=r<b?r:b;
 wire[7:0]max_rgb=g>max_rb?g:max_rb,min_rgb=g<min_rb?g:min_rb;
 wire[7:0]span=max_rgb-min_rgb;
 wire[7:0]min_rg=r<g?r:g;
 wire chroma_ok=max_rgb>=48&&{span,4'b0000}>={4'd0,max_rgb};
 // Conservative yellow for this pale block: reject warm cardboard. Below
 // this illumination the target is absent and motion stops; never brighten
 // the detector independently of the actual pixels to force an acceptance.
 wire colour_yellow=r>=160&&g>=160&&{1'b0,r}>{1'b0,b}+9'd18&&{1'b0,g}>{1'b0,b}+9'd18&&
  {1'b0,r}+9'd16>={1'b0,g}&&{1'b0,g}+9'd16>={1'b0,r}&&
  ({3'd0,min_rg}<<2)>(({3'd0,b}<<2)+{3'd0,b});
 wire colour_green={1'b0,g}>{1'b0,r}+9'd14&&{1'b0,g}>{1'b0,b}+9'd12&&
  {(g-max_rb),3'b000}>={3'd0,g};
 // Pale blue: retain absolute B-R separation and channel ordering;
 // use1.20 rather than1.125, then demand7/9 blue support to reject background speckles.
 wire colour_blue={1'b0,b}>{1'b0,r}+9'd20&&{1'b0,b}+9'd6>={1'b0,g}&&
  (({3'd0,b}<<2)+{3'd0,b})>(({3'd0,r}<<2)+({3'd0,r}<<1))&&
  !colour_green&&!colour_yellow&&!colour_red;
 wire colour_selected=current_colour==0?colour_red:
  chroma_ok&&(current_colour==1?colour_yellow:current_colour==2?colour_green:colour_blue);
 wire in_roi=x>=ROI_X0&&x<=ROI_X1&&y>=ROI_Y0&&y<=ROI_Y1;
 // Three sampled rows/columns: remove unsupported red centres before
 // accumulation. No frame RAM or whole-image blur; all original frame checks
 // stay active. A 3x3 centre needs five red samples including itself.
 localparam GRID_COLUMNS=(WIDTH+(1<<SAMPLE_SHIFT)-1)>>SAMPLE_SHIFT;
 localparam[15:0]GRID_STEP=(1<<SAMPLE_SHIFT);
 reg[1:0]red_rows[0:GRID_COLUMNS-1];reg[1:0]red_row_q;
 reg filter_pending,filter_red;reg[7:0]filter_column;
 reg[15:0]filter_x,filter_y;reg[2:0]column1,column2;
 wire[2:0]column0={red_row_q,filter_red};
 wire[3:0]support={3'd0,column0[0]}+{3'd0,column0[1]}+{3'd0,column0[2]}+
  {3'd0,column1[0]}+{3'd0,column1[1]}+{3'd0,column1[2]}+
  {3'd0,column2[0]}+{3'd0,column2[1]}+{3'd0,column2[2]};
 wire filtered_red=column1[1]&&support>=((working_colour==0||working_colour==3)?7:5);
 wire[15:0]filtered_x=filter_x-GRID_STEP,filtered_y=filter_y-GRID_STEP;
 // No RAM reset. First two sample rows rebuild all history before it is used.
 always @(posedge clk)begin
  if(taking&&de&&x<WIDTH&&y<HEIGHT&&state==IDLE&&take)red_row_q<=red_rows[x>>SAMPLE_SHIFT];
  if(filter_pending)red_rows[filter_column]<={red_row_q[0],filter_red};
 end
 wire good=active&&!bad&&complete&&all_samples==N&&source_valid;
 reg component_start;
 wire component_done,component_found,component_failed,component_busy;
 wire[13:0]component_count;wire[24:0]component_sum_x;wire[23:0]component_sum_y;
 wire[15:0]component_x0,component_x1,component_y0,component_y1;
 wire component_abort=!source_valid||(origin&&state!=IDLE)||
  (state!=IDLE&&(source_frame!=held_frame||colour_select!=held_colour));
 wire write_component_mask=filter_pending&&state==IDLE&&filter_column>=2&&filter_y>=2*GRID_STEP;
 sparse_component_selector #(.COLUMNS(GRID_COLUMNS),.ROWS((HEIGHT+GRID_STEP-1)>>SAMPLE_SHIFT),
  .SHIFT(SAMPLE_SHIFT),.MIN_COUNT(MIN_RED),.MAX_WIDTH(MAX_BOX_WIDTH),.MAX_HEIGHT(MAX_BOX_HEIGHT)) components(
  .clk(clk),.rst_n(rst_n),.abort(component_abort),.start(component_start),.overload(red_count>1536),
  .write_mask(write_component_mask),.mask_value(filtered_red),.mask_x(filter_column-8'd1),.mask_y((filter_y>>SAMPLE_SHIFT)-7'd1),
  .busy(component_busy),.done(component_done),.found(component_found),.failed(component_failed),
  .count(component_count),.sum_x(component_sum_x),.sum_y(component_sum_y),
  .min_x(component_x0),.max_x(component_x1),.min_y(component_y0),.max_y(component_y1));
 wire[15:0]next_remainder={remainder,dividend[division_bit]};
 wire division_one=next_remainder>={2'd0,denominator};
 assign busy=state!=IDLE;
 always @(posedge clk)begin
  if(!rst_n)begin
   component_start<=0;filter_pending<=0;filter_red<=0;filter_column<=0;filter_x<=0;filter_y<=0;column1<=0;column2<=0;
   state<=IDLE;active<=0;seen<=0;bad<=0;complete<=0;last_id<=0;working_id<=0;held_frame<=0;
   expected_x<=0;expected_y<=0;all_samples<=0;red_count<=0;denominator<=1;
   x_total<=0;y_total<=0;held_y<=0;x0<=65535;x1<=0;y0<=65535;y1<=0;
   dividend<=0;quotient<=0;remainder<=0;division_bit<=0;
   working_colour<=0;held_colour<=0;result_colour<=0;
   result_commit<=0;result_valid<=0;found<=0;result_frame<=0;result_sequence<=0;
   center_x<=0;center_y<=0;min_x<=0;max_x<=0;min_y<=0;max_y<=0;red_samples<=0;reason<=0;
  end else begin
   result_commit<=0;filter_pending<=0;component_start<=0;
   if(origin)begin
    active<=fresh&&state==IDLE;working_id<=source_frame;working_colour<=colour_select;expected_x<=0;expected_y<=0;
    bad<=0;complete<=0;all_samples<=0;red_count<=0;x_total<=0;y_total<=0;
    x0<=65535;x1<=0;y0<=65535;y1<=0;
    if(source_valid)begin seen<=1;last_id<=source_frame;end
   end
   if(taking&&de&&x<WIDTH&&y<HEIGHT&&state==IDLE)begin
    if(!pixel_valid||(!origin&&(complete||x!=expected_x||y!=expected_y)))bad<=1;
    if(x==WIDTH-1)begin expected_x<=0;expected_y<=y+1'b1;if(y==HEIGHT-1)complete<=1;end
    else expected_x<=x+1'b1;
    if(take)begin
     all_samples<=(origin?15'd0:all_samples)+1'b1;
     filter_pending<=1;filter_red<=colour_selected&&in_roi&&pixel_valid;
     filter_column<=x>>SAMPLE_SHIFT;filter_x<=x;filter_y<=y;
    end
   end
   if(filter_pending&&state==IDLE)begin
    if(filter_column==0)begin column2<=0;column1<=column0;end
    else begin column2<=column1;column1<=column0;end
    if(write_component_mask&&filtered_red)red_count<=red_count+1'b1;
   end
   if(active&&!de&&y>=HEIGHT)begin
    active<=0;result_colour<=working_colour;
    if(good)begin
     held_frame<=working_id;held_colour<=working_colour;component_start<=1;state<=COMPONENT;
    end else begin
     result_commit<=1;result_valid<=0;found<=0;result_frame<=working_id;result_sequence<=result_sequence+1'b1;reason<=1;
    end
   end
   case(state)
    COMPONENT:if(component_done)begin
     red_samples<=component_count;min_x<=component_x0;max_x<=component_x1;min_y<=component_y0;max_y<=component_y1;
     if(component_found&&!component_failed)begin
      held_y<=component_sum_y;denominator<=component_count;dividend<=component_sum_x;
      quotient<=0;remainder<=0;division_bit<=24;state<=DIVX;
     end else begin
      result_commit<=1;result_valid<=1;found<=0;result_frame<=held_frame;result_colour<=held_colour;
      result_sequence<=result_sequence+1'b1;reason<=red_count<MIN_RED&&!component_failed?4'd2:4'd3;state<=IDLE;
     end
    end
    DIVX,DIVY:begin
     remainder<=division_one?next_remainder-denominator:next_remainder;
     quotient[division_bit]<=division_one;
     if(division_bit==0)begin
      if(state==DIVX)begin center_x<={quotient[15:1],division_one};dividend<={1'b0,held_y};quotient<=0;remainder<=0;division_bit<=24;state<=DIVY;end
      else begin center_y<={quotient[15:1],division_one};state<=PUBLISH;end
     end else division_bit<=division_bit-1'b1;
    end
    PUBLISH:begin
     result_commit<=1;result_valid<=1;found<=1;result_colour<=held_colour;reason<=0;result_frame<=held_frame;result_sequence<=result_sequence+1'b1;state<=IDLE;
    end
    default:;
   endcase
   if((active&&!origin&&(source_frame!=working_id||colour_select!=working_colour))||
      (state!=IDLE&&!origin&&(source_frame!=held_frame||colour_select!=held_colour))||!source_valid)begin
    active<=0;bad<=1;state<=IDLE;found<=0;result_valid<=0;reason<=4;
   end
   if(origin&&state!=IDLE)begin active<=0;state<=IDLE;found<=0;result_valid<=0;reason<=5;end
  end
 end
endmodule
