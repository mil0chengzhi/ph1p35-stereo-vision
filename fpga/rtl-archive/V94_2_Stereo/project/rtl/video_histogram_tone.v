`timescale 1ns/1ps
// Contrast-limited GLOBAL histogram tone adjustment of HSV value (max RGB),
// not CLAHE. Shared RGB gain preserves channel ratios before final quantization.
// 64 bins, sparse 16x16 samples, completed NEW source frames only. Histogram
// clipping redistributes excess evenly. Blend 1/4 CDF +3/4 identity, bounded
// gain192..320, maximum mapped-V change1/accepted frame. No frame buffer or DDR.
// A separate gain for every input V and nearest-rounded division reproduce a
// monotone integer target; 64-bin constant gains can introduce false contours.
// Exactly four sampled stages, matching video_tone_map. Table swaps in blanking.
// ERAM variant: one synchronous lookup read shared with the blanking engine,
// and one synchronous histogram read followed by its read/modify/write.
module video_histogram_tone #(
 parameter WIDTH=1280,HEIGHT=720,SAMPLE_SHIFT=4,GAIN_STEP=4
)(
 input wire clk,rst_n,enable,
 input wire [23:0] rgb_in,input wire valid_in,
 input wire [15:0] x,y,input wire de,pixel_valid,
 input wire [31:0] source_frame,input wire source_valid,
 output reg [23:0] rgb_out,output reg valid_out,
 output reg [15:0] updates,output reg confidence,
 output reg [11:0] measured_samples,
 output reg [7:0] first_percentile,last_percentile,
 output wire busy
);
 localparam N=((WIDTH+(1<<SAMPLE_SHIFT)-1)>>SAMPLE_SHIFT)*((HEIGHT+(1<<SAMPLE_SHIFT)-1)>>SAMPLE_SHIFT);
 localparam CLIP=N/16;
 localparam IDLE=0,CLIPPING=1,LOOKUP=2,PREPARE=3,DIVIDE=4,BOUND=5,STORE=6,CLEAR=7,INITIALIZE=8,CLIP_READ=9,NORMALIZE=10,EQUALIZE=11,MIX=12;
 reg [3:0] state;
 reg [8:0] reset_index;
 (* ram_style="bram" *) reg [11:0] histogram[0:63];
 reg [11:0] histogram_read;
 reg sample_pending;reg [5:0] pending_bin;
 (* ram_style="bram" *) reg [8:0] gains[0:511];
 reg bank;
 reg [7:0] index;
 reg [12:0] excess,cdf;
 reg [8:0] old_gain,candidate;
 reg [7:0] desired_target;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg [12:0] cdf_total_reg;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg [38:0] cdf_product_reg;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg [7:0] equalized_reg;
 reg [19:0] numerator,quotient;
 reg [9:0] remainder,denominator;
 reg [4:0] division_bit;
 reg commit_table;
 reg active,seen,bad,complete;
 reg [31:0] last_id,working_id;
 reg [15:0] expected_x,expected_y;
 reg [11:0] count;
 reg [5:0] low_bin,high_bin;
 reg [7:0] r1,g1,b1,max1,r2,g2,b2;
 reg [8:0] gain2;
 reg [16:0] rp3,gp3,bp3;
 reg v1,v2,v3,en1,en2,en3;
 wire [7:0] rgmax=rgb_in[23:16]>rgb_in[15:8]?rgb_in[23:16]:rgb_in[15:8];
 wire [7:0] maximum=rgmax>rgb_in[7:0]?rgmax:rgb_in[7:0];
 wire origin=de&&x==0&&y==0;
 wire fresh=source_valid&&(!seen||last_id!=source_frame);
 wire collecting=(origin?fresh&&state==IDLE:active)&&source_valid;
 wire take=x<WIDTH&&y<HEIGHT&&x[SAMPLE_SHIFT-1:0]==0&&y[SAMPLE_SHIFT-1:0]==0;
 wire [11:0] clipped=histogram_read>CLIP ? CLIP:histogram_read;
 wire [12:0] cumulative=cdf+histogram_read+(excess>>6)+(index[7:2]<excess[5:0]);
 // Fixed reciprocal replaces a large combinational divide. At production
 // N=3600 the pre-rounding CDF error is <0.007 levels; rounded error <=1.
 // Both mappings remain monotone. The identity blend reduces its influence.
 localparam [25:0] CDF_GAIN=(64'd255*262144+N/2)/N;
 wire [38:0] cdf_rounded=(cdf_product_reg+39'd131072)>>18;
 wire [7:0] equalized=equalized_reg;
 wire [7:0] centre=index;
 wire [9:0] mixed={2'b0,centre}+{1'b0,centre,1'b0}+{2'b0,equalized}+10'd2;
 wire [7:0] raw_target=mixed[9:2];
 wire [15:0] lower_product=centre*192;
 wire [16:0] upper_target_product=centre*320;
 wire [8:0] lower_value=({1'b0,lower_product}+17'd255)>>8;
 wire [8:0] upper_value=upper_target_product>>8;
 wire [8:0] bounded_target=raw_target<lower_value ? lower_value:
                         raw_target>upper_value ? upper_value:{1'b0,raw_target};
 wire [7:0] target_value=index==0 ? 8'd0:bounded_target>254 ? 8'd254:bounded_target[7:0];
 wire [16:0] previous_product=old_gain*index;
 wire [17:0] previous_rounded={1'b0,previous_product}+18'd128;
 wire [8:0] previous_value=previous_rounded>>8;
 wire [7:0] smooth_target=previous_value+1<desired_target ? previous_value+1:
                           desired_target+1<previous_value ? previous_value-1:desired_target;
 wire [10:0] next_remainder={remainder,numerator[division_bit]};
 assign busy=state!=IDLE;
 wire take_histogram=collecting&&de&&x<WIDTH&&y<HEIGHT&&take;
 wire [5:0] histogram_address=take_histogram ? maximum[7:2]:
       (state==LOOKUP||state==PREPARE||state==DIVIDE||state==BOUND||state==STORE) ? index[7:2]:index[5:0];
 wire [8:0] gain_address=state==LOOKUP ? {bank,index}:{bank,max1};
 integer k;
 initial begin
  for(k=0;k<512;k=k+1)gains[k]=9'd256;
 end
 function [7:0] scaled_channel(input [16:0] product);
  reg [17:0] rounded;
  begin rounded={1'b0,product}+18'd128;scaled_channel=rounded>=18'd65280 ? 8'd255:rounded[15:8];end
 endfunction
 always @(posedge clk)begin
  if(!rst_n)begin
   r1<=0;g1<=0;b1<=0;max1<=0;r2<=0;g2<=0;b2<=0;
   rp3<=0;gp3<=0;bp3<=0;v1<=0;v2<=0;v3<=0;en1<=0;en2<=0;en3<=0;rgb_out<=0;valid_out<=0;
  end else begin
   r1<=rgb_in[23:16];g1<=rgb_in[15:8];b1<=rgb_in[7:0];max1<=maximum;v1<=valid_in;en1<=enable;
   r2<=r1;g2<=g1;b2<=b1;v2<=v1;en2<=en1;
   rp3<=r2*(en2?gain2:9'd256);gp3<=g2*(en2?gain2:9'd256);bp3<=b2*(en2?gain2:9'd256);v3<=v2;en3<=en2;
   valid_out<=v3;rgb_out<=v3?{scaled_channel(rp3),scaled_channel(gp3),scaled_channel(bp3)}:24'd0;
  end
 end
 // Reset-free synchronous reads allow the RAM output register to be absorbed.
 // The validity pipeline and blanking initialization hide unspecified outputs.
 always @(posedge clk)gain2<=gains[gain_address];
 always @(posedge clk)histogram_read<=histogram[histogram_address];
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=INITIALIZE;reset_index<=0;index<=0;bank<=0;excess<=0;cdf<=0;old_gain<=256;candidate<=256;desired_target<=0;
   numerator<=0;quotient<=0;remainder<=0;denominator<=1;division_bit<=0;commit_table<=0;
   cdf_total_reg<=0;cdf_product_reg<=0;equalized_reg<=0;
   updates<=0;confidence<=0;measured_samples<=0;first_percentile<=0;last_percentile<=255;
   active<=0;seen<=0;bad<=0;complete<=0;last_id<=0;working_id<=0;expected_x<=0;expected_y<=0;
   count<=0;low_bin<=0;high_bin<=63;sample_pending<=0;pending_bin<=0;
  end else begin
   sample_pending<=take_histogram;pending_bin<=maximum[7:2];
   if(sample_pending)histogram[pending_bin]<=histogram_read+1'b1;
   if(origin)begin
    active<=fresh&&state==IDLE;bad<=0;complete<=0;working_id<=source_frame;count<=0;
    expected_x<=0;expected_y<=0;
    if(source_valid)begin seen<=1;last_id<=source_frame;end
    if(active)confidence<=0;
   end
   if(collecting&&de&&x<WIDTH&&y<HEIGHT)begin
    if(!pixel_valid||(!origin&&(complete||x!=expected_x||y!=expected_y)))bad<=1;
    if(x==WIDTH-1)begin expected_x<=0;expected_y<=y+1'b1;if(y==HEIGHT-1)complete<=1;end
    else expected_x<=x+1'b1;
    if(take)count<=(origin?12'd0:count)+1'b1;
   end
   if(active&&(!source_valid||(!origin&&source_frame!=working_id)))begin
    active<=0;bad<=1;confidence<=0;state<=CLEAR;index<=0;
   end
   if(active&&!de&&y>=HEIGHT)begin
    active<=0;measured_samples<=count;confidence<=!bad&&complete&&count==N;
    index<=0;excess<=0;cdf<=0;commit_table<=!bad&&complete&&count==N&&enable;
    state<=!bad&&complete&&count==N&&enable ? CLIP_READ:CLEAR;
   end
   case(state)
    CLIP_READ:state<=CLIPPING;
    CLIPPING:begin
     excess<=excess+histogram_read-clipped;histogram[index]<=clipped;
     cdf<=cdf+histogram_read;
     if(cdf<N/100&&cdf+histogram_read>=N/100)first_percentile<={index[5:0],2'b00};
     if(cdf<(N*99)/100&&cdf+histogram_read>=(N*99)/100)last_percentile<={index[5:0],2'b11};
     if(index==63)begin index<=0;cdf<=0;state<=LOOKUP;end
     else begin index<=index+1'b1;state<=CLIP_READ;end
    end
    LOOKUP:state<=PREPARE;
    PREPARE:begin
     old_gain<=gain2;
     cdf_total_reg<=cumulative;
     // Wait one more state for old_gain, then normalize the temporally smoothed
     // target. Histogram cumulative advances only once per group of four values.
     if(index[1:0]==3)cdf<=cumulative;
     state<=NORMALIZE;
    end
    NORMALIZE:begin cdf_product_reg<=cdf_total_reg*CDF_GAIN;state<=EQUALIZE;end
    EQUALIZE:begin equalized_reg<=cdf_rounded>255 ? 8'd255:cdf_rounded[7:0];state<=MIX;end
    MIX:begin desired_target<=target_value;state<=BOUND;end
    DIVIDE:begin
     if(next_remainder>=denominator)begin remainder<=next_remainder-denominator;quotient[division_bit]<=1;end
     else begin remainder<=next_remainder;quotient[division_bit]<=0;end
     if(division_bit==0)state<=STORE;else division_bit<=division_bit-1'b1;
    end
    BOUND:begin
     numerator<=({12'd0,smooth_target}<<8)+(index>>1);quotient<=0;remainder<=0;
     denominator<=index==0 ? 10'd1:{2'd0,index};division_bit<=19;state<=DIVIDE;
    end
    STORE:begin
      gains[{!bank,index}]<=index==0 ? 9'd256:quotient[8:0];
      if(index==255)begin
       bank<=!bank;updates<=updates+1'b1;index<=0;state<=CLEAR;commit_table<=0;
      end else begin index<=index+1'b1;state<=LOOKUP;end
    end
    CLEAR:begin
     histogram[index]<=0;
     if(index==63)begin index<=0;state<=IDLE;end else index<=index+1'b1;
    end
    INITIALIZE:begin
     gains[reset_index]<=256;
     if(reset_index==511)begin index<=0;state<=CLEAR;end else reset_index<=reset_index+1'b1;
    end
    default:;
   endcase
   // A shortened/malformed blanking interval cannot expose a half-written LUT.
   // The currently selected table remains intact; abandon only the inactive one.
   if(origin&&(state!=IDLE||active))begin
    // Initialization itself must finish even if the first video origin arrives early.
    if(state!=INITIALIZE)begin state<=CLEAR;index<=0;end
    commit_table<=0;active<=0;confidence<=0;
   end
  end
 end
endmodule
