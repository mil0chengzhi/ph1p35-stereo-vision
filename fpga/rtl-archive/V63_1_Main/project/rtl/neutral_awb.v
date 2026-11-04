`timescale 1ns/1ps
// Same-clock feedback AWB. Inputs are the linear, post-gain values immediately
// before the existing gamma/565 conversion. No image RAM, divider or I2C owner.
// One sample per 16x16 output pixels in the 1280x720 centre crop. Frames with
// missing groups, input faults, too few neutral candidates, poor spatial or
// luminance coverage cannot change gains. This is a scene-statistics estimate,
// not an absolute colour calibration; ambiguous illuminant/object colour remains.
module neutral_awb #(
 parameter GROUPS_PER_ROW=640,ROWS=960,CROP_TOP=120,CROP_HEIGHT=720,
 parameter SAMPLE_GROUP_SHIFT=3,SAMPLE_ROW_SHIFT=4,
 parameter INITIAL_R=335,INITIAL_B=346,MIN_GAIN=160,MAX_GAIN=512,
 parameter MIN_NEUTRAL=96,UPDATE_FRAMES=4
)(
 input wire clk,rst_n,frame_start,input_fault,
 input wire enable,allow_update,sample_valid,
 input wire [7:0] linear_r,linear_g,linear_b,
 output reg [10:0] gain_r,gain_b,
 output reg result_valid,confidence,at_limit,
 output reg [15:0] result_sequence,updates,
 output reg [12:0] neutral_count,
 output reg [19:0] sum_r,sum_g,sum_b,
 output reg [7:0] neutral_min,neutral_max
);
 localparam EXPECTED_SAMPLES=((GROUPS_PER_ROW+(1<<SAMPLE_GROUP_SHIFT)-1)>>SAMPLE_GROUP_SHIFT)*
                            ((CROP_HEIGHT+(1<<SAMPLE_ROW_SHIFT)-1)>>SAMPLE_ROW_SHIFT);
 reg [15:0] group_x,row_y;
 reg seen,complete,bad;
 reg [12:0] count,all_count;
 reg [19:0] red_total,green_total,blue_total;
 reg [7:0] lowest,highest;
 reg [3:0] quadrants;
 reg [7:0] eligible_frames;
 reg pending_sample,pending_neutral;
 reg [7:0] pending_r,pending_g,pending_b,pending_lo,pending_hi;
 reg [1:0] pending_quadrant;
 wire [15:0] crop_y=row_y-CROP_TOP;
 wire in_crop=row_y>=CROP_TOP&&row_y<CROP_TOP+CROP_HEIGHT;
 wire take=in_crop&&group_x[SAMPLE_GROUP_SHIFT-1:0]==0&&crop_y[SAMPLE_ROW_SHIFT-1:0]==0;
 wire [7:0] rg_hi=linear_r>linear_g?linear_r:linear_g;
 wire [7:0] hi=rg_hi>linear_b?rg_hi:linear_b;
 wire [7:0] rg_lo=linear_r<linear_g?linear_r:linear_g;
 wire [7:0] lo=rg_lo<linear_b?rg_lo:linear_b;
 // Saturation <=3/8; exclude near-black and near-clipping samples.
 wire neutral=lo>=12&&hi<=224&&({3'b0,hi-lo}<<3)<=(({3'b0,hi}<<1)+{3'b0,hi});
 wire good=seen&&complete&&!bad&&all_count==EXPECTED_SAMPLES;
 wire trustworthy=good&&count>=MIN_NEUTRAL&&quadrants==4'b1111&&highest-lowest>=24;
 // Exact integer equivalents of 32*R <31*G and 32*R >33*G.
 // The strict comparisons account for fractional thresholds without a DSP.
 wire [19:0] low_threshold=green_total-(green_total>>5);
 wire [20:0] high_threshold={1'b0,green_total}+{1'b0,green_total>>5};
 wire red_low=red_total<low_threshold;
 wire red_high={1'b0,red_total}>high_threshold;
 wire blue_low=blue_total<low_threshold;
 wire blue_high={1'b0,blue_total}>high_threshold;
 always @(posedge clk)begin
  if(!rst_n)begin
   gain_r<=INITIAL_R;gain_b<=INITIAL_B;result_valid<=0;confidence<=0;at_limit<=0;
   result_sequence<=0;updates<=0;neutral_count<=0;sum_r<=0;sum_g<=0;sum_b<=0;
   neutral_min<=0;neutral_max<=0;group_x<=0;row_y<=0;seen<=0;complete<=0;bad<=0;
   count<=0;all_count<=0;red_total<=0;green_total<=0;blue_total<=0;
   lowest<=255;highest<=0;quadrants<=0;eligible_frames<=0;
   pending_sample<=0;pending_neutral<=0;pending_r<=0;pending_g<=0;pending_b<=0;pending_lo<=0;pending_hi<=0;pending_quadrant<=0;
  end else if(frame_start)begin
   result_valid<=good;confidence<=trustworthy;at_limit<=0;
   if(good)begin
    result_sequence<=result_sequence+1'b1;neutral_count<=count;
    sum_r<=red_total;sum_g<=green_total;sum_b<=blue_total;neutral_min<=lowest;neutral_max<=highest;
   end
   if(!enable||!allow_update||!trustworthy)eligible_frames<=0;
   else if(eligible_frames>=UPDATE_FRAMES-1)begin
    eligible_frames<=0;
    if(red_low&&gain_r<MAX_GAIN)gain_r<=gain_r+1'b1;
    else if(red_high&&gain_r>MIN_GAIN)gain_r<=gain_r-1'b1;
    if(blue_low&&gain_b<MAX_GAIN)gain_b<=gain_b+1'b1;
    else if(blue_high&&gain_b>MIN_GAIN)gain_b<=gain_b-1'b1;
    if((red_low&&gain_r<MAX_GAIN)||(red_high&&gain_r>MIN_GAIN)||
       (blue_low&&gain_b<MAX_GAIN)||(blue_high&&gain_b>MIN_GAIN))updates<=updates+1'b1;
    at_limit<=(red_low&&gain_r>=MAX_GAIN)||(red_high&&gain_r<=MIN_GAIN)||
              (blue_low&&gain_b>=MAX_GAIN)||(blue_high&&gain_b<=MIN_GAIN);
   end else eligible_frames<=eligible_frames+1'b1;
   group_x<=0;row_y<=0;seen<=1;complete<=0;bad<=input_fault;
   count<=0;all_count<=0;red_total<=0;green_total<=0;blue_total<=0;
   lowest<=255;highest<=0;quadrants<=0;pending_sample<=0;
  end else begin
   pending_sample<=0;
   if(pending_sample)begin
    all_count<=all_count+1'b1;
    if(pending_neutral)begin
     count<=count+1'b1;
     red_total<=red_total+pending_r;green_total<=green_total+pending_g;blue_total<=blue_total+pending_b;
     if(pending_lo<lowest)lowest<=pending_lo;if(pending_hi>highest)highest<=pending_hi;
     quadrants[pending_quadrant]<=1'b1;
    end
   end
   if(input_fault)bad<=1;
   if(sample_valid&&seen)begin
    if(complete)bad<=1;
    if(group_x==GROUPS_PER_ROW-1)begin
     group_x<=0;
     if(row_y==ROWS-1)complete<=1;
     else row_y<=row_y+1'b1;
    end else group_x<=group_x+1'b1;
    if(take&&!complete)begin
     // Separate the neutral classifier from accumulator clock enables so the
     // complete decision path does not run from Bayer output to a DSP CE.
     pending_sample<=1;pending_neutral<=neutral;
     pending_r<=linear_r;pending_g<=linear_g;pending_b<=linear_b;pending_lo<=lo;pending_hi<=hi;
     pending_quadrant<={row_y>=CROP_TOP+CROP_HEIGHT/2,group_x>=GROUPS_PER_ROW/2};
    end
   end
  end
 end
endmodule
