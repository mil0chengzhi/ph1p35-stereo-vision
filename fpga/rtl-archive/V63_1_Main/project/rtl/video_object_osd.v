`timescale 1ns/1ps
// V33: up to eight completed-frame detections, each with a box and Chinese label.
// All inputs are in clk domain. Detector buses MUST remain stable between commits.
// result_valid is a one-cycle commit, not a persistent validity indication.
// snapshot is a one-cycle pulse in vertical blank, >= 384 clocks before active video.
// Coordinate boxes are inclusive, in HDMI screen coordinates (not camera-local).
// class: 0 unknown, 1 triangle, 2 circle, 3 quadrilateral, 4 colour block.
// colour: 0 none, 1 red, 2 green, 3 blue, 4 yellow.
// Object numbers are snapshot slot numbers, not persistent tracking identities.
// x_out/y_out/de_out/rgb have exactly ONE clock latency, including font lookup.
// Snapshot results are complete analysis frames, accepted at source age 0..2.
// Source invalid/stale, age > 2, or detector overflow withdraws every old box.
module video_object_osd #(
 parameter SCREEN_WIDTH=1280, SCREEN_HEIGHT=720, ROI_LEFT=160, ROI_RIGHT=1119, RESERVE_CONTEST_HEADER=0
)(
 input wire clk,rst_n,
 input wire [15:0] x,y,
 input wire de,
 input wire [23:0] base_rgb,
 input wire enable,signal_valid,stale,snapshot,
 input wire [31:0] source_frame_id,
 input wire result_valid,
 input wire [31:0] result_frame_id,
 input wire overflow,
 input wire [7:0] valid,
 input wire [127:0] bbox_x0,bbox_x1,bbox_y0,bbox_y1,
 input wire [31:0] class_id,
 input wire [23:0] color_id,
 output reg [15:0] x_out,y_out,
 output reg de_out,
 output wire [23:0] rgb,
 output wire [7:0] shown_valid,
 output wire shown_result_valid,
 output reg [31:0] shown_frame_id,
 output reg shown_overflow,
 output reg label_crowded
);
 localparam [10:0] LABEL_W=11'd68, ROI_L=ROI_LEFT, ROI_R=ROI_RIGHT;
 localparam [10:0] LABEL_MAX_X=ROI_RIGHT+1-68;
 localparam [9:0] LABEL_H=10'd20, IMAGE_HEIGHT=SCREEN_HEIGHT;
 localparam [9:0] LABEL_MAX_Y=SCREEN_HEIGHT-20;
 localparam [23:0] PANEL=24'h091522, INK=24'hf4f8fc;
 reg have_result,ready,accepted_result;
 reg [7:0] active_valid,label_valid;
 reg [10:0] left[0:7],right[0:7];
 reg [9:0] top[0:7],bottom[0:7];
 reg [3:0] shape[0:7];
 reg [2:0] colour[0:7];
 reg [10:0] label_x[0:7];
 reg [9:0] label_y[0:7];
 wire [31:0] incoming_age=source_frame_id-result_frame_id;
 wire [31:0] shown_age=source_frame_id-shown_frame_id;
 wire visible=enable&&signal_valid&&!stale&&ready&&accepted_result&&!shown_overflow&&shown_age<=2;
 assign shown_valid=visible?active_valid:8'd0;
 // Persistent validity is distinct from |shown_valid: an accepted empty frame
 // has shown_result_valid=1 and shown_valid=0; startup has both zero.
 assign shown_result_valid=visible;

 // Bounded blanking-time packing: above, below, right, left, then fallback.
 // Check all active object rectangles as well as previously placed labels.
 // Ordinary placements never hide another target. Crowded fallback is explicit.
 // At most48 cycles/object including loading:384 clocks before active video.
 reg [1:0] place_state;reg [2:0] attempt;
 reg [2:0] object_index,compare_index;
 reg [10:0] candidate_x;
 reg [9:0] candidate_y;
 // Select one object during blanking. Validate narrowed coordinates here, rather
 // than building eight copies of the full 16-bit range checking arithmetic.
 wire [10:0] place_left=left[object_index],place_right=right[object_index];
 wire [9:0] place_top=top[object_index],place_bottom=bottom[object_index];
 wire place_good=place_left>=ROI_L&&place_right<=ROI_R&&place_left<=place_right&&
  place_top<=place_bottom&&place_bottom<IMAGE_HEIGHT;
 wire [10:0] previous_label_right=label_x[compare_index]+LABEL_W;
 wire [10:0] candidate_right=candidate_x+LABEL_W;
 wire [9:0] previous_label_bottom=label_y[compare_index]+LABEL_H;
 wire [9:0] candidate_bottom=candidate_y+LABEL_H;
 // The contest header and official logo have priority over labels.
 wire candidate_header_overlap=RESERVE_CONTEST_HEADER&&candidate_y<10'd72&&candidate_bottom>10'd16&&
  ((candidate_x<11'd310&&candidate_right>11'd20)||(candidate_x<11'd992&&candidate_right>11'd352));
 wire candidate_label_overlap=label_valid[compare_index]&&
  candidate_x<previous_label_right&&label_x[compare_index]<candidate_right&&
  candidate_y<previous_label_bottom&&label_y[compare_index]<candidate_bottom;
 wire candidate_box_overlap=active_valid[compare_index]&&
  candidate_x<=right[compare_index]&&left[compare_index]<candidate_right&&
  candidate_y<=bottom[compare_index]&&top[compare_index]<candidate_bottom;
 wire candidate_overlap=candidate_label_overlap||candidate_box_overlap;
 function [9:0] below_y;
  input [9:0] by;
  begin below_y=(by<=LABEL_MAX_Y-10'd2)?by+10'd2:LABEL_MAX_Y;end
 endfunction
 integer j;
 always @(posedge clk) begin
  if(!rst_n) begin
   have_result<=0;ready<=0;accepted_result<=0;active_valid<=0;label_valid<=0;
   shown_frame_id<=0;shown_overflow<=0;label_crowded<=0;
   place_state<=0;attempt<=0;object_index<=0;compare_index<=0;
   candidate_x<=0;candidate_y<=0;
   for(j=0;j<8;j=j+1) begin
    left[j]<=0;right[j]<=0;top[j]<=0;bottom[j]<=0;
    shape[j]<=0;colour[j]<=0;label_x[j]<=0;label_y[j]<=0;
   end
  end else begin
   if(result_valid) have_result<=1;
   if(!signal_valid||stale) begin
    have_result<=0;ready<=0;accepted_result<=0;active_valid<=0;label_valid<=0;place_state<=0;
   end else if(snapshot) begin
    shown_frame_id<=result_frame_id;shown_overflow<=overflow;
    accepted_result<=(have_result||result_valid)&&incoming_age<=2;
    ready<=0;label_valid<=0;label_crowded<=0;
    object_index<=0;place_state<=1;
    for(j=0;j<8;j=j+1) begin
     active_valid[j]<=(have_result||result_valid)&&signal_valid&&!stale&&
      incoming_age<=2&&!overflow&&valid[j]&&
      !(|bbox_x0[j*16+11+:5])&&!(|bbox_x1[j*16+11+:5])&&
      !(|bbox_y0[j*16+10+:6])&&!(|bbox_y1[j*16+10+:6]);
     left[j]<=bbox_x0[j*16+:11];right[j]<=bbox_x1[j*16+:11];
     top[j]<=bbox_y0[j*16+:10];bottom[j]<=bbox_y1[j*16+:10];
     shape[j]<=class_id[j*4+:4];colour[j]<=color_id[j*3+:3];
    end
   end else case(place_state)
    1:begin
     if(active_valid[object_index]&&place_good) begin
      candidate_x<=place_left>LABEL_MAX_X?LABEL_MAX_X:place_left;
      candidate_y<=place_top>=10'd22 ?
       place_top-10'd22:below_y(place_bottom);
      attempt<=0;compare_index<=0;place_state<=2;
     end else begin
      active_valid[object_index]<=0;
      if(object_index==7) begin ready<=1;place_state<=0;end
      else object_index<=object_index+1'b1;
     end
    end
    2:begin
     if((candidate_header_overlap||candidate_overlap)&&attempt<4)begin
      attempt<=attempt+1'b1;compare_index<=0;
      if(attempt==0) candidate_y<=below_y(place_bottom);
      else if(attempt==1)begin
       candidate_x<=(place_right+11'd2<=LABEL_MAX_X) ? place_right+11'd2 : LABEL_MAX_X;
       candidate_y<=place_top<=LABEL_MAX_Y?place_top:LABEL_MAX_Y;
      end else if(attempt==2)begin
       candidate_x<=(place_left>=ROI_L+11'd70) ? place_left-11'd70 : ROI_L;
       candidate_y<=place_top<=LABEL_MAX_Y?place_top:LABEL_MAX_Y;
      end else begin
       candidate_x<=place_left>LABEL_MAX_X?LABEL_MAX_X:place_left;
       candidate_y<=place_top<=LABEL_MAX_Y?place_top:LABEL_MAX_Y;
      end
     end else if(candidate_header_overlap)begin
      candidate_y<=10'd74;compare_index<=0;label_crowded<=1;
     end else if(compare_index==7)begin
      if(candidate_overlap)label_crowded<=1;
      label_x[object_index]<=candidate_x;label_y[object_index]<=candidate_y;
      label_valid[object_index]<=1;
      if(object_index==7)begin ready<=1;place_state<=0;end
      else begin object_index<=object_index+1'b1;place_state<=1;end
     end else begin
      if(candidate_overlap)label_crowded<=1;
      compare_index<=compare_index+1'b1;
     end
    end
    default:place_state<=0;
   endcase
  end
 end

 function [23:0] object_colour;
  input [2:0] c;
  input [3:0] s;
  begin
   case(c)
    1:object_colour=24'hff6060;
    2:object_colour=24'h65f29b;
    3:object_colour=24'h61a9ff;
    4:object_colour=24'hffe366;
    default:object_colour=s==0?24'hb7c4d1:24'h37dbe0;
   endcase
  end
 endfunction
 // Compact font IDs: 0 space, 1..8 digits; 9..22 Chinese (see ROM).
 function [4:0] label_glyph;
  input [2:0] number;
  input [1:0] column;
  input [3:0] s;
  input [2:0] c;
  begin
   label_glyph=0;
   if(column==0) label_glyph={2'b00,number}+1'b1;
   else if(s==4&&c>=1&&c<=4) begin
    case(column) 1:label_glyph=8+c;2:label_glyph=13;3:label_glyph=14;endcase
   end else case(s)
    1:case(column) 1:label_glyph=15;2:label_glyph=16;3:label_glyph=18;endcase
    2:case(column) 1:label_glyph=17;2:label_glyph=18;default:label_glyph=0;endcase
    3:case(column) 1:label_glyph=19;2:label_glyph=20;3:label_glyph=18;endcase
    default:case(column) 1:label_glyph=21;2:label_glyph=22;default:label_glyph=0;endcase
   endcase
  end
 endfunction
 wire box_hit,label_hit;
 wire [7:0] box_hits,label_hits;
 wire [10:0] pixel_x=x[10:0],pixel_x_before=x[10:0]-11'd1,pixel_x_after=x[10:0]+11'd1;
 wire [9:0] pixel_y=y[9:0],pixel_y_before=y[9:0]-10'd1,pixel_y_after=y[9:0]+10'd1;
 // In the image ROI x is always >= 160, so x-68 cannot underflow.
 // Shift the pixel once instead of adding label dimensions in all eight slots.
 wire [10:0] label_left_limit=pixel_x-LABEL_W;
 wire [9:0] label_top_limit=pixel_y-LABEL_H;
 wire pixel_visible=de&&visible&&x>=ROI_LEFT&&x<=ROI_RIGHT&&y<SCREEN_HEIGHT;
 genvar g;
 generate for(g=0;g<8;g=g+1) begin:pixel_slots
  wire inside_box=pixel_x>=left[g]&&pixel_x<=right[g]&&
   pixel_y>=top[g]&&pixel_y<=bottom[g];
  assign box_hits[g]=pixel_visible&&active_valid[g]&&inside_box&&
   (pixel_x==left[g]||pixel_x_before==left[g]||
    pixel_x==right[g]||pixel_x_after==right[g]||
    pixel_y==top[g]||pixel_y_before==top[g]||
    pixel_y==bottom[g]||pixel_y_after==bottom[g]);
  assign label_hits[g]=pixel_visible&&label_valid[g]&&
   pixel_x>=label_x[g]&&((ROI_LEFT<68&&pixel_x<LABEL_W)||label_x[g]>label_left_limit)&&
   pixel_y>=label_y[g]&&(pixel_y<LABEL_H||label_y[g]>label_top_limit);
 end endgenerate
 assign box_hit=|box_hits;
 assign label_hit=|label_hits;
 reg [2:0] box_slot,label_slot;
 wire [2:0] ink_slot=label_hit?label_slot:box_slot;
 wire [23:0] selected_ink=object_colour(colour[ink_slot],shape[ink_slot]);
 reg text_hit,label_border;
 reg [4:0] glyph;
 reg [3:0] glyph_x,glyph_y;
 reg [10:0] relative_x;
 reg [9:0] relative_y;
 integer k;
 always @* begin
  // Reverse traversal gives the original lowest-slot overlap priority, without
  // duplicating the coordinate subtractors or glyph/colour decoders per slot.
  box_slot=0;label_slot=0;
  for(k=7;k>=0;k=k-1) begin
   if(box_hits[k]) box_slot=k[2:0];
   if(label_hits[k]) label_slot=k[2:0];
  end
  relative_x=pixel_x-label_x[label_slot];
  relative_y=pixel_y-label_y[label_slot];
  label_border=relative_y<10'd2;
  text_hit=label_hit&&relative_x>=11'd2&&relative_x<11'd66&&relative_y>=10'd2&&relative_y<10'd18;
  relative_x=relative_x-11'd2;relative_y=relative_y-10'd2;
  glyph=label_glyph(label_slot,relative_x[5:4],shape[label_slot],colour[label_slot]);
  glyph_x=relative_x[3:0];glyph_y=relative_y[3:0];
 end
 wire foreground;
 object_label_font font(.clk(clk),.rst_n(rst_n),.glyph(glyph),
  .x(glyph_x),.y(glyph_y),.foreground(foreground));
 reg [23:0] background_q;
 reg text_q;
 always @(posedge clk) begin
  if(!rst_n) begin
   x_out<=0;y_out<=0;de_out<=0;background_q<=0;text_q<=0;
  end else begin
   x_out<=x;y_out<=y;de_out<=de;
   text_q<=text_hit;
   if(!de) background_q<=0;
   else if(label_hit) background_q<=label_border?selected_ink:PANEL;
   else if(box_hit) background_q<=selected_ink;
   else background_q<=base_rgb;
  end
 end
 assign rgb=(text_q&&foreground)?INK:background_q;
endmodule
