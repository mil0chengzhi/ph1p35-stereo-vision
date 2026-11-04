`timescale 1ns/1ps
// Real, published Z16 -> orthographic XYZ -> yaw/pitch -> 240x135 framebuffer.
// Source intrinsics: cx=159.610, cy=87.811, f=467.648 at 320x180, rounded
// to (160,90,468) for this visual renderer only. Depth itself is unchanged.
// No interpolation, plane replacement, invalid-depth fill, or hidden surfaces.
// The immutable source job is provided only after the depth engine has drained.
// One shared signed multiplier, a 4-bit occlusion buffer and RGB333 display banks.
// Handshake publication changes the visible bank only in vertical blank.
module stereo_cloud_rgb_projector #(parameter ORBIT_CYCLES=12000000)(
 input wire memory_clk,pixel_clk,rst_n,
 input wire enable_memory,orbit_memory,enable_pixel,
 input wire[15:0]x,y,input wire de,
 output wire line_request,line_new,output wire[15:0]line_row,
 input wire line_ready,line_write,input wire[127:0]line_data,input wire[7:0]line_address,
 input wire line_done,line_good,input wire[31:0]line_job,
 output wire[15:0]pixel_data,output wire pixel_valid,
 output wire[31:0]pixel_job,output wire pixel_cloud_valid,
 output reg[31:0]rendered_frames,rejected_frames,output reg[15:0]visible_points,
 output wire buffer_write,output wire[15:0]buffer_address,buffer_colour,
 output wire buffer_clear,buffer_bank,input wire buffer_ready,
 output wire flush_request,input wire flush_done,input wire[15:0]scan_rgb,input wire scan_valid,
 output wire front_bank,output reg fault,output wire[5:0]view_index,output wire[5:0]state_debug
);
 reg bank_q;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]memory_reset,pixel_reset;
 always @(posedge memory_clk or negedge rst_n)if(!rst_n)memory_reset<=0;else memory_reset<={memory_reset[0],1'b1};
 always @(posedge pixel_clk or negedge rst_n)if(!rst_n)pixel_reset<=0;else pixel_reset<={pixel_reset[0],1'b1};
 localparam IDLE=0,CLEAR=1,REQUEST=2,WAIT_LINE=3,READ_POINT=4,DECODE=5,
  MUL_X=6,MUL_Y=7,MUL_XC=8,MUL_ZS=9,MUL_ZC=10,MUL_XS=11,
  MUL_YC=12,MUL_ZP=13,MUL_ZCP=14,MUL_YP=15,PROJECT=16,ZWAIT=17,
  ZTEST=18,NEXT=19,PUBLISH=20,ACK=21,PAUSE=22,FAILED=23,MEDIAN=24,MUL_SCREEN_X=25,MUL_SCREEN_Y=26,REWIND=27;
 reg[5:0]state;assign state_debug=state;
 reg[127:0]row_buffer[0:79];reg[127:0]row_word;
 reg[3:0]zbuffer[0:32399];
 // Two immutable colour banks. Only back_bank is written; ACK prevents reuse
 // until the pixel-domain publication has changed bank in vertical blank.
 wire[8:0]colour0_q,colour1_q;
 wire[8:0]colour333={buffer_colour[15:13],buffer_colour[10:8],buffer_colour[4:2]};
 wire[8:0]shown333=bank_q?colour1_q:colour0_q;
 wire[15:0]shown565={shown333[8:6],shown333[8:7],shown333[5:3],shown333[5:3],shown333[2:0],shown333[2:1]};
 reg[3:0]zread;
 reg[15:0]clear_address,draw_address,draw_colour,point_rgb;reg[4:0]draw_depth;
 reg back_bank;reg[8:0]u;reg[7:0]v;reg[7:0]beats;
 reg[31:0]source_job;reg[19:0]read_age;reg[$clog2(ORBIT_CYCLES+1)-1:0]orbit_age;reg[15:0]z,pivot_z,min_z,max_z;
 wire[31:0]source_point=row_word>>(u[1:0]*32);
 wire[15:0]point=source_point[31:16];
 // A few distant outliers must not choose the orbit centre. This histogram
 // selects the median occupied Z bin. It changes only presentation, never Z.
 reg[15:0]depth_histogram[0:15];reg[15:0]valid_point_count;
 reg[3:0]histogram_index;reg[16:0]histogram_running;
 wire[3:0]point_bin=point>=1460?4'd15:(point-16'd500)>>6;
 wire[16:0]histogram_next=histogram_running+{1'b0,depth_histogram[histogram_index]};
 reg[5:0]zoom;reg measure_pass;
 function[5:0]zoom_for_bin(input[3:0]bin);
 begin case(bin)
  0:zoom_for_bin=19;1:zoom_for_bin=17;2:zoom_for_bin=15;3:zoom_for_bin=14;
  4:zoom_for_bin=13;5:zoom_for_bin=12;6:zoom_for_bin=11;7:zoom_for_bin=10;
  8:zoom_for_bin=10;9:zoom_for_bin=9;10:zoom_for_bin=9;11:zoom_for_bin=8;
  12:zoom_for_bin=8;13:zoom_for_bin=7;14:zoom_for_bin=7;15:zoom_for_bin=7;
 endcase end endfunction
 reg[5:0]angle;
 assign view_index=angle;
 reg signed[9:0]cosine,sine;
 // Case table is inserted by the source generator, integer Q8 coefficients.
 always @* begin
 case(angle)
  0:begin cosine=10'sd256;sine=10'sd0;end // 0 degrees
  1:begin cosine=10'sd255;sine=10'sd18;end // 4 degrees
  2:begin cosine=10'sd254;sine=10'sd36;end // 8 degrees
  3:begin cosine=10'sd250;sine=10'sd53;end // 12 degrees
  4:begin cosine=10'sd246;sine=10'sd71;end // 16 degrees
  5:begin cosine=10'sd241;sine=10'sd88;end // 20 degrees
  6:begin cosine=10'sd234;sine=10'sd104;end // 24 degrees
  7:begin cosine=10'sd241;sine=10'sd88;end // 20 degrees
  8:begin cosine=10'sd246;sine=10'sd71;end // 16 degrees
  9:begin cosine=10'sd250;sine=10'sd53;end // 12 degrees
  10:begin cosine=10'sd254;sine=10'sd36;end // 8 degrees
  11:begin cosine=10'sd255;sine=10'sd18;end // 4 degrees
  12:begin cosine=10'sd256;sine=10'sd0;end // 0 degrees
  13:begin cosine=10'sd255;sine=-10'sd18;end // -4 degrees
  14:begin cosine=10'sd254;sine=-10'sd36;end // -8 degrees
  15:begin cosine=10'sd250;sine=-10'sd53;end // -12 degrees
  16:begin cosine=10'sd246;sine=-10'sd71;end // -16 degrees
  17:begin cosine=10'sd241;sine=-10'sd88;end // -20 degrees
  18:begin cosine=10'sd234;sine=-10'sd104;end // -24 degrees
  19:begin cosine=10'sd241;sine=-10'sd88;end // -20 degrees
  20:begin cosine=10'sd246;sine=-10'sd71;end // -16 degrees
  21:begin cosine=10'sd250;sine=-10'sd53;end // -12 degrees
  22:begin cosine=10'sd254;sine=-10'sd36;end // -8 degrees
  23:begin cosine=10'sd255;sine=-10'sd18;end // -4 degrees
  default:begin cosine=256;sine=0;end
 endcase
end

 reg signed[25:0]mx,xc,yc,zc,rx,rz,ry,tmp;
 reg signed[11:0]my;
 wire signed[37:0]product=mx*my;
 wire signed[25:0]scaled_product=product>>>8;
 wire signed[25:0]point_zc=($signed({1'b0,z})-$signed({1'b0,pivot_z}))*26'sd468;
 reg signed[25:0]screen_x,screen_y;
 wire signed[25:0]depth_key=26'sd8+((tmp+scaled_product)>>>15);
 reg[3:0]point_key;
 wire publish_busy;wire[32:0]published,published_ack;
 wire[32:0]publication={source_job,back_bank};
 cdc_mode #(.WIDTH(33)) cloud_present(.src_clk(memory_clk),.dst_clk(pixel_clk),.rst_n(rst_n),
  .write(state==PUBLISH&&flush_done&&!publish_busy),.mode(publication),.busy(publish_busy),
  .commit(x==0&&y==720),.destination_mode(published),.src_mode(published_ack));
 assign pixel_job=published[32:1];assign pixel_cloud_valid=published[32:1]!=0;
 // Exact 16/3 nearest-neighbour raster. Counters avoid wide divide operators.
 reg[7:0]px,py;reg[3:0]xf,yf;
 wire[7:0]read_y=y>=720?8'd0:x==0?(y==0?8'd0:(yf>=13?py+1'b1:py)):py;
 wire[7:0]read_x=x==0||x>=1280?8'd0:px;
 wire[15:0]pixel_address=({read_y,8'd0}-{read_y,4'd0})+read_x;
 function[15:0]palette(input[3:0]n);
  begin case(n)
   0:palette=16'h0842;1:palette=16'hf800;2:palette=16'hfa00;3:palette=16'hfc00;
   4:palette=16'hfe00;5:palette=16'hffe0;6:palette=16'hafe0;7:palette=16'h57e0;
   8:palette=16'h07e0;9:palette=16'h07ea;10:palette=16'h07f5;11:palette=16'h07ff;
   12:palette=16'h055f;13:palette=16'h02bf;14:palette=16'h001f;15:palette=16'h501f;
  endcase end
 endfunction
 // At x=0 the next pixel is still in the same enlarged source cell, so
 // registered px advancement aligns with the one-cycle read like depth_lines.
 always @(posedge pixel_clk)begin
  if(!pixel_reset[1])begin px<=0;py<=0;xf<=0;yf<=0;bank_q<=0;end
  else begin
   if(x==0)begin px<=0;xf<=3;
    if(y==0)begin py<=0;yf<=0;end
    else if(y<720)begin if(yf>=13)begin yf<=yf-13;py<=py+1'b1;end else yf<=yf+3;end
   end else if(de)begin
    if(xf>=13)begin xf<=xf-13;px<=px+1'b1;end else xf<=xf+3;
   end
   bank_q<=published[0];
   
  end
 end
 cloud_colour_bank colour0(.write_clk(memory_clk),.write_enable(buffer_write&&buffer_ready&&!back_bank),
  .write_address(buffer_address[14:0]),.write_data(colour333),.read_clk(pixel_clk),.read_address(pixel_address[14:0]),.read_data(colour0_q));
 cloud_colour_bank colour1(.write_clk(memory_clk),.write_enable(buffer_write&&buffer_ready&&back_bank),
  .write_address(buffer_address[14:0]),.write_data(colour333),.read_clk(pixel_clk),.read_address(pixel_address[14:0]),.read_data(colour1_q));
 assign pixel_data=shown565;assign pixel_valid=enable_pixel&&pixel_cloud_valid;assign front_bank=published[0];
 assign buffer_bank=back_bank;assign buffer_clear=state==CLEAR;assign flush_request=state==PUBLISH;
 assign line_request=state==REQUEST&&enable_memory&&!fault;
 assign line_new=v==0;assign line_row={8'd0,v};
 assign buffer_write=state==CLEAR||(state==ZTEST&&point_key<=zread);
 assign buffer_address=state==CLEAR?clear_address:draw_address;
 assign buffer_colour=state==CLEAR?16'h0842:draw_colour;
 wire[3:0]buffer_depth=state==CLEAR?4'd15:point_key;
 always @(posedge memory_clk)begin
  if(state==CLEAR&&clear_address<16)depth_histogram[clear_address[3:0]]<=0;
  else if(state==DECODE&&enable_memory&&measure_pass&&point>=500&&point<=1500)
   depth_histogram[point_bin]<=depth_histogram[point_bin]+1'b1;
 end
 always @(posedge memory_clk)begin
  if(line_write&&state==WAIT_LINE&&line_address<80)row_buffer[line_address]<=line_data;
  row_word<=row_buffer[u[8:2]];
  zread<=zbuffer[draw_address];
  if(buffer_write&&buffer_ready)begin
   zbuffer[buffer_address]<=buffer_depth;


  end
 end
 always @(posedge memory_clk)begin
  if(!memory_reset[1])begin
   state<=IDLE;back_bank<=1;clear_address<=0;draw_address<=0;draw_colour<=0;point_rgb<=0;draw_depth<=0;
   u<=0;v<=0;beats<=0;source_job<=0;read_age<=0;orbit_age<=0;z<=0;pivot_z<=750;min_z<=1500;max_z<=500;
   angle<=0;measure_pass<=1;zoom<=13;valid_point_count<=0;histogram_index<=0;histogram_running<=0;screen_x<=0;screen_y<=0;mx<=0;my<=0;xc<=0;yc<=0;zc<=0;rx<=0;rz<=0;ry<=0;tmp<=0;point_key<=15;
   rendered_frames<=0;rejected_frames<=0;visible_points<=0;fault<=0;
  end else begin
   if(state==CLEAR&&clear_address==0)valid_point_count<=0;
   if(state==WAIT_LINE)begin
    if(read_age!=20'hfffff)read_age<=read_age+1'b1;
    if(line_write)begin
     if(line_address!=beats||beats>=160)fault<=1;
     else beats<=beats+1'b1;
    end
   end
   case(state)
    IDLE:begin fault<=0;angle<=0;measure_pass<=1;
     if(enable_memory&&!publish_busy)begin back_bank<=!published_ack[0];clear_address<=0;visible_points<=0;min_z<=1500;max_z<=500;state<=CLEAR;end
    end
    CLEAR:if(!enable_memory)state<=IDLE;
     else if(!buffer_ready)begin end
     else if(clear_address==32399)begin v<=0;u<=0;state<=REQUEST;end
     else clear_address<=clear_address+1'b1;
    REQUEST:begin read_age<=0;beats<=0;
     if(!enable_memory)state<=IDLE;
     else if(line_ready)state<=WAIT_LINE;
    end
    WAIT_LINE:if(line_done)begin
     if(!enable_memory)state<=IDLE;
     else if(v==0&&!measure_pass&&line_good&&line_job!=source_job)state<=IDLE;
     else if(!line_good||beats!=160||fault||line_job==0||((v!=0||!measure_pass)&&source_job!=line_job))begin fault<=1;rejected_frames<=rejected_frames+1'b1;state<=FAILED;end
     else begin if(v==0&&measure_pass)source_job<=line_job;u<=0;state<=READ_POINT;end
    end else if(read_age==1000000)begin fault<=1;end // Retain owner until DONE, never abandon DDR.
    READ_POINT:state<=DECODE;
    DECODE:if(!enable_memory)state<=IDLE;
     else if(point<500||point>1500)state<=NEXT;
     else if(measure_pass)begin valid_point_count<=valid_point_count+1'b1;state<=NEXT;end
     else begin point_rgb<=source_point[15:0];z<=point;mx<=$signed({1'b0,u})-26'sd160;my<=$signed({1'b0,point});state<=MUL_X;
      if(point<min_z)min_z<=point;if(point>max_z)max_z<=point;end
    MUL_X:begin xc<=product;mx<=$signed({1'b0,v})-26'sd90;my<=$signed({1'b0,z});zc<=point_zc;state<=MUL_Y;end
    MUL_Y:begin yc<=product;mx<=xc;my<=cosine;state<=MUL_XC;end
    MUL_XC:begin tmp<=scaled_product;mx<=zc;my<=sine;state<=MUL_ZS;end
    MUL_ZS:begin rx<=tmp+scaled_product;mx<=zc;my<=cosine;state<=MUL_ZC;end
    MUL_ZC:begin tmp<=scaled_product;mx<=xc;my<=sine;state<=MUL_XS;end
    MUL_XS:begin rz<=tmp-scaled_product;mx<=yc;my<=12'sd247;state<=MUL_YC;end
    MUL_YC:begin tmp<=scaled_product;mx<=rz;my<=-12'sd66;state<=MUL_ZP;end
    MUL_ZP:begin ry<=tmp+scaled_product;mx<=rz;my<=12'sd247;state<=MUL_ZCP;end
    MUL_ZCP:begin tmp<=scaled_product;mx<=yc;my<=12'sd66;state<=MUL_YP;end
    MUL_YP:begin point_key<=depth_key<1?1:depth_key>14?14:depth_key[3:0];mx<=rx;my<=$signed({1'b0,zoom});state<=MUL_SCREEN_X;end
    MUL_SCREEN_X:begin screen_x<=26'sd120+($signed(product)>>>14);mx<=ry;my<=$signed({1'b0,zoom});state<=MUL_SCREEN_Y;end
    MUL_SCREEN_Y:begin screen_y<=26'sd67+($signed(product)>>>14);state<=PROJECT;end
    PROJECT:if(screen_x<0||screen_x>=240||screen_y<0||screen_y>=135)state<=NEXT;
     else begin draw_address<=({screen_y[7:0],8'd0}-{screen_y[7:0],4'd0})+screen_x[7:0];draw_colour<=point_rgb;state<=ZWAIT;end
    ZWAIT:state<=ZTEST;
    ZTEST:if(point_key>zread||buffer_ready)begin if(point_key<=zread&&visible_points!=65535)visible_points<=visible_points+1'b1;state<=NEXT;end
    NEXT:if(!enable_memory)state<=IDLE;
     else if(u!=319)begin u<=u+1'b1;state<=READ_POINT;end
     else if(v!=179)begin v<=v+1'b1;state<=REQUEST;end
     else if(measure_pass)begin histogram_index<=0;histogram_running<=0;state<=MEDIAN;end
     else state<=PUBLISH;
    MEDIAN:if(valid_point_count==0)state<=REWIND;
     else if(histogram_next>=(({1'b0,valid_point_count}+17'd1)>>1)||histogram_index==15)begin
      pivot_z<=16'd532+{histogram_index,6'd0};zoom<=zoom_for_bin(histogram_index);state<=REWIND;
     end else begin histogram_running<=histogram_next;histogram_index<=histogram_index+1'b1;end
    REWIND:begin measure_pass<=0;v<=0;u<=0;state<=REQUEST;end
    PUBLISH:if(flush_done&&!publish_busy)begin state<=ACK;rendered_frames<=rendered_frames+1'b1;end
    ACK:if(!publish_busy)begin orbit_age<=0;state<=PAUSE;end
    PAUSE:begin
     if(!enable_memory)state<=IDLE;
     else if(orbit_memory&&orbit_age==ORBIT_CYCLES-1&&!publish_busy)begin
      orbit_age<=0;back_bank<=!published_ack[0];clear_address<=0;visible_points<=0;min_z<=1500;max_z<=500;
      angle<=angle==23?0:angle+1'b1;
      state<=CLEAR;
     end else if(orbit_memory)orbit_age<=orbit_age+1'b1;
    end
    FAILED:if(!enable_memory)state<=IDLE;
    default:begin fault<=1;state<=FAILED;end
   endcase
  end
 end
endmodule


// Exact TD simple dual-clock template: power-of-two depth and bounded ports.
// No RAM reset. The back bank is fully cleared before publication.
module cloud_colour_bank(input wire write_clk,write_enable,
 input wire[14:0]write_address,input wire[8:0]write_data,
 input wire read_clk,input wire[14:0]read_address,output reg[8:0]read_data);
 (* ram_style="bram" *) reg[8:0]memory[0:32767];
 always @(posedge write_clk)if(write_enable)memory[write_address]<=write_data;
 always @(posedge read_clk)read_data<=memory[read_address];
endmodule
