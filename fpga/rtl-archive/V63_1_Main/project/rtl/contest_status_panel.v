`timescale 1ns/1ps
// One synchronous font-ROM stage plus one output register. Header is drawn
// inside the1280x720 picture; enable=0 is bit-exact passthrough. Six16bit
// values convert in16 blanking clocks; no per-pixel decimal divider.
module contest_status_panel(
 input wire clk,rst_n,input wire [2:0] mode,input wire camera_remote,
 input wire [15:0] x,y,input wire [23:0] rgb,input wire de,valid,hs,vs,
 input wire [15:0] fps,latency_ms,input wire fps_valid,latency_valid,
 input wire track_visible,input wire [15:0] track_id,center_x,center_y,
 input wire motion_found,lighting_change,input wire [15:0] motion_count,
 output reg [15:0] out_x,out_y,output reg [23:0] out_rgb,
 output reg out_de,out_valid,out_hs,out_vs
);
 // Late blank leaves frame-boundary CDC updates and the effects pipeline settled.
 wire snapshot=x==0&&y==724;
 wire [119:0] decimal_bcd;wire decimal_busy,decimal_done;
 ui_decimal #(.COUNT(6)) decimal_values(.clk(clk),.rst_n(rst_n),.load(snapshot),
  .values({motion_count,center_y,center_x,track_id,latency_ms,fps}),
  .bcd(decimal_bcd),.busy(decimal_busy),.done(decimal_done));
 reg [119:0] shown_bcd;reg shown_fps,shown_latency,shown_track,shown_lighting;
 reg shown_camera;reg [2:0] shown_mode;
 always @(posedge clk)begin
  if(!rst_n)begin shown_bcd<=0;shown_fps<=0;shown_latency<=0;shown_track<=0;shown_lighting<=0;shown_camera<=0;shown_mode<=0;end
  else begin
   if(snapshot)begin shown_fps<=fps_valid;shown_latency<=latency_valid;shown_track<=track_visible;
    shown_lighting<=lighting_change;shown_camera<=camera_remote;shown_mode<=mode;end
   if(decimal_done)shown_bcd<=decimal_bcd;
  end
 end
 wire panel=de&&valid&&mode!=0&&x>=352&&x<992&&y>=16&&y<72;
 wire text_row=(y>=16&&y<32)||(y>=36&&y<52)||(y>=56&&y<72);
 wire [1:0] row=y<36?0:(y<56?1:2);
 wire [5:0] column=(x-352)>>4;
 wire [3:0] gx=x[3:0],gy=row==0?(y-16):(row==1?(y-36):(y-56));
 reg [5:0] glyph;
 reg [3:0] digit;
 function [5:0] digit_glyph;input [119:0] all_bcd;input integer field,place;input enabled;
  reg [3:0] number;begin number=all_bcd[field*20+place*4+:4];
   digit_glyph=enabled&&number<10?6'd1+number:6'd26;end
 endfunction
 always @*begin
  glyph=0;digit=0;
  if(row==0)begin
   case(shown_mode)
    3'd0:case(column)
    default:glyph=0;endcase
    3'd1:case(column)
    5:glyph=6'd27;
    6:glyph=6'd28;
    7:glyph=6'd29;
    8:glyph=6'd30;
    default:glyph=0;endcase
    3'd2:case(column)
    5:glyph=6'd31;
    6:glyph=6'd32;
    default:glyph=0;endcase
    3'd3:case(column)
    5:glyph=6'd33;
    6:glyph=6'd34;
    default:glyph=0;endcase
    3'd4:case(column)
    5:glyph=6'd35;
    6:glyph=6'd36;
    default:glyph=0;endcase
    3'd5:case(column)
    5:glyph=6'd37;
    6:glyph=6'd38;
    default:glyph=0;endcase
    3'd6:case(column)
    5:glyph=6'd39;
    6:glyph=6'd40;
    7:glyph=6'd41;
    8:glyph=6'd42;
    default:glyph=0;endcase
    3'd7:case(column)
    5:glyph=6'd43;
    6:glyph=6'd44;
    7:glyph=6'd45;
    8:glyph=6'd46;
    default:glyph=0;endcase

    default:glyph=0;
   endcase
   case(column)
    0:glyph=6'd11;
    1:glyph=6'd12;
    2:glyph=6'd13;

    10:glyph=6'd14;
    11:glyph=6'd15;
    12:glyph=6'd16;
    3:glyph=shown_camera?6'd3:6'd2;
    14:glyph=(shown_fps) ? (6'd1+shown_bcd[12+:4]) : 6'd26;
    15:glyph=(shown_fps) ? (6'd1+shown_bcd[8+:4]) : 6'd26;
    16:glyph=(shown_fps) ? (6'd1+shown_bcd[4+:4]) : 6'd26;
    17:glyph=(shown_fps) ? (6'd1+shown_bcd[0+:4]) : 6'd26;

    default:begin end
   endcase
  end else if(row==1)begin
   case(column)
    0:glyph=6'd2;
    1:glyph=6'd3;
    2:glyph=6'd9;
    3:glyph=6'd1;
    4:glyph=6'd24;
    5:glyph=6'd8;
    6:glyph=6'd3;
    7:glyph=6'd1;
    9:glyph=6'd47;
    10:glyph=6'd48;
    15:glyph=6'd22;
    16:glyph=6'd23;

    11:glyph=(shown_latency) ? (6'd1+shown_bcd[32+:4]) : 6'd26;
    12:glyph=(shown_latency) ? (6'd1+shown_bcd[28+:4]) : 6'd26;
    13:glyph=(shown_latency) ? (6'd1+shown_bcd[24+:4]) : 6'd26;
    14:glyph=(shown_latency) ? (6'd1+shown_bcd[20+:4]) : 6'd26;

    default:glyph=0;
   endcase
  end else if(shown_mode==6)begin
   case(column)
    0:glyph=6'd19;

    1:glyph=6'd25;
    2:glyph=(shown_track) ? (6'd1+shown_bcd[52+:4]) : 6'd26;
    3:glyph=(shown_track) ? (6'd1+shown_bcd[48+:4]) : 6'd26;
    4:glyph=(shown_track) ? (6'd1+shown_bcd[44+:4]) : 6'd26;
    5:glyph=(shown_track) ? (6'd1+shown_bcd[40+:4]) : 6'd26;

    7:glyph=6'd20;
    8:glyph=(shown_track) ? (6'd1+shown_bcd[72+:4]) : 6'd26;
    9:glyph=(shown_track) ? (6'd1+shown_bcd[68+:4]) : 6'd26;
    10:glyph=(shown_track) ? (6'd1+shown_bcd[64+:4]) : 6'd26;
    11:glyph=(shown_track) ? (6'd1+shown_bcd[60+:4]) : 6'd26;

    13:glyph=6'd21;
    14:glyph=(shown_track) ? (6'd1+shown_bcd[92+:4]) : 6'd26;
    15:glyph=(shown_track) ? (6'd1+shown_bcd[88+:4]) : 6'd26;
    16:glyph=(shown_track) ? (6'd1+shown_bcd[84+:4]) : 6'd26;
    17:glyph=(shown_track) ? (6'd1+shown_bcd[80+:4]) : 6'd26;

    default:glyph=0;
   endcase
  end else if(shown_mode==7)begin
   case(column)
    0:glyph=6'd43;
    1:glyph=6'd44;

    2:glyph=(1'b1) ? (6'd1+shown_bcd[116+:4]) : 6'd26;
    3:glyph=(1'b1) ? (6'd1+shown_bcd[112+:4]) : 6'd26;
    4:glyph=(1'b1) ? (6'd1+shown_bcd[108+:4]) : 6'd26;
    5:glyph=(1'b1) ? (6'd1+shown_bcd[104+:4]) : 6'd26;
    6:glyph=(1'b1) ? (6'd1+shown_bcd[100+:4]) : 6'd26;

    default:glyph=0;
   endcase
   if(shown_lighting)case(column)
    8:glyph=6'd49;
    9:glyph=6'd50;
    10:glyph=6'd51;
    11:glyph=6'd52;

    default:begin end
   endcase
  end
 end
 wire foreground;
 contest_panel_font font(.clk(clk),.rst_n(rst_n),.glyph(glyph),.x(gx),.y(gy),.foreground(foreground));
 reg panel_q,text_q;reg [15:0] xq,yq;reg [23:0] rgbq;reg deq,validq,hsq,vsq;
 always @(posedge clk)begin
  if(!rst_n)begin panel_q<=0;text_q<=0;xq<=0;yq<=0;rgbq<=0;deq<=0;validq<=0;hsq<=1;vsq<=1;
   out_x<=0;out_y<=0;out_rgb<=0;out_de<=0;out_valid<=0;out_hs<=1;out_vs<=1;end
  else begin
   panel_q<=panel;text_q<=text_row;xq<=x;yq<=y;rgbq<=rgb;deq<=de;validq<=valid;hsq<=hs;vsq<=vs;
   out_x<=xq;out_y<=yq;out_de<=deq;out_valid<=validq;out_hs<=hsq;out_vs<=vsq;
   out_rgb<=panel_q?(text_q&&foreground?24'hffffff:24'h15243b):rgbq;
  end
 end
endmodule
