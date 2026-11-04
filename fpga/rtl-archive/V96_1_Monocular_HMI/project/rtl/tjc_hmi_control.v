`timescale 1ns/1ps
// Independent 115200 UART. Only image controls are emitted; no motor command path.
module tjc_hmi_control #(parameter DIV=434,HEARTBEAT=50000000,LINK_TIMEOUT=150000000)(
 input wire clk,rst_n,input wire rx,output wire tx,
 input wire[2:0]mode_applied,input wire mode_busy,
 input wire[1:0]adjust_applied,input wire adjust_busy,
 input wire noise_applied,noise_requested,noise_busy,ae_enabled,awb_enabled,
 input wire debug_write,
 output wire control_write,output reg[15:0]control_address,output reg[7:0]control_data,
 output wire[255:0]diagnostics
);
 wire[7:0]rx_data,tx_data;wire rx_valid,frame_error,tx_valid,tx_ready;
 uart_byte #(.DIV(DIV))hmi_uart(.clk(clk),.rst_n(rst_n),.rx(rx),.tx(tx),
 .tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),.rx_data(rx_data),.rx_valid(rx_valid),.framing_error(frame_error));
 wire page_valid,touch_valid,touch_down;wire[2:0]page;wire[15:0]touch_x,touch_y;
 wire[31:0]rx_bytes,rx_packets,rx_errors,tx_bytes;
 tjc_packet_rx packets(.clk(clk),.rst_n(rst_n),.data(rx_data),.valid(rx_valid),.framing_error(frame_error),
 .page_valid(page_valid),.page(page),.touch_valid(touch_valid),.touch_down(touch_down),.touch_x(touch_x),.touch_y(touch_y),
 .bytes_received(rx_bytes),.packets_received(rx_packets),.framing_errors(rx_errors));
 reg connected,have_page;reg[2:0]current_page;reg[27:0]link_age;reg[25:0]heartbeat;
 reg refresh,poll_pending,press_pending,message_pending;reg[5:0]press_command,message_command;
 reg[2:0]phase;reg[2:0]old_mode;reg[1:0]old_adjust;reg[2:0]old_isp;
 reg[31:0]accepted,rejected;reg[15:0]last_x,last_y;
 reg pressed_valid,control_pending;reg[5:0]pressed_command;
 wire pending_busy=(control_address==16'h0290&&mode_busy)||
  (control_address==16'h0291&&adjust_busy)||(control_address==16'h0280&&noise_busy);
 assign control_write=control_pending&&!debug_write&&!pending_busy;
 wire[2:0]isp={noise_applied,awb_enabled,ae_enabled};
 reg hit;reg[5:0]down_command;reg[15:0]hit_address;reg[7:0]hit_data;reg hit_busy;
 always @*begin
  hit=0;down_command=0;hit_address=0;hit_data=0;hit_busy=debug_write;
  if(current_page==1)begin
   if((touch_y>=51&&touch_y<113)||(touch_y>=122&&touch_y<184))begin
    if(touch_x>=68&&touch_x<160)begin hit=1;hit_data=0;end
    else if(touch_x>=171&&touch_x<263)begin hit=1;hit_data=1;end
    else if(touch_x>=274&&touch_x<366)begin hit=1;hit_data=2;end
    else if(touch_x>=377&&touch_x<469)begin hit=1;hit_data=3;end
    if(touch_y>=122)hit_data=hit_data+4;
    hit_address=16'h0290;down_command=32+hit_data;hit_busy=debug_write||mode_busy;
   end else if(touch_y>=206&&touch_y<238)begin
    if(touch_x>=169&&touch_x<234)begin hit=1;hit_data=0;end
    else if(touch_x>=243&&touch_x<308)begin hit=1;hit_data=1;end
    else if(touch_x>=317&&touch_x<382)begin hit=1;hit_data=2;end
    else if(touch_x>=391&&touch_x<456)begin hit=1;hit_data=3;end
    hit_address=16'h0291;down_command=40+hit_data;hit_busy=debug_write||adjust_busy;
   end
  end else if(current_page==2&&touch_x>=246&&touch_x<306)begin
   if(touch_y>=57&&touch_y<89)begin hit=1;hit_address=16'h00e0;hit_data={7'd0,!ae_enabled};down_command=44;end
   else if(touch_y>=108&&touch_y<140)begin hit=1;hit_address=16'h0300;hit_data={7'd0,!awb_enabled};down_command=45;end
   else if(touch_y>=159&&touch_y<191)begin hit=1;hit_address=16'h0280;hit_data={7'd0,!noise_requested};down_command=46;hit_busy=debug_write||noise_busy;end
  end
 end
 reg[5:0]command;reg command_valid;wire command_ready;
 tjc_command_sender sender(.clk(clk),.rst_n(rst_n),.command(command),.command_valid(command_valid),.command_ready(command_ready),
 .tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),.tx_bytes(tx_bytes));
 always @*begin
  command=0;command_valid=0;
  if(phase!=0)begin
   case(phase)
    1:begin command=1+current_page;command_valid=1;end
    2:begin command=current_page==3?26:current_page==4?27:7;command_valid=1;end
    3:if(current_page==1)begin command=8+mode_applied;command_valid=1;end
      else if(current_page==2)begin command=20+ae_enabled;command_valid=1;end
      else if(current_page==5)begin command=29;command_valid=1;end
    4:if(current_page==1)begin command=16+adjust_applied;command_valid=1;end
      else if(current_page==2)begin command=22+awb_enabled;command_valid=1;end
    5:if(current_page==2)begin command=24+noise_applied;command_valid=1;end
    default:begin command=0;command_valid=0;end
   endcase
  end else if(press_pending)begin command=press_command;command_valid=1;end
  else if(message_pending)begin command=message_command;command_valid=1;end
  else if(poll_pending)begin command=0;command_valid=1;end
 end
 assign diagnostics={tx_bytes,rx_errors,rejected,accepted,rx_packets,rx_bytes,last_y,last_x,
  16'd0,5'd0,current_page,3'd0,have_page,connected,mode_applied};
 always @(posedge clk)begin
  if(!rst_n)begin connected<=0;have_page<=0;current_page<=0;link_age<=0;heartbeat<=HEARTBEAT-5000000;
   refresh<=0;poll_pending<=0;press_pending<=0;message_pending<=0;press_command<=0;message_command<=0;
   phase<=0;old_mode<=0;old_adjust<=0;old_isp<=0;accepted<=0;rejected<=0;last_x<=0;last_y<=0;
   pressed_valid<=0;pressed_command<=0;control_pending<=0;control_address<=0;control_data<=0;end
  else begin
   if(control_pending&&!debug_write)begin
    control_pending<=0;
    if(pending_busy)begin rejected<=rejected+1'b1;message_pending<=1;message_command<=28;end
    else accepted<=accepted+1'b1;
   end
   if(heartbeat>=HEARTBEAT-1)begin heartbeat<=0;poll_pending<=1;end else heartbeat<=heartbeat+1'b1;
   if(link_age<LINK_TIMEOUT)link_age<=link_age+1'b1;else connected<=0;
   if(phase!=0)begin
    if(!command_valid||command_ready)begin if(phase==5)phase<=0;else phase<=phase+1'b1;end
   end else if(command_valid&&command_ready)begin
    if(press_pending)press_pending<=0;
    else if(message_pending)message_pending<=0;
    else if(poll_pending)poll_pending<=0;
   end else if(refresh&&connected)begin phase<=1;refresh<=0;end
   if(mode_applied!=old_mode||adjust_applied!=old_adjust||isp!=old_isp)begin
    old_mode<=mode_applied;old_adjust<=adjust_applied;old_isp<=isp;refresh<=1;
   end
   if(page_valid)begin
    connected<=1;have_page<=1;link_age<=0;
    if(!connected||!have_page||current_page!=page)begin refresh<=1;press_pending<=0;pressed_valid<=0;message_pending<=0;phase<=0;end
    current_page<=page;
   end
   if(touch_valid&&connected&&have_page)begin
    last_x<=touch_x;last_y<=touch_y;
    if(touch_down)pressed_valid<=hit;
    else begin pressed_valid<=0;press_pending<=0;if(current_page==1||current_page==2)refresh<=1;end
    if(hit)begin
     if(touch_down)begin press_pending<=1;press_command<=down_command;pressed_command<=down_command;end
     else if(pressed_valid&&pressed_command==down_command)begin
      press_pending<=0;refresh<=1;
      if(hit_busy||control_pending)begin rejected<=rejected+1'b1;message_pending<=1;message_command<=28;end
      else begin control_pending<=1;control_address<=hit_address;control_data<=hit_data;end
     end
    end else if(!touch_down&&touch_x>=68&&(current_page==3||current_page==4))begin
     message_pending<=1;message_command<=current_page==3?26:27;
    end
   end
  end
 end
endmodule
