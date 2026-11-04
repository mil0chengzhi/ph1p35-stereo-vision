`timescale 1ns/1ps
// CRC-committed host chunks reuse the existing native model receiver and DDR.
// Host payload remains immutable until receiver consume and its DDR fence.
module usb_model_staging(
 input wire host_clk,memory_clk,rst_n,
 input wire host_write,input wire[15:0]host_address,input wire[7:0]host_data,
 input wire block_commit,output wire host_selected,output reg[7:0]host_read_data,
 input wire net_valid,input wire[191:0]net_header,output wire net_consume,
 input wire[7:0]net_payload,
 output wire packet_valid,output wire[191:0]packet_header,input wire packet_consume,
 input wire[10:0]payload_address,output wire[7:0]payload_data,
 input wire reply_valid,input wire[7:0]reply_kind,input wire[31:0]reply_job,input wire[15:0]reply_sequence,
 output wire reply_ready,output wire net_reply_valid,input wire net_reply_ready
);
 (* ram_style="bram" *)reg[7:0]payload[0:1279];reg[7:0]payload_q;
 reg[31:0]job;reg[15:0]packet_seq;reg[7:0]kind;
 reg[10:0]staged_count;reg verified,host_error;reg[31:0]published;
 reg owner_valid,owner_host;
 wire source_busy,source_write,source_valid;wire[55:0]source_data;
 wire publish=host_write&&host_address==16'h1507&&host_data==1;
 assign source_write=publish&&!source_busy&&!host_error&&job!=0&&
  ((kind==4&&packet_seq==0)||(kind==1&&packet_seq<2880&&verified));
 wire payload_write=host_write&&host_address>=16'h1000&&host_address<16'h1500;
 wire staged_write=host_write&&host_address>=16'h1000&&host_address<16'h1507;
 always @(posedge host_clk)if(payload_write&&!source_busy)payload[host_address-16'h1000]<=host_data;
 always @(posedge memory_clk)payload_q<=payload[payload_address];
 follow_payload_mailbox #(.WIDTH(56))to_memory(.source_clk(host_clk),.destination_clk(memory_clk),.rst_n(rst_n),
  .source_write(source_write),.source_data({kind,job,packet_seq}),.source_busy(source_busy),
  .valid(source_valid),.data(source_data),.consume(packet_consume&&owner_valid&&owner_host));
 always @(posedge memory_clk)begin
  if(!rst_n)begin owner_valid<=0;owner_host<=0;end
  else if(packet_consume)owner_valid<=0;
  else if(!owner_valid&&(source_valid||net_valid))begin owner_valid<=1;owner_host<=source_valid;end
 end
 assign packet_valid=owner_valid&&(owner_host?source_valid:net_valid);
 assign packet_header=owner_host?{40'd0,source_data[55:48],16'd0,source_data[47:16],source_data[15:0],80'd0}:net_header;
 assign payload_data=owner_host?payload_q:net_payload;
 assign net_consume=packet_consume&&owner_valid&&!owner_host;
 assign reply_ready=owner_valid&&(owner_host?1'b1:net_reply_ready);
 assign net_reply_valid=reply_valid&&owner_valid&&!owner_host;
 wire reply_busy,reply_host_valid;wire[55:0]reply_host_data;
 follow_payload_mailbox #(.WIDTH(56))to_host(.source_clk(memory_clk),.destination_clk(host_clk),.rst_n(rst_n),
  .source_write(reply_valid&&reply_ready&&owner_host),.source_data({reply_kind,reply_job,reply_sequence}),.source_busy(reply_busy),
  .valid(reply_host_valid),.data(reply_host_data),.consume(1'b1));
 reg[55:0]last_reply;
 assign host_selected=host_address>=16'h1500&&host_address<16'h1520;
 always @*begin
  host_read_data=0;
  case(host_address)
   16'h1508:host_read_data={4'd0,verified,source_busy,host_error,1'b0};
   16'h1509:host_read_data=last_reply[55:48];
   16'h150a:host_read_data=last_reply[7:0];
   16'h150b:host_read_data=last_reply[15:8];
   16'h150c:host_read_data=last_reply[23:16];
   16'h150d:host_read_data=last_reply[31:24];
   16'h150e:host_read_data=last_reply[39:32];
   16'h150f:host_read_data=last_reply[47:40];
   16'h1510:host_read_data=published[7:0];
   16'h1511:host_read_data=published[15:8];
   16'h1512:host_read_data=published[23:16];
   16'h1513:host_read_data=published[31:24];
   default:host_read_data=0;
  endcase
 end
 always @(posedge host_clk)begin
  if(!rst_n)begin job<=0;packet_seq<=0;kind<=0;staged_count<=0;verified<=0;host_error<=0;published<=0;last_reply<=0;end
  else begin
   if(reply_host_valid)last_reply<=reply_host_data;
   if(staged_write)begin
    if(source_busy)host_error<=1;
    else begin
     if(host_address==16'h1000)begin staged_count<=1;verified<=0;end
     else if(host_address==16'h1000+staged_count)staged_count<=staged_count+1'b1;
     else if(host_address<16'h1500)begin staged_count<=0;verified<=0;end
     if(host_address>=16'h1500&&host_address<16'h1504)job[(host_address-16'h1500)*8+:8]<=host_data;
     if(host_address==16'h1504)packet_seq[7:0]<=host_data;
     if(host_address==16'h1505)packet_seq[15:8]<=host_data;
     if(host_address==16'h1506)kind<=host_data;
    end
   end
   if(block_commit&&staged_count==1287&&!source_busy)verified<=1;
   if(source_write)begin published<=published+1'b1;verified<=0;end
   if(publish&&!source_write)host_error<=1;
   if(host_write&&host_address==16'h1507&&host_data==2&&!source_busy)begin host_error<=0;verified<=0;staged_count<=0;end
  end
 end
endmodule
