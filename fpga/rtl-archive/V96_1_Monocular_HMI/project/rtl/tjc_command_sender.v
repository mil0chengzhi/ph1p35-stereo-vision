`timescale 1ns/1ps
// ROM of complete, manufacturer-native commands. UART consumes one byte/handshake.
module tjc_command_sender #(parameter FILE="V96_1_Monocular_HMI/project/rtl/tjc_commands.mem")(
 input wire clk,rst_n,input wire[5:0]command,input wire command_valid,output wire command_ready,
 output wire[7:0]tx_data,output wire tx_valid,input wire tx_ready,output reg[31:0]tx_bytes
);
 (* ram_style="block" *)reg[7:0]rom[0:4095];initial $readmemh(FILE,rom);
 reg busy,loaded;reg[11:0]address;reg[8:0]remaining;reg[7:0]q;
 reg[11:0]base;reg[8:0]length;
 always @*begin base=0;length=0;case(command)
   6'd0:begin base=12'd0;length=9'd9;end
   6'd1:begin base=12'd9;length=9'd12;end
   6'd2:begin base=12'd21;length=9'd12;end
   6'd3:begin base=12'd33;length=9'd12;end
   6'd4:begin base=12'd45;length=9'd12;end
   6'd5:begin base=12'd57;length=9'd12;end
   6'd6:begin base=12'd69;length=9'd12;end
   6'd7:begin base=12'd81;length=9'd57;end
   6'd8:begin base=12'd138;length=9'd27;end
   6'd9:begin base=12'd165;length=9'd28;end
   6'd10:begin base=12'd193;length=9'd28;end
   6'd11:begin base=12'd221;length=9'd28;end
   6'd12:begin base=12'd249;length=9'd28;end
   6'd13:begin base=12'd277;length=9'd29;end
   6'd14:begin base=12'd306;length=9'd29;end
   6'd15:begin base=12'd335;length=9'd29;end
   6'd16:begin base=12'd364;length=9'd29;end
   6'd17:begin base=12'd393;length=9'd29;end
   6'd18:begin base=12'd422;length=9'd29;end
   6'd19:begin base=12'd451;length=9'd29;end
   6'd20:begin base=12'd480;length=9'd44;end
   6'd21:begin base=12'd524;length=9'd44;end
   6'd22:begin base=12'd568;length=9'd45;end
   6'd23:begin base=12'd613;length=9'd45;end
   6'd24:begin base=12'd658;length=9'd45;end
   6'd25:begin base=12'd703;length=9'd45;end
   6'd26:begin base=12'd748;length=9'd58;end
   6'd27:begin base=12'd806;length=9'd59;end
   6'd28:begin base=12'd865;length=9'd58;end
   6'd29:begin base=12'd923;length=9'd54;end
   6'd32:begin base=12'd977;length=9'd27;end
   6'd33:begin base=12'd1004;length=9'd28;end
   6'd34:begin base=12'd1032;length=9'd28;end
   6'd35:begin base=12'd1060;length=9'd28;end
   6'd36:begin base=12'd1088;length=9'd28;end
   6'd37:begin base=12'd1116;length=9'd29;end
   6'd38:begin base=12'd1145;length=9'd29;end
   6'd39:begin base=12'd1174;length=9'd29;end
   6'd40:begin base=12'd1203;length=9'd29;end
   6'd41:begin base=12'd1232;length=9'd29;end
   6'd42:begin base=12'd1261;length=9'd29;end
   6'd43:begin base=12'd1290;length=9'd29;end
   6'd44:begin base=12'd1319;length=9'd27;end
   6'd45:begin base=12'd1346;length=9'd29;end
   6'd46:begin base=12'd1375;length=9'd29;end
   default:begin base=0;length=0;end
  endcase end
 assign command_ready=!busy;assign tx_valid=busy&&loaded;assign tx_data=q;
 always @(posedge clk)begin
  if(!rst_n)begin busy<=0;loaded<=0;address<=0;remaining<=0;q<=0;tx_bytes<=0;end
  else if(!busy)begin
   if(command_valid&&length!=0)begin busy<=1;loaded<=0;address<=base;remaining<=length;end
  end else if(!loaded)begin q<=rom[address];loaded<=1;end
  else if(tx_ready)begin
   tx_bytes<=tx_bytes+1'b1;loaded<=0;
   if(remaining==1)busy<=0;
   else begin address<=address+1'b1;remaining<=remaining-1'b1;end
  end
 end
endmodule
