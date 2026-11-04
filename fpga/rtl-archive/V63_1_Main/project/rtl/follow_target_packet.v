`timescale 1ns/1ps
// One atomic record for all four colours. No pixel, NV32 header or disparity
// is repurposed as control data. EtherType 88B6 carries this 256-bit payload.
// [255:224] A source epoch; [223:192] frame; [191:176] message sequence;
// [175:174] reserved zero; [173] source valid; [172] overflow;
// [171:168] colour-valid mask; each low 42-bit box is {y1[9:0],y0[9:0],x1[10:0],x0[10:0]}.
module follow_target_packet(
 input wire clk,rst_n,result_commit,source_valid,overflow,
 input wire[31:0]epoch,frame_id,
 input wire[3:0]valid,
 input wire[63:0]x0,x1,y0,y1,
 input wire mailbox_busy,
 output wire mailbox_write,output wire[255:0]mailbox_data
);
 reg seen;reg[31:0]last_frame,last_epoch;reg last_valid;
 reg[15:0]message_sequence;
 (* keep=1,preserve=1,allow_backward_retiming=0,allow_forward_retiming=0 *)reg[1:0]local_reset;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)local_reset<=0;else local_reset<={local_reset[0],1'b1};
 wire fresh=!seen||epoch!=last_epoch||(frame_id!=last_frame&&(frame_id-last_frame)<32'h80000000);
 wire safe_source=source_valid&&!overflow;
 assign mailbox_write=local_reset[1]&&result_commit&&!mailbox_busy&&(fresh||(!safe_source&&last_valid));
 wire[3:0]qualified;
 genvar k;
 generate for(k=0;k<4;k=k+1)begin:boxes
  assign qualified[k]=safe_source&&valid[k]&&x1[k*16+:16]>=x0[k*16+:16]&&
   y1[k*16+:16]>=y0[k*16+:16]&&x1[k*16+:16]<1280&&y1[k*16+:16]<720;
  assign mailbox_data[k*42+:42]={y1[k*16+:10],y0[k*16+:10],x1[k*16+:11],x0[k*16+:11]};
 end endgenerate
 assign mailbox_data[255:168]={epoch,frame_id,message_sequence+16'd1,2'd0,source_valid,overflow,qualified};
 always @(posedge clk or negedge local_reset[1])begin
  if(!local_reset[1])begin seen<=0;last_frame<=0;last_epoch<=0;last_valid<=0;message_sequence<=0;end
  else if(mailbox_write)begin
   seen<=1;last_frame<=frame_id;last_epoch<=epoch;last_valid<=safe_source;
   message_sequence<=message_sequence+1'b1;
  end
 end
endmodule

module follow_target_unpack(input wire[255:0]payload,input wire[1:0]colour,
 output wire[199:0]target);
 reg[41:0]box;
 always @*case(colour)
  0:box=payload[41:0];1:box=payload[83:42];
  2:box=payload[125:84];default:box=payload[167:126];
 endcase
 wire[15:0]x0={5'd0,box[10:0]},x1={5'd0,box[21:11]};
 wire[15:0]y0={6'd0,box[31:22]},y1={6'd0,box[41:32]};
 wire[15:0]cx=(x0+x1)>>1,cy=(y0+y1)>>1;
 wire source_good=payload[175:174]==0&&payload[173]&&!payload[172];
 wire found=source_good&&payload[168+colour];
 assign target={1'b0,colour,source_good,found?4'd0:4'd1,found,found,14'd0,
  y1,y0,x1,x0,cy,cx,payload[255:224],payload[223:192],payload[191:176]};
endmodule
