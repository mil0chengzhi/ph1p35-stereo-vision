`timescale 1ns/1ps
// 320x180 logical points, each{Z_mm16,RGB56516}, become a640x180 word frame.
// Four points per128-bit DATA. START/DATA/END use the existing130-bit FIFO ABI.
// frame_sent means END was handed to the store, not that DDR published it.
module stereo_result_packer #(parameter WIDTH=320,HEIGHT=180,X_BITS=9,Y_BITS=8)(
 input wire clk,rst,input wire start_valid,output wire start_ready,
 input wire[31:0]start_frame,start_session,input wire abort_frame,
 input wire in_valid,output wire in_ready,input wire[15:0]in_Z_mm,in_colour,
 input wire[31:0]in_frame_tag,input wire[X_BITS-1:0]in_x,input wire[Y_BITS-1:0]in_y,
 output wire token_empty,input wire token_read,output reg token_valid,output reg[129:0]token_data,
 output wire busy,output reg frame_sent,frame_good,frame_fault,output wire configuration_error
);
 localparam CONFIG_OK=WIDTH>=4&&WIDTH%4==0&&WIDTH<=1280&&HEIGHT>=1&&HEIGHT<=720&&X_BITS>=$clog2(WIDTH)&&Y_BITS>=$clog2(HEIGHT);
 localparam IDLE=0,START=1,COLLECT=2,WORD=3,ENDING=4;
 reg[2:0]state;reg[31:0]frame_id,session;reg[19:0]pixels;
 reg[X_BITS-1:0]expected_x;reg[Y_BITS-1:0]expected_y;
 reg[1:0]lane;reg[127:0]packed_word;reg bad;
 assign configuration_error=!CONFIG_OK;
 assign busy=state!=IDLE||token_valid;
 assign start_ready=!rst&&CONFIG_OK&&state==IDLE&&!token_valid&&!abort_frame&&
                    start_session[31:24]==8'hD2&&start_session[21:0]!=0;
 assign in_ready=!rst&&state==COLLECT&&!bad&&!abort_frame;
 assign token_empty=rst||!(state==START||state==WORD||state==ENDING);
 wire pop=token_read&&!token_empty;
 wire good_end=!bad&&!abort_frame&&pixels==WIDTH*HEIGHT;
 always @(posedge clk)begin
  if(rst)begin state<=IDLE;frame_id<=0;session<=0;pixels<=0;expected_x<=0;expected_y<=0;lane<=0;packed_word<=0;
   bad<=0;token_valid<=0;token_data<=0;frame_sent<=0;frame_good<=0;frame_fault<=0;end
  else begin
   token_valid<=0;frame_sent<=0;frame_good<=0;
   if(abort_frame&&state!=IDLE)begin bad<=1;frame_fault<=1;if(state==COLLECT)state<=ENDING;end
   case(state)
    IDLE:if(start_valid&&start_ready)begin
     frame_id<=start_frame;session<=start_session;pixels<=0;expected_x<=0;expected_y<=0;lane<=0;bad<=0;frame_fault<=0;state<=START;
    end
    START:if(pop)begin
     token_data<={2'b01,64'd0,session,frame_id};token_valid<=1;state<=bad||abort_frame?ENDING:COLLECT;
    end
    COLLECT:if(in_valid&&in_ready)begin
     if(in_frame_tag!=frame_id||in_x!=expected_x||in_y!=expected_y||pixels>=WIDTH*HEIGHT)begin
      bad<=1;frame_fault<=1;state<=ENDING;
     end else begin
      packed_word[lane*32+:32]<={in_Z_mm,in_colour};pixels<=pixels+1'b1;
      if(expected_x==WIDTH-1)begin expected_x<=0;expected_y<=expected_y+1'b1;end else expected_x<=expected_x+1'b1;
      if(lane==3)begin state<=WORD;lane<=0;end else lane<=lane+1'b1;
     end
    end
    WORD:if(pop)begin token_data<={2'b00,packed_word};token_valid<=1;state<=bad||abort_frame||pixels==WIDTH*HEIGHT?ENDING:COLLECT;end
    ENDING:if(pop)begin
     token_data<={2'b10,63'd0,session,frame_id,good_end};token_valid<=1;
     frame_sent<=1;frame_good<=good_end;state<=IDLE;
    end
    default:begin frame_fault<=1;bad<=1;state<=ENDING;end
   endcase
  end
 end
endmodule
