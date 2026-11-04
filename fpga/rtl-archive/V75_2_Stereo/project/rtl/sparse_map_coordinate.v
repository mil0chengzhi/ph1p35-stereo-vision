// Isolated coordinate generator. External ROMs have one synchronous read cycle.
// No camera/DDR/metric depth, calibration selection or board configuration here.
module sparse_map_coordinate #(
    parameter integer WIDTH=320, HEIGHT=180, GRID_LOG2=3, TAG_WIDTH=32,
    parameter integer NX=((WIDTH-1)>>GRID_LOG2)+2,
    parameter integer NY=((HEIGHT-1)>>GRID_LOG2)+2,
    parameter integer NODE_ADDR_BITS=$clog2(2*NX*NY),
    parameter integer MASK_ADDR_BITS=$clog2(2*WIDTH*HEIGHT)
)(
    input wire clk,rst,
    input wire in_valid,output wire in_ready,
    input wire in_camera,
    input wire [10:0] in_x,in_y,
    input wire [TAG_WIDTH-1:0] in_tag,
    output wire node_en,output reg [NODE_ADDR_BITS-1:0] node_addr,
    input wire [35:0] node_data,
    output wire mask_en,output wire [MASK_ADDR_BITS-1:0] mask_addr,
    input wire mask_data,
    output wire configuration_error,
    output reg out_valid,input wire out_ready,
    output reg out_camera,out_usable,
    output reg signed [17:0] out_x_q5,out_y_q5,
    output reg [TAG_WIDTH-1:0] out_tag
);
    localparam integer STEP=1<<GRID_LOG2;
    localparam integer CONFIG_OK=(WIDTH>=1&&WIDTH<=1280&&HEIGHT>=1&&HEIGHT<=720&&
        GRID_LOG2>=0&&GRID_LOG2<=5&&NX==((WIDTH-1)/STEP)+2&&NY==((HEIGHT-1)/STEP)+2);
    localparam [4:0] IDLE=0,REQ0=1,WAIT0=2,REQ1=3,WAIT1=4,REQ2=5,WAIT2=6,
        REQ3=7,WAIT3=8,HORIZ_X=9,VERT_X=10,HORIZ_Y=11,VERT_Y=12,EMIT=13,HOLD=14;
    reg [4:0] state;
    reg [NODE_ADDR_BITS-1:0] base_addr;
    reg [MASK_ADDR_BITS-1:0] mask_index;
    reg [4:0] fx,fy;
    reg [35:0] n00,n10,n01,n11;
    reg [TAG_WIDTH-1:0] tag;
    reg camera,request_in_range,original_valid;
    reg signed [23:0] top_sum,bottom_sum;
    reg signed [17:0] qx,qy;
    wire select_y=state==HORIZ_Y;
    wire signed [17:0] c00=select_y?$signed(n00[35:18]):$signed(n00[17:0]);
    wire signed [17:0] c10=select_y?$signed(n10[35:18]):$signed(n10[17:0]);
    wire signed [17:0] c01=select_y?$signed(n01[35:18]):$signed(n01[17:0]);
    wire signed [17:0] c11=select_y?$signed(n11[35:18]):$signed(n11[17:0]);
    wire signed [18:0] dt=$signed({c10[17],c10})-$signed({c00[17],c00});
    wire signed [18:0] db=$signed({c11[17],c11})-$signed({c01[17],c01});
    wire signed [24:0] pt=dt*$signed({1'b0,fx});
    wire signed [24:0] pb=db*$signed({1'b0,fx});
    wire signed [25:0] ht=($signed({{8{c00[17]}},c00})<<<GRID_LOG2)+$signed(pt);
    wire signed [25:0] hb=($signed({{8{c01[17]}},c01})<<<GRID_LOG2)+$signed(pb);
    wire signed [24:0] dv=$signed({bottom_sum[23],bottom_sum})-$signed({top_sum[23],top_sum});
    wire signed [30:0] pv=dv*$signed({1'b0,fy});
    wire signed [31:0] numerator=($signed({{8{top_sum[23]}},top_sum})<<<GRID_LOG2)+$signed(pv);
    wire signed [31:0] rounded=(numerator+(1<<(2*GRID_LOG2)))>>>(2*GRID_LOG2+1);
    assign configuration_error=!CONFIG_OK;
    assign in_ready=CONFIG_OK&&state==IDLE&&!rst;
    assign node_en=!rst&&(state==REQ0||state==REQ1||state==REQ2||state==REQ3);
    assign mask_en=!rst&&state==REQ0;
    assign mask_addr=mask_index;
    always @*begin
        node_addr=base_addr;
        case(state)
            REQ1:node_addr=base_addr+1;
            REQ2:node_addr=base_addr+NX;
            REQ3:node_addr=base_addr+NX+1;
            default:node_addr=base_addr;
        endcase
    end
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;out_valid<=0;out_usable<=0;out_camera<=0;out_tag<=0;
            out_x_q5<=0;out_y_q5<=0;fx<=0;fy<=0;base_addr<=0;mask_index<=0;
            n00<=0;n10<=0;n01<=0;n11<=0;tag<=0;camera<=0;
            request_in_range<=0;original_valid<=0;top_sum<=0;bottom_sum<=0;qx<=0;qy<=0;
        end else case(state)
            IDLE:if(in_valid&&in_ready)begin
                tag<=in_tag;camera<=in_camera;
                request_in_range<=in_x<WIDTH&&in_y<HEIGHT;
                if(in_x<WIDTH&&in_y<HEIGHT)begin
                    base_addr<=(in_camera?NX*NY:0)+(in_y>>GRID_LOG2)*NX+(in_x>>GRID_LOG2);
                    mask_index<=(in_camera?WIDTH*HEIGHT:0)+in_y*WIDTH+in_x;
                    fx<=in_x&(STEP-1);fy<=in_y&(STEP-1);
                end else begin
                    // Invalid requests still yield one tagged invalid result; never address outside ROM.
                    base_addr<=0;mask_index<=0;fx<=0;fy<=0;
                end
                state<=REQ0;
            end
            REQ0:state<=WAIT0;
            WAIT0:begin n00<=node_data;original_valid<=mask_data;state<=REQ1;end
            REQ1:state<=WAIT1;
            WAIT1:begin n10<=node_data;state<=REQ2;end
            REQ2:state<=WAIT2;
            WAIT2:begin n01<=node_data;state<=REQ3;end
            REQ3:state<=WAIT3;
            WAIT3:begin n11<=node_data;state<=HORIZ_X;end
            HORIZ_X:begin top_sum<=ht[23:0];bottom_sum<=hb[23:0];state<=VERT_X;end
            VERT_X:begin qx<=rounded[17:0];state<=HORIZ_Y;end
            HORIZ_Y:begin top_sum<=ht[23:0];bottom_sum<=hb[23:0];state<=VERT_Y;end
            VERT_Y:begin qy<=rounded[17:0];state<=EMIT;end
            EMIT:begin
                out_camera<=camera;out_tag<=tag;out_x_q5<=qx;out_y_q5<=qy;
                out_usable<=request_in_range&&original_valid&&qx>=0&&qy>=0&&qx<(1279*32)&&qy<(719*32);
                out_valid<=1;state<=HOLD;
            end
            HOLD:if(out_ready)begin out_valid<=0;state<=IDLE;end
            default:begin state<=IDLE;out_valid<=0;end
        endcase
    end
endmodule
