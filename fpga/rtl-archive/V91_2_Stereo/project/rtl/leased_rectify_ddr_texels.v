// ISOLATED UNQUALIFIED leased video-frame texel reader, not a board module.
// Fixed physical layout: A slots4..7/crop0; B slots0..3/crop120.
// Both source image geometries are normalized native1280x720 for maps.
// Full48-bit lease cookie and local2-bit slot are checked while samples are live.
// Reset still requires verified outer DDR/consumer drain or whole-MC reset.
// Read-only isolated pAXI client for normalized 1280x720 RGB565 snapshots.
// Snapshot creation/ownership, bus arbitration and true camera pairing are external.
// Both snapshot slots must be immutable under the matching camera/cookie lease.
// Addresses count 16-bit DDR words; each accepted command returns one 128-bit beat.
module leased_rectify_ddr_texels #(
    parameter integer TAG_WIDTH=32,TIMEOUT=20000,
    parameter integer SLOT_WORDS=2097152,ADDRESS_WORDS=33554432
)(
    input wire clk,rst,
    input wire in_valid,output wire in_ready,
    input wire in_camera,in_usable,
    input wire signed [17:0] in_x_q5,in_y_q5,
    input wire [47:0] in_cookie,input wire [TAG_WIDTH-1:0] in_tag,
    input wire lease_valid,lease_camera,input wire [47:0]lease_cookie,input wire[1:0]lease_slot,
    output wire [24:0]araddr,output wire arvalid,input wire arready,
    input wire [127:0]rdata,input wire rvalid,rlast,output wire rready,
    output wire read_active,output wire configuration_error,
    output reg out_valid,input wire out_ready,
    output reg out_usable,out_camera,
    output reg [7:0]p00,p10,p01,p11,
    output reg[15:0]out_rgb565,
    output reg [4:0]frac_x,frac_y,
    output reg [47:0]out_cookie,output reg [TAG_WIDTH-1:0]out_tag,
    output reg fault,output reg [2:0]fault_reason
);
    // Physical A=0 uses remote slots4..7, normalized720 rows.
    // Physical B=1 uses local slots0..3, native960 rows, crop starts at120.
    localparam CONFIG_OK=(SLOT_WORDS>=1280*960&&SLOT_WORDS%8==0&&
        8*SLOT_WORDS<=ADDRESS_WORDS&&ADDRESS_WORDS<=33554432&&TIMEOUT>=4);
    localparam IDLE=0,CMD=1,DATA=2,EMIT=3,HOLD=4,FAILED=5;
    reg[2:0]state;
    reg[1:0]beat_kind;
    reg[2:0]lane;
    reg[24:0]first_addr,address;
    reg outstanding,usable,camera,command_live;reg[1:0]slot;
    reg[47:0]cookie;reg[31:0]age;
    reg[TAG_WIDTH-1:0]tag;
    reg[4:0]fx,fy;
    reg[15:0]c00,c10,c01,c11;
    wire lease_matches=lease_valid&&lease_camera==camera&&lease_cookie==cookie&&lease_slot==slot;
    wire request_bounds=in_x_q5>=0&&in_y_q5>=0&&in_x_q5<(1279*32)&&in_y_q5<(719*32);
    wire request_lease=lease_valid&&lease_camera==in_camera&&lease_cookie==in_cookie;
    wire[24:0]row_index={15'b0,in_y_q5[14:5]}+(in_camera?25'd120:25'd0);
    wire[31:0]global_slot=(in_camera?32'd0:32'd4)+{30'd0,lease_slot};
    wire[31:0]request_address_wide=global_slot*SLOT_WORDS+
        (row_index<<10)+(row_index<<8)+{in_x_q5[15:8],3'b000};
    wire[24:0]request_address=request_address_wide[24:0];
    wire[15:0]near_pixel=rdata[lane*16+:16];
    wire[2:0]next_lane=lane+3'd1;
    wire[15:0]next_pixel=rdata[next_lane*16+:16];
    function[7:0]luma565;
        input[15:0]rgb;
        reg[7:0]r,g,b;
        reg[15:0]sum,wr,wg,wb;
        begin
            r={rgb[15:11],rgb[15:13]};g={rgb[10:5],rgb[10:9]};b={rgb[4:0],rgb[4:2]};
            wr={8'b0,r};wg={8'b0,g};wb={8'b0,b};
            // Exact77R+150G+29B; explicit shifts reserve DSPs for matching/warp.
            sum=(wr<<6)+(wr<<3)+(wr<<2)+wr+(wg<<7)+(wg<<4)+(wg<<2)+(wg<<1)+
                (wb<<5)-(wb<<1)-wb+128;luma565=sum[15:8];
        end
    endfunction
    assign configuration_error=!CONFIG_OK;
    assign in_ready=CONFIG_OK&&!rst&&!fault&&state==IDLE;
    assign araddr=address;
    assign arvalid=!rst&&command_live; // Posted AR remains stable until accepted, including on fault.
    assign rready=!rst&&(outstanding||fault);
    assign read_active=!rst&&(state==CMD||state==DATA||command_live||outstanding);
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;beat_kind<=0;lane<=0;first_addr<=0;address<=0;outstanding<=0;command_live<=0;slot<=0;
            usable<=0;camera<=0;cookie<=0;age<=0;tag<=0;fx<=0;fy<=0;
            c00<=0;c10<=0;c01<=0;c11<=0;out_valid<=0;out_usable<=0;out_camera<=0;
            p00<=0;p10<=0;p01<=0;p11<=0;out_rgb565<=0;frac_x<=0;frac_y<=0;out_cookie<=0;out_tag<=0;
            fault<=0;fault_reason<=0;
        end else if(fault)begin
            // A failed in-flight request keeps bus ownership until a final beat
            // arrives; never reinterpret a late response as a new pixel.
            out_valid<=0;
            if(arvalid&&arready)begin command_live<=0;outstanding<=1;end
            if(rvalid&&rready&&rlast)outstanding<=0;
        end else begin
            if(state==CMD||state==DATA)age<=age+1'b1;else age<=0;
            if(usable&&(state==CMD||state==DATA||state==EMIT||state==HOLD)&&!lease_matches)begin
                fault<=1;fault_reason<=1;state<=FAILED;out_valid<=0;
                if(arvalid&&arready)begin command_live<=0;outstanding<=1;end
                if(rvalid&&rready&&rlast)outstanding<=0;
            end else if((state==CMD||state==DATA)&&age==TIMEOUT-1)begin
                fault<=1;fault_reason<=2;state<=FAILED;out_valid<=0;
                if(arvalid&&arready)begin command_live<=0;outstanding<=1;end
                if(rvalid&&rready&&rlast)outstanding<=0;
            end else if(rvalid&&state!=DATA)begin
                fault<=1;fault_reason<=3;state<=FAILED;out_valid<=0;
                if(arvalid&&arready)begin command_live<=0;outstanding<=1;end
                if(rready&&rlast)outstanding<=0;
            end else case(state)
                IDLE:if(in_valid&&in_ready)begin
                    camera<=in_camera;cookie<=in_cookie;tag<=in_tag;slot<=lease_slot;
                    fx<=in_x_q5[4:0];fy<=in_y_q5[4:0];
                    c00<=0;c10<=0;c01<=0;c11<=0;
                    usable<=in_usable&&request_bounds&&request_lease;
                    if(in_usable&&request_bounds&&request_lease)begin
                        lane<=in_x_q5[7:5];first_addr<=request_address;address<=request_address;
                        beat_kind<=0;age<=0;state<=CMD;command_live<=1;
                    end else state<=EMIT;
                end
                CMD:if(arvalid&&arready)begin command_live<=0;outstanding<=1;state<=DATA;end
                DATA:if(rvalid&&rready)begin
                    if(!rlast)begin fault<=1;fault_reason<=4;state<=FAILED;out_valid<=0;end
                    else begin
                        outstanding<=0;
                        case(beat_kind)
                            0:begin
                                c00<=near_pixel;
                                if(lane==7)begin beat_kind<=1;address<=first_addr+8;end
                                else begin c10<=next_pixel;beat_kind<=2;address<=first_addr+1280;end
                                state<=CMD;command_live<=1;
                            end
                            1:begin c10<=rdata[15:0];beat_kind<=2;address<=first_addr+1280;state<=CMD;command_live<=1;end
                            2:begin
                                c01<=near_pixel;
                                if(lane==7)begin beat_kind<=3;address<=first_addr+1288;state<=CMD;command_live<=1;end
                                else begin c11<=next_pixel;state<=EMIT;end
                            end
                            3:begin c11<=rdata[15:0];state<=EMIT;end
                        endcase
                    end
                end
                EMIT:begin
                    out_camera<=camera;out_cookie<=cookie;out_tag<=tag;out_usable<=usable;
                    p00<=usable?luma565(c00):0;p10<=usable?luma565(c10):0;
                    p01<=usable?luma565(c01):0;p11<=usable?luma565(c11):0;
                    out_rgb565<=!usable?16'd0:(fy[4]?(fx[4]?c11:c01):(fx[4]?c10:c00));
                    frac_x<=fx;frac_y<=fy;out_valid<=1;state<=HOLD;
                end
                HOLD:if(out_ready)begin out_valid<=0;state<=IDLE;end
                default:begin fault<=1;fault_reason<=5;state<=FAILED;end
            endcase
        end
    end
endmodule
