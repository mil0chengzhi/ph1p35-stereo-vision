// Isolated feature top, NOT a board design. Default calibration remains off.
// Physical A=0: four remote native720 slots. Physical B=1: local960/crop120.
// Real-rig default logical LEFT=B (LEFT_CAMERA=1); ROM order stays logical L/R.
// Complete48-bit cookies and both local slots stay fixed across each whole job.
// Isolated complete two-snapshot feature generator, not a board top.
// External snapshots must remain immutable under both physical-camera leases.
// ROM order is logical left/right; LEFT_CAMERA binds that to physical A/B.
// Calibration is disabled by default. Test callers explicitly use synthetic ROMs.
module stereo_leased_rectified_features #(
    parameter integer WIDTH=320,HEIGHT=180,GRID_LOG2=5,LEFT_CAMERA=1,
    parameter integer CALIBRATION_READY=0,
    parameter [31:0] MAP_ID=0,
    parameter NODE_FILE="nodes.mem",MASK_FILE="masks.mem",
    parameter integer X_BITS=$clog2(WIDTH+4),Y_BITS=$clog2(HEIGHT+4),MEM_BITS=$clog2(5*(WIDTH+4))
)(
    input wire clk,rst,input wire start_valid,output wire start_ready,
    input wire[31:0]pair_tag,calibration_id,input wire[47:0]start_cookie_A,start_cookie_B,
    input wire[1:0]lease_valid,input wire[47:0]lease_cookie_A,lease_cookie_B,input wire[1:0]lease_slot_A,lease_slot_B,
    output wire[24:0]araddr,output wire arvalid,input wire arready,
    input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,read_active,
    output wire left_valid,input wire left_ready,output wire[23:0]left_descriptor,
    output wire left_usable,output wire[31:0]left_frame_tag,
    output wire[X_BITS-1:0]left_x,output wire[Y_BITS-1:0]left_y,
    output wire right_valid,input wire right_ready,output wire[23:0]right_descriptor,
    output wire right_usable,output wire[31:0]right_frame_tag,
    output wire[X_BITS-1:0]right_x,output wire[Y_BITS-1:0]right_y,
    output wire colour_valid,output wire[31:0]colour_index,output wire[15:0]colour_rgb,
    output reg frame_done,output reg busy,output reg fault,output reg[3:0]fault_reason,
    output wire configuration_error
);
    localparam TOTAL=WIDTH*HEIGHT,CX_BITS=$clog2(2*WIDTH+4);
    reg[CX_BITS-1:0]merged_input_x;
    localparam CONFIG_OK=CALIBRATION_READY&&MAP_ID!=0&&(LEFT_CAMERA==0||LEFT_CAMERA==1)&&
        WIDTH>=5&&HEIGHT>=5&&WIDTH<=1280&&HEIGHT<=720;
    reg[31:0]active_pair;reg[47:0]cookie_A,cookie_B;reg[1:0]slot_A,slot_B;
    reg[10:0]request_x,request_y;
    reg request_right,requests_done,reader_camera;
    reg[31:0]row_start,received_left,received_right;
    reg[1:0]completed;
    wire left_start_ready,right_start_ready,left_done,right_done,left_fault,right_fault;
    wire map_config_error,reader_config_error,reader_fault;
    wire[2:0]reader_fault_reason;
    wire initial_leases=lease_valid==3&&lease_cookie_A==start_cookie_A&&lease_cookie_B==start_cookie_B;
    wire active_leases=lease_valid==3&&lease_cookie_A==cookie_A&&lease_cookie_B==cookie_B&&lease_slot_A==slot_A&&lease_slot_B==slot_B;
    assign configuration_error=!CONFIG_OK||map_config_error||reader_config_error;
    assign start_ready=!rst&&!busy&&!fault&&!configuration_error&&calibration_id==MAP_ID&&
        initial_leases&&left_start_ready&&right_start_ready;
    wire take_start=start_valid&&start_ready;
    wire request_camera=request_right^(LEFT_CAMERA!=0);
    wire[47:0]request_cookie=request_camera?cookie_B:cookie_A;
    wire map_in_ready,map_valid,map_ready,map_right,map_usable;
    wire signed[17:0]map_x_q5,map_y_q5;
    wire[79:0]map_tag;
    wire mapped_camera=map_right^(LEFT_CAMERA!=0);
    hd_sparse_map_rom #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.GRID_LOG2(GRID_LOG2),.TAG_WIDTH(80),
        .NODE_FILE(NODE_FILE),.MASK_FILE(MASK_FILE))map(
        .clk(clk),.rst(rst),.in_valid(busy&&!requests_done&&!fault),.in_ready(map_in_ready),
        .in_camera(request_right),.in_x(request_x),.in_y(request_y),.in_tag({request_cookie,row_start+request_x}),
        .configuration_error(map_config_error),.out_valid(map_valid),.out_ready(map_ready),
        .out_camera(map_right),.out_usable(map_usable),.out_x_q5(map_x_q5),.out_y_q5(map_y_q5),.out_tag(map_tag));
    wire reader_ready,sample_valid,sample_ready,sample_camera,sample_usable;
    wire[47:0]sample_cookie;wire[31:0]sample_tag;
    wire[7:0]p00,p10,p01,p11;wire[4:0]frac_x,frac_y;wire[15:0]sample_rgb;
    // The next ROM result may refer to another camera while a DDR request is
    // still live. Select its lease only at acceptance; retain the active camera.
    wire lease_camera_select=reader_ready?mapped_camera:reader_camera;
    assign map_ready=reader_ready&&!fault;
    hd_cached_texels reader(
        .clk(clk),.rst(rst),.in_valid(map_valid&&!fault),.in_ready(reader_ready),
        .in_camera(mapped_camera),.in_usable(map_usable),.in_x_q5(map_x_q5),.in_y_q5(map_y_q5),
        .in_cookie(map_tag[79:32]),.in_tag(map_tag[31:0]),
        .lease_valid(lease_valid[lease_camera_select]),.lease_camera(lease_camera_select),
        .lease_cookie(lease_camera_select?lease_cookie_B:lease_cookie_A),.lease_slot(lease_camera_select?lease_slot_B:lease_slot_A),
        .araddr(araddr),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rvalid(rvalid),.rlast(rlast),.rready(rready),
        .read_active(read_active),.configuration_error(reader_config_error),
        .out_valid(sample_valid),.out_ready(sample_ready),.out_usable(sample_usable),.out_camera(sample_camera),
        .p00(p00),.p10(p10),.p01(p01),.p11(p11),.out_rgb565(sample_rgb),.frac_x(frac_x),.frac_y(frac_y),
        .out_cookie(sample_cookie),.out_tag(sample_tag),.fault(reader_fault),.fault_reason(reader_fault_reason));
    wire sample_right=sample_camera!=(LEFT_CAMERA!=0);
    assign colour_valid=sample_valid&&sample_ready&&!sample_right;
    assign colour_index=sample_tag;assign colour_rgb=sample_rgb;
    wire sample_matches=(sample_right==(merged_input_x>=WIDTH))&&sample_cookie==(sample_camera?cookie_B:cookie_A)&&
        sample_tag==(sample_right?received_right:received_left);
    wire left_input_ready,right_input_ready;
    wire raw_left_valid,raw_right_valid;
    assign left_valid=raw_left_valid&&!fault;
    assign right_valid=raw_right_valid&&!fault;
    assign sample_ready=!fault&&sample_matches&&(sample_right?right_input_ready:left_input_ready);
    wire shared_valid,shared_ready,shared_input_ready,shared_usable,shared_fault,shared_done,shared_start_ready;
    wire[23:0]shared_descriptor;wire[31:0]shared_frame;
    wire[CX_BITS-1:0]shared_x;wire[Y_BITS-1:0]shared_y;
    wire shared_right=shared_x>=WIDTH;
    wire[CX_BITS-1:0]shared_local_x=shared_right?shared_x-WIDTH:shared_x;
    assign left_input_ready=shared_input_ready;assign right_input_ready=shared_input_ready;
    assign left_start_ready=shared_start_ready;assign right_start_ready=shared_start_ready;
    assign left_done=shared_done;assign right_done=shared_done;
    assign left_fault=shared_fault;assign right_fault=shared_fault;
    assign raw_left_valid=shared_valid&&!shared_right;
    assign raw_right_valid=shared_valid&&shared_right;
    assign shared_ready=shared_right?right_ready:left_ready;
    assign left_descriptor=shared_descriptor;assign right_descriptor=shared_descriptor;
    assign left_usable=shared_usable;assign right_usable=shared_usable;
    assign left_frame_tag=shared_frame;assign right_frame_tag=shared_frame;
    assign left_x=shared_local_x;assign right_x=shared_local_x;
    assign left_y=shared_y;assign right_y=shared_y;
    // The reader already emits complete logical-left then logical-right rows.
    // A single wide scan preserves that order and each image's own borders.
    rectification_feature_gaussian_stream #(.WIDTH(2*WIDTH),.HEIGHT(HEIGHT),.SPLIT_COLUMN(WIDTH),
      .X_BITS(CX_BITS),.Y_BITS(Y_BITS),.MEM_BITS($clog2(5*(2*WIDTH+4))))shared_features(
      .clk(clk),.rst(rst),.start_valid(take_start),.start_ready(shared_start_ready),.frame_tag(pair_tag),
      .in_valid(sample_valid&&sample_matches&&!fault),.in_ready(shared_input_ready),
      .p00(p00),.p10(p10),.p01(p01),.p11(p11),.frac_x(frac_x),.frac_y(frac_y),.sample_usable(sample_usable),
      .out_valid(shared_valid),.out_ready(shared_ready&&!fault),.descriptor(shared_descriptor),
      .descriptor_usable(shared_usable),.out_frame_tag(shared_frame),.out_x(shared_x),.out_y(shared_y),
      .frame_done(shared_done),.fault(shared_fault));
    always @(posedge clk)begin
        if(rst)begin
            busy<=0;fault<=0;fault_reason<=0;frame_done<=0;active_pair<=0;cookie_A<=0;cookie_B<=0;slot_A<=0;slot_B<=0;
            request_x<=0;request_y<=0;request_right<=0;requests_done<=0;row_start<=0;
            received_left<=0;received_right<=0;completed<=0;reader_camera<=0;merged_input_x<=0;
        end else begin
            frame_done<=0;
            if(take_start)begin
                busy<=1;active_pair<=pair_tag;cookie_A<=start_cookie_A;cookie_B<=start_cookie_B;slot_A<=lease_slot_A;slot_B<=lease_slot_B;
                request_x<=0;request_y<=0;request_right<=0;requests_done<=0;row_start<=0;
                received_left<=0;received_right<=0;completed<=0;merged_input_x<=0;
            end
            if(busy&&!requests_done&&!fault&&map_in_ready)begin
                if(request_x==WIDTH-1)begin
                    request_x<=0;
                    if(!request_right)request_right<=1;
                    else begin
                        request_right<=0;
                        if(request_y==HEIGHT-1)requests_done<=1;
                        else begin request_y<=request_y+1'b1;row_start<=row_start+WIDTH;end
                    end
                end else request_x<=request_x+1'b1;
            end
            if(map_valid&&map_ready)reader_camera<=mapped_camera;
            if(sample_valid&&sample_ready)begin
                if(merged_input_x==2*WIDTH-1)merged_input_x<=0;
                else merged_input_x<=merged_input_x+1'b1;
                if(sample_right)received_right<=received_right+1'b1;
                else received_left<=received_left+1'b1;
            end
            if(busy&&!fault)begin
                if(!active_leases||calibration_id!=MAP_ID)begin fault<=1;fault_reason<=1;end
                else if((sample_valid&&!sample_matches)||(raw_left_valid&&left_frame_tag!=active_pair)||
                    (raw_right_valid&&right_frame_tag!=active_pair))begin fault<=1;fault_reason<=2;end
                else if(left_fault)begin fault<=1;fault_reason<=3;end
                else if(right_fault)begin fault<=1;fault_reason<=4;end
                else if(reader_fault)begin fault<=1;fault_reason<=5;end
                else begin
                    completed<=completed|{right_done,left_done};
                    if((completed|{right_done,left_done})==3)begin
                        if(!requests_done||received_left!=TOTAL||received_right!=TOTAL||read_active)begin fault<=1;fault_reason<=6;end
                        else begin busy<=0;frame_done<=1;end
                    end
                end
            end
        end
    end
endmodule
