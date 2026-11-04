// Memory-domain coordinator for serial, STATIC camera snapshots and row reads.
// This is an auxiliary client: requires the separately protected video/aux DDR
// arbiter. A/B capture is sequential and does NOT synchronize camera exposure.
module snapshot_diagnostic_engine #(
    parameter integer CAPTURE_TIMEOUT=10000000,BUS_TIMEOUT=200000,ROW_TIMEOUT=1000000
)(
    input wire clk,rst,input wire[1:0]tap_valid,input wire[259:0]tap_data,
    input wire command_valid,output wire command_ready,input wire[159:0]command,
    output reg done,output reg[255:0]response,input wire external_fault,
    output wire fault,configuration_error,
    output wire ram_write,output wire[7:0]ram_address,output wire[127:0]ram_data,
    output wire[24:0]awaddr,araddr,output wire[127:0]wdata,output wire[15:0]wstrb,
    output wire awvalid,wvalid,wlast,arvalid,rready,read_active,
    input wire awready,wready,arready,rvalid,rlast,input wire[127:0]rdata
);
    localparam IDLE=0,CAP_ARM=1,CAP_WAIT=2,LEASE_CHECK=3,LEASE_WAIT=4,
        RELEASE_CHECK=5,RELEASE_WAIT=6,ROW_REQ=7,ROW_WAIT=8,FINISH=9;
    reg[3:0]state;
    reg[7:0]opcode,error_code;
    reg camera;reg[15:0]row;
    reg[31:0]session,expected_A,expected_B,tag,lease_owner,row_crc;
    reg sticky_fault;
    wire writer_fault,writer_config,row_fault,row_config,mux_fault;
    wire arm_ready,capture_done,capture_success,writer_busy,lease_ready;
    wire[3:0]capture_reason;
    wire[1:0]snapshot_valid,lease_locked;
    wire[31:0]epoch_A,epoch_B,frame_A,frame_B,session_A,session_B,tag_A,tag_B;
    wire[24:0]cap_awaddr,cap_araddr;wire[127:0]cap_wdata,cap_rdata;wire[15:0]cap_wstrb;
    wire cap_awvalid,cap_wvalid,cap_wlast,cap_arvalid,cap_rready,cap_read_active;
    wire cap_awready,cap_wready,cap_arready,cap_rvalid,cap_rlast;
    wire[24:0]feat_araddr;wire[127:0]feat_rdata;
    wire feat_arvalid,feat_arready,feat_rvalid,feat_rlast,feat_rready,feat_read_active;
    wire row_ready,row_done,row_good,row_camera;wire[9:0]row_number;
    wire[31:0]row_epoch,row_tag,row_crc_result;wire[3:0]row_error;
    wire[1:0]mux_selected;
    wire capture_claim=state==CAP_ARM||state==CAP_WAIT||writer_busy||cap_read_active||cap_awvalid||cap_wvalid;
    wire row_claim=state==ROW_REQ||state==ROW_WAIT||feat_read_active;
    wire current_epochs=expected_A==epoch_A&&expected_B==epoch_B;
    wire release_ok=lease_locked==3&&tag==lease_owner;
    assign configuration_error=writer_config||row_config;
    assign fault=sticky_fault||writer_fault||row_fault||mux_fault||external_fault||configuration_error;
    assign command_ready=!rst&&!fault&&state==IDLE;
    stereo_snapshot_writer #(.QUEUE_BITS(5),.CAPTURE_TIMEOUT(CAPTURE_TIMEOUT),.BUS_TIMEOUT(BUS_TIMEOUT))writer(
        .clk(clk),.rst(rst),.tap_valid(tap_valid),.tap_data(tap_data),
        .arm_valid(state==CAP_ARM&&!fault),.arm_ready(arm_ready),.arm_camera(camera),.arm_session(session),.arm_tag(tag),.cancel(fault),
        .capture_done(capture_done),.capture_success(capture_success),.capture_reason(capture_reason),
        .lease_acquire(state==LEASE_CHECK&&lease_ready&&!fault),.lease_ready(lease_ready),
        .lease_epoch_A(expected_A),.lease_epoch_B(expected_B),.lease_tag(tag),
        .lease_release(state==RELEASE_CHECK&&release_ok&&!fault),.release_tag(tag),
        .snapshot_valid(snapshot_valid),.lease_locked(lease_locked),.epoch_A(epoch_A),.epoch_B(epoch_B),
        .frame_A(frame_A),.frame_B(frame_B),.session_A(session_A),.session_B(session_B),.tag_A(tag_A),.tag_B(tag_B),
        .busy(writer_busy),.configuration_error(writer_config),
        .awaddr(cap_awaddr),.awvalid(cap_awvalid),.awready(cap_awready),.wdata(cap_wdata),.wstrb(cap_wstrb),
        .wvalid(cap_wvalid),.wlast(cap_wlast),.wready(cap_wready),.araddr(cap_araddr),.arvalid(cap_arvalid),.arready(cap_arready),
        .rdata(cap_rdata),.rvalid(cap_rvalid),.rlast(cap_rlast),.rready(cap_rready),.read_active(cap_read_active),.fault(writer_fault));
    snapshot_row_reader #(.TIMEOUT(ROW_TIMEOUT))row_reader(
        .clk(clk),.rst(rst),.request_valid(state==ROW_REQ&&!fault&&row<720&&lease_locked==3&&snapshot_valid==3&&current_epochs),.request_ready(row_ready),
        .request_camera(camera),.request_row(row[9:0]),.request_epoch(camera?expected_B:expected_A),.request_tag(tag),
        .lease_valid(lease_locked),.lease_epoch_A(epoch_A),.lease_epoch_B(epoch_B),
        .araddr(feat_araddr),.arvalid(feat_arvalid),.arready(feat_arready),.rdata(feat_rdata),.rvalid(feat_rvalid),
        .rlast(feat_rlast),.rready(feat_rready),.read_active(feat_read_active),
        .ram_write(ram_write),.ram_address(ram_address),.ram_data(ram_data),.result_done(row_done),.result_good(row_good),
        .result_camera(row_camera),.result_row(row_number),.result_epoch(row_epoch),.result_tag(row_tag),.result_crc32(row_crc_result),
        .result_error(row_error),.fault(row_fault),.configuration_error(row_config));
    snapshot_feature_aux_mux mux(
        .clk(clk),.rst(rst),.capture_claim(capture_claim),.feature_claim(row_claim),
        .cap_awaddr(cap_awaddr),.cap_araddr(cap_araddr),.cap_wdata(cap_wdata),.cap_wstrb(cap_wstrb),
        .cap_awvalid(cap_awvalid),.cap_wvalid(cap_wvalid),.cap_wlast(cap_wlast),.cap_arvalid(cap_arvalid),
        .cap_rready(cap_rready),.cap_read_active(cap_read_active),.cap_awready(cap_awready),.cap_wready(cap_wready),
        .cap_arready(cap_arready),.cap_rvalid(cap_rvalid),.cap_rlast(cap_rlast),.cap_rdata(cap_rdata),
        .feat_araddr(feat_araddr),.feat_arvalid(feat_arvalid),.feat_rready(feat_rready),.feat_read_active(feat_read_active),
        .feat_arready(feat_arready),.feat_rvalid(feat_rvalid),.feat_rlast(feat_rlast),.feat_rdata(feat_rdata),
        .awaddr(awaddr),.araddr(araddr),.wdata(wdata),.wstrb(wstrb),.awvalid(awvalid),.wvalid(wvalid),.wlast(wlast),
        .arvalid(arvalid),.rready(rready),.read_active(read_active),.awready(awready),.wready(wready),.arready(arready),
        .rvalid(rvalid),.rlast(rlast),.rdata(rdata),.fault(mux_fault),.selected(mux_selected));
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;done<=0;response<=0;sticky_fault<=0;opcode<=0;error_code<=0;
            camera<=0;row<=0;session<=0;expected_A<=0;expected_B<=0;tag<=0;lease_owner<=0;row_crc<=0;
        end else begin
            done<=0;if(fault)sticky_fault<=1;
            if(!fault)case(state)
                IDLE:if(command_valid&&command_ready)begin
                    opcode<=command[159:152];camera<=command[151];row<=command[150:135];session<=command[134:103];
                    expected_A<=command[102:71];expected_B<=command[70:39];tag<=command[38:7];error_code<=0;row_crc<=0;
                    case(command[159:152])
                        1:state<=CAP_ARM;2:state<=LEASE_CHECK;3:state<=RELEASE_CHECK;4:state<=ROW_REQ;
                        default:begin error_code<=8'h10;state<=FINISH;end
                    endcase
                end
                CAP_ARM:begin
                    if(lease_locked[camera])begin error_code<=8'h11;state<=FINISH;end
                    else if(arm_ready)state<=CAP_WAIT;
                end
                CAP_WAIT:if(capture_done)begin
                    if(!capture_success)error_code<=8'h20|{4'd0,capture_reason};state<=FINISH;
                end
                LEASE_CHECK:begin
                    if(lease_ready)begin lease_owner<=tag;state<=LEASE_WAIT;end
                    else begin error_code<=8'h12;state<=FINISH;end
                end
                LEASE_WAIT:if(lease_locked==3)state<=FINISH;
                RELEASE_CHECK:begin
                    if(release_ok)state<=RELEASE_WAIT;else begin error_code<=8'h13;state<=FINISH;end
                end
                RELEASE_WAIT:if(lease_locked==0)state<=FINISH;
                ROW_REQ:begin
                    if(row>=720)begin error_code<=8'h14;state<=FINISH;end
                    else if(lease_locked!=3||snapshot_valid!=3||!current_epochs)begin error_code<=8'h15;state<=FINISH;end
                    else if(row_ready)state<=ROW_WAIT;
                end
                ROW_WAIT:if(row_done)begin
                    row_crc<=row_crc_result;
                    if(!row_good||row_camera!=camera||row_number!=row[9:0]||row_tag!=tag||
                        row_epoch!=(camera?expected_B:expected_A)||!current_epochs||lease_locked!=3)
                        error_code<=8'h30|{4'd0,row_error};
                    state<=FINISH;
                end
                FINISH:begin
                    // LE words: header, request-tag, epochs/row-epoch+CRC,
                    // frameA,frameB,sessionA,sessionB. All frozen before ACK.
                    response[31:0]<={lease_locked,snapshot_valid,row[9:0],camera,1'b0,error_code,opcode};
                    response[63:32]<=tag;response[95:64]<=opcode==4?(camera?expected_B:expected_A):epoch_A;
                    response[127:96]<=opcode==4?row_crc:epoch_B;
                    response[159:128]<=frame_A;response[191:160]<=frame_B;
                    response[223:192]<=session_A;response[255:224]<=session_B;
                    done<=1;state<=IDLE;
                end
                default:begin sticky_fault<=1;end
            endcase
        end
    end
endmodule
