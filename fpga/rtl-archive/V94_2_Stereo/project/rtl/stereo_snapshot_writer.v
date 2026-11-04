// Experimental DDR-token tee. No ready/backpressure signal reaches the video tap.
// Captures one physical camera at a time into dedicated normalized snapshot slots.
// Camera0=A(peer720); camera1=B(local960 cropped120..839), for a future B board.
// Requires an external pAXI arbiter with write-drain protection before reads.
module stereo_snapshot_writer #(
    parameter integer WIDTH=1280,OUT_HEIGHT=720,
    parameter integer SOURCE_HEIGHT0=720,CROP_Y0=0,SOURCE_HEIGHT1=960,CROP_Y1=120,
    parameter integer QUEUE_BITS=4,CAPTURE_TIMEOUT=10000000,BUS_TIMEOUT=200000,
    parameter integer SNAPSHOT_BASE=12582912,SLOT_WORDS=2097152,
    parameter integer PROTECTED_END=12582912,ADDRESS_WORDS=33554432
)(
    input wire clk,rst,input wire[1:0]tap_valid,input wire[259:0]tap_data,
    input wire arm_valid,output wire arm_ready,input wire arm_camera,
    input wire[31:0]arm_session,arm_tag,input wire cancel,
    output reg capture_done,capture_success,output reg[3:0]capture_reason,
    input wire lease_acquire,output wire lease_ready,input wire[31:0]lease_epoch_A,lease_epoch_B,lease_tag,
    input wire lease_release,input wire[31:0]release_tag,
    output reg[1:0]snapshot_valid,lease_locked,
    output reg[31:0]epoch_A,epoch_B,frame_A,frame_B,session_A,session_B,tag_A,tag_B,
    output wire busy,configuration_error,
    output wire[24:0]awaddr,output wire awvalid,input wire awready,
    output wire[127:0]wdata,output wire[15:0]wstrb,output wire wvalid,wlast,input wire wready,
    output wire[24:0]araddr,output wire arvalid,input wire arready,
    input wire[127:0]rdata,input wire rvalid,rlast,output wire rready,read_active,
    output reg fault
);
    localparam ROW_BEATS=WIDTH/8,OUT_BEATS=ROW_BEATS*OUT_HEIGHT,DEPTH=1<<QUEUE_BITS;
    localparam CONFIG_OK=WIDTH>0&&WIDTH<=1280&&WIDTH%8==0&&OUT_HEIGHT>0&&OUT_HEIGHT<=720&&
        SOURCE_HEIGHT0>0&&SOURCE_HEIGHT0<=960&&SOURCE_HEIGHT1>0&&SOURCE_HEIGHT1<=960&&QUEUE_BITS>=1&&QUEUE_BITS<=8&&
        CROP_Y0>=0&&CROP_Y1>=0&&CROP_Y0+OUT_HEIGHT<=SOURCE_HEIGHT0&&CROP_Y1+OUT_HEIGHT<=SOURCE_HEIGHT1&&
        SNAPSHOT_BASE>=PROTECTED_END&&SNAPSHOT_BASE%8==0&&SLOT_WORDS>=WIDTH*OUT_HEIGHT&&SLOT_WORDS%8==0&&
        SNAPSHOT_BASE+2*SLOT_WORDS<=ADDRESS_WORDS&&ADDRESS_WORDS<=33554432&&CAPTURE_TIMEOUT>=8&&BUS_TIMEOUT>=8;
    localparam IDLE=0,ARMED=1,CAPTURE=2,DRAIN=3,ABORT=4;
    localparam B_IDLE=0,B_LOAD=1,B_WRITE=2,B_AR=3,B_DATA=4,B_QUARANTINE=5;
    reg[2:0]state,backend;
    reg camera,aw_done,w_done;
    reg[31:0]expected_session,request_tag,active_frame,active_session,lease_owner;
    reg[31:0]source_beats,crop_first,crop_end,received,enqueued,written,capture_age,bus_age;
    reg[24:0]base_address,last_address;
    reg[127:0]write_word,last_word;
    reg[127:0]queue[0:DEPTH-1],queue_q;
    reg[QUEUE_BITS-1:0]write_ptr,read_ptr;
    reg[QUEUE_BITS:0]queue_count;
    reg fence_done,fence_good;
    wire take_lease=lease_acquire&&lease_ready;
    assign configuration_error=!CONFIG_OK;
    assign busy=state!=IDLE;
    assign lease_ready=CONFIG_OK&&!rst&&!fault&&state==IDLE&&snapshot_valid==3&&lease_locked==0&&
        lease_epoch_A==epoch_A&&lease_epoch_B==epoch_B;
    assign arm_ready=CONFIG_OK&&!rst&&!fault&&state==IDLE&&backend==B_IDLE&&queue_count==0&&
        !lease_locked[arm_camera]&&!take_lease;
    wire take_arm=arm_valid&&arm_ready;
    wire current_valid=tap_valid[camera];
    wire[129:0]token=tap_data[camera*130+:130];
    wire[1:0]token_type=token[129:128];
    wire selected_word=received>=crop_first&&received<crop_end;
    reg abort_event;reg[3:0]abort_reason;
    always @*begin
        abort_event=0;abort_reason=0;
        if(state==ARMED||state==CAPTURE||state==DRAIN)begin
            if(cancel)begin abort_event=1;abort_reason=5;end
            else if(capture_age==CAPTURE_TIMEOUT-1)begin abort_event=1;abort_reason=6;end
            else if(fault)begin abort_event=1;abort_reason=7;end
            else if(state==ARMED&&current_valid&&token_type==1&&token[63:32]!=expected_session)begin
                abort_event=1;abort_reason=1;
            end else if(state==CAPTURE&&current_valid)begin
                if(token_type==1||token_type==3||(token_type==0&&received>=source_beats))begin
                    abort_event=1;abort_reason=2;
                end else if(token_type==0&&selected_word&&queue_count==DEPTH)begin
                    abort_event=1;abort_reason=3;
                end else if(token_type==2&&(!token[0]||token[32:1]!=active_frame||token[64:33]!=active_session||
                    received!=source_beats||enqueued!=OUT_BEATS))begin abort_event=1;abort_reason=4;end
            end
        end
    end
    wire push=state==CAPTURE&&current_valid&&token_type==0&&selected_word&&!abort_event;
    wire pop=backend==B_IDLE&&(state==CAPTURE||state==DRAIN)&&queue_count!=0&&!abort_event;
    always @(posedge clk)begin
        if(push)queue[write_ptr]<=token[127:0];
        if(pop)queue_q<=queue[read_ptr];
    end
    always @(posedge clk)begin
        if(rst)begin write_ptr<=0;read_ptr<=0;queue_count<=0;end
        else if(take_arm||abort_event||state==ABORT)begin write_ptr<=0;read_ptr<=0;queue_count<=0;end
        else begin
            if(push)write_ptr<=write_ptr+1'b1;
            if(pop)read_ptr<=read_ptr+1'b1;
            case({push,pop})2'b10:queue_count<=queue_count+1'b1;2'b01:queue_count<=queue_count-1'b1;default:begin end endcase
        end
    end
    always @(posedge clk)begin
        if(rst)begin
            state<=IDLE;camera<=0;expected_session<=0;request_tag<=0;active_frame<=0;active_session<=0;
            source_beats<=0;crop_first<=0;crop_end<=0;received<=0;enqueued<=0;capture_age<=0;base_address<=0;
            capture_done<=0;capture_success<=0;capture_reason<=0;snapshot_valid<=0;lease_locked<=0;lease_owner<=0;
            epoch_A<=0;epoch_B<=0;frame_A<=0;frame_B<=0;session_A<=0;session_B<=0;tag_A<=0;tag_B<=0;
        end else begin
            capture_done<=0;
            if(state==IDLE)capture_age<=0;else capture_age<=capture_age+1'b1;
            if(take_lease)begin lease_locked<=3;lease_owner<=lease_tag;end
            else if(lease_release&&lease_locked==3&&release_tag==lease_owner)lease_locked<=0;
            if(abort_event)begin state<=ABORT;capture_reason<=abort_reason;capture_success<=0;end
            else case(state)
                IDLE:if(take_arm)begin
                    state<=ARMED;camera<=arm_camera;expected_session<=arm_session;request_tag<=arm_tag;
                    base_address<=SNAPSHOT_BASE+(arm_camera?SLOT_WORDS:0);
                    source_beats<=(arm_camera?SOURCE_HEIGHT1:SOURCE_HEIGHT0)*ROW_BEATS;
                    crop_first<=(arm_camera?CROP_Y1:CROP_Y0)*ROW_BEATS;
                    crop_end<=((arm_camera?CROP_Y1:CROP_Y0)+OUT_HEIGHT)*ROW_BEATS;
                    received<=0;enqueued<=0;capture_success<=0;capture_reason<=0;
                    snapshot_valid[arm_camera]<=0;
                end
                ARMED:if(current_valid&&token_type==1)begin
                    active_frame<=token[31:0];active_session<=token[63:32];state<=CAPTURE;
                end
                CAPTURE:if(current_valid)begin
                    if(token_type==0)begin received<=received+1'b1;if(push)enqueued<=enqueued+1'b1;end
                    else if(token_type==2)state<=DRAIN;
                end
                DRAIN:if(fence_done)begin
                    if(fence_good&&!fault)begin
                        snapshot_valid[camera]<=1;
                        if(camera)begin epoch_B<=epoch_B+1'b1;frame_B<=active_frame;session_B<=active_session;tag_B<=request_tag;end
                        else begin epoch_A<=epoch_A+1'b1;frame_A<=active_frame;session_A<=active_session;tag_A<=request_tag;end
                        capture_done<=1;capture_success<=1;capture_reason<=0;state<=IDLE;
                    end else begin state<=ABORT;capture_success<=0;capture_reason<=7;end
                end
                ABORT:if(backend==B_IDLE&&queue_count==0)begin state<=IDLE;capture_done<=1;capture_success<=0;end
                default:begin state<=ABORT;capture_reason<=2;end
            endcase
        end
    end
    assign awaddr=base_address+{written[21:0],3'b000};
    assign awvalid=!rst&&backend==B_WRITE&&!aw_done;
    assign wvalid=!rst&&backend==B_WRITE&&!w_done;
    assign wdata=write_word;assign wstrb=0;assign wlast=1;
    assign araddr=last_address;assign arvalid=!rst&&backend==B_AR;
    assign rready=!rst&&(backend==B_DATA||backend==B_QUARANTINE||fault);
    assign read_active=!rst&&(backend==B_AR||backend==B_DATA||backend==B_QUARANTINE);
    always @(posedge clk)begin
        if(rst)begin
            backend<=B_IDLE;aw_done<=0;w_done<=0;write_word<=0;last_word<=0;last_address<=0;
            written<=0;bus_age<=0;fence_done<=0;fence_good<=0;fault<=0;
        end else begin
            fence_done<=0;
            if(backend==B_WRITE||backend==B_AR||backend==B_DATA)begin
                if(bus_age==BUS_TIMEOUT-1)fault<=1;else bus_age<=bus_age+1'b1;
            end else bus_age<=0;
            if(rvalid&&backend!=B_DATA&&backend!=B_QUARANTINE)fault<=1;
            case(backend)
                B_IDLE:begin
                    if(take_arm)written<=0;
                    if(pop)backend<=B_LOAD;
                    else if(state==DRAIN&&queue_count==0&&written==OUT_BEATS&&!abort_event&&!fence_done)backend<=B_AR;
                end
                B_LOAD:begin
                    if(state==ABORT||abort_event)backend<=B_IDLE;
                    else begin write_word<=queue_q;aw_done<=0;w_done<=0;backend<=B_WRITE;end
                end
                B_WRITE:begin
                    // Finish a posted AW/W pair even if the snapshot is cancelled.
                    if(awvalid&&awready)aw_done<=1;
                    if(wvalid&&wready)w_done<=1;
                    if((aw_done||(awvalid&&awready))&&(w_done||(wvalid&&wready)))begin
                        last_word<=write_word;last_address<=awaddr;written<=written+1'b1;backend<=B_IDLE;
                    end
                end
                B_AR:if(arvalid&&arready)backend<=B_DATA;
                B_DATA:if(rvalid&&rready)begin
                    if(!rlast)begin fault<=1;backend<=B_QUARANTINE;end
                    else begin fence_done<=1;fence_good<=rdata==last_word;backend<=B_IDLE;end
                end
                B_QUARANTINE:if(rvalid&&rready&&rlast)backend<=B_IDLE;
                default:begin fault<=1;backend<=B_IDLE;end
            endcase
        end
    end
endmodule
