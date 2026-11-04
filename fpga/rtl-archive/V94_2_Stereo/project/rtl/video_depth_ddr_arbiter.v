// Isolated candidate: video clients0/1 retain priority; client2 is best effort.
// Read commands are registered before reaching the MC, so a newly arriving video
// request cannot retract an already posted MC AR command during auxiliary yield.
module video_depth_ddr_arbiter #(
    parameter integer WRITE_DRAIN=128,TIMEOUT=200000,MAX_OUTSTANDING=16,
    parameter integer AUX_MAX_READS=4,AUX_MAX_CYCLES=4096,AUX_GAP=8
)(
    input wire clk,rst_n,enable,
    input wire[74:0]c_awaddr,c_araddr,
    input wire[2:0]c_awvalid,c_wvalid,c_wlast,c_arvalid,c_rready,c_read_active,
    input wire[383:0]c_wdata,input wire[47:0]c_wstrb,
    output reg[2:0]c_awready,c_wready,c_arready,c_rvalid,c_rlast,
    output wire[383:0]c_rdata,
    output reg[24:0]awaddr,araddr,output reg[127:0]wdata,output reg[15:0]wstrb,
    output reg awvalid,wvalid,wlast,arvalid,rready,
    input wire awready,wready,arready,input wire[127:0]rdata,input wire rvalid,rlast,
    output reg fault,output wire configuration_error,output wire[31:0]debug,output wire transaction_read_active
);
    localparam CONFIG_OK=WRITE_DRAIN>=1&&WRITE_DRAIN<=65536&&TIMEOUT>AUX_MAX_CYCLES+WRITE_DRAIN&&
        MAX_OUTSTANDING>=1&&MAX_OUTSTANDING<=16&&AUX_MAX_READS>=1&&AUX_MAX_READS<=16&&
        AUX_MAX_CYCLES>WRITE_DRAIN+4&&AUX_GAP>=0&&AUX_GAP<=65535;
    localparam IDLE=0,WRITE=1,DRAIN=2,READ=3;
    reg[1:0]state,owner;
    reg prefer_read,prefer_write,aw_done,w_done,command_valid;
    reg aux_write_turn;
    reg[15:0]drain,gap;
    reg[5:0]video_quiet;
    reg[9:0]aux_wait;
    reg[31:0]age;
    reg[5:0]outstanding,memory_outstanding;
    reg[4:0]aux_reads;
    reg[24:0]command_address;
    wire video_read_pending=|c_arvalid[1:0];
    wire video_write_pending=|(c_awvalid[1:0]|c_wvalid[1:0]);
    wire video_pending=video_read_pending||video_write_pending;
    wire aux_limit=aux_reads>=AUX_MAX_READS||age>=AUX_MAX_CYCLES;
    // Once a quiet-slot auxiliary epoch has paid WRITE_DRAIN, commit at least
    // its first held request. Otherwise periodic video writes can repeatedly
    // preempt it after every drain and starve a read despite idle bandwidth.
    wire aux_admit=!aux_limit&&(aux_reads==0||!video_pending);
    wire client_take=c_arvalid[owner]&&c_arready[owner];
    wire memory_take=arvalid&&arready;
    wire return_take=rvalid&&rready;
    assign configuration_error=!CONFIG_OK;
    assign transaction_read_active=state==DRAIN||state==READ||command_valid||outstanding!=0;
    assign c_rdata={rdata,rdata,rdata};
    assign debug={10'b0,fault,command_valid,owner,state,outstanding,aux_reads,aw_done,w_done,prefer_read,prefer_write,1'b0};
    always @*begin
        c_awready=0;c_wready=0;c_arready=0;c_rvalid=0;c_rlast=0;
        awaddr=c_awaddr[owner*25+:25];wdata=c_wdata[owner*128+:128];wstrb=c_wstrb[owner*16+:16];
        araddr=command_address;awvalid=0;wvalid=0;wlast=0;arvalid=0;rready=fault;
        if(enable&&!fault&&CONFIG_OK)begin
            if(state==WRITE)begin
                awvalid=c_awvalid[owner]&&!aw_done;wvalid=c_wvalid[owner]&&!w_done;wlast=c_wlast[owner];
                c_awready[owner]=awready&&!aw_done;c_wready[owner]=wready&&!w_done;
            end
            if(state==READ)begin
                // A command already accepted from a client must reach the MC
                // even if higher priority work arrives while arready is low.
                arvalid=command_valid;
                if((owner!=2||aux_admit)&&outstanding<(owner==2?1:MAX_OUTSTANDING)&&(!command_valid||arready))
                    c_arready[owner]=1;
                if(memory_outstanding!=0)begin c_rvalid[owner]=rvalid;c_rlast[owner]=rlast;rready=c_rready[owner];end
            end
        end
    end
    always @(posedge clk)begin
        if(!rst_n)begin
            state<=IDLE;owner<=0;prefer_read<=0;prefer_write<=0;aw_done<=0;w_done<=0;
            command_valid<=0;command_address<=0;drain<=0;gap<=0;age<=0;outstanding<=0;memory_outstanding<=0;aux_reads<=0;fault<=0;aux_write_turn<=0;video_quiet<=0;aux_wait<=0;
        end else if(!enable)begin
            if(state!=IDLE||command_valid||outstanding!=0)fault<=1;
        end else if(!fault&&CONFIG_OK)begin
            if(gap!=0)gap<=gap-1'b1;
            // Prototype: transient FETCH gaps do not admit auxiliary AR.
            // Existing posted-AR, drain, ownership and write fairness stay.
            // An AR held for1023cycles may use the next slot without video requests.
            // A completed client command resets the age; no posted MC AR is withdrawn.
            if(!c_arvalid[2]||(client_take&&owner==2))aux_wait<=0;
            else if(!(&aux_wait))aux_wait<=aux_wait+1'b1;
            if(video_pending)video_quiet<=0;
            else if(video_quiet<32)video_quiet<=video_quiet+1'b1;
            if(state==IDLE)age<=0;
            else if(age==TIMEOUT-1)fault<=1;
            else age<=age+1'b1;
            if(rvalid&&(state!=READ||memory_outstanding==0))fault<=1;
            case(state)
                IDLE:begin
                    aw_done<=0;w_done<=0;command_valid<=0;outstanding<=0;memory_outstanding<=0;aux_reads<=0;
                    if(video_read_pending)begin
                        owner<=c_arvalid[prefer_read]?{1'b0,prefer_read}:{1'b0,!prefer_read};
                        drain<=WRITE_DRAIN-1;state<=DRAIN;
                    end else if(aux_write_turn&&gap==0&&(c_awvalid[2]||c_wvalid[2]))begin
                        // Keep video read priority. After an actual video write
                        // completes, admit one pending auxiliary write before
                        // another video write. No additional RAM or exception.
                        owner<=2;state<=WRITE;
                    end else if(video_write_pending)begin
                        owner<=(c_awvalid[prefer_write]||c_wvalid[prefer_write])?{1'b0,prefer_write}:{1'b0,!prefer_write};
                        state<=WRITE;
                    end else if(gap==0&&(video_quiet==32||(&aux_wait))&&c_arvalid[2])begin owner<=2;drain<=WRITE_DRAIN-1;state<=DRAIN;end
                    else if(gap==0&&(c_awvalid[2]||c_wvalid[2]))begin owner<=2;state<=WRITE;end
                end
                WRITE:begin
                    if(awvalid&&awready)aw_done<=1;
                    if(wvalid&&wready)begin w_done<=1;if(!wlast)fault<=1;end
                    if((aw_done||(awvalid&&awready))&&(w_done||(wvalid&&wready)))begin
                        state<=IDLE;
                        if(owner==2)begin gap<=0;aux_write_turn<=0;end
                        else begin
                            prefer_write<=!owner[0];
                            aux_write_turn<=c_awvalid[2]||c_wvalid[2];
                        end
                    end
                end
                DRAIN:if(drain==0)state<=READ;else drain<=drain-1'b1;
                READ:begin
                    if(client_take)begin
                        command_address<=c_araddr[owner*25+:25];command_valid<=1;
                        if(owner==2)aux_reads<=aux_reads+1'b1;
                    end else if(memory_take)command_valid<=0;
                    case({client_take,return_take})
                        2'b10:outstanding<=outstanding+1'b1;
                        2'b01:begin if(outstanding==0)fault<=1;else outstanding<=outstanding-1'b1;end
                        default:begin end
                    endcase
                    case({memory_take,return_take})
                        2'b10:memory_outstanding<=memory_outstanding+1'b1;
                        2'b01:begin if(memory_outstanding==0)fault<=1;else memory_outstanding<=memory_outstanding-1'b1;end
                        default:begin end
                    endcase
                    if(return_take&&!rlast)fault<=1;
                    if(!c_read_active[owner]&&outstanding!=0)fault<=1;
                    // No queued command or return may cross an ownership switch.
                    if(outstanding==0&&!command_valid&&!client_take&&
                        (!c_read_active[owner]||(owner==2&&((video_pending&&(aux_reads!=0||!c_arvalid[2]))||aux_limit))))begin
                        state<=IDLE;
                        if(owner==2)gap<=AUX_GAP;else prefer_read<=!owner[0];
                    end
                end
                default:begin fault<=1;state<=IDLE;end
            endcase
        end
    end
endmodule
