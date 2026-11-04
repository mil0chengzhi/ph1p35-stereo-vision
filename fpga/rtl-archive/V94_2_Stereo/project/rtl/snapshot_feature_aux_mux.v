// One auxiliary arbiter client, two mutually exclusive masters.
// Ownership persists through queued ARs and split AW/W; claims alone never route
// a late response. A coordinator must claim before presenting a transaction.
module snapshot_feature_aux_mux(
    input wire clk,rst,input wire capture_claim,feature_claim,
    input wire[24:0]cap_awaddr,cap_araddr,input wire[127:0]cap_wdata,input wire[15:0]cap_wstrb,
    input wire cap_awvalid,cap_wvalid,cap_wlast,cap_arvalid,cap_rready,cap_read_active,
    output wire cap_awready,cap_wready,cap_arready,cap_rvalid,cap_rlast,output wire[127:0]cap_rdata,
    input wire[24:0]feat_araddr,input wire feat_arvalid,feat_rready,feat_read_active,
    output wire feat_arready,feat_rvalid,feat_rlast,output wire[127:0]feat_rdata,
    output wire[24:0]awaddr,araddr,output wire[127:0]wdata,output wire[15:0]wstrb,
    output wire awvalid,wvalid,wlast,arvalid,rready,read_active,
    input wire awready,wready,arready,rvalid,rlast,input wire[127:0]rdata,
    output reg fault,output wire[1:0]selected
);
    localparam IDLE=0,CAPTURE=1,FEATURE=2;
    reg[1:0]owner;
    reg aw_half,w_half;
    reg[5:0]pending;
    wire take_aw=awvalid&&awready,take_w=wvalid&&wready;
    wire take_ar=arvalid&&arready,take_r=rvalid&&rready;
    wire selected_active=owner==CAPTURE?cap_read_active:feat_read_active;
    assign selected=owner;
    assign awaddr=cap_awaddr;assign wdata=cap_wdata;assign wstrb=cap_wstrb;assign wlast=cap_wlast;
    assign araddr=owner==CAPTURE?cap_araddr:feat_araddr;
    assign awvalid=!rst&&owner==CAPTURE&&cap_awvalid;
    assign wvalid=!rst&&owner==CAPTURE&&cap_wvalid;
    assign arvalid=!rst&&((owner==CAPTURE&&cap_arvalid)||(owner==FEATURE&&feat_arvalid));
    assign rready=!rst&&((owner==CAPTURE&&cap_rready)||(owner==FEATURE&&feat_rready));
    assign read_active=!rst&&owner!=IDLE&&(selected_active||pending!=0);
    assign cap_awready=owner==CAPTURE&&!rst&&awready;
    assign cap_wready=owner==CAPTURE&&!rst&&wready;
    assign cap_arready=owner==CAPTURE&&!rst&&arready;
    assign feat_arready=owner==FEATURE&&!rst&&arready;
    assign cap_rvalid=owner==CAPTURE&&!rst&&rvalid&&pending!=0;
    assign feat_rvalid=owner==FEATURE&&!rst&&rvalid&&pending!=0;
    assign cap_rlast=rlast;assign feat_rlast=rlast;assign cap_rdata=rdata;assign feat_rdata=rdata;
    always @(posedge clk)begin
        if(rst)begin owner<=IDLE;aw_half<=0;w_half<=0;pending<=0;fault<=0;end
        else begin
            if(capture_claim&&feature_claim)fault<=1;
            if(rvalid&&(owner==IDLE||pending==0))fault<=1;
            if(owner!=IDLE&&pending!=0&&!selected_active)fault<=1;
            if(take_r&&!rlast)fault<=1;
            if(take_aw)aw_half<=1;
            if(take_w)begin w_half<=1;if(!wlast)fault<=1;end
            if((aw_half||take_aw)&&(w_half||take_w))begin aw_half<=0;w_half<=0;end
            case({take_ar,take_r})
                2'b10:begin if(pending==31)fault<=1;else pending<=pending+1'b1;end
                2'b01:begin if(pending==0)fault<=1;else pending<=pending-1'b1;end
                default:begin end
            endcase
            case(owner)
                IDLE:begin
                    if(cap_awvalid||cap_wvalid||cap_arvalid||feat_arvalid)begin
                        // Requests concurrent with their first claim are legal;
                        // they remain stalled until registered ownership commits.
                        if((cap_awvalid||cap_wvalid||cap_arvalid)&&!capture_claim)fault<=1;
                        if(feat_arvalid&&!feature_claim)fault<=1;
                    end
                    if(!fault&&!(capture_claim&&feature_claim))begin
                        if(capture_claim)owner<=CAPTURE;
                        else if(feature_claim)owner<=FEATURE;
                    end
                end
                CAPTURE:if(!capture_claim&&!cap_awvalid&&!cap_wvalid&&!cap_arvalid&&!cap_read_active&&
                    pending==0&&!aw_half&&!w_half&&!take_aw&&!take_w&&!take_ar)owner<=IDLE;
                FEATURE:if(!feature_claim&&!feat_arvalid&&!feat_read_active&&pending==0&&!take_ar)owner<=IDLE;
                default:begin fault<=1;owner<=IDLE;end
            endcase
        end
    end
endmodule
