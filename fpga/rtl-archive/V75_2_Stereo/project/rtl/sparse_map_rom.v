// Frozen two-camera table wrapper for isolated synthesis and simulation.
// The files are generated from an explicitly bound calibration or synthetic fixture.
module sparse_map_rom #(
    parameter integer WIDTH=320,HEIGHT=180,GRID_LOG2=3,TAG_WIDTH=32,
    parameter NODE_FILE="nodes.mem",MASK_FILE="masks.mem"
)(
    input wire clk,rst,input wire in_valid,output wire in_ready,
    input wire in_camera,input wire [10:0]in_x,in_y,
    input wire [TAG_WIDTH-1:0]in_tag,
    output wire configuration_error,
    output wire out_valid,input wire out_ready,
    output wire out_camera,out_usable,
    output wire signed [17:0]out_x_q5,out_y_q5,
    output wire [TAG_WIDTH-1:0]out_tag
);
    localparam NX=((WIDTH-1)>>GRID_LOG2)+2,NY=((HEIGHT-1)>>GRID_LOG2)+2;
    localparam NB=$clog2(2*NX*NY),MB=$clog2(2*WIDTH*HEIGHT);
    wire node_en,mask_en;wire[NB-1:0]node_addr;wire[MB-1:0]mask_addr;
    reg[35:0]nodes[0:2*NX*NY-1];reg masks[0:2*WIDTH*HEIGHT-1];
    reg[35:0]node_data;reg mask_data;
    initial begin $readmemh(NODE_FILE,nodes);$readmemh(MASK_FILE,masks);end
    always @(posedge clk)begin
        if(node_en)node_data<=nodes[node_addr];
        if(mask_en)mask_data<=masks[mask_addr];
    end
    sparse_map_coordinate #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.GRID_LOG2(GRID_LOG2),.TAG_WIDTH(TAG_WIDTH)) core(
        .clk(clk),.rst(rst),.in_valid(in_valid),.in_ready(in_ready),
        .in_camera(in_camera),.in_x(in_x),.in_y(in_y),.in_tag(in_tag),
        .node_en(node_en),.node_addr(node_addr),.node_data(node_data),
        .mask_en(mask_en),.mask_addr(mask_addr),.mask_data(mask_data),
        .configuration_error(configuration_error),.out_valid(out_valid),.out_ready(out_ready),
        .out_camera(out_camera),.out_usable(out_usable),.out_x_q5(out_x_q5),.out_y_q5(out_y_q5),.out_tag(out_tag));
endmodule
