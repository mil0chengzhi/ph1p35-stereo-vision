// Native720 coordinates; calibrated masks are losslessly encoded row runs.
// Coordinate arithmetic remains the independently checked Q6->Q5 core.
module hd_sparse_map_rom #(
 parameter WIDTH=1280,HEIGHT=720,GRID_LOG2=5,TAG_WIDTH=80,RUNS=1,
 parameter NODE_FILE="nodes.mem",MASK_FILE="row_runs.mem"
)(
 input wire clk,rst,in_valid,output wire in_ready,
 input wire in_camera,input wire[10:0]in_x,in_y,input wire[TAG_WIDTH-1:0]in_tag,
 output wire configuration_error,out_valid,input wire out_ready,
 output wire out_camera,out_usable,output wire signed[17:0]out_x_q5,out_y_q5,
 output wire[TAG_WIDTH-1:0]out_tag
);
 localparam NX=((WIDTH-1)>>GRID_LOG2)+2,NY=((HEIGHT-1)>>GRID_LOG2)+2;
 localparam NB=$clog2(2*NX*NY),MB=$clog2(2*WIDTH*HEIGHT),RB=$clog2(2*HEIGHT);
 reg[35:0]nodes[0:2*NX*NY-1];reg[22*RUNS-1:0]spans[0:2*HEIGHT-1];
 reg[35:0]node_data;reg[22*RUNS-1:0]span_data;
 reg[RB-1:0]span_address;reg[10:0]saved_x,mask_x;
 wire node_en,mask_en;wire[NB-1:0]node_address;wire[MB-1:0]unused_mask_address;
 reg mask_data;integer i;
 initial begin $readmemh(NODE_FILE,nodes);$readmemh(MASK_FILE,spans);end
 always @(posedge clk)begin
  if(in_valid&&in_ready)begin span_address<=(in_camera?HEIGHT:0)+in_y;saved_x<=in_x;end
  if(node_en)node_data<=nodes[node_address];
  if(mask_en)begin span_data<=spans[span_address];mask_x<=saved_x;end
 end
 always @*begin
  mask_data=0;
  for(i=0;i<RUNS;i=i+1)if(mask_x>=span_data[i*22+:11]&&mask_x<=span_data[i*22+11+:11])mask_data=1;
 end
 sparse_map_coordinate #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.GRID_LOG2(GRID_LOG2),.TAG_WIDTH(TAG_WIDTH))coordinates(
  .clk(clk),.rst(rst),.in_valid(in_valid),.in_ready(in_ready),.in_camera(in_camera),.in_x(in_x),.in_y(in_y),.in_tag(in_tag),
  .node_en(node_en),.node_addr(node_address),.node_data(node_data),.mask_en(mask_en),.mask_addr(unused_mask_address),.mask_data(mask_data),
  .configuration_error(configuration_error),.out_valid(out_valid),.out_ready(out_ready),.out_camera(out_camera),.out_usable(out_usable),
  .out_x_q5(out_x_q5),.out_y_q5(out_y_q5),.out_tag(out_tag));
endmodule
