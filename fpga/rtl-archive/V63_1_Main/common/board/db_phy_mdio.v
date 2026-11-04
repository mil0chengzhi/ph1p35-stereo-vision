// YT8511C diagnostic sequencer, 50 MHz clock-enable logic. Uses local datasheet
// pp.30,46-47,51-56: MII 2/3 ID, 0x11 link/speed, 0x1e/1f extended access.
// Only MDIO write is extended register address selection; no power, reset,
// speed or delay configuration is silently changed. 1G full duplex only.
module db_phy_mdio #(
    parameter integer CLK_HZ = 50000000,
    parameter integer MDC_HZ = 1000000,
    parameter integer POLL_CYCLES = 1000000,
    parameter [4:0] PHY_ADDR = 5'd4
) (
    input wire clk_50, input wire rst_n,
    output wire eth_mdc, inout wire eth_mdio,
    output reg [31:0] phy_id32,
    output reg [15:0] phy_status16,
    output reg [15:0] phy_delay16,
    output reg mdio_done, output reg mdio_error,
    output wire phy_ready
);
    localparam integer WAIT_BITS = (POLL_CYCLES > 1) ? $clog2(POLL_CYCLES) : 1;
    localparam [1:0] ISSUE=0, WAIT_DONE=1, POLL_WAIT=2;
    reg [1:0] state;
    reg [2:0] step;
    reg [WAIT_BITS-1:0] poll_count;
    reg start;
    wire busy, done, error;
    wire [15:0] read_data;
    reg [4:0] reg_addr;
    reg write_enable;
    always @* begin
        write_enable = 1'b0;
        case (step)
            0: reg_addr = 5'h02;
            1: reg_addr = 5'h03;
            2: begin reg_addr = 5'h1e; write_enable = 1'b1; end
            3: reg_addr = 5'h1f;
            default: reg_addr = 5'h11;
        endcase
    end
    db_mdio_master #(.CLK_HZ(CLK_HZ), .MDC_HZ(MDC_HZ)) master (
        .clk_50(clk_50), .rst_n(rst_n), .start(start),
        .write_enable(write_enable), .phy_addr(PHY_ADDR), .reg_addr(reg_addr),
        .write_data(16'h000c), .busy(busy), .done(done), .read_error(error),
        .read_data(read_data), .eth_mdc(eth_mdc), .eth_mdio(eth_mdio)
    );
    assign phy_ready = rst_n && !mdio_error &&
                       phy_id32 != 32'd0 && phy_id32 != 32'hffffffff &&
                       phy_delay16[0] && phy_delay16[7:4] == 4'h5 &&
                       phy_status16[15:14] == 2'b10 &&
                       phy_status16[13] && phy_status16[11] && phy_status16[10];
    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
            state <= ISSUE; step <= 0; poll_count <= 0; start <= 0;
            phy_id32 <= 0; phy_status16 <= 0; phy_delay16 <= 0;
            mdio_done <= 0; mdio_error <= 0;
        end else begin
            start <= 0; mdio_done <= 0;
            case (state)
                ISSUE: if (!busy) begin start <= 1; state <= WAIT_DONE; end
                WAIT_DONE: if (done) begin
                    if (error) mdio_error <= 1;
                    case (step)
                        0: phy_id32[31:16] <= read_data;
                        1: phy_id32[15:0] <= read_data;
                        3: phy_delay16 <= read_data;
                        4: begin phy_status16 <= read_data; mdio_done <= 1; end
                        default: begin end
                    endcase
                    if (step == 4) begin
                        state <= POLL_WAIT; poll_count <= 0; step <= 2;
                    end else begin step <= step + 1'b1; state <= ISSUE; end
                end
                POLL_WAIT: if (poll_count == POLL_CYCLES-1) state <= ISSUE;
                           else poll_count <= poll_count + 1'b1;
                default: state <= ISSUE;
            endcase
        end
    end
endmodule
