// Clause 22 MDIO engine. All sequential logic uses clk_50; MDC is a clock-enable
// controlled output, never an internal fabric clock. Read data is synchronized.
// 32 preamble + ST + OP + PHYAD + REGAD + TA + 16 data bits. No PHY writes are
// performed without an explicit request from the diagnostic sequencer.
module db_mdio_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer MDC_HZ = 1000000
) (
    input wire clk_50, input wire rst_n,
    input wire start, input wire write_enable,
    input wire [4:0] phy_addr, input wire [4:0] reg_addr,
    input wire [15:0] write_data,
    output reg busy, output reg done, output reg read_error,
    output reg [15:0] read_data,
    output reg eth_mdc, inout wire eth_mdio
);
    localparam integer HALF_CYCLES = CLK_HZ / (2 * MDC_HZ);
    localparam integer DIV_BITS = (HALF_CYCLES > 1) ? $clog2(HALF_CYCLES) : 1;
    reg [DIV_BITS-1:0] divider;
    reg [63:0] shift_out;
    reg [5:0] bit_index;
    reg read_transaction;
    reg mdio_oe;
    (* ASYNC_REG = "TRUE" *) reg [1:0] mdio_sync;
    assign eth_mdio = mdio_oe ? shift_out[63] : 1'bz;

    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) mdio_sync <= 2'b11;
        else mdio_sync <= {mdio_sync[0], eth_mdio};
    end

    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; done <= 1'b0; read_error <= 1'b0;
            read_data <= 16'd0; divider <= 0; eth_mdc <= 1'b0;
            shift_out <= 64'hffffffffffffffff; bit_index <= 6'd63;
            read_transaction <= 1'b0; mdio_oe <= 1'b0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                eth_mdc <= 1'b0; divider <= 0; mdio_oe <= 1'b0;
                if (start) begin
                    busy <= 1'b1; bit_index <= 6'd63;
                    read_transaction <= !write_enable;
                    read_error <= 1'b0; read_data <= 16'd0;
                    shift_out <= {32'hffffffff, 2'b01,
                                  (write_enable ? 2'b01 : 2'b10),
                                  phy_addr, reg_addr, 2'b10, write_data};
                    mdio_oe <= 1'b1;
                end
            end else if (divider == HALF_CYCLES-1) begin
                divider <= 0;
                if (!eth_mdc) begin
                    eth_mdc <= 1'b1;
                    if (read_transaction && bit_index == 6'd16)
                        read_error <= mdio_sync[1]; // PHY must drive TA=0.
                    if (read_transaction && bit_index < 6'd16)
                        read_data <= {read_data[14:0], mdio_sync[1]};
                end else begin
                    eth_mdc <= 1'b0;
                    if (bit_index == 0) begin
                        busy <= 1'b0; done <= 1'b1; mdio_oe <= 1'b0;
                    end else begin
                        bit_index <= bit_index - 1'b1;
                        shift_out <= {shift_out[62:0], 1'b1};
                        if (read_transaction && bit_index == 6'd18)
                            mdio_oe <= 1'b0; // release both turnaround bits.
                    end
                end
            end else divider <= divider + 1'b1;
        end
    end
endmodule
