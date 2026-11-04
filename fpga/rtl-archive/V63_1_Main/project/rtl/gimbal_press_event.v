// Debounced falling-edge key event. An initially held key cannot start motion.
module gimbal_press_event #(
    parameter integer STABLE_CYCLES = 500000
)(
    input wire clk, rst_n, key_n,
    output reg pressed
);
    reg [1:0] key_sync;
    reg [31:0] stable_count;
    reg armed;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) key_sync <= 2'b00;
        else key_sync <= {key_sync[0], key_n};
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stable_count <= 0;
            armed <= 0;
            pressed <= 0;
        end else begin
            pressed <= 0;
            if (!armed) begin
                if (!key_sync[1]) stable_count <= 0;
                else if (stable_count == STABLE_CYCLES - 1) begin
                    armed <= 1;
                    stable_count <= 0;
                end else stable_count <= stable_count + 1'b1;
            end else begin
                if (key_sync[1]) stable_count <= 0;
                else if (stable_count == STABLE_CYCLES - 1) begin
                    pressed <= 1;
                    armed <= 0;
                    stable_count <= 0;
                end else stable_count <= stable_count + 1'b1;
            end
        end
    end
endmodule
