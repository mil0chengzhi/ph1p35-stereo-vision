// D36A single-axis bounded STEP/DIR/EN generator, Verilog-2001.
// Board EN is connected to SLEEP#: 1 = awake/holding, 0 = asleep.
// clk must match CLK_HZ. start, direction and halt MUST be synchronous to clk.
// Synchronize/debounce physical switches in the top level; do not connect them
// directly. A rising edge of start is accepted only while idle and !halt.
// Edges received while busy are discarded, never queued. Holding start high
// does not repeat a move. Keep start low while releasing reset.
//
// Reset: STEP=0, EN=0. start: latch DIR, set EN=1, wait WAKE_US, then
// produce exactly PULSES complete high/low cycles. EN stays 1 afterwards.
// halt: prohibit subsequent STEP rising edges; finish an existing high phase
// and its low phase before returning idle. done pulses only on normal finish.
// DIR stays unchanged throughout busy, including the final full low phase.
// Asynchronous reset overrides this pulse-width guarantee: it immediately
// lowers STEP and EN. EN=0 releases holding torque. Waking from sleep resets
// the driver's electrical phase; it is NOT an exact continuation of position.
//
// Driver limits: high/low >= 1 us; DIR setup/hold >= 200 ns; wake >= 1 ms.
// Valid parameters: CLK_HZ, STEP_HZ, PULSES > 0; WAKE_US >= 1000;
// HALF_TICKS must meet the minimum pulse width. Counters <= 2^31-1 ticks.
// The half-period rounds UP to whole clocks, so frequency never exceeds the
// request. Defaults at 50 MHz: 2 ms wait, 44 pulses at 200 Hz, 2.5 ms phases.
// 44 pulses are approximately 4.95 degrees ONLY for 1.8 deg + 1/16 stepping.
module d36a_step_burst #(
    parameter integer CLK_HZ  = 50000000,
    parameter integer STEP_HZ = 200,
    parameter integer PULSES  = 44,
    parameter integer WAKE_US = 2000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire direction,
    input  wire halt,
    output reg  step,
    output reg  dir,
    output reg  en,
    output reg  busy,
    output reg  done
);
    function integer bits_for;
        input integer value;
        integer v;
        begin
            v = value - 1;
            bits_for = 0;
            while (v > 0) begin
                bits_for = bits_for + 1;
                v = v >> 1;
            end
            if (bits_for < 1) bits_for = 1;
        end
    endfunction

    // Use 64-bit constant arithmetic to avoid overflow at 50 MHz * 2000 us.
    localparam [63:0] HALF_WIDE = (STEP_HZ > 0) ?
        (64'd1 * CLK_HZ + 64'd2 * STEP_HZ - 1) /
        (64'd2 * STEP_HZ) : 64'd1;
    localparam [63:0] WAKE_WIDE =
        (64'd1 * CLK_HZ * WAKE_US + 64'd999999) / 64'd1000000;
    localparam [63:0] MIN_PHASE_WIDE =
        (64'd1 * CLK_HZ + 64'd999999) / 64'd1000000;
    localparam integer HALF_TICKS = HALF_WIDE;
    localparam integer WAKE_TICKS = WAKE_WIDE;
    localparam integer MAX_TICKS =
        (HALF_TICKS > WAKE_TICKS) ? HALF_TICKS : WAKE_TICKS;
    localparam integer TIMER_BITS = bits_for(MAX_TICKS);
    localparam integer COUNT_BITS = bits_for(PULSES) + 1;

    localparam [1:0] IDLE = 2'd0, WAKE = 2'd1,
                     HIGH_PHASE = 2'd2, LOW_PHASE = 2'd3;
    reg [1:0] state;
    reg [TIMER_BITS-1:0] ticks_left;
    reg [COUNT_BITS-1:0] pulses_left;
    reg start_d;
    reg abort_pending;
    wire start_edge = start && !start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            ticks_left    <= 0;
            pulses_left   <= 0;
            start_d       <= 1'b0;
            abort_pending <= 1'b0;
            step          <= 1'b0;
            dir           <= 1'b0;
            en            <= 1'b0;
            busy          <= 1'b0;
            done          <= 1'b0;
        end else begin
            start_d <= start;
            done    <= 1'b0;
            case (state)
                IDLE: begin
                    step <= 1'b0;
                    busy <= 1'b0;
                    abort_pending <= 1'b0;
                    if (start_edge && !halt) begin
                        dir         <= direction;
                        en          <= 1'b1;
                        busy        <= 1'b1;
                        pulses_left <= PULSES;
                        ticks_left  <= WAKE_TICKS - 1;
                        state       <= WAKE;
                    end
                end
                WAKE: begin
                    if (halt) begin
                        busy  <= 1'b0;
                        state <= IDLE;
                    end else if (ticks_left == 0) begin
                        step       <= 1'b1;
                        ticks_left <= HALF_TICKS - 1;
                        state      <= HIGH_PHASE;
                    end else begin
                        ticks_left <= ticks_left - 1'b1;
                    end
                end
                HIGH_PHASE: begin
                    if (halt) abort_pending <= 1'b1;
                    if (ticks_left == 0) begin
                        step        <= 1'b0;
                        pulses_left <= pulses_left - 1'b1;
                        ticks_left  <= HALF_TICKS - 1;
                        state       <= LOW_PHASE;
                    end else begin
                        ticks_left <= ticks_left - 1'b1;
                    end
                end
                LOW_PHASE: begin
                    if (halt) abort_pending <= 1'b1;
                    if (ticks_left == 0) begin
                        if (abort_pending || halt) begin
                            busy  <= 1'b0;
                            state <= IDLE;
                        end else if (pulses_left == 0) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= IDLE;
                        end else begin
                            step       <= 1'b1;
                            ticks_left <= HALF_TICKS - 1;
                            state      <= HIGH_PHASE;
                        end
                    end else begin
                        ticks_left <= ticks_left - 1'b1;
                    end
                end
                default: begin
                    state <= IDLE;
                    step  <= 1'b0;
                    busy  <= 1'b0;
                    // Preserve EN: recovery must not release a held load.
                end
            endcase
        end
    end

    // Simulation-only validation; all runtime hardware above is synthesizable.
    // synthesis translate_off
    initial begin
        if (CLK_HZ <= 0 || STEP_HZ <= 0 || PULSES <= 0 ||
            WAKE_US < 1000 || HALF_WIDE < MIN_PHASE_WIDE ||
            HALF_WIDE > 64'd2147483647 || WAKE_WIDE == 0 ||
            WAKE_WIDE > 64'd2147483647) begin
            $display("ERROR: invalid d36a_step_burst parameters");
            $finish;
        end
    end
    // synthesis translate_on
endmodule
