module mcu_auto_exposure #(
 parameter FIRMWARE_FILE="",
 parameter SAMPLES=2700,LOW=105,HIGH=130,SETTLE_FRAMES=6,
 parameter VERIFY_DELAY=5000000,COMMAND_TIMEOUT=1500000,STALE_CYCLES=50000000,
 parameter BOOT_HALF=3991,MAX_HALF=3991,MIN_HALF=32,FLICKER_HALF=0,
 parameter RUNTIME_TARGET=0,HIGHLIGHT_DIV=64,DARK_HIGHLIGHT_DIV=128,
 parameter MAX_GAIN_COARSE=8'h0b,MAX_GAIN_FINE=8'h80
)(
 input wire clk,rst_n,enable,boot_ready,
 input wire [1:0] target_mode,
 input wire meter_valid,input wire [19:0] meter_sum,input wire [11:0] meter_count,meter_highlights,
 input wire [15:0] meter_sequence,
 output wire request,input wire grant,output wire cmd_valid,input wire cmd_ready,
 output wire cmd_read,output wire [15:0] cmd_address,output wire [7:0] cmd_value,
 input wire cmd_done,input wire [1:0] cmd_error,input wire [7:0] read_data,
 output wire busy,hold_motion,output wire fault,stable,at_limit,stale,
 output wire [7:0] fault_code,output wire [11:0] half_lines,
 output wire [7:0] gain_coarse,gain_fine,output wire [15:0] updates,
 output wire [3:0] settling,output wire[2:0]debug_state,output wire[3:0]debug_step,output wire debug_initialized
);
 wire compute_request,compute_ready,compute_done,compute_fault;
 wire control_fault,control_hold_motion,control_stable;wire[7:0]control_fault_code;
 assign fault=control_fault||compute_fault;
 assign hold_motion=control_hold_motion||compute_fault;
 assign fault_code=compute_fault?8'h06:control_fault_code;
 assign stable=control_stable&&!compute_fault;

 wire[31:0]compute_result;wire cpu_busy,cpu_boot;wire[15:0]cpu_ticket,cpu_completed;wire[31:0]cpu_fetch;
 mcu_math_service #(.FIRMWARE_FILE(FIRMWARE_FILE))arithmetic(.clk(clk),.rst_n(rst_n),.request(compute_request),
 .exposure({gain_fine,gain_coarse,4'd0,half_lines}),.meter_sum(meter_sum),.highlights(meter_highlights),.target_mode(target_mode),
 .ready(compute_ready),.busy(cpu_busy),.done(compute_done),.fault(compute_fault),.boot_seen(cpu_boot),.result(compute_result),.ticket(cpu_ticket),.completed(cpu_completed),.last_fetch(cpu_fetch));
 auto_exposure_mcu_control #(.SAMPLES(SAMPLES),.LOW(LOW),.HIGH(HIGH),.SETTLE_FRAMES(SETTLE_FRAMES),.VERIFY_DELAY(VERIFY_DELAY),.COMMAND_TIMEOUT(COMMAND_TIMEOUT),.STALE_CYCLES(STALE_CYCLES),.BOOT_HALF(BOOT_HALF),.MAX_HALF(MAX_HALF),.MIN_HALF(MIN_HALF),.FLICKER_HALF(FLICKER_HALF),.RUNTIME_TARGET(RUNTIME_TARGET),.HIGHLIGHT_DIV(HIGHLIGHT_DIV),.DARK_HIGHLIGHT_DIV(DARK_HIGHLIGHT_DIV),.MAX_GAIN_COARSE(MAX_GAIN_COARSE),.MAX_GAIN_FINE(MAX_GAIN_FINE))control(
 .clk(clk),.rst_n(rst_n),.enable(enable&&!compute_fault),.boot_ready(boot_ready),.target_mode(target_mode),.meter_valid(meter_valid),.meter_sum(meter_sum),.meter_count(meter_count),.meter_highlights(meter_highlights),.meter_sequence(meter_sequence),.request(request),.grant(grant),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_read(cmd_read),.cmd_address(cmd_address),.cmd_value(cmd_value),.cmd_done(cmd_done),.cmd_error(cmd_error),.read_data(read_data),.busy(busy),.hold_motion(control_hold_motion),.fault(control_fault),.stable(control_stable),.at_limit(at_limit),.stale(stale),.fault_code(control_fault_code),.half_lines(half_lines),.gain_coarse(gain_coarse),.gain_fine(gain_fine),.updates(updates),.settling(settling),.debug_state(debug_state),.debug_step(debug_step),.debug_initialized(debug_initialized),
 .compute_request(compute_request),.compute_ready(compute_ready),.compute_done(compute_done),.compute_fault(compute_fault),.compute_result(compute_result));
endmodule
