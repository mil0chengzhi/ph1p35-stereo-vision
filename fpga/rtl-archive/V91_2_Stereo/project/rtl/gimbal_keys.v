// One bounded sequence per reset. SRAM commissioning only; no automatic start.
// KEY1 start after release/debounce; KEY2/KEY3 cancel retaining hold.
// KEY4 resets and releases holding torque. CN1 then full 5s pause then CN2.
module gimbal_keys #(
    parameter integer CLK_HZ=50000000,
    parameter integer UART_DIV=434,
    parameter integer POWERUP_CYCLES=CLK_HZ/50,
    parameter integer DEBOUNCE_CYCLES=CLK_HZ/100,
    parameter integer STEP_HZ=200,
    parameter integer PULSES=44,
    parameter integer GAP_TICKS=5*CLK_HZ,
    parameter integer TELEMETRY_TICKS=CLK_HZ/20
)(input wire clk, input wire [3:0] key_n,
  output wire [63:0] diagnostic, output wire step1,dir1,en1,step2,dir2,en2,
  output wire [3:0] led);
    reg [31:0] powerup_count=0;
    always @(posedge clk or negedge key_n[3]) begin
        if(!key_n[3]) powerup_count<=0;
        else if(powerup_count<POWERUP_CYCLES) powerup_count<=powerup_count+1'b1;
    end
    wire reset_request_n=key_n[3] && powerup_count==POWERUP_CYCLES;
    reg [1:0] reset_sync=0;
    always @(posedge clk or negedge reset_request_n) begin
        if(!reset_request_n) reset_sync<=0;
        else reset_sync<={reset_sync[0],1'b1};
    end
    wire rst_n=reset_sync[1];
    wire key_start,uart_start,uart_stop;
    gimbal_press_event #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) key_start_event
        (.clk(clk),.rst_n(rst_n),.key_n(key_n[0]),.pressed(key_start));
    reg [1:0] stop2_sync,stop3_sync;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin stop2_sync<=0;stop3_sync<=0;end
        else begin stop2_sync<={stop2_sync[0],~key_n[1]};stop3_sync<={stop3_sync[0],~key_n[2]};end
    end
    wire [3:0] phase;
    wire [15:0] count1,count2,last_token;
    wire [31:0] gap_ticks;
    wire consumed,busy1,busy2;
    seq_control #(.CLK_HZ(CLK_HZ),.STEP_HZ(STEP_HZ),.PULSES(PULSES),.GAP_TICKS(GAP_TICKS)) sequence_ctrl
        (.clk(clk),.rst_n(rst_n),.start(key_start|uart_start),
         .stop(stop2_sync[1]|stop3_sync[1]|uart_stop),
         .step1(step1),.dir1(dir1),.en1(en1),.step2(step2),.dir2(dir2),.en2(en2),
         .phase(phase),.count1(count1),.count2(count2),.consumed(consumed),
         .gap_ticks(gap_ticks),.busy1(busy1),.busy2(busy2));
    wire active=phase>=1 && phase<=5;
    wire [7:0] flags={1'b0,phase==7,consumed,busy2,busy1,en2,en1,active};
    assign uart_start=1'b0;assign uart_stop=1'b0;
    assign diagnostic={16'b0,8'b0,flags,count2,count1};
    assign led={phase==6,active,en2,en1};
endmodule
