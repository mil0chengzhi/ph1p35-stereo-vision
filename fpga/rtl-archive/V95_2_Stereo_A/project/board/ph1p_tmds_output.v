`timescale 1ns/1ps
// 功能：按UG1310的HR OSERDES(DDRX5)及差分输出原语发送三个TMDS通道和随路时钟。
// 输入pixel/serial为同PLL的1:5时钟；reset高有效；ch0..2每像素10位，最低位先出。
// 输出data_p/n各3位，clock_p/n各1位，名称表示FPGA封装P/N端；采用TMDS33。
// HX1P35B_FT_40P_2放大核对：H14(IO_R10P)接HDMI_D2_P，H13(IO_R10N)接HDMI_D2_N。
// 三个通道及随路时钟均P/N直连，不能额外反相；核对截图见docs/vendor_extracts/hdmi_pin_verified.png。
// 无参数/背压；串行物理延迟由器件决定，后续仿真与实现检查确认字边界。
module ph1p_tmds_output(
    input wire pixel,input wire serial,input wire reset,
    input wire [9:0] ch0,input wire [9:0] ch1,input wire [9:0] ch2,
    output wire [2:0] data_p,output wire [2:0] data_n,
    output wire clock_p,output wire clock_n
);
// OBUF_DIFF的t为高有效输出使能；TD映射TSMUX=INV，t=0会令物理PAD三态。
    wire [3:0] serial_data;
    PH1P_LOGIC_HR_OSERDES #(.ODDRMODE("DDRX5")) ser0(.sclk(serial),.pclk(pixel),.rst(reset),.idata(ch0),.odata(serial_data[0]));
    PH1P_LOGIC_HR_OSERDES #(.ODDRMODE("DDRX5")) ser1(.sclk(serial),.pclk(pixel),.rst(reset),.idata(ch1),.odata(serial_data[1]));
    PH1P_LOGIC_HR_OSERDES #(.ODDRMODE("DDRX5")) ser2(.sclk(serial),.pclk(pixel),.rst(reset),.idata(ch2),.odata(serial_data[2]));
    PH1P_LOGIC_HR_OSERDES #(.ODDRMODE("DDRX5")) serc(.sclk(serial),.pclk(pixel),.rst(reset),.idata(10'b1111100000),.odata(serial_data[3]));
    PH1P_LOGIC_OBUF_DIFF ob0(.i(serial_data[0]),.t(1'b1),.o_p(data_p[0]),.o_n(data_n[0]));
    PH1P_LOGIC_OBUF_DIFF ob1(.i(serial_data[1]),.t(1'b1),.o_p(data_p[1]),.o_n(data_n[1]));
    PH1P_LOGIC_OBUF_DIFF ob2(.i(serial_data[2]),.t(1'b1),.o_p(data_p[2]),.o_n(data_n[2]));
    PH1P_LOGIC_OBUF_DIFF obc(.i(serial_data[3]),.t(1'b1),.o_p(clock_p),.o_n(clock_n));
endmodule
