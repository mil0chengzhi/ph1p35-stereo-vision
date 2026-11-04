`timescale 1ns/1ps
// Fixed-profile arithmetic worker. Actual MCU core=50MHz, reset AHB=25MHz.
// The FPGA bridge uses the same 50MHz clock as the core input, with two stages
// for AHB reads AND write metadata (one actual AHB cycle). Writes are level
// idempotent; completion is accepted only once for the currently owned ticket.
module mcu_math_service #(
 parameter FIRMWARE_FILE="",ROM_WORDS=1024,INITIAL_TICKET=0
)(
 input wire clk,rst_n,input wire request,input wire[31:0]exposure,
 input wire[31:0]box_rdata,output wire[3:0]box_read_word,box_write_word,
 output wire box_write,output wire[31:0]box_wdata,
 input wire[19:0]awb_red,awb_green,awb_blue,input wire[15:0]awb_sequence,
 input wire awb_valid,awb_confidence,awb_consumer_ready,
 output reg awb_done,output reg[19:0]awb_classification,
 input wire[19:0]meter_sum,input wire[11:0]highlights,input wire[1:0]target_mode,
 output wire ready,output reg busy,done,fault,boot_seen,
 output wire[31:0]result,output reg[15:0]ticket,completed,
 output reg[31:0]last_fetch
);
 localparam RB=$clog2(ROM_WORDS);
 (* ram_style="bram" *)reg[31:0]code[0:ROM_WORDS-1];
 (* ram_style="bram" *)reg[31:0]private_ram[0:255];
 initial $readmemh(FIRMWARE_FILE,code);
 wire core_reset;wire ahb_feedback,lock,freq_lock,core_sysrst;
 assign core_reset=!rst_n;
 wire[1:0]ht;wire hw;wire[30:0]ha;wire[31:0]wd;wire[2:0]hs,hb;wire[3:0]hp;wire hm;
 reg[31:0]rom_q,ram_q,mmio_q,rd;
 reg[1:0]region_q,valid_pipe,write_pipe,bad_pipe;
 reg[15:0]address0,address1;reg[2:0]size0,size1;
 reg[31:0]held_exposure,staged_result;reg[19:0]held_sum;
 reg[11:0]held_highlights;reg[1:0]held_mode;reg awb_ever_completed;
 assign result=staged_result;
 assign ready=!busy&&!fault;
 wire code_region=ha[30:14]==0;
 wire ram_region=ha[30:10]==21'h10;
 wire mmio_region=ha[30:7]==24'h100;
 // AHB read address and prior write-data phase have distinct word ports.
 assign box_read_word=ha[5:2];assign box_write_word=address1[5:2];assign box_wdata=wd;
 assign box_write=boot_seen&&!fault&&valid_pipe[1]&&write_pipe[1]&&!bad_pipe[1]&&
  size1==2&&address1[1:0]==0&&address1[15:6]==10'h201;
 // Both ERAM outputs are synchronous. The following rd register is the
 // second core-clock stage; do not collapse this into a single-stage ROM.
 always @(posedge clk)begin
  if(!rst_n||ht[1]&&!hw)rom_q<=code[!rst_n?{RB{1'b0}}:ha[RB+1:2]];
  if(ht[1]&&!hw)ram_q<=private_ram[ha[9:2]];
  if(!rst_n)begin region_q<=0;mmio_q<=0;end
  else if(ht[1]&&!hw)begin
   region_q<=code_region?0:ram_region?1:mmio_region?2:3;
   case(ha[6:2])
    0:mmio_q<={busy,15'd0,ticket};1:mmio_q<=held_exposure;
    2:mmio_q<={12'd0,held_sum};3:mmio_q<={14'd0,held_mode,4'd0,held_highlights};
    4:mmio_q<=32'h41454d31;5:mmio_q<=staged_result;
    6:mmio_q<={completed,ticket};7:mmio_q<={31'd0,boot_seen};
    8:mmio_q<={13'd0,awb_consumer_ready,awb_confidence,awb_valid,awb_sequence};
    9:mmio_q<={12'd0,awb_red};10:mmio_q<={12'd0,awb_green};11:mmio_q<={12'd0,awb_blue};
    12:mmio_q<={awb_classification[19:4],12'd0,awb_classification[3:0]};
    default:mmio_q<=ha[6]?box_rdata:32'd0;
   endcase
  end
  case(region_q)0:rd<=rom_q;1:rd<=ram_q;2:rd<=mmio_q;default:rd<=0;endcase
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   valid_pipe<=0;write_pipe<=0;bad_pipe<=0;address0<=0;address1<=0;size0<=0;size1<=0;
   held_exposure<=0;held_sum<=0;held_highlights<=0;held_mode<=0;staged_result<=0;
   awb_done<=0;awb_classification<=0;awb_ever_completed<=0;busy<=0;done<=0;fault<=0;boot_seen<=0;ticket<=INITIAL_TICKET;completed<=0;last_fetch<=0;
  end else begin
   done<=0;awb_done<=0;
   valid_pipe<={valid_pipe[0],ht[1]};write_pipe<={write_pipe[0],hw};
   bad_pipe<={bad_pipe[0],|ha[30:16]};address0<=ha[15:0];address1<=address0;size0<=hs;size1<=size0;
   if(ht[1]&&!hw&&!hp[0])last_fetch<={1'b0,ha};
   if(request&&ready)begin
    held_exposure<=exposure;held_sum<=meter_sum;held_highlights<=highlights;held_mode<=target_mode;
    ticket<=ticket+1'b1;busy<=1;
   end
   if(valid_pipe[1]&&write_pipe[1])begin
    if(bad_pipe[1]||size1!=2||address1[1:0]!=0)fault<=1;
    else if(address1[15:10]==6'h10)private_ram[address1[9:2]]<=wd;
    else if(address1[15:6]==10'h200)begin
     case(address1[5:2])
      5:staged_result<=wd;
      6:if(busy&&wd=={16'd0,ticket})begin busy<=0;done<=1;completed<=completed+1'b1;end
      7:if(wd==32'h4d435531)boot_seen<=1;else fault<=1;
      // AHB phase is repeated in board50; accept an AWB generation once.
      // Dynamic statistics reuse the ALREADY EXISTING coherent AWB snapshot.
      // A later statistics frame cancels an obsolete CPU write, never AE.
      12:if(wd[15:4]!=0)fault<=1;
         else if(awb_valid&&awb_confidence&&awb_consumer_ready&&wd[31:16]==awb_sequence&&
                 (!awb_ever_completed||wd[31:16]!=awb_classification[19:4]))begin
          awb_classification<={wd[31:16],wd[3:0]};awb_done<=1;awb_ever_completed<=1;
         end
      default:fault<=1;
     endcase
    end else if(address1[15:6]==10'h201)begin
     if(address1[5:2]<4||address1[5:2]>14)fault<=1;
    end else fault<=1;
   end
  end
 end
 // Real PH1P hard MCU; unused native GPIO outputs are left unconnected.
 PH1P_PHY_MCU #(.MCU_JTAG_SEL("1"),.MCU_RESET_VECTOR(32'h80000000),
  .MCU_DPLL_FEED_TYPE("EXTERNAL"),.MCU_DPLL_BYPASS("ENABLE"),.MCU_DPLL_POST_DIV(0),.AHB_FREQ("50"),.QSPI1_FREQ("20"))hard_mcu(
  .core_clk(clk),.timer_clk(clk),.ahb_clk(ahb_feedback),.fb_clk(ahb_feedback),
  .core_reset(core_reset),.por_reset(core_reset),.core_clk_en(1'b1),
  .dpll_freq_lock(freq_lock),.dpll_lock(lock),.core_sysrst(core_sysrst),
  .dbg_stop(1'b0),.dbg_stop_on_reset(1'b0),.dbg_stop_at_boot(1'b0),.dbg_mux_sel(2'd3),
  .nmi(1'b0),.clic_irq(23'd0),.jtag_tck(1'b0),.jtag_tms(1'b0),.jtag_tdi(1'b0),
  .gpio0_in(1'b0),.gpio1_in(1'b0),.gpio2_in(1'b0),.gpio3_in(1'b0),
  .gpio4_in(1'b0),.gpio5_in(1'b0),.gpio6_in(1'b0),.gpio7_in(1'b0),
  .uart0_rx(1'b1),.uart1_rx(1'b1),.i2c_sda_in(1'b1),.i2c_scl_in(1'b1),
  .qspi1_d0_in(1'b0),.qspi1_d1_in(1'b0),.qspi1_d2_in(1'b0),.qspi1_d3_in(1'b0),
  .qspi0_dw32_sel(1'b0),.qspi0_spare_sel(1'b0),
  .htrans(ht),.hwrite(hw),.haddr(ha),.hsize(hs),.hburst(hb),.hprot(hp),.hmastlock(hm),.hwdata(wd),.hrdata(rd),.hresp(2'd0),.hready(1'b1),
  .apb_clk(clk),.apb_rst(!core_reset),.prdata(32'd0),.pready(1'b1),.pslverr(1'b0),
  .scan_mode_n(1'b1),.scan_en_n(1'b1),.pll_bypass(1'b0),.pib_pll_bypass(1'b0),.scan_rst_n(1'b0),
  .scan_in(73'd0),.wrapper_scan_in_0(1'b0),.wrapper_scan_in_1(1'b0),.chain_scan_in(1'b0),
  .dft_jtag_tck(1'b0),.dft_jtag_rst(1'b0),.dft_jtag_tms(1'b0),.dft_jtag_tdi(1'b0));
 PH1P_LOGIC_QSPI2MCU configuration_mux(.qspi_pib_sel_i(1'b1));
endmodule
