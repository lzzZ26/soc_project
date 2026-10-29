//------------------------------------------------------------------------------
// system_top.v
// SoC Top for MES50HP (PGL50H-6IFBG484)
// Architecture: CMSDK MCU + External AXI Crossbar for FIR/DMA
//
// External crossbar (connected via cmsdk_mcu_system ext_* port):
//   Master 0 : CPU (Cortex-M0 external window, register access for FIR/DMA)
//   Master 1 : DMA engine (data transfers)
//   Slave  0 : FIR Filter            @ 0x5000_XXXX
//   Slave  1 : DMA Controller regs   @ 0x5001_XXXX
//   Slave  2 : CMSDK MCU system      (everything else: SRAM@0x2000, APB@0x4000)
//
// Internal CMSDK arbiter (u_sys_arb):
//   Master 0 : Cortex-M0 internal transactions
//   Master 1 : extm_* ingress (DMA accesses to SRAM / APB)
//   Slave  0 : CMSDK system bus
//
// Memory map:
//   0x0000_0000  FLASH (Boot ROM, loaded from hex)
//   0x2000_0000  SRAM (64KB)
//   0x4000_4000  UART0
//   0x4001_0000  GPIO0 (LEDs)
//   0x5000_0000  FIR Filter
//   0x5001_0000  DMA Controller registers
//------------------------------------------------------------------------------
`timescale 1ns / 1ps

module system_top (
    input  wire                       clk_50m,   // 50MHz board clock (P20)
    input  wire                       key1_n,    // Reset button (K18, active low)

    output wire                       uart0_tx,  // UART0 TX -> CP2102 (R9)
    input  wire                       uart0_rx,  // UART0 RX <- CP2102 (R8)

    output wire [7:0]                 led        // LED1~LED8 (B2~F8)
);

    // =========================================================================
    // Clock and Reset
    // =========================================================================
    wire            sys_clk;
    wire            sys_rst_n;

    assign sys_clk  = clk_50m;

    // Power-on reset: KEY1/K18 level was found to be unstable on this board
    // (frequently crossing the threshold when released -> repeated system
    // reboots). The button is therefore NOT connected to the reset chain.
    // rst_cnt powers up at 0 (FPGA register initial value) and free-runs
    // to 2^19 (~10.5ms @50MHz), then holds, releasing sys_rst_n once after
    // configuration. Reset low phase >=10.5ms satisfies CMSDK PORESETn
    // minimum pulse width. Pressing KEY1 no longer affects the user design.
    reg [19:0] rst_cnt = 20'd0;
    always @(posedge sys_clk) begin
        if (rst_cnt != 20'hFFFFF)
            rst_cnt <= rst_cnt + 20'd1;
    end
    assign sys_rst_n = rst_cnt[19];

    // =========================================================================
    // CMSDK MCU System
    // =========================================================================
    wire            mcu_sleeping;
    wire            mcu_lockup;
    wire            mcu_sys_reset_req;

    // CPU external AXI master port signals (to crossbar m0)
    wire            cpu_ext_aw_sel;
    wire            cpu_ext_aw_valid;
    wire            cpu_ext_aw_ready;
    wire  [2:0]     cpu_ext_aw_size;
    wire  [1:0]     cpu_ext_aw_burst;
    wire  [7:0]     cpu_ext_aw_len;
    wire  [31:0]    cpu_ext_aw_addr;
    wire            cpu_ext_w_valid;
    wire            cpu_ext_w_ready;
    wire            cpu_ext_w_last;
    wire  [31:0]    cpu_ext_w_data;
    wire            cpu_ext_b_valid;
    wire            cpu_ext_b_ready;
    wire  [1:0]     cpu_ext_b_resp;
    wire            cpu_ext_ar_sel;
    wire            cpu_ext_ar_valid;
    wire            cpu_ext_ar_ready;
    wire  [2:0]     cpu_ext_ar_size;
    wire  [1:0]     cpu_ext_ar_burst;
    wire  [7:0]     cpu_ext_ar_len;
    wire  [31:0]    cpu_ext_ar_addr;
    wire            cpu_ext_r_valid;
    wire            cpu_ext_r_ready;
    wire            cpu_ext_r_last;
    wire  [31:0]    cpu_ext_r_data;
    wire  [1:0]     cpu_ext_r_resp;

    // extm ingress signals (crossbar s2 -> CMSDK internal arbiter m1)
    wire            extm_aw_valid;
    wire            extm_aw_ready;
    wire  [2:0]     extm_aw_size;
    wire  [1:0]     extm_aw_burst;
    wire  [7:0]     extm_aw_len;
    wire  [31:0]    extm_aw_addr;
    wire            extm_w_valid;
    wire            extm_w_ready;
    wire            extm_w_last;
    wire  [31:0]    extm_w_data;
    wire            extm_b_valid;
    wire            extm_b_ready;
    wire  [1:0]     extm_b_resp;
    wire            extm_ar_valid;
    wire            extm_ar_ready;
    wire  [2:0]     extm_ar_size;
    wire  [1:0]     extm_ar_burst;
    wire  [7:0]     extm_ar_len;
    wire  [31:0]    extm_ar_addr;
    wire            extm_r_valid;
    wire            extm_r_ready;
    wire            extm_r_last;
    wire  [31:0]    extm_r_data;
    wire  [1:0]     extm_r_resp;

    // Interrupts from custom peripherals
    wire            dma_irq;
    wire            fir_irq;

    // GPIO output (single driver)
    wire [15:0]     p0_out;

    // =========================================================================
    // CMSDK MCU System
    // =========================================================================
    wire            uart0_txd_from_mcu;
    wire            uart0_txen_from_mcu;

   // =========================================================================
   // GPIO1 Pin Mux: bridges MCU control signals to physical pins P1
   // =========================================================================
   wire [15:0]     p1_in_from_mux;
   wire [15:0]     p1_io;
   wire [15:0]     p1_out_from_mcu;
   wire [15:0]     p1_outen_from_mcu;
   wire [15:0]     p1_altfunc_from_mcu;

   // =========================================================================
   // GPIO1 Pin Mux instance
   // =========================================================================
   cmsdk_mcu_pin_mux u_pin_mux (
       // UART - RX from physical pin R8, TX to physical pin R9
       .uart0_rxd        (p1_in_from_mux[0]),
       .uart0_txd        (uart0_txd_from_mcu),
       .uart0_txen       (uart0_txen_from_mcu),
       .uart1_rxd        (),
       .uart1_txd        (),
       .uart1_txen       (),
       .uart2_rxd        (),
       .uart2_txd        (),
       .uart2_txen       (),
       .timer0_extin     (),
       .timer1_extin     (),

       // GPIO0 (unused)
       .p0_in            (),
       .p0_out           ({16{1'b0}}),
       .p0_outen         ({16{1'b0}}),
       .p0_altfunc       ({16{1'b0}}),

       // GPIO1 - connected to MCU and physical pins
       .p1_in            (p1_in_from_mux),
       .p1_out           (p1_out_from_mcu),
       .p1_outen         (p1_outen_from_mcu),
       .p1_altfunc       (p1_altfunc_from_mcu),

       // Debug (unused)
       .i_trst_n         (),
       .i_swditms        (),
       .i_swclktck       (),
       .i_tdi            (),
       .i_tdo            (1'b0),
       .i_tdoen_n        (1'b1),
       .i_swdo           (1'b0),
       .i_swdoen         (1'b1),

       // Physical pins P1
       .P1               (p1_io),

       .nTRST            (1'b1),
       .TDI              (1'b0),
       .SWDIOTMS         (),
       .SWCLKTCK         (1'b1),
       .TDO              ()
   );

    cmsdk_mcu_system #(
        .filename       ("image_words.hex"),
        .BASEADDR_GPIO0 (32'h4001_0000),
        .BASEADDR_GPIO1 (32'h4001_1000)
    ) u_cmsdk_mcu_system (
        .FCLK           (sys_clk),
        .SCLK           (sys_clk),
        .PORESETn       (sys_rst_n),
        .ACLK           (sys_clk),
        .ARESETn        (sys_rst_n),
        .PCLK           (sys_clk),
        .PCLKG          (sys_clk),
        .PCLKEN         (1'b1),
        .PRESETn        (sys_rst_n),
        .APBACTIVE      (),
        .SLEEPING       (mcu_sleeping),
        .SLEEPDEEP      (),
        .SYSRESETREQ    (mcu_sys_reset_req),
        .WDOGRESETREQ   (),
        .LOCKUP         (mcu_lockup),
        .LOCKUPRESET    (),
        .PMUENABLE      (),

        // UART0
        .uart0_rxd      (uart0_rx),          // direct from pin R8 (was p1_in_from_mux[0] -> port dangling)
        .uart0_txd      (uart0_txd_from_mcu),
        .uart0_txen     (uart0_txen_from_mcu),
        .uart1_rxd      (1'b1),
        .uart1_txd      (),
        .uart1_txen     (),
        .uart2_rxd      (1'b1),
        .uart2_txd      (),
        .uart2_txen     (),
        .timer0_extin   (1'b0),
        .timer1_extin   (1'b0),

        // GPIO0 for LEDs
        .p0_in          ({16{1'b1}}),
        .p0_out         (p0_out),
        .p0_outen       (),           // output, unused
        .p0_altfunc     (),           // output, unused

        // GPIO1
        .p1_in          (p1_in_from_mux),     // GPIO input from pin_mux
        .p1_out         (p1_out_from_mcu),   // output to pin_mux
        .p1_outen       (p1_outen_from_mcu), // output enable to pin_mux
        .p1_altfunc     (p1_altfunc_from_mcu), // altfunc control to pin_mux

        .DFTSE          (1'b0),

        // External AXI master port -> crossbar m0 (for FIR/DMA register access)
        .ext_aw_sel     (cpu_ext_aw_sel),
        .ext_aw_valid   (cpu_ext_aw_valid),
        .ext_aw_ready   (cpu_ext_aw_ready),
        .ext_aw_size    (cpu_ext_aw_size),
        .ext_aw_burst   (cpu_ext_aw_burst),
        .ext_aw_len     (cpu_ext_aw_len),
        .ext_aw_addr    (cpu_ext_aw_addr),
        .ext_w_valid    (cpu_ext_w_valid),
        .ext_w_ready    (cpu_ext_w_ready),
        .ext_w_last     (cpu_ext_w_last),
        .ext_w_data     (cpu_ext_w_data),
        .ext_b_valid    (cpu_ext_b_valid),
        .ext_b_ready    (cpu_ext_b_ready),
        .ext_b_resp     (cpu_ext_b_resp),
        .ext_ar_sel     (cpu_ext_ar_sel),
        .ext_ar_valid   (cpu_ext_ar_valid),
        .ext_ar_ready   (cpu_ext_ar_ready),
        .ext_ar_size    (cpu_ext_ar_size),
        .ext_ar_burst   (cpu_ext_ar_burst),
        .ext_ar_len     (cpu_ext_ar_len),
        .ext_ar_addr    (cpu_ext_ar_addr),
        .ext_r_valid    (cpu_ext_r_valid),
        .ext_r_ready    (cpu_ext_r_ready),
        .ext_r_last     (cpu_ext_r_last),
        .ext_r_data     (cpu_ext_r_data),
        .ext_r_resp     (cpu_ext_r_resp),

        // External AXI master ingress : crossbar s2 (DMA accesses to SRAM/APB)
        .extm_aw_valid  (extm_aw_valid),
        .extm_aw_ready  (extm_aw_ready),
        .extm_aw_size   (extm_aw_size),
        .extm_aw_burst  (extm_aw_burst),
        .extm_aw_len    (extm_aw_len),
        .extm_aw_addr   (extm_aw_addr),
        .extm_w_valid   (extm_w_valid),
        .extm_w_ready   (extm_w_ready),
        .extm_w_last    (extm_w_last),
        .extm_w_data    (extm_w_data),
        .extm_b_valid   (extm_b_valid),
        .extm_b_ready   (extm_b_ready),
        .extm_b_resp    (extm_b_resp),
        .extm_ar_valid  (extm_ar_valid),
        .extm_ar_ready  (extm_ar_ready),
        .extm_ar_size   (extm_ar_size),
        .extm_ar_burst  (extm_ar_burst),
        .extm_ar_len    (extm_ar_len),
        .extm_ar_addr   (extm_ar_addr),
        .extm_r_valid   (extm_r_valid),
        .extm_r_ready   (extm_r_ready),
        .extm_r_last    (extm_r_last),
        .extm_r_data    (extm_r_data),
        .extm_r_resp    (extm_r_resp),

        // Custom peripheral interrupts
        .dma_irq        (dma_irq),
        .fir_irq        (fir_irq)
    );

    // =========================================================================
    // UART0: direct connection to pins (bypass GPIO1 pin_mux)
    //   TX: firmware never writes GPIO1 ALTFUNCSET, so p1_altfunc[1]=0 and
    //       p1_io[1] would be tristated -> R9 floats, PC sees nothing.
    //   RX: top-level uart0_rx must reach the UART RX shifter; going through
    //       pin_mux.P1[0] would leave the port dangling (pnr warning).
    // =========================================================================
    assign uart0_tx = uart0_txd_from_mcu;

    // =========================================================================
    // External AXI Crossbar
    //   M0 = CPU (ext_* port),  M1 = DMA engine
    //   S0 = FIR @ 0x5000,  S1 = DMA regs @ 0x5001,  S2 = CMSDK (extm_*)
    // PRIO_M0=1: CPU wins on AW/AR when both request, breaking A<->B deadlock
    // with the internal arbiter (which has PRIO_M1=1 for the B channel).
    // =========================================================================
    // DMA master port signals
    wire            dma_m_aw_valid;
    wire            dma_m_aw_ready;
    wire  [2:0]     dma_m_aw_size;
    wire  [1:0]     dma_m_aw_burst;
    wire  [7:0]     dma_m_aw_len;
    wire  [31:0]    dma_m_aw_addr;
    wire            dma_m_w_valid;
    wire            dma_m_w_ready;
    wire            dma_m_w_last;
    wire  [31:0]    dma_m_w_data;
    wire            dma_m_b_valid;
    wire            dma_m_b_ready;
    wire  [1:0]     dma_m_b_resp;
    wire            dma_m_ar_valid;
    wire            dma_m_ar_ready;
    wire  [2:0]     dma_m_ar_size;
    wire  [1:0]     dma_m_ar_burst;
    wire  [7:0]     dma_m_ar_len;
    wire  [31:0]    dma_m_ar_addr;
    wire            dma_m_r_valid;
    wire            dma_m_r_ready;
    wire            dma_m_r_last;
    wire  [31:0]    dma_m_r_data;
    wire  [1:0]     dma_m_r_resp;

    // Crossbar slave side (flattened, 3 slaves)
    wire [2:0]      sb_aw_valid;
    wire [2:0]      sb_aw_ready;
    wire  [8:0]     sb_aw_size;
    wire  [5:0]     sb_aw_burst;
    wire [23:0]     sb_aw_len;
    wire [95:0]     sb_aw_addr;

    wire [2:0]      sb_w_valid;
    wire [2:0]      sb_w_ready;
    wire [2:0]      sb_w_last;
    wire [95:0]     sb_w_data;

    wire [2:0]      sb_b_valid;
    wire [2:0]      sb_b_ready;
    wire  [5:0]     sb_b_resp;

    wire [2:0]      sb_ar_valid;
    wire [2:0]      sb_ar_ready;
    wire  [8:0]     sb_ar_size;
    wire  [5:0]     sb_ar_burst;
    wire [23:0]     sb_ar_len;
    wire [95:0]     sb_ar_addr;

    wire [2:0]      sb_r_valid;
    wire [2:0]      sb_r_ready;
    wire [2:0]      sb_r_last;
    wire [95:0]     sb_r_data;
    wire  [5:0]     sb_r_resp;

    axi_crossbar #(
        .NSLV   (3),
        .PRIO_M0(1),    // CPU wins AW/AR arbitration (breaks A<->B deadlock)
        // m0 is the CPU external-window port (only 0x5000/0x5001 => s0/s1);
        // s2 is the CMSDK loop-back used only by m1 (DMA).  Masking s2 from
        // m0's ready feedback breaks the STA-3001 combinational ring.
        .M0_CH_MASK(3'b011)
    ) u_axi_crossbar (
        .ACLK       (sys_clk),
        .ARESETn    (sys_rst_n),

        // Master 0: CPU (register access for FIR/DMA)
        .m0_aw_valid    (cpu_ext_aw_valid),
        .m0_aw_ready    (cpu_ext_aw_ready),
        .m0_aw_size     (cpu_ext_aw_size),
        .m0_aw_burst    (cpu_ext_aw_burst),
        .m0_aw_len      (cpu_ext_aw_len),
        .m0_aw_addr     (cpu_ext_aw_addr),
        .m0_w_valid     (cpu_ext_w_valid),
        .m0_w_ready     (cpu_ext_w_ready),
        .m0_w_last      (cpu_ext_w_last),
        .m0_w_data      (cpu_ext_w_data),
        .m0_b_valid     (cpu_ext_b_valid),
        .m0_b_ready     (cpu_ext_b_ready),
        .m0_b_resp      (cpu_ext_b_resp),
        .m0_ar_valid    (cpu_ext_ar_valid),
        .m0_ar_ready    (cpu_ext_ar_ready),
        .m0_ar_size     (cpu_ext_ar_size),
        .m0_ar_burst    (cpu_ext_ar_burst),
        .m0_ar_len      (cpu_ext_ar_len),
        .m0_ar_addr     (cpu_ext_ar_addr),
        .m0_r_valid     (cpu_ext_r_valid),
        .m0_r_ready     (cpu_ext_r_ready),
        .m0_r_last      (cpu_ext_r_last),
        .m0_r_data      (cpu_ext_r_data),
        .m0_r_resp      (cpu_ext_r_resp),

        // Master 1: DMA engine (data transfers)
        .m1_aw_valid    (dma_m_aw_valid),
        .m1_aw_ready    (dma_m_aw_ready),
        .m1_aw_size     (dma_m_aw_size),
        .m1_aw_burst    (dma_m_aw_burst),
        .m1_aw_len      (dma_m_aw_len),
        .m1_aw_addr     (dma_m_aw_addr),
        .m1_w_valid     (dma_m_w_valid),
        .m1_w_ready     (dma_m_w_ready),
        .m1_w_last      (dma_m_w_last),
        .m1_w_data      (dma_m_w_data),
        .m1_b_valid     (dma_m_b_valid),
        .m1_b_ready     (dma_m_b_ready),
        .m1_b_resp      (dma_m_b_resp),
        .m1_ar_valid    (dma_m_ar_valid),
        .m1_ar_ready    (dma_m_ar_ready),
        .m1_ar_size     (dma_m_ar_size),
        .m1_ar_burst    (dma_m_ar_burst),
        .m1_ar_len      (dma_m_ar_len),
        .m1_ar_addr     (dma_m_ar_addr),
        .m1_r_valid     (dma_m_r_valid),
        .m1_r_ready     (dma_m_r_ready),
        .m1_r_last      (dma_m_r_last),
        .m1_r_data      (dma_m_r_data),
        .m1_r_resp      (dma_m_r_resp),

        // Slave 0: FIR Filter @ 0x5000_XXXX
        .s_aw_valid     (sb_aw_valid),
        .s_aw_ready     (sb_aw_ready),
        .s_aw_size      (sb_aw_size),
        .s_aw_burst     (sb_aw_burst),
        .s_aw_len       (sb_aw_len),
        .s_aw_addr      (sb_aw_addr),
        .s_w_valid      (sb_w_valid),
        .s_w_ready      (sb_w_ready),
        .s_w_last       (sb_w_last),
        .s_w_data       (sb_w_data),
        .s_b_valid      (sb_b_valid),
        .s_b_ready      (sb_b_ready),
        .s_b_resp       (sb_b_resp),
        .s_ar_valid     (sb_ar_valid),
        .s_ar_ready     (sb_ar_ready),
        .s_ar_size      (sb_ar_size),
        .s_ar_burst     (sb_ar_burst),
        .s_ar_len       (sb_ar_len),
        .s_ar_addr      (sb_ar_addr),
        .s_r_valid      (sb_r_valid),
        .s_r_ready      (sb_r_ready),
        .s_r_last       (sb_r_last),
        .s_r_data       (sb_r_data),
        .s_r_resp       (sb_r_resp)
    );

    // =========================================================================
    // Crossbar slave 2 <-> CMSDK external master ingress (extm_*)
    // Slave 2 covers all addresses not matching 0x5000_xxxx / 0x5001_xxxx
    // (i.e. SRAM 0x2000_xxxx and APB 0x4000_xxxx) for the DMA master port.
    // =========================================================================
    // AW channel (crossbar drives valid/addr, CMSDK drives ready)
    assign extm_aw_valid  = sb_aw_valid[2];
    assign sb_aw_ready[2] = extm_aw_ready;
    assign extm_aw_size   = sb_aw_size[8:6];
    assign extm_aw_burst  = sb_aw_burst[5:4];
    assign extm_aw_len    = sb_aw_len[23:16];
    assign extm_aw_addr   = sb_aw_addr[95:64];
    // W channel
    assign extm_w_valid   = sb_w_valid[2];
    assign sb_w_ready[2]  = extm_w_ready;
    assign extm_w_last    = sb_w_last[2];
    assign extm_w_data    = sb_w_data[95:64];
    // B channel (CMSDK drives valid/resp, crossbar drives ready)
    assign sb_b_valid[2]  = extm_b_valid;
    assign extm_b_ready   = sb_b_ready[2];
    assign sb_b_resp[5:4] = extm_b_resp;
    // AR channel
    assign extm_ar_valid  = sb_ar_valid[2];
    assign sb_ar_ready[2] = extm_ar_ready;
    assign extm_ar_size   = sb_ar_size[8:6];
    assign extm_ar_burst  = sb_ar_burst[5:4];
    assign extm_ar_len    = sb_ar_len[23:16];
    assign extm_ar_addr   = sb_ar_addr[95:64];
    // R channel
    assign sb_r_valid[2]  = extm_r_valid;
    assign extm_r_ready   = sb_r_ready[2];
    assign sb_r_last[2]   = extm_r_last;
    assign sb_r_data[95:64] = extm_r_data;
    assign sb_r_resp[5:4] = extm_r_resp;

    // =========================================================================
    // AXI DMA Controller
    //   Slave port: CPU writes DMA registers (via crossbar slave 1)
    //   Master port: DMA data transfers (via crossbar master 1)
    // =========================================================================
    axi_dma #(
        .NUM_CH (6)
    ) u_axi_dma (
        .ACLK       (sys_clk),
        .ARESETn    (sys_rst_n),

        // AXI slave port for register access (CPU -> crossbar s1 -> DMA)
        .s_aw_valid     (sb_aw_valid[1]),
        .s_aw_ready     (sb_aw_ready[1]),
        .s_aw_size      (sb_aw_size[8:6]),
        .s_aw_burst     (sb_aw_burst[5:4]),
        .s_aw_len       (sb_aw_len[23:16]),
        .s_aw_addr      (sb_aw_addr[63:32]),
        .s_w_valid      (sb_w_valid[1]),
        .s_w_ready      (sb_w_ready[1]),
        .s_w_last       (sb_w_last[1]),
        .s_w_data       (sb_w_data[63:32]),
        .s_b_valid      (sb_b_valid[1]),
        .s_b_ready      (sb_b_ready[1]),
        .s_b_resp       (sb_b_resp[3:2]),
        .s_ar_valid     (sb_ar_valid[1]),
        .s_ar_ready     (sb_ar_ready[1]),
        .s_ar_size      (sb_ar_size[8:6]),
        .s_ar_burst     (sb_ar_burst[5:4]),
        .s_ar_len       (sb_ar_len[23:16]),
        .s_ar_addr      (sb_ar_addr[63:32]),
        .s_r_valid      (sb_r_valid[1]),
        .s_r_ready      (sb_r_ready[1]),
        .s_r_last       (sb_r_last[1]),
        .s_r_data       (sb_r_data[63:32]),
        .s_r_resp       (sb_r_resp[3:2]),

        // AXI master port for data transfers (via crossbar m1)
        .m_aw_valid     (dma_m_aw_valid),
        .m_aw_ready     (dma_m_aw_ready),
        .m_aw_size      (dma_m_aw_size),
        .m_aw_burst     (dma_m_aw_burst),
        .m_aw_len       (dma_m_aw_len),
        .m_aw_addr      (dma_m_aw_addr),
        .m_w_valid      (dma_m_w_valid),
        .m_w_ready      (dma_m_w_ready),
        .m_w_last       (dma_m_w_last),
        .m_w_data       (dma_m_w_data),
        .m_b_valid      (dma_m_b_valid),
        .m_b_ready      (dma_m_b_ready),
        .m_b_resp       (dma_m_b_resp),
        .m_ar_valid     (dma_m_ar_valid),
        .m_ar_ready     (dma_m_ar_ready),
        .m_ar_size      (dma_m_ar_size),
        .m_ar_burst     (dma_m_ar_burst),
        .m_ar_len       (dma_m_ar_len),
        .m_ar_addr      (dma_m_ar_addr),
        .m_r_valid      (dma_m_r_valid),
        .m_r_ready      (dma_m_r_ready),
        .m_r_last       (dma_m_r_last),
        .m_r_data       (dma_m_r_data),
        .m_r_resp       (dma_m_r_resp),

        .irq_o          (dma_irq)
    );

    // =========================================================================
    // AXI FIR Wrapper
    // Connected via crossbar slave 0
    // =========================================================================
    wire            fir_in_valid;
    wire            fir_in_ready;
    wire signed[15:0] fir_in_data;
    wire            fir_out_valid;
    wire signed[15:0] fir_out_data;
    wire            fir_clr;

    wire            coef_wr_en;
    wire [6:0]      coef_idx;
    wire signed[15:0] coef_wdata;
    wire signed[15:0] coef_rdata;

    axi_fir_wrap u_axi_fir (
        .ACLK        (sys_clk),
        .ARESETn     (sys_rst_n),

        // AXI slave from crossbar s0 (CPU register access)
        .s_aw_valid  (sb_aw_valid[0]),
        .s_aw_ready  (sb_aw_ready[0]),
        .s_aw_size   (sb_aw_size[2:0]),
        .s_aw_burst  (sb_aw_burst[1:0]),
        .s_aw_len    (sb_aw_len[7:0]),
        .s_aw_addr   (sb_aw_addr[31:0]),
        .s_w_valid   (sb_w_valid[0]),
        .s_w_ready   (sb_w_ready[0]),
        .s_w_last    (sb_w_last[0]),
        .s_w_data    (sb_w_data[31:0]),
        .s_b_valid   (sb_b_valid[0]),
        .s_b_ready   (sb_b_ready[0]),
        .s_b_resp    (sb_b_resp[1:0]),
        .s_ar_valid  (sb_ar_valid[0]),
        .s_ar_ready  (sb_ar_ready[0]),
        .s_ar_size   (sb_ar_size[2:0]),
        .s_ar_burst  (sb_ar_burst[1:0]),
        .s_ar_len    (sb_ar_len[7:0]),
        .s_ar_addr   (sb_ar_addr[31:0]),
        .s_r_valid   (sb_r_valid[0]),
        .s_r_ready   (sb_r_ready[0]),
        .s_r_last    (sb_r_last[0]),
        .s_r_data    (sb_r_data[31:0]),
        .s_r_resp    (sb_r_resp[1:0]),

        // Stream interface to fir_81tap
        .fir_in_valid  (fir_in_valid),
        .fir_in_ready  (fir_in_ready),
        .fir_in_data   (fir_in_data),
        .fir_out_valid (fir_out_valid),
        .fir_out_data  (fir_out_data),

        // Coefficient load interface
        .coef_wr_en    (coef_wr_en),
        .coef_idx      (coef_idx),
        .coef_wdata    (coef_wdata),
        .coef_rdata    (coef_rdata),

        .fir_clr       (fir_clr),

        .irq_o         (fir_irq)
    );

    // =========================================================================
    // 81-tap FIR filter core
    // =========================================================================
    fir_81tap u_fir_core (
        .clk        (sys_clk),
        .rst_n      (sys_rst_n),
        .clr        (fir_clr),
        .in_valid   (fir_in_valid),
        .in_ready   (fir_in_ready),
        .in_data    (fir_in_data),
        .out_valid  (fir_out_valid),
        .out_data   (fir_out_data),
        .coef_wr_en (coef_wr_en),
        .coef_idx   (coef_idx),
        .coef_wdata (coef_wdata),
        .coef_rdata (coef_rdata)
    );

    // =========================================================================
    // Status LEDs
    // =========================================================================
    reg  [25:0]  status_cnt;
    reg          status_toggle;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (~sys_rst_n) begin
            status_cnt    <= 0;
            status_toggle <= 0;
        end else begin
            if (status_cnt >= 25_000_000) begin  // ~1 Hz at 50MHz
                status_cnt    <= 0;
                status_toggle <= ~status_toggle;
            end else begin
                status_cnt <= status_cnt + 1'b1;
            end
        end
    end

    // Single driver: LED[0] = status toggle, LED[7:1] = GPIO0[7:1]
    assign led[0]   = status_toggle;
    assign led[7:1] = p0_out[7:1];

endmodule
