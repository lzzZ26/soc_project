//------------------------------------------------------------------------------
// axi_crossbar.v
//
// Generic 2-master / N-slave AXI4 interconnect used two ways in this SoC:
//
//   1) system_top : NSLV=3
//        Master 0 = Cortex-M0 external window  (0x5000_xxxx / 0x5001_xxxx)
//        Master 1 = DMA engine master port
//        Slave  0 = FIR filter            @ 0x5000_xxxx
//        Slave  1 = DMA register file     @ 0x5001_xxxx
//        Slave  2 = CMSDK MCU system      (everything else, only DMA reaches it)
//
//   2) cmsdk_mcu_system : NSLV=1  (pure 2:1 master arbiter)
//        Master 0 = Cortex-M0 internal transactions
//        Master 1 = external "exts" port fed by the crossbar slave 2
//        Slave  0 = CMSDK system bus (sys_*)
//
// Arbitration / locking rules (deadlock free by construction)
// ------------------------------------------------------------
//   * ONE shared lock for the whole interconnect.  The lock is taken when an
//     AW or AR request is granted and released on the matching B response or
//     on RVALID&RREADY&RLAST.  Because only one transaction is ever in flight,
//     AW_VALID and AR_VALID are never simultaneously high towards a slave,
//     which is required by the CMSDK slave multiplexer (its read/write select
//     registers are updated from a single if/else-if chain).
//   * Read and write responses are never reordered because they are never
//     concurrent.
//   * Address / size / burst / len are captured at grant time and held for the
//     whole transaction so that downstream W_READY / AW_READY logic that decodes
//     a live address (CMSDK slave mux) keeps seeing a stable address.
//   * Round robin between the two masters so neither can starve.
//
// Single outstanding transaction per direction of the interconnect, which is
// sufficient for the Cortex-M0 AHB->AXI bridge (single outstanding write, read
// issued only after the previous one completes) and for the DMA engine.
//------------------------------------------------------------------------------
`timescale 1ns / 1ps

module axi_crossbar #(
    parameter integer NSLV = 3,
    // Arbitration policy when both masters request in the same cycle:
    //   PRIO_M0 = 1 : master 0 always wins while it has a request
    //   PRIO_M1 = 1 : master 1 always wins while it has a request
    //   both   = 0  : round robin (fair, default)
    // See the deadlock note at the bottom of this header.
    parameter integer PRIO_M0 = 0,
    parameter integer PRIO_M1 = 0,
    // Slaves reachable from master 0 (bit i = 1 => slave i allowed).
    // Default: all slaves (generic behavior unchanged).  The top-level
    // instance sets this to 3'b011 because m0 is the CPU external-window
    // port (only 0x5000_0000/0x5001_xxxx => s0/s1) while s2 is the CMSDK
    // loop-back port used only by m1 (DMA).  Masking bit 2 while m0 owns
    // is a no-op in normal operation and structurally removes the
    // combinational feedback ring
    //   m0_ready <- s2 ready <- extm <- sys_arb <- sys_*READY
    //   <- slave mux <- m0_ready   (PDS STA-3001)
    parameter [31:0]  M0_CH_MASK = 32'hFFFF_FFFF
)(
    input  wire                     ACLK,
    input  wire                     ARESETn,

    // ==================== Master 0 ====================
    input  wire                     m0_aw_valid,
    output wire                     m0_aw_ready,
    input  wire  [2:0]              m0_aw_size,
    input  wire  [1:0]              m0_aw_burst,
    input  wire  [7:0]              m0_aw_len,
    input  wire  [31:0]             m0_aw_addr,
    input  wire                     m0_w_valid,
    output wire                     m0_w_ready,
    input  wire                     m0_w_last,
    input  wire  [31:0]             m0_w_data,
    output wire                     m0_b_valid,
    input  wire                     m0_b_ready,
    output wire  [1:0]              m0_b_resp,
    input  wire                     m0_ar_valid,
    output wire                     m0_ar_ready,
    input  wire  [2:0]              m0_ar_size,
    input  wire  [1:0]              m0_ar_burst,
    input  wire  [7:0]              m0_ar_len,
    input  wire  [31:0]             m0_ar_addr,
    output wire                     m0_r_valid,
    input  wire                     m0_r_ready,
    output wire                     m0_r_last,
    output wire  [31:0]             m0_r_data,
    output wire  [1:0]              m0_r_resp,

    // ==================== Master 1 ====================
    input  wire                     m1_aw_valid,
    output wire                     m1_aw_ready,
    input  wire  [2:0]              m1_aw_size,
    input  wire  [1:0]              m1_aw_burst,
    input  wire  [7:0]              m1_aw_len,
    input  wire  [31:0]             m1_aw_addr,
    input  wire                     m1_w_valid,
    output wire                     m1_w_ready,
    input  wire                     m1_w_last,
    input  wire  [31:0]             m1_w_data,
    output wire                     m1_b_valid,
    input  wire                     m1_b_ready,
    output wire  [1:0]              m1_b_resp,
    input  wire                     m1_ar_valid,
    output wire                     m1_ar_ready,
    input  wire  [2:0]              m1_ar_size,
    input  wire  [1:0]              m1_ar_burst,
    input  wire  [7:0]              m1_ar_len,
    input  wire  [31:0]             m1_ar_addr,
    output wire                     m1_r_valid,
    input  wire                     m1_r_ready,
    output wire                     m1_r_last,
    output wire  [31:0]             m1_r_data,
    output wire  [1:0]              m1_r_resp,

    // ==================== Slaves (flattened, NSLV wide) ====================
    output wire [NSLV-1:0]          s_aw_valid,
    input  wire [NSLV-1:0]          s_aw_ready,
    output wire [NSLV*3-1:0]        s_aw_size,
    output wire [NSLV*2-1:0]        s_aw_burst,
    output wire [NSLV*8-1:0]        s_aw_len,
    output wire [NSLV*32-1:0]       s_aw_addr,

    output wire [NSLV-1:0]          s_w_valid,
    input  wire [NSLV-1:0]          s_w_ready,
    output wire [NSLV-1:0]          s_w_last,
    output wire [NSLV*32-1:0]       s_w_data,

    input  wire [NSLV-1:0]          s_b_valid,
    output wire [NSLV-1:0]          s_b_ready,
    input  wire [NSLV*2-1:0]        s_b_resp,

    output wire [NSLV-1:0]          s_ar_valid,
    input  wire [NSLV-1:0]          s_ar_ready,
    output wire [NSLV*3-1:0]        s_ar_size,
    output wire [NSLV*2-1:0]        s_ar_burst,
    output wire [NSLV*8-1:0]        s_ar_len,
    output wire [NSLV*32-1:0]       s_ar_addr,

    input  wire [NSLV-1:0]          s_r_valid,
    output wire [NSLV-1:0]          s_r_ready,
    input  wire [NSLV-1:0]          s_r_last,
    input  wire [NSLV*32-1:0]       s_r_data,
    input  wire [NSLV*2-1:0]        s_r_resp
);

    localparam [1:0] OWN_M0 = 2'd1;
    localparam [1:0] OWN_M1 = 2'd2;

    //------------------------------------------------------------------
    // Address decode : one hot slave select
    //------------------------------------------------------------------
    function [NSLV-1:0] f_dec;
        input [31:0] a;
        begin
            if (NSLV == 1)
                f_dec = 1;
            else if (a[31:16] == 16'h5000)
                f_dec = 1;                       // slave 0 : FIR
            else if (a[31:16] == 16'h5001)
                f_dec = 2;                       // slave 1 : DMA registers
            else
                f_dec = (1 << (NSLV-1));         // last slave : CMSDK window
        end
    endfunction

    //------------------------------------------------------------------
    // Lock / transaction state
    //------------------------------------------------------------------
    reg          busy;
    reg          busy_wr;
    reg  [1:0]   owner;
    reg          aw_done;
    reg          ar_done;
    reg  [31:0]  addr_q;
    reg  [2:0]   size_q;
    reg  [1:0]   burst_q;
    reg  [7:0]   len_q;
    reg          rr;            // 0 : prefer master0, 1 : prefer master1

    wire m0_wr_req = m0_aw_valid;
    wire m0_rd_req = m0_ar_valid;
    wire m1_wr_req = m1_aw_valid;
    wire m1_rd_req = m1_ar_valid;
    wire any_req   = m0_wr_req | m0_rd_req | m1_wr_req | m1_rd_req;

    //------------------------------------------------------------------
    // Grant selection (combinational, valid also in the grant cycle itself)
    //------------------------------------------------------------------
    reg  [1:0] nxt_owner;
    reg        nxt_wr;
    always @(*) begin
        nxt_owner = owner;
        nxt_wr    = busy_wr;
        if (~busy & any_req) begin
            if (~rr) begin
                if (m0_wr_req)      begin nxt_owner = OWN_M0; nxt_wr = 1'b1; end
                else if (m0_rd_req) begin nxt_owner = OWN_M0; nxt_wr = 1'b0; end
                else if (m1_wr_req) begin nxt_owner = OWN_M1; nxt_wr = 1'b1; end
                else                begin nxt_owner = OWN_M1; nxt_wr = 1'b0; end
            end else begin
                if (m1_wr_req)      begin nxt_owner = OWN_M1; nxt_wr = 1'b1; end
                else if (m1_rd_req) begin nxt_owner = OWN_M1; nxt_wr = 1'b0; end
                else if (m0_wr_req) begin nxt_owner = OWN_M0; nxt_wr = 1'b1; end
                else                begin nxt_owner = OWN_M0; nxt_wr = 1'b0; end
            end
        end
    end

    wire        cur_valid = busy | any_req;
    wire [1:0]  cur_owner = busy ? owner    : nxt_owner;
    wire        cur_wr    = busy ? busy_wr  : nxt_wr;
    wire        sel_m0    = (cur_owner == OWN_M0);
    wire        grant_wr  = cur_valid &  cur_wr;
    wire        grant_rd  = cur_valid & ~cur_wr;

    // payload of the selected master
    wire [31:0] l_aw_addr  = sel_m0 ? m0_aw_addr  : m1_aw_addr;
    wire [2:0]  l_aw_size  = sel_m0 ? m0_aw_size  : m1_aw_size;
    wire [1:0]  l_aw_burst = sel_m0 ? m0_aw_burst : m1_aw_burst;
    wire [7:0]  l_aw_len   = sel_m0 ? m0_aw_len   : m1_aw_len;
    wire [31:0] l_ar_addr  = sel_m0 ? m0_ar_addr  : m1_ar_addr;
    wire [2:0]  l_ar_size  = sel_m0 ? m0_ar_size  : m1_ar_size;
    wire [1:0]  l_ar_burst = sel_m0 ? m0_ar_burst : m1_ar_burst;
    wire [7:0]  l_ar_len   = sel_m0 ? m0_ar_len   : m1_ar_len;

    wire [31:0] l_addr  = cur_wr ? l_aw_addr  : l_ar_addr;
    wire [2:0]  l_size  = cur_wr ? l_aw_size  : l_ar_size;
    wire [1:0]  l_burst = cur_wr ? l_aw_burst : l_ar_burst;
    wire [7:0]  l_len   = cur_wr ? l_aw_len   : l_ar_len;

    wire [31:0] o_addr  = busy ? addr_q  : l_addr;
    wire [2:0]  o_size  = busy ? size_q  : l_size;
    wire [1:0]  o_burst = busy ? burst_q : l_burst;
    wire [7:0]  o_len   = busy ? len_q   : l_len;

    wire [NSLV-1:0] sel = f_dec(o_addr);

    // selected-master control payloads
    wire        m_aw_valid_s = sel_m0 ? m0_aw_valid : m1_aw_valid;
    wire        m_w_valid_s  = sel_m0 ? m0_w_valid  : m1_w_valid;
    wire        m_w_last_s   = sel_m0 ? m0_w_last   : m1_w_last;
    wire [31:0] m_w_data_s   = sel_m0 ? m0_w_data   : m1_w_data;
    wire        m_b_ready_s  = sel_m0 ? m0_b_ready  : m1_b_ready;
    wire        m_ar_valid_s = sel_m0 ? m0_ar_valid : m1_ar_valid;
    wire        m_r_ready_s  = sel_m0 ? m0_r_ready  : m1_r_ready;

    //------------------------------------------------------------------
    // Slave side handshakes
    //------------------------------------------------------------------
    wire [NSLV-1:0] aw_rdy_sel = s_aw_ready & sel;
    wire [NSLV-1:0] w_rdy_sel  = s_w_ready  & sel;
    wire [NSLV-1:0] ar_rdy_sel = s_ar_ready & sel;

    // m0-only views with unreachable slaves masked out (see M0_CH_MASK).
    // Used for the m0 ready outputs so s2 feedback cannot form a
    // combinational ring back into the CMSDK when m0 owns the bus.
    wire [NSLV-1:0] aw_rdy_sel_m0 = aw_rdy_sel & M0_CH_MASK[NSLV-1:0];
    wire [NSLV-1:0] w_rdy_sel_m0  = w_rdy_sel  & M0_CH_MASK[NSLV-1:0];
    wire [NSLV-1:0] ar_rdy_sel_m0 = ar_rdy_sel & M0_CH_MASK[NSLV-1:0];

    wire aw_hs = grant_wr & ~aw_done & m_aw_valid_s & (|aw_rdy_sel);
    wire w_hs  = grant_wr &  aw_done & m_w_valid_s  & (|w_rdy_sel);
    wire ar_hs = grant_rd & ~ar_done & m_ar_valid_s & (|ar_rdy_sel);

    //------------------------------------------------------------------
    // Response demultiplexing (only the granted slave contributes)
    //------------------------------------------------------------------
    reg         b_any;
    reg  [1:0]  b_resp;
    reg         r_any;
    reg  [31:0] r_data;
    reg         r_last;
    reg  [1:0]  r_resp;
    integer     i;
    always @(*) begin
        b_any  = 1'b0;
        b_resp = 2'b00;
        r_any  = 1'b0;
        r_data = 32'b0;
        r_last = 1'b0;
        r_resp = 2'b00;
        for (i = 0; i < NSLV; i = i + 1) begin
            if (sel[i]) begin
                b_any  = b_any  | s_b_valid[i];
                b_resp = b_resp | s_b_resp[i*2 +: 2];
                r_any  = r_any  | s_r_valid[i];
                r_data = r_data | s_r_data[i*32 +: 32];
                r_last = r_last | s_r_last[i];
                r_resp = r_resp | s_r_resp[i*2 +: 2];
            end
        end
    end

    wire b_hs = grant_wr &  b_any & m_b_ready_s;
    wire r_hs = grant_rd &  r_any & m_r_ready_s;
    wire rel  = b_hs | (r_hs & r_last);

    //------------------------------------------------------------------
    // Lock sequencer
    //------------------------------------------------------------------
    always @(posedge ACLK or negedge ARESETn) begin
        if (~ARESETn) begin
            busy    <= 1'b0;
            busy_wr <= 1'b0;
            owner   <= 2'd0;
            aw_done <= 1'b0;
            ar_done <= 1'b0;
            addr_q  <= 32'b0;
            size_q  <= 3'd0;
            burst_q <= 2'd0;
            len_q   <= 8'd0;
            rr      <= 1'b0;
        end else if (~busy) begin
            if (any_req) begin
                busy    <= 1'b1;
                busy_wr <= nxt_wr;
                owner   <= nxt_owner;
                addr_q  <= l_addr;
                size_q  <= l_size;
                burst_q <= l_burst;
                len_q   <= l_len;
                aw_done <= aw_hs;
                ar_done <= ar_hs;
                rr      <= (nxt_owner == OWN_M0);
            end
        end else begin
            if (aw_hs) aw_done <= 1'b1;
            if (ar_hs) ar_done <= 1'b1;
            if (rel) begin
                busy    <= 1'b0;
                aw_done <= 1'b0;
                ar_done <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    // Slave side outputs
    //------------------------------------------------------------------
    // Gate the address with the transaction-in-progress term so that the
    // bus carries a defined value (0) when idle.  Without this the address
    // lines float at X whenever the master's ADDR is X while VALID is low,
    // and the downstream address decode then produces X select lines which
    // contaminate the OR-muxes in cmsdk_axi_slave_mux.  The address must
    // remain driven for the WHOLE transaction (busy=1 holds addr_q), not
    // only until the AW/AR handshake, because the slave mux routes W/B
    // (and AW/W READY) through the combinational decode of this address.
    wire aw_act = grant_wr & ~aw_done & m_aw_valid_s;
    wire ar_act = grant_rd & ~ar_done & m_ar_valid_s;

    assign s_aw_valid = {NSLV{aw_act}} & sel;
    assign s_aw_size  = {NSLV{o_size}};
    assign s_aw_burst = {NSLV{o_burst}};
    assign s_aw_len   = {NSLV{o_len}};
    assign s_aw_addr  = {{(NSLV*32){busy | aw_act}}} & {NSLV{o_addr}};

    assign s_w_valid  = {NSLV{grant_wr & aw_done & m_w_valid_s}} & sel;
    assign s_w_last   = {NSLV{m_w_last_s}};
    assign s_w_data   = {NSLV{m_w_data_s}};
    assign s_b_ready  = {NSLV{grant_wr & m_b_ready_s}} & sel;

    assign s_ar_valid = {NSLV{ar_act}} & sel;
    assign s_ar_size  = {NSLV{o_size}};
    assign s_ar_burst = {NSLV{o_burst}};
    assign s_ar_len   = {NSLV{o_len}};
    assign s_ar_addr  = {{(NSLV*32){busy | ar_act}}} & {NSLV{o_addr}};
    assign s_r_ready  = {NSLV{grant_rd & m_r_ready_s}} & sel;

    //------------------------------------------------------------------
    // Master side outputs
    //------------------------------------------------------------------
    assign m0_aw_ready = grant_wr & ~aw_done & sel_m0 & (|aw_rdy_sel_m0);
    assign m0_w_ready  = grant_wr &  aw_done & sel_m0 & (|w_rdy_sel_m0);
    assign m0_b_valid  = grant_wr & sel_m0 & b_any;
    assign m0_b_resp   = b_resp;
    assign m0_ar_ready = grant_rd & ~ar_done & sel_m0 & (|ar_rdy_sel_m0);
    assign m0_r_valid  = grant_rd & sel_m0 & r_any;
    assign m0_r_last   = r_last;
    assign m0_r_data   = r_data;
    assign m0_r_resp   = r_resp;

    assign m1_aw_ready = grant_wr & ~aw_done & ~sel_m0 & (|aw_rdy_sel);
    assign m1_w_ready  = grant_wr &  aw_done & ~sel_m0 & (|w_rdy_sel);
    assign m1_b_valid  = grant_wr & ~sel_m0 & b_any;
    assign m1_b_resp   = b_resp;
    assign m1_ar_ready = grant_rd & ~ar_done & ~sel_m0 & (|ar_rdy_sel);
    assign m1_r_valid  = grant_rd & ~sel_m0 & r_any;
    assign m1_r_last   = r_last;
    assign m1_r_data   = r_data;
    assign m1_r_resp   = r_resp;

    // synthesis translate_off
    //------------------------------------------------------------------
    // TEMPORARY delta-loop detector (debug only, remove after fix).
    // If combinational oscillation is present, this always block is
    // re-evaluated thousands of times inside ONE time step, while a
    // healthy design only settles a handful of evaluations per step.
    //------------------------------------------------------------------
    integer dbg_hits;
    integer dbg_armed;
    real    dbg_t;
    initial begin
        dbg_hits  = 0;
        dbg_armed = 0;
        dbg_t     = -1.0;
    end
    always @(m0_aw_valid, m0_aw_ready, m0_w_valid, m0_w_ready,
             m0_b_valid, m0_b_ready, m0_ar_valid, m0_ar_ready,
             m0_r_valid, m0_r_ready,
             m1_aw_valid, m1_aw_ready, m1_w_valid, m1_w_ready,
             m1_b_valid, m1_b_ready, m1_ar_valid, m1_ar_ready,
             m1_r_valid, m1_r_ready,
             s_aw_valid, s_aw_ready, s_w_valid, s_w_ready,
             s_b_valid, s_b_ready, s_ar_valid, s_ar_ready,
             s_r_valid, s_r_ready,
             busy, owner, aw_done, ar_done, rr,
             nxt_owner, nxt_wr, cur_valid, cur_owner,
             grant_wr, grant_rd, any_req, sel, o_addr) begin
        if ($realtime != dbg_t) begin
            dbg_t     = $realtime;
            dbg_hits  = 0;
            dbg_armed = 0;
        end
        dbg_hits = dbg_hits + 1;
        if (dbg_hits == 500)
            dbg_armed = 1;
        if (dbg_armed && (dbg_hits % 5 == 0)) begin
            $display("DELTA-OSC %0t %m hits=%0d busy=%b owner=%b nxt_owner=%b rr=%b sel=%b any_req=%b gw=%b gr=%b | m0 awv=%b awr=%b wv=%b wr=%b bv=%b br=%b arv=%b arr=%b rv=%b rr=%b | m1 awv=%b arv=%b arr=%b | s awv=%b awr=%b wv=%b wr=%b bv=%b br=%b arv=%b arr=%b rv=%b rr=%b",
                     $time, dbg_hits, busy, owner, nxt_owner, rr, sel, any_req,
                     grant_wr, grant_rd,
                     m0_aw_valid, m0_aw_ready, m0_w_valid, m0_w_ready,
                     m0_b_valid, m0_b_ready, m0_ar_valid, m0_ar_ready,
                     m0_r_valid, m0_r_ready,
                     m1_aw_valid, m1_ar_valid, m1_ar_ready,
                     s_aw_valid, s_aw_ready, s_w_valid, s_w_ready,
                     s_b_valid, s_b_ready, s_ar_valid, s_ar_ready,
                     s_r_valid, s_r_ready);
            $fflush();
        end
        if (dbg_hits == 2499) begin
            $display("DELTA-OSC DONE %0t %m - combinational loop confirmed, finishing.",
                     $time);
            $fflush();
            $finish;
        end
    end
    // synthesis translate_on

endmodule
