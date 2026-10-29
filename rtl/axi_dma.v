//------------------------------------------------------------------------------
// axi_dma.v
//
// 6 channel memory DMA with a proper AXI4 slave (register file) and a proper
// AXI4 master (data engine).
//
// Slave register map  (base 0x5001_0000, channel n = addr[11:8], step 0x100)
// ---------------------------------------------------------------------------
//   n*0x100 + 0x00  CTRL     w: [0] EN  [1] START  [2] IRQ_EN
//                           r: [0] EN  [2] IRQ_EN [3] BUSY [4] DONE [5] ERR
//   n*0x100 + 0x04  SRC      rw  source address (4 byte aligned)
//   n*0x100 + 0x08  DST      rw  destination address (4 byte aligned)
//   n*0x100 + 0x0C  LEN      rw  transfer length in BYTES (multiple of 4)
//   n*0x100 + 0x10  CFG      rw  [1:0] burst beats: 0=1 1=4 2=8 3=16
//                                [2]    src increment (mem2mem only)
//                                [4]    dst increment (mem2mem only)
//                                [7:6]  type 00 mem2mem 01 mem2fir
//                                              10 fir2mem 11 mem2uart
//   n*0x100 + 0x14  STATUS   r   bytes still to be transferred
//
//   global (addr[11:8] == 6)
//   +0x600  IRQ_STAT   rw1c  bit n = channel n finished
//   +0x604  IRQ_EN     rw    bit n = channel n interrupt enable
//   +0x608  IRQ_CLR    w     write 1s to clear IRQ_STAT
//   +0x60C  VERSION    r     0x444D_4131  ("DMA1")
//
// Engine
// ---------------------------------------------------------------------------
//   * one channel at a time, lowest index first with a rotating start pointer
//   * each chunk : read burst  ->  16 word buffer  ->  write burst
//   * chunk beats = min(cfg burst, words remaining, room to the 4KB boundary)
//   * multi beat bursts are only issued when the address falls in SRAM
//     (0x2000_xxxx) or the FIR/DMA window (0x5000/0x5001_xxxx).  Everything
//     else - notably the APB/UART block at 0x4000_xxxx - is forced to single
//     beat transfers because the AXI2APB bridge only handles AWLEN=0.
//   * INCR when the side increments, FIXED when it is a peripheral stream port
//------------------------------------------------------------------------------
`timescale 1ns / 1ps

module axi_dma #(
    parameter integer NUM_CH = 6
)(
    input  wire                     ACLK,
    input  wire                     ARESETn,

    // ---------------- AXI4 slave : register file ----------------
    input  wire                     s_aw_valid,
    output wire                     s_aw_ready,
    input  wire  [2:0]              s_aw_size,
    input  wire  [1:0]              s_aw_burst,
    input  wire  [7:0]              s_aw_len,
    input  wire  [31:0]             s_aw_addr,

    input  wire                     s_w_valid,
    output wire                     s_w_ready,
    input  wire                     s_w_last,
    input  wire  [31:0]             s_w_data,

    output wire                     s_b_valid,
    input  wire                     s_b_ready,
    output wire  [1:0]              s_b_resp,

    input  wire                     s_ar_valid,
    output wire                     s_ar_ready,
    input  wire  [2:0]              s_ar_size,
    input  wire  [1:0]              s_ar_burst,
    input  wire  [7:0]              s_ar_len,
    input  wire  [31:0]             s_ar_addr,

    output wire                     s_r_valid,
    input  wire                     s_r_ready,
    output wire                     s_r_last,
    output wire  [31:0]             s_r_data,
    output wire  [1:0]              s_r_resp,

    // ---------------- AXI4 master : data engine ----------------
    output wire                     m_aw_valid,
    input  wire                     m_aw_ready,
    output wire  [2:0]              m_aw_size,
    output wire  [1:0]              m_aw_burst,
    output wire  [7:0]              m_aw_len,
    output wire  [31:0]             m_aw_addr,

    output wire                     m_w_valid,
    input  wire                     m_w_ready,
    output wire                     m_w_last,
    output wire  [31:0]             m_w_data,

    input  wire                     m_b_valid,
    output wire                     m_b_ready,
    input  wire  [1:0]              m_b_resp,

    output wire                     m_ar_valid,
    input  wire                     m_ar_ready,
    output wire  [2:0]              m_ar_size,
    output wire  [1:0]              m_ar_burst,
    output wire  [7:0]              m_ar_len,
    output wire  [31:0]             m_ar_addr,

    input  wire                     m_r_valid,
    output wire                     m_r_ready,
    input  wire                     m_r_last,
    input  wire  [31:0]             m_r_data,
    input  wire  [1:0]              m_r_resp,

    output wire                     irq_o
);

    localparam integer BUF_AW = 4;                  // 16 word scratch buffer
    localparam integer BUF_DP = (1 << BUF_AW);

    //================================================================
    // 1. Channel register file  (single driver : the engine block)
    //================================================================
    reg  [31:0]          ch_src  [0:NUM_CH-1];
    reg  [31:0]          ch_dst  [0:NUM_CH-1];
    reg  [31:0]          ch_len  [0:NUM_CH-1];
    reg  [31:0]          ch_cfg  [0:NUM_CH-1];
    reg  [31:0]          ch_rem  [0:NUM_CH-1];
    reg  [NUM_CH-1:0]    ch_en;
    reg  [NUM_CH-1:0]    ch_pend;
    reg  [NUM_CH-1:0]    ch_busy;
    reg  [NUM_CH-1:0]    ch_done;
    reg  [NUM_CH-1:0]    ch_err;
    reg  [NUM_CH-1:0]    irq_en;                    // single driver : reg block
    reg  [NUM_CH-1:0]    irq_stat;                  // single driver : reg block

    //================================================================
    // 2. Engine state
    //================================================================
    localparam [2:0] ST_IDLE = 3'd0,
                     ST_RD   = 3'd1,
                     ST_WR   = 3'd2,
                     ST_B    = 3'd3;

    reg  [2:0]           st;
    reg  [2:0]           cur_ch;
    reg  [31:0]          cur_src;
    reg  [31:0]          cur_dst;
    reg  [31:0]          cur_rem;
    reg  [31:0]          cur_cfg;
    reg  [2:0]           rr_ptr;
    reg  [BUF_AW-1:0]    rd_cnt;
    reg  [BUF_AW-1:0]    wr_cnt;
    reg                  aw_seen;
    reg                  ar_seen;
    reg  [31:0]          bufq [0:BUF_DP-1];
    reg  [NUM_CH-1:0]    start_pulse;

    //================================================================
    // 3. AXI slave channel state  (single driver : the register block)
    //================================================================
    reg          w_act;
    reg          b_pend;
    reg  [31:0]  w_addr;
    reg  [7:0]   w_len;
    reg  [7:0]   w_beat;
    reg  [1:0]   w_burst;
    reg  [2:0]   w_size;

    reg          r_act;
    reg  [31:0]  r_addr;
    reg  [7:0]   r_len;
    reg  [7:0]   r_beat;
    reg  [1:0]   r_burst;
    reg  [2:0]   r_size;

    assign s_aw_ready = ~w_act & ~b_pend;
    assign s_w_ready  =  w_act;
    assign s_b_valid  =  b_pend;
    assign s_b_resp   =  2'b00;
    assign s_ar_ready = ~r_act;
    assign s_r_valid  =  r_act;
    assign s_r_last   = (r_beat == r_len);
    assign s_r_resp   =  2'b00;

    wire aw_hs = s_aw_valid & s_aw_ready;
    wire w_hs  = s_w_valid  & s_w_ready;
    wire b_hs  = s_b_valid  & s_b_ready;
    wire ar_hs = s_ar_valid & s_ar_ready;
    wire r_hs  = s_r_valid  & s_r_ready;

    wire [31:0] w_addr_nxt = (w_burst == 2'b01) ? (w_addr + (32'd1 << w_size)) : w_addr;
    wire [31:0] r_addr_nxt = (r_burst == 2'b01) ? (r_addr + (32'd1 << r_size)) : r_addr;

    //================================================================
    // 4. Register decode (needs engine state, declared above)
    //================================================================
    wire [3:0]  w_ch  = w_addr[11:8];
    wire [7:0]  w_off = w_addr[7:0];
    wire [3:0]  r_ch  = r_addr[11:8];
    wire [7:0]  r_off = r_addr[7:0];

    wire        w_is_ch = (w_ch < NUM_CH);
    wire        w_is_gl = (w_ch == NUM_CH);
    wire        r_is_ch = (r_ch < NUM_CH);
    wire        r_is_gl = (r_ch == NUM_CH);

    wire [2:0]  w_cidx = w_is_ch ? w_ch[2:0] : 3'd0;
    wire [2:0]  r_cidx = r_is_ch ? r_ch[2:0] : 3'd0;

    wire [31:0] r_ctrl_word = {26'b0,
                               ch_err [r_cidx],
                               ch_done[r_cidx],
                               ch_busy[r_cidx],
                               1'b0,
                               irq_en [r_cidx],
                               ch_en  [r_cidx]};

    reg  [31:0] rd_mux;
    always @(*) begin
        rd_mux = 32'h0000_0000;
        if (r_is_gl) begin
            case (r_off)
                8'h00: rd_mux = {{(32-NUM_CH){1'b0}}, irq_stat};
                8'h04: rd_mux = {{(32-NUM_CH){1'b0}}, irq_en};
                8'h0C: rd_mux = 32'h444D_4131;      // "DMA1"
                default: rd_mux = 32'h0000_0000;
            endcase
        end else if (r_is_ch) begin
            case (r_off)
                8'h00: rd_mux = r_ctrl_word;
                8'h04: rd_mux = ch_src[r_cidx];
                8'h08: rd_mux = ch_dst[r_cidx];
                8'h0C: rd_mux = ch_len[r_cidx];
                8'h10: rd_mux = ch_cfg[r_cidx];
                8'h14: rd_mux = (r_cidx == cur_ch) ? cur_rem : ch_rem[r_cidx];
                default: rd_mux = 32'h0000_0000;
            endcase
        end
    end
    assign s_r_data = rd_mux;

    //================================================================
    // 5. Engine combinational helpers
    //================================================================
    wire [1:0] cur_type = cur_cfg[7:6];
    wire cur_src_incr = (cur_type == 2'b10) ? 1'b0 :
                        (cur_type == 2'b00) ? cur_cfg[2] : 1'b1;
    wire cur_dst_incr = (cur_type == 2'b01 || cur_type == 2'b11) ? 1'b0 :
                        (cur_type == 2'b00) ? cur_cfg[4] : 1'b1;

    reg [4:0] cfg_beats;
    always @(*) begin
        case (cur_cfg[1:0])
            2'b00:   cfg_beats = 5'd1;
            2'b01:   cfg_beats = 5'd4;
            2'b10:   cfg_beats = 5'd8;
            default: cfg_beats = 5'd16;
        endcase
    end

    function [4:0] f_room4k;
        input [11:0] off;
        reg   [12:0] left;
        begin
            left = 13'd4096 - {1'b0, off};
            if (left >= 13'd64) f_room4k = 5'd16;
            else                f_room4k = left[6:2];
        end
    endfunction

    function f_multi;
        input [31:0] a;
        begin
            f_multi = (a[31:16] == 16'h2000) |
                      (a[31:16] == 16'h5000) |
                      (a[31:16] == 16'h5001);
        end
    endfunction

    wire [4:0] words_rem = (cur_rem >= 32'd64) ? 5'd16 : cur_rem[5:2];

    reg [4:0] beats;
    always @(*) begin
        beats = cfg_beats;
        if (words_rem < beats) beats = words_rem;
        if (f_room4k(cur_src[11:0]) < beats) beats = f_room4k(cur_src[11:0]);
        if (f_room4k(cur_dst[11:0]) < beats) beats = f_room4k(cur_dst[11:0]);
        if (!f_multi(cur_src) || !f_multi(cur_dst)) beats = 5'd1;
        if (beats == 5'd0) beats = 5'd1;
    end

    wire [6:0] chunk_bytes = {beats, 2'b00};
    wire [31:0] chunk_bytes32 = {25'd0, chunk_bytes};

    function [2:0] f_enc;
        input [NUM_CH-1:0] v;
        integer j;
        begin
            f_enc = 3'd7;
            for (j = 0; j < NUM_CH; j = j + 1)
                if (f_enc == 3'd7 && v[j]) f_enc = j;
        end
    endfunction

    wire [NUM_CH-1:0] one_v    = {{(NUM_CH-1){1'b0}}, 1'b1};
    wire [NUM_CH-1:0] mask_lo  = (one_v << rr_ptr) - one_v;
    wire [NUM_CH-1:0] pend_rot = ch_pend & ~mask_lo;
    wire [2:0]        pick     = (f_enc(pend_rot) != 3'd7) ? f_enc(pend_rot)
                                                           : f_enc(ch_pend);

    wire             irq_set_ev = (st == ST_B) & m_b_valid & m_b_ready &
                                  (cur_rem <= chunk_bytes32);

    //================================================================
    // 6. Master port outputs
    //================================================================
    assign m_ar_valid = (st == ST_RD) & ~ar_seen;
    assign m_ar_addr  = cur_src;
    assign m_ar_size  = 3'd2;
    assign m_ar_burst = cur_src_incr ? 2'b01 : 2'b00;
    assign m_ar_len   = {3'd0, beats} - 8'd1;
    assign m_r_ready  = (st == ST_RD);

    assign m_aw_valid = (st == ST_WR) & ~aw_seen;
    assign m_aw_addr  = cur_dst;
    assign m_aw_size  = 3'd2;
    assign m_aw_burst = cur_dst_incr ? 2'b01 : 2'b00;
    assign m_aw_len   = {3'd0, beats} - 8'd1;
    assign m_w_valid  = (st == ST_WR);
    assign m_w_data   = bufq[wr_cnt];
    assign m_w_last   = (wr_cnt == (beats[BUF_AW-1:0] - 1'b1));
    assign m_b_ready  = (st == ST_B);

    //================================================================
    // 7. Engine  +  channel register file  (one driver for ch_*)
    //================================================================
    integer ci;

    always @(posedge ACLK or negedge ARESETn) begin
        if (~ARESETn) begin
            st          <= ST_IDLE;
            cur_ch      <= 3'd0;
            cur_src     <= 32'd0;
            cur_dst     <= 32'd0;
            cur_rem     <= 32'd0;
            cur_cfg     <= 32'd0;
            rr_ptr      <= 3'd0;
            rd_cnt      <= {BUF_AW{1'b0}};
            wr_cnt      <= {BUF_AW{1'b0}};
            aw_seen     <= 1'b0;
            ar_seen     <= 1'b0;
            ch_en       <= {NUM_CH{1'b0}};
            ch_pend     <= {NUM_CH{1'b0}};
            ch_busy     <= {NUM_CH{1'b0}};
            ch_done     <= {NUM_CH{1'b0}};
            ch_err      <= {NUM_CH{1'b0}};
            start_pulse <= {NUM_CH{1'b0}};
            for (ci = 0; ci < NUM_CH; ci = ci + 1) begin
                ch_src[ci] <= 32'd0;
                ch_dst[ci] <= 32'd0;
                ch_len[ci] <= 32'd0;
                ch_cfg[ci] <= 32'd0;
                ch_rem[ci] <= 32'd0;
            end
        end else begin
            start_pulse <= {NUM_CH{1'b0}};

            //----------------------------------------------------------
            // Engine sequencer
            //----------------------------------------------------------
            case (st)
            ST_IDLE: begin
                if (pick != 3'd7) begin
                    cur_ch      <= pick;
                    cur_src     <= ch_src[pick];
                    cur_dst     <= ch_dst[pick];
                    cur_rem     <= ch_rem[pick];
                    cur_cfg     <= ch_cfg[pick];
                    ch_pend[pick] <= 1'b0;
                    ch_busy[pick] <= 1'b1;
                    rr_ptr      <= (pick == (NUM_CH-1)) ? 3'd0 : (pick + 3'd1);
                    rd_cnt      <= {BUF_AW{1'b0}};
                    wr_cnt      <= {BUF_AW{1'b0}};
                    aw_seen     <= 1'b0;
                    ar_seen     <= 1'b0;
                    st          <= ST_RD;
                end
            end

            ST_RD: begin
                if (m_ar_valid & m_ar_ready) ar_seen <= 1'b1;
                if (m_r_valid & m_r_ready) begin
                    bufq[rd_cnt] <= m_r_data;
                    rd_cnt       <= rd_cnt + 1'b1;
                    if (m_r_last) begin
                        wr_cnt  <= {BUF_AW{1'b0}};
                        aw_seen <= 1'b0;
                        st      <= ST_WR;
                    end
                end
            end

            ST_WR: begin
                if (m_aw_valid & m_aw_ready) aw_seen <= 1'b1;
                if (m_w_valid  & m_w_ready)  wr_cnt  <= wr_cnt + 1'b1;
                if ((aw_seen | (m_aw_valid & m_aw_ready)) &
                    (m_w_valid  & m_w_ready) &
                    (wr_cnt == (beats[BUF_AW-1:0] - 1'b1))) begin
                    st <= ST_B;
                end
            end

            ST_B: begin
                if (m_b_valid & m_b_ready) begin
                    if (cur_src_incr) begin
                        cur_src           <= cur_src + chunk_bytes32;
                        ch_src[cur_ch]    <= cur_src + chunk_bytes32;
                    end
                    if (cur_dst_incr) begin
                        cur_dst           <= cur_dst + chunk_bytes32;
                        ch_dst[cur_ch]    <= cur_dst + chunk_bytes32;
                    end
                    cur_rem <= cur_rem - chunk_bytes32;
                    if (cur_rem <= chunk_bytes32) begin
                        ch_rem [cur_ch] <= 32'd0;
                        ch_busy[cur_ch] <= 1'b0;
                        ch_done[cur_ch] <= 1'b1;
                        st <= ST_IDLE;
                    end else begin
                        ch_rem[cur_ch] <= cur_rem - chunk_bytes32;
                        rd_cnt  <= {BUF_AW{1'b0}};
                        ar_seen <= 1'b0;
                        st      <= ST_RD;
                    end
                end
            end

            default: st <= ST_IDLE;
            endcase

            //----------------------------------------------------------
            // Channel register writes (CTRL / SRC / DST / LEN / CFG)
            //----------------------------------------------------------
            if (w_hs && w_is_ch) begin
                case (w_off)
                    8'h00: begin
                        ch_en[w_cidx] <= s_w_data[0];
                        if (s_w_data[1]) ch_done[w_cidx] <= 1'b0;
                    end
                    8'h04: ch_src[w_cidx] <= s_w_data;
                    8'h08: ch_dst[w_cidx] <= s_w_data;
                    8'h0C: ch_len[w_cidx] <= s_w_data;
                    8'h10: ch_cfg[w_cidx] <= s_w_data;
                    default: ;
                endcase
                // START : arm the channel when it is idle and the setup is sane
                // (only a CTRL write can start a channel, not SRC/DST/LEN/CFG)
                if ((w_off == 8'h00) && s_w_data[1] && !ch_busy[w_cidx] && !ch_pend[w_cidx]) begin
                    if (ch_len[w_cidx][1:0] != 2'b00 ||
                        ch_src [w_cidx][1:0] != 2'b00 ||
                        ch_dst [w_cidx][1:0] != 2'b00 ||
                        ch_len [w_cidx] == 32'd0) begin
                        ch_err[w_cidx] <= 1'b1;
                    end else begin
                        start_pulse[w_cidx] <= 1'b1;
                    end
                end
            end

            // one cycle later : latch the start conditions
            if (start_pulse != {NUM_CH{1'b0}}) begin
                for (ci = 0; ci < NUM_CH; ci = ci + 1) begin
                    if (start_pulse[ci]) begin
                        ch_pend[ci] <= 1'b1;
                        ch_done[ci] <= 1'b0;
                        ch_err [ci] <= 1'b0;
                        ch_rem [ci] <= ch_len[ci];
                    end
                end
            end
        end
    end

    //================================================================
    // 8. Register file : AXI channels + interrupt registers
    //================================================================
    always @(posedge ACLK or negedge ARESETn) begin
        if (~ARESETn) begin
            irq_en   <= {NUM_CH{1'b0}};
            irq_stat <= {NUM_CH{1'b0}};
            w_act    <= 1'b0;
            b_pend   <= 1'b0;
            w_addr   <= 32'd0;
            w_len    <= 8'd0;
            w_beat   <= 8'd0;
            w_burst  <= 2'b00;
            w_size   <= 3'd2;
            r_act    <= 1'b0;
            r_addr   <= 32'd0;
            r_len    <= 8'd0;
            r_beat   <= 8'd0;
            r_burst  <= 2'b00;
            r_size   <= 3'd2;
        end else begin
            //---------------- write channel ----------------
            if (aw_hs) begin
                w_act   <= 1'b1;
                w_addr  <= s_aw_addr;
                w_len   <= s_aw_len;
                w_beat  <= 8'd0;
                w_burst <= s_aw_burst;
                w_size  <= s_aw_size;
            end
            if (w_hs) begin
                w_beat <= w_beat + 8'd1;
                w_addr <= w_addr_nxt;
                if (w_beat == w_len) begin
                    w_act  <= 1'b0;
                    b_pend <= 1'b1;
                end
                if (w_is_ch && (w_off == 8'h00)) irq_en[w_cidx] <= s_w_data[2];
                if (w_is_gl) begin
                    case (w_off)
                        8'h00: irq_stat <= irq_stat & ~s_w_data[NUM_CH-1:0];
                        8'h04: irq_en    <=  s_w_data[NUM_CH-1:0];
                        8'h08: irq_stat  <= irq_stat & ~s_w_data[NUM_CH-1:0];
                        default: ;
                    endcase
                end
            end
            if (b_hs) b_pend <= 1'b0;

            //---------------- read channel -----------------
            if (ar_hs) begin
                r_act   <= 1'b1;
                r_addr  <= s_ar_addr;
                r_len   <= s_ar_len;
                r_beat  <= 8'd0;
                r_burst <= s_ar_burst;
                r_size  <= s_ar_size;
            end
            if (r_hs) begin
                r_beat <= r_beat + 8'd1;
                r_addr <= r_addr_nxt;
                if (r_beat == r_len) r_act <= 1'b0;
            end

            //---------------- interrupt status --------------
            if (irq_set_ev)
                irq_stat <= irq_stat | (({{(32-NUM_CH){1'b0}}, 1'b1} << cur_ch) & {{(32-NUM_CH){1'b0}}, {NUM_CH{1'b1}}});
        end
    end

    assign irq_o = |(irq_stat & irq_en);

endmodule
