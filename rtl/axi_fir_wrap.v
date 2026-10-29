//------------------------------------------------------------------------------
// axi_fir_wrap.v
//
// AXI4 slave wrapper around the 82 tap (81st order) fir_81tap core.
//
//   * proper AXI4 slave : AW -> W -> B   /   AR -> R, INCR and FIXED bursts
//   * input  sample FIFO (write DATA_IN)  -> fir_81tap.in_*
//   * output sample FIFO (read  DATA_OUT) <- fir_81tap.out_*
//   * runtime coefficient load / read back
//
// Register map (byte offsets from 0x5000_0000)
// ---------------------------------------------------------------
//   0x00  CTRL        w  [0] EN     enable the filter
//                        [1] CLR    flush both FIFOs and counters
//                        [2] IRQ_EN
//                      r  same bits
//   0x04  STATUS      r  [0] BUSY      operation in progress
//                        [1] DONE      every sample pushed has been read out
//                        [2] IN_FULL   input FIFO full
//                        [3] OUT_EMPTY output FIFO empty
//                        [4] OUT_OVF   output FIFO overflow (sticky)
//                        [5] IN_UDF    DATA_OUT read while empty (sticky)
//                        [15:8]  IN_CNT   samples in input  FIFO (sat 255)
//                        [23:16] OUT_CNT  samples in output FIFO (sat 255)
//   0x08  DATA_IN     w  [15:0] push one input sample
//                      r  last sample pushed
//   0x0C  DATA_OUT    r  [15:0] pop one output sample (sign extended)
//   0x10  IRQ_STAT    r  [0] result ready      w1c
//   0x14  IRQ_EN      rw [0]
//   0x18  COEF_INDEX  rw [6:0] 0..81
//   0x1C  COEF_DATA   w  write coef[COEF_INDEX] then auto increment
//                      r  read  coef[COEF_INDEX]
//   0x20  COEF_NUM    r  82
//   0x24  IN_CNT_TOT  r  total samples pushed
//   0x28  OUT_CNT_TOT r  total samples popped
//
// DATA_IN / DATA_OUT are register based, so the DMA streams them with a FIXED
// burst (every beat lands on the same offset) and INCR bursts work too.
//------------------------------------------------------------------------------
`timescale 1ns / 1ps

module axi_fir_wrap (
    input  wire                     ACLK,
    input  wire                     ARESETn,

    // ---- AXI4 slave ----
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

    // ---- FIR core ----
    output wire                     fir_in_valid,
    input  wire                     fir_in_ready,
    output wire signed [15:0]       fir_in_data,
    input  wire                     fir_out_valid,
    input  wire signed [15:0]       fir_out_data,

    output wire                     coef_wr_en,
    output wire [6:0]               coef_idx,
    output wire signed [15:0]       coef_wdata,
    input  wire signed [15:0]       coef_rdata,

    // ---- FIR core clear (CTRL bit1 pulse) ----
    output wire                     fir_clr,

    // ---- interrupt ----
    output wire                     irq_o
);

    localparam integer FIFO_AW = 9;                 // 512 entries
    localparam integer FIFO_DP = (1 << FIFO_AW);

    //------------------------------------------------------------------
    // Register file
    //------------------------------------------------------------------
    reg          r_en;
    reg          r_irq_en;
    reg          r_irq_stat;
    reg  [6:0]   r_coef_idx;
    reg          r_out_ovf;
    reg          r_in_udf;
    reg  [31:0]  r_in_tot;
    reg  [31:0]  r_out_tot;
    reg  [15:0]  r_last_in;

    //------------------------------------------------------------------
    // Input sample FIFO
    //------------------------------------------------------------------
    reg  [15:0]          in_mem  [0:FIFO_DP-1];
    reg  [FIFO_AW-1:0]   in_wptr, in_rptr;
    reg  [FIFO_AW:0]     in_cnt;
    wire in_empty = (in_cnt == 0);
    wire in_full  = (in_cnt == FIFO_DP);

    //------------------------------------------------------------------
    // Output sample FIFO
    //------------------------------------------------------------------
    reg  [15:0]          out_mem [0:FIFO_DP-1];
    reg  [FIFO_AW-1:0]   out_wptr, out_rptr;
    reg  [FIFO_AW:0]     out_cnt;
    wire out_empty = (out_cnt == 0);
    wire out_full  = (out_cnt == FIFO_DP);
    // stop feeding the core when the output FIFO is close to full
    wire out_afull = (out_cnt >= (FIFO_DP - 16));

    //------------------------------------------------------------------
    // AXI write channel :  AW -> W -> B
    //------------------------------------------------------------------
    reg          w_act;
    reg          b_pend;
    reg  [31:0]  w_addr;
    reg  [7:0]   w_len;
    reg  [7:0]   w_beat;
    reg  [1:0]   w_burst;
    reg  [2:0]   w_size;

    assign s_aw_ready = ~w_act & ~b_pend;
    assign s_w_ready  =  w_act;
    assign s_b_valid  =  b_pend;
    assign s_b_resp   =  2'b00;

    wire aw_hs = s_aw_valid & s_aw_ready;
    wire w_hs  = s_w_valid  & s_w_ready;
    wire b_hs  = s_b_valid  & s_b_ready;

    wire [31:0] w_inc   = (32'd1 << w_size);
    wire [31:0] w_addr_nxt = (w_burst == 2'b01) ? (w_addr + w_inc) : w_addr;

    //------------------------------------------------------------------
    // AXI read channel :  AR -> R
    //------------------------------------------------------------------
    reg          r_act;
    reg  [31:0]  r_addr;
    reg  [7:0]   r_len;
    reg  [7:0]   r_beat;
    reg  [1:0]   r_burst;
    reg  [2:0]   r_size;

    assign s_ar_ready = ~r_act;
    assign s_r_valid  =  r_act;
    assign s_r_last   = (r_beat == r_len);
    assign s_r_resp   =  2'b00;

    wire ar_hs = s_ar_valid & s_ar_ready;
    wire r_hs  = s_r_valid  & s_r_ready;

    wire [31:0] r_inc   = (32'd1 << r_size);
    wire [31:0] r_addr_nxt = (r_burst == 2'b01) ? (r_addr + r_inc) : r_addr;

    //------------------------------------------------------------------
    // Decode of the address currently presented on each channel
    //------------------------------------------------------------------
    wire [7:0]  off_w = w_addr[7:0];
    wire [7:0]  off_r = r_addr[7:0];

    wire        wr_ctrl  = w_hs & (off_w == 8'h00);
    wire        clr      = wr_ctrl & s_w_data[1];
    assign fir_clr = clr;

    wire [7:0]  in_cnt_sat  = (in_cnt  > 255) ? 8'd255 : in_cnt [7:0];
    wire [7:0]  out_cnt_sat = (out_cnt > 255) ? 8'd255 : out_cnt[7:0];
    wire        fir_done    = (r_in_tot == r_out_tot) & (r_in_tot != 32'd0);

    //------------------------------------------------------------------
    // Register read mux (combinational, stable while r_act is high)
    //------------------------------------------------------------------
    assign s_r_data =
        (off_r == 8'h00) ? {29'b0, r_irq_en, 1'b0, r_en} :
        (off_r == 8'h04) ? { 8'b0, out_cnt_sat, in_cnt_sat,
                             2'b00, r_in_udf, r_out_ovf,
                             out_empty, in_full,
                             fir_done, (r_in_tot != r_out_tot) } :
        (off_r == 8'h08) ? {16'b0, r_last_in} :
        (off_r == 8'h0C) ? (out_empty ? 32'h0000_0000
                                      : {{16{out_mem[out_rptr][15]}}, out_mem[out_rptr]}) :
        (off_r == 8'h10) ? {31'b0, r_irq_stat} :
        (off_r == 8'h14) ? {31'b0, r_irq_en} :
        (off_r == 8'h18) ? {25'b0, r_coef_idx} :
        (off_r == 8'h1C) ? {{16{coef_rdata[15]}}, coef_rdata} :
        (off_r == 8'h20) ? 32'd82 :
        (off_r == 8'h24) ? r_in_tot :
        (off_r == 8'h28) ? r_out_tot :
                           32'h0000_0000;

    //------------------------------------------------------------------
    // Coefficient port : COEF_INDEX is the shared read/write index
    //------------------------------------------------------------------
    assign coef_idx   = r_coef_idx;
    assign coef_wr_en = w_hs & (off_w == 8'h1C);
    assign coef_wdata = s_w_data[15:0];

    //------------------------------------------------------------------
    // Sample flow handshakes
    //------------------------------------------------------------------
    assign fir_in_valid = r_en & ~in_empty & ~out_afull;
    assign fir_in_data  = in_mem[in_rptr];

    wire core_fire = fir_in_valid & fir_in_ready;              // input pop

    wire push_in   = w_hs & (off_w == 8'h08) & ~in_full;
    wire pop_out   = r_hs & (off_r == 8'h0C) & ~out_empty;
    wire push_out  = fir_out_valid & (~out_full | pop_out);

    wire out_ovf_ev = fir_out_valid & out_full & ~pop_out;
    wire in_udf_ev  = r_hs & (off_r == 8'h0C) & out_empty;

    //------------------------------------------------------------------
    // AXI channels + register file
    //------------------------------------------------------------------
    always @(posedge ACLK or negedge ARESETn) begin
        if (~ARESETn) begin
            r_en       <= 1'b0;
            r_irq_en   <= 1'b0;
            r_irq_stat <= 1'b0;
            r_coef_idx <= 7'd0;
            r_out_ovf  <= 1'b0;
            r_in_udf   <= 1'b0;
            r_in_tot   <= 32'd0;
            r_out_tot  <= 32'd0;
            r_last_in  <= 16'd0;
            w_act      <= 1'b0;
            b_pend     <= 1'b0;
            w_addr     <= 32'd0;
            w_len      <= 8'd0;
            w_beat     <= 8'd0;
            w_burst    <= 2'b00;
            w_size     <= 3'd2;
            r_act      <= 1'b0;
            r_addr     <= 32'd0;
            r_len      <= 8'd0;
            r_beat     <= 8'd0;
            r_burst    <= 2'b00;
            r_size     <= 3'd2;
        end else begin
            //---------------- write address / data / response -------------
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
                case (off_w)
                    8'h00: begin r_en <= s_w_data[0]; r_irq_en <= s_w_data[2]; end
                    8'h10: r_irq_stat <= r_irq_stat & ~s_w_data[0];
                    8'h14: r_irq_en   <= s_w_data[0];
                    8'h18: r_coef_idx <= s_w_data[6:0];
                    8'h1C: if (r_coef_idx < 7'd81)
                               r_coef_idx <= r_coef_idx + 7'd1;
                    default: ;
                endcase
            end
            if (b_hs) b_pend <= 1'b0;

            //---------------- read address / data --------------------------
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

            //---------------- counters / sticky flags ----------------------
            if (clr) begin
                r_in_tot   <= 32'd0;
                r_out_tot  <= 32'd0;
                r_out_ovf  <= 1'b0;
                r_in_udf   <= 1'b0;
                r_last_in  <= 16'd0;
            end else begin
                if (push_in) begin
                    r_in_tot  <= r_in_tot + 32'd1;
                    r_last_in <= s_w_data[15:0];
                end
                if (pop_out)  r_out_tot <= r_out_tot + 32'd1;
                if (out_ovf_ev) r_out_ovf <= 1'b1;
                if (in_udf_ev)  r_in_udf  <= 1'b1;
            end

            // first result became available -> sticky IRQ flag
            if (push_out & out_empty & ~pop_out) r_irq_stat <= 1'b1;
        end
    end

    //------------------------------------------------------------------
    // Input sample FIFO
    //------------------------------------------------------------------
    always @(posedge ACLK or negedge ARESETn) begin
        if (~ARESETn) begin
            in_wptr <= {FIFO_AW{1'b0}};
            in_rptr <= {FIFO_AW{1'b0}};
            in_cnt  <= {(FIFO_AW+1){1'b0}};
        end else if (clr) begin
            in_wptr <= {FIFO_AW{1'b0}};
            in_rptr <= {FIFO_AW{1'b0}};
            in_cnt  <= {(FIFO_AW+1){1'b0}};
        end else begin
            if (push_in) begin
                in_mem[in_wptr] <= s_w_data[15:0];
                in_wptr         <= in_wptr + 1'b1;
            end
            if (core_fire) in_rptr <= in_rptr + 1'b1;
            in_cnt <= in_cnt + (push_in ? 1'b1 : 1'b0) - (core_fire ? 1'b1 : 1'b0);
        end
    end

    //------------------------------------------------------------------
    // Output sample FIFO
    //------------------------------------------------------------------
    always @(posedge ACLK or negedge ARESETn) begin
        if (~ARESETn) begin
            out_wptr <= {FIFO_AW{1'b0}};
            out_rptr <= {FIFO_AW{1'b0}};
            out_cnt  <= {(FIFO_AW+1){1'b0}};
        end else if (clr) begin
            out_wptr <= {FIFO_AW{1'b0}};
            out_rptr <= {FIFO_AW{1'b0}};
            out_cnt  <= {(FIFO_AW+1){1'b0}};
        end else begin
            if (push_out) begin
                out_mem[out_wptr] <= fir_out_data[15:0];
                out_wptr          <= out_wptr + 1'b1;
            end
            if (pop_out) out_rptr <= out_rptr + 1'b1;
            out_cnt <= out_cnt + (push_out ? 1'b1 : 1'b0) - (pop_out ? 1'b1 : 1'b0);
        end
    end

    assign irq_o = r_irq_en & r_irq_stat;

endmodule
