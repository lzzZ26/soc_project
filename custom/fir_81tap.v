`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// fir_81tap.v  (file/module name kept for compatibility)
//
// 82-tap (81st order) FIR filter supporting arbitrary non-symmetric
// coefficients.
//
//   Architecture : 82 parallel multipliers, split by synthesis attribute:
//                    taps  0..40 : 41 multipliers inferred as APM
//                    taps 41..81 : 41 multipliers forced to LUT logic
//                  followed by a fully pipelined binary adder tree.
//   Throughput   : 1 output sample per clock (1 sample/cycle).
//   Latency      : 9 clock cycles.
//   Coefficients : 82 x 16-bit signed (Q1.15), runtime writable.
//   Data width   : 16-bit signed input/output (Q1.15).
//
//   The 41 LUT multipliers carry (* syn_multstyle = "logic" *), the
//   Synplify/PDS attribute that keeps them out of the APM blocks.  The
//   41 APM multipliers use the default block_mult mapping.
//
//   Interface matches axi_fir_wrap.v expectations exactly:
//     clk, rst_n, in_valid/ready, out_valid, out_data,
//     coef_wr_en/idx/wdata/rdata.
//----------------------------------------------------------------------------

// 16x16 signed multiplier inferred as an APM/DSP block.
module fir_mul_apm #(
    parameter integer W = 16
) (
    input  wire signed [W-1:0]     a,
    input  wire signed [W-1:0]     b,
    output wire signed [2*W-1:0]   p
);
  assign p = a * b;
endmodule

// 16x16 signed multiplier forced into LUT logic.
(* syn_multstyle = "logic" *)
module fir_mul_lut #(
    parameter integer W = 16
) (
    input  wire signed [W-1:0]     a,
    input  wire signed [W-1:0]     b,
    output wire signed [2*W-1:0]   p
);
  assign p = a * b;
endmodule

module fir_81tap #(
    parameter integer DATA_W = 16,
    parameter integer COEF_W = 16
) (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      clr,
    input  wire                      in_valid,
    input  wire signed [DATA_W-1:0]  in_data,
    output wire                      in_ready,
    output wire                      out_valid,
    output reg  signed [DATA_W-1:0]  out_data,
    input  wire                      coef_wr_en,
    input  wire [6:0]                coef_idx,
    input  wire signed [COEF_W-1:0]  coef_wdata,
    output wire signed [COEF_W-1:0]  coef_rdata
);

  localparam integer TAPS   = 82;
  localparam integer PROD_W = DATA_W + COEF_W;  // 32

  // Adder-tree level widths.  Each level adds one carry bit.
  localparam integer L1_W = PROD_W + 1;  // 33
  localparam integer L2_W = PROD_W + 2;  // 34
  localparam integer L3_W = PROD_W + 3;  // 35
  localparam integer L4_W = PROD_W + 4;  // 36
  localparam integer L5_W = PROD_W + 5;  // 37
  localparam integer L6_W = PROD_W + 6;  // 38
  localparam integer L7_W = PROD_W + 7;  // 39

  localparam integer ACC_W = 41;
  localparam signed [ACC_W-1:0] ROUND_CONST = (1 << (COEF_W - 2)); // 2^14
  localparam signed [ACC_W-1:0] SAT_MAX     =  32767;
  localparam signed [ACC_W-1:0] SAT_MIN     = -32768;

`include "fir_coeffs.vh"

  assign in_ready = 1'b1;  // one sample accepted every clock

  //-------------------------------------------------------------------------
  // Coefficient RAM: 82 x 16, initialised from fir_coeffs.vh at reset.
  // The CPU can rewrite any coefficient through COEF_INDEX/COEF_DATA.
  //-------------------------------------------------------------------------
  reg signed [COEF_W-1:0] coef_q [0:TAPS-1];
  integer ci;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (ci = 0; ci < TAPS; ci = ci + 1)
        coef_q[ci] <= fir_coeff(ci);
    end else if (coef_wr_en && (coef_idx < TAPS)) begin
      coef_q[coef_idx] <= coef_wdata;
    end
  end
  assign coef_rdata = coef_q[coef_idx];

  //-------------------------------------------------------------------------
  // Input sample shift register: x_sr[i] = x[n-i], depth TAPS.
  //-------------------------------------------------------------------------
  reg signed [DATA_W-1:0] x_sr [0:TAPS-1];
  integer si;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || clr) begin
      for (si = 0; si < TAPS; si = si + 1)
        x_sr[si] <= {DATA_W{1'b0}};
    end else if (in_valid) begin
      x_sr[0] <= in_data;
      for (si = 1; si < TAPS; si = si + 1)
        x_sr[si] <= x_sr[si - 1];
    end
  end

  //-------------------------------------------------------------------------
  // 82 parallel products: 41 APM + 41 LUT.
  //-------------------------------------------------------------------------
  wire signed [PROD_W-1:0] prod [0:TAPS-1];

  genvar gk, gl;
  generate
    for (gk = 0; gk < 41; gk = gk + 1) begin : g_apm
      fir_mul_apm #(.W(DATA_W)) u_mul (
        .a (x_sr[gk]),
        .b (coef_q[gk]),
        .p (prod[gk])
      );
    end
    for (gl = 41; gl < 82; gl = gl + 1) begin : g_lut
      (* syn_multstyle = "logic" *)
      fir_mul_lut #(.W(DATA_W)) u_mul (
        .a (x_sr[gl]),
        .b (coef_q[gl]),
        .p (prod[gl])
      );
    end
  endgenerate

  //-------------------------------------------------------------------------
  // Product pipeline register (stage 1).
  //-------------------------------------------------------------------------
  reg signed [PROD_W-1:0] p1 [0:TAPS-1];
  integer pi;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || clr) begin
      for (pi = 0; pi < TAPS; pi = pi + 1)
        p1[pi] <= {PROD_W{1'b0}};
    end else begin
      for (pi = 0; pi < TAPS; pi = pi + 1)
        p1[pi] <= prod[pi];
    end
  end

  //-------------------------------------------------------------------------
  // Pipelined binary adder tree.
  //   L1: 82 -> 41 (33 bit)   L2: 41 -> 21 (34 bit)
  //   L3: 21 -> 11 (35 bit)   L4: 11 ->  6 (36 bit)
  //   L5:  6 ->  3 (37 bit)   L6:  3 ->  2 (38 bit)
  //   L7:  2 ->  1 (39 bit)
  // Every level is registered, so the critical path is one 16x16
  // multiplier plus one adder.
  //-------------------------------------------------------------------------
  wire signed [L1_W-1:0] l1 [0:40];
  wire signed [L2_W-1:0] l2 [0:20];
  wire signed [L3_W-1:0] l3 [0:10];
  wire signed [L4_W-1:0] l4 [0:5];
  wire signed [L5_W-1:0] l5 [0:2];
  wire signed [L6_W-1:0] l6 [0:1];
  wire signed [L7_W-1:0] l7;

  // Registers between tree levels (stages 2..8).
  reg signed [L1_W-1:0] l1_r [0:40];
  reg signed [L2_W-1:0] l2_r [0:20];
  reg signed [L3_W-1:0] l3_r [0:10];
  reg signed [L4_W-1:0] l4_r [0:5];
  reg signed [L5_W-1:0] l5_r [0:2];
  reg signed [L6_W-1:0] l6_r [0:1];
  reg signed [L7_W-1:0] l7_r;

  genvar g1, g2, g3, g4, g5, g6;
  generate
    for (g1 = 0; g1 < 41; g1 = g1 + 1) begin : g_l1
      assign l1[g1] = {p1[2*g1][PROD_W-1], p1[2*g1]} +
                      {p1[2*g1+1][PROD_W-1], p1[2*g1+1]};
    end

    for (g2 = 0; g2 < 20; g2 = g2 + 1) begin : g_l2
      assign l2[g2] = {l1_r[2*g2][L1_W-1], l1_r[2*g2]} +
                      {l1_r[2*g2+1][L1_W-1], l1_r[2*g2+1]};
    end
    assign l2[20] = {l1_r[40][L1_W-1], l1_r[40]};

    for (g3 = 0; g3 < 10; g3 = g3 + 1) begin : g_l3
      assign l3[g3] = {l2_r[2*g3][L2_W-1], l2_r[2*g3]} +
                      {l2_r[2*g3+1][L2_W-1], l2_r[2*g3+1]};
    end
    assign l3[10] = {l2_r[20][L2_W-1], l2_r[20]};

    for (g4 = 0; g4 < 5; g4 = g4 + 1) begin : g_l4
      assign l4[g4] = {l3_r[2*g4][L3_W-1], l3_r[2*g4]} +
                      {l3_r[2*g4+1][L3_W-1], l3_r[2*g4+1]};
    end
    assign l4[5] = {l3_r[10][L3_W-1], l3_r[10]};

    for (g5 = 0; g5 < 3; g5 = g5 + 1) begin : g_l5
      assign l5[g5] = {l4_r[2*g5][L4_W-1], l4_r[2*g5]} +
                      {l4_r[2*g5+1][L4_W-1], l4_r[2*g5+1]};
    end

    for (g6 = 0; g6 < 1; g6 = g6 + 1) begin : g_l6
      assign l6[0] = {l5_r[0][L5_W-1], l5_r[0]} +
                     {l5_r[1][L5_W-1], l5_r[1]};
    end
    assign l6[1] = {l5_r[2][L5_W-1], l5_r[2]};

    assign l7 = {l6_r[0][L6_W-1], l6_r[0]} +
                {l6_r[1][L6_W-1], l6_r[1]};
  endgenerate

  integer q;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || clr) begin
      for (q = 0; q < 41; q = q + 1) l1_r[q] <= {L1_W{1'b0}};
      for (q = 0; q < 21; q = q + 1) l2_r[q] <= {L2_W{1'b0}};
      for (q = 0; q < 11; q = q + 1) l3_r[q] <= {L3_W{1'b0}};
      for (q = 0; q <  6; q = q + 1) l4_r[q] <= {L4_W{1'b0}};
      for (q = 0; q <  3; q = q + 1) l5_r[q] <= {L5_W{1'b0}};
      for (q = 0; q <  2; q = q + 1) l6_r[q] <= {L6_W{1'b0}};
      l7_r <= {L7_W{1'b0}};
    end else begin
      for (q = 0; q < 41; q = q + 1) l1_r[q] <= l1[q];
      for (q = 0; q < 21; q = q + 1) l2_r[q] <= l2[q];
      for (q = 0; q < 11; q = q + 1) l3_r[q] <= l3[q];
      for (q = 0; q <  6; q = q + 1) l4_r[q] <= l4[q];
      for (q = 0; q <  3; q = q + 1) l5_r[q] <= l5[q];
      for (q = 0; q <  2; q = q + 1) l6_r[q] <= l6[q];
      l7_r <= l7;
    end
  end

  //-------------------------------------------------------------------------
  // Output rounding / saturation (stage 9).
  //-------------------------------------------------------------------------
  wire signed [ACC_W-1:0] sum_ext =
      {{(ACC_W-L7_W){l7_r[L7_W-1]}}, l7_r};
  wire signed [ACC_W-1:0] sh =
      (sum_ext + ROUND_CONST) >>> (COEF_W - 1);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || clr) begin
      out_data <= {DATA_W{1'b0}};
    end else if (vld[8]) begin
      if (sh > SAT_MAX)
        out_data <= 16'sd32767;
      else if (sh < SAT_MIN)
        out_data <= -16'sd32768;
      else
        out_data <= sh[DATA_W-1:0];
    end
  end

  //-------------------------------------------------------------------------
  // Valid pipeline: 9 stages, matching the 9-cycle data latency.
  //-------------------------------------------------------------------------
  reg [8:0] vld;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || clr)
      vld <= 9'd0;
    else
      vld <= {vld[7:0], in_valid};
  end

  // Register out_valid one cycle after the final tree register so that
  // out_valid and the registered out_data are aligned for downstream
  // posedge sampling (AXI wrapper / output FIFO).
  reg out_valid_r;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || clr)
      out_valid_r <= 1'b0;
    else
      out_valid_r <= vld[8];
  end
  assign out_valid = out_valid_r;

endmodule
