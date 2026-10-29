`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_fir_82tap_parallel.v
//
// Self-checking testbench for the 82-tap parallel FIR:
//   * writes 82 arbitrary (non-symmetric) coefficients
//   * drives N samples back-to-back at 1 sample/cycle
//   * captures N outputs and compares against a 64-bit behavioral model
//     with the same Q1.15 rounding and saturation
//   * checks that outputs come out at 1 sample/cycle (no gaps)
//
// Run:
//   iverilog -g2001 -I custom -o tb_fir_82tap.vvp \
//            custom/fir_81tap.v custom/tb_fir_82tap_parallel.v
//   vvp tb_fir_82tap.vvp
//----------------------------------------------------------------------------
module tb_fir_82tap_parallel;

  localparam integer TAPS = 82;
  localparam integer N    = 400;

  reg                    clk = 1'b0;
  reg                    rst_n = 1'b0;
  reg                    in_valid = 1'b0;
  reg  signed [15:0]     in_data = 16'sd0;
  wire                   in_ready;
  wire                   out_valid;
  wire signed [15:0]     out_data;
  reg                    coef_wr_en = 1'b0;
  reg  [6:0]             coef_idx = 7'd0;
  reg  signed [15:0]     coef_wdata = 16'sd0;
  wire signed [15:0]     coef_rdata;

  fir_81tap dut (
    .clk        (clk),
    .rst_n      (rst_n),
    .clr        (1'b0),
    .in_valid   (in_valid),
    .in_data    (in_data),
    .in_ready   (in_ready),
    .out_valid  (out_valid),
    .out_data   (out_data),
    .coef_wr_en (coef_wr_en),
    .coef_idx   (coef_idx),
    .coef_wdata (coef_wdata),
    .coef_rdata (coef_rdata)
  );

  always #10 clk = ~clk;   // 50 MHz

  reg signed [15:0] coef [0:TAPS-1];
  reg signed [15:0] stim [0:N-1];
  reg signed [15:0] got  [0:N-1];

  integer out_cnt = 0;
  integer cycle_cnt = 0;
  integer first_cycle = -1;
  integer last_cycle = -1;

  reg prev_out_valid = 1'b0;
  integer throughput_errors = 0;

  always @(posedge clk) begin
    if (rst_n) begin
      cycle_cnt <= cycle_cnt + 1;
      prev_out_valid <= out_valid;
      if (out_valid) begin
        if (out_cnt < N)
          got[out_cnt] = out_data;
        if (out_cnt == 0)
          first_cycle <= cycle_cnt;
        last_cycle <= cycle_cnt;
        if ((out_cnt > 0) && (prev_out_valid !== 1'b1))
          throughput_errors = throughput_errors + 1;
        out_cnt = out_cnt + 1;
      end
    end
  end

  integer i, k, n, errors;
  reg signed [63:0] acc;
  reg signed [63:0] sh;
  reg signed [63:0] exp_val;

  initial begin
    // Arbitrary non-symmetric coefficients and stimulus.
    for (i = 0; i < TAPS; i = i + 1)
      coef[i] = (i * 37 + 11) % 2001 - 1000;
    for (i = 0; i < N; i = i + 1)
      stim[i] = (i * 97 + 23) % 4001 - 2000;

    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);

    // Load all 82 coefficients.
    for (i = 0; i < TAPS; i = i + 1) begin
      @(negedge clk);
      coef_wr_en = 1'b1;
      coef_idx   = i[6:0];
      coef_wdata = coef[i];
    end
    @(negedge clk);
    coef_wr_en = 1'b0;

    // Drive N samples back-to-back.
    for (n = 0; n < N; n = n + 1) begin
      @(negedge clk);
      in_valid = 1'b1;
      in_data  = stim[n];
    end
    @(negedge clk);
    in_valid = 1'b0;
    in_data  = 16'sd0;

    // Wait for all outputs (one extra edge lets the NBA updates settle).
    wait (out_cnt == N);
    @(posedge clk);

    // Compare against a 64-bit behavioral model.
    errors = 0;
    for (n = 0; n < N; n = n + 1) begin
      acc = 64'sd0;
      for (k = 0; k < TAPS; k = k + 1)
        if (n - k >= 0)
          acc = acc + $signed(stim[n-k]) * $signed(coef[k]);

      sh = (acc + 64'sd16384) >>> 15;
      if (sh > 64'sd32767)
        exp_val = 64'sd32767;
      else if (sh < -64'sd32768)
        exp_val = -64'sd32768;
      else
        exp_val = sh;

      if (got[n] !== exp_val[15:0]) begin
        errors = errors + 1;
        if (errors <= 10)
          $display("MISMATCH n=%0d got=%0d exp=%0d", n, got[n], $signed(exp_val[15:0]));
      end
    end

    $display("");
    $display("=========================================================");
    $display("  captured outputs : %0d / %0d", out_cnt, N);
    $display("  math mismatches  : %0d", errors);
    $display("  throughput gaps  : %0d", throughput_errors);
    if ((first_cycle >= 0) && (last_cycle >= 0))
      $display("  output span      : %0d cycles for %0d samples",
               last_cycle - first_cycle + 1, N);
    if ((errors == 0) && (throughput_errors == 0) &&
        ((last_cycle - first_cycle + 1) == N) && (out_cnt == N))
      $display("  RESULT: PASS (82 taps, 1 sample/cycle, bit-exact)");
    else
      $display("  RESULT: FAIL");
    $display("=========================================================");
    $finish;
  end

endmodule
