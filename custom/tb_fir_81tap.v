`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_fir_81tap.v -- unit level verification for fir_81tap
//
//   * reads fir_stim.hex      : 5404 input samples
//   * reads fir_ref_fx.hex    : bit accurate fixed point reference (must match 100%)
//   * reads fir_golden.hex    : float reference rounded to int16  (<0.1 % criterion)
//   * runs the whole vector set twice:
//       pass 0 : back to back, 1 sample / cycle (the throughput requirement)
//       pass 1 : with bubbles, checks there is no dependence on a free running input
//
//   expected: ./run_fir.sh
//----------------------------------------------------------------------------
module tb_fir_81tap;

  `include "fir_coeffs.vh"

  localparam integer SCALE = 32768;
  // 0.1 % of full scale = 32.768 LSB  ->  pass while worst * 1000 < SCALE

  reg               clk   = 1'b0;
  reg               rst_n = 1'b0;
  reg               in_valid = 1'b0;
  reg signed [15:0] in_data  = 16'sd0;
  wire              in_ready;
  wire              out_valid;
  wire signed [15:0] out_data;

  fir_81tap dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .in_valid  (in_valid),
    .in_data   (in_data),
    .in_ready  (in_ready),
    .out_valid (out_valid),
    .out_data  (out_data),
    .coef_wr_en(1'b0),
    .coef_idx  (7'd0),
    .coef_wdata(16'sd0),
    .coef_rdata()
  );

  always #10 clk = ~clk;                 // 50 MHz

  //-------------------------------------------------------------------------
  // memories
  //-------------------------------------------------------------------------
  reg signed [15:0] stim   [0:FIR_NSAMP-1];
  reg signed [15:0] ref_fx [0:FIR_NSAMP-1];
  reg signed [15:0] ref_fl [0:FIR_NSAMP-1];
  reg signed [15:0] cap    [0:FIR_NSAMP-1];

  //-------------------------------------------------------------------------
  // capture
  //-------------------------------------------------------------------------
  integer cap_n = 0;
  reg     cap_en = 1'b0;

  always @(posedge clk) begin
    if (rst_n && cap_en && out_valid) begin
      cap[cap_n] = out_data;
      cap_n      = cap_n + 1;
    end
  end

  //-------------------------------------------------------------------------
  // stimulus driver : one sample per clock when gap == 0
  //-------------------------------------------------------------------------
  task drive;
    input integer gap;
    integer i;
    begin
      for (i = 0; i < FIR_NSAMP; i = i + 1) begin
        @(negedge clk);
        in_data  = stim[i];
        in_valid = 1'b1;
        if (gap && ((i % 4) == 3)) begin
          @(negedge clk);
          in_valid = 1'b0;
        end
      end
      @(negedge clk);
      in_valid = 1'b0;
      in_data  = 16'sd0;
    end
  endtask

  //-------------------------------------------------------------------------
  // checker
  //-------------------------------------------------------------------------
  integer worst_fx, worst_fl, mism_fx, mism_fl;
  reg     pass_ok;

  task check;
    input integer p;
    integer i, d;
    begin
      worst_fx = 0;  worst_fl = 0;
      mism_fx  = 0;  mism_fl  = 0;
      for (i = 0; i < FIR_NSAMP; i = i + 1) begin
        d = $signed(cap[i]) - $signed(ref_fx[i]);
        if (d < 0) d = -d;
        if (d != 0) mism_fx = mism_fx + 1;
        if (d > worst_fx) worst_fx = d;

        d = $signed(cap[i]) - $signed(ref_fl[i]);
        if (d < 0) d = -d;
        if (d != 0) mism_fl = mism_fl + 1;
        if (d > worst_fl) worst_fl = d;
      end
      $display("");
      $display("--- pass %0d : %0d samples captured (expected %0d) ---",
               p, cap_n, FIR_NSAMP);
      $display("    vs bit-accurate fixed-point model : %0d mismatch, worst %0d LSB",
               mism_fx, worst_fx);
      $display("    vs float reference                : %0d mismatch, worst %0d LSB = %f %% of full scale",
               mism_fl, worst_fl, 100.0 * worst_fl / SCALE);
      if (cap_n != FIR_NSAMP)
        $display("    RESULT: FAIL (sample count)");
      else if (mism_fx != 0)
        $display("    RESULT: FAIL (does not match fixed-point model)");
      else if ((worst_fl * 1000) >= SCALE)
        $display("    RESULT: FAIL (error >= 0.1 %% of full scale)");
      else
        $display("    RESULT: PASS");
    end
  endtask

  //-------------------------------------------------------------------------
  // main
  //-------------------------------------------------------------------------
  integer ci;
  integer csum;
  integer sym_ok;

  initial begin
    $readmemh("fir_stim.hex",   stim);
    $readmemh("fir_ref_fx.hex", ref_fx);
    $readmemh("fir_golden.hex", ref_fl);

`ifdef DUMP
    $dumpfile("fir.vcd");
    $dumpvars(0, tb_fir_81tap);
`endif

    $display("=========================================================");
    $display(" tb_fir_81tap : %0d taps, %0d samples", 81, FIR_NSAMP);

    // coefficient sanity
    csum = 0;
    for (ci = 0; ci < 81; ci = ci + 1) csum = csum + fir_coeff(ci);
    sym_ok = 1;
    for (ci = 0; ci < 81; ci = ci + 1)
      if (fir_coeff(ci) !== fir_coeff(80 - ci)) sym_ok = 0;
    $display(" sum(H0..H80)   = %0d (expect 32767)", csum);
    $display(" symmetric      = %0s", sym_ok ? "yes" : "NO");
    $display(" multipliers    = 41  (40 symmetric pairs + centre tap)");
    $display("=========================================================");

    repeat (5) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ---- pass 0 : back to back, 1 sample / cycle ----
    cap_en = 1'b1;
    cap_n  = 0;
    drive(0);
    repeat (20) @(negedge clk);
    cap_en = 1'b0;
    check(0);

    // ---- pass 1 : with bubbles ----
    @(negedge clk);
    cap_en = 1'b1;
    cap_n  = 0;
    drive(1);
    repeat (20) @(negedge clk);
    cap_en = 1'b0;
    check(1);

    $display("");
    $display("=========================================================");
    $display(" tb_fir_81tap finished");
    $display("=========================================================");
    $finish;
  end

  // safety net
  initial begin
    #50_000_000;
    $display("ERROR: global timeout");
    $finish;
  end

endmodule
