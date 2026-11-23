`default_nettype none
`timescale 1ns / 1ps

/* Polyglot top-level testbench wrapper (RTL and gate-level).
   uio pins listed in od_mask model an open-drain bus with pull-ups:
   the line is low if the chip drives it low OR the cocotb model pulls it low
   (uio_drv bit = 0).  Other uio pins simply take uio_drv. */
module tb ();

  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_drv;
  reg [7:0] od_mask;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  wire [7:0] chip_low = uio_oe & ~uio_out;
  wire [7:0] uio_in   = (od_mask & ~chip_low & uio_drv) | (~od_mask & uio_drv);

  tt_um_polyglot user_project (
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

endmodule
