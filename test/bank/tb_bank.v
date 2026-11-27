// Lock-step check: the flop model of pg_bank against the IHP SRAM's own
// Verilog model (PG_SRAM_MACRO), under random reads and writes.
`timescale 1ns/10ps
module tb_bank;
  reg clk = 0, we = 0;
  reg [7:0] addr = 0;
  reg [31:0] wdata = 0;
  wire [31:0] q_flop, q_sram;
  pg_bank_flop u_f (.clk(clk), .we(we), .addr(addr), .wdata(wdata), .rdata(q_flop));
  pg_bank      u_s (.clk(clk), .we(we), .addr(addr), .wdata(wdata), .rdata(q_sram));
  always #10 clk = ~clk;
  integer i, errs = 0, checks = 0;
  reg read_seen = 0;   // DOUT is undefined until the first read (power-up)
  initial begin
    // fill both so the SRAM model has no X contents
    for (i = 0; i < 256; i = i + 1) begin
      @(negedge clk); we = 1; addr = i; wdata = $random;
    end
    for (i = 0; i < 20000; i = i + 1) begin
      @(negedge clk);
      if (read_seen) begin
        if (q_flop !== q_sram) begin
          errs = errs + 1;
          if (errs < 10) $display("MISMATCH t=%0t flop=%h sram=%h", $time, q_flop, q_sram);
        end
        checks = checks + 1;
      end
      if (i > 0 && !we) read_seen = 1;
      we = ($random & 3) == 0;  addr = $random;  wdata = $random;
    end
    $display("bank lock-step: %0d checks, %0d mismatches", checks, errs);
    $finish;
  end
endmodule
