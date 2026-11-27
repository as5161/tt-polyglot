// Polyglot cluster co-simulation testbench.
// Reads per-cycle stimulus (stim.hex), writes per-cycle state (rtl_trace.txt)
// which run_cosim.py compares against tools/polyglot/sim.py.
`default_nettype none
`timescale 1ns/1ps
module tb_cosim;
  reg clk = 0, rst_n = 0;
  reg hw_en = 0, hr_en = 0, hr_peek = 0;
  reg [14:0] haddr = 0;
  reg [23:0] hwdata = 0;
  reg [31:0] pins = 0;
  wire [23:0] hrdata;
  wire [15:0] pout, poe;

  pg_cluster #(.AW(7), .NC(4)) u (
    .clk(clk), .rst_n(rst_n), .hw_en(hw_en), .hr_en(hr_en), .hr_peek(hr_peek), .haddr(haddr),
    .hwdata(hwdata), .hrdata(hrdata), .pins_in(pins), .pout(pout), .poe(poe));

  reg [75:0] stim [0:65535];
  integer n, k, f;
  reg [23:0] rd;

`define CORE(i) u.g_core[i].u_core.pc, u.g_core[i].u_core.x, u.g_core[i].u_core.y, \
  u.g_core[i].u_core.isr, u.g_core[i].u_core.osr, u.g_core[i].u_core.crc, \
  u.g_core[i].u_core.isr_cnt, u.g_core[i].u_core.osr_cnt, u.g_core[i].u_core.dly, \
  u.g_core[i].u_tx.cnt, u.g_core[i].u_rx.cnt, u.g_core[i].exec_pend, \
  (u.g_core[i].u_rx.empty ? 16'h0 : u.g_core[i].u_rx.rdata), u.g_core[i].bank_q[23:0]

  initial begin
    if (!$value$plusargs("N=%d", n)) n = 1000;
    $readmemh("stim.hex", stim);
    f = $fopen("rtl_trace.txt", "w");
    #5 rst_n = 1;
    for (k = 0; k < n; k = k + 1) begin
      {hr_peek, hw_en, hr_en, haddr, hwdata, pins} = stim[k][73:0];
      #4 rd = hrdata;
      #1 clk = 1;
      #1;
      $fwrite(f, "%h %h %h %h %h", rd, pout, poe, u.flags, u.en);
      $fwrite(f, " %h %h %h %h %h %h %h %h %h %h %h %h %h %h", `CORE(0));
      $fwrite(f, " %h %h %h %h %h %h %h %h %h %h %h %h %h %h", `CORE(1));
      $fwrite(f, " %h %h %h %h %h %h %h %h %h %h %h %h %h %h", `CORE(2));
      $fwrite(f, " %h %h %h %h %h %h %h %h %h %h %h %h %h %h\n", `CORE(3));
      #4 clk = 0;
    end
    $fclose(f);
    $finish;
  end
endmodule
