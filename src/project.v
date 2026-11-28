/*
 * Polyglot - a programmable, cycle-exact protocol emulator
 * Copyright (c) 2026 Polyglot authors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pin map
 *   ui[4:0]   general inputs           (pin space 16..20)
 *   ui[5]     host SPI MOSI
 *   ui[6]     host SPI SCK
 *   ui[7]     host SPI CS_n
 *   uo[6:0]   general outputs          (pin space 8..14)
 *   uo[7]     host SPI MISO
 *   uio[7:0]  general bidirectional    (pin space 0..7)
 */
`default_nettype none
/* verilator lint_off DECLFILENAME */

module tt_um_polyglot (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
  // 2-flop input synchronisers (deterministic 2-cycle input latency)
  reg [7:0] ui_s0, ui_s1, uio_s0, uio_s1;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ui_s0  <= 8'hE0;  ui_s1  <= 8'hE0;   // CS_n idle high
      uio_s0 <= 8'h00;  uio_s1 <= 8'h00;
    end else begin
      ui_s0  <= ui_in;  ui_s1  <= ui_s0;
      uio_s0 <= uio_in; uio_s1 <= uio_s0;
    end
  end

  wire        hw_en, hr_en, hr_peek, miso;
  wire        sh_ld, sh_shift, sh_clr, rd_msb, rd_d23;
  wire [14:0] haddr;
  wire [23:0] hwdata, hrdata;
  wire [15:0] pout, poe;

  pg_host_spi u_host (
      .clk(clk), .rst_n(rst_n),
      .cs_n(ui_s1[7]), .sck(ui_s1[6]), .mosi(ui_s1[5]), .miso(miso),
      .hw_en(hw_en), .hr_en(hr_en), .hr_peek(hr_peek), .haddr(haddr),
      .hwdata(hwdata),
      .sh_ld(sh_ld), .sh_shift(sh_shift), .sh_clr(sh_clr), .rd_msb(rd_msb), .rd_d23(rd_d23));

  wire [31:0] pins = {8'h00, ui_s1, miso, pout[14:8], uio_s1};

  pg_cluster #(.AW(7), .NC(4)) u_cluster (   // 4 cores, 128 words each (SRAM banks)
      .clk(clk), .rst_n(rst_n),
      .hw_en(hw_en), .hr_en(hr_en), .hr_peek(hr_peek), .haddr(haddr),
      .hwdata(hwdata), .hrdata(hrdata),
      .sh_ld(sh_ld), .sh_shift(sh_shift), .sh_clr(sh_clr), .rd_msb(rd_msb), .rd_d23(rd_d23),
      .pins_in(pins), .pout(pout), .poe(poe));

  assign uo_out  = {miso, pout[14:8]};
  assign uio_out = pout[7:0];
  assign uio_oe  = poe[7:0];

  wire _unused = &{ena, pout[15], poe[15:8], hrdata, 1'b0};   // hrdata: debug view only
endmodule
