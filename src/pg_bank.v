/*
 * Polyglot - one core's instruction bank: 256 x 32-bit, synchronous read.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Every clock edge either writes wdata at addr (we = 1) or reads addr into
 * rdata (we = 0).  A write cycle leaves rdata unchanged.  This is exactly the
 * behaviour of the IHP RM_IHPSG13_1P_256x32_c2_bm_bist SRAM, which the ASIC
 * flow uses (PG_SRAM_MACRO is set in src/config.json); simulation uses the
 * flop model below, which behaves the same.
 */
`default_nettype none

module pg_bank (
    input  wire        clk,
    input  wire        we,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    output wire [31:0] rdata
);
`ifdef PG_SRAM_MACRO
  // The macro's BIST port is unused; byte mask all ones; A_DLY = 1 is the
  // data sheet's recommended read timing.
  (* keep *) RM_IHPSG13_1P_256x32_c2_bm_bist sram (
      .A_CLK(clk),
      .A_MEN(1'b1),
      .A_WEN(we),
      .A_REN(~we),
      .A_ADDR(addr),
      .A_DIN(wdata),
      .A_DLY(1'b1),
      .A_DOUT(rdata),
      .A_BM(32'hFFFF_FFFF),
      .A_BIST_CLK(1'b0),
      .A_BIST_EN(1'b0),
      .A_BIST_MEN(1'b0),
      .A_BIST_WEN(1'b0),
      .A_BIST_REN(1'b0),
      .A_BIST_ADDR(8'd0),
      .A_BIST_DIN(32'd0),
      .A_BIST_BM(32'd0));
`else
  reg [31:0] mem [0:255];
  reg [31:0] q;
  always @(posedge clk) begin
    if (we) mem[addr] <= wdata;
    else    q <= mem[addr];
  end
  assign rdata = q;
`ifndef SYNTHESIS
  // simulation only: start from zeros instead of X (the silicon SRAM powers
  // up random, which is harmless: a core never runs before it is loaded)
  integer i;
  initial begin
    for (i = 0; i < 256; i = i + 1) mem[i] = 32'h0;
    q = 32'h0;
  end
`endif
`endif
endmodule
