/*
 * Polyglot - small synchronous FIFO (depth 4)
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module pg_fifo #(
    parameter W = 16
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clr,
    input  wire         push,
    input  wire [W-1:0] wdata,
    input  wire         pop,
    output wire [W-1:0] rdata,
    output wire         empty,
    output wire         full,
    output wire [2:0]   level
);
  reg [W-1:0] mem0, mem1, mem2, mem3;
  reg [1:0]   rp, wp;
  reg [2:0]   cnt;

  assign empty = (cnt == 3'd0);
  assign full  = (cnt == 3'd4);
  assign level = cnt;
  assign rdata = (rp == 2'd0) ? mem0 : (rp == 2'd1) ? mem1 : (rp == 2'd2) ? mem2 : mem3;

  wire do_push = push & ~full;
  wire do_pop  = pop & ~empty;

  always @(posedge clk) begin
    if (do_push) begin
      case (wp)
        2'd0: mem0 <= wdata;
        2'd1: mem1 <= wdata;
        2'd2: mem2 <= wdata;
        default: mem3 <= wdata;
      endcase
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rp  <= 2'd0;
      wp  <= 2'd0;
      cnt <= 3'd0;
    end else if (clr) begin
      rp  <= 2'd0;
      wp  <= 2'd0;
      cnt <= 3'd0;
    end else begin
      if (do_push) wp <= wp + 2'd1;
      if (do_pop)  rp <= rp + 2'd1;
      cnt <= cnt + {2'b0, do_push} - {2'b0, do_pop};
    end
  end
`ifdef FORMAL
  reg f_init = 1'b1;
  always @(posedge clk) f_init <= 1'b0;
  always @* if (f_init) assume (!rst_n);   // start in reset
  always @* if (rst_n) begin
    assert (cnt <= 3'd4);
    assert (cnt[1:0] == wp - rp);
  end
`endif
endmodule
