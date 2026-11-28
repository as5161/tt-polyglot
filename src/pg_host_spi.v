/*
 * Polyglot - host SPI slave (mode 0, MSB first), oversampled by clk.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Frame:  CS low, 16-bit header {W, ADDR[14:0]}, then any number of
 * 24-bit data words.  The address auto-increments after each word, except
 * for the per-core FIFO registers (0x85, 0x8D, 0x95, 0x9D) so bursts stream.
 * hr_peek marks the cycle in which haddr first points at the word about to
 * be read (synchronous memories latch their read address on that edge).
 * SCK must be <= clk/8 (inputs pass a 2-flop synchroniser).
 *
 * Reads are side-effect free while a word is being shifted out (the data is
 * a peek).  Only when a word has been completely transferred, and only if it
 * carried valid FIFO data, is hr_en pulsed to consume it.  So aborting a
 * frame early, or the speculative prefetch of the next word, never loses data.
 */
`default_nettype none

module pg_host_spi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cs_n,     // synchronised
    input  wire        sck,      // synchronised
    input  wire        mosi,     // synchronised
    output wire        miso,

    output reg         hw_en,
    output reg         hr_en,    // consume (pop) the word at haddr
    output wire        hr_peek,  // haddr is a new read address this cycle
    output wire [14:0] haddr,
    output reg  [23:0] hwdata,
    // Bit-serial read-back: the read word is never brought here in parallel.
    // Each source (every core, plus the cluster's global registers) keeps its
    // own 24-bit shift register next to its data; this block only says when
    // to load, shift and clear them, and MISO is the OR of their MSBs.  Same
    // timing as the old central shift register (clear > shift > load).
    output wire        sh_ld,     // load the word at haddr (the peek)
    output wire        sh_shift,  // shift left one bit (SCK rising edge)
    output wire        sh_clr,    // clear (CS_n high)
    input  wire        rd_msb,    // OR of the shift registers' MSBs
    input  wire        rd_d23     // bit 23 of the word being loaded (FIFO valid)
);
  reg        sck_q;
  reg [4:0]  bitcnt;
  reg        hdr;         // receiving header
  reg        wr;
  reg [14:0] addr;
  reg [14:0] waddr;
  reg [22:0] shin;
  reg        rd_req;      // load the read shift registers (peek)
  reg        rd_next;     // peek after the pop has happened
  reg        peek_valid;  // peeked word was valid FIFO data

  wire rise = sck & ~sck_q & ~cs_n;
  wire [23:0] shin_n = {shin[22:0], mosi};
  wire is_fifo = (addr[14:8] == 7'd0) && (addr[7:5] == 3'b100) && (addr[2:0] == 3'd5);
  wire [14:0] addr_inc = is_fifo ? addr : addr + 15'd1;

  assign haddr   = hw_en ? waddr : addr;
  assign hr_peek = rd_next;
  assign miso     = rd_msb;
  assign sh_ld    = rd_req & ~hdr;
  assign sh_shift = rise;
  assign sh_clr   = cs_n;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sck_q      <= 1'b0;
      bitcnt     <= 5'd0;
      hdr        <= 1'b1;
      wr         <= 1'b0;
      addr       <= 15'h0;
      waddr      <= 15'h0;
      shin       <= 23'h0;
      rd_req     <= 1'b0;
      rd_next    <= 1'b0;
      peek_valid <= 1'b0;
      hw_en      <= 1'b0;
      hr_en      <= 1'b0;
      hwdata     <= 24'h0;
    end else begin
      sck_q   <= sck;
      hw_en   <= 1'b0;
      hr_en   <= 1'b0;
      rd_req  <= rd_next;
      rd_next <= 1'b0;
      if (rd_req && !hdr)
        peek_valid <= is_fifo & rd_d23;
      if (cs_n) begin
        bitcnt     <= 5'd0;
        hdr        <= 1'b1;
        peek_valid <= 1'b0;
        rd_req     <= 1'b0;
        rd_next    <= 1'b0;
      end else if (rise) begin
        shin  <= shin_n[22:0];
        if (hdr) begin
          if (bitcnt == 5'd15) begin
            hdr     <= 1'b0;
            bitcnt  <= 5'd0;
            wr      <= shin_n[15];
            addr    <= shin_n[14:0];
            rd_next <= ~shin_n[15];
          end else bitcnt <= bitcnt + 5'd1;
        end else begin
          if (bitcnt == 5'd23) begin
            bitcnt <= 5'd0;
            if (wr) begin
              hw_en  <= 1'b1;
              waddr  <= addr;
              hwdata <= shin_n;
              addr   <= addr_inc;
            end else begin
              hr_en      <= peek_valid;   // consume the word just sent
              peek_valid <= 1'b0;
              rd_next    <= 1'b1;         // then peek the next one
              addr       <= addr_inc;
            end
          end else bitcnt <= bitcnt + 5'd1;
        end
      end
    end
  end
`ifdef FORMAL
  reg f_init = 1'b1;
  always @(posedge clk) f_init <= 1'b0;
  always @* if (f_init) assume (!rst_n);   // start in reset
  reg f_past = 1'b0;
  always @(posedge clk) f_past <= 1'b1;
  always @(posedge clk) if (f_past && $past(rst_n)) assume (rst_n);  // reset is not re-asserted
  always @* if (rst_n) begin
    assert (!(hw_en && hr_en));                // one bus operation at a time
    if (hr_en) assert (is_fifo && rd_next && !hdr);   // only FIFO reads have side effects
    if (peek_valid) assert (is_fifo && !hdr);
    if (hdr) assert (bitcnt <= 5'd15);
    else     assert (bitcnt <= 5'd23);
  end
`endif
endmodule
