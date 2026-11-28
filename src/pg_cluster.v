/*
 * Polyglot - cluster: 4 cores, each with its own 256-word instruction bank,
 * FIFOs, flags, pin arbitration and the host register file.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host register map (24-bit words, 15-bit address) - see docs/registers.md
 *   0x0400 + 0x100*c + w   IMEM bank of core c, word w (2^AW words, AW=7 in
 *                          project.v).  Accessible only while core c is
 *                          stopped (EN[c] = 0): writes are ignored and reads
 *                          return 0 while it runs.
 *   0x80+8c+n  per-core registers, c = 0..3:
 *        n=0 CLKDIV    [15:0]
 *        n=1 EXECCTRL  WRAP_BOT[6:0] WRAP_TOP[13:7] JMP_PIN[18:14] SIDE_PINDIR[19] OD[20]
 *        n=2 SHIFTCTRL OUT_DIR[0] IN_DIR[1] AUTOPULL[2] AUTOPUSH[3] PULL_TH[8:4]
 *                      PUSH_TH[13:9] CRC_SRC[15:14] CRC_REFLECT[16]
 *        n=3 PINCTRL   OUT_BASE[4:0] OUT_CNT[9:5] SET_BASE[14:10] SET_CNT[18:15]
 *        n=4 PINCTRL2  IN_BASE[4:0] SIDE_BASE[9:5] SIDE_CNT[11:10]
 *        n=5 FIFO      W: push TX FIFO   R: RX head {valid, 7'b0, data}; hr_en pops
 *        n=6 PC        W: jump           R: current PC
 *        n=7 EXEC      W: execute word immediately   R: core status
 *   0xA0 CTRL      W: EN[3:0] RESTART[7:4] DIVSYNC[11:8] FIFOCLR[15:12]   R: EN
 *   (NC = number of cores, 4 in project.v; bits/registers of missing cores are ignored / read 0)
 *   0xA1 STEP      W: single-step mask[3:0]
 *   0xA2 FLAGS     R/W [7:0]
 *   0xA3 FIFOSTAT  R: TXFULL[3:0] TXEMPTY[7:4] RXFULL[11:8] RXEMPTY[15:12]
 *   0xA4 PINS      R: pin space [23:0]
 *   0xA5 ID        R: 0x504702 ("PG", v2)
 *   0xA6 TIME      R: free-running cycle counter
 *   0xA8+c POLY    R/W CRC polynomial of core c
 *   0xAC+c X  0xB0+c Y  0xB4+c CRC   (R: debug)
 *   (all of 0x00-0xFF lives in page 0: address bits [14:8] = 0)
 */
`default_nettype none

module pg_cluster #(
    parameter AW = 7,           // core PC width: 2^AW words per bank (<= 8)
    parameter NC = 4
) (
    input  wire        clk,
    input  wire        rst_n,

    // host bus
    input  wire        hw_en,
    input  wire        hr_en,
    input  wire        hr_peek,
    input  wire [14:0] haddr,
    input  wire [23:0] hwdata,
    output wire [23:0] hrdata,   // word at haddr (simulation/debug: not used on chip)

    // bit-serial read-back (see pg_host_spi): one shift register per core,
    // next to that core's registers, and one for the global registers.  Only
    // these 3 strobes go out to the cores and 1 bit per core comes back,
    // instead of every readable register crossing the chip to one mux.
    input  wire        sh_ld,
    input  wire        sh_shift,
    input  wire        sh_clr,
    output wire        rd_msb,   // OR of the shift registers' MSBs (= MISO)
    output wire        rd_d23,   // bit 23 of the word at haddr

    // pins
    input  wire [31:0] pins_in,
    output reg  [15:0] pout,
    output reg  [15:0] poe
);
  localparam DEPTH = (1 << AW);

  // ------------------------------------------------------------ global state
  reg [NC-1:0] en;
  reg [7:0]  flags;
  reg [15:0] timer;

  wire       page0     = (haddr[14:8] == 7'd0);
  wire [7:0] ra        = haddr[7:0];
  wire       host_core = page0 && (ra[7:5] == 3'b100);
  wire [1:0] hc  = ra[4:3];
  wire [2:0] hn  = ra[2:0];

  // IMEM banks: 0x0400 + 0x100*bank + word
  wire       imem_hit  = (haddr[14:10] == 5'b00001) && (ra < DEPTH);
  wire [1:0] imem_bank = haddr[9:8];

  wire w_ctrl  = hw_en && page0 && ra == 8'hA0;
  wire w_step  = hw_en && page0 && ra == 8'hA1;
  wire w_flags = hw_en && page0 && ra == 8'hA2;

  // ------------------------------------------------------------ per-core
  // The register map always has 4 core slots (NC may be 1..4).  Slots
  // without a core read as 0, and their FIFOs report "empty".
  wire [31:0] c_pvm [0:NC-1];
  wire [31:0] c_pvv [0:NC-1];
  wire [31:0] c_pdm [0:NC-1];
  wire [31:0] c_pdv [0:NC-1];
  wire [7:0]  c_fset [0:NC-1];
  wire [7:0]  c_fclr [0:NC-1];
  wire [23:0] c_imem [0:3];        // bank read port output (0 while the core runs)
  wire [15:0] c_x [0:3];
  wire [15:0] c_y [0:3];
  wire [15:0] c_crc [0:3];
  wire [3:0]  tx_full_v, tx_empty_v, rx_full_v, rx_empty_v;
  wire [23:0] c_word [0:3];        // this core's part of the word at haddr (0 if not its own)
  wire [3:0]  c_msb;               // MSB of this core's read shift register

  genvar c;
  generate
    for (c = 0; c < NC; c = c + 1) begin : g_core
      wire sel  = host_core && (hc == c);
      wire wsel = hw_en && sel;

      reg [15:0] clkdiv;
      reg [AW-1:0] wrap_bot, wrap_top;
      reg [4:0]  jmp_pin;
      reg        side_pindir, od;
      reg        out_dir, in_dir, autopull, autopush;
      reg [4:0]  pull_th, push_th;
      reg [1:0]  crc_src;
      reg        crc_reflect;
      reg [4:0]  out_base, out_cnt, set_base, in_base, side_base;
      reg [3:0]  set_cnt;
      reg [1:0]  side_cnt;
      reg [15:0] poly;
      reg [15:0] divcnt;
      reg        exec_pend;
      reg [23:0] exec_ir;

      wire w_poly = hw_en && page0 && ra == (8'hA8 + c);

      always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          clkdiv <= 16'd0;  wrap_bot <= {AW{1'b0}};  wrap_top <= {AW{1'b1}};
          jmp_pin <= 5'd0;  side_pindir <= 1'b0;  od <= 1'b0;
          out_dir <= 1'b0;  in_dir <= 1'b0;  autopull <= 1'b0;  autopush <= 1'b0;
          pull_th <= 5'd0;  push_th <= 5'd0;  crc_src <= 2'd0;  crc_reflect <= 1'b0;
          out_base <= 5'd0; out_cnt <= 5'd0; set_base <= 5'd0; set_cnt <= 4'd0;
          in_base <= 5'd0;  side_base <= 5'd0; side_cnt <= 2'd0; poly <= 16'd0;
        end else begin
          if (wsel && hn == 3'd0) clkdiv <= hwdata[15:0];
          if (wsel && hn == 3'd1) begin
            wrap_bot <= hwdata[AW-1:0];  wrap_top <= hwdata[7+AW-1:7];
            jmp_pin <= hwdata[18:14];  side_pindir <= hwdata[19];  od <= hwdata[20];
          end
          if (wsel && hn == 3'd2) begin
            out_dir <= hwdata[0];  in_dir <= hwdata[1];  autopull <= hwdata[2];  autopush <= hwdata[3];
            pull_th <= hwdata[8:4];  push_th <= hwdata[13:9];  crc_src <= hwdata[15:14];  crc_reflect <= hwdata[16];
          end
          if (wsel && hn == 3'd3) begin
            out_base <= hwdata[4:0];  out_cnt <= hwdata[9:5];  set_base <= hwdata[14:10];  set_cnt <= hwdata[18:15];
          end
          if (wsel && hn == 3'd4) begin
            in_base <= hwdata[4:0];  side_base <= hwdata[9:5];  side_cnt <= hwdata[11:10];
          end
          if (w_poly) poly <= hwdata[15:0];
        end
      end

      // clock divider / tick
      wire restart = w_ctrl && hwdata[4+c];
      wire divsync = w_ctrl && hwdata[8+c];
      wire fifoclr = w_ctrl && hwdata[12+c];
      wire step    = w_step && hwdata[c];
      wire [15:0] div_m1 = (clkdiv == 16'd0) ? 16'd0 : clkdiv - 16'd1;
      wire div_tick = en[c] && (divcnt == 16'd0);
      wire tick = div_tick | step;

      always @(posedge clk or negedge rst_n) begin
        if (!rst_n) divcnt <= 16'd0;
        else if (divsync) divcnt <= 16'd0;
        else if (en[c]) divcnt <= (divcnt >= div_m1) ? 16'd0 : divcnt + 16'd1;
      end

      // FIFOs
      wire        tx_pop, rx_push;
      wire [15:0] rx_wdata, tx_rdata, rx_rdata;
      wire        tx_empty, tx_full, rx_empty, rx_full;
      wire [2:0]  tx_level, rx_level;
      wire host_fifo = sel && hn == 3'd5;

      pg_fifo #(.W(16)) u_tx (
          .clk(clk), .rst_n(rst_n), .clr(fifoclr),
          .push(hw_en && host_fifo), .wdata(hwdata[15:0]),
          .pop(tx_pop), .rdata(tx_rdata),
          .empty(tx_empty), .full(tx_full), .level(tx_level));
      pg_fifo #(.W(16)) u_rx (
          .clk(clk), .rst_n(rst_n), .clr(fifoclr),
          .push(rx_push), .wdata(rx_wdata),
          .pop(hr_en && host_fifo), .rdata(rx_rdata),
          .empty(rx_empty), .full(rx_full), .level(rx_level));

      assign tx_full_v[c]  = tx_full;
      assign tx_empty_v[c] = tx_empty;
      assign rx_full_v[c]  = rx_full;
      assign rx_empty_v[c] = rx_empty;

      // exec injection
      wire exec_done;
      always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          exec_pend <= 1'b0;
          exec_ir   <= 24'h0;
        end else if (wsel && hn == 3'd7) begin
          exec_pend <= 1'b1;
          exec_ir   <= hwdata;
        end else if (exec_done || restart) begin
          exec_pend <= 1'b0;
        end
      end

      wire [AW-1:0] pc, pc_next;
      // Instruction bank.  Every cycle it reads imem[pc_next], so its output
      // register is the core's fetch register (EXT_IR): it always holds
      // imem[pc].  The host may use the port only while this core is
      // stopped: a write, or the read of a word it is about to shift out.
      // The cycle after, the bank fetches imem[pc] again.
      wire bank_host = imem_hit && imem_bank == c && !en[c];
      wire bank_we   = hw_en && bank_host;
      wire bank_hrd  = hr_peek && bank_host;
      wire [31:0] bank_q;
      wire [7:0]  bank_a = (bank_we | bank_hrd) ? ra : {{(8-AW){1'b0}}, pc_next};
      pg_bank u_bank (
          .clk(clk), .we(bank_we), .addr(bank_a),
          .wdata({8'h0, hwdata}), .rdata(bank_q));
      wire [23:0] imem_next = bank_q[23:0];
      assign c_imem[c] = en[c] ? 24'h0 : bank_q[23:0];
      wire _unused_bank = &{1'b0, bank_q[31:24]};
      pg_core #(.AW(AW), .EXT_IR(1)) u_core (
          .clk(clk), .rst_n(rst_n),
          .tick(tick), .exec_valid(exec_pend), .exec_instr(exec_ir), .exec_done(exec_done),
          .imem_next(imem_next), .pc_o(pc), .pc_next(pc_next),
          .pc_we(wsel && hn == 3'd6), .pc_wdata(hwdata[AW-1:0]), .restart(restart),
          .wrap_bot(wrap_bot), .wrap_top(wrap_top), .jmp_pin(jmp_pin),
          .side_pindir(side_pindir), .od(od),
          .out_dir(out_dir), .in_dir(in_dir), .autopull(autopull), .autopush(autopush),
          .pull_th_f(pull_th), .push_th_f(push_th),
          .crc_src(crc_src), .crc_reflect(crc_reflect), .crc_poly(poly),
          .out_base(out_base), .out_cnt(out_cnt), .set_base(set_base), .set_cnt(set_cnt),
          .in_base(in_base), .side_base(side_base), .side_cnt(side_cnt),
          .pins(pins_in), .flags(flags), .time_i(timer),
          .tx_empty(tx_empty), .tx_data(tx_rdata), .tx_level(tx_level), .tx_pop(tx_pop),
          .rx_full(rx_full), .rx_level(rx_level), .rx_push(rx_push), .rx_data(rx_wdata),
          .pv_mask(c_pvm[c]), .pv_val(c_pvv[c]), .pd_mask(c_pdm[c]), .pd_val(c_pdv[c]),
          .flag_set(c_fset[c]), .flag_clr(c_fclr[c]),
          .x_o(c_x[c]), .y_o(c_y[c]), .crc_o(c_crc[c]));

      reg [23:0] rd;
      always @* begin
        case (hn)
          3'd0: rd = {8'h0, clkdiv};
          3'd1: rd = {3'b0, od, side_pindir, jmp_pin, {(7-AW){1'b0}}, wrap_top, {(7-AW){1'b0}}, wrap_bot};
          3'd2: rd = {7'b0, crc_reflect, crc_src, push_th, pull_th, autopush, autopull, in_dir, out_dir};
          3'd3: rd = {5'b0, set_cnt, set_base, out_cnt, out_base};
          3'd4: rd = {12'b0, side_cnt, side_base, in_base};
          3'd5: rd = {~rx_empty, 7'b0, rx_empty ? 16'h0 : rx_rdata};
          3'd6: rd = {{(24-AW){1'b0}}, pc};
          default: rd = {16'h0, exec_pend, 1'b0, rx_level, tx_level};
        endcase
      end

      // read-back: this core's own share of the word at haddr, kept local
      wire [5:0] rgrp = ra[7:2];
      wire       rown = (ra[1:0] == c);
      reg [23:0] word;
      always @* begin
        word = 24'h0;
        if (imem_hit) begin
          if (imem_bank == c) word = c_imem[c];
        end else if (page0) begin
          if (sel)                          word = rd;
          else if (rgrp == 6'h2A && rown)   word = {8'h0, poly};       // 0xA8+c
          else if (rgrp == 6'h2B && rown)   word = {8'h0, c_x[c]};     // 0xAC+c
          else if (rgrp == 6'h2C && rown)   word = {8'h0, c_y[c]};     // 0xB0+c
          else if (rgrp == 6'h2D && rown)   word = {8'h0, c_crc[c]};   // 0xB4+c
        end
      end
      reg [23:0] rsh;
      always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        rsh <= 24'h0;
        else if (sh_clr)   rsh <= 24'h0;
        else if (sh_shift) rsh <= {rsh[22:0], 1'b0};
        else if (sh_ld)    rsh <= word;
      end
      assign c_word[c] = word;
      assign c_msb[c]  = rsh[23];
    end
  endgenerate

  genvar u;
  generate
    for (u = NC; u < 4; u = u + 1) begin : g_absent
      assign c_imem[u] = 24'h0;
      assign c_x[u]    = 16'h0;
      assign c_y[u]    = 16'h0;
      assign c_crc[u]  = 16'h0;
      assign tx_full_v[u]  = 1'b0;
      assign tx_empty_v[u] = 1'b1;
      assign rx_full_v[u]  = 1'b0;
      assign rx_empty_v[u] = 1'b1;
      assign c_word[u]     = 24'h0;
      assign c_msb[u]      = 1'b0;
    end
  endgenerate

  // ------------------------------------------------------------ global regs
  integer i;
  reg [7:0] fset, fclr;
  reg [15:0] npout, npoe;
  always @* begin
    fset  = 8'h0;
    fclr  = 8'h0;
    npout = pout;
    npoe  = poe;
    for (i = 0; i < NC; i = i + 1) begin   // higher core index wins
      fset  = fset | c_fset[i];
      fclr  = fclr | c_fclr[i];
      npout = (npout & ~c_pvm[i][15:0]) | (c_pvv[i][15:0] & c_pvm[i][15:0]);
      npoe  = (npoe  & ~c_pdm[i][15:0]) | (c_pdv[i][15:0] & c_pdm[i][15:0]);
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      en    <= {NC{1'b0}};
      flags <= 8'h0;
      timer <= 16'h0;
      pout  <= 16'h0;
      poe   <= 16'h0;
    end else begin
      timer <= timer + 16'd1;
      if (w_ctrl) en <= hwdata[NC-1:0];
      if (w_flags) flags <= hwdata[7:0];
      else flags <= (flags & ~fclr) | fset;
      pout <= npout;
      poe  <= npoe;
    end
  end

  // ------------------------------------------------------------ host read-back
  // Global registers only; per-core words are produced inside g_core.  Exactly
  // one source is non-zero for any address, so OR-ing them gives the word.
  reg [23:0] gword;
  always @* begin
    gword = 24'h0;
    if (page0 && !host_core) begin
      case (ra)
        8'hA0: gword = {{(24-NC){1'b0}}, en};
        8'hA2: gword = {16'h0, flags};
        8'hA3: gword = {8'h0, rx_empty_v, rx_full_v, tx_empty_v, tx_full_v};
        8'hA4: gword = pins_in[23:0];
        8'hA5: gword = 24'h504702;
        8'hA6: gword = {8'h0, timer};
        default: gword = 24'h0;
      endcase
    end
  end
  reg [23:0] gsh;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)        gsh <= 24'h0;
    else if (sh_clr)   gsh <= 24'h0;
    else if (sh_shift) gsh <= {gsh[22:0], 1'b0};
    else if (sh_ld)    gsh <= gword;
  end

  assign hrdata = gword | c_word[0] | c_word[1] | c_word[2] | c_word[3];
  assign rd_d23 = hrdata[23];
  assign rd_msb = gsh[23] | (|c_msb);

endmodule
