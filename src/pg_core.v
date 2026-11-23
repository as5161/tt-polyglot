/*
 * Polyglot protocol emulator - bit-engine core
 * Copyright (c) 2026 Polyglot authors
 * SPDX-License-Identifier: Apache-2.0
 *
 * One core = one cycle-exact protocol thread.  Every instruction takes
 * exactly (1 + delay) ticks unless it stalls on a WAIT / FIFO condition.
 * See docs/isa.md for the instruction set.  tools/pgsim.py is the golden
 * model; test/cosim checks this RTL against it cycle by cycle.
 */
`default_nettype none

module pg_core #(
    parameter AW = 6
) (
    input  wire        clk,
    input  wire        rst_n,

    // sequencing
    input  wire        tick,          // divided-clock enable (or single-step)
    input  wire        exec_valid,    // host-injected instruction pending
    input  wire [23:0] exec_instr,
    output wire        exec_done,
    input  wire [23:0] imem_next,     // imem[pc_next] (with host-write bypass)
    output wire [AW-1:0] pc_o,
    output reg  [AW-1:0] pc_next,

    // host control
    input  wire        pc_we,
    input  wire [AW-1:0] pc_wdata,
    input  wire        restart,

    // configuration
    input  wire [AW-1:0] wrap_bot,
    input  wire [AW-1:0] wrap_top,
    input  wire [4:0]  jmp_pin,
    input  wire        side_pindir,
    input  wire        od,
    input  wire        out_dir,       // 0: shift right (LSB first)  1: left (MSB first)
    input  wire        in_dir,
    input  wire        autopull,
    input  wire        autopush,
    input  wire [4:0]  pull_th_f,     // 0 => 16
    input  wire [4:0]  push_th_f,
    input  wire [1:0]  crc_src,       // bit0: OUT bits, bit1: IN bits
    input  wire        crc_reflect,
    input  wire [15:0] crc_poly,
    input  wire [4:0]  out_base,
    input  wire [4:0]  out_cnt,
    input  wire [4:0]  set_base,
    input  wire [3:0]  set_cnt,
    input  wire [4:0]  in_base,
    input  wire [4:0]  side_base,
    input  wire [1:0]  side_cnt,

    // environment
    input  wire [31:0] pins,
    input  wire [7:0]  flags,
    input  wire [15:0] time_i,
    input  wire        tx_empty,
    input  wire [15:0] tx_data,
    input  wire [2:0]  tx_level,
    output reg         tx_pop,
    input  wire        rx_full,
    input  wire [2:0]  rx_level,
    output reg         rx_push,
    output reg  [15:0] rx_data,

    // pin writes this cycle (32-bit pin space)
    output wire [31:0] pv_mask,
    output wire [31:0] pv_val,
    output wire [31:0] pd_mask,
    output wire [31:0] pd_val,
    output reg  [7:0]  flag_set,
    output reg  [7:0]  flag_clr,

    output wire [15:0] x_o,
    output wire [15:0] y_o,
    output wire [15:0] crc_o
);

  // ---------------------------------------------------------------- opcodes
  localparam OP_JMP = 4'd0, OP_WAIT = 4'd1, OP_IN = 4'd2, OP_OUT = 4'd3,
             OP_PP  = 4'd4, OP_MOV  = 4'd5, OP_FLAG = 4'd6, OP_SET = 4'd7,
             OP_ADD = 4'd8;

  // ---------------------------------------------------------------- state
  reg [AW-1:0] pc;
  reg [23:0]   ir_q;      // fetch register: always equals imem[pc]
  reg [15:0]   x, y, isr, osr, crc;
  reg [4:0]    isr_cnt, osr_cnt, dly;

  assign pc_o  = pc;
  assign x_o   = x;
  assign y_o   = y;
  assign crc_o = crc;

  // ---------------------------------------------------------------- helpers
  /* verilator lint_off UNUSEDSIGNAL */
  function [31:0] rotl32(input [31:0] v, input [4:0] s);
    reg [63:0] t;
    begin
      t = {v, v} << s;
      rotl32 = t[63:32];
    end
  endfunction

  function [31:0] rotr32(input [31:0] v, input [4:0] s);
    reg [63:0] t;
    begin
      t = {v, v} >> s;
      rotr32 = t[31:0];
    end
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  function [15:0] mask16(input [4:0] n);  // n in 0..16
    begin
      mask16 = (n >= 5'd16) ? 16'hffff : ((16'h1 << n) - 16'h1);
    end
  endfunction

  function [15:0] rev16(input [15:0] v);
    integer i;
    begin
      for (i = 0; i < 16; i = i + 1) rev16[i] = v[15-i];
    end
  endfunction

  // one CRC step (serial bit b)
  function [15:0] crc_step(input [15:0] c, input b, input refl, input [15:0] poly);
    reg fb;
    begin
      if (refl) begin
        fb = c[0] ^ b;
        crc_step = (c >> 1) ^ (fb ? poly : 16'h0);
      end else begin
        fb = c[15] ^ b;
        crc_step = (c << 1) ^ (fb ? poly : 16'h0);
      end
    end
  endfunction

  // ---------------------------------------------------------------- decode
  wire [23:0] ir      = exec_valid ? exec_instr : ir_q;
  wire [3:0]  op      = ir[23:20];
  wire        side_en = ir[19];
  wire [2:0]  side_v  = ir[18:16];
  wire [4:0]  idly    = ir[15:11];
  wire [10:0] a       = ir[10:0];
  wire [2:0]  f3      = a[10:8];
  wire [4:0]  nbits   = (a[3:0] == 4'd0) ? 5'd16 : {1'b0, a[3:0]};

  wire        go      = exec_valid | (tick & (dly == 5'd0));

  wire [4:0]  pull_th = (pull_th_f == 5'd0) ? 5'd16 : pull_th_f;
  wire [4:0]  push_th = (push_th_f == 5'd0) ? 5'd16 : push_th_f;

  wire [31:0] rin     = rotr32(pins, in_base);
  wire        osre    = (osr_cnt >= pull_th);   // OSR "empty"

  wire [15:0] status  = {flags, 2'b00, rx_level, tx_level};

  // ---------------------------------------------------------------- execute
  reg         stall;
  reg         jump;
  reg [AW-1:0] jtarget;
  reg [15:0]  nx, ny, nisr, nosr, ncrc;
  reg [4:0]   nisr_cnt, nosr_cnt;

  reg         mw_en, mw_dir;
  reg [4:0]   mw_base, mw_cnt;
  reg [15:0]  mw_data;

  // temporaries
  reg [15:0]  srcv, movv;
  reg         cond, wsat;
  reg [15:0]  simm;
  reg         crc_feed;

  // ---------------------------------------------------------------- shared datapath
  // Computed outside the execute block (pure functions of state + decode).
  // IN path
  reg  [15:0] in_src;
  always @* begin
    case (f3)
      3'd0: in_src = rin[15:0];
      3'd1: in_src = x;
      3'd2: in_src = y;
      3'd3: in_src = 16'h0;
      3'd4: in_src = isr;
      3'd5: in_src = osr;
      3'd6: in_src = crc;
      default: in_src = time_i;
    endcase
  end
  wire [15:0] idata   = in_src & mask16(nbits);
  wire [15:0] isr_sh  = in_dir ? ((isr << nbits) | idata) : ((isr >> nbits) | (idata << (5'd16 - nbits)));
  wire [4:0]  cnt_sum = (({1'b0, isr_cnt} + {1'b0, nbits}) > 6'd16) ? 5'd16 : (isr_cnt + nbits);
  // OUT path (autopull refills the OSR in the same instruction)
  wire        refill   = autopull & osre & ~tx_empty;
  wire [15:0] osr_src  = refill ? tx_data : osr;
  wire [4:0]  osr_csrc = refill ? 5'd0 : osr_cnt;
  wire [15:0] odata    = out_dir ? (osr_src >> (5'd16 - nbits)) : (osr_src & mask16(nbits));
  wire [15:0] osr_sh   = out_dir ? (osr_src << nbits) : (osr_src >> nbits);
  wire [4:0]  osr_csum = (({1'b0, osr_csrc} + {1'b0, nbits}) > 6'd16) ? 5'd16 : (osr_csrc + nbits);
  // CRC is fed by 1-bit IN / OUT shifts only (one engine per core)
  wire        crc_bit = (op == OP_IN) ? idata[0] : odata[0];
  wire [15:0] crc_out = crc_step(crc, crc_bit, crc_reflect, crc_poly);
  // right-justified push data (IN autopush or PUSH)
  wire [15:0] j_v     = (op == OP_IN) ? isr_sh : isr;
  wire [4:0]  j_cnt   = (op == OP_IN) ? cnt_sum : isr_cnt;
  wire [15:0] j_out   = in_dir ? j_v : (j_v >> (5'd16 - j_cnt));

  always @* begin
    stall    = 1'b0;
    jump     = 1'b0;
    jtarget  = {AW{1'b0}};
    nx       = x;
    ny       = y;
    nisr     = isr;
    nosr     = osr;
    ncrc     = crc;
    nisr_cnt = isr_cnt;
    nosr_cnt = osr_cnt;
    mw_en    = 1'b0;
    mw_dir   = 1'b0;
    mw_base  = out_base;
    mw_cnt   = out_cnt;
    mw_data  = 16'h0;
    tx_pop   = 1'b0;
    rx_push  = 1'b0;
    rx_data  = 16'h0;
    flag_set = 8'h0;
    flag_clr = 8'h0;
    srcv     = 16'h0;
    movv     = 16'h0;
    cond     = 1'b0;
    wsat     = 1'b0;
    simm     = {{8{a[7]}}, a[7:0]};
    crc_feed = 1'b0;

    case (op)
      // ------------------------------------------------------------ JMP
      OP_JMP: begin
        case (a[10:7])
          4'd0:  cond = 1'b1;
          4'd1:  cond = (x == 16'h0);
          4'd2:  begin cond = (x != 16'h0); nx = x - 16'h1; end
          4'd3:  cond = (y == 16'h0);
          4'd4:  begin cond = (y != 16'h0); ny = y - 16'h1; end
          4'd5:  cond = (x != y);
          4'd6:  cond = pins[jmp_pin];
          4'd7:  cond = ~osre;
          4'd8:  cond = ~pins[jmp_pin];
          4'd9:  cond = ~tx_empty;
          4'd10: cond = ~rx_full;
          default: cond = 1'b0;
        endcase
        jump    = cond;
        jtarget = a[AW-1:0];
      end

      // ------------------------------------------------------------ WAIT
      OP_WAIT: begin
        case (a[9:8])
          2'd0: wsat = (pins[a[4:0]] == a[10]);
          2'd1: wsat = (rin[a[4:0]] == a[10]);
          2'd2: begin
            wsat = (flags[a[2:0]] == a[10]);
            if (wsat && a[10]) flag_clr[a[2:0]] = 1'b1;
          end
          default: wsat = (((rin[15:0] & y) == x) == a[10]);
        endcase
        stall = ~wsat;
        if (stall) flag_clr = 8'h0;
      end

      // ------------------------------------------------------------ IN
      OP_IN: begin
        crc_feed = crc_src[1] && (nbits == 5'd1);
        if (autopush && (cnt_sum >= push_th)) begin
          if (rx_full) stall = 1'b1;
          else begin
            rx_push  = 1'b1;
            rx_data  = j_out;
            nisr     = 16'h0;
            nisr_cnt = 5'd0;
          end
        end else begin
          nisr     = isr_sh;
          nisr_cnt = cnt_sum;
        end
      end

      // ------------------------------------------------------------ OUT
      OP_OUT: begin
        if (autopull && osre && tx_empty) stall = 1'b1;
        tx_pop   = refill;
        nosr     = osr_sh;
        nosr_cnt = osr_csum;
        crc_feed = crc_src[0] && (nbits == 5'd1);
        case (f3)
          3'd0: begin mw_en = 1'b1; mw_data = odata; end           // PINS
          3'd1: nx = odata;
          3'd2: ny = odata;
          3'd3: ;                                                   // NULL
          3'd4: begin mw_en = 1'b1; mw_dir = 1'b1; mw_data = odata; end
          3'd5: begin jump = 1'b1; jtarget = odata[AW-1:0]; end
          3'd6: begin nisr = odata; nisr_cnt = nbits; end
          default: ;
        endcase
      end

      // ------------------------------------------------------------ PUSH / PULL
      OP_PP: begin
        if (a[10]) begin                                            // PULL
          if (!(a[9] && !osre)) begin
            if (tx_empty) begin
              if (a[8]) stall = 1'b1;
              else begin nosr = x; nosr_cnt = 5'd0; end
            end else begin
              tx_pop = 1'b1; nosr = tx_data; nosr_cnt = 5'd0;
            end
          end
        end else begin                                              // PUSH
          if (!(a[9] && (isr_cnt < push_th))) begin
            if (rx_full && a[8]) stall = 1'b1;
            else begin
              rx_push  = ~rx_full;
              rx_data  = j_out;
              nisr     = 16'h0;
              nisr_cnt = 5'd0;
            end
          end
        end
      end

      // ------------------------------------------------------------ MOV
      OP_MOV: begin
        case (a[3:0])
          4'd0: srcv = rin[15:0];
          4'd1: srcv = x;
          4'd2: srcv = y;
          4'd3: srcv = 16'h0;
          4'd4: srcv = crc;
          4'd5: srcv = time_i;
          4'd6: srcv = isr;
          4'd7: srcv = osr;
          4'd8: srcv = status;
          default: srcv = 16'h0;
        endcase
        case (a[7:6])
          2'd0: movv = srcv;
          2'd1: movv = ~srcv;
          2'd2: movv = rev16(srcv);
          default: movv = {srcv[7:0], srcv[15:8]};
        endcase
        case (f3)
          3'd0: begin mw_en = 1'b1; mw_data = movv; end
          3'd1: nx = movv;
          3'd2: ny = movv;
          3'd3: ncrc = movv;
          3'd4: begin mw_en = 1'b1; mw_dir = 1'b1; mw_data = movv; end
          3'd5: begin jump = 1'b1; jtarget = movv[AW-1:0]; end
          3'd6: begin nisr = movv; nisr_cnt = 5'd0; end
          default: begin nosr = movv; nosr_cnt = 5'd0; end
        endcase
      end

      // ------------------------------------------------------------ FLAG
      OP_FLAG: begin
        if (a[10:9] == 2'd0) flag_set[a[2:0]] = 1'b1;
        else if (a[10:9] == 2'd1) flag_clr[a[2:0]] = 1'b1;
      end

      // ------------------------------------------------------------ SET
      OP_SET: begin
        case (f3)
          3'd0: begin mw_en = 1'b1; mw_base = set_base; mw_cnt = {1'b0, set_cnt}; mw_data = {8'h0, a[7:0]}; end
          3'd1: nx = {8'h0, a[7:0]};
          3'd2: ny = {8'h0, a[7:0]};
          3'd3: ncrc = {8'h0, a[7:0]};
          3'd4: begin mw_en = 1'b1; mw_dir = 1'b1; mw_base = set_base; mw_cnt = {1'b0, set_cnt}; mw_data = {8'h0, a[7:0]}; end
          3'd5: nx = {a[7:0], x[7:0]};
          3'd6: ny = {a[7:0], y[7:0]};
          default: ;
        endcase
      end

      // ------------------------------------------------------------ ADD
      OP_ADD: begin
        if (a[10]) ny = y + simm;
        else       nx = x + simm;
      end

      default: ;  // reserved opcodes execute as NOP
    endcase

    if (crc_feed) ncrc = crc_out;

    if (!go || stall) begin
      // nothing but side-set happens
      jump     = 1'b0;
      nx       = x;
      ny       = y;
      nisr     = isr;
      nosr     = osr;
      ncrc     = crc;
      nisr_cnt = isr_cnt;
      nosr_cnt = osr_cnt;
      mw_en    = 1'b0;
      tx_pop   = 1'b0;
      rx_push  = 1'b0;
      flag_set = 8'h0;
      flag_clr = 8'h0;
    end
  end


  // ---------------------------------------------------------------- pin writes
  // main op write
  wire [15:0] mw_m16  = mask16(mw_cnt > 5'd16 ? 5'd16 : mw_cnt);
  wire [31:0] mw_mask = rotl32({16'h0, mw_m16}, mw_base);
  wire [31:0] mw_val  = rotl32({16'h0, mw_data & mw_m16}, mw_base);
  // side-set write (applied whenever the instruction is presented, even if stalled)
  wire        sw_en   = go & side_en & (side_cnt != 2'd0);
  wire [15:0] sw_m16  = mask16({3'b0, side_cnt});
  wire [31:0] sw_mask = rotl32({16'h0, sw_m16}, side_base);
  wire [31:0] sw_val  = rotl32({16'h0, {13'h0, side_v} & sw_m16}, side_base);

  // classify into value / direction writes (open-drain: value write => dir = ~v, value = 0)
  wire        m_isdir = mw_dir;
  wire        s_isdir = side_pindir;
  wire [31:0] m_vm = (mw_en & ~m_isdir) ? mw_mask : 32'h0;
  wire [31:0] m_dm = (mw_en & (m_isdir | od)) ? mw_mask : 32'h0;
  wire [31:0] m_dv = m_isdir ? mw_val : ~mw_val;
  wire [31:0] m_vv = od ? 32'h0 : mw_val;
  wire [31:0] s_vm = (sw_en & ~s_isdir) ? sw_mask : 32'h0;
  wire [31:0] s_dm = (sw_en & (s_isdir | od)) ? sw_mask : 32'h0;
  wire [31:0] s_dv = s_isdir ? sw_val : ~sw_val;
  wire [31:0] s_vv = od ? 32'h0 : sw_val;

  assign pv_mask = m_vm | s_vm;
  assign pv_val  = (s_vv & s_vm) | (m_vv & m_vm & ~s_vm);
  assign pd_mask = m_dm | s_dm;
  assign pd_val  = (s_dv & s_dm) | (m_dv & m_dm & ~s_dm);

  assign exec_done = exec_valid & ~stall;

  // ---------------------------------------------------------------- sequential
  wire [AW-1:0] pc_inc = (pc == wrap_top) ? wrap_bot : pc + 1'b1;

  // next PC is computed combinationally so the cluster can fetch imem[pc_next]
  // into ir_q in the same cycle: the 64:1 IMEM read leaves the execute path.
  always @* begin
    pc_next = pc;
    if (go && !stall) begin
      if (jump)             pc_next = jtarget;
      else if (!exec_valid) pc_next = pc_inc;
    end
    if (pc_we) pc_next = pc_wdata;
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pc      <= {AW{1'b0}};
      ir_q    <= 24'h0;
      x       <= 16'h0;
      y       <= 16'h0;
      isr     <= 16'h0;
      osr     <= 16'h0;
      crc     <= 16'h0;
      isr_cnt <= 5'd0;
      osr_cnt <= 5'd16;
      dly     <= 5'd0;
    end else begin
      pc   <= pc_next;
      ir_q <= imem_next;
      if (go && !stall) begin
        x       <= nx;
        y       <= ny;
        isr     <= nisr;
        osr     <= nosr;
        crc     <= ncrc;
        isr_cnt <= nisr_cnt;
        osr_cnt <= nosr_cnt;
        dly     <= idly;
      end else if (tick && !exec_valid && dly != 5'd0) begin
        dly <= dly - 5'd1;
      end
      if (pc_we) dly <= 5'd0;
      if (restart) begin
        isr     <= 16'h0;
        osr     <= 16'h0;
        isr_cnt <= 5'd0;
        osr_cnt <= 5'd16;
        dly     <= 5'd0;
      end
    end
  end

  // ---------------------------------------------------------------- formal
`ifdef FORMAL
  reg f_init = 1'b1;
  always @(posedge clk) f_init <= 1'b0;
  always @* if (f_init) assume (!rst_n);   // start in reset
  reg f_past = 1'b0;
  always @(posedge clk) f_past <= 1'b1;
  wire f_host = pc_we | restart;

  always @* if (rst_n) begin
    assert (isr_cnt <= 5'd16);                 // shift counters never exceed width
    assert (osr_cnt <= 5'd16);
    if (rx_push) assert (!rx_full);            // never push into a full FIFO
    if (tx_pop)  assert (!tx_empty);           // never pop an empty FIFO
    if (exec_done) assert (exec_valid);
    if (!go) assert (!rx_push && !tx_pop && flag_set == 8'h0 && flag_clr == 8'h0 && pv_mask == 32'h0 && pd_mask == 32'h0);
  end

  always @(posedge clk) if (f_past && rst_n && $past(rst_n) && !$past(f_host)) begin
    // a stalled instruction has no architectural side effect
    if ($past(go && stall)) begin
      assert (pc == $past(pc));
      assert (x == $past(x) && y == $past(y) && crc == $past(crc));
      assert (isr == $past(isr) && osr == $past(osr));
      assert (isr_cnt == $past(isr_cnt) && osr_cnt == $past(osr_cnt));
    end
    // cycle-exact timing: a completed instruction loads exactly its delay ...
    if ($past(go && !stall)) assert (dly == $past(idly));
    // ... and every tick of delay counts down by one without executing
    if ($past(!exec_valid && tick && dly != 5'd0)) begin
      assert (dly == $past(dly) - 5'd1);
      assert (pc == $past(pc) && x == $past(x) && osr_cnt == $past(osr_cnt));
    end
    // no tick, no exec => frozen
    if ($past(!tick && !exec_valid)) assert (pc == $past(pc) && dly == $past(dly) && x == $past(x));
  end
`endif

endmodule
