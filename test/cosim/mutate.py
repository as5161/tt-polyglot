#!/usr/bin/env python3
"""Mutation testing of the verification environment.

Each mutant injects one realistic RTL bug.  The co-simulation must detect
every mutant; a surviving mutant means a hole in the stimulus/checker.

SPDX-License-Identifier: Apache-2.0
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", "..", "src"))

MUTANTS = [
    ("pg_core.v", "4'd2:  begin cond = (x != 16'h0); nx = x - 16'h1; end",
     "4'd2:  begin cond = (x != 16'h0); nx = x - 16'h2; end", "X-- decrements by 2"),
    ("pg_core.v", "4'd5:  cond = (x != y);", "4'd5:  cond = (x == y);", "X!=Y inverted"),
    ("pg_core.v", "4'd7:  cond = ~osre;", "4'd7:  cond = osre;", "!OSRE inverted"),
    ("pg_core.v", "wire        osre    = (osr_cnt >= pull_th);", "wire        osre    = (osr_cnt > pull_th);",
     "OSR-empty off by one"),
    ("pg_core.v", "if (autopush && (cnt_sum >= push_th)) begin", "if (autopush && (cnt_sum > push_th)) begin",
     "autopush threshold off by one"),
    ("pg_core.v", "wire [15:0] j_out   = in_dir ? j_v : (j_v >> (5'd16 - j_cnt));",
     "wire [15:0] j_out   = j_v;", "push not right-justified"),
    ("pg_core.v", "2'd2: movv = rev16(srcv);", "2'd2: movv = srcv;", "MOV bit-reverse missing"),
    ("pg_core.v", "fb = t[0] ^ s[i];", "fb = t[0] ^ s[i];", "(placeholder)"),
    ("pg_core.v", "fb = c[0] ^ b;", "fb = c[1] ^ b;", "reflected CRC taps wrong bit"),
    ("pg_core.v", "assign pv_val  = (s_vv & s_vm) | (m_vv & m_vm & ~s_vm);",
     "assign pv_val  = (m_vv & m_vm) | (s_vv & s_vm & ~m_vm);", "side-set priority swapped"),
    ("pg_core.v", "wire [31:0] m_vv = od ? 32'h0 : mw_val;", "wire [31:0] m_vv = mw_val;",
     "open-drain drives high"),
    ("pg_core.v", "end else if (tick && !exec_valid && dly != 5'd0) begin",
     "end else if (!exec_valid && dly != 5'd0) begin", "delay counts sysclk not ticks"),
    ("pg_core.v", "if (wsat && a[10]) flag_clr[a[2:0]] = 1'b1;", "", "WAIT FLAG doesn't auto-clear"),
    ("pg_core.v", "wire [AW-1:0] pc_inc = (pc == wrap_top) ? wrap_bot : pc + 1'b1;",
     "wire [AW-1:0] pc_inc = pc + 1'b1;", "wrap ignored"),
    ("pg_core.v", "3'd5: nx = {a[7:0], x[7:0]};", "3'd5: nx = {a[7:0], 8'h0};", "SET XH clobbers low byte"),
    ("pg_cluster.v", "npout = (npout & ~c_pvm[i][15:0]) | (c_pvv[i][15:0] & c_pvm[i][15:0]);",
     "npout = npout | (c_pvv[i][15:0] & c_pvm[i][15:0]);", "pin write can't clear"),
    ("pg_cluster.v", "else if (en[c]) divcnt <= (divcnt >= div_m1) ? 16'd0 : divcnt + 16'd1;",
     "else divcnt <= (divcnt >= div_m1) ? 16'd0 : divcnt + 16'd1;", "divider runs when disabled"),
    ("pg_fifo.v", "assign full  = (cnt == 3'd4);", "assign full  = (cnt == 3'd3);", "FIFO depth 3"),
    ("pg_cluster.v", "end else if (exec_done || restart) begin", "end else if (exec_done) begin",
     "restart doesn't cancel EXEC"),
    ("pg_cluster.v", "wire [23:0] imem_next = imem_wr_hit ? hwdata : imem[pc_next];",
     "wire [23:0] imem_next = imem[pc_next];", "fetch bypass missing"),
    ("pg_core.v", "    if (pc_we) pc_next = pc_wdata;\n", "", "host PC write not fetched"),
    ("pg_core.v", "4'd8:  cond = ~pins[jmp_pin];", "4'd8:  cond = ~pins[jmp_pin+1];", "!PIN wrong pin"),
]


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    survived = 0
    tested = 0
    for fname, old, new, desc in MUTANTS:
        if old == new:
            continue
        tmp = tempfile.mkdtemp()
        for f in os.listdir(SRC):
            shutil.copy(os.path.join(SRC, f), tmp)
        p = os.path.join(tmp, fname)
        s = open(p).read()
        if old not in s:
            print(f"  [skip] pattern not found: {desc}")
            continue
        open(p, "w").write(s.replace(old, new, 1))
        r = subprocess.run([sys.executable, os.path.join(HERE, "run_cosim.py"), "--seeds", str(seeds),
                            "--cycles", "2000", "--src", tmp], capture_output=True, text=True)
        tested += 1
        killed = r.returncode != 0
        survived += not killed
        first = next((ln for ln in r.stdout.splitlines() if "MISMATCH" in ln), "")
        print(f"  [{'KILLED' if killed else 'SURVIVED'}] {desc:36s} {first[:70]}")
        shutil.rmtree(tmp)
    print(f"mutation score: {tested - survived}/{tested} mutants killed")
    sys.exit(1 if survived else 0)


if __name__ == "__main__":
    main()
