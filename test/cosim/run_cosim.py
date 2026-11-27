#!/usr/bin/env python3
"""Constrained-random lock-step co-simulation: RTL (iverilog) vs golden ISS.

For every seed: generate a random program image + random configuration +
random host traffic + random pin activity, run both models for N cycles and
compare ~50 architectural state fields every single cycle.

    python3 run_cosim.py --seeds 200 --cycles 4000

SPDX-License-Identifier: Apache-2.0
"""
import argparse
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from polyglot.sim import Cluster  # noqa: E402
from polyglot.asm import disassemble  # noqa: E402

DEPTH = 128  # words per core bank (AW=7)
NC = 4       # cores (project.v)
BANK = 0x400  # IMEM of core c: BANK + 0x100 * c + word
SRC = ["pg_cluster.v", "pg_core.v", "pg_fifo.v", "pg_bank.v"]
FIELDS = ["hrdata", "pout", "poe", "flags", "en"] + [
    f"c{i}.{n}" for i in range(NC) for n in
    ("pc", "x", "y", "isr", "osr", "crc", "isr_cnt", "osr_cnt", "dly", "tx_cnt", "rx_cnt", "exec_pend", "rx_head", "ir")]


WEIGHTS = [14, 6, 12, 12, 8, 12, 5, 12, 8, 2]


def rand_instr(r, mode, weights=None):
    """Random instruction, biased towards meaningful encodings."""
    if r.random() < 0.03:
        return r.getrandbits(24)                     # anything, incl. reserved ops
    op = r.choices(range(10), weights=weights or WEIGHTS)[0]
    if op == 9:
        op = r.randrange(9, 16)
    side = r.random() < (0.3 if mode != "side" else 0.8)
    dly = r.choice([0, 0, 0, 0, 1, 2, 3, r.randrange(32)]) if mode != "fast" else r.choice([0, 0, 1])
    a = r.getrandbits(11)
    if op == 0:
        # jump target in [6:0]; mostly inside the loaded region so programs loop
        a = (r.choices(range(16), weights=[3] + [4] * 10 + [1] * 5)[0] << 7) | r.choice([r.randrange(24), r.randrange(128)])
    elif op == 1:
        src = r.choice([0, 1, 2, 3])
        a = (r.getrandbits(1) << 10) | (src << 8) | (r.randrange(24) if src != 2 else r.randrange(8))
    elif op in (2, 3) and r.random() < 0.25:
        a = (a & ~0xF) | 1                             # 1-bit shifts feed the CRC
    w = (op << 20) | (side << 19) | (r.getrandbits(3) << 16) | (dly << 11) | a
    return w


def gen(seed, cycles):
    r = random.Random(seed)
    stim = []

    def op(hw=0, hr=0, addr=0, data=0, pk=0):
        stim.append([hw, hr, addr, data, pk])

    mode = r.choice(["fast", "normal", "normal", "side"])
    # per-seed opcode mix: half the seeds focus on a random subset of opcodes
    wts = WEIGHTS if r.random() < 0.5 else [w * r.choice([0, 1, 4, 8]) + (8 if i == 0 else 0)
                                             for i, w in enumerate(WEIGHTS)]
    # each core gets its own random image; a short program region keeps most
    # control flow inside it
    plen = r.choice([24, 48, DEPTH])
    for c in range(4):
        for i in range(plen):
            op(1, 0, BANK + 0x100 * c + i, rand_instr(r, mode, wts))
    for c in range(4):
        b = 0x80 + 8 * c
        op(1, 0, b + 0, r.choice([0, 0, 1, 2, 3, r.randrange(8)]))
        wb, wt = sorted([r.randrange(plen), r.randrange(plen)])
        op(1, 0, b + 1, wb | wt << 7 | r.randrange(32) << 14 | r.getrandbits(2) << 19)
        op(1, 0, b + 2, r.getrandbits(17))
        pb = lambda: r.choice([r.randrange(32), r.randrange(4), r.randrange(8, 12)])  # noqa: E731
        ob, sb = pb(), pb()
        if mode == "side":
            ob = sb = r.randrange(8)       # force side-set / main-op pin overlap
        op(1, 0, b + 3, ob | r.randrange(1, 17) << 5 | sb << 10 | r.randrange(1, 9) << 15)
        op(1, 0, b + 4, pb() | sb << 5 | r.randrange(1, 4) << 10)
        op(1, 0, b + 6, r.randrange(plen))
        op(1, 0, 0xA8 + c, r.getrandbits(16))
    op(1, 0, 0xA0, r.randrange(1, 16) | 0xFF0)
    rate = r.choice([0.02, 0.05, 0.2])
    while len(stim) < cycles:
        if r.random() < rate:
            k = r.random()
            c = r.randrange(4)
            b = 0x80 + 8 * c
            if k < 0.35:
                op(1, 0, b + 5, r.getrandbits(24))            # push TX
            elif k < 0.6:
                op(0, 1, b + 5)                               # pop RX
            elif k < 0.64:
                op(0, 1, r.randrange(256))                    # random read (side effects!)
            elif k < 0.7:
                # IMEM read peek, sometimes followed by the "data" cycle
                a = BANK + 0x100 * c + r.randrange(DEPTH + 8)
                op(0, 0, a, 0, 1)
                if r.random() < 0.7:
                    op(0, r.random() < 0.2, a)
            elif k < 0.76:
                op(1, 0, b + 7, rand_instr(r, mode, wts))          # exec
            elif k < 0.8:
                op(1, 0, b + 6, r.randrange(128))             # pc
            elif k < 0.85:
                op(1, 0, 0xA0, r.getrandbits(16))             # ctrl
            elif k < 0.89:
                op(1, 0, 0xA1, r.getrandbits(4))              # step
            elif k < 0.92:
                op(1, 0, 0xA2, r.getrandbits(8))              # flags
            elif k < 0.97:
                op(1, 0, b + r.randrange(5), r.getrandbits(24))  # reconfigure
            else:
                # bank write: running core => ignored; also out-of-range words
                # and other pages
                a = r.choice([BANK + 0x100 * c + r.randrange(DEPTH + 8), r.getrandbits(15)])
                op(1, 0, a, rand_instr(r, mode, wts))
        else:
            op(0, 0, r.choice([r.randrange(256), r.getrandbits(15)]), r.getrandbits(24),
               int(r.random() < 0.02))
    del stim[cycles:]      # a two-cycle operation can overshoot by one
    # pin activity: random walk with per-seed toggle rate
    pins = r.getrandbits(32)
    tog = r.choice([0.01, 0.05, 0.3])
    for s in stim:
        for _ in range(3):
            if r.random() < tog:
                pins ^= 1 << r.randrange(24)
        s.append(pins)
    return stim


COV = {}


def run(seed, cycles, workdir, verbose=False):
    stim = gen(seed, cycles)
    with open(os.path.join(workdir, "stim.hex"), "w") as f:
        for hw, hr, addr, data, pk, pins in stim:
            v = (pk << 73) | (hw << 72) | (hr << 71) | (addr << 56) | (data << 32) | pins
            f.write(f"{v:019x}\n")
    subprocess.run(["vvp", "-n", os.path.join(workdir, "sim.vvp"), f"+N={cycles}"],
                   cwd=workdir, check=True, stdout=subprocess.DEVNULL)
    with open(os.path.join(workdir, "rtl_trace.txt")) as f:
        txt = f.read()
    if True:
        rtl = [[int(t, 16) if not set(t) & set("xXzZ") else t for t in ln.split()] for ln in txt.splitlines()]

    m = Cluster()
    for c in m.cores:
        c.cov = COV
    for cyc, (hw, hr, addr, data, pk, pins) in enumerate(stim):
        pcs = [c.pc for c in m.cores]
        irs = list(m.dout)
        rd = m.cycle(hw, hr, addr, data, pins, pk)
        exp = [rd] + m.snapshot()
        got = rtl[cyc]
        for i, (e, g) in enumerate(zip(exp, got)):
            if i == 0 and e is None:
                continue
            if False:
                # empty FIFO read returns stale RAM contents: only compare the valid bit
                if 0x80 <= addr < 0xA0 and addr & 7 == 5 and not (e >> 23):
                    e = e >> 23
                    g = (g >> 23) if isinstance(g, int) else (int(g[0], 16) >> 3 if g[0] in "0123456789abcdef" else g)
            if e != g:
                print(f"seed {seed}: MISMATCH cycle {cyc} field {FIELDS[i]}: iss={e} rtl={g}")
                print(f"  stim hw={hw} hr={hr} peek={pk} addr={addr:04x} data={data:06x} pins={pins:08x}")
                for ci, pc in enumerate(pcs):
                    w = irs[ci]
                    print(f"  core{ci} pc={pc:3d} ir={w:06x} {disassemble(w)}")
                return False
    if verbose:
        print(f"seed {seed}: ok ({cycles} cycles)")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=50)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--cycles", type=int, default=3000)
    ap.add_argument("-v", action="store_true")
    ap.add_argument("--cov", action="store_true")
    ap.add_argument("--src", default=os.path.join(ROOT, "src"))
    ap.add_argument("--work", default=os.path.join(HERE, "work"), help="scratch directory (one per concurrent run)")
    ns = ap.parse_args()
    work = ns.work
    os.makedirs(work, exist_ok=True)
    subprocess.run(["iverilog", "-g2005", "-o", os.path.join(work, "sim.vvp"),
                    os.path.join(HERE, "tb_cosim.v")] +
                   [os.path.join(ns.src, s) for s in SRC], check=True)
    fails = 0
    for s in range(ns.start, ns.start + ns.seeds):
        if not run(s, ns.cycles, work, ns.v):
            fails += 1
            if fails >= 3:
                break
    total = ns.seeds
    if ns.cov:
        names = ["jmp", "wait", "in", "out", "push/pull", "mov", "flag", "set", "add"]
        bins = [(o, s) for o in range(9) for s in
                {0: range(11), 1: range(4), 2: range(8), 3: range(7), 4: range(8),
                 5: range(8), 6: [0], 7: range(7), 8: [0]}[o]]
        hit = sum(1 for b in bins if COV.get(b))
        print(f"coverage: {hit}/{len(bins)} opcode/variant bins hit")
        for b in bins:
            if not COV.get(b):
                print("  unhit:", names[b[0]], b[1])
        st = {k[1]: v for k, v in COV.items() if k[0] == "stall"}
        print("  stall events by opcode:", {names[k]: v for k, v in st.items() if k < 9})
    print(f"cosim: {total - fails if fails == 0 else '?'}/{total} seeds passed, "
          f"{total * ns.cycles} cycles compared" if fails == 0 else f"cosim: {fails} FAILURES")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
