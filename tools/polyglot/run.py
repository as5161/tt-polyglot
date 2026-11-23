"""pgrun - run Polyglot programs on the cycle-accurate model, no hardware needed.

    python -m polyglot.run programs/uart_tx.pga --push 0x48,0x69 --cycles 3000 --vcd uart.vcd
    python -m polyglot.run a.pga b.pga --push 1:0x9f   # several programs, one per core

Loads each program on its own core (packing them into IMEM), pushes the given
words into the TX FIFOs, runs, then prints RX FIFO contents and writes a VCD
of all pins (open it in GTKWave or Surfer).

SPDX-License-Identifier: Apache-2.0
"""
import argparse
import sys

from .asm import AsmError, load_file
from .sim import Cluster


class Bus:
    """Applies host operations to the model one per cycle."""

    def __init__(self, m):
        self.m = m
        self.pins = 0
        self.trace = []

    def cyc(self, hw=0, hr=0, addr=0, data=0):
        r = self.m.cycle(hw, hr, addr, data, self.pins)
        self.trace.append((self.m.pout, self.m.poe, self.pins))
        return r

    def write(self, addr, *words):
        for i, w in enumerate(words):
            self.cyc(1, 0, addr + i if addr < 0x80 else addr, w)


def load(bus, prog, core, origin):
    bus.write(origin, *prog.relocated(origin))
    r = prog.regs(origin)
    for n in range(5):
        bus.cyc(1, 0, 0x80 + 8 * core + n, r[n])
    bus.cyc(1, 0, 0xA8 + core, prog.cfg["poly"])
    bus.cyc(1, 0, 0x80 + 8 * core + 6, origin)


def write_vcd(path, trace, names):
    ids = {n: chr(33 + i) for i, n in enumerate(names)}
    with open(path, "w") as f:
        f.write("$timescale 1ns $end\n$scope module polyglot $end\n")
        for n in names:
            f.write(f"$var wire 1 {ids[n]} {n} $end\n")
        f.write("$upscope $end\n$enddefinitions $end\n")
        prev = {}
        for t, (pout, poe, pins) in enumerate(trace):
            vals = {}
            for i in range(8):
                vals[f"uio{i}"] = (pout >> i) & 1 if (poe >> i) & 1 else (pins >> i) & 1
                vals[f"uio{i}_oe"] = (poe >> i) & 1
            for i in range(7):
                vals[f"uo{i}"] = (pout >> (8 + i)) & 1
            for i in range(5):
                vals[f"ui{i}"] = (pins >> (16 + i)) & 1
            ch = [n for n in names if prev.get(n) != vals[n]]
            if ch:
                f.write(f"#{t}\n" + "".join(f"{vals[n]}{ids[n]}\n" for n in ch))
            prev = vals


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run Polyglot programs on the ISS")
    ap.add_argument("programs", nargs="+")
    ap.add_argument("--push", default="", help="words for core 0, or core:w,w;core:w")
    ap.add_argument("--cycles", type=int, default=2000)
    ap.add_argument("--clkdiv", type=int, help="override clkdiv of all programs")
    ap.add_argument("--vcd")
    ns = ap.parse_args(argv)
    m = Cluster()
    bus = Bus(m)
    origin = 0
    try:
        progs = [load_file(p) for p in ns.programs]
    except AsmError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    for c, p in enumerate(progs):
        if ns.clkdiv is not None:
            p.cfg["clkdiv"] = ns.clkdiv
        if origin + len(p.words) > 64:
            print("error: programs do not fit in 64 words of IMEM", file=sys.stderr)
            return 1
        load(bus, p, c, origin)
        print(f"core {c}: {p.name} at {origin}..{origin + len(p.words) - 1}")
        origin += len(p.words)
    pushes = {}
    if ns.push:
        for grp in ns.push.split(";"):
            core, vals = (grp.split(":", 1) if ":" in grp else ("0", grp))
            pushes[int(core)] = [int(v, 0) for v in vals.split(",") if v]
    mask = (1 << len(progs)) - 1
    bus.cyc(1, 0, 0xA0, mask | mask << 4 | mask << 8)
    t0 = len(bus.trace)
    queue = {c: list(v) for c, v in pushes.items()}
    rx = {c: [] for c in range(len(progs))}
    while len(bus.trace) - t0 < ns.cycles:
        did = False
        for c, q in queue.items():
            if q and not m.cores[c].tx.full:
                bus.cyc(1, 0, 0x80 + 8 * c + 5, q.pop(0))
                did = True
                break
        if not did:
            for c in rx:
                if not m.cores[c].rx.empty:
                    rx[c].append(bus.cyc(0, 1, 0x80 + 8 * c + 5) & 0xFFFF)
                    did = True
                    break
        if not did:
            bus.cyc()
    for c, v in rx.items():
        if v:
            print(f"core {c} RX: " + " ".join(f"0x{x:04x}" for x in v))
    print(f"flags=0x{m.flags:02x}  " + "  ".join(f"c{i}.pc={k.pc}" for i, k in enumerate(m.cores[:len(progs)])))
    if ns.vcd:
        names = [f"uo{i}" for i in range(7)] + [f"uio{i}" for i in range(8)] + \
                [f"uio{i}_oe" for i in range(8)] + [f"ui{i}" for i in range(5)]
        write_vcd(ns.vcd, bus.trace, names)
        print(f"wrote {ns.vcd}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
