#!/usr/bin/env python3
"""Minimal NLDM static timing estimate for a yosys-mapped netlist.

Not sign-off: no wires (a fixed per-fanout wire cap is added), no clock tree,
no SI.  It is for early architectural feedback before running the Tiny Tapeout
(LibreLane/OpenROAD) flow, which is the authoritative timing check.

usage: mini_sta.py netlist.v cells.lib [--wire-ff 4.0] [--top N]
SPDX-License-Identifier: Apache-2.0
"""
import argparse
import bisect
import re
import sys
from collections import defaultdict

from liberty.parser import parse_liberty


def interp2(i1, i2, tab, x, y):
    def bracket(ax, v):
        k = max(1, min(len(ax) - 1, bisect.bisect_left(ax, v)))
        return k - 1, k
    a0, a1 = bracket(i1, x)
    b0, b1 = bracket(i2, y)
    x0, x1, y0, y1 = i1[a0], i1[a1], i2[b0], i2[b1]
    tx = (x - x0) / (x1 - x0) if x1 != x0 else 0
    ty = (y - y0) / (y1 - y0) if y1 != y0 else 0
    q00, q01, q10, q11 = tab[a0][b0], tab[a0][b1], tab[a1][b0], tab[a1][b1]
    return (q00 * (1 - tx) * (1 - ty) + q10 * tx * (1 - ty) + q01 * (1 - tx) * ty + q11 * tx * ty)


def _nums(s):
    if isinstance(s, list):
        s = ",".join(str(x) for x in s)
    return [float(v) for v in str(s).replace('"', "").replace("\\", "").split(",") if v.strip()]


def table(g):
    i1 = _nums(g["index_1"]) if g["index_1"] is not None else [0.0]
    i2 = _nums(g["index_2"]) if g["index_2"] is not None else [0.0]
    vals = g["values"]
    vals = vals if isinstance(vals, list) else [vals]
    rows = [_nums(v) for v in vals]
    if len(rows) == 1 and len(i2) == 1 and len(rows[0]) == len(i1):   # 1-D table
        rows = [[v] for v in rows[0]]
    if len(i2) == 1:
        i2 = [0.0, 1.0]
        rows = [[r[0], r[0]] for r in rows]
    if len(i1) == 1:
        i1 = [0.0, 1.0]
        rows = [rows[0], rows[0]]
    return i1, i2, rows


def load_lib(path):
    lib = parse_liberty(open(path).read())
    cells = {}
    for c in lib.get_groups("cell"):
        name = str(c.args[0]).strip('"')
        pins = {}
        arcs = []
        seq = bool(c.get_groups("ff")) or bool(c.get_groups("latch"))
        for p in c.get_groups("pin"):
            pn = str(p.args[0]).strip('"')
            d = str(p["direction"]).strip('"')
            cap = float(p["capacitance"]) if p["capacitance"] is not None else 0.0
            pins[pn] = (str(d), cap)
            for t in p.get_groups("timing"):
                rel = str(t["related_pin"]).strip('"')
                ttype = str(t["timing_type"] or "combinational").strip('"')
                tabs = {}
                for k in ("cell_rise", "cell_fall", "rise_transition", "fall_transition",
                          "rise_constraint", "fall_constraint"):
                    gs = t.get_groups(k)
                    if gs:
                        tabs[k] = table(gs[0])
                arcs.append((rel, pn, ttype, tabs))
        cells[name] = dict(pins=pins, arcs=arcs, seq=seq)
    return cells


def parse_netlist(path):
    txt = open(path).read()
    insts = []
    for m in re.finditer(r"\b(sg13\w+)\s+(\S+)\s*\((.*?)\);", txt, re.S):
        conns = dict(re.findall(r"\.(\w+)\(([^()]*?)\)", m.group(3)))
        insts.append((m.group(1), m.group(2), {k: v.strip() for k, v in conns.items()}))
    return insts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("netlist")
    ap.add_argument("lib")
    ap.add_argument("--wire-ff", type=float, default=4.0, help="wire cap per fanout (fF)")
    ap.add_argument("--in-slew", type=float, default=0.1)
    ap.add_argument("--max-fanout", type=int, default=8,
                    help="model resizer buffering: nets above this get a buffer tree")
    ap.add_argument("--path", action="store_true")
    ns = ap.parse_args()
    cells = load_lib(ns.lib)
    insts = parse_netlist(ns.netlist)
    unit = 1.0  # liberty units: ns / pF (IHP) -> capacitance in pF
    wire = ns.wire_ff / 1000.0

    drivers = {}
    loads = defaultdict(list)
    for ci, (ct, name, conns) in enumerate(insts):
        cell = cells[ct]
        for pn, net in conns.items():
            if pn not in cell["pins"] or not net:
                continue
            d, cap = cell["pins"][pn]
            if d == "output":
                drivers[net] = (ci, pn)
            else:
                loads[net].append((ci, pn, cap))
    # Buffer-tree model (what OpenROAD repair_design would insert): the driver
    # sees at most max_fanout loads; every 4x of extra fanout costs one buf_4.
    import math
    buf = cells.get("sg13cmos5l_buf_4")
    avg_in = 0.003
    netcap, netextra = {}, {}
    for n, l in loads.items():
        f = len(l)
        if f > ns.max_fanout:
            stages = math.ceil(math.log(f / ns.max_fanout, 4))
            arc = next(a for a in buf["arcs"] if a[1] == "X")[3]
            bd = interp2(*arc["cell_rise"], 0.1, 4 * (avg_in + wire))
            netcap[n] = ns.max_fanout * (avg_in + wire)
            netextra[n] = stages * bd
        else:
            netcap[n] = sum(c for _, _, c in l) + wire * f
            netextra[n] = 0.0

    arrival = {}   # net -> (time, slew, path)
    # start points: flop outputs (clk->Q) and primary inputs (t=0)
    order = []
    indeg = {}
    comb_out = {}
    for ci, (ct, name, conns) in enumerate(insts):
        cell = cells[ct]
        if cell["seq"]:
            for rel, pn, tt, tabs in cell["arcs"]:
                if tt.startswith("rising_edge") and pn in conns:
                    net = conns[pn]
                    load = netcap.get(net, 0)
                    t = max(interp2(*tabs[k], ns.in_slew, load) for k in ("cell_rise", "cell_fall") if k in tabs)
                    s = max(interp2(*tabs[k], ns.in_slew, load) for k in ("rise_transition", "fall_transition") if k in tabs)
                    t += netextra.get(net, 0.0)
                    if t > arrival.get(net, (-1,))[0]:
                        arrival[net] = (t, s, [f"{name}({ct}).{pn} -> {net}"])
        else:
            ins = [conns[p] for p in conns if p in cell["pins"] and cell["pins"][p][0] == "input"]
            indeg[ci] = sum(1 for n in ins if n in drivers and not cells[insts[drivers[n][0]][0]]["seq"])
            comb_out[ci] = ins
    for n in loads:
        if n not in drivers and n not in arrival:
            arrival[n] = (0.0, ns.in_slew, [f"input {n}"])
    # topological propagation
    succ = defaultdict(list)
    for net, (ci, pn) in drivers.items():
        if cells[insts[ci][0]]["seq"]:
            continue
        for lci, _, _ in loads.get(net, []):
            if not cells[insts[lci][0]]["seq"]:
                succ[ci].append(lci)
    ready = [ci for ci, d in indeg.items() if d == 0]
    done = 0
    while ready:
        ci = ready.pop()
        done += 1
        ct, name, conns = insts[ci]
        cell = cells[ct]
        for rel, pn, tt, tabs in cell["arcs"]:
            if rel not in conns or pn not in conns or "cell_rise" not in tabs and "cell_fall" not in tabs:
                continue
            a = arrival.get(conns[rel])
            if a is None:
                continue
            net = conns[pn]
            load = netcap.get(net, 0)
            d = max(interp2(*tabs[k], a[1], load) for k in ("cell_rise", "cell_fall") if k in tabs)
            s = max(interp2(*tabs[k], a[1], load) for k in ("rise_transition", "fall_transition") if k in tabs)
            t = a[0] + d + netextra.get(net, 0.0)
            if t > arrival.get(net, (-1,))[0]:
                arrival[net] = (t, s, a[2] + [f"{name}({ct}).{pn} +{d:.3f} -> {net} @{t:.2f}"])
        for s2 in succ[ci]:
            indeg[s2] -= 1
            if indeg[s2] == 0:
                ready.append(s2)
    if done != len(indeg):
        print(f"warning: {len(indeg) - done} combinational cells not reached (loops?)", file=sys.stderr)
    # endpoints: flop D pins (+ setup)
    worst = (0, None, None)
    for ci, (ct, name, conns) in enumerate(insts):
        cell = cells[ct]
        if not cell["seq"]:
            continue
        for rel, pn, tt, tabs in cell["arcs"]:
            if tt.startswith("setup_rising") and pn in conns and conns[pn] in arrival:
                a = arrival[conns[pn]]
                su = max(interp2(*tabs[k], a[1], ns.in_slew) for k in ("rise_constraint", "fall_constraint") if k in tabs)
                if a[0] + su > worst[0]:
                    worst = (a[0] + su, f"{name}.{pn}", a[2])
    print(f"worst reg->reg path: {worst[0]:.2f} ns (endpoint {worst[1]}, {len(worst[2])} stages)")
    print(f"  => fmax ~ {1000 / worst[0]:.0f} MHz (no wires/clock skew; add margin)")
    if ns.path:
        for step in worst[2]:
            print("   ", step)


if __name__ == "__main__":
    main()
