#!/usr/bin/env python3
"""Arithmetic checks on the macro floorplan in src/config.json, run before
any GDS build (seconds instead of a 5-minute CI round trip).

Checks every SRAM instance against the tile geometry: on the placement-site
grid, inside the core, clear of the other macros *including both halos*,
and that no placement obstruction covers a macro.

    python3 tools/check_floorplan.py

SPDX-License-Identifier: Apache-2.0
"""
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Tiny Tapeout 8x4 tile on SG13CMOS5L (from the gds logs: 3580 sites x 186 rows)
DIE_W, DIE_H = 1724.16, 710.64
SITE_W, ROW_H = 0.48, 3.78
CORE_X0 = 6 * SITE_W              # LEFT_MARGIN_MULT 6
CORE_Y0 = 1 * ROW_H               # BOTTOM_MARGIN_MULT 1
CORE_X1 = DIE_W - 6 * SITE_W
CORE_Y1 = CORE_Y0 + 186 * ROW_H
MACRO_SIZE = {"RM_IHPSG13_1P_256x32_c2_bm_bist": (416.64, 118.78)}
EPS = 1e-6


def on_grid(v, origin, pitch):
    q = (v - origin) / pitch
    return abs(q - round(q)) < 1e-6


def main():
    cfg = json.load(open(os.path.join(ROOT, "src", "config.json")))
    hx = float(cfg.get("FP_MACRO_HORIZONTAL_HALO", 0))
    hy = float(cfg.get("FP_MACRO_VERTICAL_HALO", 0))
    errs = []
    boxes = []
    for master, m in cfg.get("MACROS", {}).items():
        w, h = MACRO_SIZE[master]
        for name, inst in m["instances"].items():
            x, y = inst["location"]
            box = (x, y, x + w, y + h)
            boxes.append((name, box))
            if not on_grid(x, CORE_X0, SITE_W):
                errs.append(f"{name}: x={x} is not on the {SITE_W} um site grid")
            if not on_grid(y, CORE_Y0, ROW_H):
                errs.append(f"{name}: y={y} is not on a row boundary")
            if x < CORE_X0 - EPS or y < CORE_Y0 - EPS or box[2] > CORE_X1 + EPS or box[3] > CORE_Y1 + EPS:
                errs.append(f"{name}: {box} leaves the core {CORE_X0, CORE_Y0, CORE_X1, CORE_Y1}")
    boxes.sort(key=lambda b: b[1][0])
    for (n1, b1), (n2, b2) in zip(boxes, boxes[1:]):
        if b1[1] < b2[3] and b2[1] < b1[3]:              # same band
            gap = b2[0] - b1[2]
            if gap < -EPS:
                errs.append(f"{n1} and {n2} overlap by {-gap:.2f} um")
            elif gap < 2 * hx - EPS:
                errs.append(f"{n1}/{n2}: gap {gap:.2f} um < two halos {2 * hx:.2f} um")
    for ob in cfg.get("FP_OBSTRUCTIONS", []) or []:
        for name, b in boxes:
            if ob[0] < b[2] - EPS and b[0] < ob[2] - EPS and ob[1] < b[3] - EPS and b[1] < ob[3] - EPS:
                errs.append(f"obstruction {ob} covers {name}")
    for name, b in boxes:
        print(f"{name:36s} x {b[0]:8.2f}..{b[2]:8.2f}  y {b[1]:6.2f}..{b[3]:6.2f}  halo {hx}/{hy}")
    if errs:
        print("\n".join("ERROR: " + e for e in errs))
        return 1
    print(f"floorplan OK: {len(boxes)} macros")
    return 0


if __name__ == "__main__":
    sys.exit(main())
