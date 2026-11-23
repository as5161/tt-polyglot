"""Polyglot assembler (pgasm).

Source format: one program per file; see docs/isa.md and programs/*.pga.

    .program uart_tx
    .clkdiv  520                 ; core clock divider
    .side    uo0 1               ; side-set base pin, count (1..3)
    .shift   out right autopull 8
    start:
        pull          side 1     ; comments with ';' or '#'
        set x, 7      side 0 [7]
    .wrap_target
    bit:
        out pins, 1
        jmp x-- bit   [6]
    .wrap

Usage:  python -m polyglot.asm prog.pga [-o prog.json] [--origin N]

SPDX-License-Identifier: Apache-2.0
"""
import argparse
import json
import re
import sys

PINS = {}
for _i in range(8):
    PINS[f"uio{_i}"] = _i
    PINS[f"ui{_i}"] = 16 + _i
for _i in range(7):
    PINS[f"uo{_i}"] = 8 + _i

JMP_COND = {"": 0, "always": 0, "!x": 1, "x--": 2, "!y": 3, "y--": 4, "x!=y": 5,
            "pin": 6, "!osre": 7, "!pin": 8, "txne": 9, "rxnf": 10}
WAIT_SRC = {"gpio": 0, "pin": 1, "flag": 2, "match": 3}
IN_SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "isr": 4, "osr": 5, "crc": 6, "time": 7}
OUT_DST = {"pins": 0, "x": 1, "y": 2, "null": 3, "pindirs": 4, "pc": 5, "isr": 6}
MOV_DST = {"pins": 0, "x": 1, "y": 2, "crc": 3, "pindirs": 4, "pc": 5, "isr": 6, "osr": 7}
MOV_SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "crc": 4, "time": 5, "isr": 6, "osr": 7,
           "status": 8}
SET_DST = {"pins": 0, "x": 1, "y": 2, "crc": 3, "pindirs": 4, "xh": 5, "yh": 6}

OP = dict(jmp=0, wait=1, in_=2, out=3, pp=4, mov=5, flag=6, set=7, add=8, nop=15)


class AsmError(Exception):
    pass


def enc(op, a=0, side=None, delay=0):
    w = (op & 0xF) << 20
    if side is not None:
        w |= (1 << 19) | ((side & 7) << 16)
    w |= (delay & 31) << 11
    w |= a & 0x7FF
    return w


class Program:
    def __init__(self, name="prog"):
        self.name = name
        self.words = []
        self.labels = {}
        self.origin = 0
        self.wrap_bot = None
        self.wrap_top = None
        self.cfg = dict(clkdiv=0, jmp_pin=0, side_pindir=0, od=0, out_dir=0, in_dir=0,
                        autopull=0, autopush=0, pull_th=0, push_th=0, crc_src=0,
                        crc_reflect=0, out_base=0, out_cnt=0, set_base=0, set_cnt=0,
                        in_base=0, side_base=0, side_cnt=0, poly=0)

    # register images (relative wrap is resolved at load time)
    def regs(self, origin=None):
        o = self.origin if origin is None else origin
        c = self.cfg
        wb = o + (self.wrap_bot if self.wrap_bot is not None else 0)
        wt = o + (self.wrap_top if self.wrap_top is not None else len(self.words) - 1)
        return {
            0: c["clkdiv"],
            1: wb | wt << 7 | c["jmp_pin"] << 14 | c["side_pindir"] << 19 | c["od"] << 20,
            2: (c["out_dir"] | c["in_dir"] << 1 | c["autopull"] << 2 | c["autopush"] << 3 |
                c["pull_th"] << 4 | c["push_th"] << 9 | c["crc_src"] << 14 |
                c["crc_reflect"] << 16),
            3: c["out_base"] | c["out_cnt"] << 5 | c["set_base"] << 10 | c["set_cnt"] << 15,
            4: c["in_base"] | c["side_base"] << 5 | c["side_cnt"] << 10,
        }

    def relocated(self, origin):
        """Return instruction words relocated to `origin` (JMP targets fixed)."""
        out = []
        for w in self.words:
            if (w >> 20) & 0xF == 0:
                w = (w & ~0x3F) | ((w + origin) & 0x3F)
            out.append(w)
        return out

    def to_json(self):
        return dict(name=self.name, origin=self.origin, words=self.words,
                    wrap_bot=self.wrap_bot, wrap_top=self.wrap_top, cfg=self.cfg,
                    labels=self.labels)


def _num(tok, defines):
    tok = tok.strip()
    if tok in defines:
        return defines[tok]
    if tok.lower() in PINS:
        return PINS[tok.lower()]
    try:
        return int(tok, 0)
    except ValueError:
        raise AsmError(f"bad number '{tok}'")


def assemble(src, name="prog"):
    prog = Program(name)
    defines = {}
    lines = []
    # pass 1: directives, labels, collect instruction lines
    for lineno, raw in enumerate(src.splitlines(), 1):
        line = re.split(r"[;#]", raw, maxsplit=1)[0].strip()
        if not line:
            continue
        try:
            while True:
                m = re.match(r"^([A-Za-z_]\w*):\s*(.*)$", line)
                if not m:
                    break
                prog.labels[m.group(1)] = len(lines)
                line = m.group(2).strip()
            if not line:
                continue
            if line.startswith("."):
                _directive(prog, line, defines, len(lines))
                continue
            lines.append((lineno, line))
        except AsmError as e:
            raise AsmError(f"{name}:{lineno}: {e}") from None
    if prog.wrap_top is None and prog.wrap_bot is not None:
        prog.wrap_top = len(lines) - 1
    # pass 2: encode
    for lineno, line in lines:
        try:
            prog.words.append(_instr(line, prog.labels, defines))
        except AsmError as e:
            raise AsmError(f"{name}:{lineno}: {e}") from None
    if len(prog.words) > 64:
        raise AsmError(f"{name}: program too long ({len(prog.words)} > 64)")
    return prog


def _directive(prog, line, defines, pc):
    t = line.replace(",", " ").split()
    d = t[0].lower()
    c = prog.cfg
    n = lambda i: _num(t[i], defines)  # noqa: E731
    if d == ".program":
        prog.name = t[1]
    elif d == ".define":
        defines[t[1]] = _num(t[2], defines)
    elif d == ".origin":
        prog.origin = n(1)
    elif d == ".wrap_target":
        prog.wrap_bot = pc
    elif d == ".wrap":
        prog.wrap_top = pc - 1
        if prog.wrap_bot is None:
            prog.wrap_bot = 0
    elif d == ".clkdiv":
        c["clkdiv"] = n(1)
    elif d == ".out":
        c["out_base"], c["out_cnt"] = n(1), n(2)
    elif d == ".set":
        c["set_base"], c["set_cnt"] = n(1), n(2)
    elif d == ".in":
        c["in_base"] = n(1)
    elif d == ".side":
        c["side_base"], c["side_cnt"] = n(1), n(2)
        if len(t) > 3 and t[3].lower() == "pindirs":
            c["side_pindir"] = 1
    elif d == ".jmp_pin":
        c["jmp_pin"] = n(1)
    elif d == ".opendrain":
        c["od"] = 1
    elif d == ".shift":
        which = t[1].lower()
        if which not in ("in", "out"):
            raise AsmError(".shift in|out ...")
        rest = [x.lower() for x in t[2:]]
        i = 0
        while i < len(rest):
            k = rest[i]
            if k in ("right", "lsb"):
                c[f"{which}_dir"] = 0
            elif k in ("left", "msb"):
                c[f"{which}_dir"] = 1
            elif k in ("autopull", "autopush"):
                c[k] = 1
                if i + 1 < len(rest) and re.match(r"^\d+$", rest[i + 1]):
                    c["pull_th" if k == "autopull" else "push_th"] = int(rest[i + 1]) & 31
                    i += 1
            elif re.match(r"^\d+$", k):
                c["pull_th" if which == "out" else "push_th"] = int(k) & 31
            else:
                raise AsmError(f"bad .shift option '{k}'")
            i += 1
    elif d == ".crc":
        c["poly"] = n(1)
        for k in [x.lower() for x in t[2:]]:
            if k == "reflect":
                c["crc_reflect"] = 1
            elif k == "out":
                c["crc_src"] |= 1
            elif k == "in":
                c["crc_src"] |= 2
            else:
                raise AsmError(f"bad .crc option '{k}'")
    else:
        raise AsmError(f"unknown directive {d}")


def _instr(line, labels, defines):
    side = None
    delay = 0
    m = re.search(r"\[\s*([^\]]+)\]\s*$", line)
    if m:
        delay = _num(m.group(1), defines)
        if not 0 <= delay <= 31:
            raise AsmError("delay must be 0..31")
        line = line[:m.start()].strip()
    m = re.search(r"\bside\s+(\S+)\s*$", line)
    if m:
        side = _num(m.group(1), defines)
        line = line[:m.start()].strip()
    m = re.search(r"\[\s*([^\]]+)\]\s*$", line)  # allow [d] before side too
    if m:
        delay = _num(m.group(1), defines)
        line = line[:m.start()].strip()

    parts = line.split(None, 1)
    mn = parts[0].lower()
    args = [x.strip() for x in parts[1].split(",")] if len(parts) > 1 else []
    E = lambda op, a: enc(op, a, side, delay)  # noqa: E731

    def bitcount(tok):
        v = _num(tok, defines)
        if not 1 <= v <= 16:
            raise AsmError("bit count must be 1..16")
        return v & 15

    def target(tok):
        if tok in labels:
            return labels[tok]
        return _num(tok, defines)

    if mn == "nop":
        return E(OP["nop"], 0)
    if mn == "jmp":
        toks = " ".join(args).split()
        if len(toks) == 1:
            cond, tgt = "", toks[0]
        elif len(toks) == 2:
            cond, tgt = toks[0].lower(), toks[1]
        else:
            raise AsmError("jmp [cond] target")
        if cond not in JMP_COND:
            raise AsmError(f"bad jmp condition '{cond}'")
        return E(OP["jmp"], JMP_COND[cond] << 7 | (target(tgt) & 0x3F))
    if mn == "wait":
        toks = " ".join(args).split()
        pol = _num(toks[0], defines) & 1
        src = toks[1].lower()
        if src not in WAIT_SRC:
            raise AsmError(f"bad wait source '{src}'")
        idx = _num(toks[2], defines) & 31 if src != "match" else 0
        return E(OP["wait"], pol << 10 | WAIT_SRC[src] << 8 | idx)
    if mn == "in":
        src = args[0].lower()
        if src not in IN_SRC:
            raise AsmError(f"bad in source '{src}'")
        return E(OP["in_"], IN_SRC[src] << 8 | bitcount(args[1]))
    if mn == "out":
        dst = args[0].lower()
        if dst not in OUT_DST:
            raise AsmError(f"bad out destination '{dst}'")
        return E(OP["out"], OUT_DST[dst] << 8 | bitcount(args[1]))
    if mn in ("push", "pull"):
        opts = [x.lower() for x in " ".join(args).split()]
        a = (1 << 10) if mn == "pull" else 0
        block = 1
        for o in opts:
            if o in ("iffull", "ifempty"):
                a |= 1 << 9
            elif o == "block":
                block = 1
            elif o == "noblock":
                block = 0
            else:
                raise AsmError(f"bad {mn} option '{o}'")
        return E(OP["pp"], a | block << 8)
    if mn == "mov":
        dst = args[0].lower()
        src = args[1].replace(" ", "").lower()
        mop = 0
        if src.startswith(("!", "~")):
            mop, src = 1, src[1:]
        elif src.startswith("::"):
            mop, src = 2, src[2:]
        elif src.startswith("><"):
            mop, src = 3, src[2:]
        if dst not in MOV_DST or src not in MOV_SRC:
            raise AsmError(f"bad mov '{line}'")
        return E(OP["mov"], MOV_DST[dst] << 8 | mop << 6 | MOV_SRC[src])
    if mn == "flag":
        toks = " ".join(args).split()
        mode = {"set": 0, "clr": 1, "clear": 1}.get(toks[0].lower())
        if mode is None:
            raise AsmError("flag set|clr n")
        return E(OP["flag"], mode << 9 | (_num(toks[1], defines) & 7))
    if mn == "set":
        dst = args[0].lower()
        if dst not in SET_DST:
            raise AsmError(f"bad set destination '{dst}'")
        v = _num(args[1], defines)
        if not 0 <= v <= 255:
            raise AsmError("set immediate must be 0..255")
        return E(OP["set"], SET_DST[dst] << 8 | v)
    if mn == "add":
        dst = args[0].lower()
        if dst not in ("x", "y"):
            raise AsmError("add x|y, imm")
        v = _num(args[1], defines)
        if not -128 <= v <= 127:
            raise AsmError("add immediate must be -128..127")
        return E(OP["add"], (1 << 10 if dst == "y" else 0) | (v & 0xFF))
    raise AsmError(f"unknown mnemonic '{mn}'")


def disassemble(w):
    op = (w >> 20) & 15
    a = w & 0x7FF
    f3 = (a >> 8) & 7
    nb = (a & 15) or 16
    inv = lambda d: {v: k for k, v in d.items() if k not in ("always",)}  # noqa: E731
    if op == 0:
        s = f"jmp {inv(JMP_COND).get((a >> 7) & 15, '?')} {a & 63}".replace("jmp  ", "jmp ")
    elif op == 1:
        s = f"wait {a >> 10 & 1} {inv(WAIT_SRC)[(a >> 8) & 3]} {a & 31}"
    elif op == 2:
        s = f"in {inv(IN_SRC)[f3]}, {nb}"
    elif op == 3:
        s = f"out {inv(OUT_DST).get(f3, 'null')}, {nb}"
    elif op == 4:
        s = ("pull" if a >> 10 & 1 else "push") + (" if" if a >> 9 & 1 else "") + \
            (" block" if a >> 8 & 1 else " noblock")
    elif op == 5:
        s = f"mov {inv(MOV_DST)[f3]}, {['', '!', '::', '><'][(a >> 6) & 3]}{inv(MOV_SRC).get(a & 15, 'null')}"
    elif op == 6:
        s = f"flag {['set', 'clr', 'rsv', 'rsv'][(a >> 9) & 3]} {a & 7}"
    elif op == 7:
        s = f"set {inv(SET_DST).get(f3, 'rsv')}, {a & 255}"
    elif op == 8:
        s = f"add {'y' if a >> 10 & 1 else 'x'}, {(a & 255) - 256 if a & 128 else a & 255}"
    else:
        s = "nop"
    if w >> 19 & 1:
        s += f" side {(w >> 16) & 7}"
    if (w >> 11) & 31:
        s += f" [{(w >> 11) & 31}]"
    return s


def load_file(path):
    with open(path) as f:
        return assemble(f.read(), name=path)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Polyglot assembler")
    ap.add_argument("src")
    ap.add_argument("-o", "--output")
    ap.add_argument("--origin", type=int)
    ap.add_argument("--hex", action="store_true", help="print hex words")
    ns = ap.parse_args(argv)
    try:
        p = load_file(ns.src)
    except AsmError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    if ns.origin is not None:
        p.origin = ns.origin
    if ns.hex or not ns.output:
        for i, w in enumerate(p.relocated(p.origin)):
            print(f"{p.origin + i:02x}: {w:06x}   {disassemble(w)}")
    if ns.output:
        with open(ns.output, "w") as f:
            json.dump(p.to_json(), f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
