"""Polyglot host driver for the Tiny Tapeout demo board (RP2350, MicroPython).

Untested on silicon (none exists yet); written against the tt-micropython-firmware
SDK. Assemble programs on a PC first:

    python -m polyglot.asm programs/uart_tx.pga -o uart_tx.json

copy the .json to the board, then:

    import polyglot_mp as pg
    chip = pg.Polyglot.from_demoboard(clock_hz=50_000_000)
    chip.load_json("uart_tx.json", core=0)
    chip.start(0b0001)
    chip.push(0, *b"Hello")

SPI is bit-banged on the project's ui[7:5] / uo[7] pins through the SDK so it
works whatever GPIO mapping the board revision uses. The chip accepts
SCK <= clk/8, so even a slow loop is fine.

SPDX-License-Identifier: Apache-2.0
"""
import json

R_CTRL, R_STEP, R_FLAGS, R_FIFOSTAT, R_PINS, R_ID = 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5
R_POLY = 0xA8
CHIP_ID = 0x504702
IMEM_WORDS = 128          # words per core bank
BANK_BASE = 0x400         # IMEM of core c: BANK_BASE + 0x100 * c


def creg(core, n):
    return 0x80 + 8 * core + n


class Polyglot:
    def __init__(self, set_cs, set_sck, set_mosi, get_miso):
        self._cs, self._sck, self._mosi, self._miso = set_cs, set_sck, set_mosi, get_miso
        self._cs(1)
        self._sck(0)
        self.next_origin = [0, 0, 0, 0]   # per core bank

    @classmethod
    def from_demoboard(cls, clock_hz=50_000_000, project="tt_um_polyglot"):
        from ttboard.demoboard import DemoBoard  # tt-micropython-firmware
        tt = DemoBoard.get()
        getattr(tt.shuttle, project).enable()
        tt.clock_project_PWM(clock_hz)
        tt.reset_project(True)
        tt.reset_project(False)
        ui, uo = tt.ui_in, tt.uo_out
        # SDK ports support per-bit indexing: tt.ui_in[7] = 1, tt.uo_out[7]
        return cls(lambda v: ui.__setitem__(7, v),
                   lambda v: ui.__setitem__(6, v),
                   lambda v: ui.__setitem__(5, v),
                   lambda: uo[7])

    # ---------------------------------------------------------------- SPI
    def _xfer(self, data):
        out = bytearray()
        self._cs(0)
        for b in data:
            r = 0
            for i in range(7, -1, -1):
                self._mosi((b >> i) & 1)
                self._sck(0)
                r = (r << 1) | (self._miso() & 1)
                self._sck(1)
            out.append(r)
        self._sck(0)
        self._cs(1)
        return out

    def write(self, addr, *words):
        b = bytearray([0x80 | (addr >> 8) & 0x7F, addr & 0xFF])
        for w in words:
            b += bytes([(w >> 16) & 255, (w >> 8) & 255, w & 255])
        self._xfer(b)

    def read(self, addr, n=1):
        r = self._xfer(bytes([(addr >> 8) & 0x7F, addr & 0xFF]) + bytes(3 * n))
        vals = [(r[2 + 3 * i] << 16) | (r[3 + 3 * i] << 8) | r[4 + 3 * i] for i in range(n)]
        return vals if n > 1 else vals[0]

    # ---------------------------------------------------------------- API
    def check(self):
        i = self.read(R_ID)
        if i != CHIP_ID:
            raise OSError("Polyglot not responding (ID=0x%06x)" % i)

    def load(self, words, regs, poly, core, origin=None):
        """words: relocatable program words; regs: list of 5 config words
        computed for origin 0 (wrap fields are re-based here).  The program
        goes into `core`'s own bank; the core must be stopped."""
        o = self.next_origin[core] if origin is None else origin
        if o + len(words) > IMEM_WORDS:
            raise ValueError("IMEM full")
        reloc = [(w & ~0x7F) | ((w + o) & 0x7F) if (w >> 20) & 15 == 0 else w for w in words]
        self.write(BANK_BASE + 0x100 * core + o, *reloc)
        wb, wt = regs[1] & 0x7F, (regs[1] >> 7) & 0x7F
        regs = list(regs)
        regs[1] = (regs[1] & ~0x3FFF) | (wb + o) | ((wt + o) << 7)
        self.write(creg(core, 0), *regs)
        self.write(R_POLY + core, poly)
        self.write(creg(core, 6), o)
        self.next_origin[core] = max(self.next_origin[core], o + len(words))
        return o

    def load_json(self, path, core, origin=None):
        with open(path) as f:
            p = json.load(f)
        c = p["cfg"]
        n = len(p["words"])
        wb = p["wrap_bot"] if p["wrap_bot"] is not None else 0
        wt = p["wrap_top"] if p["wrap_top"] is not None else n - 1
        regs = [c["clkdiv"],
                wb | wt << 7 | c["jmp_pin"] << 14 | c["side_pindir"] << 19 | c["od"] << 20,
                c["out_dir"] | c["in_dir"] << 1 | c["autopull"] << 2 | c["autopush"] << 3 |
                c["pull_th"] << 4 | c["push_th"] << 9 | c["crc_src"] << 14 | c["crc_reflect"] << 16,
                c["out_base"] | c["out_cnt"] << 5 | c["set_base"] << 10 | c["set_cnt"] << 15,
                c["in_base"] | c["side_base"] << 5 | c["side_cnt"] << 10]
        return self.load(p["words"], regs, c["poly"], core, origin)

    def start(self, mask):
        self.write(R_CTRL, (mask & 15) | (mask & 15) << 4 | (mask & 15) << 8)

    def stop(self):
        self.write(R_CTRL, 0)

    def push(self, core, *vals):
        for v in vals:
            while self.read(R_FIFOSTAT) >> core & 1:
                pass
            self.write(creg(core, 5), v & 0xFFFF)

    def pop(self, core, n=4):
        """Drain up to n words in one burst."""
        ws = self.read(creg(core, 5), n)
        ws = ws if isinstance(ws, list) else [ws]
        return [w & 0xFFFF for w in ws if w >> 23 & 1]

    def exec(self, core, word):
        self.write(creg(core, 7), word)

    def step(self, mask):
        self.write(R_STEP, mask)

    def flags(self):
        return self.read(R_FLAGS)
