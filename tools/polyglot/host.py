"""Polyglot host driver.

Transport-agnostic: give it any callable `xfer(bytes) -> bytes` that performs
one full-duplex SPI transaction with CS asserted (mode 0, MSB first).  The
same code drives the cocotb testbench, the Python ISS and real silicon
(see firmware/polyglot_mp.py for the RP2350 / MicroPython version).

SPDX-License-Identifier: Apache-2.0
"""

CORE_BASE = 0x80
R_CLKDIV, R_EXECCTRL, R_SHIFTCTRL, R_PINCTRL, R_PINCTRL2, R_FIFO, R_PC, R_EXEC = range(8)
R_CTRL, R_STEP, R_FLAGS, R_FIFOSTAT, R_PINS, R_ID, R_TIME = 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6
R_POLY, R_DBG_X, R_DBG_Y, R_DBG_CRC = 0xA8, 0xAC, 0xB0, 0xB4
CHIP_ID = 0x504702
BANK_BASE = 0x400   # IMEM of core c: BANK_BASE + 0x100 * c + word


def creg(core, n):
    return CORE_BASE + 8 * core + n


def imem(core, word=0):
    return BANK_BASE + 0x100 * core + word


def write_frame(addr, words):
    b = bytearray([0x80 | (addr >> 8) & 0x7F, addr & 0xFF])
    for w in words:
        b += bytes([(w >> 16) & 0xFF, (w >> 8) & 0xFF, w & 0xFF])
    return bytes(b)


def read_frame(addr, n):
    return bytes([(addr >> 8) & 0x7F, addr & 0xFF]) + bytes(3 * n)


def parse_read(resp, n):
    out = []
    for i in range(n):
        o = 2 + 3 * i
        out.append((resp[o] << 16) | (resp[o + 1] << 8) | resp[o + 2])
    return out


class Host:
    def __init__(self, xfer):
        self.xfer = xfer

    # -- raw access
    def write(self, addr, *words):
        self.xfer(write_frame(addr, words))

    def read(self, addr, n=1):
        r = parse_read(self.xfer(read_frame(addr, n)), n)
        return r if n > 1 else r[0]

    # -- convenience
    def chip_id(self):
        return self.read(R_ID)

    def load(self, prog, core, origin=None, start=False):
        """Load an assembled Program into `core`'s bank (core stopped) and configure it."""
        o = prog.origin if origin is None else origin
        self.write(imem(core, o), *prog.relocated(o))
        regs = prog.regs(o)
        self.write(creg(core, R_CLKDIV), regs[0], regs[1], regs[2], regs[3], regs[4])
        self.write(R_POLY + core, prog.cfg["poly"])
        self.write(creg(core, R_PC), o)
        if start:
            self.enable(1 << core, restart=True)
        return o

    def enable(self, mask, restart=False, divsync=True):
        v = mask & 0xF
        if restart:
            v |= (mask & 0xF) << 4
        if divsync:
            v |= (mask & 0xF) << 8
        self.write(R_CTRL, v)

    def stop(self):
        self.write(R_CTRL, 0)

    def exec(self, core, word):
        self.write(creg(core, R_EXEC), word)

    def push(self, core, *vals):
        self.write(creg(core, R_FIFO), *[v & 0xFFFF for v in vals])

    def pop(self, core):
        """Pop one RX word.  Returns None if the FIFO was empty."""
        v = self.read(creg(core, R_FIFO))
        return (v & 0xFFFF) if v & 0x800000 else None

    def fifostat(self):
        return self.read(R_FIFOSTAT)

    def tx_full(self, core):
        return bool(self.fifostat() >> core & 1)

    def rx_empty(self, core):
        return bool(self.fifostat() >> (12 + core) & 1)
