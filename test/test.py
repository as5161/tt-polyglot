# SPDX-FileCopyrightText: © 2026 Polyglot authors
# SPDX-License-Identifier: Apache-2.0
"""Top-level protocol tests.

Everything goes through the real host SPI port exactly as the RP2350 on the
demo board would do it: programs are assembled from programs/*.pga, loaded
into instruction memory, configured and started over SPI.  Independent
protocol models (UART, SPI slave, I2C EEPROM, WS2812, Manchester, USB) check
the pins.  Runs on RTL and on the gate-level netlist (GATES=yes).
"""
import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
from polyglot import asm  # noqa: E402
from polyglot import host as H  # noqa: E402

PROG = os.path.join(HERE, "..", "programs")
HALF = 4  # host SCK half period in clk cycles (SCK = clk/8)


def prog(name, **cfg):
    p = asm.load_file(os.path.join(PROG, name + ".pga"))
    p.cfg.update(cfg)
    return p


def bit(v, n):
    return (int(v) >> n) & 1


class Chip:
    """Async host driver: bit-bangs the SPI host port."""

    def __init__(self, dut):
        self.dut = dut
        self.ui = 0x80          # ui7 = CS_n (idle high), ui6 = SCK, ui5 = MOSI
        self.next_origin = [0, 0, 0, 0]   # next free word in each core's bank
        dut.ui_in.value = self.ui

    def set_ui(self, n, v):
        self.ui = (self.ui & ~(1 << n)) | ((v & 1) << n)
        self.dut.ui_in.value = self.ui

    async def xfer(self, data):
        d = self.dut
        out = bytearray()
        self.set_ui(7, 0)
        await ClockCycles(d.clk, HALF)
        for byte in data:
            r = 0
            for i in range(7, -1, -1):
                self.set_ui(5, (byte >> i) & 1)
                self.set_ui(6, 0)
                await ClockCycles(d.clk, HALF)
                r = (r << 1) | bit(d.uo_out.value, 7)
                self.set_ui(6, 1)
                await ClockCycles(d.clk, HALF)
            out.append(r)
        self.set_ui(6, 0)
        await ClockCycles(d.clk, HALF)
        self.set_ui(7, 1)
        await ClockCycles(d.clk, HALF + 2)
        return bytes(out)

    async def write(self, addr, *words):
        await self.xfer(H.write_frame(addr, words))

    async def read(self, addr, n=1):
        r = H.parse_read(await self.xfer(H.read_frame(addr, n)), n)
        return r if n > 1 else r[0]

    async def load(self, p, core, origin=None):
        """Load p into core's own bank (the core must be stopped) and configure it."""
        o = self.next_origin[core] if origin is None else origin
        assert o + len(p.words) <= asm.IMEM_WORDS, "IMEM bank full"
        self.next_origin[core] = max(self.next_origin[core], o + len(p.words))
        await self.write(H.imem(core, o), *p.relocated(o))
        r = p.regs(o)
        await self.write(H.creg(core, 0), r[0], r[1], r[2], r[3], r[4])
        await self.write(H.R_POLY + core, p.cfg["poly"])
        await self.write(H.creg(core, H.R_PC), o)
        return o

    async def start(self, mask):
        await self.write(H.R_CTRL, (mask & 0xF) | (mask & 0xF) << 4 | (mask & 0xF) << 8)

    async def push(self, core, *vals):
        for v in vals:
            while (await self.read(H.R_FIFOSTAT)) >> core & 1:
                pass
            await self.write(H.creg(core, H.R_FIFO), v & 0xFFFF)

    async def pop(self, core):
        v = await self.read(H.creg(core, H.R_FIFO))
        return (v & 0xFFFF) if v >> 23 & 1 else None

    async def pop_n(self, core, n, timeout=200):
        """Drain n RX words using burst reads (FIFO address does not increment)."""
        out = []
        for _ in range(timeout):
            ws = await self.read(H.creg(core, H.R_FIFO), 4)
            got = [w & 0xFFFF for w in ws if w >> 23 & 1]
            out += got
            if len(out) >= n:
                break
            if not got:
                await ClockCycles(self.dut.clk, 50)
        return out[:n]


async def record(dut, n, fn, out):
    """Record fn(uo_out) for n cycles."""
    for _ in range(n):
        await RisingEdge(dut.clk)
        out.append(fn(int(dut.uo_out.value)))


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 16, unit="ns").start())   # ~60 MHz
    dut.ena.value = 1
    dut.uio_drv.value = 0xFF
    dut.od_mask.value = 0
    dut.rst_n.value = 0
    chip = Chip(dut)
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return chip


# ------------------------------------------------------------------ monitors
async def uart_rx_monitor(dut, pin, period, nbytes, out):
    """Decode 8N1 bytes from uo_out[pin]."""
    for _ in range(nbytes):
        while bit(dut.uo_out.value, pin):
            await RisingEdge(dut.clk)
        await ClockCycles(dut.clk, period // 2)
        assert bit(dut.uo_out.value, pin) == 0, "bad start bit"
        v = 0
        for i in range(8):
            await ClockCycles(dut.clk, period)
            v |= bit(dut.uo_out.value, pin) << i
        await ClockCycles(dut.clk, period)
        assert bit(dut.uo_out.value, pin) == 1, "bad stop bit"
        out.append(v)


async def drive_uart(chip, pin, period, data, stop_ok=True):
    for b in data:
        frame = [0] + [(b >> i) & 1 for i in range(8)] + [1 if stop_ok else 0]
        for v in frame:
            chip.set_ui(pin, v)
            await ClockCycles(chip.dut.clk, period)
        chip.set_ui(pin, 1)
        await ClockCycles(chip.dut.clk, period)


def crc16_modbus(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


async def last_core(chip):
    """Index of the highest core on this build (src/project.v sets NC): EN
    keeps only the bits of cores that exist.

    Every bank first gets "jmp 0" at word 0 (where a core starts after reset),
    so the briefly enabled cores spin on a known instruction.  On the
    gate-level netlist the IHP SRAM model starts as X (silicon: random), and a
    core executing X would poison its state and the read-back path."""
    spin = asm.assemble("jmp 0").words[0]
    for c in range(4):
        await chip.write(H.imem(c, 0), spin)
    await chip.write(H.R_CTRL, 0xF)
    en = await chip.read(H.R_CTRL)
    await chip.write(H.R_CTRL, 0)
    assert en in (0x1, 0x3, 0x7, 0xF), hex(en)
    return bin(en).count("1") - 1


# ------------------------------------------------------------------ tests
@cocotb.test()
async def test_host_interface(dut):
    """ID, IMEM/register read-back, EXEC injection, single-step."""
    chip = await setup(dut)
    assert await chip.read(H.R_ID) == H.CHIP_ID
    L = await last_core(chip)
    await chip.write(H.imem(L - 1, 5), 0x00BEEF)
    await chip.write(H.imem(L, 5), 0x123456, 0xABCDEF, 0x0F0F0F)
    assert await chip.read(H.imem(L, 5), 3) == [0x123456, 0xABCDEF, 0x0F0F0F]
    assert await chip.read(H.imem(L - 1, 5)) == 0x00BEEF    # banks are separate
    assert await chip.read(0x00) == 0                   # page 0 below 0x80 is unmapped
    await chip.write(H.creg(2, 3), 0x7FFFF)
    assert await chip.read(H.creg(2, 3)) == 0x7FFFF
    # EXEC: set x, 0x5A on core 1 (core disabled) -> visible in debug register
    await chip.write(H.creg(1, H.R_EXEC), asm.assemble("set x, 0x5A").words[0])
    assert await chip.read(H.R_DBG_X + 1) == 0x5A
    # single step: program "add x,1 ; add x,1 ; jmp 0" on core 0
    p = asm.assemble("add x, 1\nadd x, 2\njmp 0")
    await chip.load(p, 0, 0)
    for k, exp in enumerate([1, 3, 3, 4]):
        await chip.write(H.R_STEP, 1)
        assert await chip.read(H.R_DBG_X) == exp, f"step {k}"
    assert await chip.read(H.creg(0, H.R_PC)) == 1


@cocotb.test()
async def test_uart_tx(dut):
    chip = await setup(dut)
    await chip.load(prog("uart_tx", clkdiv=2), 0)
    await chip.start(1)
    msg = b"Hi\x00\xff\x55"
    got = []
    mon = cocotb.start_soon(uart_rx_monitor(dut, 0, 16, len(msg), got))
    await chip.push(0, *msg)
    await mon
    assert bytes(got) == msg


@cocotb.test()
async def test_uart_rx_modbus_crc(dut):
    chip = await setup(dut)
    chip.set_ui(0, 1)                     # idle line high before the core starts
    await chip.load(prog("modbus_rx", clkdiv=2), 0)
    await chip.start(1)
    await ClockCycles(dut.clk, 100)
    frame = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])
    feeder = cocotb.start_soon(drive_uart(chip, 0, 16, frame))
    got = await chip.pop_n(0, len(frame))
    await feeder
    assert bytes(got) == frame, got
    crc = await chip.read(H.R_DBG_CRC)
    assert crc == crc16_modbus(frame), f"{crc:04x} vs {crc16_modbus(frame):04x}"
    # append the CRC (LSB first): residue must become zero
    await drive_uart(chip, 0, 16, bytes([crc & 0xFF, crc >> 8]))
    await chip.pop_n(0, 2)
    assert await chip.read(H.R_DBG_CRC) == 0
    assert (await chip.read(H.R_FLAGS)) & 1 == 0, "unexpected framing error"
    # framing error detection
    await drive_uart(chip, 0, 16, b"\x42", stop_ok=False)
    await ClockCycles(dut.clk, 100)
    assert (await chip.read(H.R_FLAGS)) & 1 == 1


async def spi_slave(dut, replies, mosi_log):
    """Mode-0 SPI slave on uio0=SCK uio1=CS_n uio2=MOSI uio3=MISO."""
    idx = 0
    sh_out = replies[0]
    cnt = 0
    rx = 0
    prev_sck, prev_cs = 0, 1
    while True:
        await RisingEdge(dut.clk)
        o = int(dut.uio_out.value)
        sck, cs, mosi = o & 1, (o >> 1) & 1, (o >> 2) & 1
        if prev_cs and not cs:
            cnt = 0
        if not cs:
            if sck and not prev_sck:
                rx = (rx << 1) | mosi
                cnt += 1
                if cnt == 8:
                    mosi_log.append(rx & 0xFF)
                    cnt = 0
                    idx += 1
                    sh_out = replies[idx % len(replies)]
            if not sck and prev_sck and cnt:
                sh_out = (sh_out << 1) & 0xFF
        drv = (sh_out >> 7) & 1
        dut.uio_drv.value = 0xF7 | (drv << 3)
        prev_sck, prev_cs = sck, cs


@cocotb.test()
async def test_spi_master(dut):
    chip = await setup(dut)
    await chip.load(prog("spi_master"), 0)
    await chip.start(1)
    replies = [0xA5, 0x3C, 0x81, 0x7E]
    log = []
    cocotb.start_soon(spi_slave(dut, replies, log))
    await ClockCycles(dut.clk, 20)
    assert int(dut.uio_oe.value) & 7 == 7
    tx = [0x9F, 0x00, 0x12, 0xFE]
    await chip.push(0, *tx)
    got = await chip.pop_n(0, 4)
    assert log == tx, log
    assert got == replies, [hex(g) for g in got]
    assert bit(dut.uio_out.value, 1) == 1, "CS_n must return high when idle"


class I2CEeprom:
    """I2C slave model: 7-bit address 0x50, 256-byte memory with pointer,
    optional clock stretching after every ACK."""

    def __init__(self, dut, addr=0x50, stretch=0):
        self.dut = dut
        self.addr = addr
        self.mem = bytearray(256)
        self.ptr = 0
        self.stretch = stretch
        self.sda_low = False
        self.scl_low = False
        self.log = []

    def drive(self):
        v = 0xFF
        if self.sda_low:
            v &= ~(1 << 5)
        if self.scl_low:
            v &= ~(1 << 4)
        self.dut.uio_drv.value = v

    async def run(self):
        d = self.dut
        prev_scl = prev_sda = 1
        state = "idle"
        cnt = 0
        sh = 0
        rd = False
        first = True
        tx_byte = 0
        while True:
            await RisingEdge(d.clk)
            v = int(d.uio_in.value)
            scl, sda = (v >> 4) & 1, (v >> 5) & 1
            if prev_scl and scl and prev_sda and not sda:
                state, cnt, sh, first = "rx", 0, 0, True
                self.sda_low = False
                self.log.append("S")
            elif prev_scl and scl and not prev_sda and sda:
                state = "idle"
                self.sda_low = False
                self.log.append("P")
            elif not prev_scl and scl:  # rising edge: sample
                if state in ("rx", "ack_rx"):
                    if state == "rx":
                        sh = (sh << 1) | sda
                        cnt += 1
                elif state == "tx":
                    cnt += 1
                elif state == "ack_tx":
                    if sda:  # NACK from master
                        state = "nack"
            elif prev_scl and not scl:  # falling edge: drive next bit
                if state == "rx" and cnt == 8:
                    if first:
                        hit = (sh >> 1) == self.addr
                        rd = bool(sh & 1)
                        self.log.append(("A", sh))
                        if not hit:
                            state = "idle"
                        else:
                            self.sda_low, state = True, "ack_rx"
                    else:
                        if self.ptr_pending:
                            self.ptr = sh
                            self.ptr_pending = False
                        else:
                            self.mem[self.ptr] = sh
                            self.ptr = (self.ptr + 1) & 0xFF
                        self.log.append(("W", sh))
                        self.sda_low, state = True, "ack_rx"
                elif state == "ack_rx":
                    self.sda_low = False
                    if self.stretch:
                        self.scl_low = True
                        self.drive()
                        await ClockCycles(d.clk, self.stretch)
                        self.scl_low = False
                    if first and rd:
                        state, cnt = "tx", 0
                        tx_byte = self.mem[self.ptr]
                        self.sda_low = not (tx_byte >> 7) & 1
                    else:
                        if first:
                            self.ptr_pending = True
                        state, cnt, sh = "rx", 0, 0
                    first = False
                elif state == "tx":
                    if cnt == 8:
                        self.sda_low = False
                        state = "ack_tx"
                        self.log.append(("R", tx_byte))
                        self.ptr = (self.ptr + 1) & 0xFF
                    else:
                        self.sda_low = not (tx_byte >> (7 - cnt)) & 1
                elif state == "ack_tx":
                    tx_byte = self.mem[self.ptr]
                    state, cnt = "tx", 0
                    self.sda_low = not (tx_byte >> 7) & 1
                elif state == "nack":
                    self.sda_low = False
                    state = "idle"
            self.drive()
            prev_scl, prev_sda = scl, sda


def i2c_word(byte, start=False, stop=False, ack=1):
    return (int(start) << 15) | (int(stop) << 14) | (((byte << 1) | ack) << 5)


@cocotb.test()
async def test_i2c_master(dut):
    chip = await setup(dut)
    dut.od_mask.value = 0x30
    ee = I2CEeprom(dut, stretch=150)
    ee.ptr_pending = False
    cocotb.start_soon(ee.run())
    await chip.load(prog("i2c_master", clkdiv=2), 0)
    await chip.start(1)
    # write 0xDE 0xAD 0xBE to address 0x10
    seq = [i2c_word(0x50 << 1, start=True), i2c_word(0x10), i2c_word(0xDE), i2c_word(0xAD),
           i2c_word(0xBE, stop=True)]
    for w in seq:
        await chip.push(0, w)
    acks = await chip.pop_n(0, 5)
    assert [a & 1 for a in acks] == [0] * 5, acks
    assert ee.mem[0x10:0x13] == b"\xDE\xAD\xBE"
    # random read: set pointer, repeated START, read 3 bytes (ACK, ACK, NACK+STOP)
    seq = [i2c_word(0x50 << 1, start=True), i2c_word(0x10),
           i2c_word((0x50 << 1) | 1, start=True),
           i2c_word(0xFF, ack=0), i2c_word(0xFF, ack=0), i2c_word(0xFF, ack=1, stop=True)]
    for w in seq:
        await chip.push(0, w)
    rx = await chip.pop_n(0, 6)
    data = [r >> 1 for r in rx[3:]]
    assert data == [0xDE, 0xAD, 0xBE], [hex(x) for x in data]
    # a missing device NACKs
    await chip.push(0, i2c_word(0x23 << 1, start=True, stop=True))
    (r,) = await chip.pop_n(0, 1)
    assert r & 1 == 1, "expected NACK"
    assert ee.log.count("S") == 4 and ee.log.count("P") == 3


@cocotb.test()
async def test_ws2812(dut):
    chip = await setup(dut)
    await chip.load(prog("ws2812", clkdiv=1), 0)
    words = [0xFF00, 0x0F5A, 0x8001]
    await chip.push(0, *words)            # pre-fill: the stream must not underrun
    tr = []
    rec = cocotb.start_soon(record(dut, 12 * 48 + 1200, lambda o: (o >> 1) & 1, tr))
    await chip.start(1)
    await rec
    highs, n = [], 0
    for v in tr:
        if v:
            n += 1
        elif n:
            highs.append(n)
            n = 0
    bits = [1 if h >= 6 else 0 for h in highs]
    assert len(bits) == 48, len(bits)
    assert set(highs) == {4, 8}, set(highs)
    v = [int("".join(map(str, bits[i:i + 16])), 2) for i in range(0, 48, 16)]
    assert v == words, [hex(x) for x in v]


@cocotb.test()
async def test_manchester_10mbit(dut):
    chip = await setup(dut)
    await chip.load(prog("manchester_tx"), 0)
    words = [0x5555, 0xD555, 0xBEEF]     # preamble, SFD, payload
    await chip.push(0, *words)
    trace = []
    rec = cocotb.start_soon(record(dut, 6 * 48 + 1200, lambda o: ((o >> 2) & 1, (o >> 3) & 1), trace))
    await chip.start(1)
    await rec
    t0 = next(i for i, (p, _) in enumerate(trace) if p)   # first mid-bit edge (bit0 = 1)
    bits = [trace[t0 + 6 * k + 1][0] for k in range(48)]
    for p, n in trace[t0:t0 + 6 * 48]:
        assert p != n, "TD+/TD- not complementary"
    # every bit must have a mid-bit transition
    for k in range(48):
        assert trace[t0 + 6 * k - 1][0] != trace[t0 + 6 * k + 1][0]
    got = [sum(bits[16 * w + i] << i for i in range(16)) for w in range(3)]
    assert got == words, [hex(x) for x in got]


def usb_crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c ^ 0xFFFF


@cocotb.test()
async def test_usb_lowspeed_tx(dut):
    chip = await setup(dut)
    await chip.load(prog("usb_ls_tx"), 0)
    payload = bytes([0xFF, 0xFF])      # forces bit stuffing
    crc = usb_crc16(payload)
    pkt = bytes([0x80, 0xC3]) + payload + bytes([crc & 0xFF, crc >> 8])   # SYNC, DATA0
    words = [pkt[i] | pkt[i + 1] << 8 for i in range(0, len(pkt), 2)]
    await chip.push(0, *words)
    trace = []
    rec = cocotb.start_soon(record(dut, 40 * 75 + 1000, lambda o: ((o >> 4) & 1, (o >> 5) & 1), trace))
    await chip.start(1)
    await rec
    trace = trace[trace.index((0, 1)):]    # skip reset state before "set pins, J"
    t0 = next(i for i, s in enumerate(trace) if s != (0, 1))        # first K
    prev = (0, 1)
    bits, ones, i = [], 0, t0 + 20
    eop = None
    while i < len(trace):
        s = trace[i]
        if s == (0, 0):
            eop = i
            break
        b = 1 if s == prev else 0
        prev = s
        if ones == 6:
            assert b == 0, "missing stuffed bit"
            ones = 0
        else:
            bits.append(b)
            ones = ones + 1 if b else 0
        i += 40
    assert eop is not None, "no EOP"
    while trace[eop - 1] == (0, 0):
        eop -= 1
    se0 = 0
    while trace[eop + se0] == (0, 0):
        se0 += 1
    assert 75 <= se0 <= 85, se0
    assert trace[eop + se0] == (0, 1), "EOP must end in J"
    got = bytes(sum(bits[8 * k + j] << j for j in range(8)) for k in range(len(bits) // 8))
    assert got == pkt, got.hex()
    assert len(bits) == 8 * len(pkt)
    await ClockCycles(dut.clk, 200)
    assert (await chip.read(H.R_FLAGS)) >> 1 & 1


@cocotb.test()
async def test_three_protocols_at_once(dut):
    """UART TX, SPI master and a runtime-written pulse meter run
    concurrently; WS2812 is then loaded into core 2's bank while cores 0
    and 1 keep running (each core owns its bank, so reloading one core never
    disturbs the others)."""
    chip = await setup(dut)
    pulse = asm.assemble("""
        .program pulse_meter      ; written at test time: measures high pulses on ui1
        .in ui1
        .jmp_pin ui1
        .shift in left
        .wrap_target
            set x, 0
            wait 1 pin 0
        hi: add x, 1
            jmp pin hi
            in x, 16
            push
        .wrap
    """)
    await chip.load(prog("uart_tx", clkdiv=2), 0)
    await chip.load(prog("spi_master"), 1)
    await chip.load(pulse, 2)
    await chip.start(0x7)
    log = []
    cocotb.start_soon(spi_slave(dut, [0x11, 0x22], log))
    got = []
    mon = cocotb.start_soon(uart_rx_monitor(dut, 0, 16, 3, got))
    await chip.push(0, *b"abc")
    await chip.push(1, 0x55, 0xAA)
    for width in (40, 100):
        chip.set_ui(1, 1)
        await ClockCycles(dut.clk, width)
        chip.set_ui(1, 0)
        await ClockCycles(dut.clk, 30)
    widths = await chip.pop_n(2, 2)
    spi = await chip.pop_n(1, 2)
    await mon
    assert bytes(got) == b"abc"
    assert log == [0x55, 0xAA] and spi == [0x11, 0x22]
    assert [abs(w * 2 - t) <= 3 for w, t in zip(widths, (40, 100))] == [True, True], widths
    # re-purpose core 2 as a WS2812 driver while cores 0 and 1 stay enabled
    await chip.write(H.R_CTRL, 0x3)                     # stop core 2 only
    await chip.load(prog("ws2812", clkdiv=1), 2, origin=0)
    await chip.push(2, 0xA5A5)
    tr = []
    rec = cocotb.start_soon(record(dut, 12 * 16 + 1200, lambda o: (o >> 1) & 1, tr))
    await chip.write(H.R_CTRL, 0x7 | 0x4 << 4 | 0x4 << 8)   # restart core 2, keep 0-1 running
    await rec
    highs, n = [], 0
    for v in tr:
        if v:
            n += 1
        elif n:
            highs.append(n)
            n = 0
    bits = [1 if h >= 6 else 0 for h in highs]
    assert int("".join(map(str, bits[:16])), 2) == 0xA5A5, bits


@cocotb.test()
async def test_bank_ownership(dut):
    """A running core's bank is protected: host writes are ignored and reads
    return 0; once the core stops the bank is accessible again."""
    chip = await setup(dut)
    L = await last_core(chip)            # the highest core (3 on 4-core builds)
    p = asm.assemble("add x, 1\njmp 0")
    await chip.load(p, L, 0)
    await chip.start(1 << L)
    await chip.write(H.imem(L, 0), 0xFFFFFF)            # ignored: core L runs
    assert await chip.read(H.imem(L, 0)) == 0
    x1 = await chip.read(H.R_DBG_X + L)
    await ClockCycles(dut.clk, 50)
    assert (await chip.read(H.R_DBG_X + L) - x1) & 0xFFFF > 10   # program intact
    await chip.write(H.R_CTRL, 0)
    assert await chip.read(H.imem(L, 0), 2) == p.words
    st = await chip.read(H.R_FIFOSTAT)
    assert (st >> 7) & 1 and (st >> 15) & 1 and not (st >> 3) & 1   # slot 3 FIFOs empty (or absent)
