"""1-Wire master (programs/onewire.pga) against a 1-Wire device model.

Runs the cycle-accurate Polyglot model (tools/polyglot/sim.py) with the bus
on uio6 as a wired-AND: low if the master drives it (open-drain) or the
device pulls it.  The device model follows the Maxim/Analog timing:

  reset   master low >= 480 us -> device waits 30 us, pulls low 120 us
  write   device samples 30 us after the master's falling edge
  read    if the device's next bit is 0 it holds the bus low 30 us from the
          master's falling edge, else it leaves it released

Checks: presence detected, presence absent with no device, bytes written
arrive LSB first, bytes read come back in the RX FIFO.

    python3 test/proto/test_onewire.py

SPDX-License-Identifier: Apache-2.0
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools"))

from polyglot.asm import load_file  # noqa: E402
from polyglot.run import Bus, load  # noqa: E402
from polyglot.sim import Cluster  # noqa: E402

PIN = 6
US = 60  # cycles per microsecond at clkdiv 60 (1 tick = 1 us)


class Device:
    """A 1-Wire slave, sampled once per clock cycle."""

    def __init__(self, present=True, reply=()):
        self.present = present
        self.reply_bits = []
        for b in reply:
            self.reply_bits += [(b >> i) & 1 for i in range(8)]
        self.got_bits = []
        self.resets = 0
        self.prev = 1
        self.low_since = None
        self.fall = None
        self.pull_until = -1
        self.pull_from = -1
        self.sample_at = None
        self.read_bit = None
        self.pending = None
        self.t = 0

    def pull(self):
        """Is the device holding the bus low this cycle?"""
        return self.pull_from <= self.t < self.pull_until

    def step(self, bus_level):
        t = self.t
        if self.prev == 1 and bus_level == 0 and not self.pull():   # master's falling edge
            self.low_since = t
            self.sample_at = t + 30 * US
            self.pending = None
            self.read_bit = self.reply_bits[0] if self.reply_bits else None
            if self.read_bit == 0:
                self.pull_from, self.pull_until = t, t + 30 * US
        if self.sample_at is not None and t == self.sample_at:
            self.sample_at = None
            if bus_level == 1:            # released before 30 us: a 1 slot, done
                self.commit(1)
            else:                         # still low: a 0 slot, or a reset (decided at release)
                self.pending = 0
        if self.prev == 0 and bus_level == 1 and self.low_since is not None:   # release
            low = t - self.low_since
            if low >= 480 * US:                        # bus reset
                self.resets += 1
                self.pull_from, self.pull_until = ((t + 30 * US, t + 150 * US) if self.present
                                                   else (-1, -1))
            elif low < 120 * US and self.pending is not None:
                self.commit(self.pending)
            self.low_since = None
            self.pending = None
        self.prev = bus_level
        self.t += 1

    def commit(self, bit):
        if self.read_bit is not None:
            self.reply_bits.pop(0)
        self.got_bits.append(bit)

    def got_bytes(self):
        b = self.got_bits
        return [sum(b[i + k] << k for k in range(8)) for i in range(0, len(b) - 7, 8)]


def run(words, device, cycles):
    m = Cluster(aw=8)
    bus = Bus(m)
    prog = load_file(os.path.join(HERE, "..", "..", "programs", "onewire.pga"))
    load(bus, prog, 0, 0)
    bus.cyc(1, 0, 0xA0, 1 | 1 << 4 | 1 << 8)
    q = list(words)
    rx = []
    level = 1
    for _ in range(cycles):
        master_low = (m.poe >> PIN) & 1 and not (m.pout >> PIN) & 1
        level = 0 if (master_low or device.pull()) else 1
        device.step(level)
        bus.pins = level << PIN
        if q and not m.cores[0].tx.full:
            bus.cyc(1, 0, 0x80 + 5, q.pop(0))
        elif not m.cores[0].rx.empty:
            rx.append(bus.cyc(0, 1, 0x80 + 5) & 0xFFFF)
        else:
            bus.cyc()
    return rx


def rx_byte(w):
    """RX words come back right-justified (bit 23 of a raw read is the FIFO-valid flag)."""
    return w & 0xFF


def main():
    fails = 0

    def check(name, ok, detail=""):
        nonlocal fails
        print(("PASS " if ok else "FAIL ") + name + (f"  ({detail})" if detail else ""))
        fails += 0 if ok else 1

    reset_cyc = 1100 * US
    byte_cyc = 8 * 70 * US

    # 1. reset with a device present: presence bit 0
    d = Device(present=True)
    rx = run([1], d, reset_cyc)
    check("reset sees presence", d.resets == 1 and len(rx) == 1 and rx[0] & 1 == 0,
          f"resets={d.resets} rx={[hex(x) for x in rx]}")

    # 2. reset with no device: presence bit 1
    d = Device(present=False)
    rx = run([1], d, reset_cyc)
    check("reset without device", len(rx) == 1 and rx[0] & 1 == 1, f"rx={[hex(x) for x in rx]}")

    # 3. write 0xCC (Skip ROM) then 0x44 (Convert T): device receives them
    d = Device(present=True)
    rx = run([1, 0xCC << 1, 0x44 << 1], d, reset_cyc + 2 * byte_cyc + 200 * US)
    check("write 0xCC 0x44", d.got_bytes() == [0xCC, 0x44], f"got={[hex(x) for x in d.got_bytes()]}")

    # 4. read: device answers 0xA5 0x3C to two 0xFF read bytes
    d = Device(present=True, reply=[0xA5, 0x3C])
    rx = run([1, 0xFF << 1, 0xFF << 1], d, reset_cyc + 2 * byte_cyc + 200 * US)
    check("read 0xA5 0x3C", len(rx) == 3, f"rx={[hex(x) for x in rx]}")
    if len(rx) == 3:
        got = [rx_byte(rx[1]), rx_byte(rx[2])]
        check("read data", got == [0xA5, 0x3C], f"got={[hex(x) for x in got]}")

    print("onewire:", "all passed" if not fails else f"{fails} failed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
