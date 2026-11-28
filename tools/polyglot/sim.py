"""Polyglot cycle-accurate instruction-set simulator (golden model).

This model mirrors src/pg_cluster.v + src/pg_core.v at clock-cycle
granularity.  test/cosim drives identical stimulus into the RTL and this
model and compares the full architectural state every cycle.

SPDX-License-Identifier: Apache-2.0
"""

M16 = 0xFFFF
M32 = 0xFFFFFFFF

OP_JMP, OP_WAIT, OP_IN, OP_OUT, OP_PP, OP_MOV, OP_FLAG, OP_SET, OP_ADD = range(9)


def rotl32(v, s):
    s &= 31
    v &= M32
    return ((v << s) | (v >> (32 - s))) & M32 if s else v


def rotr32(v, s):
    s &= 31
    v &= M32
    return ((v >> s) | (v << (32 - s))) & M32 if s else v


def mask16(n):
    return M16 if n >= 16 else (1 << n) - 1


def rev16(v):
    return int(f"{v & M16:016b}"[::-1], 2)


def crc_step(c, b, refl, poly):
    if refl:
        fb = (c & 1) ^ b
        return ((c >> 1) ^ (poly if fb else 0)) & M16
    fb = ((c >> 15) & 1) ^ b
    return ((c << 1) ^ (poly if fb else 0)) & M16


class Fifo:
    """Depth-4 circular FIFO, bit-exact with pg_fifo.v (including stale data)."""

    def __init__(self):
        self.mem = [None] * 4
        self.rp = 0
        self.wp = 0
        self.cnt = 0

    @property
    def empty(self):
        return self.cnt == 0

    @property
    def full(self):
        return self.cnt == 4

    @property
    def head(self):
        return self.mem[self.rp]

    def update(self, clr, push, wdata, pop):
        do_push = push and not self.full
        do_pop = pop and not self.empty
        if do_push:
            self.mem[self.wp] = wdata & M16
        if clr:
            self.rp = self.wp = self.cnt = 0
            return
        if do_push:
            self.wp = (self.wp + 1) & 3
        if do_pop:
            self.rp = (self.rp + 1) & 3
        self.cnt += int(do_push) - int(do_pop)


class CoreCfg:
    def __init__(self, aw):
        self.clkdiv = 0
        self.wrap_bot = 0
        self.wrap_top = (1 << min(aw, 7)) - 1   # 7-bit field (in-page when aw = 8)
        self.jmp_pin = 0
        self.side_pindir = 0
        self.od = 0
        self.out_dir = 0
        self.in_dir = 0
        self.autopull = 0
        self.autopush = 0
        self.pull_th = 0
        self.push_th = 0
        self.crc_src = 0
        self.crc_reflect = 0
        self.out_base = 0
        self.out_cnt = 0
        self.set_base = 0
        self.set_cnt = 0
        self.in_base = 0
        self.side_base = 0
        self.side_cnt = 0
        self.poly = 0


class Core:
    def __init__(self, aw=7):
        self.aw = aw
        # JMP and wrap fields are 7 bits: with 256-word banks (aw = 8) they
        # address the current 128-word page (MOV/OUT PC reach any word)
        self.page_mask = 0x7F if aw > 7 else (1 << aw) - 1
        self.cfg = CoreCfg(aw)
        self.pc = 0
        self.x = self.y = self.isr = self.osr = self.crc = 0
        self.isr_cnt = 0
        self.osr_cnt = 16
        self.dly = 0
        self.divcnt = 0
        self.exec_pend = 0
        self.exec_ir = 0
        self.tx = Fifo()
        self.rx = Fifo()
        self.cov = None   # optional functional-coverage dict

    # ------------------------------------------------------------------
    def evaluate(self, tick, imem_word, pins, flags, time):
        """Combinational evaluation.  Returns a dict of effects; does not
        modify state."""
        c = self.cfg
        aw_mask = (1 << self.aw) - 1
        ev = self.exec_pend
        ir = self.exec_ir if ev else imem_word
        op = (ir >> 20) & 0xF
        side_en = (ir >> 19) & 1
        side_v = (ir >> 16) & 7
        idly = (ir >> 11) & 31
        a = ir & 0x7FF
        f3 = (a >> 8) & 7
        nbits = (a & 15) or 16
        go = bool(ev or (tick and self.dly == 0))
        pull_th = c.pull_th or 16
        push_th = c.push_th or 16
        rin = rotr32(pins, c.in_base)
        osre = self.osr_cnt >= pull_th
        tx_level, rx_level = self.tx.cnt, self.rx.cnt
        status = (flags << 8) | (rx_level << 3) | tx_level
        x, y, isr, osr, crc = self.x, self.y, self.isr, self.osr, self.crc

        st = dict(stall=False, jump=False, jt=0, x=x, y=y, isr=isr, osr=osr, crc=crc,
                  isr_cnt=self.isr_cnt, osr_cnt=self.osr_cnt, tx_pop=False,
                  rx_push=False, rx_data=0, fset=0, fclr=0)
        mw = None  # (dir, base, cnt, data)
        crc_feed = False
        crc_bit = 0

        def justify(v, cnt):
            return v if c.in_dir else (v >> (16 - cnt)) & M16

        if op == OP_JMP:
            cond_sel = (a >> 7) & 15
            cond = False
            if cond_sel == 0:
                cond = True
            elif cond_sel == 1:
                cond = x == 0
            elif cond_sel == 2:
                cond = x != 0
                st["x"] = (x - 1) & M16
            elif cond_sel == 3:
                cond = y == 0
            elif cond_sel == 4:
                cond = y != 0
                st["y"] = (y - 1) & M16
            elif cond_sel == 5:
                cond = x != y
            elif cond_sel == 6:
                cond = bool((pins >> c.jmp_pin) & 1)
            elif cond_sel == 7:
                cond = not osre
            elif cond_sel == 8:
                cond = not ((pins >> c.jmp_pin) & 1)
            elif cond_sel == 9:
                cond = not self.tx.empty
            elif cond_sel == 10:
                cond = not self.rx.full
            st["jump"] = cond
            pm = self.page_mask
            st["jt"] = (self.pc & ~pm & aw_mask) | (a & pm)   # in-page target (128-word pages when aw = 8)

        elif op == OP_WAIT:
            pol = (a >> 10) & 1
            src = (a >> 8) & 3
            idx = a & 31
            if src == 0:
                sat = ((pins >> idx) & 1) == pol
            elif src == 1:
                sat = ((rin >> idx) & 1) == pol
            elif src == 2:
                sat = ((flags >> (idx & 7)) & 1) == pol
                if sat and pol:
                    st["fclr"] = 1 << (idx & 7)
            else:
                sat = ((rin & M16 & y) == x) == bool(pol)
            st["stall"] = not sat

        elif op == OP_IN:
            srcv = [rin & M16, x, y, 0, isr, osr, crc, time & M16][f3]
            idata = srcv & mask16(nbits)
            if c.in_dir:
                isr_sh = ((isr << nbits) | idata) & M16
            else:
                isr_sh = ((isr >> nbits) | (idata << (16 - nbits))) & M16
            cnt_sum = min(16, self.isr_cnt + nbits)
            if c.crc_src & 2 and nbits == 1:
                crc_feed, crc_bit = True, idata & 1
            if c.autopush and cnt_sum >= push_th:
                if self.rx.full:
                    st["stall"] = True
                else:
                    st["rx_push"] = True
                    st["rx_data"] = justify(isr_sh, cnt_sum)
                    st["isr"] = 0
                    st["isr_cnt"] = 0
            else:
                st["isr"] = isr_sh
                st["isr_cnt"] = cnt_sum

        elif op == OP_OUT:
            osr_src, osr_csrc = osr, self.osr_cnt
            if c.autopull and osre:
                if self.tx.empty:
                    st["stall"] = True
                else:
                    st["tx_pop"] = True
                    osr_src, osr_csrc = self.tx.head, 0
            if c.out_dir:
                odata = (osr_src >> (16 - nbits)) & M16
                st["osr"] = (osr_src << nbits) & M16
            else:
                odata = osr_src & mask16(nbits)
                st["osr"] = (osr_src >> nbits) & M16
            st["osr_cnt"] = min(16, osr_csrc + nbits)
            if c.crc_src & 1 and nbits == 1:
                crc_feed, crc_bit = True, odata & 1
            if f3 == 0:
                mw = (0, c.out_base, c.out_cnt, odata)
            elif f3 == 1:
                st["x"] = odata
            elif f3 == 2:
                st["y"] = odata
            elif f3 == 4:
                mw = (1, c.out_base, c.out_cnt, odata)
            elif f3 == 5:
                st["jump"], st["jt"] = True, odata & aw_mask
            elif f3 == 6:
                st["isr"], st["isr_cnt"] = odata, nbits

        elif op == OP_PP:
            if (a >> 10) & 1:  # PULL
                if not (((a >> 9) & 1) and not osre):
                    if self.tx.empty:
                        if (a >> 8) & 1:
                            st["stall"] = True
                        else:
                            st["osr"], st["osr_cnt"] = x, 0
                    else:
                        st["tx_pop"] = True
                        st["osr"], st["osr_cnt"] = self.tx.head, 0
            else:  # PUSH
                if not (((a >> 9) & 1) and self.isr_cnt < push_th):
                    if self.rx.full and (a >> 8) & 1:
                        st["stall"] = True
                    else:
                        st["rx_push"] = not self.rx.full
                        st["rx_data"] = justify(isr, self.isr_cnt)
                        st["isr"], st["isr_cnt"] = 0, 0

        elif op == OP_MOV:
            s4 = a & 15
            srcv = [rin & M16, x, y, 0, crc, time & M16, isr, osr, status][s4] if s4 <= 8 else 0
            mop = (a >> 6) & 3
            if mop == 0:
                v = srcv
            elif mop == 1:
                v = ~srcv & M16
            elif mop == 2:
                v = rev16(srcv)
            else:
                v = ((srcv & 0xFF) << 8) | ((srcv >> 8) & 0xFF)
            if f3 == 0:
                mw = (0, c.out_base, c.out_cnt, v)
            elif f3 == 1:
                st["x"] = v
            elif f3 == 2:
                st["y"] = v
            elif f3 == 3:
                st["crc"] = v
            elif f3 == 4:
                mw = (1, c.out_base, c.out_cnt, v)
            elif f3 == 5:
                st["jump"], st["jt"] = True, v & aw_mask
            elif f3 == 6:
                st["isr"], st["isr_cnt"] = v, 0
            else:
                st["osr"], st["osr_cnt"] = v, 0

        elif op == OP_FLAG:
            mode = (a >> 9) & 3
            if mode == 0:
                st["fset"] = 1 << (a & 7)
            elif mode == 1:
                st["fclr"] = 1 << (a & 7)

        elif op == OP_SET:
            imm = a & 0xFF
            if f3 == 0:
                mw = (0, c.set_base, c.set_cnt, imm)
            elif f3 == 1:
                st["x"] = imm
            elif f3 == 2:
                st["y"] = imm
            elif f3 == 3:
                st["crc"] = imm
            elif f3 == 4:
                mw = (1, c.set_base, c.set_cnt, imm)
            elif f3 == 5:
                st["x"] = (imm << 8) | (x & 0xFF)
            elif f3 == 6:
                st["y"] = (imm << 8) | (y & 0xFF)

        elif op == OP_ADD:
            simm = (a & 0xFF) - 256 if a & 0x80 else a & 0xFF
            if (a >> 10) & 1:
                st["y"] = (y + simm) & M16
            else:
                st["x"] = (x + simm) & M16

        if crc_feed:
            st["crc"] = crc_step(crc, crc_bit, c.crc_reflect, c.poly)

        active = go and not st["stall"]
        if not active:
            mw = None
            st.update(jump=False, tx_pop=False, rx_push=False, fset=0, fclr=0)

        # ---------------- pin writes
        pvm = pvv = pdm = pdv = 0
        m_vm = m_vv = m_dm = m_dv = 0
        if mw is not None:
            isdir, base, cnt, data = mw
            m16 = mask16(min(cnt, 16))
            mask = rotl32(m16, base)
            val = rotl32(data & m16, base)
            if not isdir:
                m_vm = mask
            if isdir or c.od:
                m_dm = mask
            m_dv = val if isdir else (~val & M32)
            m_vv = 0 if c.od else val
        s_vm = s_vv = s_dm = s_dv = 0
        if go and side_en and c.side_cnt:
            m16 = mask16(c.side_cnt)
            mask = rotl32(m16, c.side_base)
            val = rotl32(side_v & m16, c.side_base)
            if not c.side_pindir:
                s_vm = mask
            if c.side_pindir or c.od:
                s_dm = mask
            s_dv = val if c.side_pindir else (~val & M32)
            s_vv = 0 if c.od else val
        pvm = m_vm | s_vm
        pvv = (s_vv & s_vm) | (m_vv & m_vm & ~s_vm & M32)
        pdm = m_dm | s_dm
        pdv = (s_dv & s_dm) | (m_dv & m_dm & ~s_dm & M32)

        if active and self.cov is not None:
            key = (op, (a >> 7) & 15 if op == 0 else (a >> 8) & 3 if op == 1 else f3 if op in (2, 3, 5, 7) else (a >> 8) & 7 if op == 4 else 0)
            self.cov[key] = self.cov.get(key, 0) + 1
            if st["stall"]:
                pass
        if go and st["stall"] and self.cov is not None:
            self.cov[("stall", op)] = self.cov.get(("stall", op), 0) + 1
        st.update(go=go, active=active, ev=ev, idly=idly, pvm=pvm, pvv=pvv, pdm=pdm, pdv=pdv,
                  exec_done=bool(ev and not st["stall"]), tick=tick)
        return st

    def commit(self, st, pc_we, pc_wdata, restart):
        if st["active"]:
            self.x, self.y = st["x"], st["y"]
            self.isr, self.osr, self.crc = st["isr"], st["osr"], st["crc"]
            self.isr_cnt, self.osr_cnt = st["isr_cnt"], st["osr_cnt"]
            self.dly = st["idly"]
            if st["jump"]:
                self.pc = st["jt"]
            elif not st["ev"]:
                pm = self.page_mask
                if (self.pc & pm) == (self.cfg.wrap_top & pm):
                    self.pc = (self.pc & ~pm & ((1 << self.aw) - 1)) | (self.cfg.wrap_bot & pm)
                else:
                    self.pc = (self.pc + 1) & ((1 << self.aw) - 1)
        elif st["tick"] and not st["ev"] and self.dly != 0:
            self.dly -= 1
        if pc_we:
            self.pc = pc_wdata & ((1 << self.aw) - 1)
            self.dly = 0
        if restart:
            self.isr = self.osr = 0
            self.isr_cnt = 0
            self.osr_cnt = 16
            self.dly = 0


class Cluster:
    """Bit/cycle-exact model of pg_cluster."""

    ID = 0x504702
    BANK_BASE = 0x400          # IMEM of core c: BANK_BASE + 0x100 * c + word

    def __init__(self, aw=8, nc=4):
        self.aw = aw
        self.nc = nc
        # one 256-word bank per core slot; dout = the bank's output register,
        # which is the core's fetch register (it holds bank[pc])
        self.banks = [[0] * 256 for _ in range(4)]
        self.dout = [0] * 4
        self.cores = [Core(aw) for _ in range(nc)]
        self.en = 0
        self.flags = 0
        self.timer = 0
        self.pout = 0
        self.poe = 0

    def imem_slot(self, addr):
        """(bank, word) if addr is an IMEM word, else None."""
        if (addr >> 10) == 1 and (addr & 0xFF) < (1 << self.aw):
            return (addr >> 8) & 3, addr & 0xFF
        return None

    # ------------------------------------------------------------------ host read
    def read(self, addr, pins):
        im = self.imem_slot(addr)
        if im is not None:
            b = im[0]
            return self.dout[b] if b < self.nc and not (self.en >> b) & 1 else 0
        if addr >> 8:
            return 0
        if addr < 0x80:
            return 0
        if 0x80 <= addr < 0xA0:
            if (addr >> 3) & 3 >= self.nc:
                return 0          # empty core slot
            core = self.cores[(addr >> 3) & 3]
            c = core.cfg
            n = addr & 7
            if n == 0:
                return c.clkdiv
            if n == 1:
                return c.wrap_bot | (c.wrap_top << 7) | (c.jmp_pin << 14) | (c.side_pindir << 19) | (c.od << 20)
            if n == 2:
                return (c.out_dir | c.in_dir << 1 | c.autopull << 2 | c.autopush << 3 | c.pull_th << 4 |
                        c.push_th << 9 | c.crc_src << 14 | c.crc_reflect << 16)
            if n == 3:
                return c.out_base | c.out_cnt << 5 | c.set_base << 10 | c.set_cnt << 15
            if n == 4:
                return c.in_base | c.side_base << 5 | c.side_cnt << 10
            if n == 5:
                return 0 if core.rx.empty else (1 << 23) | core.rx.head
            if n == 6:
                return core.pc
            return (core.exec_pend << 7) | (core.rx.cnt << 3) | core.tx.cnt
        if addr == 0xA0:
            return self.en
        if addr == 0xA2:
            return self.flags
        if addr == 0xA3:
            v = 0
            for i in range(self.nc, 4):                     # empty slots read "empty"
                v |= 1 << (4 + i) | 1 << (12 + i)
            for i, core in enumerate(self.cores):
                v |= core.tx.full << i
                v |= core.tx.empty << (4 + i)
                v |= core.rx.full << (8 + i)
                v |= core.rx.empty << (12 + i)
            return v
        if addr == 0xA4:
            return pins & 0xFFFFFF
        if addr == 0xA5:
            return self.ID
        if addr == 0xA6:
            return self.timer
        if 0xA8 <= addr < 0xAC:
            return self.cores[addr & 3].cfg.poly if (addr & 3) < self.nc else 0
        if 0xAC <= addr < 0xB0:
            return self.cores[addr & 3].x if (addr & 3) < self.nc else 0
        if 0xB0 <= addr < 0xB4:
            return self.cores[addr & 3].y if (addr & 3) < self.nc else 0
        if 0xB4 <= addr < 0xB8:
            return self.cores[addr & 3].crc if (addr & 3) < self.nc else 0
        return 0

    # ------------------------------------------------------------------ one clock
    def cycle(self, hw_en=0, hr_en=0, haddr=0, hwdata=0, pins=0, hr_peek=0):
        rdata = self.read(haddr, pins)
        en_now = self.en
        w = lambda a: hw_en and haddr == a  # noqa: E731
        w_ctrl, w_step, w_flags = w(0xA0), w(0xA1), w(0xA2)

        effects = []
        for i, core in enumerate(self.cores):
            c = core.cfg
            div_tick = bool((self.en >> i) & 1) and core.divcnt == 0
            step = w_step and (hwdata >> i) & 1
            tick = bool(div_tick or step)
            st = core.evaluate(tick, self.dout[i], pins, self.flags, self.timer)
            effects.append(st)

        # flags & pins
        fset = fclr = 0
        npout, npoe = self.pout, self.poe
        for st in effects:
            fset |= st["fset"]
            fclr |= st["fclr"]
            npout = (npout & ~st["pvm"] & 0xFFFF) | (st["pvv"] & st["pvm"] & 0xFFFF)
            npoe = (npoe & ~st["pdm"] & 0xFFFF) | (st["pdv"] & st["pdm"] & 0xFFFF)

        for i, (core, st) in enumerate(zip(self.cores, effects)):
            c = core.cfg
            base = 0x80 + 8 * i
            restart = w_ctrl and (hwdata >> (4 + i)) & 1
            divsync = w_ctrl and (hwdata >> (8 + i)) & 1
            fifoclr = w_ctrl and (hwdata >> (12 + i)) & 1
            host_fifo = haddr == base + 5
            core.tx.update(fifoclr, hw_en and host_fifo, hwdata, st["tx_pop"])
            core.rx.update(fifoclr, st["rx_push"], st["rx_data"], hr_en and host_fifo)
            # exec
            if w(base + 7):
                core.exec_pend, core.exec_ir = 1, hwdata
            elif st["exec_done"] or restart:
                core.exec_pend = 0
            # divider
            div_m1 = 0 if c.clkdiv == 0 else c.clkdiv - 1
            if divsync:
                core.divcnt = 0
            elif (self.en >> i) & 1:
                core.divcnt = 0 if core.divcnt >= div_m1 else core.divcnt + 1
            core.commit(st, w(base + 6), hwdata, restart)
            # config
            if w(base + 0):
                c.clkdiv = hwdata & 0xFFFF
            if w(base + 1):
                am = (1 << min(self.aw, 7)) - 1
                c.wrap_bot = hwdata & am
                c.wrap_top = (hwdata >> 7) & am
                c.jmp_pin = (hwdata >> 14) & 31
                c.side_pindir = (hwdata >> 19) & 1
                c.od = (hwdata >> 20) & 1
            if w(base + 2):
                c.out_dir = hwdata & 1
                c.in_dir = (hwdata >> 1) & 1
                c.autopull = (hwdata >> 2) & 1
                c.autopush = (hwdata >> 3) & 1
                c.pull_th = (hwdata >> 4) & 31
                c.push_th = (hwdata >> 9) & 31
                c.crc_src = (hwdata >> 14) & 3
                c.crc_reflect = (hwdata >> 16) & 1
            if w(base + 3):
                c.out_base = hwdata & 31
                c.out_cnt = (hwdata >> 5) & 31
                c.set_base = (hwdata >> 10) & 31
                c.set_cnt = (hwdata >> 15) & 15
            if w(base + 4):
                c.in_base = hwdata & 31
                c.side_base = (hwdata >> 5) & 31
                c.side_cnt = (hwdata >> 10) & 3
            if w(0xA8 + i):
                c.poly = hwdata & 0xFFFF

        # instruction banks: the host owns a stopped core's port for a write or
        # a read peek; otherwise every bank fetches bank[pc_next] (= new pc)
        im = self.imem_slot(haddr)
        for i, core in enumerate(self.cores):
            host = im is not None and im[0] == i and not (en_now >> i) & 1
            if host and hw_en:
                self.banks[i][im[1]] = hwdata & 0xFFFFFF
            elif host and hr_peek:
                self.dout[i] = self.banks[i][im[1]]
            else:
                self.dout[i] = self.banks[i][core.pc]
        self.timer = (self.timer + 1) & 0xFFFF
        if w_ctrl:
            self.en = hwdata & ((1 << self.nc) - 1)
        self.flags = (hwdata & 0xFF) if w_flags else ((self.flags & ~fclr) | fset) & 0xFF
        self.pout, self.poe = npout, npoe
        return rdata

    def snapshot(self):
        """Architectural state compared against the RTL every cycle."""
        s = [self.pout, self.poe, self.flags, self.en]
        for i, c in enumerate(self.cores):
            s += [c.pc, c.x, c.y, c.isr, c.osr, c.crc, c.isr_cnt, c.osr_cnt, c.dly,
                  c.tx.cnt, c.rx.cnt, c.exec_pend, 0 if c.rx.empty else c.rx.head, self.dout[i]]
        return s
