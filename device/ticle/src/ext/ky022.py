__version__ = "1.0.0"
__author__ = "PlanX Lab Development Team"

from . import (
    utime,
    machine, micropython,
)


class KY022:
    REPEAT   = -1
    BADSTART = -2
    BADBLOCK = -3
    BADREP   = -4
    OVERRUN  = -5
    BADDATA  = -6
    BADADDR  = -7

    PROTOCOL_NEC_8      = micropython.const(1)   # LG/Normal NEC (8-bit addr)
    PROTOCOL_NEC_16     = micropython.const(2)   # NEC Expanded (16-bit addr)
    PROTOCOL_SAMSUNG    = micropython.const(3)   # NEC Modified (Reader space 4.5ms)
    PROTOCOL_SIRC12     = micropython.const(4)   # SONY SIRC 12-bit
    PROTOCOL_SIRC15     = micropython.const(5)   # SONY SIRC 15-bit
    PROTOCOL_SIRC20     = micropython.const(6)   # SONY SIRC 20-bit
    PROTOCOL_RC5        = micropython.const(7)   # Philips RC5 (Manchester, 14b)
    PROTOCOL_RC6        = micropython.const(8)   # Philips RC6 Mode0 (Manchester)
    PROTOCOL_PANA       = micropython.const(9)   # Panasonic(Kaseikyo, 48b NEC series)
    PROTOCOL_CARRIER40  = micropython.const(10)  # Carrier 40-bit
    PROTOCOL_CARRIER84  = micropython.const(11)  # Carrier 84-bit
    PROTOCOL_CARRIER128 = micropython.const(12)  # Carrier 128-bit
    PROTOCOL_HVAC_NEC   = micropython.const(13)  # Generic HVAC NEC-series (bit count specified)

    def __init__(self, pin,
                 *,
                 on_receive: callable = None,
                 protocol: int = PROTOCOL_NEC_8,
                 queue_size: int = 16,
                 tol_pct: int = 25, 
                 irq_trigger=None,
                 # NEC Repeat Suppression/Indication/Throttle
                 emit_repeat: bool = True,
                 repeat_first_delay_ms: int = 150,
                 repeat_min_interval_ms: int = 100,
                 hold_throttle_ms: int = 0,       # Minimum interval between same key reissues (0=off)
                 # HVAC-NEC Universal Selection Parameters
                 hvac_bits: int = 0,              # Number of bits used in PROTOCOL_HVAC_NEC (ex: 56/84/104/128…)
                 hvac_zero_space_us: int = 560,   # 0 space Approximate width
                 hvac_one_space_us: int = 1690,   # 1 space Approximate width
                 hvac_hdr_mark_us: int = 9000,    # leader mark Approximate width
                 hvac_hdr_space_us: int = 4500    # leader space Approximate width(No repeat frames, usually 4.5ms or more)
    ):
        self.__proto = int(protocol)        
        self.__on_receive = on_receive
        self.__capturing = False 

        # Timing parameters
        self.__tol_pct = int(tol_pct)
        # Last frame (NEC affiliation repeat)
        self.__addr_last = 0
        self.__cmd_last  = None
        # NEC/Samsung
        self.__nec_one_bound = 1120  # 0/1 Boundary (~1.12ms)
        # NEC Repeat/Emit/Throttle
        self.__emit_repeat = bool(emit_repeat)
        self.__repeat_first_delay_ms = int(repeat_first_delay_ms)
        self.__repeat_min_interval_ms = int(repeat_min_interval_ms)
        self.__hold_throttle_ms = int(hold_throttle_ms)
        self.__last_full_ms = 0
        self.__last_repeat_ms = 0
        self.__last_emit_ms = 0
        self.__last_emit_cmd = None
        self.__last_emit_addr = 0
        # SIRC
        self.__sirc_T = 600  # us
        # RC5/RC6 (Half bit T)
        self.__rc5_T = 889   # us
        self.__rc6_T = 444   # us
        self.__rc6_hdr_mark = 6 * self.__rc6_T   # ~2664us
        self.__rc6_hdr_space = 2 * self.__rc6_T  # ~888us
        # Panasonic(Kaseikyo) Approximate timing
        self.__pana_hdr_mark = 3500
        self.__pana_hdr_space = 1750
        self.__pana_zero_space = 430
        self.__pana_one_space = 1290
        self.__pana_bits = 48
        # HVAC-NEC Universal (Various Air Conditioner Remote Controls) Basic Approximation
        self.__hvac_bits = int(hvac_bits)
        self.__hvac_zero_us = int(hvac_zero_space_us)
        self.__hvac_one_us = int(hvac_one_space_us)
        self.__hvac_hdr_mark = int(hvac_hdr_mark_us)
        self.__hvac_hdr_space = int(hvac_hdr_space_us)
        # Queue
        self.__q  = [None] * max(2, int(queue_size))
        self.__qh = 0
        self.__qt = 0
        # IRQ Trigger
        self.__irq_trigger = irq_trigger if irq_trigger is not None else machine.Pin.IRQ_FALLING
        self.__ir_rx = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        self.__ir_rx.irq(handler=self.__cb_start, trigger=self.__irq_trigger)

    @property
    def on_receive(self):
        return self.__on_receive

    @on_receive.setter
    def on_receive(self, cb):
        self.__on_receive = cb

    def deinit(self):
        self.__ir_rx.irq(handler=None)

    def get(self, block=False, timeout_ms=1000):
        if not block:
            return self.__q_get_nowait()
        deadline = utime.ticks_add(utime.ticks_ms(), int(timeout_ms))
        while True:
            evt = self.__q_get_nowait()
            if evt is not None:
                return evt
            if utime.ticks_diff(deadline, utime.ticks_ms()) <= 0:
                return None
            utime.sleep_ms(1)

    @micropython.native
    def __cb_start(self, pin_obj):
        if self.__capturing:
            return

        self.__capturing = True
        self.__ir_rx.irq(handler=None)

        try:
            micropython.schedule(self.__decode_hw_sched, 0)
        except Exception:
            self.__capturing = False
            self.__ir_rx.irq(handler=self.__cb_start, trigger=self.__irq_trigger)

    def __decode_hw_sched(self, _):
        try:
            cmd, addr, ext, is_rep = self.__capture_frame()
            self.__addr_last = addr
            if cmd >= 0:
                self.__cmd_last = cmd
            self.__finish_ok(cmd, addr, ext, is_rep)
        except RuntimeError as e:
            pass
        finally:
            self.__capturing = False
            self.__ir_rx.irq(handler=self.__cb_start, trigger=(machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING))

    def __pulse(self, level, timeout_us):
        us = machine.time_pulse_us(self.__ir_rx, level, timeout_us)
        if us < 0:
            raise RuntimeError(self.BADBLOCK)
        return us

    def __close(self, val, tgt, tol_pct):
        tol_abs = (tgt * tol_pct) // 100
        return abs(val - tgt) <= tol_abs  # abs(val - tgt) * 100 <= tgt * tol_pct

    def __capture_frame(self):
        p = self.__proto
        if p in (self.PROTOCOL_NEC_8, self.PROTOCOL_NEC_16, self.PROTOCOL_SAMSUNG):
            return self.__cap_nec_like()
        if p in (self.PROTOCOL_SIRC12, self.PROTOCOL_SIRC15, self.PROTOCOL_SIRC20):
            return self.__cap_sirc()
        if p == self.PROTOCOL_PANA:
            return self.__cap_panasonic()
        if p == self.PROTOCOL_RC5:
            return self.__cap_rc5()
        if p == self.PROTOCOL_RC6:
            return self.__cap_rc6()
        if p in (self.PROTOCOL_CARRIER40, self.PROTOCOL_CARRIER84, self.PROTOCOL_CARRIER128, self.PROTOCOL_HVAC_NEC):
            return self.__cap_hvac_nec()
        raise RuntimeError(self.BADBLOCK)

    def __cap_nec_like(self):
        mark1  = self.__pulse(0, 40000)  # ~9ms
        space1 = self.__pulse(1, 40000)  # 4.5ms(normal) / 2.25ms(repeat)

        if not self.__close(mark1, 9000, self.__tol_pct):
            raise RuntimeError(self.BADSTART)

        normal = space1 > 3000
        repeat = 1700 < space1 <= 3000
        if not (normal or repeat):
            raise RuntimeError(self.BADSTART)

        if repeat:
            try:
                m2 = self.__pulse(0, 5000)
            except RuntimeError:
                raise RuntimeError(self.BADREP)
            if not self.__close(m2, 560, self.__tol_pct):
                raise RuntimeError(self.BADREP)
            if self.__cmd_last is None:
                raise RuntimeError(self.BADREP)
            return (self.__cmd_last, self.__addr_last, 0, True)

        val = 0
        pulse = self.__pulse
        nec_one = self.__nec_one_bound
        for _ in range(32):
            m = pulse(0, 3000)   # ~560
            if not self.__close(m, 560, self.__tol_pct):
                raise RuntimeError(self.BADDATA)
            s = pulse(1, 5000)   # ~560 / ~1690
            val >>= 1
            if s > nec_one:
                val |= 0x80000000

        a  = val & 0xFF
        na = (val >> 8 ) & 0xFF
        c  = (val >> 16) & 0xFF
        nc = (val >> 24) & 0xFF
        if (c ^ nc) & 0xFF != 0xFF:
            raise RuntimeError(self.BADDATA)

        if self.__proto == self.PROTOCOL_NEC_8:
            if (a ^ na) & 0xFF != 0xFF:
                raise RuntimeError(self.BADADDR)
            addr = a
        else:
            addr = a | (na << 8)

        cmd = c
        return (cmd, addr, 0, False)

    def __cap_sirc(self):
        T = self.__sirc_T
        m = self.__pulse(0, 25000)
        s = self.__pulse(1, 25000)
        if not (self.__close(m, 4*T, self.__tol_pct) and self.__close(s, 1*T, self.__tol_pct)):
            raise RuntimeError(self.BADSTART)

        bits = 12 if self.__proto == self.PROTOCOL_SIRC12 else (15 if self.__proto == self.PROTOCOL_SIRC15 else 20)
        val = 0
        for i in range(bits):
            dm = self.__pulse(0, 4000)  # ~1T
            ds = self.__pulse(1, 4000)  # 1T or 2T
            if not self.__close(dm, 1*T, self.__tol_pct):
                raise RuntimeError(self.BADDATA)
            b = 1 if self.__close(ds, 2*T, self.__tol_pct) else 0
            val |= (b << i)

        cmd =  val        & 0x7F
        if bits == 12:
            addr = (val >> 7) & 0x1F; ext = 0
        elif bits == 15:
            addr = (val >> 7) & 0xFF; ext = 0
        else:
            addr = (val >> 7)  & 0x1F
            ext  = (val >> 12) & 0xFF
        return (cmd, addr, ext, False)

    def __cap_panasonic(self):
        m = self.__pulse(0, 30000)
        s = self.__pulse(1, 30000)
        if not (self.__close(m, self.__pana_hdr_mark, self.__tol_pct) and
                self.__close(s, self.__pana_hdr_space, self.__tol_pct)):
            raise RuntimeError(self.BADSTART)

        bits = self.__pana_bits
        val = 0
        for i in range(bits):
            _m = self.__pulse(0, 3000)  # ~430
            sp = self.__pulse(1, 5000)  # 430 / 1290
            b = 1 if abs(sp - self.__pana_one_space) < abs(sp - self.__pana_zero_space) else 0
            val |= (b << i)

        addr = (val) & 0xFFFF
        data = ((val >>16)) & 0xFFFFFFFF
        cmd  = data & 0xFF
        ext  = (data >> 8) & 0xFFFFFF
        return (cmd, addr, ext, False)

    def __cap_hvac_nec(self):
        p = self.__proto
        if p in (self.PROTOCOL_CARRIER40, self.PROTOCOL_CARRIER84, self.PROTOCOL_CARRIER128):
            bits = 40 if p == self.PROTOCOL_CARRIER40 else (84 if p == self.PROTOCOL_CARRIER84 else 128)
        if p == self.PROTOCOL_HVAC_NEC:
            if self.__hvac_bits <= 0:
                raise RuntimeError(self.BADBLOCK)
            bits = self.__hvac_bits

        hdr_mark = self.__hvac_hdr_mark
        hdr_space = self.__hvac_hdr_space
        zero_us = self.__hvac_zero_us
        one_us = self.__hvac_one_us

        m = self.__pulse(0, 50000)
        s = self.__pulse(1, 50000)
        if not (self.__close(m, hdr_mark, self.__tol_pct) and self.__close(s, hdr_space, self.__tol_pct)):
            if not (m >= 2500 and s >= 3000):
                raise RuntimeError(self.BADSTART)

        val = 0
        thr = (zero_us + one_us) // 2
        pulse = self.__pulse

        for i in range(bits):
            try:
                mbit = pulse(0, 8000)
            except RuntimeError:
                raise RuntimeError(self.BADDATA)
            if not (300 <= mbit <= 1200):
                pass
            sp = pulse(1, 20000)
            b = 1 if sp > thr else 0
            val |= (b << i)

        b0 = val & 0xFF
        b1 = (val >> 8 ) & 0xFF
        b2 = (val >> 16) & 0xFF
        b3 = (val >> 24) & 0xFF
        b4 = (val >> 32) & 0xFF if bits >= 40 else 0

        cmd = b0
        addr = b1 | (b2 << 8)
        ext = b3 | (b4 << 8)

        return (cmd, addr, ext, False)

    def __cap_rc5(self):
        T = self.__rc5_T
        half = []
        lvl = 0 
        for _ in range(40):
            try:
                d0 = self.__pulse(0, 4000)  # Low
                n0 = max(1, min(2, int((d0 + T//2)//T)))
                for _ in range(n0):
                    half.append(lvl); lvl ^= 1
                d1 = self.__pulse(1, 4000)  # High
                n1 = max(1, min(2, int((d1 + T//2)//T)))
                for _ in range(n1):
                    half.append(lvl); lvl ^= 1
            except RuntimeError:
                break
            if len(half) >= 28:  # 14bit * 2 half
                break
        if len(half) < 28:
            raise RuntimeError(self.BADBLOCK)

        def to_bits(offset):
            bits = []
            i = offset
            while i + 1 < len(half) and len(bits) < 14:
                a, b = half[i], half[i+1]
                bits.append(1 if (a == 0 and b == 1) else 0)  # 01->1, 10->0
                i += 2
            return bits

        cand0, cand1 = to_bits(0), to_bits(1)
        bits = None
        if len(cand0) >= 2 and cand0[0] == 1 and cand0[1] == 1:
            bits = cand0
        elif len(cand1) >= 2 and cand1[0] == 1 and cand1[1] == 1:
            bits = cand1
        else:
            raise RuntimeError(self.BADSTART)
        
        if len(bits) < 14:
            raise RuntimeError(self.BADBLOCK)

        S1, S2, Tgl = bits[0], bits[1], bits[2]
        addr = 0
        for b in bits[3:8]:
            addr = (addr << 1) | b
        
        cmd = 0
        for b in bits[8:14]:
            cmd = (cmd << 1) | b
        
        if S2 == 0:
            cmd |= 0x40
        return (cmd, addr, Tgl, False)

    def __cap_rc6(self):
        T = self.__rc6_T
        m = self.__pulse(0, 20000)
        s = self.__pulse(1, 20000)
        if not (self.__close(m, self.__rc6_hdr_mark, self.__tol_pct) and self.__close(s, self.__rc6_hdr_space, self.__tol_pct)):
            raise RuntimeError(self.BADSTART)

        half = []
        lvl = 0
        for _ in range(50):
            try:
                d0 = self.__pulse(0, 5000); n0 = max(1, min(2, int((d0 + T//2)//T)))
                for _ in range(n0):
                    half.append(lvl); lvl ^= 1
                d1 = self.__pulse(1, 5000); n1 = max(1, min(2, int((d1 + T//2)//T)))
                for _ in range(n1):
                    half.append(lvl); lvl ^= 1
            except RuntimeError:
                break
            if len(half) >= 44:
                break
        if len(half) < 22:
            raise RuntimeError(self.BADBLOCK)

        def to_bits(offset):
            bits = []
            i = offset
            while i + 1 < len(half) and len(bits) < 21:
                a, b = half[i], half[i+1]
                bits.append(1 if (a == 0 and b == 1) else 0)
                i += 2
            return bits

        cand = []
        for off in (0, 1):
            b = to_bits(off)
            if len(b) >= 5 and b[0] == 1 and b[1] == 0 and b[2] == 0 and b[3] == 0:
                cand.append(b)
        if not cand:
            raise RuntimeError(self.BADSTART)
        bits = cand[0]
        nbits = min(len(bits), 21)

        tgl = bits[4] if nbits > 4 else 0
        idx = 5
        addr = 0
        for _ in range(8):
            if idx >= nbits: break
            addr = (addr << 1) | bits[idx]; idx += 1
        cmd = 0
        while idx < nbits and (idx - 5 - 8) < 8:
            cmd = (cmd << 1) | bits[idx]; idx += 1
        return (cmd, addr, tgl, False)

    def __finish_ok(self, cmd, addr, ext=0, is_repeat=False):
        now_ms = utime.ticks_ms()

        if is_repeat:
            if not self.__emit_repeat:
                return
            if utime.ticks_diff(now_ms, self.__last_full_ms) < self.__repeat_first_delay_ms:
                return
            if utime.ticks_diff(now_ms, self.__last_repeat_ms) < self.__repeat_min_interval_ms:
                return
            self.__last_repeat_ms = now_ms
        else:
            self.__last_full_ms = now_ms
            self.__last_repeat_ms = 0

        if self.__proto in (self.PROTOCOL_CARRIER40, self.PROTOCOL_CARRIER84, self.PROTOCOL_CARRIER128, self.PROTOCOL_HVAC_NEC):
            base_cmd = cmd & ~0x0C
        else:
            base_cmd = cmd

        # Hold throttle
        if self.__hold_throttle_ms > 0 and base_cmd == self.__last_emit_cmd and addr == self.__last_emit_addr and utime.ticks_diff(now_ms, self.__last_emit_ms) < self.__hold_throttle_ms:
            return

        self.__cmd_last = cmd if cmd >= 0 else self.__cmd_last
        if self.__on_receive:
            try:
                self.__on_receive(cmd, addr, ext)
            except Exception:
                pass
        else:
            self.__q_put((cmd, addr, ext))
            
        self.__last_emit_ms = now_ms
        self.__last_emit_cmd = base_cmd
        self.__last_emit_addr = addr

    def __q_put(self, evt):
        nxt = (self.__qt + 1) % len(self.__q)
        if nxt == self.__qh:
            self.__qh = (self.__qh + 1) % len(self.__q)
        self.__q[self.__qt] = evt
        self.__qt = nxt

    def __q_get_nowait(self):
        if self.__qh == self.__qt:
            return None
        evt = self.__q[self.__qh]
        self.__q[self.__qh] = None
        self.__qh = (self.__qh + 1) % len(self.__q)
        return evt

