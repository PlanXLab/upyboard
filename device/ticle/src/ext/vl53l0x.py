__version__ = "1.0.0"
__author__ = "PlanX Lab Development Team"

from . import (
    utime,
    machine, micropython,
    I2c,
)


class VL53L0X:
    __SYSRANGE_START                      = 0x00
    __SYSTEM_SEQUENCE_CONFIG              = 0x01
    __SYSTEM_INTERMEASUREMENT_PERIOD      = 0x04
    __SYSTEM_INTERRUPT_CONFIG_GPIO        = 0x0A
    __SYSTEM_INTERRUPT_CLEAR              = 0x0B
    __GPIO_HV_MUX_ACTIVE_HIGH             = 0x84
    __RESULT_INTERRUPT_STATUS             = 0x13
    __RESULT_RANGE_STATUS                 = 0x14
    __RESULT_RANGE_MM_BE16                = __RESULT_RANGE_STATUS + 10  # 0x1E..0x1F
    __FINAL_RANGE_MIN_CNT_RATE_RTN_LIMIT  = 0x44
    __MSRC_CONFIG_CONTROL                 = 0x60
    __MSRC_CONFIG_TIMEOUT_MACROP          = 0x46
    __PRE_RANGE_CONFIG_VCSEL_PERIOD       = 0x50
    __PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI  = 0x51
    __FINAL_RANGE_CONFIG_VCSEL_PERIOD     = 0x70
    __FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI= 0x71
    __IDENTIFICATION_MODEL_ID             = 0xC0
   
    __VCSEL_PERIOD_PRE_RANGE              = 0
    __VCSEL_PERIOD_FINAL_RANGE            = 1

    MODE_PRESET_DEFAULT                   = 1
    MODE_PRESET_LONG_RANGE                = 2
    MODE_PRESET_HIGH_ACCURACY             = 3
    MODE_PRESET_HIGH_SPEED                = 4

    def __init__(self, scl: int, sda: int,
                 *,
                 address: int = 0x29,
                 freq: int = 400_000,
                 callback: callable | None = None,
                 preset: str = MODE_PRESET_DEFAULT,
                 period_ms: int = 0,           # 0=B2B, >0=Timed
                 timeout_ms: int = 250,
                 min_valid_mm: int = 50,
                 max_valid_mm: int | None = 1200,
                 consistency_pct: int = 15,
                 consistency_mm: int = 40,
                 filter_median: bool = True,
                 median_window: int = 5,
                 median_min_fill: int = 3):
        self.__i2c = I2c(scl=scl, sda=sda, addr=address, freq=freq)

        # State
        self.__timeout_ms = timeout_ms
        self.__stop_variable = 0
        self.__started = False

        # Filters & acceptance
        self.__min_valid_mm = min_valid_mm
        self.__max_valid_mm = max_valid_mm
        self.__consis_pct = consistency_pct
        self.__consis_mm  = consistency_mm
        self.__last_good_mm = -1

        # Async poll
        self.__tmr = None
        self.__cb = None
        self.__scheduled = False
        self.__poll_busy = False

        # Median filter
        self.set_median(enable=filter_median, window=median_window, min_fill=median_min_fill)
        
        # Ping
        try:
            _ = self.__i2c.read_u8(self.__IDENTIFICATION_MODEL_ID)
        except OSError:
            raise RuntimeError("VL53L0X not responding")

        # Core init & start            
        self.__core_init()
        
        if preset == self.MODE_PRESET_LONG_RANGE:
            self.set_signal_rate_limit_mcps(0.05)
            self.set_measurement_timing_budget_us(200_000)
        elif preset == self.MODE_PRESET_HIGH_ACCURACY:
            self.set_signal_rate_limit_mcps(0.25)
            self.set_measurement_timing_budget_us(200_000)
        elif preset == self.MODE_PRESET_HIGH_SPEED:
            self.set_signal_rate_limit_mcps(0.25)
            self.set_measurement_timing_budget_us(20_000)
        elif preset == self.MODE_PRESET_DEFAULT:
            self.set_signal_rate_limit_mcps(0.25)
            self.set_measurement_timing_budget_us(33_000)
        else:
            raise ValueError("Invalid preset")

        if callback is not None:
            self.on_read = callback
        else:
            self.__start_continuous(period_ms=period_ms)

    def read(self):
        if not self.__started:
            raise RuntimeError("not started")
        deadline = utime.ticks_add(utime.ticks_ms(), self.__timeout_ms)
        while not self.__data_ready():
            if utime.ticks_diff(deadline, utime.ticks_ms()) <= 0:
                return None
            utime.sleep_ms(1)
        return self.__read_nowait()

    @property
    def on_read(self):
        return self.__cb

    @on_read.setter
    def on_read(self, callback):
        if callback is None:
            self.deinit()
            return
        self.__cb = callback

        budget_ms = max(10, self.get_measurement_timing_budget_us() // 1000)
        self.__poll_interval_ms = min(max(10, budget_ms // 2), 60)

        if self.__tmr:
            self.__stop_async()
        self.__tmr = machine.Timer()
        self.__tmr.init(period=self.__poll_interval_ms, mode=machine.Timer.PERIODIC, callback=self.__tmr_isr)

    def set_median(self, enable: bool, window: int, min_fill: int):
        self.__use_median = enable
        if enable is False:
            return
        
        self.__med_win = window if window >= 3 else 3
        if self.__med_win % 2 == 0:
            self.__med_win += 1
        self.__med_buf = [0] * self.__med_win
        self.__med_cnt = 0
        self.__med_idx = 0
        self.__med_min_fill = max(1, min(min_fill, self.__med_win))

    def set_signal_rate_limit_mcps(self, limit_mcps: float = 0.25):
        if limit_mcps < 0.0:
            limit_mcps = 0.0
        val = int(limit_mcps * (1 << 7)) & 0xFFFF
        self.__i2c.write_u16(self.__FINAL_RANGE_MIN_CNT_RATE_RTN_LIMIT, val, little_endian=False)

    def get_measurement_timing_budget_us(self) -> int:
        tcc, dss, msrc, pre, final = self.__get_sequence_step_enables()
        msrc_us, pre_us, final_us, _, _ = self.__get_sequence_step_timeouts(pre)
        budget = 1910
        if tcc:
            budget += msrc_us + 590
        
        if dss:
            budget += 2 * (msrc_us + 690)
        elif msrc:
            budget += msrc_us + 660
        
        if pre:
            budget += pre_us + 660
        if final:
            budget += final_us + 550
        return int(budget)

    def set_measurement_timing_budget_us(self, budget_us: int):
        if budget_us < 20000:
            budget_us = 20000
        tcc, dss, msrc, pre, final = self.__get_sequence_step_enables()
        msrc_us, pre_us, _, final_vcsel_pclks, pre_mclks = self.__get_sequence_step_timeouts(pre)

        used = 1910
        if tcc:
            used += msrc_us + 590
        
        if dss:
            used += 2 * (msrc_us + 690)
        elif msrc:
            used += msrc_us + 660
        
        if pre:
            used += pre_us + 660
        if final:
            used += 550
            if used > budget_us:
                raise ValueError("budget too small")
            final_timeout_us = budget_us - used
            final_timeout_mclks = self.__timeout_us_to_mclks(final_timeout_us, final_vcsel_pclks)
            if pre:
                final_timeout_mclks += pre_mclks
            self.__i2c.write_u16(self.__FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, self.__encode_timeout(final_timeout_mclks), little_endian=False)

    def deinit(self):
        if self.__cb:
            self.__stop_async()
        if self.__started:
            self.__stop()

    def __core_init(self):
        for reg, val in ((0x88, 0x00), (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00)):
            self.__i2c.write_u8(reg, val)

        self.__stop_variable = self.__i2c.read_u8(0x91)
        for reg, val in ((0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self.__i2c.write_u8(reg, val)

        self.__i2c.write_u8(self.__SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        self.__i2c.write_u8(self.__GPIO_HV_MUX_ACTIVE_HIGH, 0x00)
        self.__i2c.write_u8(self.__SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.__i2c.write_u8(self.__MSRC_CONFIG_CONTROL, self.__i2c.read_u8(self.__MSRC_CONFIG_CONTROL) | 0x12)

        # Initialize SPADs
        self.__i2c.write_u8(self.__SYSTEM_SEQUENCE_CONFIG, 0xFF)
        tuning = (
            (0xFF,0x01),(0x00,0x00),(0xFF,0x00),(0x09,0x00),(0x10,0x00),(0x11,0x00),
            (0x24,0x01),(0x25,0xFF),(0x75,0x00),(0xFF,0x01),(0x4E,0x2C),(0x48,0x00),
            (0x30,0x20),(0xFF,0x00),(0x30,0x09),(0x54,0x00),(0x31,0x04),(0x32,0x03),
            (0x40,0x83),(0x46,0x25),(0x60,0x00),(0x27,0x00),(0x50,0x06),(0x51,0x00),
            (0x52,0x96),(0x56,0x08),(0x57,0x30),(0x61,0x00),(0x62,0x00),(0x64,0x00),
            (0x65,0x00),(0x66,0xA0),(0xFF,0x01),(0x22,0x32),(0x47,0x14),(0x49,0xFF),
            (0x4A,0x00),(0xFF,0x00),(0x7A,0x0A),(0x7B,0x00),(0x78,0x21),(0xFF,0x01),
            (0x23,0x34),(0x42,0x00),(0x44,0xFF),(0x45,0x26),(0x46,0x05),(0x40,0x40),
            (0x0E,0x06),(0x20,0x1A),(0x43,0x40),(0xFF,0x00),(0x34,0x03),(0x35,0x44),
            (0xFF,0x01),(0x31,0x04),(0x4B,0x09),(0x4C,0x05),(0x4D,0x04),(0xFF,0x00),
        )
        for reg, val in tuning:
            self.__i2c.write_u8(reg, val)

    def __start_continuous(self, period_ms=0):
        for reg, val in ((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, self.__stop_variable), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self.__i2c.write_u8(reg, val)

        if period_ms and period_ms > 0:
            b = bytes(((period_ms >> 24) & 0xFF, (period_ms >> 16) & 0xFF, (period_ms >> 8) & 0xFF, period_ms & 0xFF))
            self.__i2c.writeto_mem(self.__SYSTEM_INTERMEASUREMENT_PERIOD, b, addrsize=8)
            self.__i2c.write_u8(self.__SYSRANGE_START, 0x04)  # Timed continuous
        else:
            self.__i2c.write_u8(self.__SYSRANGE_START, 0x02)  # Back-to-back

        t_end = utime.ticks_add(utime.ticks_ms(), 50)
        while (self.__i2c.read_u8(self.__SYSRANGE_START) & 0x01) > 0:
            if utime.ticks_diff(t_end, utime.ticks_ms()) <= 0:
                break
        self.__started = True

    def __stop(self):
        self.__i2c.write_u8(self.__SYSRANGE_START, 0x01)
        self.__started = False

    def __stop_async(self):
        if self.__tmr:
            try:
                self.__tmr.deinit()
            except Exception:
                pass
            self.__tmr = None
        self.__cb = None
        self.__scheduled = False
        self.__poll_busy = False

    def __accept(self, mm: int, st: int) -> bool:
        if mm == 0xFFFF:
            return False
        if self.__min_valid_mm and mm < self.__min_valid_mm:
            return False
        if (self.__max_valid_mm is not None) and mm > self.__max_valid_mm:
            return False
        if st in (0, 5, 11):  # ok status
            return True
        if (st in (1, 2)) and (self.__last_good_mm >= 0):  # soft status
            thr = max(self.__consis_mm, (self.__last_good_mm * self.__consis_pct) // 100)
            if abs(mm - self.__last_good_mm) <= thr:
                return True
        return False

    def __data_ready(self) -> bool:
        if not self.__started:
            return False
        if (self.__i2c.read_u8(self.__RESULT_INTERRUPT_STATUS) & 0x07) != 0:
            return True
        mm = self.__i2c.read_u16(self.__RESULT_RANGE_MM_BE16, little_endian=False)
        st = (self.__i2c.read_u8(self.__RESULT_RANGE_STATUS) & 0x78) >> 3
        return self.__accept(mm, st)

    def __read_nowait(self):
        if not self.__started:
            return None

        st = (self.__i2c.read_u8(self.__RESULT_RANGE_STATUS) & 0x78) >> 3
        mm = self.__i2c.read_u16(self.__RESULT_RANGE_MM_BE16, little_endian=False)
        self.__i2c.write_u8(self.__SYSTEM_INTERRUPT_CLEAR, 0x01)

        if not self.__accept(mm, st):
            return None
        val = self.__median_push_get(mm) if self.__use_median else mm
        self.__last_good_mm = val
        return val
    
    def __median_push_get(self, mm: int) -> int | None:
        self.__med_buf[self.__med_idx] = int(mm)
        self.__med_idx = (self.__med_idx + 1) % self.__med_win
        if self.__med_cnt < self.__med_win:
            self.__med_cnt += 1
        if self.__med_cnt < self.__med_min_fill:
            return mm
        vals = self.__med_buf[:self.__med_cnt]
        vals.sort()
        return vals[self.__med_cnt // 2]

    def __get_sequence_step_enables(self):
        sc = self.__i2c.read_u8(self.__SYSTEM_SEQUENCE_CONFIG)
        tcc = ((sc >> 4) & 1) != 0
        dss = ((sc >> 3) & 1) != 0
        msrc = ((sc >> 2) & 1) != 0
        pre = ((sc >> 6) & 1) != 0
        final = ((sc >> 7) & 1) != 0
        return tcc, dss, msrc, pre, final

    def __get_vcsel_pulse_period(self, which):
        if which == self.__VCSEL_PERIOD_PRE_RANGE:
            return self.__i2c.read_u8(self.__PRE_RANGE_CONFIG_VCSEL_PERIOD)
        else:
            return self.__i2c.read_u8(self.__FINAL_RANGE_CONFIG_VCSEL_PERIOD)

    def __get_sequence_step_timeouts(self, pre_enabled: bool):
        pre_vcsel = self.__get_vcsel_pulse_period(self.__VCSEL_PERIOD_PRE_RANGE)
        msrc_mclks = (self.__i2c.read_u8(self.__MSRC_CONFIG_TIMEOUT_MACROP) + 1) & 0xFF
        msrc_us = self.__timeout_mclks_to_us(msrc_mclks, pre_vcsel)

        pre_mclks = self.__decode_timeout(self.__i2c.read_u16(self.__PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, little_endian=False))
        pre_us = self.__timeout_mclks_to_us(pre_mclks, pre_vcsel)

        final_vcsel = self.__get_vcsel_pulse_period(self.__VCSEL_PERIOD_FINAL_RANGE)
        final_mclks = self.__decode_timeout(self.__i2c.read_u16(self.__FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, little_endian=False))
        if pre_enabled:
            final_mclks -= pre_mclks
        final_us = self.__timeout_mclks_to_us(final_mclks, final_vcsel)
        return msrc_us, pre_us, final_us, final_vcsel, pre_mclks

    def __decode_timeout(self, val):
        ls = val & 0xFF
        ms = (val >> 8) & 0xFF
        return int(ls * (1 << ms) + 1)

    def __encode_timeout(self, timeout_mclks):
        if timeout_mclks <= 0:
            return 0
        ls = int(timeout_mclks) - 1
        ms = 0
        while ls > 255:
            ls >>= 1
            ms += 1
        return ((ms << 8) | (ls & 0xFF)) & 0xFFFF

    def __macro_period_ns(self, vcsel_period_pclks: int) -> int:
        return ((2304 * vcsel_period_pclks * 1655) + 500) // 1000

    def __timeout_mclks_to_us(self, timeout_mclks: int, vcsel_period_pclks: int) -> int:
        mp_ns = self.__macro_period_ns(vcsel_period_pclks)
        return ((timeout_mclks * mp_ns) + (mp_ns // 2)) // 1000

    def __timeout_us_to_mclks(self, timeout_us: int, vcsel_period_pclks: int) -> int:
        mp_ns = self.__macro_period_ns(vcsel_period_pclks)
        return ((timeout_us * 1000) + (mp_ns // 2)) // mp_ns

    def __tmr_isr(self, _):
        if self.__scheduled:
            return
        
        self.__scheduled = True
        try:
            micropython.schedule(self.__poll_sched, 0)
        except Exception:
            self.__scheduled = False

    def __poll_sched(self, _):
        self.__scheduled = False
        if self.__poll_busy or self.__cb is None:
            return
        
        self.__poll_busy = True
        try:
            mm = self.__read_nowait()
            if mm is not None:
                try:
                    self.__cb(mm)
                except Exception:
                    pass
        finally:
            self.__poll_busy = False

