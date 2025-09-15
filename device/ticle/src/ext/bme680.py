__version__ = "1.0.0"
__author__ = "PlanX Lab Development Team"

from . import (
    math, utime, ustruct,
    machine, micropython,
    I2c
)


class BME680:
    __FORCED_MODE      = 0x01
    __SLEEP_MODE       = 0x00
    __REG_STATUS       = 0x1D
    __REG_CTRL_GAS1    = 0x71
    __REG_GAS_WAIT_0   = 0x64
    __BIT_NEW_DATA     = 0x80
    __BITS_GAS_OK      = 0x30
    
    __NORM_BASE_PERIOD = 3000  # ms

    def __init__(self, scl: int, sda: int,
                 *,
                 address: int = 0x77,
                 freq: int = 400_000
        ):
        self.__i2c = I2c(scl=scl, sda=sda, addr=address, freq=freq)
        
        chip_id = self.__i2c.readfrom_mem(0xD0, 1)[0]
        if chip_id != 0x61:
            raise OSError("BME680 not detected")

        # soft reset
        self.__i2c.writeto_mem(0xE0, bytes([0xB6]))
        utime.sleep_ms(5)

        # sleep mode
        self.__set_power_mode(self.__SLEEP_MODE)

        # == Calibration value parsing (datasheet specification) =====
        # Block 1: 0x8A.. (23B): T2/T3, P1..P10 etc.
        cal1 = self.__i2c.readfrom_mem(0x8A, 23)
        # Block 2: 0xE1.. (14B): H1/H2 assembly + H3..H7 + T1 + GH1..GH3
        cal2 = self.__i2c.readfrom_mem(0xE1, 14)
        # Block 3: 0x00.. (5B): res_heat_val(0x00), res_heat_range(0x02[5:4]), range_sw_err(0x04[7:4])
        cal3 = self.__i2c.readfrom_mem(0x00, 5)

        # Temperature calibration (par_t1, par_t2, par_t3)
        par_t2 = ustruct.unpack('<h', cal1[0:2])[0]        # 0x8A-0x8B (s16)
        par_t3 = ustruct.unpack('b',  cal1[2:3])[0]        # 0x8C     (s8)
        par_t1 = ustruct.unpack('<H', cal2[8:10])[0]       # 0xE9-0xEA (u16)
        self.__par_t1, self.__par_t2, self.__par_t3 = par_t1, par_t2, par_t3

        # Pressure calibration (par_p1..par_p10) 
        par_p1  = ustruct.unpack('<H', cal1[4:6])[0]       # u16
        par_p2  = ustruct.unpack('<h', cal1[6:8])[0]       # s16
        par_p3  = ustruct.unpack('b',  cal1[8:9])[0]       # s8
        par_p4  = ustruct.unpack('<h', cal1[10:12])[0]     # s16
        par_p5  = ustruct.unpack('<h', cal1[12:14])[0]     # s16
        par_p7  = ustruct.unpack('b',  cal1[14:15])[0]     # s8
        par_p6  = ustruct.unpack('b',  cal1[15:16])[0]     # s8
        par_p8  = ustruct.unpack('<h', cal1[18:20])[0]     # s16
        par_p9  = ustruct.unpack('<h', cal1[20:22])[0]     # s16
        par_p10 = cal1[22]                                                    # 0xA0
        self.__pressure_calibration = [par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10]

        # Humidity calibration (par_h1..par_h7) — H1/H2 is bit mixing
        # E1: H2_MSB, E2: H2_LSB/H1_LSB, E3: H1_MSB
        h2_msb = cal2[0]           # 0xE1
        h1_lsb = cal2[1]           # 0xE2 (low nibble for H1, high nibble for H2)
        h1_msb = cal2[2]           # 0xE3
        par_h1 = (h1_msb << 4) | (h1_lsb & 0x0F)
        par_h2 = (h2_msb << 4) | (h1_lsb >> 4)

        par_h3 = ustruct.unpack('b', bytes([cal2[3]]))[0]  # 0xE4
        par_h4 = ustruct.unpack('b', bytes([cal2[4]]))[0]  # 0xE5
        par_h5 = ustruct.unpack('b', bytes([cal2[5]]))[0]  # 0xE6
        par_h6 = cal2[6]                                   # 0xE7 (u8)
        par_h7 = ustruct.unpack('b', bytes([cal2[7]]))[0]  # 0xE8
        self.__humidity_calibration = [par_h1, par_h2, par_h3, par_h4, par_h5, par_h6, par_h7]

        # Gas/Heater calibration
        par_gh2 = ustruct.unpack('<h', cal2[10:12])[0]     # 0xEB-0xEC
        par_gh1 = ustruct.unpack('b',  cal2[12:13])[0]     # 0xED
        par_gh3 = ustruct.unpack('b',  cal2[13:14])[0]     # 0xEE
        self.__par_gh1, self.__par_gh2, self.__par_gh3 = par_gh1, par_gh2, par_gh3

        res_heat_val   = ustruct.unpack('b', bytes([cal3[0]]))[0]  # 0x00 (signed)
        res_heat_range = (cal3[2] >> 4) & 0x03                     # 0x02[5:4]
        range_sw_err   = (cal3[4] >> 4) & 0x0F                     # 0x04[7:4]
        self.__res_heat_val   = res_heat_val
        self.__res_heat_range = res_heat_range
        self.__sw_err         = range_sw_err

        # Measurement/Filter Settings
        # ctrl_hum (0x72): OSRS_H = x1
        self.__i2c.writeto_mem(0x72, bytes([0b001]))
        # ctrl_meas (0x74): OSRS_T = x2 (010), OSRS_P = x4 (011), mode is set in the function
        self.__i2c.writeto_mem(0x74, bytes([(0b010 << 5) | (0b011 << 2)]))
        # config (0x75): IIR filter coeff = 8 (011)
        self.__i2c.writeto_mem(0x75, bytes([0b011 << 2]))

        # Internal state
        self.__temperature_correction = 0
        self.__t_fine = None
        self.__adc_pres = None
        self.__adc_temp = None
        self.__adc_hum  = None
        self.__adc_gas  = None
        self.__gas_range = None
        
        self.__last_ctrl_gas1 = 0  # self.__REG_CTRL_GAS1 last value (toggle minimize)
        
        self.__ready_gas = False
        self.__last_valid_gas = None        
        self.__gas_baseline = 90_000  # 90 kOhm
        self.__gas_baseline_auto_update_ms = 300_000  # 5 minutes
 
        self.__period_ms = self.__NORM_BASE_PERIOD
        self.__next_due_ms = utime.ticks_ms()
  
        self.__tmr = None
        self.__cb = None
        self.__poll_busy = False
        self.__scheduled = False
        self.__is_gas = False
        self.__is_read_callback = True

    def __auto_heater_profile(self):
        new_dur = min(120, max(20, int(120 * self.__period_ms / self.__NORM_BASE_PERIOD)))
        tau = 1500  # ms
        self.__set_run_gas(False)
        self.__perform_reading()
        Tamb = self.__temperature()
        Ttarget = 320.0
        Tstart = Tamb + (Ttarget - Tamb) * math.exp(-self.__period_ms / tau)
        self.set_heater_profile(target_temp_c=320, duration_ms=new_dur, amb_temp_c=Tstart)

    @property
    def period_ms(self):
        return self.__period_ms 

    @period_ms.setter
    def period_ms(self, value: int):
        """
        Set the period in milliseconds.

        :param value: The period in milliseconds. Minimum 500ms.
        """
        if value < 500:
            raise ValueError("Minimum period is 500ms.")
        
        self.__period_ms = value
        self.__next_due_ms = utime.ticks_add(utime.ticks_ms(), self.__period_ms)
        self.__auto_heater_profile()

    @property
    def gas_baseline(self):
        return self.__gas_baseline
    
    @gas_baseline.setter
    def gas_baseline(self, value: int):
        """
        Set the gas baseline value.

        :param value: The gas baseline value in Ohms. range 80_000 - 200_000 Ohm
        """
        self.__gas_baseline = max(80_000, min(200_000, value))

    @property
    def gas_baseline_auto_update_ms(self):
        return self.__gas_baseline_auto_update_ms

    @gas_baseline_auto_update_ms.setter
    def gas_baseline_auto_update_ms(self, update_ms: int):
        """
        Set the gas baseline auto update interval in milliseconds.
        
        :param update_ms: The update interval in milliseconds. minimum 60_000ms (1 minute)
        """
        self.__gas_baseline_auto_update_ms = max(60_000, update_ms)

    @property
    def on_read(self):
        return self.__cb

    @on_read.setter
    def on_read(self, callback):
        self.__is_read_callback = True
        self.__on_read_iaq(callback)
        
    @property
    def on_iaq(self):
        return self.__cb
    
    @on_iaq.setter
    def on_iaq(self, callback):
        self.__is_read_callback = False
        self.__on_read_iaq(callback)
        
    def read(self, *, wait=True):
        if wait:
            now = utime.ticks_ms()
            due = self.__next_due_ms
            self.__next_due_ms = utime.ticks_add(self.__next_due_ms, self.__period_ms)
            delay = utime.ticks_diff(due, now)
            if delay > 0:
                utime.sleep_ms(delay)
            else:
                while utime.ticks_diff(self.__next_due_ms, now) <= 0:
                    self.__next_due_ms = utime.ticks_add(self.__next_due_ms, self.__period_ms)
        
        self.__set_run_gas(False)
        self.__perform_reading()
        temp, pres, humi = self.__temperature(), self.__pressure(), self.__humidity()
        
        if not self.__is_gas:
            gas = None
        else:
            self.__set_run_gas(True)
            self.__perform_reading()
            gas_ok = self.__ready_gas
            self.__set_run_gas(False)
            if gas_ok:
                gas = self.__gas()
                self.__last_valid_gas = gas
            else:
                gas = self.__last_valid_gas if (self.__last_valid_gas is not None) else None
            
        return temp, pres, humi, gas

    def iaq(self,
            *,
            temp_weighting=0.08,  
            pressure_weighting=0.02, 
            humi_weighting=0.15, 
            gas_weighting=0.75, 
            gas_ema_alpha=0.02,       # baseline EMA(Exponential Moving Average) (1-2%). Automatic Upward Learning.
            temp_baseline=25.0,
            pressure_baseline=1013.25, 
            humi_baseline=50.0,
            wait=True
            ):
        """
        IAQ (0-500): Higher numbers indicate worse conditions.
        Configuration:
        - T/H/P deviation = worse (the further away from the baseline, the higher the increase)
        - GAS decreases from baseline = worse (gas_resistance decreases -> worse up)
        Measurement:
        - T/P/H first (heater OFF) → minimizes heater effect
        - Gas is used only after FORCED + gas_valid/heat_stab testing
        Baseline:
        - In a clean environment only, slowly increase/correct by 2% EMA every 5 minutes
        """
        total_weighting = temp_weighting + pressure_weighting + humi_weighting + gas_weighting
        if abs(total_weighting - 1.0) > 0.001:
            raise ValueError("The sum of weightings must be 1.0 to keep IAQ meaningful.")
        if not (0.0 < gas_ema_alpha <= 0.2):
            raise ValueError("gas_ema_alpha should be in (0, 0.2].")
    
        if "_BME680__iaq_last_baseline_update_ms" not in self.__dict__:
            self.__iaq_last_baseline_update_ms = utime.ticks_ms()

        temp, pres, humi, gas = self.read(wait=wait)
        gas_ok = self.__ready_gas

        if gas_ok:
            now = utime.ticks_ms()
            near_env = (abs(temp - temp_baseline) <= 1.0) and (abs(humi - humi_baseline) <= 5.0)
            cleaner  = (gas >= self.__gas_baseline * 0.98)
            if near_env and cleaner and utime.ticks_diff(now, self.__iaq_last_baseline_update_ms) >= self.__gas_baseline_auto_update_ms:
                self.__gas_baseline = (1.0 - gas_ema_alpha) * self.__gas_baseline + gas_ema_alpha * gas
                self.__iaq_last_baseline_update_ms = now

        hum_bad  = min(abs(humi - humi_baseline) / max(humi_baseline * 2.0, 1e-9), 1.0) * (humi_weighting * 100.0)
        temp_bad = min(abs(temp - temp_baseline) / 10.0, 1.0) * (temp_weighting * 100.0)
        pres_bad = min(abs(pres - pressure_baseline) / 50.0, 1.0) * (pressure_weighting * 100.0)

        drop = max((self.__gas_baseline - gas) / max(self.__gas_baseline, 1e-9), 0.0)  # 0..1
        gamma = 0.8
        gas_bad = min(pow(drop, gamma), 1.0) * (gas_weighting * 100.0)

        iaq = int(min(max(5.0 * (hum_bad + temp_bad + pres_bad + gas_bad), 0.0), 500.0))
        return iaq, temp, pres, humi, gas


    def burnIn(self, threshold=0.02, count=8, timeout_sec=1800):
        self.__is_gas = True
        prev = None
        stable = 0

        self.__auto_heater_profile()
        start_ms = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start_ms) < timeout_sec * 1000:
            self.__set_run_gas(True)
            self.__perform_reading()
            gas_ok = self.__ready_gas
            self.__set_run_gas(False)
            
            if not gas_ok:
                utime.sleep_ms(200)
                continue
            
            curr = self.__gas()

            if prev is not None:
                delta = abs((curr - prev) / max(prev, 1e-9))
                stable = stable + 1 if delta <= threshold else 0
                done = (stable >= count)
                yield (done, curr, delta)
                if done:
                    return
            else:
                yield (False, curr, 1.0)

            prev = curr
            utime.sleep_ms(self.period_ms)

        yield (False, 0.0, 0.0)

    def set_heater_profile(self, target_temp_c: int = 320, duration_ms: int = 150, amb_temp_c: float = None):
        """
        Set heater target temperature/duration (single profile: res_heat_0, gas_wait_0)

        :param target_temp_c: 200-400°C recommended
        :param duration_ms  : 20-150ms recommended
        :param amb_temp_c   : Variable temperature (if none, current measurement)
        """
        if amb_temp_c is None:
            self.__perform_reading()
            amb_temp_c = self.__temperature()

        rh = self.__calc_res_heat(target_temp_c, amb_temp_c)
        gw = self.__encode_gas_wait(duration_ms)

        self.__i2c.writeto_mem(0x5A, bytes([rh]))   # res_heat_0
        self.__i2c.writeto_mem(self.__REG_GAS_WAIT_0, bytes([gw]))   # gas_wait_0

    def adjust_temperature_correction(self, delta):
        self.__temperature_correction += float(delta)

    def sealevel(self, altitude):
        self.__perform_reading()
        press = self.__pressure()
        return press / pow((1 - altitude/44330.0), 5.255), press

    def altitude(self, sealevel):
        self.__perform_reading()
        press = self.__pressure()
        return 44330.0 * (1.0 - pow(press / sealevel, 1/5.255)), press

    def __set_power_mode(self, value):
        tmp = self.__i2c.readfrom_mem(0x74, 1)[0]
        tmp &= ~0x03
        tmp |= (value & 0x03)
        self.__i2c.writeto_mem(0x74, bytes([tmp]))

    def __decode_gas_wait_ms(self, reg_val: int) -> int:
        factor = (reg_val >> 6) & 0x03
        mant   = reg_val & 0x3F
        return mant * (1 << (2 * factor))

    def __estimate_tph_time_ms(self) -> int:
        ctrl_meas = self.__i2c.readfrom_mem(0x74, 1)[0]
        ctrl_hum  = self.__i2c.readfrom_mem(0x72, 1)[0]
        osrs_t = (ctrl_meas >> 5) & 0x07
        osrs_p = (ctrl_meas >> 2) & 0x07
        osrs_h = (ctrl_hum  >> 0) & 0x07
        t_ms = 1.25 + (2.3*osrs_t) + (2.3*osrs_p + 0.575) + (2.3*osrs_h + 0.575)
        return int(t_ms + 0.5)

    def __perform_reading(self):
        if self.__i2c.readfrom_mem(self.__REG_STATUS, 1)[0] & self.__BIT_NEW_DATA:
            _ = self.__i2c.readfrom_mem(self.__REG_STATUS, 17)

        ctrl_gas1 = self.__i2c.readfrom_mem(self.__REG_CTRL_GAS1, 1)[0]
        run_gas_enabled = (ctrl_gas1 & (1 << 4)) != 0
        gas_wait_reg = self.__i2c.readfrom_mem(self.__REG_GAS_WAIT_0, 1)[0]
        gas_wait_ms = self.__decode_gas_wait_ms(gas_wait_reg) if run_gas_enabled else 0

        tph_ms = self.__estimate_tph_time_ms()
        t_est_ms = tph_ms + gas_wait_ms
        margin_ms = max(60, t_est_ms // 2)

        def _wait_once(extra_margin=0):
            self.__set_power_mode(self.__FORCED_MODE)

            t0 = utime.ticks_add(utime.ticks_ms(), 5)
            while utime.ticks_diff(t0, utime.ticks_ms()) > 0:
                st = self.__i2c.readfrom_mem(self.__REG_STATUS, 1)[0]
                if st & 0x20:  # measuring=1
                    break

            deadline = utime.ticks_add(utime.ticks_ms(), t_est_ms + margin_ms + extra_margin)
            while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
                st = self.__i2c.readfrom_mem(self.__REG_STATUS, 1)[0]
                if st & self.__BIT_NEW_DATA:  # new_data_0
                    return True
                utime.sleep_ms(3)
            return False
        
        if not _wait_once():
            if not _wait_once(extra_margin=margin_ms):
                raise OSError(f"BME680 sensor data not ready (tph={tph_ms}ms, gas_wait={gas_wait_ms}ms, est={t_est_ms}ms)")
                
        data = self.__i2c.readfrom_mem(self.__REG_STATUS, 17)

        self.__adc_pres = (data[2] << 12) | (data[3] << 4) | (data[4] >> 4)
        self.__adc_temp = (data[5] << 12) | (data[6] << 4) | (data[7] >> 4)
        self.__adc_hum = ustruct.unpack(">H", bytes(data[8:10]))[0]
        self.__adc_gas = (ustruct.unpack(">H", bytes(data[13:15]))[0] >> 6)
        self.__gas_range = data[14] & 0x0F

        var1 = (self.__adc_temp / 8.0) - (self.__par_t1 * 2.0)
        var2 = (var1 * self.__par_t2) / 2048.0
        var3 = ((var1 / 2.0) * (var1 / 2.0)) / 4096.0
        var3 = (var3 * self.__par_t3 * 16.0) / 16384.0
        self.__t_fine = int(var2 + var3)
         
        self.__ready_gas = (data[14] & self.__BITS_GAS_OK) == self.__BITS_GAS_OK

    def __es_hPa(self, T_c):
        # Magnus (over water): hPa
        return 6.112 * math.exp((17.62 * T_c) / (243.12 + T_c))

    def __compensate_rh_for_temp(self, rh_meas, temp):
        # e = RH_meas/100 * es(T_meas)
        T_meas = temp - self.__temperature_correction
        T_corr = temp
        e = (max(0.0, min(100.0, rh_meas)) / 100.0) * self.__es_hPa(T_meas)
        rh_corr = 100.0 * e / self.__es_hPa(T_corr)
        rh_corr = 0.0 if rh_corr < 0.0 else 100.0 if rh_corr > 100.0 else rh_corr
        return rh_corr

    def __temperature(self):
        return ((((self.__t_fine * 5) + 128) / 256.0) / 100.0) + self.__temperature_correction

    def __pressure(self):
        p = self.__pressure_calibration
        var1 = (self.__t_fine / 2.0) - 64000.0
        var2 = ((var1 / 4.0) * (var1 / 4.0)) / 2048.0
        var2 = (var2 * p[5]) / 4.0
        var2 = var2 + (var1 * p[4] * 2.0)
        var2 = (var2 / 4.0) + (p[3] * 65536.0)
        var1 = ((((var1 / 4.0) * (var1 / 4.0)) / 8192.0) * (p[2] * 32.0) / 8.0) + ((p[1] * var1) / 2.0)
        var1 = var1 / 262144.0
        var1 = ((32768.0 + var1) * p[0]) / 32768.0
        calc_pres = 1048576.0 - self.__adc_pres
        calc_pres = (calc_pres - (var2 / 4096.0)) * 3125.0
        calc_pres = (calc_pres / var1) * 2.0
        var1 = (p[8] * (((calc_pres / 8.0) * (calc_pres / 8.0)) / 8192.0)) / 4096.0
        var2 = ((calc_pres / 4.0) * p[7]) / 8192.0
        var3 = (((calc_pres / 256.0) ** 3) * p[9]) / 131072.0
        calc_pres += (var1 + var2 + var3 + (p[6] * 128.0)) / 16.0
        return calc_pres / 100.0  # hPa

    def __humidity(self):
        h = self.__humidity_calibration
        temp_scaled = ((self.__t_fine * 5.0) + 128.0) / 256.0
        var1 = (self.__adc_hum - (h[0] * 16.0)) - ((temp_scaled * h[2]) / 200.0)
        var2 = (h[1] * (((temp_scaled * h[3]) / 100.0) + (((temp_scaled * ((temp_scaled * h[4]) / 100.0)) / 64.0) / 100.0) + 16384.0)) / 1024.0
        var3 = var1 * var2
        var4 = (h[5] * 128.0 + ((temp_scaled * h[6]) / 100.0)) / 16.0
        var5 = ((var3 / 16384.0) * (var3 / 16384.0)) / 1024.0
        var6 = (var4 * var5) / 2.0
        calc_hum = ((((var3 + var6) / 1024.0) * 1000.0) / 4096.0) / 1000.0
        
        if abs(self.__temperature_correction) > 1e-6:
            calc_hum = self.__compensate_rh_for_temp(calc_hum, self.__temperature())
        
        return 100.0 if calc_hum > 100.0 else 0.0 if calc_hum < 0.0 else calc_hum

    def __gas(self):
        adc = int(self.__adc_gas)                 # 10-bit
        gr  = int(self.__gas_range) & 0x0F        # 0..15
        rs_err = int(self.__sw_err)

        lookup = (
            2147483647, 2147483647, 2147483647, 2147483647,
            2147483647, 2126008810, 2147483647, 2130303777,
            2147483647, 2147483647, 2143188679, 2136746228,
            2147483647, 2126008810, 2147483647, 2147483647
        )

        var1 = ((1340 + (5 * rs_err)) * lookup[gr]) >> 16
        var2 = ((adc << 15) - 16777216) + var1
        var3 = ((125000 << (15 - gr)) * var1) >> 9
        var3 += (var2 >> 1)
        gas_res_ohm = var3 // var2 if var2 != 0 else 0
        return float(gas_res_ohm)

    def __set_run_gas(self, enable: bool):
        val = (1 << 4) | 0 if enable else 0  # run_gas=[4], nb_conv=0
        if val != self.__last_ctrl_gas1:
            self.__i2c.writeto_mem(self.__REG_CTRL_GAS1, bytes([val]))
            self.__last_ctrl_gas1 = val

    def __calc_res_heat(self, target_temp_c: int, amb_temp_c: float) -> int:
        t = min(int(target_temp_c), 400)   # 400°C cap

        gh1 = float(self.__par_gh1)
        gh2 = float(self.__par_gh2)
        gh3 = float(self.__par_gh3)
        htr = float(self.__res_heat_range)   # 0..3
        htv = float(self.__res_heat_val)     # -128..127

        var1 = (gh1 / 16.0) + 49.0
        var2 = (gh2 / 32768.0) * 0.0005 + 0.00235
        var3 = gh3 / 1024.0
        var4 = var1 * (1.0 + var2 * float(t))
        var5 = var4 + (var3 * float(amb_temp_c))

        res_heat = int(3.4 * ((var5 * (4.0 / (4.0 + htr)) * (1.0 / (1.0 + (htv * 0.002)))) - 25.0))
        return max(0, min(res_heat, 255))

    def __encode_gas_wait(self, ms: int) -> int:
        """gas_wait_x(self.__REG_GAS_WAIT_0..) encoding (1..4032ms, 0xFF=max)"""
        dur = int(ms)
        if dur >= 0xFC0:  # >= 4032 ms
            return 0xFF
        factor = 0
        while dur > 0x3F and factor < 3:
            dur //= 4
            factor += 1
        return int(dur + (factor << 6))

    def __on_read_iaq(self, callback):
        if self.__tmr:
            try:
                self.__tmr.deinit()
            except Exception:
                pass
            self.__tmr = None

        self.__cb = callback
        self.__scheduled = False

        if callback is None:
            return

        self.__tmr = machine.Timer()
        self.__tmr.init(period=self.__period_ms, mode=machine.Timer.PERIODIC, callback=self.__tmr_isr)

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
        if self.__poll_busy:
            return
        
        self.__poll_busy = True
        try:
            if self.__is_read_callback:
                self.__cb(*self.read(wait=False))
            else:
                self.__cb(*self.iaq(wait=False))
        finally:

            self.__poll_busy = False
