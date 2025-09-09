import gc
import uos
import usys
import utime
import ustruct
import machine
 
from micropython import const
import xbee

import utools
from utools import ANSIEC

def get_sys_info() -> tuple:
    freq = 40_000_000 # Default frequency of EFR32MG
    temp = xbee.atcmd('TP')
    
    return freq, temp

def get_mem_info() -> tuple:
    gc.collect()
    
    free = gc.mem_free()
    used = gc.mem_alloc()
    total = free + used
    
    return free, used, total

def get_fs_info(path='/') -> tuple:
    stats = uos.statvfs(path)
    block_size = stats[0]
    total_blocks = stats[2]
    free_blocks = stats[3]

    total = block_size * total_blocks
    free = block_size * free_blocks
    used = total - free
    usage_pct = round(used / total * 100, 2)

    return total, used, free, usage_pct



XNODE_LED_PIN               = const('D9')
XNODE_SUPPLY_VOLTAGE_PIN    = const('D2')
XNODE_I2C_ID1_SCL_PIN       = const('D1')
XNODE_I2C_ID1_SDA_PIN       = const('D11')


class Din:
    LOW         = 0
    HIGH        = 1
    PULL_DOWN   = machine.Pin.PULL_DOWN
    PULL_UP     = machine.Pin.PULL_UP
    
    def __init__(self, pin:str, *, pull:int|None=None):
        self.__pin = machine.Pin(pin, machine.Pin.IN, pull=pull)

    def value(self):
        return not self.__pin.value()


class Dout:
    LOW         = 0
    HIGH        = 1
    PULL_DOWN   = machine.Pin.PULL_DOWN
    PULL_UP     = machine.Pin.PULL_UP
    
    def __init__(self, pin:str, *, pull:int|None=None, value:int|None=LOW):
        self.__pin = machine.Pin(pin, machine.Pin.OUT, pull=pull, value=value)

    def value(self):
        return not self.__pin.value()

    def value(self, n):
        self.__pin.value(n)

    def toggle(self):
        self.__pin.toggle()

def Wdt(timeout:int) -> machine.WDT:
    return machine.WDT(0, timeout)


def i2cdetect(show:bool=False) -> list | None:
    i2c = machine.I2C(1)  # only I2C bus 1 is available on XNode
    devices = i2c.scan()

    if not show:
        return devices
    else:
        print("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f")
        for i in range(0, 8):
            print("{:02x}:".format(i*16), end='')
            for j in range(0, 16):
                address = i * 16 + j
                if address in devices:
                    print(ANSIEC.FG.BRIGHT_YELLOW + " {:02x}".format(address) + ANSIEC.OP.RESET, end='')
                else:
                    print(" --", end='')
            print()


class I2c:
    def __init__(self, addr:int, freq:int=400_000):
        self.__addr = addr
        self.__i2c = machine.I2C(1, freq=freq)

    def read_u8(self, reg: int) -> int:
        data = self.__i2c.readfrom_mem(self.__addr, reg, 1)
        return data[0]

    def read_u16(self, reg: int, *, little_endian: bool = True) -> int:
        data = self.__i2c.readfrom_mem(self.__addr, reg, 2)
        order = 'little' if little_endian else 'big'
        return int.from_bytes(data, order)

    def write_u8(self, reg: int, val: int) -> None:
        self.__i2c.writeto_mem(self.__addr, reg, bytes([val & 0xFF]))

    def write_u16(self, reg: int, val: int, *, little_endian: bool = True) -> None:
        order = 'little' if little_endian else 'big'
        self.__i2c.writeto_mem(self.__addr, reg, val.to_bytes(2, order))

    def readfrom(self, nbytes: int, *, stop: bool = True) -> bytes:
        return self.__i2c.readfrom(self.__addr, nbytes, stop)

    def readinto(self, buf: bytearray, *, stop: bool = True) -> int:
        return self.__i2c.readinto(self.__addr, buf, stop)

    def readfrom_mem(self, reg: int, nbytes: int, *, addrsize: int = 8) -> bytes:
        return self.__i2c.readfrom_mem(self.__addr, reg, nbytes, addrsize=addrsize)

    def readfrom_mem_into(self, reg: int, buf: bytearray, *, addrsize: int = 8) -> int:
        return self.__i2c.readfrom_mem_into(self.__addr, reg, buf, addrsize=addrsize)

    def writeto(self, buf: bytes, *, stop: bool = True) -> int:
        return self.__i2c.writeto(self.__addr, buf, stop)

    def writeto_mem(self, reg: int, buf: bytes, *, addrsize: int = 8) -> int:
        return self.__i2c.writeto_mem(self.__addr, reg, buf, addrsize=addrsize)


class ReplSerial:
    def __init__(self, timeout:int|float|None=None):
        self.timeout = timeout
    
    @property
    def timeout(self) -> int|float|None:
        return self.__timeout
    
    @timeout.setter
    def timeout(self, n:int|float|None):
        self.__timeout = n

    def read(self, size:int=1) -> bytes: 
        if self.timeout is None:
            assert size > 0, "size must be greater than 0"
            return usys.stdin.buffer.read(size)
        elif self.timeout is not None and self.timeout == 0:
            return usys.stdin.buffer.read(-1)
        elif self.timeout is not None and self.timeout > 0:
            rx_buffer = b''
            t0 = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), t0) / 1000 < self.timeout:
                b = usys.stdin.buffer.read(-1)
                if b:
                    rx_buffer += b
                    if len(rx_buffer) >= size:
                        break
            return rx_buffer
        
    def read_until(self, expected:bytes=b'\n', size:int|None=None) -> bytes:
        rx_buffer = bytearray()
        expected_len = len(expected)

        t0 = utime.ticks_ms() if (self.timeout is not None and self.timeout > 0) else None 
        while True:
            if t0 is not None:
                ellipsis = utime.ticks_diff(utime.ticks_ms(), t0) / 1000
                if ellipsis >= self.timeout:
                    break  # Timeout

            try:
                b = usys.stdin.buffer.read(-1)
                if self.timeout is not None and self.timeout == 0:
                    return b if b else b''
            except Exception as e:
                return
                
            if not b: 
                continue

            rx_buffer.extend(b)

            if size is not None and len(rx_buffer) >= size:
                return bytes(rx_buffer[:size])

            if len(rx_buffer) >= expected_len and rx_buffer[-expected_len:] == expected:
                return bytes(rx_buffer)
        
        return bytes(rx_buffer) # Timeout occurred, return what's read so far
                    
    def write(self, data:bytes) -> int:
        assert isinstance(data, bytes), "data must be a byte type"
        
        ret = usys.stdout.write(data)
                
        return ret


class Led:
    def __init__(self):
        self.__led = Dout(XNODE_LED_PIN, value=Dout.HIGH)

    def on(self):
        self.__led.value(0)
        
    def off(self):
        self.__led.value(1)

    def toggle(self):
        self.__led.toggle()

    def value(self):
        return not self.__led.value()

    def value(self, n):
        self.__led.value(not n)


class SupplyVoltage(machine.ADC):
    def __init__(self):
        super().__init__(XNODE_SUPPLY_VOLTAGE_PIN)

    def read(self):
        return round(((super().read() * 3.3 / 4095) * (3.2/2)), 1)


class Illuminance:
    BH1750_ADDR = const(0x23)
    
    POWER_OFF       = const(0x00)
    POWER_ON        = const(0x01)
    POWER_RESET     = const(0x07)    
    CONT_HIGH_RES   = 0x10  # 1 lx / count, 120 ms typ.
    CONT_HIGH_RES_2 = 0x11  # 0.5 lx / count, 120 ms typ.
    CONT_LOW_RES    = 0x13  # 4 lx / count, 16 ms  typ.

    
    def __init__(self, *, mode=CONT_HIGH_RES):
        self.__i2c = I2c(Illuminance.BH1750_ADDR)

        self.init(mode)

    def init(self, mode):
        self.__i2c.writeto(bytes([self.POWER_ON]))
        self.__i2c.writeto(bytes([self.POWER_RESET]))
        self.__i2c.writeto(bytes([mode])) 
 
    def deinit(self):
        self.__i2c.writeto(bytes([self.POWER_OFF]))  # Power off

    def read(self):      
        data = self.__i2c.readfrom(2)
        return (data[0] << 8) | data[1]


class Tphg:
    BME680_ADDR = const(0x77)
    
    FORCED_MODE = const(0x01)
    SLEEP_MODE = const(0x00)
        
    def __set_power_mode(self, value):
        tmp = self.__i2c.readfrom_mem(0x74, 1)[0] 
        tmp &= ~0x03
        tmp |= value
        self.__i2c.writeto_mem(0x74, bytes([tmp]))
  
    def __perform_reading(self):
        self.__set_power_mode(Tphg.FORCED_MODE)
                
        gas_measuring = True
        timeout_time = utime.ticks_add(utime.ticks_ms(), 100)
        while gas_measuring and utime.ticks_diff(timeout_time, utime.ticks_ms()) > 0:
            data = self.__i2c.readfrom_mem(0x1D, 1)
            gas_measuring = data[0] & 0x20 != 0
            utime.sleep_ms(5)

        ready = False
        timeout_time = utime.ticks_add(utime.ticks_ms(), 100)
        while not ready and utime.ticks_diff(timeout_time, utime.ticks_ms()) > 0:
            data = self.__i2c.readfrom_mem(0x1D, 1)
            ready = data[0] & 0x80 != 0
            utime.sleep_ms(5)

        if not ready:
            raise OSError("BME680 sensor data not ready")
        
        data = self.__i2c.readfrom_mem(0x1D, 17)
        self._adc_pres = ((data[2] * 4096) + (data[3] * 16) + (data[4] / 16))
        self._adc_temp = ((data[5] * 4096) + (data[6] * 16) + (data[7] / 16))
        self._adc_hum = ustruct.unpack(">H", bytes(data[8:10]))[0]
        self._adc_gas = int(ustruct.unpack(">H", bytes(data[13:15]))[0] / 64)
        self._gas_range = data[14] & 0x0F
            
        var1 = (self._adc_temp / 8) - (self._temp_calibration[0] * 2)
        var2 = (var1 * self._temp_calibration[1]) / 2048
        var3 = ((var1 / 2) * (var1 / 2)) / 4096
        var3 = (var3 * self._temp_calibration[2] * 16) / 16384
        self._t_fine = int(var2 + var3)

    def __temperature(self):
        return ((((self._t_fine * 5) + 128) / 256) / 100) + self._temperature_correction
            
    def __pressure(self):
        var1 = (self._t_fine / 2) - 64000
        var2 = ((var1 / 4) * (var1 / 4)) / 2048
        var2 = (var2 * self._pressure_calibration[5]) / 4
        var2 = var2 + (var1 * self._pressure_calibration[4] * 2)
        var2 = (var2 / 4) + (self._pressure_calibration[3] * 65536)
        var1 = ((((var1 / 4) * (var1 / 4)) / 8192) * (self._pressure_calibration[2] * 32) / 8) + ((self._pressure_calibration[1] * var1) / 2)
        var1 = var1 / 262144
        var1 = ((32768 + var1) * self._pressure_calibration[0]) / 32768
        calc_pres = 1048576 - self._adc_pres
        calc_pres = (calc_pres - (var2 / 4096)) * 3125
        calc_pres = (calc_pres / var1) * 2
        var1 = (self._pressure_calibration[8] * (((calc_pres / 8) * (calc_pres / 8)) / 8192)) / 4096
        var2 = ((calc_pres / 4) * self._pressure_calibration[7]) / 8192
        var3 = (((calc_pres / 256) ** 3) * self._pressure_calibration[9]) / 131072
        calc_pres += (var1 + var2 + var3 + (self._pressure_calibration[6] * 128)) / 16
        return calc_pres / 100

    def __humidity(self):
        temp_scaled = ((self._t_fine * 5) + 128) / 256
        var1 = (self._adc_hum - (self._humidity_calibration[0] * 16)) - ((temp_scaled * self._humidity_calibration[2]) / 200)
        var2 = (self._humidity_calibration[1] * (((temp_scaled * self._humidity_calibration[3]) / 100) + 
                (((temp_scaled * ((temp_scaled * self._humidity_calibration[4]) / 100)) / 64) / 100) + 16384)) / 1024
        var3 = var1 * var2
        var4 = self._humidity_calibration[5] * 128
        var4 = (var4 + ((temp_scaled * self._humidity_calibration[6]) / 100)) / 16
        var5 = ((var3 / 16384) * (var3 / 16384)) / 1024
        var6 = (var4 * var5) / 2
        calc_hum = ((((var3 + var6) / 1024) * 1000) / 4096) / 1000
        return 100 if calc_hum > 100 else 0 if calc_hum < 0 else calc_hum
    
    def __gas(self):
        lookup_table_1 = {
            0: 2147483647.0, 1: 2126008810.0, 2: 2130303777.0, 3: 2147483647.0,
            4: 2143188679.0, 5: 2136746228.0, 6: 2126008810.0, 7: 2147483647.0
        }

        lookup_table_2 = {
            0: 4096000000.0, 1: 2048000000.0, 2: 1024000000.0, 3: 512000000.0,
            4: 255744255.0, 5: 127110228.0, 6: 64000000.0, 7: 32258064.0,
            8: 16016016.0, 9: 8000000.0, 10: 4000000.0, 11: 2000000.0,
            12: 1000000.0, 13: 500000.0, 14: 250000.0, 15: 125000.0
        }

        var1 = ((1340 + (5 * self._sw_err)) * lookup_table_1.get(self._gas_range, 2147483647.0)) / 65536 
        var2 = ((self._adc_gas * 32768) - 16777216) + var1
        var3 = (lookup_table_2.get(self._gas_range, 125000.0) * var1) / 512 
        return ((var3 + (var2 / 2)) / var2)

    def __init__(self, temp_weighting=0.10,  pressure_weighting=0.05, humi_weighting=0.20, gas_weighting=0.65, gas_ema_alpha=0.1, temp_baseline=23.0,  pressure_baseline=1013.25, humi_baseline=45.0, gas_baseline=450_000):
        self.__i2c = I2c(Tphg.BME680_ADDR)
        
        self.__i2c.writeto_mem(0xE0, bytes([0xB6]))                         # Soft reset
        utime.sleep_ms(5)        
          
        self.__set_power_mode(Tphg.SLEEP_MODE)
        
        t_calibration = self.__i2c.readfrom_mem(0x89, 25)
        t_calibration += self.__i2c.readfrom_mem(0xE1, 16)
        
        self._sw_err = (self.__i2c.readfrom_mem(0x04, 1)[0] & 0xF0) / 16

        calibration = [float(i) for i in list(ustruct.unpack("<hbBHhbBhhbbHhhBBBHbbbBbHhbb", bytes(t_calibration[1:39])))]
        self._temp_calibration = [calibration[x] for x in [23, 0, 1]]
        self._pressure_calibration = [calibration[x] for x in [3, 4, 5, 7, 8, 10, 9, 12, 13, 14]]
        self._humidity_calibration = [calibration[x] for x in [17, 16, 18, 19, 20, 21, 22]]
        #self._gas_calibration = [calibration[x] for x in [25, 24, 26]]                        # res_heat_0, idac_heat_0, gas_wait_0
        
        self._humidity_calibration[1] *= 16
        self._humidity_calibration[1] += self._humidity_calibration[0] % 16
        self._humidity_calibration[0] /= 16

        self.__i2c.writeto_mem(0x72, bytes([0b001]))                        # Humidity oversampling x1
        self.__i2c.writeto_mem(0x74, bytes([(0b010 << 5) | (0b011 << 2)]))  # Temperature oversampling x2, Pressure oversampling x4
        self.__i2c.writeto_mem(0x75, bytes([0b001 << 2]))                   # Filter coefficient 3 (only to temperature and pressure data)
        
        self.__i2c.writeto_mem(0x50, bytes([0x1F]))                         # idac_heat_0
        self.__i2c.writeto_mem(0x5A, bytes([0x73]))                         # res_heat_0
        self.__i2c.writeto_mem(0x64, bytes([0x64]))                         # gas_wait_0 is 100ms (1ms ~ 4032ms, 20ms ~ 30ms are neccessary)
                
        self.__i2c.writeto_mem(0x71, bytes([(0b1 << 4) | (0b0000)]))        # run_gas(enable gas measurements), nv_conv (index of heater set-point 0)
        utime.sleep_ms(50)
        
        self._temperature_correction = -10
        self._t_fine = None
        self._adc_pres = None
        self._adc_temp = None
        self._adc_hum = None
        self._adc_gas = None
        self._gas_range = None
        
        self.temp_weighting = temp_weighting
        self.pressure_weighting = pressure_weighting
        self.humi_weighting = humi_weighting
        self.gas_weighting = gas_weighting
        self.gas_ema_alpha = gas_ema_alpha
        self.temp_baseline = temp_baseline
        self.pressure_baseline = pressure_baseline
        self.humi_baseline = humi_baseline
        self.gas_baseline = gas_baseline
        
        total_weighting = temp_weighting + pressure_weighting + humi_weighting + gas_weighting
        if abs(total_weighting - 1.0) > 0.001:
             raise ValueError("The sum of weightings is not equal to 1.  This may lead to unexpected IAQ results.")
            
    def set_temperature_correction(self, value):
        self._temperature_correction += value

    def read(self, gas=False):
        self.__perform_reading()
        if not gas:
            return self.__temperature(), self.__pressure(), self.__humidity(), None
        else:
            return self.__temperature(), self.__pressure(), self.__humidity(), self.__gas()
        
    def sealevel(self, altitude):
        self.__perform_reading()
        press = self.__pressure()  
        return press / pow((1-altitude/44330), 5.255), press
        
    def altitude(self, sealevel): 
        self.__perform_reading()
        press = self.__pressure()
        return 44330 * (1.0-pow(press/sealevel,1/5.255)), press

    def iaq(self):
        self.__perform_reading()
        temp = self.__temperature()
        pres = self.__pressure()
        humi = self.__humidity()
        gas = self.__gas()

        hum_offset = humi - self.humi_baseline
        hum_score = (1 - min(max(abs(hum_offset) / (self.humi_baseline * 2), 0), 1)) * (self.humi_weighting * 100)

        temp_offset = temp - self.temp_baseline
        temp_score = (1- min(max(abs(temp_offset) / 10, 0), 1)) * (self.temp_weighting * 100)

        self.gas_baseline = (self.gas_ema_alpha * gas) + ((1 - self.gas_ema_alpha) * self.gas_baseline) # EMA for gas_baseline
        gas_offset = self.gas_baseline - gas
        gas_score = (gas_offset / self.gas_baseline) * (self.gas_weighting * 100)
        gas_score = max(0, min(gas_score, self.gas_weighting * 100))
        
        pressure_offset = pres - self.pressure_baseline
        pressure_score =  (1 - min(max(abs(pressure_offset) / 50, 0), 1)) * (self.pressure_weighting * 100)

        iaq_score = round((hum_score + temp_score + gas_score + pressure_score) * 5)

        return iaq_score, temp, pres, humi, gas
        
    def burnIn(self, threshold=0.01, count=10, timeout_sec=180): 
        self.__perform_reading()
        prev_gas = self.__gas()
        
        counter  = 0
        timeout_time = utime.ticks_us()  
        interval_time = utime.ticks_us()
                 
        while True:
            if utime.ticks_diff(utime.ticks_us(), interval_time) > 1_000_000:
                self.__perform_reading()
                curr_gas = self.__gas()
                gas_change = abs((curr_gas - prev_gas) / prev_gas)
                yield False, curr_gas, gas_change

                counter  = counter + 1 if gas_change <= threshold else 0

                if counter > count:
                    yield True, curr_gas, 0.0
                    break
                else:
                    yield False, curr_gas, gas_change
                    
                prev_gas = curr_gas
                interval_time = utime.ticks_us()
            
            if utime.ticks_diff(utime.ticks_us(), timeout_time) > timeout_sec * 1_000_000:
                yield False, 0.0, 0.0
                break
