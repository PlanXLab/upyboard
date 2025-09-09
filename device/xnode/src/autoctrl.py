import utime
import ustruct
import machine 

from micropython import const
from . import Din, Dout, I2c

class Dio:
    LOW = const(0)
    HIGH = const(1)

    IN = machine.Pin.IN
    OUT = machine.Pin.OUT    
    PULL_UP = machine.Pin.PULL_UP
    PULL_DOWN = machine.Pin.PULL_DOWN

    class Device:
        RELAY = const(0) 
        PWM = const(1)

    P_RELAYS = {0:'D0', 1:'D6', 2:'D5'} 

    @staticmethod
    def Relays(value=LOW):
        return [Dout(Dio.P_RELAYS[i], value=value) for i in range(len(Dio.P_RELAYS))]

    @staticmethod
    def P18(): #Only Input of ActiveHigh(5V ~ 6V) Device(ex: GasDetector). It has a built-in divider resistor that halves the 12V.
        return Din('D7')
    
    @staticmethod
    def P17(): #Only Input of ActiveLow(GND) Device(ex:PIR, LimitSiwtch ...)
        return Din('P2') #D12
    
    @staticmethod
    def P8(mode, *, pull=None, value=LOW): #Operation 3V3 IN/OUT
        if mode == Dio.IN:
            p = machine.Pin('P0', mode, pull=pull)
        else:
            p = machine.Pin('P0', mode, pull=pull, value=value)
        return p
    
    @staticmethod
    def P23(mode, *, pull=None, value=LOW): #Operation 3V3 IN/OUT
        if mode == Dio.IN:
            p = machine.Pin('D4', mode, pull=pull)
        else:
            p = machine.Pin('D4', mode, pull=pull, value=value)
        return p


def __doorlock_open(self):
    ret = self.is_opened()
    if not ret:
        self.work()
    return not ret

def __doorlock_close(self):
    ret = self.is_opened()
    if ret:
        self.work()
    return ret

def __doorlock_is_opened(self):
    current = self.__feedback.value()
    return (current == 1) if self.__active_low else (current == 0)


class DoorLock:
    def __init__(self, relay, *, feedback=None, active_low=True): #feedback is only Input (ActiveHigh). Dio Pin object (ex P17)
        self.__doorlock = relay
        if feedback:
            self.__feedback = feedback
            self.__active_low = active_low
            setattr(DoorLock, 'open', __doorlock_open)
            setattr(DoorLock, 'close', __doorlock_close)
            setattr(DoorLock, 'is_opened', __doorlock_is_opened)

    def work(self):
        self.__doorlock.on()
        utime.sleep_ms(800)
        self.__doorlock.off()
        utime.sleep_ms(800)


class PCA9685:
    ADDR = const(0x40)
    
    def __init__(self, freq=100):
        self.__i2c = I2c(PCA9685.ADDR)

        self.__i2c.writeto_mem(0x00, bytes([0x06])) # Mode 1, reset(0x06)
        utime.sleep_us(10)                
        self.__i2c.writeto_mem(0x00, bytes([0xA0])) # Mode 1, restart(0x80) | auto increment(0x20)
        self.__i2c.writeto_mem(0x26, ustruct.pack('<HH', 4096, 0)) #Enable A(0, 1) <-- LED8_ON_L(0x26), HIGH
        self.__i2c.writeto_mem(0x2A, ustruct.pack('<HH', 4096, 0)) #Enable B(2, 3) <-- LED9_ON_L(0x2A), HIGH        
        self.__i2c.writeto_mem(0x06, ustruct.pack('<HHHHHHHH', 0, 4096, 0, 4096, 0, 4096, 0, 4096)) #ch(0..4) is output 0 <-- LED0_L(0x06) + 4 * ch, LOW
        
        self.freq(freq)
                        
    def freq(self, freq):
        prescale = round(25000000.0 / (4096.0 * freq)) - 1
        prescale = 3 if prescale < 3 else 255 if prescale > 255 else prescale
        old_mode = self.__i2c.readfrom_mem(0x40, 0X00, 1)[0] # Mode 1 read
        self.__i2c.writeto_mem(0x00, bytes([(old_mode & 0x7F) | 0x10])) # Mode 1, ~restart(0x80) | sleep(0x10)
        self.__i2c.writeto_mem(0xFE, bytes([prescale])) # set prescale
        self.__i2c.writeto_mem(0x00, bytes([old_mode])) # Mode 1 restore 
        utime.sleep_us(500)

    def pwm(self, ch, on, off):
        self.__i2c.writeto_mem(0x06 + 4 * ch,  ustruct.pack('<HH', on, off))
        
    def duty(self, ch, value):
        if value == 0:
            self.pwm(ch, 0, 4096)
        elif value == 100:
            self.pwm(ch, 4096, 0)
        else:
            self.pwm(ch, 0, int((value - 1) * (4095 - 1) / (99 - 1) + 1))


class GasDetector: #ND-102D (red(12V), black(GND), white(P18))
    def __init__(self):
        self.__pin = DIO.P18()

    def read(self):
        return self.__pin.value()


class GasBreaker: #SV-20H (red(PWM), black(PWM))    
    def __init__(self, red, black):        
        self.__red = red
        self.__black = black

        self.__pwm = Pwm()
    
    def init(self):
        self.close(True)
    
    def open(self):
        self.__pwm.duty(self.__red, 100)
        self.__pwm.duty(self.__black, 0)
    
    def close(self, init=False):
        self.__pwm.duty(self.__red, 0)
        self.__pwm.duty(self.__black, 100)
    
    def stop(self):
        self.__pwm.duty(self.__red, 0)
        self.__pwm.duty(self.__black, 0)
    

__FAN   = const(0)
__LIGHT = const(1)

def __FanLight(id, type, ch, value=0):    
    class __pwm(Pwm):
        FAN_TBL = {0:0, 1:40, 2:55, 3:70, 4:85, 5:100}
        LIGHT_TBL = {0:0, 1:2, 2:6, 3:20, 4:40, 5:100}
        DUMMY_TBL = {0:0, 1:0, 2:0, 3:0, 4:0, 5:0}
                
        def __init__(self, id, ch):
            super().__init__()
            self.__ch = ch
            self.__level = self.FAN_TBL if id == __FAN else self.LIGHT_TBL if id == __LIGHT else self.DUMMY_TBL
            self.__pos = 0
            
        def on(self):            
            self.duty(self.__ch, self.__level[5])

        def off(self):            
            self.duty(self.__ch, self.__level[0])

        def change(self, i):
            self.__pos = i
            self.duty(self.__ch, self.__level[self.__pos])
            
        def state(self):
            return self.__pos
    
    if type == DIO.Device.RELAY:
        return  Relay(DIO.P_RELAY[ch], value)
    elif type == DIO.Device.PWM:
        return __pwm(id, ch)

def Fan(type, ch, value=0):
    return __FanLight(__FAN, type, ch, value)
    
def Light(type, ch, value=0):
    return __FanLight(__LIGHT, type, ch, value)