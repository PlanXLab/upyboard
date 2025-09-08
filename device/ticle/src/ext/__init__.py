import sys
import math
import utime
import ustruct
import array

import gc
import machine
import micropython
import rp2

import utools
import ticle


__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"


__all__ = (
    "Button", "Relay", "ServoMotor", "SR04", "KY022", "HD44780_PCF8574", "VL53L0X", "BME680", "WS2812Matrix"
)

_lazy_map = {
    "Button": (".button", "Button"),
    "Relay": (".relay", "Relay"),
    "ServoMotor": (".servo_motor", "ServoMotor"),
    "SR04": (".sr04", "SR04"),
    "KY022": (".ky022", "KY022"),
    "HD44780_PCF8574": (".hd44780_pcf8574", "HD44780_PCF8574"),
    "VL53L0X": (".vl53l0x", "VL53L0X"),
    "BME680": (".bme680", "BME680"),
    "WS2812Matrix": (".ws2812", "WS2812Matrix"),
}

def __getattr__(name):
    try:
        rel_mod, attr = _lazy_map[name]
    except KeyError:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from None
    
    fullname = __name__ + rel_mod
    __import__(fullname)
    mod = sys.modules[fullname]
        
    obj = getattr(mod, attr)
    globals()[name] = obj
    return obj

def __dir__():
    return sorted(list(globals().keys()) + list(_lazy_map.keys()))