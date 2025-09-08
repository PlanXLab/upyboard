import json

import utime
import math
import urandom

import utools
import machine
import ticle
import ticle.ext as ext

__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"


__all__ = (
    "DistanceScanner", "WS2812Matrix_Effect", "BT_Audio_Amplifier", "ServoFnd", "UltrasonicGrid"
)

_lazy_map = {
    "DistanceScanner": (".distance_scanner", "DistanceScanner"),
    "WS2812Matrix_Effect": (".ws2812_matrix_effect", "WS2812Matrix_Effect"),
    "BT_Audio_Amplifier": (".bt_audio_amp", "BT_Audio_Amplifier"),
    "ServoFnd": (".servo_fnd", "ServoFnd"),
    "UltrasonicGrid": (".ultrasonic_grid", "UltrasonicGrid"),
}

def __getattr__(name):
    try:
        rel_mod, attr = _lazy_map[name]
    except KeyError:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from None

    mod = __import__(__name__ + rel_mod, fromlist=[attr])
    obj = getattr(mod, attr)
    globals()[name] = obj
    return obj

def __dir__():
    return sorted(list(globals().keys()) + list(_lazy_map.keys()))