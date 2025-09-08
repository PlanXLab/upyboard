import gc
import usys
import utime
import uos
import uselect

import network
import machine
import micropython
import rp2

import utools
from utools import ANSIEC


__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"


try:
    micropython.alloc_emergency_exception_buf(128)
except Exception:
    pass

@micropython.native
def get_sys_info() -> tuple:
    freq = machine.freq()

    try:
        machine.Pin(43, machine.Pin.IN)
        TEMP_ADC  = 8
    except ValueError: # rp2350a (pico2w)
        TEMP_ADC  = 4
                         
    raw = machine.ADC(TEMP_ADC).read_u16()
    temp = 27 - ((raw * 3.3 / 65535) - 0.706) / 0.001721
    
    return freq, temp


@micropython.native
def get_mem_info() -> tuple:
    gc.collect()
    
    free = gc.mem_free()
    used = gc.mem_alloc()
    total = free + used
    
    return free, used, total

@micropython.native
def get_fs_info(path: str = '/') -> tuple:
    stats = uos.statvfs(path)
    block_size = stats[0]
    total_blocks = stats[2]
    free_blocks = stats[3]

    total = block_size * total_blocks
    free = block_size * free_blocks
    used = total - free
    usage_pct = round(used / total * 100, 2)

    return total, used, free, usage_pct


class WifiManager:    
    def __init__(self):
        self.__iface = network.WLAN(network.STA_IF)
        if not self.__iface.active():
            self.__iface.active(True)
           
    def scan(self) -> list[tuple[str,int,int,int]]:
        return self.__iface.scan()

    def available_ssids(self) -> list[str]:
        aps = self.scan()
        ssids = set()
        for ap in aps:
            ssid = ap[0].decode('utf-8', 'ignore')
            if ssid:
                ssids.add(ssid)
        return list(ssids)

    def connect(self, ssid: str, password: str, timeout: float = 20.0) -> bool:
        if self.__iface.isconnected():
            return True

        self.__iface.connect(ssid, password)
        start = utime.ticks_ms()
        while not self.__iface.isconnected():
            if utime.ticks_diff(utime.ticks_ms(), start) > int(timeout * 1000):
                return False
            utime.sleep_ms(200)
        return True

    def disconnect(self) -> None:
        if self.__iface.isconnected():
            self.__iface.disconnect()
            utime.sleep_ms(100)
    
    def ifconfig(self) -> tuple | None:
        if not self.is_connected:
            return None
        return self.__iface.ifconfig()

    @property
    def is_connected(self) -> bool:
        return self.__iface.isconnected()

    @property
    def ip(self) -> str | None:
        if not self.is_connected:
            return None
        return self.__iface.ifconfig()[0]


class Led(machine.Pin):    
    def __init__(self):
        super().__init__("WL_GPIO0", machine.Pin.OUT)


class Button:    
    @staticmethod
    def read() -> bool:
        return rp2.bootsel_button() == 1


class Din:
    LOW         = 0
    HIGH        = 1
    
    PULL_DOWN   = machine.Pin.PULL_DOWN
    PULL_UP     = machine.Pin.PULL_UP
    OPEN_DRAIN  = machine.Pin.OPEN_DRAIN
    CB_FALLING  = machine.Pin.IRQ_FALLING
    CB_RISING   = machine.Pin.IRQ_RISING
    CB_BOTH     = machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING
        
    def __init__(self, pins: list[int]|tuple[int, ...]):
        if not pins:
            raise ValueError("At least one pin must be provided")
            
        self._pins = list(pins)
        n = len(self._pins)
        
        try:
            self._din = [machine.Pin(pin, machine.Pin.IN) for pin in self._pins]
        except Exception as e:
            raise OSError(f"Failed to initialize GPIO pins: {e}")
        
        self._pull_config = [None] * n 
        
        self._user_callbacks = [None] * n
        self._edge_config = [0] * n  # No edge detection by default
        self._measurement_enabled = [False] * n
        self._irq_handlers = [None] * n
        self._debounce_us = [0] * n 
        
    def deinit(self) -> None:
        try:
            for i in range(len(self._pins)):
                self._measurement_enabled[i] = False
                if self._irq_handlers[i] is not None:
                    try:
                        self._din[i].irq(handler=None)
                        self._irq_handlers[i] = None
                    except:
                        pass
        except:
            pass

    def __getitem__(self, idx: int|slice) -> "_DinView":
        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._pins))))
            return Din._DinView(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._pins)):
                raise IndexError("Pin index out of range")
            return Din._DinView(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        return len(self._pins)


    @property
    def pins(self) -> list:
        return self._din


    def _setup_irq(self, idx: int) -> None:
        if (self._user_callbacks[idx] is not None and self._edge_config[idx] != 0 and self._measurement_enabled[idx]):
            
            if self._irq_handlers[idx] is None:
                def irq_handler(pin_obj):
                    pin_num = self._pins[idx]
                    current_value = pin_obj.value()
                    rising = bool(current_value)
                    
                    try:
                        micropython.schedule(
                            lambda _: self._user_callbacks[idx](pin_num, rising), 0
                        )
                    except RuntimeError:
                        try:
                            self._user_callbacks[idx](pin_num, rising)
                        except:
                            pass
                
                self._irq_handlers[idx] = irq_handler
                self._din[idx].irq(trigger=self._edge_config[idx], handler=irq_handler)
        else:
            if self._irq_handlers[idx] is not None:
                self._din[idx].irq(handler=None)
                self._irq_handlers[idx] = None

    @staticmethod
    def _get_pull_list(parent, indices: list[int]) -> list[int|None]:
        return [parent._pull_config[i] for i in indices]

    @staticmethod
    def _set_pull_all(parent, pull: int|None, indices: list[int]) -> None:
        for i in indices:
            parent._pull_config[i] = pull
            parent._din[i].init(mode=machine.Pin.IN, pull=pull)
            
    @staticmethod
    def _get_value_list(parent, indices: list[int]) -> list[int]:
        return [parent._din[i].value() for i in indices]

    @staticmethod
    def _get_callback_list(parent, indices: list[int]) -> list[callable]:
        return [parent._user_callbacks[i] for i in indices]

    @staticmethod
    def _set_callback_all(parent, callback: callable, indices: list[int]) -> None:
        for i in indices:
            parent._user_callbacks[i] = callback
            parent._setup_irq(i)

    @staticmethod
    def _get_edge_list(parent, indices: list[int]) -> list[int]:
        return [parent._edge_config[i] for i in indices]

    @staticmethod
    def _set_edge_all(parent, edge: int, indices: list[int]) -> None:
        for i in indices:
            parent._edge_config[i] = edge
            parent._setup_irq(i)

    @staticmethod
    def _get_measurement_list(parent, indices: list[int]) -> list[bool]:
        return [parent._measurement_enabled[i] for i in indices]

    @staticmethod
    def _set_measurement_all(parent, enabled: bool, indices: list[int]) -> None:
        for i in indices:
            parent._measurement_enabled[i] = enabled
            parent._setup_irq(i)

    @staticmethod
    def _get_debounce_list(parent, indices: list[int]) -> list[int]:
        return [parent._debounce_us[i] for i in indices]

    @staticmethod
    def _set_debounce_all(parent, debounce_us: int, indices: list[int]) -> None:
        for i in indices:
            parent._debounce_us[i] = debounce_us

    class _DinView:
        def __init__(self, parent: "Din", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Din._DinView":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Din._DinView(self._parent, selected_indices)
            else:
                return Din._DinView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        @property
        def pull(self) -> list[int|None]:
            return Din._get_pull_list(self._parent, self._indices)

        @pull.setter
        def pull(self, pull_type: int|None|list[int|None]):
            if isinstance(pull_type, (list, tuple)):
                if len(pull_type) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, pull in zip(self._indices, pull_type):
                    Din._set_pull_all(self._parent, pull, [i])
            else:
                Din._set_pull_all(self._parent, pull_type, self._indices)
      
        @property
        def value(self) -> list[int]:
            return Din._get_value_list(self._parent, self._indices)

        @property
        def callback(self) -> list[callable]:
            return Din._get_callback_list(self._parent, self._indices)

        @callback.setter
        def callback(self, fn: callable | list[callable]):
            if callable(fn) or fn is None:
                Din._set_callback_all(self._parent, fn, self._indices)
            else:
                if len(fn) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, callback in zip(self._indices, fn):
                    if not (callable(callback) or callback is None):
                        raise TypeError("Each callback must be callable or None")
                    self._parent._user_callbacks[i] = callback
                    self._parent._setup_irq(i)

        @property
        def edge(self) -> list[int]:
            return Din._get_edge_list(self._parent, self._indices)

        @edge.setter
        def edge(self, edge_type: int):
            Din._set_edge_all(self._parent, edge_type, self._indices)

        @property
        def debounce_us(self) -> list[int]:
            return Din._get_debounce_list(self._parent, self._indices)

        @debounce_us.setter
        def debounce_us(self, us: int):
            Din._set_debounce_all(self._parent, us, self._indices)

        @property
        def measurement(self) -> list[bool]:
            return Din._get_measurement_list(self._parent, self._indices)

        @measurement.setter
        def measurement(self, enabled: bool | list[bool]):
            if isinstance(enabled, bool):
                Din._set_measurement_all(self._parent, enabled, self._indices)
            else:
                if len(enabled) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, en in zip(self._indices, enabled):
                    self._parent._measurement_enabled[i] = en
                    self._parent._setup_irq(i)

        def measure_pulse_width(self, level: int, timeout_ms: int = 1000) -> int:
            if len(self._indices) != 1:
                raise ValueError("Pulse width measurement only works with single pin. Use individual pin access like din[0].measure_pulse_width() instead of din[:].measure_pulse_width()")
            
            pin = self._parent._din[self._indices[0]]
            
            return machine.time_pulse_us(pin, level, timeout_ms * 1000)            


class Dout:
    LOW         = 0
    HIGH        = 1
    LOGIC_HIGH  = True   # Active High: HIGH = active
    LOGIC_LOW   = False  # Active Low: LOW = active
    PULL_DOWN   = machine.Pin.PULL_DOWN
    PULL_UP     = machine.Pin.PULL_UP
    OPEN_DRAIN  = machine.Pin.OPEN_DRAIN

    def __init__(self, pins: list[int]|tuple[int, ...]):
        if not pins:
            raise ValueError("At least one pin must be provided")
        
        self._pins = list(pins)
        n = len(self._pins)
        
        try:
            self._dout = [machine.Pin(pin, machine.Pin.IN) for pin in self._pins]
        except Exception as e:
            raise OSError(f"Failed to initialize GPIO pins: {e}")
        
        self._pull_config = [None] * n
        self._active_logic = [None] * n 

    def deinit(self) -> None:
        try:
            for i, pin in enumerate(self._dout):
                if self._active_logic[i] == Dout.LOGIC_HIGH:
                    pin.value(0)  # Active high: LOW = inactive
                else:
                    pin.value(1)  # Active low: HIGH = inactive
            
            utime.sleep_ms(50)
            
            for pin in self._dout:
                pin.init(mode=machine.Pin.IN, pull=machine.Pin.PULL_DOWN)
        except:
            pass

    def __getitem__(self, idx: int|slice) -> "_DoutView":
        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._pins))))
            return Dout._DoutView(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._pins)):
                raise IndexError("Pin index out of range")
            return Dout._DoutView(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        return len(self._pins)

    @property
    def pins(self) -> list:
        return self._dout

    @staticmethod
    def _get_pull_list(parent, indices: list[int]) -> list[int|None]:
        return [parent._pull_config[i] for i in indices]

    @staticmethod
    def _set_pull_all(parent, pull: int|None, indices: list[int]) -> None:
        for i in indices:
            parent._pull_config[i] = pull
            parent._dout[i].init(mode=machine.Pin.OUT, pull=pull)

    @staticmethod
    def _get_active_list(parent, indices: list[int]) -> list[bool]:
        return [parent._active_logic[i] for i in indices]

    @staticmethod
    def _set_active_all(parent, active_logic: bool, indices: list[int]) -> None:
        for i in indices:
            if parent._active_logic[i] is None:
                parent._dout[i].init(mode=machine.Pin.OUT)
                parent._active_logic[i] = active_logic
                 
                if active_logic == Dout.LOGIC_HIGH:
                    parent._dout[i].value(0)
                else:
                    parent._dout[i].value(1)
            else:
                parent._active_logic[i] = active_logic

    @staticmethod
    def _get_value_list(parent, indices: list[int]) -> list[int]:
        result = []
        for i in indices:
            physical = parent._dout[i].value()
            if parent._active_logic[i] == Dout.LOGIC_HIGH:
                logical = physical
            else:
                logical = 1 - physical
            result.append(logical)
        return result

    @staticmethod
    def _set_value_all(parent, logical_value: int, indices: list[int]) -> None:
        for i in indices:
            if parent._active_logic[i] == Dout.LOGIC_HIGH:
                physical_value = logical_value
            else:
                physical_value = 1 - logical_value
            parent._dout[i].value(physical_value)

    @staticmethod
    def _set_value_list(parent, logical_values: list[int], indices: list[int]) -> None:
        if len(logical_values) != len(indices):
            raise ValueError(f"Value list length ({len(logical_values)}) must match pin count ({len(indices)})")
        for i, logical_value in zip(indices, logical_values):
            if parent._active_logic[i] == Dout.LOGIC_HIGH:
                physical_value = logical_value
            else:
                physical_value = 1 - logical_value
            parent._dout[i].value(physical_value)

    @staticmethod
    def _toggle_all(parent, indices: list[int]) -> None:
        for i in indices:
            physical = parent._dout[i].value()
            if parent._active_logic[i] == Dout.LOGIC_HIGH:
                logical = physical
            else:
                logical = 1 - physical
            
            new_logical = 1 - logical
            
            if parent._active_logic[i] == Dout.LOGIC_HIGH:
                new_physical = new_logical
            else:
                new_physical = 1 - new_logical
            
            parent._dout[i].value(new_physical)

    class _DoutView:
        def __init__(self, parent: "Dout", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Dout._DoutView":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Dout._DoutView(self._parent, selected_indices)
            else:
                return Dout._DoutView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        @property
        def active(self) -> list[bool]:
            return Dout._get_active_list(self._parent, self._indices)

        @active.setter
        def active(self, logic_type: bool | list[bool]):
            if isinstance(logic_type, (list, tuple)):
                if len(logic_type) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, logic in zip(self._indices, logic_type):
                    Dout._set_active_all(self._parent, logic, [i])
            else:
                Dout._set_active_all(self._parent, logic_type, self._indices)

        @property
        def pull(self) -> list[int|None]:
            return Dout._get_pull_list(self._parent, self._indices)

        @pull.setter
        def pull(self, pull_type: int|None|list[int|None]):
            if isinstance(pull_type, (list, tuple)):
                if len(pull_type) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, pull in zip(self._indices, pull_type):
                    Dout._set_pull_all(self._parent, pull, [i])
            else:
                Dout._set_pull_all(self._parent, pull_type, self._indices)

        @property
        def value(self) -> list[int]:
            return Dout._get_value_list(self._parent, self._indices)

        @value.setter
        def value(self, val: int | list[int]):
            if isinstance(val, (list, tuple)):
                Dout._set_value_list(self._parent, val, self._indices)
            else:
                Dout._set_value_all(self._parent, val, self._indices)

        def toggle(self) -> None:
            Dout._toggle_all(self._parent, self._indices)

        @property
        def physical_value(self) -> list[int]:
            return [self._parent._dout[i].value() for i in self._indices]


class Adc:    
    def __init__(self, pins: list[int]|tuple[int, ...]):
        if not pins:
            raise ValueError("At least one pin must be provided")
        
        valid_pins = {26, 27, 28}
        invalid_pins = set(pins) - valid_pins
        if invalid_pins:
            raise ValueError(f"Invalid ADC pins: {invalid_pins}. Only GPIO 26, 27, 28 support ADC on RP2350.")
        
        self._pins = list(pins)
        n = len(self._pins)
        
        try:
            self._adc = [machine.ADC(machine.Pin(pin)) for pin in self._pins]
        except Exception as e:
            raise OSError(f"Failed to initialize ADC pins: {e}")
        
        self._user_callbacks = [None] * n
        self._period_ms = [20] * n  # 0 = disabled
        self._measurement_enabled = [False] * n
        self._timers = [None] * n

    def deinit(self) -> None:
        try:
            for i in range(len(self._pins)):
                self._measurement_enabled[i] = False
                if self._timers[i] is not None:
                    try:
                        self._timers[i].deinit()
                        self._timers[i] = None
                    except:
                        pass
        except:
            pass

    def __getitem__(self, idx: int|slice) -> "_AdcView":
        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._pins))))
            return Adc._AdcView(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._pins)):
                raise IndexError("ADC channel index out of range")
            return Adc._AdcView(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        return len(self._pins)

    def _setup_timer(self, idx: int) -> None:
        if (self._user_callbacks[idx] is not None and 
            self._period_ms[idx] > 0 and 
            self._measurement_enabled[idx]):
            
            if self._timers[idx] is None:
                def timer_callback(timer):
                    pin_num = self._pins[idx]
                    raw = self._adc[idx].read_u16()
                    voltage = round(raw * (3.3 / 65535), 3)
                    
                    try:
                        micropython.schedule(lambda _: self._user_callbacks[idx](pin_num, voltage, raw), 0)
                    except RuntimeError:
                        try:
                            self._user_callbacks[idx](pin_num, voltage, raw)
                        except:
                            pass
                
                self._timers[idx] = machine.Timer()
                self._timers[idx].init(mode=machine.Timer.PERIODIC, period=self._period_ms[idx], callback=timer_callback)
        else:
            if self._timers[idx] is not None:
                self._timers[idx].deinit()
                self._timers[idx] = None
            
    @staticmethod
    @micropython.native
    def _get_value_list(parent, indices: list[int]) -> list[float]:
        result = []
        for i in indices:
            raw = parent._adc[i].read_u16()
            voltage = raw * (3.3 / 65535)
            result.append(round(voltage, 3))
        return result

    @staticmethod
    def _get_raw_value_list(parent, indices: list[int]) -> list[int]:
        return [parent._adc[i].read_u16() for i in indices]

    @staticmethod
    def _get_callback_list(parent, indices: list[int]) -> list[callable]:
        return [parent._user_callbacks[i] for i in indices]

    @staticmethod
    def _set_callback_all(parent, callback: callable, indices: list[int]) -> None:
        for i in indices:
            parent._user_callbacks[i] = callback
            parent._setup_timer(i)

    @staticmethod
    def _get_period_list(parent, indices: list[int]) -> list[int]:
        """Get period in milliseconds for specified ADC channels."""
        return [parent._period_ms[i] for i in indices]

    @staticmethod
    def _set_period_all(parent, period_ms: int, indices: list[int]) -> None:
        for i in indices:
            parent._period_ms[i] = period_ms
            parent._setup_timer(i)

    @staticmethod
    def _get_measurement_list(parent, indices: list[int]) -> list[bool]:
        return [parent._measurement_enabled[i] for i in indices]

    @staticmethod
    def _set_measurement_all(parent, enabled: bool, indices: list[int]) -> None:
        for i in indices:
            parent._measurement_enabled[i] = enabled
            parent._setup_timer(i)

    class _AdcView:
        def __init__(self, parent: "Adc", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Adc._AdcView":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Adc._AdcView(self._parent, selected_indices)
            else:
                return Adc._AdcView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        @property
        def value(self) -> list[float]:
            return Adc._get_value_list(self._parent, self._indices)

        @property
        def raw_value(self) -> list[int]:
            return Adc._get_raw_value_list(self._parent, self._indices)

        @property
        def callback(self) -> list[callable]:
            return Adc._get_callback_list(self._parent, self._indices)

        @callback.setter
        def callback(self, fn: callable | list[callable]):
            if callable(fn) or fn is None:
                Adc._set_callback_all(self._parent, fn, self._indices)
            else:
                if len(fn) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, callback in zip(self._indices, fn):
                    if not (callable(callback) or callback is None):
                        raise TypeError("Each callback must be callable or None")
                    self._parent._user_callbacks[i] = callback
                    self._parent._setup_timer(i)

        @property
        def period_ms(self) -> list[int]:
            return Adc._get_period_list(self._parent, self._indices)

        @period_ms.setter
        def period_ms(self, ms: int):
            Adc._set_period_all(self._parent, ms, self._indices)

        @property
        def measurement(self) -> list[bool]:
            return Adc._get_measurement_list(self._parent, self._indices)

        @measurement.setter
        def measurement(self, enabled: bool | list[bool]):
            if isinstance(enabled, bool):
                Adc._set_measurement_all(self._parent, enabled, self._indices)
            else:
                if len(enabled) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, en in zip(self._indices, enabled):
                    self._parent._measurement_enabled[i] = en
                    self._parent._setup_timer(i)


class Pwm:
    __FULL_RANGE     = 65_535
    __MICROS_PER_SEC = 1_000_000

    def __init__(self, pins: list[int]|tuple[int, ...]):
        if not pins:
            raise ValueError("At least one pin must be provided")
        
        self._pins = list(pins)
        n = len(self._pins)
        
        try:
            self._pwm = [machine.PWM(machine.Pin(pin)) for pin in self._pins]
        except Exception as e:
            raise OSError(f"Failed to initialize PWM pins: {e}")
        
        self._freq_hz = [1000] * n
        self._duty_pct = [0] * n
        self._enabled = [True] * n
        
        for i in range(n):
            self._pwm[i].freq(self._freq_hz[i])
            self._pwm[i].duty_u16(0)  # Start with 0% duty for safety

    def deinit(self) -> None:
        try:
            for pwm in self._pwm:
                pwm.duty_u16(0)
            
            utime.sleep_ms(50)  # Allow hardware to settle
            
            for pwm in self._pwm:
                pwm.deinit()
        except:
            pass

    def __getitem__(self, idx: int|slice) -> "_PwmView":
        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._pins))))
            return Pwm._PwmView(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._pins)):
                raise IndexError("PWM channel index out of range")
            return Pwm._PwmView(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        return len(self._pins)

    @staticmethod
    def _get_freq_list(parent, indices: list[int]) -> list[int]:
        return [parent._freq_hz[i] for i in indices]

    @staticmethod
    def _set_freq_all(parent, freq: int, indices: list[int]) -> None:
        for i in indices:
            parent._freq_hz[i] = freq
            parent._pwm[i].freq(freq)

    @staticmethod
    @micropython.native
    def _get_period_list(parent, indices: list[int]) -> list[int]:
        return [Pwm.__MICROS_PER_SEC // parent._freq_hz[i] for i in indices]

    @staticmethod
    def _set_period_all(parent, period_us: int, indices: list[int]) -> None:
        freq = Pwm.__MICROS_PER_SEC // period_us
        for i in indices:
            parent._freq_hz[i] = freq
            parent._pwm[i].freq(freq)

    @staticmethod
    def _get_duty_list(parent, indices: list[int]) -> list[int]:
        return [parent._duty_pct[i] for i in indices]

    @staticmethod
    def _set_duty_all(parent, duty_pct: int, indices: list[int]) -> None:
        for i in indices:
            parent._duty_pct[i] = duty_pct
            if parent._enabled[i]:
                raw_duty = int(duty_pct * Pwm.__FULL_RANGE / 100)
                parent._pwm[i].duty_u16(raw_duty)
            else:
                parent._pwm[i].duty_u16(0)

    @staticmethod
    def _get_duty_u16_list(parent, indices: list[int]) -> list[int]:
        return [parent._pwm[i].duty_u16() for i in indices]

    @staticmethod
    def _set_duty_u16_all(parent, duty_raw: int, indices: list[int]) -> None:
        duty_pct = round(duty_raw * 100 / Pwm.__FULL_RANGE)
        for i in indices:
            parent._duty_pct[i] = duty_pct
            if parent._enabled[i]:
                parent._pwm[i].duty_u16(duty_raw)
            else:
                parent._pwm[i].duty_u16(0)

    @staticmethod
    def _get_duty_us_list(parent, indices: list[int]) -> list[int]:
        result = []
        for i in indices:
            period_us = Pwm.__MICROS_PER_SEC // parent._freq_hz[i]
            duty_us = int(parent._duty_pct[i] * period_us / 100)
            result.append(duty_us)
        return result

    @staticmethod
    @micropython.native
    def _set_duty_us_all(parent, duty_us: int, indices: list[int]) -> None:
        for i in indices:
            period_us = Pwm.__MICROS_PER_SEC // parent._freq_hz[i]
            duty_pct = int(duty_us * 100 / period_us)
            duty_pct = max(0, min(100, duty_pct))
            parent._duty_pct[i] = duty_pct
            
            if parent._enabled[i]:
                duty_raw = int(duty_us * Pwm.__FULL_RANGE / period_us)
                duty_raw = max(0, min(Pwm.__FULL_RANGE, duty_raw))
                parent._pwm[i].duty_u16(duty_raw)
            else:
                parent._pwm[i].duty_u16(0)

    @staticmethod
    def _get_enabled_list(parent, indices: list[int]) -> list[bool]:
        return [parent._enabled[i] for i in indices]

    @staticmethod
    def _set_enabled_all(parent, enabled: bool, indices: list[int]) -> None:
        for i in indices:
            parent._enabled[i] = enabled
            if enabled:
                raw_duty = int(parent._duty_pct[i] * Pwm.__FULL_RANGE / 100)
                parent._pwm[i].duty_u16(raw_duty)
            else:
                parent._pwm[i].duty_u16(0)

    class _PwmView:
        def __init__(self, parent: "Pwm", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Pwm._PwmView":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Pwm._PwmView(self._parent, selected_indices)
            else:
                return Pwm._PwmView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        @property
        def freq(self) -> list[int]:
            return Pwm._get_freq_list(self._parent, self._indices)

        @freq.setter
        def freq(self, hz: int | list[int]):
            if isinstance(hz, (list, tuple)):
                if len(hz) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, f in zip(self._indices, hz):
                    Pwm._set_freq_all(self._parent, f, [i])
            else:
                Pwm._set_freq_all(self._parent, hz, self._indices)

        @property
        def period(self) -> list[int]:
            return Pwm._get_period_list(self._parent, self._indices)

        @period.setter
        def period(self, us: int | list[int]):
            if isinstance(us, (list, tuple)):
                if len(us) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, p in zip(self._indices, us):
                    Pwm._set_period_all(self._parent, p, [i])
            else:
                Pwm._set_period_all(self._parent, us, self._indices)

        @property
        def duty(self) -> list[int]:
            return Pwm._get_duty_list(self._parent, self._indices)

        @duty.setter
        def duty(self, pct: int | list[int]):
            if isinstance(pct, (list, tuple)):
                if len(pct) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, p in zip(self._indices, pct):
                    Pwm._set_duty_all(self._parent, p, [i])
            else:
                Pwm._set_duty_all(self._parent, pct, self._indices)

        @property
        def duty_u16(self) -> list[int]:
            return Pwm._get_duty_u16_list(self._parent, self._indices)

        @duty_u16.setter
        def duty_u16(self, raw: int | list[int]):
            if isinstance(raw, (list, tuple)):
                if len(raw) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, r in zip(self._indices, raw):
                    Pwm._set_duty_u16_all(self._parent, r, [i])
            else:
                Pwm._set_duty_u16_all(self._parent, raw, self._indices)

        @property
        def duty_us(self) -> list[int]:
            return Pwm._get_duty_us_list(self._parent, self._indices)

        @duty_us.setter
        def duty_us(self, us: int | list[int]):
            if isinstance(us, (list, tuple)):
                if len(us) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, u in zip(self._indices, us):
                    Pwm._set_duty_us_all(self._parent, u, [i])
            else:
                Pwm._set_duty_us_all(self._parent, us, self._indices)

        @property
        def enabled(self) -> list[bool]:
            return Pwm._get_enabled_list(self._parent, self._indices)

        @enabled.setter
        def enabled(self, flag: bool | list[bool]):
            if isinstance(flag, (list, tuple)):
                if len(flag) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, en in zip(self._indices, flag):
                    Pwm._set_enabled_all(self._parent, en, [i])
            else:
                Pwm._set_enabled_all(self._parent, flag, self._indices)


def __i2c_detect_check(id, scl, sda, show):
    i2c = machine.I2C(id=id, scl=machine.Pin(scl), sda=machine.Pin(sda))
    return i2c.scan()


def i2cdetect(show: bool = False) -> list | None:
    I2C_PIN_MAP = {
        0: ((0, 1), (4, 5), (8, 9), (12, 13), (16, 17), (20, 21)),
        1: ((2, 3), (6, 7), (10, 11), (14, 15), (18, 19), (26, 27)),
    }
    
    for id, pairs in I2C_PIN_MAP.items():
        for sda, scl in pairs:
            devices =  __i2c_detect_check(id, scl, sda, show)
            if devices == []:
                continue
            print(f"I2C{id} on SDA={sda}, SCL={scl}: {len(devices)} device(s) found")
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
    def __init__(self, scl:int, sda:int, addr:int, freq:int=400_000):     
        I2C_PIN_MAP = {
            0: {'sda': {0,4,8,12,16,20}, 'scl': {1,5,9,13,17,21}},
            1: {'sda': {2,6,10,14,18,26}, 'scl': {3,7,11,15,19,27}},
        }
        
        bus = None
        for id, pins in I2C_PIN_MAP.items():
            if sda in pins['sda'] and scl in pins['scl']:
                bus = id
                break
        if bus is None:
            raise ValueError(f"Invalid I2C pins: SDA={sda}, SCL={scl}")

        self.__addr = addr
        self.__i2c = machine.I2C(bus, scl=machine.Pin(scl), sda=machine.Pin(sda), freq=freq)

    def read_u8(self, reg:int) -> int:
        data = self.__i2c.readfrom_mem(self.__addr, reg, 1)
        return data[0]

    def read_u16(self, reg:int, *, little_endian:bool=True) -> int:
        data = self.__i2c.readfrom_mem(self.__addr, reg, 2)
        order = 'little' if little_endian else 'big'
        return int.from_bytes(data, order)

    def write_u8(self, reg:int, val:int) -> None:
        self.__i2c.writeto_mem(self.__addr, reg, bytes([val & 0xFF]))

    def write_u16(self, reg:int, val:int, *, little_endian:bool=True) -> None:
        order = 'little' if little_endian else 'big'
        self.__i2c.writeto_mem(self.__addr, reg, (val & 0xFFFF).to_bytes(2, order))

    def readfrom(self, nbytes:int, *, stop:bool=True) -> bytes:
        return self.__i2c.readfrom(self.__addr, nbytes, stop)

    def readinto(self, buf:bytearray, *, stop:bool=True) -> int:
        return self.__i2c.readinto(self.__addr, buf, stop)

    def readfrom_mem(self, reg:int, nbytes:int, *, addrsize:int=8) -> bytes:
        return self.__i2c.readfrom_mem(self.__addr, reg, nbytes, addrsize=addrsize)

    def readfrom_mem_into(self, reg:int, buf:bytearray, *, addrsize:int=8) -> int:
        return self.__i2c.readfrom_mem_into(self.__addr, reg, buf, addrsize=addrsize)

    def writeto(self, buf:bytes, *, stop:bool=True) -> int:
        return self.__i2c.writeto(self.__addr, buf, stop)

    def writeto_mem(self, reg:int, buf:bytes, *, addrsize:int=8) -> int:
        return self.__i2c.writeto_mem(self.__addr, reg, buf, addrsize=addrsize)


class ReplSerial:
    def __init__(self, timeout:float|None=None, *, bufsize:int=512, poll_ms:int=10):
        self._timeout   = timeout
        self._stdin     = usys.stdin.buffer
        self._stdout    = usys.stdout
        self._buf       = utools.RingBuffer(bufsize)
        self._scheduled = False
        self._tmr = machine.Timer(-1)
        self._tmr.init(period=poll_ms, mode=machine.Timer.PERIODIC, callback=self.__tick)

    def __tick(self, t):
        if not self._scheduled:
            self._scheduled = True
            try:
                micropython.schedule(self.__pump, None)
            except RuntimeError:
                self._scheduled = False

    def __pump(self, _):
        try:
            # read 1 byte at a time as long as data is ready
            while uselect.select([self._stdin], [], [], 0)[0]:
                b = self._stdin.read(1)
                if not b:
                    break
                self._buf.put(b)
        except Exception:
            pass
        finally:
            self._scheduled = False

    def __wait(self, deadline_ms:int):
        while not self._buf.avail():
            if deadline_ms is not None and utime.ticks_diff(deadline_ms, utime.ticks_ms()) <= 0:
                return
            dur = None if deadline_ms is None else max(0,
                utime.ticks_diff(deadline_ms, utime.ticks_ms())) / 1000
            uselect.select([self._stdin], [], [], dur)

    @property
    def timeout(self) -> float|None:
        return self._timeout
    
    @timeout.setter
    def timeout(self, value:float|None):
        self._timeout = value

    def read(self, size:int=1) -> bytes:
        if size <= 0:
            return b''
        dl = None if self._timeout is None else utime.ticks_add(utime.ticks_ms(), int(self._timeout*1000))
        self.__wait(dl)
        return self._buf.get(size)

    def read_until(self, expected:bytes=b'\r', max_size:int|None=None) -> bytes:
        if self._timeout == 0:
            if max_size and self._buf.avail() >= max_size:
                return self._buf.get(max_size)
            
            data = self._buf.get_until(expected, max_size)
            return data or b''

        deadline = None
        if self._timeout is not None:
            deadline = utime.ticks_add(utime.ticks_ms(), int(self._timeout * 1000))

        while True:
            if max_size and self._buf.avail() >= max_size:
                return self._buf.get(max_size)

            data = self._buf.get_until(expected, max_size)
            if data is not None:
                return data

            if deadline is not None:
                if utime.ticks_diff(deadline, utime.ticks_ms()) <= 0:
                    return b''

            # wait for incoming data
            self.__wait(deadline)

    def write(self, data:bytes) -> int:
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("data must be bytes or bytearray")
        return self._stdout.write(data)

    def close(self):
        self._tmr.deinit()


def input(prompt:str="") -> str:
    @micropython.native
    def __char_width(ch: str) -> int:
        return 1 if len(ch.encode('utf-8')) == 1 else 2

    repl_in = usys.stdin.buffer
    repl_out = usys.stdout
    
    BACKSPACE = (0x08, 0x7F)
    ENTER = (0x0D, 0x0A)
        
    if prompt:
        repl_out.write(prompt.encode('utf-8'))

    buf = []
    pos = 0
    push = None
    
    while True:
        if push is not None:
            b = push
            push = None
        else:
            while not uselect.select([repl_in], [], [], 0)[0]:
                pass
            b = repl_in.read(1)
            if not b:
                continue
        byte = b[0]

        if byte in ENTER:
            repl_out.write(b"\n")
            while uselect.select([repl_in], [], [], 0)[0]:
                nxt = repl_in.read(1)
                if not nxt:
                    continue
                if nxt[0] in ENTER:
                    continue
                push = nxt
                break
            break

        if byte == 0x1B:
            seq = repl_in.read(2)
            # left key
            if seq == b'[D' and pos > 0:
                w = __char_width(buf[pos-1])
                repl_out.write(f"\x1b[{w}D".encode())
                pos -= 1
            # right key
            elif seq == b'[C' and pos < len(buf):
                w = __char_width(buf[pos])
                repl_out.write(f"\x1b[{w}C".encode())
                pos += 1
            # Delete (ESC [ 3 ~)
            elif seq == b'[3' and repl_in.read(1) == b'~' and pos < len(buf):
                buf.pop(pos)
                repl_out.write(b"\x1b[K")
                tail = ''.join(buf[pos:])
                if tail:
                    repl_out.write(tail.encode('utf-8'))
                    ws = sum(__char_width(c) for c in tail)
                    repl_out.write(f"\x1b[{ws}D".encode())
            continue

        if byte in BACKSPACE and pos > 0:
            pos -= 1
            removed = buf.pop(pos)
            w = __char_width(removed)
            repl_out.write(f"\x1b[{w}D".encode())
            repl_out.write(b"\x1b[K")
            tail = ''.join(buf[pos:])
            if tail:
                repl_out.write(tail.encode('utf-8'))
                ws = sum(__char_width(c) for c in tail)
                repl_out.write(f"\x1b[{ws}D".encode())
            continue

        first = byte
        if first < 0x80:
            seq = b
        elif (first & 0xE0) == 0xC0:
            seq = b + repl_in.read(1)
        elif (first & 0xF0) == 0xE0:
            seq = b + repl_in.read(2)
        elif (first & 0xF8) == 0xF0:
            seq = b + repl_in.read(3)
        else:
            continue

        try:
            ch = seq.decode('utf-8')
        except UnicodeError:
            continue

        buf.insert(pos, ch)
        w = __char_width(ch)
        tail = ''.join(buf[pos+1:])

        repl_out.write(seq)
        if tail:
            repl_out.write(tail.encode('utf-8'))
            ws = sum(__char_width(c) for c in tail)
            repl_out.write(f"\x1b[{ws}D".encode())
        pos += 1


    return ''.join(buf)
