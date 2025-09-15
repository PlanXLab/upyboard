__version__ = "1.0.0"
__author__ = "PlanX Lab Development Team"

from . import (
    utime, array,
    machine, micropython,
    ticle
)


class SR04:
    def __init__(self, sensor_configs: list[tuple[int, int]], *, 
                 temp_c: float = 20.0, R: float = 25.0, Q: float = 4.0, 
                 period_ms: int = 50):

        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._temp_c))))
            return SR04._SR04View(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._temp_c)):
                raise IndexError("Sensor index out of range")
            return SR04._SR04View(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        return len(self._temp_c)

    def deinit(self) -> None:
        try:
            for i in range(len(self)):
                self._measurement_enabled[i] = 0
                self._measurement_active[i] = 0
            
            try:
                self._stop_timer()
            except Exception:
                pass
            
            try:
                self._trig.deinit()
                self._echo.deinit()
            except Exception:
                pass
        except Exception:
            pass

    @property
    def period_ms(self) -> int:
        return self._period_ms

    @period_ms.setter
    def period_ms(self, ms: int):
        if ms < 10:
            raise ValueError("Period must be at least 10ms")
        self._period_ms = int(ms)
        
        if self._timer_active:
            self._stop_timer()
            has_active = any(self._measurement_enabled[i] and self._nonblocking[i] for i in range(len(self)))
            if has_active:
                self._start_timer()

    def _cm_per_us(self, temp: float) -> float:
        speed_ms = 331.3 + 0.606 * temp  # Speed in m/s
        speed_cm_us = (speed_ms * 100.0) / 1_000_000  # Convert to cm/µs
        return speed_cm_us / 2.0  # Divide by 2 for round-trip

    def _trigger(self, idx: int):
        self._trig[idx].value = 0  # Ensure clean LOW state
        utime.sleep_us(2)    # Brief settling time
        self._trig[idx].value = 1  # Set HIGH
        utime.sleep_us(10)   # 10us trigger pulse
        self._trig[idx].value = 0  # Return to LOW

    def _kalman1d(self, idx: int, z: float, dt: float = 0.06) -> float:
        # Prediction step
        self._x[idx] += self._v[idx] * dt
        P = self._P[idx]
        P[0][0] += dt * (2 * P[1][0] + dt * P[1][1]) + self._Q[idx]
        P[0][1] += dt * P[1][1]
        P[1][0] += dt * P[1][1]

        # Update step
        innovation = z - self._x[idx]  # Measurement residual
        S = P[0][0] + self._R[idx]     # Innovation covariance
        K0 = P[0][0] / S               # Kalman gain for position
        K1 = P[1][0] / S               # Kalman gain for velocity

        # State update
        self._x[idx] += K0 * innovation
        self._v[idx] += K1 * innovation

        # Covariance update
        P[0][0] -= K0 * P[0][0]
        P[0][1] -= K0 * P[0][1]
        P[1][0] -= K1 * P[0][0]
        P[1][1] -= K1 * P[0][1]

        return self._x[idx]

    def _read_single(self, idx: int) -> float | None:
        return self._measure_single_sensor(idx)
    
    def _safe_call_callback(self, idx: int, distance: float | None):
        callback = self._user_callbacks[idx]
        if callback:
            try:
                micropython.schedule(lambda _: callback(self._trig_pins[idx], distance), 0)
            except RuntimeError:
                try:
                    callback(self._trig_pins[idx], distance)
                except Exception:
                    pass

    def _measure_single_sensor(self, idx: int, timeout_us: int = 38000) -> float | None:
        if self._nonblocking[idx]:
            return None

        echo =self._echo.pins[idx]
        self._trigger(idx)
        
        try:
            duration_us = machine.time_pulse_us(echo, 1, timeout_us)
        except Exception:
            return None
        
        if duration_us < 0:
            return None
        
        speed_factor = self._cm_per_us(self._temp_c[idx])
        raw_distance = duration_us * speed_factor
        
        if not (2.0 <= raw_distance <= 400.0):
            return None
        
        filtered_distance = self._kalman1d(idx, raw_distance)
        return max(2.0, min(filtered_distance, 400.0))

    def _manage_timer(self):
        has_active_sensors = any(
            self._measurement_enabled[i] and self._nonblocking[i] 
            for i in range(len(self))
        )
        
        if has_active_sensors and not self._timer_active:
            self._start_timer()
        elif not has_active_sensors and self._timer_active:
            self._stop_timer()

    def _timer_callback(self, timer):
        for i in range(len(self)):
            if (not self._nonblocking[i] or 
                not self._measurement_enabled[i] or 
                self._user_callbacks[i] is None):
                continue
            
            if self._measurement_active[i] == 0:
                self._start_measurement_for_sensor(i)

    def _start_measurement_for_sensor(self, idx: int):
        try:
            self._measurement_active[idx] = 1
            self._trigger(idx)
            
            echo_pin = machine.Pin(self._echo_pins[idx], machine.Pin.IN)
            duration_us = machine.time_pulse_us(echo_pin, 1, 30_000)
            
            if duration_us > 0:
                speed_factor = self._cm_per_us(self._temp_c[idx])
                raw_distance = duration_us * speed_factor
                
                if 2.0 <= raw_distance <= 400.0:
                    filtered_distance = self._kalman1d(idx, raw_distance)
                    distance = max(2.0, min(filtered_distance, 400.0))
                    self._safe_call_callback(idx, distance)
                else:
                    self._safe_call_callback(idx, None)
            else:
                self._safe_call_callback(idx, None)
                
        except Exception:
            self._safe_call_callback(idx, None)
        finally:
            self._measurement_active[idx] = 0

    def _start_timer(self):
        if not self._timer_active:
            self._timer.init(period=self._period_ms, mode=machine.Timer.PERIODIC, callback=self._timer_callback)
            self._timer_active = True

    def _stop_timer(self):
        if self._timer_active:
            self._timer.deinit()
            self._timer_active = False
        
        # Reset measurement states
        for i in range(len(self)):
            self._measurement_active[i] = 0

    class _SR04View:
        @property
        def period_ms(self) -> int:
            return self._parent.period_ms

        @period_ms.setter
        def period_ms(self, ms: int):
            self._parent.period_ms = ms

        def __init__(self, parent: "SR04", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "SR04._SR04View":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return SR04._SR04View(self._parent, selected_indices)
            else:
                return SR04._SR04View(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        def _get_values(self) -> list[int|None]:
            results = []
            for idx in self._indices:
                value = self._parent._read_single(idx)
                if value is not None:
                    results.append(int(round(value)))
                else:
                    results.append(None)
            return results
        
        def reset_filter(self):
            for i in self._indices:
                self._parent._x[i] = 0.0
                self._parent._v[i] = 0.0
                self._parent._P[i] = [[1.0, 0.0], [0.0, 1.0]]

        @property
        def measurement(self) -> list[bool]:
            return [bool(self._parent._measurement_enabled[i]) for i in self._indices]

        @measurement.setter  
        def measurement(self, enable: bool | list[bool]):
            if isinstance(enable, bool):
                enable_val = 1 if enable else 0
                for i in self._indices:
                    if enable and self._parent._nonblocking[i]:
                        if self._parent._user_callbacks[i] is not None:
                            self._parent._measurement_enabled[i] = enable_val
                        else:
                            print(f"Warning: Sensor {i} has no callback, skipping")
                    elif not enable:
                        self._parent._measurement_enabled[i] = 0
                        self._parent._measurement_active[i] = 0
            else:
                if len(enable) != len(self._indices):
                    raise ValueError("list length must match number of sensors")
                for i, en in zip(self._indices, enable):
                    enable_val = 1 if en else 0
                    if en and self._parent._nonblocking[i]:
                        if self._parent._user_callbacks[i] is not None:
                            self._parent._measurement_enabled[i] = enable_val
                        else:
                            print(f"Warning: Sensor {i} has no callback, skipping")
                    elif not en:
                        self._parent._measurement_enabled[i] = 0
                        self._parent._measurement_active[i] = 0
            
            self._parent._manage_timer()

        @property
        def filter_states(self) -> list[dict]:
            return [
                {
                    'position': self._parent._x[i],
                    'velocity': self._parent._v[i], 
                    'covariance': [row[:] for row in self._parent._P[i]],
                    'measurement_noise': self._parent._R[i],
                    'process_noise': self._parent._Q[i]
                }
                for i in self._indices
            ]

        @property
        def value(self) -> list[int | None]:
            return self._get_values()

        @property
        def nonblocking(self) -> list[bool]:
            return [bool(self._parent._nonblocking[i]) for i in self._indices]

        @nonblocking.setter
        def nonblocking(self, flag: bool):
            flag_val = 1 if flag else 0
            for i in self._indices:
                self._parent._nonblocking[i] = flag_val

        @property
        def temperature(self) -> list[float]:
            return [self._parent._temp_c[i] for i in self._indices]

        @temperature.setter
        def temperature(self, temp_c: float):
            if isinstance(temp_c, (int, float)):
                if not (-40.0 <= temp_c <= 85.0):
                    raise ValueError("Temperature must be between -40°C and +85°C")
                temp_val = float(temp_c)
                for i in self._indices:
                    self._parent._temp_c[i] = temp_val
            else:
                if len(temp_c) != len(self._indices):
                    raise ValueError("list length must match number of sensors")
                for i, temp in zip(self._indices, temp_c):
                    if not (-40.0 <= temp <= 85.0):
                        raise ValueError("Temperature must be between -40°C and +85°C")
                    self._parent._temp_c[i] = float(temp)

        @property
        def filter(self) -> list[dict]:
            return [{'R': self._parent._R[i], 'Q': self._parent._Q[i]} 
                   for i in self._indices]

        @filter.setter
        def filter(self, params: dict):
            if not isinstance(params, dict):
                raise TypeError("Filter parameters must be a dictionary")

            R = params.get('R')
            Q = params.get('Q')

            for i in self._indices:
                if R is not None:
                    if R <= 0:
                        raise ValueError("R (measurement noise) must be positive")
                    self._parent._R[i] = float(R)
                if Q is not None:
                    if Q <= 0:
                        raise ValueError("Q (process noise) must be positive")
                    self._parent._Q[i] = float(Q)

        @property
        def callback(self) -> list[callable]:
            return [self._parent._user_callbacks[i] for i in self._indices]

        @callback.setter
        def callback(self, fn: callable | list[callable]):
            if callable(fn) or fn is None:
                for i in self._indices:
                    self._parent._user_callbacks[i] = fn
            else:
                if len(fn) != len(self._indices):
                    raise ValueError("list length must match number of sensors")
                for i, callback in zip(self._indices, fn):
                    if not (callable(callback) or callback is None):
                        raise TypeError("Each callback must be callable or None")
                    self._parent._user_callbacks[i] = callback


