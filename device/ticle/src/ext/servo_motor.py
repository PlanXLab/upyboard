from . import (
    utime, array,
    machine,
    utools, ticle
)


class ServoMotor:
    def __init__(self, pins: list[int]|tuple[int, ...], *, freq: int = 50, min_us: int = 500, max_us: int = 2500, init_angle: float = 90.0):
        if not (0.0 <= init_angle <= 180.0):
            raise ValueError("init_angle must be between 0.0 and 180.0 degrees")
            
        try:
            self._pwm = ticle.Pwm(pins)
            self._pwm[:].freq = freq
        except Exception as e:
            raise OSError(f"Failed to initialize PWM: {e}")
        
        n = len(pins)
        
        self._min_us = array.array('H', [min_us] * n)
        self._max_us = array.array('H', [max_us] * n)
        
        init_deg = utools.clamp(init_angle, 0.0, 180.0)
        self._current_angles = array.array('f', [init_deg] * n)
        self._target_angles = array.array('f', [init_deg] * n)
        
        self._nonblocking = array.array('B', [0] * n)
        self._is_moving = array.array('B', [0] * n)
        self._start_time = array.array('L', [0] * n)
        self._start_angles = array.array('f', [init_deg] * n)
        self._duration_ms = array.array('H', [1000] * n) 
        
        self._timer = machine.Timer()
        self._timer_active = False
        self._is_shutting_down = False
        
        for i in range(n):
            us = self._compute_us(init_deg, i)
            self._pwm[i].duty_us = int(us)

    def deinit(self) -> None:
        self._is_shutting_down = True
        
        try:
            for i in range(len(self._is_moving)):
                self._is_moving[i] = 0
            
            if self._timer_active:
                try:
                    self._timer.deinit()
                except:
                    pass
                finally:
                    self._timer_active = False
            
            utime.sleep_ms(50)
            try:
                self._pwm.deinit()
            except:
                pass
        except:
            pass

    def __getitem__(self, idx: int|slice) -> "_ServoMotorView":
        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._current_angles))))
            return ServoMotor._ServoMotorView(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._current_angles)):
                raise IndexError("Servo index out of range")
            return ServoMotor._ServoMotorView(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        return len(self._current_angles)

    def _compute_us(self, deg: float, idx: int) -> float:
        deg = utools.clamp(deg, 0.0, 180.0)
        span = self._max_us[idx] - self._min_us[idx]
        return self._min_us[idx] + span * deg / 180.0

    @staticmethod
    def _get_angle_list(parent, indices: list[int]) -> list[float]:
        return [parent._current_angles[i] for i in indices]

    @staticmethod
    def _set_angle_single(parent, idx: int, deg: float) -> None:
        deg = utools.clamp(deg, 0.0, 180.0)
        parent._target_angles[idx] = deg
        
        if not parent._nonblocking[idx]:
            us = parent._compute_us(deg, idx)
            parent._pwm[idx].duty_us = int(us)
            parent._current_angles[idx] = deg
            parent._is_moving[idx] = 0
        else:
            parent._start_angles[idx] = parent._current_angles[idx]
            parent._start_time[idx] = utime.ticks_ms()
            parent._is_moving[idx] = 1
            
            if not parent._timer_active:
                try:
                    parent._timer.deinit()
                except:
                    pass
                parent._timer.init(period=20, mode=machine.Timer.PERIODIC, callback=parent._timer_cb)
                parent._timer_active = True

    @staticmethod
    def _get_target_angle_list(parent, indices: list[int]) -> list[float]:
        return [parent._target_angles[i] for i in indices]

    @staticmethod
    def _get_nonblocking_list(parent, indices: list[int]) -> list[bool]:
        return [bool(parent._nonblocking[i]) for i in indices]

    @staticmethod
    def _set_nonblocking_all(parent, flag: bool, indices: list[int]) -> None:
        flag_val = 1 if flag else 0
        for i in indices:
            parent._nonblocking[i] = flag_val

    @staticmethod
    def _get_is_moving_list(parent, indices: list[int]) -> list[bool]:
        return [bool(parent._is_moving[i]) for i in indices]

    @staticmethod
    def _get_duration_ms_list(parent, indices: list[int]) -> list[int]:
        return [parent._duration_ms[i] for i in indices]

    @staticmethod
    def _set_duration_ms_all(parent, ms: int, indices: list[int]) -> None:
        if ms <= 0:
            raise ValueError("Duration must be positive")
        ms = max(100, int(ms))
        for i in indices:
            parent._duration_ms[i] = ms

    @staticmethod
    def _get_calibration_list(parent, indices: list[int]) -> list[dict]:
        return [{'min_us': parent._min_us[i], 'max_us': parent._max_us[i]} 
                for i in indices]

    @staticmethod
    def _set_calibration_all(parent, params: dict, indices: list[int]) -> None:
        if not isinstance(params, dict):
            raise TypeError("Calibration parameters must be a dictionary")
        
        min_us = params.get('min_us')
        max_us = params.get('max_us')
        
        for i in indices:
            if min_us is not None:
                if not (100 <= min_us <= 3000):
                    raise ValueError("min_us must be between 100 and 3000 microseconds")
                parent._min_us[i] = min_us
            
            if max_us is not None:
                if not (100 <= max_us <= 3000):
                    raise ValueError("max_us must be between 100 and 3000 microseconds")
                parent._max_us[i] = max_us

            if (min_us is not None and max_us is not None and 
                parent._min_us[i] >= parent._max_us[i]):
                raise ValueError("min_us must be less than max_us")

    @staticmethod
    def _wait_completion_all(parent, indices: list[int], timeout_ms: int = 10000) -> bool:
        start_time = utime.ticks_ms()
        while any(parent._is_moving[i] for i in indices):
            if utime.ticks_diff(utime.ticks_ms(), start_time) > timeout_ms:
                return False
            utime.sleep_ms(10)
        return True

    @staticmethod
    def _stop_all(parent, indices: list[int]) -> None:
        for i in indices:
            parent._is_moving[i] = 0
            parent._target_angles[i] = parent._current_angles[i]

    def _timer_cb(self, t) -> None:
        try:
            any_moving = False
            
            if self._is_shutting_down:
                return
            
            current_time = utime.ticks_ms()
            
            for idx in range(len(self._current_angles)):
                if not self._nonblocking[idx] or not self._is_moving[idx]:
                    continue
                
                elapsed_ms = utime.ticks_diff(current_time, self._start_time[idx])
                duration_ms = self._duration_ms[idx]
                
                if elapsed_ms >= duration_ms:
                    self._current_angles[idx] = self._target_angles[idx]
                    self._is_moving[idx] = 0
                    us = self._compute_us(self._target_angles[idx], idx)
                    self._pwm[idx].duty_us = int(us)
                else:
                    progress = elapsed_ms / duration_ms
                    start_angle = self._start_angles[idx]
                    target_angle = self._target_angles[idx]
                    current_angle = start_angle + (target_angle - start_angle) * progress
                    
                    self._current_angles[idx] = current_angle
                    us = self._compute_us(current_angle, idx)
                    self._pwm[idx].duty_us = int(us)
                    any_moving = True
            
            if not any_moving:
                try:
                    self._timer.deinit()
                except:
                    pass
                finally:
                    self._timer_active = False
        except:
            try:
                self._timer.deinit()
            except:
                pass
            finally:
                self._timer_active = False

    class _ServoMotorView:
        def __init__(self, parent: "ServoMotor", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "ServoMotor._ServoMotorView":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return ServoMotor._ServoMotorView(self._parent, selected_indices)
            else:
                return ServoMotor._ServoMotorView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        @property
        def angle(self) -> list[float]:
            return ServoMotor._get_angle_list(self._parent, self._indices)

        @angle.setter
        def angle(self, value: float | list[float]):
            if isinstance(value, (list, tuple)):
                if len(value) != len(self._indices):
                    raise ValueError("list length must match number of servos in view")
                for idx, deg in zip(self._indices, value):
                    if not (0.0 <= deg <= 180.0):
                        raise ValueError("Angle must be between 0.0 and 180.0 degrees")
                    ServoMotor._set_angle_single(self._parent, idx, deg)
            else:
                deg = float(value)
                if not (0.0 <= deg <= 180.0):
                    raise ValueError("Angle must be between 0.0 and 180.0 degrees")
                for i in self._indices:
                    ServoMotor._set_angle_single(self._parent, i, deg)

        @property
        def target_angle(self) -> list[float]:
            return ServoMotor._get_target_angle_list(self._parent, self._indices)

        @property
        def nonblocking(self) -> list[bool]:
            return ServoMotor._get_nonblocking_list(self._parent, self._indices)

        @nonblocking.setter
        def nonblocking(self, flag: bool):
            ServoMotor._set_nonblocking_all(self._parent, flag, self._indices)

        @property
        def is_moving(self) -> list[bool]:
            return ServoMotor._get_is_moving_list(self._parent, self._indices)

        @property
        def duration_ms(self) -> list[int]:
            return ServoMotor._get_duration_ms_list(self._parent, self._indices)

        @duration_ms.setter
        def duration_ms(self, ms: int):
            ServoMotor._set_duration_ms_all(self._parent, ms, self._indices)

        @property
        def calibration(self) -> list[dict]:
            return ServoMotor._get_calibration_list(self._parent, self._indices)

        @calibration.setter
        def calibration(self, params: dict):
            ServoMotor._set_calibration_all(self._parent, params, self._indices)

        def wait_completion(self, timeout_ms: int = 10000) -> bool:
            return ServoMotor._wait_completion_all(self._parent, self._indices, timeout_ms)

        def stop(self):
            ServoMotor._stop_all(self._parent, self._indices)

