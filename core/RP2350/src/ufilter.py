from array import array
from math import exp, pi, sqrt, tan, fabs
import micropython

__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"

class FilterError(Exception):
    pass

class FilterConfigurationError(FilterError):
    pass

class FilterOperationError(FilterError):
    pass


class BaseFilter:    
    def __init__(self) -> None:
        self._initialized = False
        self._sample_count = 0

    def update(self, x: float) -> float:
        raise NotImplementedError("Subclasses must implement update method")
    
    def reset(self) -> None:
        self._initialized = False
        self._sample_count = 0
    
    def __call__(self, x: float) -> float:
        return self.update(x)
    
    def process_batch(self, samples: list) -> list:
        if not hasattr(samples, '__iter__'):
            raise TypeError("samples must be iterable")
        
        return [self.update(sample) for sample in samples]
    
    @property
    def sample_count(self) -> int:
        return self._sample_count
    
    def _validate_numeric(self, value: float, name: str, min_val: float = None, max_val: float = None) -> float:
        try:
            value = float(value)
        except (TypeError, FilterConfigurationError):
            raise FilterConfigurationError(f"{name} must be a number, got {type(value).__name__}")

        if min_val is not None and value < min_val:
            raise FilterConfigurationError(f"{name} must be >= {min_val}, got {value}")

        if max_val is not None and value > max_val:
            raise FilterConfigurationError(f"{name} must be <= {max_val}, got {value}")

        return value


class AlphaFilter(BaseFilter):
    def __init__(self, alpha: float, initial: float = 0.0) -> None:
        super().__init__()
        if alpha <= 0.0 or alpha > 1.0:
            raise FilterConfigurationError("alpha must be in range (0, 1]")
        self._alpha = float(alpha)
        self.y = float(initial)
        self._initial_value = float(initial)

    @property
    def alpha(self) -> float:
        return self._alpha

    @alpha.setter
    def alpha(self, value: float) -> None:
        if value <= 0.0 or value > 1.0:
            raise FilterConfigurationError("alpha must be in range (0, 1]")
        self._alpha = float(value)

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        x = float(x)
        self.y = self._alpha * x + (1.0 - self._alpha) * self.y
        return self.y
    
    def reset(self) -> None:
        super().reset()
        self.y = self._initial_value


class LowPassFilter(BaseFilter):
    def __init__(self, fc: float, fs: float, initial: float = 0.0) -> None:
        super().__init__()  
        if fs <= 0:
            raise FilterConfigurationError("Sampling frequency must be positive")
        
        # Validate cutoff frequency and Nyquist criterion
        if fc <= 0 or fc >= fs / 2:
            raise FilterConfigurationError("Cutoff frequency must be between 0 and {}".format(fs/2))
        
        self.fs = float(fs)
        self.fc = float(fc)
        
        # Calculate filter coefficient using bilinear transform equivalent
        self._alpha = 1.0 - exp(-2.0 * pi * fc / fs)
        
        self.y = float(initial)
        self._initial_value = float(initial)

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        x = float(x)
        self.y = self._alpha * x + (1.0 - self._alpha) * self.y
        return self.y
    
    def reset(self) -> None:
        super().reset()
        self.y = self._initial_value


class HighPassFilter(BaseFilter):
    def __init__(self, fc: float, fs: float, initial: float = 0.0) -> None:
        super().__init__()
        if fs <= 0:
            raise FilterConfigurationError("Sampling frequency must be positive")
        if fc <= 0 or fc >= fs / 2:
            raise FilterConfigurationError("Cutoff frequency must be between 0 and {}".format(fs/2))
        
        self.fs = float(fs)
        self.fc = float(fc)
        self.a = exp(-2.0 * pi * fc / fs)
        self.y = float(initial)
        self.x_prev = float(initial)
        self._initial_value = float(initial)

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        x = float(x)
        self.y = self.a * (self.y + x - self.x_prev)
        self.x_prev = x
        return self.y
    
    def reset(self) -> None:
        super().reset()
        self.y = self._initial_value
        self.x_prev = self._initial_value


class TauLowPass(BaseFilter):
    def __init__(self, tau_s: float, initial: float = 0.0, fs: float = None) -> None:
        super().__init__()
        if tau_s <= 0.0:
            raise ValueError("tau_s must be > 0")
        self._tau = float(tau_s)

        self.y = float(initial)
        self._initial_value = float(initial)

        if fs is not None:
            if fs <= 0.0:
                raise ValueError("fs must be > 0 when provided")
            self.fs = float(fs)
            self._alpha_fixed = self._compute_alpha_fixed()
        else:
            self.fs = None
            self._alpha_fixed = None

    @property
    def tau(self) -> float:
        return self._tau

    @tau.setter
    def tau(self, value: float) -> None:
        if value <= 0.0:
            raise ValueError("tau must be > 0")
        self._tau = float(value)
        if self.fs is not None:
            self._alpha_fixed = self._compute_alpha_fixed()

    def set_cutoff(self, fc_hz: float) -> None:
        if fc_hz <= 0.0:
            raise ValueError("fc must be > 0")
        self.tau = 1.0 / (2.0 * pi * float(fc_hz))

    def _compute_alpha_fixed(self) -> float:
        return 1.0 - exp(-(1.0 / self.fs) / self._tau)

    @micropython.native
    def update_with_dt(self, x: float, dt_s: float) -> float:
        self._sample_count += 1
        x_val = float(x)
        dt = float(dt_s)

        if dt <= 0.0:
            return self.y

        a = 1.0 - exp(-dt / self._tau)
        if a < 0.0:
            a = 0.0
        elif a > 1.0:
            a = 1.0

        self.y = a * x_val + (1.0 - a) * self.y
        return self.y

    @micropython.native
    def update(self, x: float) -> float:
        if self._alpha_fixed is None:
            raise FilterOperationError(
                "TauLowPass.update() requires fixed fs. "
                "Use update_with_dt(x, dt) or pass fs in the constructor."
            )

        self._sample_count += 1
        x_val = float(x)
        a = self._alpha_fixed
        self.y = a * x_val + (1.0 - a) * self.y
        return self.y

    def reset(self) -> None:
        self._sample_count = 0
        super().reset()
        self.y = self._initial_value


class SlewRateLimiter(BaseFilter):
    def __init__(self, rise_per_s: float, fall_per_s: float = None, initial: float = 0.0, fs: float = None, deadband: float = 0.0) -> None:
        super().__init__()

        if rise_per_s is None or rise_per_s <= 0.0:
            raise ValueError("rise_per_s must be > 0")
        if fall_per_s is None:
            fall_per_s = rise_per_s
        if fall_per_s <= 0.0:
            raise ValueError("fall_per_s must be > 0")
        if deadband < 0.0:
            raise ValueError("deadband must be >= 0")

        self._rise = float(rise_per_s)
        self._fall = float(fall_per_s)
        self._deadband = float(deadband)

        self.y = float(initial)
        self._initial_value = float(initial)

        if fs is not None:
            if fs <= 0.0:
                raise ValueError("fs must be > 0 when provided")
            self.fs = float(fs)
            self._step_up = self._rise / self.fs
            self._step_down = self._fall / self.fs
        else:
            self.fs = None
            self._step_up = None
            self._step_down = None

    @property
    def rise_per_s(self) -> float:
        return self._rise

    @rise_per_s.setter
    def rise_per_s(self, value: float) -> None:
        if value <= 0.0:
            raise ValueError("rise_per_s must be > 0")
        self._rise = float(value)
        if self.fs is not None:
            self._step_up = self._rise / self.fs

    @property
    def fall_per_s(self) -> float:
        return self._fall

    @fall_per_s.setter
    def fall_per_s(self, value: float) -> None:
        if value <= 0.0:
            raise ValueError("fall_per_s must be > 0")
        self._fall = float(value)
        if self.fs is not None:
            self._step_down = self._fall / self.fs

    @property
    def deadband(self) -> float:
        return self._deadband

    @deadband.setter
    def deadband(self, value: float) -> None:
        if value < 0.0:
            raise ValueError("deadband must be >= 0")
        self._deadband = float(value)

    def set_fs(self, fs: float) -> None:
        if fs <= 0.0:
            raise ValueError("fs must be > 0")
        self.fs = float(fs)
        self._step_up = self._rise / self.fs
        self._step_down = self._fall / self.fs

    @micropython.native
    def update_with_dt(self, x: float, dt_s: float) -> float:
        self._sample_count += 1
        x_val = float(x)
        dt = float(dt_s)

        if dt <= 0.0:
            return self.y

        delta = x_val - self.y
        if fabs(delta) <= self._deadband:
            return self.y

        max_up = self._rise * dt
        max_dn = self._fall * dt

        if delta > 0.0:
            step = delta if delta <= max_up else max_up
        else:
            step = delta if delta >= -max_dn else -max_dn

        self.y += step
        return self.y

    @micropython.native
    def update(self, x: float) -> float:
        if self._step_up is None or self._step_down is None:
            raise FilterOperationError(
                "SlewRateLimiter.update() requires fixed fs. "
                "Use update_with_dt(x, dt) or call set_fs(fs)."
            )

        self._sample_count += 1
        x_val = float(x)

        delta = x_val - self.y
        if fabs(delta) <= self._deadband:
            return self.y

        if delta > 0.0:
            step = delta if delta <= self._step_up else self._step_up
        else:
            step = delta if delta >= -self._step_down else -self._step_down

        self.y += step
        return self.y

    def reset(self) -> None:
        super().reset()
        self.y = self._initial_value


class MovingAverageFilter(BaseFilter):
    def __init__(self, window_size: int, initial: float = 0.0) -> None:
        super().__init__()
        
        if not isinstance(window_size, int) or window_size <= 0:
            raise FilterConfigurationError("window_size must be a positive integer")
        
        self._window_size = window_size
        self._initial_value = float(initial)
        self._buf = array("f", [self._initial_value] * self._window_size)
        self._sum = float(self._initial_value * self._window_size)
        self._idx = 0
        self._count = 0

    @micropython.native
    def update(self, x: float) -> float:
        x_val = float(x)
        idx = int(self._idx)
        count = int(self._count)
        window_size = int(self._window_size)
        
        old_value = float(self._buf[idx])
        
        self._sum += x_val - old_value
        self._buf[idx] = x_val
        
        self._idx = (idx + 1) % window_size
        if count < window_size:
            self._count = count + 1
            count = self._count
        
        self._sample_count += 1
        
        return float(self._sum / count)

    def reset(self) -> None:
        super().reset()
        for i in range(self._window_size):
            self._buf[i] = self._initial_value
        self._sum = self._initial_value * self._window_size
        self._idx = 0
        self._count = 0


class MedianFilter(BaseFilter):
    def __init__(self, window_size: int, initial: float = 0.0) -> None:
        super().__init__()
        if not isinstance(window_size, int) or window_size <= 0:
            raise FilterConfigurationError("window_size must be positive int")

        self._window_size = window_size        
        self.reset()

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        x = float(x)

        old = float(self._ring[self._idx])
        self._ring[self._idx] = x
        self._idx = (self._idx + 1) % self._window_size
        if self._count < self._window_size:
            self._sorted[self._count] = x
            self._count += 1
            i = self._count - 1
            while i > 0 and self._sorted[i-1] > self._sorted[i]:
                self._sorted[i-1], self._sorted[i] = self._sorted[i], self._sorted[i-1]
                i -= 1
        else:
            i = 0
            n = self._window_size
            while i < n and self._sorted[i] != old:
                i += 1

            if i < n:
                while i < n-1:
                    self._sorted[i] = self._sorted[i+1]
                    i += 1

            j = n - 1
            while j > 0 and self._sorted[j-1] > x:
                self._sorted[j] = self._sorted[j-1]
                j -= 1
            self._sorted[j] = x

        n = self._count
        mid = n >> 1
        return float(self._sorted[mid]) if n & 1 else 0.5 * (self._sorted[mid-1] + self._sorted[mid])

    def reset(self) -> None:
        super().reset()
        self._ring = array("f", [float(initial)] * self._window_size)
        self._sorted = [float(initial)] * self._window_size
        self._idx = 0
        self._count = 0


class RMSFilter(BaseFilter):
    def __init__(self, window_size: int) -> None:
        super().__init__()
        
        if not isinstance(window_size, int) or window_size <= 0:
            raise FilterConfigurationError("window_size must be a positive integer")
        
        self._window_size = window_size
        self.reset()

    @micropython.native
    def update(self, x: float) -> float:
        x_val = float(x)
        idx = int(self._idx)
        count = int(self._count)
        window_size = int(self._window_size)
        
        old_value = float(self._buf[idx])
        self._sum_of_squares += x_val * x_val - old_value * old_value
        self._buf[idx] = x_val
        
        self._idx = (idx + 1) % window_size
        if count < window_size:
            self._count = count + 1
            count = self._count
        
        self._sample_count += 1
        
        return sqrt(max(0.0, self._sum_of_squares / count))

    def reset(self) -> None:
        super().reset()
        self._buf = array("f", [0.0] * self._window_size)
        self._sum_of_squares = 0.0
        self._idx = 0
        self._count = 0


class KalmanFilter(BaseFilter):
    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1, 
                 initial_estimate: float = 0.0, initial_error: float = 1.0) -> None:
        super().__init__() 
        
        if process_noise < 0:
            raise FilterConfigurationError("Process noise must be non-negative")
        if measurement_noise <= 0:
            raise FilterConfigurationError("Measurement noise must be positive")
        
        self.q = float(process_noise)
        self.r = float(measurement_noise)
        self.x = float(initial_estimate)
        self.p = float(initial_error)
        self._initial_estimate = float(initial_estimate)
        self._initial_error = float(initial_error)

    @micropython.native
    def update(self, measurement: float) -> float:
        z = float(measurement)
        
        p_pred = float(self.p + self.q)
        
        if p_pred > 100.0:
            self.p = 1.0
            p_pred = 1.0 + float(self.q)
        
        k = p_pred / (p_pred + float(self.r))
        innovation = z - float(self.x)
        
        if abs(innovation) > 3.0 * sqrt(p_pred):
            k *= 0.1  # Reduce Kalman gain for outliers
        
        self.x = float(self.x) + k * innovation
        self.p = (1.0 - k) * p_pred
        
        self._sample_count += 1
        return float(self.x)

    def reset(self) -> None:
        super().reset()
        self.x = self._initial_estimate
        self.p = self._initial_error


class AdaptiveFilter(BaseFilter):
    def __init__(self, alpha_min: float = 0.01, alpha_max: float = 0.9, 
                 threshold: float = 0.1, initial: float = 0.0) -> None:
        super().__init__()
        
        if not (0.0 < alpha_min < 1.0):
            raise FilterConfigurationError("alpha_min must be in range (0, 1)")
        if not (0.0 < alpha_max <= 1.0):
            raise FilterConfigurationError("alpha_max must be in range (0, 1]")
        if alpha_min >= alpha_max:
            raise FilterConfigurationError("alpha_min must be < alpha_max")
        if threshold < 0:
            raise FilterConfigurationError("threshold must be non-negative")
        
        self.alpha_min = float(alpha_min)
        self.alpha_max = float(alpha_max)
        self.threshold = float(threshold)
        self.y = float(initial)
        self.prev_x = float(initial)
        self._initial_value = float(initial)

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        x = float(x)
        
        change = abs(x - self.prev_x)
        
        if change > self.threshold:
            alpha = self.alpha_max  # Fast tracking for large changes
        else:
            ratio = change / self.threshold if self.threshold > 0 else 0
            alpha = self.alpha_min + ratio * (self.alpha_max - self.alpha_min)
        
        self.y = alpha * x + (1.0 - alpha) * self.y
        self.prev_x = x
        
        return self.y

    def reset(self) -> None:
        super().reset()
        self.y = self._initial_value
        self.prev_x = self._initial_value


class BiquadFilter(BaseFilter):
    def __init__(self, b0: float, b1: float, b2: float, a1: float, a2: float) -> None:
        super().__init__()
        self.set_coefficients(b0, b1, b2, a1, a2)
        self.reset()

    def set_coefficients(self, b0: float, b1: float, b2: float, a1: float, a2: float) -> None:
        self.b0 = float(b0); self.b1 = float(b1); self.b2 = float(b2)
        self.a1 = float(a1); self.a2 = float(a2)

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        x = float(x)
        # DF-II Transposed states
        w0 = x - self.a1*self.z1 - self.a2*self.z2
        y  = self.b0*w0 + self.b1*self.z1 + self.b2*self.z2
        self.z2 = self.z1
        self.z1 = w0
        return y

    def reset(self) -> None:
        super().reset()
        self.z1 = 0.0
        self.z2 = 0.0


class ButterworthFilter(BiquadFilter):    
    def __init__(self, fc: float, fs: float, filter_type: str = 'lowpass') -> None:
        if filter_type not in ('lowpass', 'highpass'):
            raise FilterConfigurationError("filter_type must be 'lowpass' or 'highpass'")
        
        if fs <= 0:
            raise FilterConfigurationError("Sampling frequency must be positive")
        if fc <= 0 or fc >= fs / 2:
            raise FilterConfigurationError("Cutoff frequency must be between 0 and {}".format(fs/2))
        
        self.fc = float(fc)
        self.fs = float(fs)
        self.filter_type = filter_type
        
        coeffs = self._design_filter(fc, fs, filter_type)
        super().__init__(*coeffs)

    def _design_filter(self, fc: float, fs: float, filter_type: str) -> tuple:
        wc = 2.0 * pi * fc / fs
        k = tan(wc / 2.0)
        
        if filter_type == 'lowpass':
            norm = 1.0 / (1.0 + sqrt(2.0) * k + k * k)
            b0 = k * k * norm
            b1 = 2.0 * b0
            b2 = b0
            a1 = 2.0 * (k * k - 1.0) * norm
            a2 = (1.0 - sqrt(2.0) * k + k * k) * norm
        else:  # highpass
            norm = 1.0 / (1.0 + sqrt(2.0) * k + k * k)
            b0 = norm
            b1 = -2.0 * b0
            b2 = b0
            a1 = 2.0 * (k * k - 1.0) * norm
            a2 = (1.0 - sqrt(2.0) * k + k * k) * norm
        
        return (b0, b1, b2, a1, a2)
    
    @micropython.native
    def update(self, x: float) -> float:
        return super().update(x)
    
    def reset(self) -> None:
        super().reset()


class FIRFilter(BaseFilter):
    def __init__(self, taps: list) -> None:
        super().__init__()
        self.taps = taps
        self.reset()

    @property
    def taps(self) -> list:
        return list(self._taps)

    @taps.setter
    def taps(self, taps: list) -> None:
        if not taps:
            raise FilterConfigurationError("taps list cannot be empty")
        self._taps = array("f", [float(t) for t in taps])
        self.n = len(self._taps)
        if hasattr(self, '_buf'):
            self._buf = array("f", [0.0] * self.n)
            self._idx = 0

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        x = float(x)
        
        self._buf[self._idx] = x
        
        acc = 0.0
        tap_i = 0
        buf_i = self._idx
        
        while tap_i < self.n:
            acc += self._buf[buf_i] * self._taps[tap_i]
            buf_i -= 1
            if buf_i < 0:
                buf_i = self.n - 1
            tap_i += 1
        
        self._idx = (self._idx + 1) % self.n
        return acc

    def reset(self) -> None:
        super().reset()
        self._buf = array("f", [0.0] * self.n)
        self._idx = 0


class FilterChain(BaseFilter):
    def __init__(self, *filters: BaseFilter) -> None:
        super().__init__()
        if not filters:
            raise FilterConfigurationError("At least one filter required")
        
        for f in filters:
            if not isinstance(f, BaseFilter):
                raise FilterConfigurationError("All items must be BaseFilter instances, got {}".format(type(f)))
        
        self.filters = list(filters)

    @micropython.native
    def update(self, x: float) -> float:
        self._sample_count += 1
        result = float(x)
        for filter_obj in self.filters:
            result = filter_obj.update(result)
        return result

    def reset(self) -> None:
        super().reset()
        for filter_obj in self.filters:
            filter_obj.reset()

    def add_filter(self, filter_obj: BaseFilter) -> None:
        if not isinstance(filter_obj, BaseFilter):
            raise FilterConfigurationError("filter_obj must be BaseFilter instance")
        self.filters.append(filter_obj)

    def remove_filter(self, index: int) -> BaseFilter:
        if not (0 <= index < len(self.filters)):
            raise FilterConfigurationError("Filter index out of range")
        return self.filters.pop(index)

