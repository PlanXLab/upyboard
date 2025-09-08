from . import (
    utime, array,
    machine, micropython,
    ticle
)


class SR04:
    """
    Multi-channel HC-SR04 ultrasonic distance sensor controller with Kalman filtering.
    
    Features:
    
        - Multiple sensor support with individual configuration
        - Distance measurement from 2cm to 400cm per sensor
        - Temperature compensation for speed of sound
        - Individual Kalman filters for noise reduction per sensor
        - Blocking and non-blocking measurement modes with user callbacks
        - Configurable measurement and process noise per sensor
        - Timeout protection for failed measurements
        - Unified indexing/slicing and property-based interface    

    """


    def __init__(self, sensor_configs: list[tuple[int, int]], *, 
                 temp_c: float = 20.0, R: float = 25.0, Q: float = 4.0, 
                 period_ms: int = 50):
        """
        Initialize multiple HC-SR04 ultrasonic sensors.
        
        This method sets up multiple ultrasonic distance sensors with individual
        Kalman filters for noise reduction and temperature compensation for
        accurate distance measurements.
        
        :param sensor_configs: list of tuples (trig_pin, echo_pin) for each sensor
        :param temp_c: Temperature in degrees Celsius for speed of sound calculation (default: 20.0)
        :param R: Measurement noise covariance (default: 25.0)
        :param Q: Process noise covariance (default: 4.0)
        :param period_ms: Measurement period in milliseconds (default: 50)        

            Recommended periods:

                - 1 sensor: 50ms
                - 2-3 sensors: 150ms
                - 4+ sensors: 200ms
        
        :raises ValueError: If no sensor configurations provided or temperature out of range
        :raises OSError: If GPIO pin initialization fails
        
        Example
        -------
        ```python
            >>> # Single sensor setup
            >>> sensor = SR04([(1, 2)])
            >>> 
            >>> # Multiple sensors with custom settings
            >>> sensors = SR04([(1, 2), (3, 4), (5, 6)], 
            ...                temp_c=25.0, R=10.0, Q=2.0, period_ms=150)
            >>> 
            >>> # Read distance from first sensor
            >>> distance = sensors[0].value[0]
            >>> print(f"Distance: {distance}cm")
        ```
        """
        if not sensor_configs:
            raise ValueError("At least one sensor configuration must be provided")
        
        if not (-40.0 <= temp_c <= 85.0):
            raise ValueError("Temperature must be between -40°C and +85°C")
        
        n = len(sensor_configs)
        
        try:
            trig_pins = [config[0] for config in sensor_configs]
            echo_pins = [config[1] for config in sensor_configs]
             
            self._trig = ticle.Dout(trig_pins)
            self._trig[:].active =  ticle.Dout.LOGIC_HIGH
            self._echo = ticle.Din(echo_pins)
        except Exception as e:
            raise OSError(f"Failed to initialize GPIO pins: {e}")
        
        self._trig_pins = trig_pins
        self._echo_pins = echo_pins
        
        self._x = array.array('f', [0.0] * n)                   # Position estimates
        self._v = array.array('f', [0.0] * n)                   # Velocity estimates    
        self._P = [[[1.0, 0.0], [0.0, 1.0]] for _ in range(n)]  # Error covariance matrices
        self._R = array.array('f', [float(R)] * n)              # Measurement noise covariances
        self._Q = array.array('f', [float(Q)] * n)              # Process noise covariances
        
        self._temp_c = array.array('f', [float(temp_c)] * n)
        
        self._nonblocking = array.array('B', [0] * n) 
        self._measurement_enabled = array.array('B', [0] * n)
        self._measurement_active = array.array('B', [0] * n)        
        self._user_callbacks = [None] * n
        
        self._period_ms = period_ms  # Timer period in milliseconds
        self._timer = machine.Timer()
        self._timer_active = False

    def __getitem__(self, idx: int|slice) -> "_SR04View":
        """
        Get a view for specific sensor(s) that supports both reading and configuration.
        
        :param idx: Index or slice of sensors to access
        :return: SR04._SR04View instance for the specified sensors
        
        :raises IndexError: If sensor index is out of range
        :raises TypeError: If index is not int or slice
        
        Example
        -------
        ```python
            >>> # Access first sensor
            >>> sensor_0 = u[0]
            >>> 
            >>> # Access multiple sensors
            >>> first_two_sensors = u[0:2]
            >>> odd_sensors = u[1::2]
        ```
        """
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
        """
        Get the number of sensors controlled by this instance.
        
        :return: Number of sensors initialized
        
        Example
        -------
        ```python
        >>> u = SR04([(1, 2), (3, 4)])
            >>> print(len(u))  # Output: 2
        ```
        """
        return len(self._temp_c)

    def deinit(self) -> None:
        """
        Deinitialize sensor controller and release resources.
        
        This method safely shuts down all sensors by stopping measurements,
        deinitializing the timer, and releasing GPIO resources. Should be
        called when the sensor controller is no longer needed.
        
        Example
        -------
        ```python
            >>> sensors = SR04([(1, 2), (3, 4)])
            >>> # ... use sensors ...
            >>> sensors.deinit()  # Clean shutdown
        ```
        """
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
        """
        Get the global measurement period for non-blocking mode.
        
        Returns the current timer period used for automatic measurements
        in non-blocking mode.
        
        :return: Period in milliseconds
        
        Example
        -------
        ```python
            >>> sensors = SR04([(1, 2), (3, 4)])
            >>> print(f"Current period: {sensors.period_ms}ms")  # 50
        ```
        """
        return self._period_ms

    @period_ms.setter
    def period_ms(self, ms: int):
        """
        Set the global measurement period for non-blocking mode.
        
        Adjusts how frequently automatic measurements are taken in
        non-blocking mode. The timer is automatically restarted if active.
        
        :param ms: Period in milliseconds (minimum 10ms)
        
        :raises ValueError: If period is less than 10ms
        
        Example:
            >>> sensors = SR04([(1, 2), (3, 4)])
            >>> sensors.period_ms = 100  # Measure every 100ms
            >>> 
            >>> # For multiple sensors, use longer periods
            >>> sensors.period_ms = 200  # Recommended for 4+ sensors
        """
        if ms < 10:
            raise ValueError("Period must be at least 10ms")
        self._period_ms = int(ms)
        
        if self._timer_active:
            self._stop_timer()
            has_active = any(self._measurement_enabled[i] and self._nonblocking[i] for i in range(len(self)))
            if has_active:
                self._start_timer()

    def _cm_per_us(self, temp: float) -> float:
        """
        Calculate the speed of sound in cm/µs based on temperature.
        
        This internal method computes the temperature-compensated speed of sound
        for accurate distance calculations. Accounts for round-trip travel time.
        
        :param temp: Temperature in degrees Celsius
        :return: Speed of sound in cm/us (one-way)
        """
        speed_ms = 331.3 + 0.606 * temp  # Speed in m/s
        speed_cm_us = (speed_ms * 100.0) / 1_000_000  # Convert to cm/µs
        return speed_cm_us / 2.0  # Divide by 2 for round-trip

    def _trigger(self, idx: int):
        """
        Send a 10us trigger pulse to a specific sensor.
        
        This internal method generates the required trigger pulse for the
        HC-SR04 sensor to start a distance measurement.
        
        :param idx: Index of the sensor to trigger
        """
        self._trig[idx].value = 0  # Ensure clean LOW state
        utime.sleep_us(2)    # Brief settling time
        self._trig[idx].value = 1  # Set HIGH
        utime.sleep_us(10)   # 10us trigger pulse
        self._trig[idx].value = 0  # Return to LOW

    def _kalman1d(self, idx: int, z: float, dt: float = 0.06) -> float:
        """
        1D Kalman filter for distance measurement smoothing for a specific sensor.
        
        This internal method applies Kalman filtering to reduce noise in distance
        measurements. Each sensor has its own filter state to handle multiple
        sensors independently.
        
        :param idx: Index of the sensor to apply Kalman filter
        :param z: Measured distance (raw sensor reading)
        :param dt: Time step in seconds (default: 0.06)
        :return: Filtered distance estimate
        """
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
        """
        Read distance from a single sensor.
        
        This internal method performs a blocking distance measurement
        from a specific sensor if it's not in non-blocking mode.
        
        :param idx: Index of the sensor to read
        :return: Measured distance in cm or None if measurement failed
        """
        return self._measure_single_sensor(idx)
    
    def _safe_call_callback(self, idx: int, distance: float | None):
        """
        Safely call user callback function for a specific sensor.
        
        This internal method handles calling user-defined callback functions
        with proper error handling and scheduling for interrupt safety.
        
        :param idx: Index of the sensor
        :param distance: Measured distance or None if measurement failed
        """
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
        """
        Perform measurement on a single sensor (blocking mode).
        
        This internal method handles the complete measurement process including
        triggering, timing the echo pulse, and applying filtering.
        
        :param idx: Index of the sensor to measure
        :param timeout_us: Timeout for echo pulse in microseconds (default: 38000)
        :return: Measured distance in cm or None if measurement failed
        """
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
        """
        Manage the timer state based on active sensors.
        
        This internal method starts or stops the measurement timer based on
        whether any sensors are configured for non-blocking operation.
        """
        has_active_sensors = any(
            self._measurement_enabled[i] and self._nonblocking[i] 
            for i in range(len(self))
        )
        
        if has_active_sensors and not self._timer_active:
            self._start_timer()
        elif not has_active_sensors and self._timer_active:
            self._stop_timer()

    def _timer_callback(self, timer):
        """
        Timer callback for non-blocking measurements.
        
        This internal method is called periodically to trigger measurements
        on sensors configured for non-blocking operation.
        
        :param timer: Timer object (unused but required by Timer callback signature)
        """
        for i in range(len(self)):
            if (not self._nonblocking[i] or 
                not self._measurement_enabled[i] or 
                self._user_callbacks[i] is None):
                continue
            
            if self._measurement_active[i] == 0:
                self._start_measurement_for_sensor(i)

    def _start_measurement_for_sensor(self, idx: int):
        """
        Start a non-blocking measurement for a specific sensor.
        
        This internal method handles the measurement process for non-blocking
        mode, including error handling and callback execution.
        
        :param idx: Index of the sensor to measure
        """
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
        """
        Start the timer for non-blocking measurements.
        
        This internal method initializes and starts the periodic timer
        used for automatic measurements in non-blocking mode.
        """
        if not self._timer_active:
            self._timer.init(period=self._period_ms, mode=machine.Timer.PERIODIC, callback=self._timer_callback)
            self._timer_active = True

    def _stop_timer(self):
        """
        Stop the timer for non-blocking measurements.
        
        This internal method stops the periodic timer and resets all
        measurement states for clean shutdown.
        """
        if self._timer_active:
            self._timer.deinit()
            self._timer_active = False
        
        # Reset measurement states
        for i in range(len(self)):
            self._measurement_active[i] = 0

    class _SR04View:
        """
        A view class for controlling individual sensors or groups of sensors.
        
        This class provides a unified interface for controlling one or more sensors
        through the same API. It's returned by SR04.__getitem__() and allows
        seamless control of single sensors or groups using the same syntax.
        """
        @property
        def period_ms(self) -> int:
            """
            Get or set the global measurement period for non-blocking mode via view.
            This simply proxies to the parent SR04 instance for consistency.
            """
            return self._parent.period_ms

        @period_ms.setter
        def period_ms(self, ms: int):
            self._parent.period_ms = ms

        def __init__(self, parent: "SR04", indices: list[int]):
            """
            Initialize sensor view with parent reference and sensor indices.
            
            :param parent: Parent SR04 instance
            :param indices: list of sensor indices this view controls
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "SR04._SR04View":
            """
            Get a sub-view of this view for further sensor selection.
            
            :param idx: Index or slice for sub-selection
            :return: New _SR04View with selected sensors
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4), (5, 6), (7, 8)])
                >>> group = sensors[0:3]        # First three sensors
                >>> subgroup = group[1:3]       # Second and third from original
                >>> distances = subgroup.value
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return SR04._SR04View(self._parent, selected_indices)
            else:
                return SR04._SR04View(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of sensors in this view.
            
            :return: Number of sensors controlled by this view
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4), (5, 6)])
                >>> group = sensors[1:3]
                >>> print(len(group))  # Output: 2
            ```
            """
            return len(self._indices)

        def _get_values(self) -> list[int|None]:
            """
            Internal method to get measurement values.
            
            This method performs blocking measurements on all sensors
            in the view and returns the results as a list.
            
            :return: list of distances in cm or None values
            """
            results = []
            for idx in self._indices:
                value = self._parent._read_single(idx)
                if value is not None:
                    results.append(int(round(value)))
                else:
                    results.append(None)
            return results
        
        def reset_filter(self):
            """
            Reset Kalman filters for sensors in this view.
            
            This method reinitializes the Kalman filter state for all sensors
            in the view, useful when environmental conditions change significantly.
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> # After moving sensors to new environment
                >>> sensors[:].reset_filter()  # Reset all filters
                >>> sensors[0].reset_filter()  # Reset first sensor only
            ```
            """
            for i in self._indices:
                self._parent._x[i] = 0.0
                self._parent._v[i] = 0.0
                self._parent._P[i] = [[1.0, 0.0], [0.0, 1.0]]

        @property
        def measurement(self) -> list[bool]:
            """
            Get measurement state for sensors in this view.
            
            Returns whether automatic measurements are enabled for each sensor
            in non-blocking mode.
            
            :return: list of measurement state flags
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> sensors[:].nonblocking = True
                >>> print(sensors[:].measurement)  # [False, False] - no callbacks set
                >>> 
                >>> # Set callbacks and enable
                >>> sensors[:].callback = lambda pin, dist: print(f"{pin}: {dist}cm")
                >>> sensors[:].measurement = True
                >>> print(sensors[:].measurement)  # [True, True]
            ```
            """
            return [bool(self._parent._measurement_enabled[i]) for i in self._indices]

        @measurement.setter  
        def measurement(self, enable: bool | list[bool]):
            """
            Enable or disable measurements for sensors in this view.
            
            Controls automatic measurements in non-blocking mode. Sensors must
            have callbacks set and be in non-blocking mode to enable measurements.
            
            :param enable: Single boolean for all sensors or list of booleans
            
            :raises ValueError: If list length doesn't match number of sensors
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> def my_callback(pin, distance):
                ...     print(f"Sensor {pin}: {distance}cm")
                >>> 
                >>> # Setup for non-blocking measurements
                >>> sensors[:].nonblocking = True
                >>> sensors[:].callback = my_callback
                >>> 
                >>> # Enable measurements
                >>> sensors[:].measurement = True
                >>> 
                >>> # Enable only first sensor
                >>> sensors[:].measurement = [True, False]
            ```
            """
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
            """
            Get Kalman filter states for sensors in this view.
            
            Returns detailed filter state information including position, velocity,
            covariance matrix, and noise parameters for each sensor.
            
            :return: list of dictionaries with filter state information
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> states = sensors[:].filter_states
                >>> for i, state in enumerate(states):
                ...     print(f"Sensor {i}:")
                ...     print(f"  Position: {state['position']:.2f}cm")
                ...     print(f"  Velocity: {state['velocity']:.2f}cm/s")
                ...     print(f"  Measurement noise: {state['measurement_noise']}")
            ```
            """
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
            """
            Get the current distance measurement value(s) as integer(s).
            
            Performs blocking measurements and returns distances in centimeters
            rounded to the nearest integer, or None if measurement failed.
            
            :return: list of integer distances in cm or None values
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4), (5, 6)])
                >>> 
                >>> # Read all sensors
                >>> distances = sensors[:].value
                >>> print(f"All distances: {distances}")  # [25, 30, None]
                >>> 
                >>> # Read specific sensors
                >>> first_two = sensors[0:2].value
                >>> print(f"First two: {first_two}")  # [25, 30]
                >>> 
                >>> # Read single sensor
                >>> single = sensors[0].value
                >>> print(f"First sensor: {single}")  # [25]
            ```
            """
            return self._get_values()

        @property
        def nonblocking(self) -> list[bool]:
            """
            Get non-blocking mode for sensors in this view.
            
            Returns whether each sensor is configured for non-blocking
            (automatic) measurements with callbacks.
            
            :return: list of non-blocking mode flags
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> 
                >>> # Check current settings
                >>> modes = sensors[:].nonblocking
                >>> print(f"Non-blocking modes: {modes}")  # [False, False]
                >>> 
                >>> # Check specific sensor
                >>> first_mode = sensors[0].nonblocking
                >>> print(f"First sensor: {first_mode}")  # [False]
            ```
            """
            return [bool(self._parent._nonblocking[i]) for i in self._indices]

        @nonblocking.setter
        def nonblocking(self, flag: bool):
            """
            Set non-blocking mode for sensors in this view.
            
            When True, sensors can perform automatic measurements with callbacks.
            When False, sensors only work in blocking mode via value property.
            
            :param flag: Boolean flag for non-blocking mode
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> 
                >>> # Enable non-blocking for all sensors
                >>> sensors[:].nonblocking = True
                >>> 
                >>> # Mixed mode: first blocking, second non-blocking
                >>> sensors[0].nonblocking = False
                >>> sensors[1].nonblocking = True
                >>> 
                >>> # Set up callback for non-blocking sensor
                >>> sensors[1].callback = lambda pin, dist: print(f"{pin}: {dist}cm")
                >>> sensors[1].measurement = True
            ```
            """
            flag_val = 1 if flag else 0
            for i in self._indices:
                self._parent._nonblocking[i] = flag_val

        @property
        def temperature(self) -> list[float]:
            """
            Get temperatures for sensors in this view.
            
            Returns the temperature values used for speed of sound compensation
            in distance calculations.
            
            :return: list of temperatures in degrees Celsius
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> temps = sensors[:].temperature
                >>> print(f"Temperatures: {temps}")  # [20.0, 20.0]
                >>> 
                >>> # Check specific sensor temperature
                >>> first_temp = sensors[0].temperature
                >>> print(f"First sensor temp: {first_temp[0]}°C")
            ```
            """
            return [self._parent._temp_c[i] for i in self._indices]

        @temperature.setter
        def temperature(self, temp_c: float):
            """
            Set temperature for sensors in this view.
            
            Updates the temperature used for speed of sound compensation.
            Higher temperatures increase sound speed, affecting distance accuracy.
            
            :param temp_c: Temperature in degrees Celsius (-40°C to +85°C)
            
            :raises ValueError: If temperature is outside valid range
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> 
                >>> # Set temperature for all sensors
                >>> sensors[:].temperature = 25.0
                >>> 
                >>> # Different temperatures for different sensors
                >>> sensors[0].temperature = 20.0  # Indoor sensor
                >>> sensors[1].temperature = 30.0  # Outdoor sensor
                >>> 
                >>> # Verify settings
                >>> print(sensors[:].temperature)  # [20.0, 30.0]
            ```
            """
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
            """
            Get Kalman filter parameters for sensors in this view.
            
            Returns the noise parameters used in the Kalman filter for
            each sensor. Lower values provide more filtering but slower response.
            
            :return: list of dictionaries with 'R' and 'Q' parameters
            
            Example
            -------
            ```python
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> params = sensors[:].filter
                >>> print(f"Filter parameters: {params}")
                >>> # [{'R': 25.0, 'Q': 4.0}, {'R': 25.0, 'Q': 4.0}]
                >>> 
                >>> # Check specific sensor
                >>> first_filter = sensors[0].filter
                >>> print(f"R: {first_filter[0]['R']}, Q: {first_filter[0]['Q']}")
            """
            return [{'R': self._parent._R[i], 'Q': self._parent._Q[i]} 
                   for i in self._indices]

        @filter.setter
        def filter(self, params: dict):
            """
            Set Kalman filter parameters for sensors in this view.
            
            Adjusts the noise parameters for the Kalman filter:
            - R (measurement noise): Higher values trust measurements less
            - Q (process noise): Higher values allow faster changes
            
            :param params: Dictionary with 'R' and/or 'Q' keys
            
            :raises ValueError: If parameters are not positive
            :raises TypeError: If params is not a dictionary
            
            Example:
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> 
                >>> # Less filtering for faster response
                >>> sensors[:].filter = {'R': 10.0, 'Q': 8.0}
                >>> 
                >>> # More filtering for stable readings
                >>> sensors[:].filter = {'R': 50.0, 'Q': 1.0}
                >>> 
                >>> # Adjust only measurement noise
                >>> sensors[0].filter = {'R': 15.0}
                >>> 
                >>> # Different settings per sensor
                >>> sensors[0].filter = {'R': 10.0, 'Q': 2.0}  # Responsive
                >>> sensors[1].filter = {'R': 40.0, 'Q': 1.0}  # Stable
            """
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
            """
            Get callback functions for sensors in this view.
            
            Returns the callback functions that will be called when
            measurements complete in non-blocking mode.
            
            :return: list of callback functions or None values
            
            Example:
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> callbacks = sensors[:].callback
                >>> print(f"Callbacks: {callbacks}")  # [None, None]
                >>> 
                >>> # Check if callback is set
                >>> if sensors[0].callback[0] is not None:
                ...     print("Callback is set for first sensor")
            """
            return [self._parent._user_callbacks[i] for i in self._indices]

        @callback.setter
        def callback(self, fn: callable | list[callable]):
            """
            Set callback function for sensors in this view.
            
            Sets the function to be called when measurements complete in
            non-blocking mode. Callback receives (pin, distance) parameters.
            
            :param fn: Callback function, None, or list of callbacks
            
            :raises ValueError: If list length doesn't match number of sensors
            :raises TypeError: If callback is not callable or None
            
            Example:
                >>> sensors = SR04([(1, 2), (3, 4)])
                >>> 
                >>> # Simple callback for all sensors
                >>> def distance_callback(pin, distance):  # pin is the trigger pin
                ...     if distance is not None:
                ...         print(f"Sensor on pin {pin}: {distance}cm")
                ...     else:
                ...         print(f"Sensor on pin {pin}: No reading")
                >>> 
                >>> sensors[:].callback = distance_callback
                >>> 
                >>> # Different callbacks for different sensors
                >>> def sensor1_callback(pin, dist):
                ...     print(f"Front sensor: {dist}cm")
                >>> 
                >>> def sensor2_callback(pin, dist):
                ...     print(f"Back sensor: {dist}cm")
                >>> 
                >>> sensors[:].callback = [sensor1_callback, sensor2_callback]
                >>> 
                >>> # Enable non-blocking measurements 
                >>> sensors[:].nonblocking = True
                >>> sensors[:].measurement = True
            """
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
