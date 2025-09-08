from . import (
    utime, array,
    machine,
    utools, ticle
)


class ServoMotor:
    """
    Advanced servo motor controller supporting multiple servos with smooth movement.
    
    This class provides comprehensive servo motor control with support for multiple servos,
    individual calibration, smooth non-blocking movements, and unified indexing/slicing
    interface. Each servo can be controlled independently or as a group with precise
    angle control and movement timing.
    
    Features:
       
        - Multiple servo support with individual configuration
        - Smooth non-blocking movements with configurable duration
        - Individual servo calibration (min/max pulse widths)
        - Unified indexing/slicing interface for intuitive control
        - Precise angle control (0-180 degrees) with clamping
        - Movement completion detection and timeout handling
        - Safe resource management with proper cleanup
        - Efficient memory usage with array-based storage
    
    Technical Specifications:
       
        - Angle Range: 0.0° to 180.0° (automatically clamped)
        - PWM Frequency: 50Hz (configurable)
        - Pulse Width: 500µs to 2500µs (individually calibratable)
        - Movement Duration: 100ms to 65535ms (configurable per servo)
        - Update Rate: 20ms for smooth movements (50Hz)
        - Memory Efficient: Uses typed arrays for optimal performance
        
    """


    def __init__(self, pins: list[int]|tuple[int, ...], *, freq: int = 50, min_us: int = 500, max_us: int = 2500, init_angle: float = 90.0):
        """
        Initialize servo motor controller with specified pins and configuration.
        
        Creates a servo controller that manages multiple servo motors with individual
        control capabilities. Each servo is initialized to the specified angle and
        configured with the given pulse width parameters.
        
        :param pins: list or tuple of GPIO pin numbers for servo control signals
                    Example: [10, 11, 12] or (10, 11, 12, 13)
        :param freq: PWM frequency in Hz (default: 50)
                    Standard servo frequency, typically 50Hz or 60Hz
        :param min_us: Minimum pulse width in microseconds (default: 500)
                      Corresponds to 0 degrees position
        :param max_us: Maximum pulse width in microseconds (default: 2500)
                      Corresponds to 180 degrees position
        :param init_angle: Initial angle for all servos in degrees (default: 90.0)
                          Must be between 0.0 and 180.0
        
        :raises ValueError: If init_angle is outside 0.0-180.0 range
        :raises OSError: If PWM initialization fails
        :raises TypeError: If pins parameter is not list or tuple
        
        Example
        -------
        ```python
            >>> # Basic servo setup
            >>> servo = ServoMotor([10])
            >>> 
            >>> # Multiple servos with custom settings
            >>> servos = ServoMotor([10, 11, 12, 13], 
            ...                    freq=60, 
            ...                    min_us=600, 
            ...                    max_us=2400, 
            ...                    init_angle=45.0)
            >>> 
            >>> # High-precision servo setup
            >>> precision_servos = ServoMotor([15, 16], 
            ...                              min_us=400, 
            ...                              max_us=2600, 
            ...                              init_angle=0.0)
            >>> 
            >>> # Digital servo with different frequency
            >>> digital_servo = ServoMotor([20], freq=333, init_angle=90.0)
        ```
        """
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
        """
        Deinitialize servo controller and release all resources.
        
        Safely shuts down the servo controller by stopping all movements,
        deinitializing timers, and releasing PWM resources. Should be called
        when servos are no longer needed to prevent resource leaks.
        
        Note: After calling deinit(), the ServoMotor instance should not be
        used for further operations.
        
        Example
        -------
        ```python
            >>> servos = ServoMotor([10, 11, 12])
            >>> servos[0].angle = 90
            >>> servos[1].angle = 45
            >>> 
            >>> # Proper cleanup
            >>> servos.deinit()
            >>> 
            >>> # Context manager pattern (recommended)
            >>> class ServoContext:
            ...     def __init__(self, pins, **kwargs):
            ...         self.servos = ServoMotor(pins, **kwargs)
            ...     def __enter__(self):
            ...         return self.servos
            ...     def __exit__(self, exc_type, exc_val, exc_tb):
            ...         self.servos.deinit()
            >>> 
            >>> with ServoContext([10, 11]) as servos:
            ...     servos[0].angle = 90
            ...     servos[1].angle = 45
            >>> # Automatically cleaned up
        ```
        """
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
        """
        Get a view for controlling specific servo(s) through indexing or slicing.
        
        Returns a ServoMotorView that provides access to individual servos or
        groups of servos using the same interface. Supports both single indexing
        and slice notation for flexible servo selection.
        
        :param idx: Index (int) or slice for servo selection
        
            - int: Single servo index (0-based)
            - slice: Range of servos (supports start:stop:step)

        :return: _ServoMotorView instance for selected servo(s)
        
        :raises IndexError: If servo index is out of range
        :raises TypeError: If index is not int or slice
        
        Example
        -------
        ```python
            >>> servos = ServoMotor([10, 11, 12, 13])
            >>> 
            >>> # Single servo access
            >>> first_servo = servos[0]          # First servo
            >>> last_servo = servos[-1]          # Last servo
            >>> 
            >>> # Multiple servo access with slicing
            >>> first_two = servos[0:2]          # First two servos
            >>> last_two = servos[2:4]           # Last two servos
            >>> all_servos = servos[:]           # All servos
            >>> even_servos = servos[::2]        # Even-indexed servos (0, 2)
            >>> odd_servos = servos[1::2]        # Odd-indexed servos (1, 3)
            >>> 
            >>> # Nested slicing
            >>> group = servos[1:4]              # Servos 1, 2, 3
            >>> subgroup = group[0:2]            # First two from group (servos 1, 2)
        ```
        """
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
        """
        Get the number of servo motors controlled by this instance.
        
        :return: Number of servos in the controller
        
        Example
        -------
        ```python
            >>> servos = ServoMotor([10, 11, 12, 13])
            >>> print(len(servos))  # Output: 4
            >>> 
            >>> # Use in loops
            >>> for i in range(len(servos)):
            ...     servos[i].angle = i * 45  # 0°, 45°, 90°, 135°
            >>> 
            >>> # Conditional operations
            >>> if len(servos) > 2:
            ...     servos[2:].angle = 180  # Set last servos to 180°
        ```
        """
        return len(self._current_angles)

    def _compute_us(self, deg: float, idx: int) -> float:
        """
        Convert angle to pulse width in microseconds for specific servo.
        
        This internal method performs the mathematical conversion from servo
        angle to the corresponding PWM pulse width, taking into account the
        individual calibration settings for each servo.
        
        :param deg: Angle in degrees (automatically clamped to 0.0-180.0)
        :param idx: Index of the servo for calibration lookup
        :return: Pulse width in microseconds
        
        Formula: pulse_width = min_us + (max_us - min_us) * (angle / 180.0)
        """
        deg = utools.clamp(deg, 0.0, 180.0)
        span = self._max_us[idx] - self._min_us[idx]
        return self._min_us[idx] + span * deg / 180.0

    @staticmethod
    def _get_angle_list(parent, indices: list[int]) -> list[float]:
        """Get current angles for specified servos."""
        return [parent._current_angles[i] for i in indices]

    @staticmethod
    def _set_angle_single(parent, idx: int, deg: float) -> None:
        """Set target angle for a specific servo."""
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
        """Get target angles for specified servos."""
        return [parent._target_angles[i] for i in indices]

    @staticmethod
    def _get_nonblocking_list(parent, indices: list[int]) -> list[bool]:
        """Get non-blocking mode for specified servos."""
        return [bool(parent._nonblocking[i]) for i in indices]

    @staticmethod
    def _set_nonblocking_all(parent, flag: bool, indices: list[int]) -> None:
        """Set non-blocking mode for specified servos."""
        flag_val = 1 if flag else 0
        for i in indices:
            parent._nonblocking[i] = flag_val

    @staticmethod
    def _get_is_moving_list(parent, indices: list[int]) -> list[bool]:
        """Get movement status for specified servos."""
        return [bool(parent._is_moving[i]) for i in indices]

    @staticmethod
    def _get_duration_ms_list(parent, indices: list[int]) -> list[int]:
        """Get movement duration for specified servos."""
        return [parent._duration_ms[i] for i in indices]

    @staticmethod
    def _set_duration_ms_all(parent, ms: int, indices: list[int]) -> None:
        """Set movement duration for specified servos."""
        if ms <= 0:
            raise ValueError("Duration must be positive")
        ms = max(100, int(ms))
        for i in indices:
            parent._duration_ms[i] = ms

    @staticmethod
    def _get_calibration_list(parent, indices: list[int]) -> list[dict]:
        """Get calibration values for specified servos."""
        return [{'min_us': parent._min_us[i], 'max_us': parent._max_us[i]} 
                for i in indices]

    @staticmethod
    def _set_calibration_all(parent, params: dict, indices: list[int]) -> None:
        """Set calibration for specified servos."""
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
        """Wait for servos to complete their movements."""
        start_time = utime.ticks_ms()
        while any(parent._is_moving[i] for i in indices):
            if utime.ticks_diff(utime.ticks_ms(), start_time) > timeout_ms:
                return False
            utime.sleep_ms(10)
        return True

    @staticmethod
    def _stop_all(parent, indices: list[int]) -> None:
        """Stop servos immediately."""
        for i in indices:
            parent._is_moving[i] = 0
            parent._target_angles[i] = parent._current_angles[i]

    def _timer_cb(self, t) -> None:
        """Timer callback for non-blocking movement."""
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
        """
        View class for controlling individual servos or groups of servos.
        
        This class provides a unified interface for controlling one or more servos
        through the same API. It's returned by ServoMotor.__getitem__() and allows
        seamless control of single servos or groups using identical syntax.
        
        Features:
        
            - Consistent API for single servo or multi-servo control
            - Property-based interface for intuitive control
            - Automatic type conversion and validation
            - Support for both individual and batch operations
            - Movement synchronization and timing control
        
        """

        def __init__(self, parent: "ServoMotor", indices: list[int]):
            """
            Initialize servo view with parent reference and servo indices.
            
            :param parent: Parent ServoMotor instance
            :param indices: list of servo indices this view controls
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "ServoMotor._ServoMotorView":
            """
            Get a sub-view of this view for further servo selection.
            
            Allows nested selection of servos from an existing view,
            enabling fine-grained control over servo groups.
            
            :param idx: Index or slice for sub-selection
            :return: New _ServoMotorView with selected servos
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> group = servos[0:3]        # First three servos
                >>> subgroup = group[1:3]      # Second and third from original
                >>> subgroup.angle = 90        # Control sub-selection
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return ServoMotor._ServoMotorView(self._parent, selected_indices)
            else:
                return ServoMotor._ServoMotorView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of servos in this view.
            
            :return: Number of servos controlled by this view
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> group = servos[1:3]
                >>> print(len(group))  # Output: 2
            ```
            """
            return len(self._indices)

        @property
        def angle(self) -> list[float]:
            """
            Get current angles of servos in this view.
            
            Returns the current angle positions for all servos in the view.
            For non-blocking servos that are moving, this reflects the
            instantaneous position, not the target angle.
            
            :return: list of current angles in degrees (0.0-180.0)
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12])
                >>> servos[0].angle = 45
                >>> servos[1].angle = 90
                >>> servos[2].angle = 135
                >>> 
                >>> # Get individual servo angle
                >>> current = servos[0].angle
                >>> print(f"First servo: {current[0]}°")  # Output: First servo: 45.0°
                >>> 
                >>> # Get multiple servo angles
                >>> angles = servos[0:2].angle
                >>> print(f"First two: {angles}")  # Output: First two: [45.0, 90.0]
                >>> 
                >>> # Get all servo angles
                >>> all_angles = servos[:].angle
                >>> print(f"All servos: {all_angles}")  # Output: All servos: [45.0, 90.0, 135.0]
                >>> 
                >>> # Monitor moving servo
                >>> servos[0].nonblocking = True
                >>> servos[0].angle = 180  # Start movement
                >>> while servos[0].is_moving[0]:
                ...     current_pos = servos[0].angle[0]
                ...     print(f"Moving: {current_pos:.1f}°")
                ...     utime.sleep_ms(100)
            ```
            """
            return ServoMotor._get_angle_list(self._parent, self._indices)

        @angle.setter
        def angle(self, value: float | list[float]):
            """
            Set target angle for servos in this view.
            
            Sets the target angle(s) for servos. Behavior depends on the
            nonblocking mode setting:
            - Blocking mode: Servo moves immediately to target position
            - Non-blocking mode: Smooth movement over configured duration
            
            :param value: Target angle(s) in degrees
                         - float: Same angle for all servos in view
                         - list[float]: Individual angles (must match servo count)
                         - Range: 0.0 to 180.0 degrees (automatically clamped)
            
            :raises ValueError: If angles are outside valid range or list length mismatch
            :raises TypeError: If value is not float or list
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> 
                >>> # Set single servo angle
                >>> servos[0].angle = 90.0           # First servo to 90°
                >>> servos[3].angle = 45             # Last servo to 45° (int auto-converted)
                >>> 
                >>> # Set multiple servos to same angle
                >>> servos[1:3].angle = 135.0        # Servos 1&2 to 135°
                >>> servos[:].angle = 0              # All servos to 0°
                >>> 
                >>> # Set individual angles with list
                >>> servos[:].angle = [0, 45, 90, 135]  # Individual positions
                >>> servos[0:2].angle = [30, 60]        # First two servos
                >>> 
                >>> # Smooth movement example
                >>> servos[0].nonblocking = True
                >>> servos[0].duration_ms = 1000     # 1 second movement
                >>> servos[0].angle = 180            # Smooth movement to 180°
                >>> 
                >>> # Simultaneous smooth movements
                >>> servos[:].nonblocking = True
                >>> servos[:].duration_ms = 2000     # 2 second movements
                >>> servos[:].angle = [180, 0, 90, 45]  # All move simultaneously
                >>> 
                >>> # Automatic clamping
                >>> servos[0].angle = 200            # Clamped to 180°
                >>> servos[1].angle = -10            # Clamped to 0°
                >>> print(servos[:].angle)           # [180.0, 0.0, 90.0, 45.0]
            ```
            """
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
            """
            Get target angles of servos in this view.
            
            Returns the target angle positions for all servos in the view.
            For non-blocking servos that are moving, this shows the final
            destination angle, not the current position.
            
            :return: list of target angles in degrees (0.0-180.0)
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12])
                >>> 
                >>> # Set targets and check them
                >>> servos[0].angle = 45
                >>> servos[1].angle = 90
                >>> servos[2].angle = 135
                >>> 
                >>> targets = servos[:].target_angle
                >>> print(f"Targets: {targets}")  # Output: Targets: [45.0, 90.0, 135.0]
                >>> 
                >>> # For non-blocking movement, target differs from current during movement
                >>> servos[0].nonblocking = True
                >>> servos[0].duration_ms = 2000
                >>> servos[0].angle = 180           # Start movement
                >>> 
                >>> print(f"Current: {servos[0].angle[0]:.1f}°")      # Shows moving position
                >>> print(f"Target: {servos[0].target_angle[0]:.1f}°") # Shows 180.0°
                >>> 
                >>> # Monitor progress
                >>> while servos[0].is_moving[0]:
                ...     current = servos[0].angle[0]
                ...     target = servos[0].target_angle[0]
                ...     progress = (current / target) * 100
                ...     print(f"Progress: {progress:.1f}%")
                ...     utime.sleep_ms(100)
            ```
            """
            return ServoMotor._get_target_angle_list(self._parent, self._indices)

        @property
        def nonblocking(self) -> list[bool]:
            """
            Get non-blocking mode status for servos in this view.
            
            Returns whether each servo is configured for non-blocking
            (smooth) movements. Non-blocking servos move gradually to
            target positions over the configured duration.
            
            :return: list of non-blocking mode flags
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> 
                >>> # Check default mode (all blocking)
                >>> modes = servos[:].nonblocking
                >>> print(f"Modes: {modes}")  # Output: Modes: [False, False, False, False]
                >>> 
                >>> # Check individual servo
                >>> first_mode = servos[0].nonblocking
                >>> print(f"First servo: {first_mode[0]}")  # Output: First servo: False
                >>> 
                >>> # After enabling non-blocking
                >>> servos[0:2].nonblocking = True
                >>> modes = servos[:].nonblocking
                >>> print(f"Updated: {modes}")  # Output: Updated: [True, True, False, False]
                >>> 
                >>> # Use in conditions
                >>> for i, is_smooth in enumerate(servos[:].nonblocking):
                ...     mode = "Smooth" if is_smooth else "Immediate"
                ...     print(f"Servo {i}: {mode} movement")
            ```
            """
            return ServoMotor._get_nonblocking_list(self._parent, self._indices)

        @nonblocking.setter
        def nonblocking(self, flag: bool):
            """
            Set non-blocking mode for servos in this view.
            
            Controls movement behavior for servos:
            
                - True: Smooth movements over configured duration
                - False: Immediate movement to target position (default)
                
            :param flag: Non-blocking mode flag for all servos in view
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> 
                >>> # Enable smooth movement for single servo
                >>> servos[0].nonblocking = True
                >>> servos[0].duration_ms = 1500     # 1.5 second movements
                >>> servos[0].angle = 90             # Smooth movement
                >>> 
                >>> # Enable for multiple servos
                >>> servos[1:3].nonblocking = True
                >>> servos[1:3].duration_ms = 2000   # 2 second movements
                >>> servos[1:3].angle = [45, 135]    # Both move smoothly
                >>> 
                >>> # Mixed mode control
                >>> servos[0].nonblocking = True     # Smooth
                >>> servos[1].nonblocking = False    # Immediate
                >>> servos[0].angle = 180            # Moves smoothly
                >>> servos[1].angle = 0              # Moves immediately
                >>> 
                >>> # Enable all for synchronized movements
                >>> servos[:].nonblocking = True
                >>> servos[:].duration_ms = 3000     # 3 second synchronized movement
                >>> servos[:].angle = [0, 60, 120, 180]  # All move together
                >>> 
                >>> # Disable for immediate positioning
                >>> servos[:].nonblocking = False
                >>> servos[:].angle = 90             # All snap to position immediately
            ```
            """
            ServoMotor._set_nonblocking_all(self._parent, flag, self._indices)

        @property
        def is_moving(self) -> list[bool]:
            """
            Get movement status for servos in this view.
            
            Returns whether each servo is currently executing a non-blocking
            movement. Only relevant for servos in non-blocking mode.
            
            :return: list of movement status flags
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12])
                >>> 
                >>> # Check movement status
                >>> moving = servos[:].is_moving
                >>> print(f"Moving: {moving}")  # Output: Moving: [False, False, False]
                >>> 
                >>> # Start smooth movement and monitor
                >>> servos[0].nonblocking = True
                >>> servos[0].duration_ms = 2000
                >>> servos[0].angle = 180           # Start movement
                >>> 
                >>> # Monitor progress
                >>> while servos[0].is_moving[0]:
                ...     current = servos[0].angle[0]
                ...     print(f"Moving to 180°, currently at {current:.1f}°")
                ...     utime.sleep_ms(200)
                >>> print("Movement completed!")
                >>> 
                >>> # Multiple servo monitoring
                >>> servos[:].nonblocking = True
                >>> servos[:].duration_ms = 1500
                >>> servos[:].angle = [0, 90, 180]  # Start all movements
                >>> 
                >>> while any(servos[:].is_moving):
                ...     statuses = servos[:].is_moving
                ...     angles = servos[:].angle
                ...     for i, (moving, angle) in enumerate(zip(statuses, angles)):
                ...         status = "Moving" if moving else "Stopped"
                ...         print(f"Servo {i}: {status} at {angle:.1f}°")
                ...     utime.sleep_ms(300)
                >>> 
                >>> # Stop movement early if needed
                >>> if servos[0].is_moving[0]:
                ...     servos[0].stop()  # Stop immediately
            ```
            """
            return ServoMotor._get_is_moving_list(self._parent, self._indices)

        @property
        def duration_ms(self) -> list[int]:
            """
            Get movement duration for servos in this view.
            
            Returns the configured duration for non-blocking movements.
            This setting only affects servos in non-blocking mode.
            
            :return: list of movement durations in milliseconds
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> 
                >>> # Check default durations
                >>> durations = servos[:].duration_ms
                >>> print(f"Durations: {durations}")  # Output: Durations: [1000, 1000, 1000, 1000]
                >>> 
                >>> # Check individual servo duration
                >>> first_duration = servos[0].duration_ms
                >>> print(f"First servo: {first_duration[0]}ms")  # Output: First servo: 1000ms
                >>> 
                >>> # After setting different durations
                >>> servos[0].duration_ms = 500      # Fast movement
                >>> servos[1].duration_ms = 2000     # Slow movement
                >>> 
                >>> durations = servos[0:2].duration_ms
                >>> print(f"Updated: {durations}")   # Output: Updated: [500, 2000]
                >>> 
                >>> # Use in movement planning
                >>> for i, duration in enumerate(servos[:].duration_ms):
                ...     print(f"Servo {i} will take {duration}ms to move")
            ```
            """
            return ServoMotor._get_duration_ms_list(self._parent, self._indices)

        @duration_ms.setter
        def duration_ms(self, ms: int):
            """
            Set movement duration for servos in this view.
            
            Sets the time duration for non-blocking movements. Only affects
            servos when they are in non-blocking mode. Minimum duration is
            100ms to ensure smooth movement.
            
            :param ms: Movement duration in milliseconds (minimum: 100)
            
            :raises ValueError: If duration is less than 100ms
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> 
                >>> # Set fast movement for single servo
                >>> servos[0].duration_ms = 500      # 0.5 second movement
                >>> servos[0].nonblocking = True
                >>> servos[0].angle = 180            # Fast movement
                >>> 
                >>> # Set slow movement for multiple servos
                >>> servos[1:3].duration_ms = 3000   # 3 second movement
                >>> servos[1:3].nonblocking = True
                >>> servos[1:3].angle = [45, 135]    # Slow synchronized movement
                >>> 
                >>> # Different speeds for different servos
                >>> servos[0].duration_ms = 500      # Fast
                >>> servos[1].duration_ms = 1000     # Medium
                >>> servos[2].duration_ms = 2000     # Slow
                >>> servos[3].duration_ms = 4000     # Very slow
                >>> 
                >>> servos[:].nonblocking = True
                >>> servos[:].angle = 90             # All move at different speeds
                >>> 
                >>> # Choreographed movement example
                >>> # Phase 1: Quick setup (500ms)
                >>> servos[:].duration_ms = 500
                >>> servos[:].angle = [0, 0, 0, 0]
                >>> servos[:].wait_completion()
                >>> 
                >>> # Phase 2: Slow wave movement (2000ms)
                >>> servos[:].duration_ms = 2000
                >>> for i in range(4):
                ...     servos[i].angle = 180
                ...     utime.sleep_ms(200)  # Staggered start
                >>> 
                >>> # Error handling
                >>> try:
                ...     servos[0].duration_ms = 50   # Too fast
                >>> except ValueError as e:
                ...     print(f"Error: {e}")         # Error: Duration must be positive
            ```
            """
            ServoMotor._set_duration_ms_all(self._parent, ms, self._indices)

        @property
        def calibration(self) -> list[dict]:
            """
            Get calibration values for servos in this view.
            
            Returns the pulse width calibration settings for each servo.
            These values define the PWM pulse widths corresponding to
            0° and 180° positions for each servo.
            
            :return: list of dictionaries with 'min_us' and 'max_us' keys
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12])
                >>> 
                >>> # Get calibration for all servos
                >>> cal = servos[:].calibration
                >>> print(f"Calibrations: {cal}")
                >>> # Output: Calibrations: [{'min_us': 500, 'max_us': 2500}, ...]
                >>> 
                >>> # Get calibration for single servo
                >>> first_cal = servos[0].calibration
                >>> print(f"First servo: {first_cal[0]}")
                >>> # Output: First servo: {'min_us': 500, 'max_us': 2500}
                >>> 
                >>> # Check specific values
                >>> for i, cal in enumerate(servos[:].calibration):
                ...     min_pulse = cal['min_us']
                ...     max_pulse = cal['max_us']
                ...     range_us = max_pulse - min_pulse
                ...     print(f"Servo {i}: {min_pulse}-{max_pulse}µs (range: {range_us}µs)")
                >>> 
                >>> # After calibration changes
                >>> servos[0].calibration = {'min_us': 600, 'max_us': 2400}
                >>> servos[1].calibration = {'min_us': 1000, 'max_us': 2000}
                >>> 
                >>> updated_cal = servos[0:2].calibration
                >>> print(f"Updated: {updated_cal}")
                >>> # Output: Updated: [{'min_us': 600, 'max_us': 2400}, {'min_us': 1000, 'max_us': 2000}]
            ```
            """
            return ServoMotor._get_calibration_list(self._parent, self._indices)

        @calibration.setter
        def calibration(self, params: dict):
            """
            Set calibration parameters for servos in this view.
            
            Updates the pulse width calibration for precise servo control.
            Different servo models may require different pulse width ranges
            for full 0-180° rotation. Both parameters are optional.
            
            :param params: Dictionary with calibration parameters:
            
                - 'min_us': Minimum pulse width in µs (0° position)
                - 'max_us': Maximum pulse width in µs (180° position)
                Range: 100-3000µs, min_us must be < max_us
            
            :raises ValueError: If pulse widths are outside valid range or min >= max
            :raises TypeError: If params is not a dictionary
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> 
                >>> # Calibrate single servo for high-precision model
                >>> servos[0].calibration = {'min_us': 600, 'max_us': 2400}
                >>> 
                >>> # Calibrate multiple servos (same settings applied to all)
                >>> servos[1:3].calibration = {'min_us': 1000, 'max_us': 2000}
                >>> 
                >>> # Partial calibration (only adjust one parameter)
                >>> servos[0].calibration = {'min_us': 550}  # Only change minimum
                >>> servos[1].calibration = {'max_us': 2300} # Only change maximum
                >>> 
                >>> # Full range calibration for digital servos
                >>> servos[:].calibration = {'min_us': 500, 'max_us': 2500}
                >>> 
                >>> # Narrow range for limited rotation servo
                >>> servos[3].calibration = {'min_us': 1200, 'max_us': 1800}
                >>> 
                >>> # Test calibration with movement
                >>> servos[0].angle = 0       # Move to minimum position
                >>> utime.sleep_ms(1000)
                >>> servos[0].angle = 180     # Move to maximum position
                >>> 
                >>> # Fine-tune if needed
                >>> if servo_not_quite_at_zero:
                ...     current_cal = servos[0].calibration[0]
                ...     servos[0].calibration = {'min_us': current_cal['min_us'] + 10}
                >>> 
                >>> # Different servos, different calibrations
                >>> # Standard servo
                >>> servos[0].calibration = {'min_us': 500, 'max_us': 2500}
                >>> # High-precision servo  
                >>> servos[1].calibration = {'min_us': 600, 'max_us': 2400}
                >>> # Digital servo with extended range
                >>> servos[2].calibration = {'min_us': 400, 'max_us': 2600}
                >>> # Continuous rotation servo (center position)
                >>> servos[3].calibration = {'min_us': 1400, 'max_us': 1600}
            ```
            """
            ServoMotor._set_calibration_all(self._parent, params, self._indices)

        def wait_completion(self, timeout_ms: int = 10000) -> bool:
            """
            Wait for servos in this view to complete their movements.
            
            Blocks execution until all servos in the view finish their
            non-blocking movements or the timeout expires. Useful for
            synchronizing movement sequences.
            
            :param timeout_ms: Maximum time to wait in milliseconds (default: 10000)
            :return: True if all movements completed, False if timeout occurred
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> servos[:].nonblocking = True
                >>> servos[:].duration_ms = 2000
                >>> 
                >>> # Start movement and wait for completion
                >>> servos[:].angle = [0, 60, 120, 180]
                >>> if servos[:].wait_completion(timeout_ms=3000):
                ...     print("All servos reached target positions")
                >>> else:
                ...     print("Timeout - some servos still moving")
                >>> 
                >>> # Sequential movements with completion waiting
                >>> positions = [[0, 0, 0, 0], [90, 90, 90, 90], [180, 180, 180, 180]]
                >>> for pos in positions:
                ...     servos[:].angle = pos
                ...     servos[:].wait_completion(timeout_ms=3000)
                ...     print(f"Reached position: {pos}")
                ...     utime.sleep_ms(500)  # Pause between movements
                >>> 
                >>> # Individual servo completion
                >>> servos[0].angle = 45
                >>> servos[1].angle = 135
                >>> 
                >>> # Wait for first servo only
                >>> if servos[0].wait_completion(timeout_ms=2000):
                ...     print("First servo completed")
                >>> 
                >>> # Wait for specific group
                >>> if servos[1:3].wait_completion(timeout_ms=2000):
                ...     print("Servos 1 and 2 completed")
                >>> 
                >>> # Error handling with timeout
                >>> servos[0].duration_ms = 5000  # Very slow movement
                >>> servos[0].angle = 180
                >>> 
                >>> if not servos[0].wait_completion(timeout_ms=2000):
                ...     print("Movement taking too long, stopping...")
                ...     servos[0].stop()
                >>> 
                >>> # Choreographed sequence
                >>> def dance_sequence():
                ...     moves = [
                ...         [0, 0, 0, 0],
                ...         [90, 0, 0, 0],
                ...         [90, 90, 0, 0],
                ...         [90, 90, 90, 0],
                ...         [90, 90, 90, 90]
                ...     ]
                ...     for move in moves:
                ...         servos[:].angle = move
                ...         servos[:].wait_completion(timeout_ms=2000)
                ...         utime.sleep_ms(200)
                >>> 
                >>> dance_sequence()
            ```
            """
            return ServoMotor._wait_completion_all(self._parent, self._indices, timeout_ms)

        def stop(self):
            """
            Stop servos in this view immediately.
            
            Immediately halts any ongoing non-blocking movements for servos
            in this view. Servos will hold their current position as the
            new target position.
            
            Example
            -------
            ```python
                >>> servos = ServoMotor([10, 11, 12, 13])
                >>> servos[:].nonblocking = True
                >>> servos[:].duration_ms = 5000  # Very slow movement
                >>> 
                >>> # Start long movement
                >>> servos[:].angle = 180
                >>> 
                >>> # Let it run for a bit
                >>> utime.sleep_ms(1000)
                >>> 
                >>> # Stop all servos
                >>> servos[:].stop()
                >>> print("All movements stopped")
                >>> 
                >>> # Check where they stopped
                >>> final_positions = servos[:].angle
                >>> print(f"Stopped at: {final_positions}")
                >>> 
                >>> # Emergency stop example
                >>> def emergency_stop_all():
                ...     servos[:].stop()
                ...     print("EMERGENCY STOP - All servos halted")
                >>> 
                >>> # Conditional stop
                >>> servos[0].angle = 180
                >>> utime.sleep_ms(500)
                >>> 
                >>> if some_emergency_condition:
                ...     servos[0].stop()
                ...     print("Stopped due to emergency condition")
                >>> 
                >>> # Stop specific servos in a group
                >>> servos[0:2].nonblocking = True
                >>> servos[2:4].nonblocking = True
                >>> servos[:].angle = [0, 45, 90, 135]  # All start moving
                >>> 
                >>> utime.sleep_ms(800)
                >>> servos[0:2].stop()  # Stop first two, let others continue
                >>> 
                >>> # Gradual stop (stop servos one by one)
                >>> for i in range(4):
                ...     utime.sleep_ms(200)
                ...     servos[i].stop()
                ...     print(f"Stopped servo {i}")
            ```
            """
            ServoMotor._stop_all(self._parent, self._indices)
