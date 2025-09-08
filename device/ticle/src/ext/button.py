from . import (
    utime,
    machine, micropython,
    ticle
)


class Button:
    """
    Advanced button controller with configurable active logic and precise event detection.
    
    This class provides comprehensive button functionality with support for both
    active-high and active-low buttons, precise timing control, and reliable
    event detection for click, double-click, and long-press patterns.
    
    Button Events:
    
        - CLICKED: Short press and release (default: 50-500ms press duration)
        - DOUBLE_CLICKED: Two clicks within time window (default: 300ms between clicks)
        - LONG_PRESSED: Extended press (default: 800ms+ press duration)
        
    Active Logic:
    
        - ACTIVE_HIGH: Button active when GPIO is HIGH (3.3V)
        - ACTIVE_LOW: Button active when GPIO is LOW (0V, default for pull-up)
    
    """
    CLICKED = 1
    DOUBLE_CLICKED = 2
    LONG_PRESSED = 3
    PRESSED = 4
    RELEASED = 5
    
    ACTIVE_HIGH = True
    ACTIVE_LOW = False

    
    def __init__(self, pins: list[int] | tuple[int, ...]):
        """
        Initialize button controller for multiple GPIO pins.
        
        Creates button instances with default active-low configuration and
        human-optimized timing parameters for reliable button detection.
        
        :param pins: list or tuple of GPIO pin numbers for buttons
        
        Example
        -------
        ```python
            >>> # Single button setup
            >>> button = Button([12])
            >>> button[0].active_type = Button.ACTIVE_LOW
            >>> button[0].on_clicked = lambda pin: print(f"Button {pin} clicked!")
            >>> 
            >>> # Multiple buttons
            >>> buttons = Button([10, 11, 12, 13])
            >>> buttons[:].active_type = Button.ACTIVE_LOW
            >>> buttons[:].measurement = True
        ```
        """
        if not pins:
            raise ValueError("At least one pin must be provided")
            
        self._pins = list(pins)
        n = len(self._pins)
        
        self._din = ticle.Din(self._pins)
        
        self._debounce_ms = [20] * n              # Minimum press time for click (debounce)
        self._double_click_window_ms = [300] * n  # Time window for second click
        self._long_press_threshold_ms = [800] * n # Long press threshold
        
        self._active_type = [Button.ACTIVE_HIGH] * n
        self._last = [0] * n
        self._click_latch = [0] * n
        
        self._current_state = [False] * n    # Current button state (active/inactive)
        self._press_start_time = [0] * n     # When button was pressed
        self._last_release_time = [0] * n    # When button was last released
        self._waiting_for_double = [False] * n # Waiting for second click
        self._long_press_fired = [False] * n   # Long press already fired
        
        self._on_clicked = [None] * n
        self._on_double_clicked = [None] * n
        self._on_long_pressed = [None] * n
        self._on_pressed = [None] * n
        self._on_released = [None] * n
        
        self._measurement_enabled = [False] * n
        
        self._timers = [None] * n
        
        self._din[:].callback = self._button_interrupt_handler
        self._din[:].edge = ticle.Din.CB_FALLING | ticle.Din.CB_RISING
        self._sync_last()
        
    def deinit(self) -> None:
        """
        Deinitialize all buttons and release resources.
        
        Stops all measurements, clears timers, and releases GPIO resources.
        Should be called when buttons are no longer needed.
        
        Example
        -------
        ```python
            >>> buttons = Button([10, 11, 12])
            >>> # ... use buttons ...
            >>> buttons.deinit()  # Clean shutdown
            >>> 
            >>> # With exception handling
            >>> try:
            ...     buttons = Button([10, 11])
            ...     # ... button operations ...
            >>> finally:
            ...     buttons.deinit()  # Ensure cleanup
        ```
        """
        try:
            for i in range(len(self._pins)):
                self._measurement_enabled[i] = False
                if self._timers[i] is not None:
                    try:
                        self._timers[i].deinit()
                        self._timers[i] = None
                    except:
                        pass
            
            self._din.deinit()            
        except:
            pass

    def __getitem__(self, idx: int | slice) -> "_ButtonView":
        """
        Get a view for controlling specific button(s).
        
        Supports both single indexing and slice notation for flexible
        button selection and control.
        
        :param idx: Index (int) or slice for button selection
        :return: _ButtonView instance for selected button(s)
        
        :raises IndexError: If button index is out of range
        :raises TypeError: If index is not int or slice
        
        Example
        -------
        ```python
            >>> buttons = Button([10, 11, 12, 13])
            >>> 
            >>> # Single button access
            >>> first_button = buttons[0]
            >>> last_button = buttons[-1]
            >>> 
            >>> # Multiple button access with slicing
            >>> first_two = buttons[0:2]
            >>> last_two = buttons[2:4]
            >>> all_buttons = buttons[:]
            >>> 
            >>> # Configure individual or groups
            >>> buttons[0].active_type = Button.ACTIVE_HIGH
            >>> buttons[1:3].debounce_ms = 50
            >>> buttons[:].measurement = True
        ```
        """
        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._pins))))
            return Button._ButtonView(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._pins)):
                raise IndexError("Button index out of range")
            return Button._ButtonView(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        """
        Get the number of buttons configured.
        
        Returns the total number of buttons in this controller instance.
        
        :return: Number of buttons configured
        
        Example
        -------
        ```python
            >>> buttons = Button([10, 11, 12, 13])
            >>> print(len(buttons))  # Output: 4
            >>> 
            >>> # Use in loops
            >>> for i in range(len(buttons)):
            ...     buttons[i].on_clicked = lambda pin: print(f"Button {pin} clicked")
            >>> 
            >>> # Conditional operations based on count
            >>> if len(buttons) > 2:
            ...     buttons[2:].long_press_threshold_ms = 1000
        ```
        """
        return len(self._pins)

    def _button_interrupt_handler(self, pin_num: int, rising: bool) -> None:
        """
        Internal interrupt handler for button state changes.
        
        Handles both active-high and active-low button logic with precise
        timing for reliable event detection.
        
        :param pin_num: GPIO pin number that triggered interrupt
        :param rising: True for rising edge, False for falling edge
        """
        try:
            pin_idx = self._pins.index(pin_num)
            
            if not self._measurement_enabled[pin_idx]:
                return
                
            current_time = utime.ticks_ms()
            active_type = self._active_type[pin_idx]
            
            if active_type == Button.ACTIVE_HIGH:
                button_active = rising  # button pressed for active-high
            else:  # ACTIVE_LOW
                button_active = not rising  #button pressed for active-low
            
            if button_active:
                self._click_latch[pin_idx] = 1
                self._handle_button_press(pin_idx, current_time)
            else:
                self._handle_button_release(pin_idx, current_time)
                
        except (ValueError, IndexError):
            pass

    def _handle_button_press(self, pin_idx: int, current_time: int) -> None:
        """Handle button press event with state management."""
        self._current_state[pin_idx] = True
        self._press_start_time[pin_idx] = current_time
        self._long_press_fired[pin_idx] = False
        
        # Call press callback
        if self._on_pressed[pin_idx]:
            try:
                micropython.schedule(
                    lambda _: self._on_pressed[pin_idx](self._pins[pin_idx]), 0
                )
            except RuntimeError:
                try:
                    self._on_pressed[pin_idx](self._pins[pin_idx])
                except:
                    pass
        
        self._setup_long_press_timer(pin_idx)

    def _handle_button_release(self, pin_idx: int, current_time: int) -> None:
        """Handle button release event with timing analysis."""
        if not self._current_state[pin_idx]:
            return  # Ignore if already released
            
        self._current_state[pin_idx] = False
        press_duration = utime.ticks_diff(current_time, self._press_start_time[pin_idx])
        
        self._cancel_timer(pin_idx)
        
        if self._on_released[pin_idx]:
            try:
                micropython.schedule(
                    lambda _: self._on_released[pin_idx](self._pins[pin_idx]), 0
                )
            except RuntimeError:
                try:
                    self._on_released[pin_idx](self._pins[pin_idx])
                except:
                    pass
        
        if self._long_press_fired[pin_idx]:
            return
        
        min_press = self._debounce_ms[pin_idx]
        long_press_threshold = self._long_press_threshold_ms[pin_idx]
        
        if min_press <= press_duration < long_press_threshold:
            self._process_click(pin_idx, current_time)
        
        self._last_release_time[pin_idx] = current_time

    def _process_click(self, pin_idx: int, current_time: int) -> None:
        """Process click event and handle double-click detection."""
        if self._waiting_for_double[pin_idx]:
            # Second click detected - fire double click
            self._waiting_for_double[pin_idx] = False
            self._cancel_timer(pin_idx)
            
            if self._on_double_clicked[pin_idx]:
                try:
                    micropython.schedule(
                        lambda _: self._on_double_clicked[pin_idx](self._pins[pin_idx]), 0
                    )
                except RuntimeError:
                    try:
                        self._on_double_clicked[pin_idx](self._pins[pin_idx])
                    except:
                        pass
        else:
            self._waiting_for_double[pin_idx] = True
            self._setup_double_click_timer(pin_idx)

    def _setup_long_press_timer(self, pin_idx: int) -> None:
        """Setup timer for long press detection."""
        def long_press_callback(timer):
            try:
                if self._current_state[pin_idx]:  # Still pressed
                    self._long_press_fired[pin_idx] = True
                    if self._on_long_pressed[pin_idx]:
                        try:
                            micropython.schedule(
                                lambda _: self._on_long_pressed[pin_idx](self._pins[pin_idx]), 0
                            )
                        except RuntimeError:
                            try:
                                self._on_long_pressed[pin_idx](self._pins[pin_idx])
                            except:
                                pass
            except:
                pass
                    
        try:
            self._cancel_timer(pin_idx)
            self._timers[pin_idx] = machine.Timer()
            self._timers[pin_idx].init(
                period=self._long_press_threshold_ms[pin_idx],
                mode=machine.Timer.ONE_SHOT,
                callback=long_press_callback
            )
        except:
            pass

    def _setup_double_click_timer(self, pin_idx: int) -> None:
        """Setup timer for double-click window timeout."""
        def double_click_timeout(timer):
            try:
                self._waiting_for_double[pin_idx] = False
                if self._on_clicked[pin_idx]:
                    try:
                        micropython.schedule(
                            lambda _: self._on_clicked[pin_idx](self._pins[pin_idx]), 0
                        )
                    except RuntimeError:
                        try:
                            self._on_clicked[pin_idx](self._pins[pin_idx])
                        except:
                            pass
            except:
                pass
        
        try:
            self._cancel_timer(pin_idx)
            self._timers[pin_idx] = machine.Timer()
            self._timers[pin_idx].init(
                period=self._double_click_window_ms[pin_idx],
                mode=machine.Timer.ONE_SHOT,
                callback=double_click_timeout
            )
        except:
            pass

    def _cancel_timer(self, pin_idx: int) -> None:
        """Cancel active timer for button."""
        if self._timers[pin_idx] is not None:
            try:
                self._timers[pin_idx].deinit()
                self._timers[pin_idx] = None
            except:
                pass

    def _raw_read(self, idx: int) -> int:
        """Read raw digital level as 0/1 from the underlying machine.Pin."""
        try:
            return 1 if self._din.pins[idx].value() else 0
        except Exception:
            vlist = self._din[idx].value
            v = vlist[0] if isinstance(vlist, (list, tuple)) else vlist
            return 1 if v else 0

    def _sync_last(self, indices=None) -> None:
        if indices is None:
            indices = range(len(self._pins))
        for i in indices:
            self._last[i] = self._raw_read(i)

    @staticmethod
    def _get_click_list(parent, indices: list[int]) -> list[bool]:
        click_states = []
        for idx in indices:
            if parent._click_latch[idx]:
                parent._click_latch[idx] = 0
                parent._last[idx] = parent._raw_read(idx)
                click_states.append(True)
                continue

            current = parent._raw_read(idx)
            last = parent._last[idx]
            if parent._active_type[idx] == Button.ACTIVE_HIGH:
                curr_act, last_act = current, last
            else:
                curr_act, last_act = 1 - current, 1 - last
            clicked = (last_act == 0) and (curr_act == 1)
            click_states.append(clicked)
            parent._last[idx] = current
        return click_states

    @staticmethod
    def _get_active_type_list(parent, indices: list[int]) -> list[bool]:
        return [parent._active_type[i] for i in indices]

    @staticmethod
    def _set_active_type_all(parent, active_type: bool, indices: list[int]) -> None:
        pull_din = ticle.Din.PULL_DOWN if active_type else ticle.Din.PULL_UP
        for i in indices:
            parent._active_type[i] = active_type
            parent._din[i].pull = pull_din
        try:
            utime.sleep_ms(1) 
        except:
            pass
        parent._sync_last(indices)

    @staticmethod
    def _get_measurement_list(parent, indices: list[int]) -> list[bool]:
        return [parent._measurement_enabled[i] for i in indices]

    @staticmethod
    def _set_measurement_all(parent, enabled: bool, indices: list[int]) -> None:
        turned_on = []
        for i in indices:
            if not parent._measurement_enabled[i] and enabled:
                turned_on.append(i)
            parent._measurement_enabled[i] = enabled
            parent._din[i].measurement = enabled

        if turned_on:
            parent._sync_last(turned_on)

    @staticmethod
    def _get_debounce_ms_list(parent, indices: list[int]) -> list[int]:
        return [parent._debounce_ms[i] for i in indices]

    @staticmethod
    def _set_debounce_ms_all(parent, ms: int, indices: list[int]) -> None:
        for i in indices:
            parent._debounce_ms[i] = ms
            
    @staticmethod
    def _get_double_click_window_ms_list(parent, indices: list[int]) -> list[int]:
        return [parent._double_click_window_ms[i] for i in indices]

    @staticmethod
    def _set_double_click_window_ms_all(parent, ms: int, indices: list[int]) -> None:
        for i in indices:
            parent._double_click_window_ms[i] = ms

    @staticmethod
    def _get_long_press_threshold_ms_list(parent, indices: list[int]) -> list[int]:
        return [parent._long_press_threshold_ms[i] for i in indices]

    @staticmethod
    def _set_long_press_threshold_ms_all(parent, ms: int, indices: list[int]) -> None:
        for i in indices:
            parent._long_press_threshold_ms[i] = ms

    @staticmethod
    def _get_on_clicked_list(parent, indices: list[int]) -> list[callable]:
        return [parent._on_clicked[i] for i in indices]

    @staticmethod
    def _set_on_clicked_all(parent, callback: callable, indices: list[int]) -> None:
        for i in indices:
            parent._on_clicked[i] = callback

    @staticmethod
    def _get_on_double_clicked_list(parent, indices: list[int]) -> list[callable]:
        return [parent._on_double_clicked[i] for i in indices]

    @staticmethod
    def _set_on_double_clicked_all(parent, callback: callable, indices: list[int]) -> None:
        for i in indices:
            parent._on_double_clicked[i] = callback

    @staticmethod
    def _get_on_long_pressed_list(parent, indices: list[int]) -> list[callable]:
        return [parent._on_long_pressed[i] for i in indices]

    @staticmethod
    def _set_on_long_pressed_all(parent, callback: callable, indices: list[int]) -> None:
        for i in indices:
            parent._on_long_pressed[i] = callback

    @staticmethod
    def _get_on_pressed_list(parent, indices: list[int]) -> list[callable]:
        return [parent._on_pressed[i] for i in indices]

    @staticmethod
    def _set_on_pressed_all(parent, callback: callable, indices: list[int]) -> None:
        for i in indices:
            parent._on_pressed[i] = callback

    @staticmethod
    def _get_on_released_list(parent, indices: list[int]) -> list[callable]:
        return [parent._on_released[i] for i in indices]

    @staticmethod
    def _set_on_released_all(parent, callback: callable, indices: list[int]) -> None:
        for i in indices:
            parent._on_released[i] = callback

    class _ButtonView:
        """View class for controlling individual buttons or groups of buttons."""

        def __init__(self, parent: "Button", indices: list[int]):
            """
            Initialize button view with parent reference and button indices.
            
            :param parent: Parent Button instance
            :param indices: list of button indices this view controls
            
            Example
            -------
            ```python
                >>> # ButtonView is typically created automatically
                >>> buttons = Button([10, 11, 12])
                >>> view = buttons[0:2]  # Creates _ButtonView for first two buttons
                >>> print(type(view))    # <class '_ButtonView'>
            ```
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int | slice) -> "Button._ButtonView":
            """
            Get a sub-view of this view for further button selection.
            
            Enables nested selection and fine-grained control over button groups.
            
            :param idx: Index or slice for sub-selection
            :return: New _ButtonView with selected buttons
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12, 13])
                >>> group = buttons[0:3]        # First three buttons
                >>> subgroup = group[1:3]       # Second and third from original
                >>> subgroup.debounce_ms = 30   # Configure sub-selection
                >>> 
                >>> # Chain selections
                >>> buttons[1:4][0:2].measurement = True  # Buttons 1 and 2
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Button._ButtonView(self._parent, selected_indices)
            else:
                return Button._ButtonView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of buttons in this view.
            
            Returns the count of buttons controlled by this view instance.
            
            :return: Number of buttons in this view
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12, 13])
                >>> group = buttons[1:3]
                >>> print(len(group))  # Output: 2
                >>> 
                >>> # Use in operations
                >>> if len(group) > 1:
                ...     group.on_clicked = lambda pin: print(f"Group button {pin}")
            ```
            """
            return len(self._indices)

        @property
        def active_type(self) -> list[bool]:
            """
            Get active logic type for buttons in this view.
            
            Returns the active logic configuration for each button.
            True = ACTIVE_HIGH, False = ACTIVE_LOW.
            
            :return: list of active logic types
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> buttons[0].active_type = Button.ACTIVE_HIGH
                >>> buttons[1:3].active_type = Button.ACTIVE_LOW
                >>> 
                >>> # Check current settings
                >>> types = buttons[:].active_type
                >>> print(types)  # [True, False, False]
                >>> 
                >>> # Verify specific button
                >>> if buttons[0].active_type[0] == Button.ACTIVE_HIGH:
                ...     print("First button is active-high")
            ```
            """
            return Button._get_active_type_list(self._parent, self._indices)

        @active_type.setter
        def active_type(self, logic_type: bool):
            """
            Set active logic type for buttons in this view.
            
            Configures whether buttons are active-high or active-low.
            Automatically adjusts pull resistors accordingly.
            
            :param logic_type: Button.ACTIVE_HIGH or Button.ACTIVE_LOW
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Set all buttons to active-low (pull-up)
                >>> buttons[:].active_type = Button.ACTIVE_LOW
                >>> 
                >>> # Mixed configuration
                >>> buttons[0].active_type = Button.ACTIVE_HIGH   # Pull-down
                >>> buttons[1:3].active_type = Button.ACTIVE_LOW  # Pull-up
                >>> 
                >>> # For different button types
                >>> # Standard momentary buttons (active-low)
                >>> buttons[0:2].active_type = Button.ACTIVE_LOW
                >>> # Special high-side switches (active-high)
                >>> buttons[2].active_type = Button.ACTIVE_HIGH
            ```
            """
            Button._set_active_type_all(self._parent, logic_type, self._indices)

        @property
        def measurement(self) -> list[bool]:
            """
            Get measurement state for buttons in this view.
            
            Returns whether button detection/measurement is enabled for each button.
            
            :return: list of measurement state flags
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current measurement states
                >>> states = buttons[:].measurement
                >>> print(states)  # [False, False, False] - initially disabled
                >>> 
                >>> # Check specific button
                >>> if buttons[0].measurement[0]:
                ...     print("First button measurement is enabled")
                >>> else:
                ...     print("First button measurement is disabled")
            ```
            """
            return Button._get_measurement_list(self._parent, self._indices)

        @measurement.setter
        def measurement(self, enabled: bool):
            """
            Enable or disable measurement for buttons in this view.
            
            Controls whether button events are detected and callbacks are triggered.
            Must be enabled for button functionality to work.
            
            :param enabled: True to enable button detection, False to disable
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Set up callbacks first
                >>> buttons[0].on_clicked = lambda pin: print(f"Button {pin} clicked")
                >>> buttons[1].on_double_clicked = lambda pin: print(f"Double click {pin}")
                >>> 
                >>> # Enable measurement to start detection
                >>> buttons[:].measurement = True
                >>> 
                >>> # Temporarily disable specific button
                >>> buttons[1].measurement = False  # Disable button 1
                >>> buttons[1].measurement = True   # Re-enable button 1
                >>> 
                >>> # Disable all for power saving
                >>> buttons[:].measurement = False
            ```
            """
            Button._set_measurement_all(self._parent, enabled, self._indices)

        @property
        def debounce_ms(self) -> list[int]:
            """
            Get minimum press time for valid click detection.
            
            Returns the debounce time in milliseconds for each button.
            This is the minimum time a button must be pressed to register.
            
            :return: list of debounce times in milliseconds
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current debounce settings
                >>> debounce_times = buttons[:].debounce_ms
                >>> print(debounce_times)  # [20, 20, 20] - default values
                >>> 
                >>> # Check specific button
                >>> first_debounce = buttons[0].debounce_ms[0]
                >>> print(f"First button debounce: {first_debounce}ms")
            ```
            """
            return Button._get_debounce_ms_list(self._parent, self._indices)

        @debounce_ms.setter
        def debounce_ms(self, ms: int):
            """
            Set minimum press time for valid click detection.
            
            Sets the debounce time to filter out noise and accidental triggers.
            Higher values provide more stability but reduce responsiveness.
            
            :param ms: Minimum press time in milliseconds (recommended: 10-100ms)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Set conservative debounce for noisy environment
                >>> buttons[:].debounce_ms = 50
                >>> 
                >>> # Different settings for different button types
                >>> buttons[0].debounce_ms = 20   # Responsive for UI
                >>> buttons[1].debounce_ms = 100  # Conservative for machinery
                >>> 
                >>> # Quick response for gaming
                >>> gaming_buttons = Button([14, 15])
                >>> gaming_buttons[:].debounce_ms = 10
            ```
            """
            Button._set_debounce_ms_all(self._parent, ms, self._indices)

        @property
        def double_click_window_ms(self) -> list[int]:
            """
            Get double-click detection window time.
            
            Returns the maximum time allowed between two clicks for 
            double-click detection.
            
            :return: list of double-click window times in milliseconds
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current double-click settings
                >>> windows = buttons[:].double_click_window_ms
                >>> print(windows)  # [300, 300, 300] - default values
                >>> 
                >>> # Check specific button timing
                >>> window = buttons[0].double_click_window_ms[0]
                >>> print(f"Double-click window: {window}ms")
            ```
            """
            return Button._get_double_click_window_ms_list(self._parent, self._indices)

        @double_click_window_ms.setter
        def double_click_window_ms(self, ms: int):
            """
            Set time window for double-click detection.
            
            Sets the maximum time between two clicks to register as a double-click.
            Shorter times require faster clicking but reduce accidental double-clicks.
            
            :param ms: Time window in milliseconds (recommended: 200-500ms)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Fast double-click for responsive UI
                >>> buttons[0].double_click_window_ms = 200
                >>> 
                >>> # Slower double-click for accessibility
                >>> buttons[1].double_click_window_ms = 500
                >>> 
                >>> # Standard setting for most applications
                >>> buttons[2].double_click_window_ms = 300
                >>> 
                >>> # Gaming setup - very fast double-clicks
                >>> gaming_button = Button([20])
                >>> gaming_button[0].double_click_window_ms = 150
            ```
            """
            Button._set_double_click_window_ms_all(self._parent, ms, self._indices)

        @property
        def long_press_threshold_ms(self) -> list[int]:
            """
            Get long press threshold time.
            
            Returns the minimum time a button must be held for 
            long-press detection.
            
            :return: list of long press thresholds in milliseconds
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current long-press settings
                >>> thresholds = buttons[:].long_press_threshold_ms
                >>> print(thresholds)  # [800, 800, 800] - default values
                >>> 
                >>> # Check specific button
                >>> threshold = buttons[0].long_press_threshold_ms[0]
                >>> print(f"Long press threshold: {threshold}ms")
            ```
            """
            return Button._get_long_press_threshold_ms_list(self._parent, self._indices)

        @long_press_threshold_ms.setter
        def long_press_threshold_ms(self, ms: int):
            """
            Set threshold time for long press detection.
            
            Sets the minimum time a button must be held to trigger a long-press event.
            Longer times prevent accidental long-presses but reduce responsiveness.
            
            :param ms: Long press threshold in milliseconds (recommended: 500-2000ms)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Quick long-press for power users
                >>> buttons[0].long_press_threshold_ms = 500
                >>> 
                >>> # Conservative long-press to avoid accidents
                >>> buttons[1].long_press_threshold_ms = 1500
                >>> 
                >>> # Different functions, different timings
                >>> power_button = Button([20])
                >>> power_button[0].long_press_threshold_ms = 2000  # Safety
                >>> 
                >>> menu_button = Button([21])
                >>> menu_button[0].long_press_threshold_ms = 800   # Responsive
            ```
            """
            Button._set_long_press_threshold_ms_all(self._parent, ms, self._indices)

        @property
        def value(self) -> list[int]:
            return [self._parent._raw_read(i) for i in self._indices]

        @property
        def pressed(self) -> list[bool]:
            out = []
            for i in self._indices:
                raw = self._parent._raw_read(i)
                if self._parent._active_type[i] == Button.ACTIVE_HIGH:
                    out.append(bool(raw))     # HIGH = pressed
                else:
                    out.append(bool(1 - raw)) # LOW  = pressed
            return out

        @property
        def click(self) -> list[bool]:
            """
            Get current click state of buttons in this view using edge detection.

            Returns the real-time click state of each button based on edge detection.
            True = clicked (transition from inactive to active), False = not clicked.
            This is a blocking read that detects state changes since last call.
            
            :return: list of click state flags based on edge detection
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check for click events (blocking read)
                >>> while True:
                ...     clicks = buttons[:].click
                ...     for i, clicked in enumerate(clicks):
                ...         if clicked:
                ...             print(f"Button {i} was clicked!")
                ...     utime.sleep_ms(50)
                >>> 
                >>> # Check specific button click
                >>> if buttons[0].click[0]:
                ...     print("First button clicked!")
                >>> 
                >>> # Simple click detection loop
                >>> while True:
                ...     if any(buttons[:].click):
                ...         print("A button was pressed!")
                ...         break
                ...     utime.sleep_ms(10)
            ```
            """
            return Button._get_click_list(self._parent, self._indices)

        @property
        def on_clicked(self) -> list[callable]:
            """
            Get click callback functions.
            
            Returns the callback functions that will be called when
            buttons are clicked (short press and release).
            
            :return: list of callback functions or None values
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current callbacks
                >>> callbacks = buttons[:].on_clicked
                >>> print(callbacks)  # [None, None, None] - no callbacks set
                >>> 
                >>> # Check if specific button has callback
                >>> if buttons[0].on_clicked[0] is not None:
                ...     print("First button has click callback")
                >>> else:
                ...     print("First button has no click callback")
            ```
            """
            return Button._get_on_clicked_list(self._parent, self._indices)

        @on_clicked.setter
        def on_clicked(self, callback: callable):
            """
            Set click callback function.
            
            Sets the function to be called when buttons are clicked.
            Callback receives the pin number as a parameter.
            
            :param callback: Function to call on click (receives pin number)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Simple callback for all buttons
                >>> def handle_click(pin):
                ...     print(f"Button on pin {pin} was clicked!")
                >>> 
                >>> buttons[:].on_clicked = handle_click
                >>> 
                >>> # Different callbacks for different buttons
                >>> def menu_click(pin):
                ...     print("Menu button clicked")
                >>> 
                >>> def action_click(pin):
                ...     print("Action button clicked")
                >>> 
                >>> buttons[0].on_clicked = menu_click
                >>> buttons[1].on_clicked = action_click
                >>> 
                >>> # Lambda functions for simple actions
                >>> buttons[2].on_clicked = lambda pin: print(f"Pin {pin}: Click!")
                >>> 
                >>> # Enable measurement to activate callbacks
                >>> buttons[:].measurement = True
            ```
            """
            Button._set_on_clicked_all(self._parent, callback, self._indices)

        @property
        def on_double_clicked(self) -> list[callable]:
            """
            Get double-click callback functions.
            
            Returns the callback functions that will be called when
            buttons are double-clicked (two quick clicks).
            
            :return: list of callback functions or None values
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current double-click callbacks
                >>> callbacks = buttons[:].on_double_clicked
                >>> print(callbacks)  # [None, None, None] - no callbacks set
                >>> 
                >>> # Verify callback is set
                >>> if buttons[0].on_double_clicked[0] is not None:
                ...     print("First button has double-click callback")
            ```
            """
            return Button._get_on_double_clicked_list(self._parent, self._indices)

        @on_double_clicked.setter
        def on_double_clicked(self, callback: callable):
            """
            Set double-click callback function.
            
            Sets the function to be called when buttons are double-clicked.
            Callback receives the pin number as a parameter.
            
            :param callback: Function to call on double-click (receives pin number)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Simple double-click handler
                >>> def handle_double_click(pin):
                ...     print(f"Button on pin {pin} was double-clicked!")
                >>> 
                >>> buttons[:].on_double_clicked = handle_double_click
                >>> 
                >>> # Specific functionality for different buttons
                >>> def zoom_toggle(pin):
                ...     print("Zoom toggled by double-click")
                >>> 
                >>> def fullscreen_toggle(pin):
                ...     print("Fullscreen toggled by double-click")
                >>> 
                >>> buttons[0].on_double_clicked = zoom_toggle
                >>> buttons[1].on_double_clicked = fullscreen_toggle
                >>> 
                >>> # Quick lambda for testing
                >>> buttons[2].on_double_clicked = lambda pin: print("Double!")
                >>> 
                >>> # Configure timing and enable
                >>> buttons[:].double_click_window_ms = 300
                >>> buttons[:].measurement = True
            ```
            """
            Button._set_on_double_clicked_all(self._parent, callback, self._indices)

        @property
        def on_long_pressed(self) -> list[callable]:
            """
            Get long-press callback functions.
            
            Returns the callback functions that will be called when
            buttons are long-pressed (held for extended time).
            
            :return: list of callback functions or None values
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current long-press callbacks
                >>> callbacks = buttons[:].on_long_pressed
                >>> print(callbacks)  # [None, None, None] - no callbacks set
                >>> 
                >>> # Check if callback is configured
                >>> if buttons[0].on_long_pressed[0] is not None:
                ...     print("First button has long-press callback")
            ```
            """
            return Button._get_on_long_pressed_list(self._parent, self._indices)

        @on_long_pressed.setter
        def on_long_pressed(self, callback: callable):
            """
            Set long-press callback function.
            
            Sets the function to be called when buttons are long-pressed.
            Callback receives the pin number as a parameter.
            
            :param callback: Function to call on long press (receives pin number)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Power/shutdown function on long press
                >>> def handle_long_press(pin):
                ...     print(f"Long press on pin {pin} - initiating shutdown")
                ...     # shutdown_system()
                >>> 
                >>> buttons[0].on_long_pressed = handle_long_press
                >>> 
                >>> # Different long-press actions
                >>> def reset_function(pin):
                ...     print("Reset triggered by long press")
                >>> 
                >>> def config_mode(pin):
                ...     print("Entering configuration mode")
                >>> 
                >>> buttons[1].on_long_pressed = reset_function
                >>> buttons[2].on_long_pressed = config_mode
                >>> 
                >>> # Emergency stop function
                >>> emergency_button = Button([20])
                >>> emergency_button[0].on_long_pressed = lambda pin: emergency_stop()
                >>> emergency_button[0].long_press_threshold_ms = 2000  # 2 second safety
                >>> emergency_button[0].measurement = True
            ```
            """
            Button._set_on_long_pressed_all(self._parent, callback, self._indices)

        @property
        def on_pressed(self) -> list[callable]:
            """
            Get press callback functions.
            
            Returns the callback functions that will be called immediately
            when buttons are pressed down (immediate press detection).
            
            :return: list of callback functions or None values
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current press callbacks
                >>> callbacks = buttons[:].on_pressed
                >>> print(callbacks)  # [None, None, None] - no callbacks set
                >>> 
                >>> # Verify press callback exists
                >>> if buttons[0].on_pressed[0] is not None:
                ...     print("First button has press callback")
            ```
            """
            return Button._get_on_pressed_list(self._parent, self._indices)

        @on_pressed.setter
        def on_pressed(self, callback: callable):
            """
            Set press callback function (immediate press detection).
            
            Sets the function to be called immediately when buttons are pressed.
            Useful for immediate feedback or real-time control.
            
            :param callback: Function to call when button is pressed (receives pin number)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Immediate feedback on press
                >>> def handle_press(pin):
                ...     print(f"Button {pin} pressed - immediate response")
                ...     # led.on()  # Turn on LED immediately
                >>> 
                >>> buttons[:].on_pressed = handle_press
                >>> 
                >>> # Real-time control applications
                >>> def start_motor(pin):
                ...     print("Motor starting...")
                ...     # motor.start()
                >>> 
                >>> def activate_relay(pin):
                ...     print("Relay activated")
                ...     # relay.on()
                >>> 
                >>> buttons[0].on_pressed = start_motor
                >>> buttons[1].on_pressed = activate_relay
                >>> 
                >>> # Game controller - immediate response needed
                >>> fire_button = Button([25])
                >>> fire_button[0].on_pressed = lambda pin: fire_weapon()
                >>> fire_button[0].measurement = True
            ```
            """
            Button._set_on_pressed_all(self._parent, callback, self._indices)

        @property
        def on_released(self) -> list[callable]:
            """
            Get release callback functions.
            
            Returns the callback functions that will be called immediately
            when buttons are released (immediate release detection).
            
            :return: list of callback functions or None values
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Check current release callbacks
                >>> callbacks = buttons[:].on_released
                >>> print(callbacks)  # [None, None, None] - no callbacks set
                >>> 
                >>> # Check if release callback is set
                >>> if buttons[0].on_released[0] is not None:
                ...     print("First button has release callback")
            ```
            """
            return Button._get_on_released_list(self._parent, self._indices)

        @on_released.setter
        def on_released(self, callback: callable):
            """
            Set release callback function (immediate release detection).
            
            Sets the function to be called immediately when buttons are released.
            Useful for stopping actions or providing release feedback.
            
            :param callback: Function to call when button is released (receives pin number)
            
            Example
            -------
            ```python
                >>> buttons = Button([10, 11, 12])
                >>> 
                >>> # Stop action on release
                >>> def handle_release(pin):
                ...     print(f"Button {pin} released - stopping action")
                ...     # led.off()  # Turn off LED when released
                >>> 
                >>> buttons[:].on_released = handle_release
                >>> 
                >>> # Paired press/release actions
                >>> def stop_motor(pin):
                ...     print("Motor stopping...")
                ...     # motor.stop()
                >>> 
                >>> def deactivate_relay(pin):
                ...     print("Relay deactivated")
                ...     # relay.off()
                >>> 
                >>> buttons[0].on_released = stop_motor
                >>> buttons[1].on_released = deactivate_relay
                >>> 
                >>> # Momentary control - hold to activate
                >>> control_button = Button([30])
                >>> control_button[0].on_pressed = lambda pin: activate_device()
                >>> control_button[0].on_released = lambda pin: deactivate_device()
                >>> control_button[0].measurement = True
            ```
            """
            Button._set_on_released_all(self._parent, callback, self._indices)
