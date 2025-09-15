__version__ = "1.0.0"
__author__ = "PlanX Lab Development Team"

from . import (
    utime,
    machine, micropython,
    ticle
)


class Button:
    CLICKED = 1
    DOUBLE_CLICKED = 2
    LONG_PRESSED = 3
    PRESSED = 4
    RELEASED = 5
    
    ACTIVE_HIGH = True
    ACTIVE_LOW = False
    
    def __init__(self, pins: list[int] | tuple[int, ...]):
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
        return len(self._pins)

    def _button_interrupt_handler(self, pin_num: int, rising: bool) -> None:
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
        self._current_state[pin_idx] = True
        self._press_start_time[pin_idx] = current_time
        self._long_press_fired[pin_idx] = False
        
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
        if not self._current_state[pin_idx]:
            return 
            
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
        if self._timers[pin_idx] is not None:
            try:
                self._timers[pin_idx].deinit()
                self._timers[pin_idx] = None
            except:
                pass

    def _raw_read(self, idx: int) -> int:
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
        def __init__(self, parent: "Button", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int | slice) -> "Button._ButtonView":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Button._ButtonView(self._parent, selected_indices)
            else:
                return Button._ButtonView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        @property
        def active_type(self) -> list[bool]:
            return Button._get_active_type_list(self._parent, self._indices)

        @active_type.setter
        def active_type(self, logic_type: bool):
            Button._set_active_type_all(self._parent, logic_type, self._indices)

        @property
        def measurement(self) -> list[bool]:
            return Button._get_measurement_list(self._parent, self._indices)

        @measurement.setter
        def measurement(self, enabled: bool):
            Button._set_measurement_all(self._parent, enabled, self._indices)

        @property
        def debounce_ms(self) -> list[int]:
            return Button._get_debounce_ms_list(self._parent, self._indices)

        @debounce_ms.setter
        def debounce_ms(self, ms: int):
            Button._set_debounce_ms_all(self._parent, ms, self._indices)

        @property
        def double_click_window_ms(self) -> list[int]:
            return Button._get_double_click_window_ms_list(self._parent, self._indices)

        @double_click_window_ms.setter
        def double_click_window_ms(self, ms: int):
            Button._set_double_click_window_ms_all(self._parent, ms, self._indices)

        @property
        def long_press_threshold_ms(self) -> list[int]:
            return Button._get_long_press_threshold_ms_list(self._parent, self._indices)

        @long_press_threshold_ms.setter
        def long_press_threshold_ms(self, ms: int):
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
            return Button._get_click_list(self._parent, self._indices)

        @property
        def on_clicked(self) -> list[callable]:
            return Button._get_on_clicked_list(self._parent, self._indices)

        @on_clicked.setter
        def on_clicked(self, callback: callable):
            Button._set_on_clicked_all(self._parent, callback, self._indices)

        @property
        def on_double_clicked(self) -> list[callable]:
            return Button._get_on_double_clicked_list(self._parent, self._indices)

        @on_double_clicked.setter
        def on_double_clicked(self, callback: callable):
            Button._set_on_double_clicked_all(self._parent, callback, self._indices)

        @property
        def on_long_pressed(self) -> list[callable]:
            return Button._get_on_long_pressed_list(self._parent, self._indices)

        @on_long_pressed.setter
        def on_long_pressed(self, callback: callable):
            Button._set_on_long_pressed_all(self._parent, callback, self._indices)

        @property
        def on_pressed(self) -> list[callable]:
            return Button._get_on_pressed_list(self._parent, self._indices)

        @on_pressed.setter
        def on_pressed(self, callback: callable):
            Button._set_on_pressed_all(self._parent, callback, self._indices)

        @property
        def on_released(self) -> list[callable]:
            return Button._get_on_released_list(self._parent, self._indices)

        @on_released.setter
        def on_released(self, callback: callable):
            Button._set_on_released_all(self._parent, callback, self._indices)


