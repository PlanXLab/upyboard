__version__ = "1.0.0"
__author__ = "PlanX Lab Development Team"

from . import (
    utime,
    ticle
)


class Relay:
    ON = 1
    OFF = 0
    
    NORMALLY_OPEN = True
    NORMALLY_CLOSED = False

    
    def __init__(self, pins: list[int] | tuple[int, ...], *, contact_type: bool = NORMALLY_OPEN):
        if not pins:
            raise ValueError("At least one pin must be provided")
            
        self._pins = list(pins)
        n = len(self._pins)
        
        self._dout = ticle.Dout(self._pins)
        self._dout[:].active = ticle.Dout.LOGIC_HIGH
        
        self._contact_type = [contact_type] * n
        self._logical_state = [Relay.OFF] * n
        
        self._interlock_groups = [None] * n
        self._interlock_auto_change = {}
        
        self._dout[:].value = ticle.Dout.LOW

    def deinit(self) -> None:
        try:
            for i in range(len(self._pins)):
                self._logical_state[i] = Relay.OFF
            self._dout[:].value = ticle.Dout.LOW
            
            utime.sleep_ms(50)
            
            self._dout.deinit()            
        except:
            pass

    def set_interlock_auto_change(self, group: str, value: bool):
        if group is not None:
            self._interlock_auto_change[group] = bool(value)

    def get_interlock_auto_change(self, group: str) -> bool:
        return self._interlock_auto_change.get(group, False)

    def __getitem__(self, idx: int | slice) -> "_RelayView":
        if isinstance(idx, slice):
            indices = list(range(*idx.indices(len(self._pins))))
            return Relay._RelayView(self, indices)
        elif isinstance(idx, int):
            if not (0 <= idx < len(self._pins)):
                raise IndexError("Relay index out of range")
            return Relay._RelayView(self, [idx])
        else:
            raise TypeError("Index must be int or slice")

    def __len__(self) -> int:
        return len(self._pins)

    def _check_interlock(self, idx: int, new_state: int) -> bool:
        group = self._interlock_groups[idx]
        if group is None:
            return True
        
        if new_state == Relay.ON:
            auto_change = self._interlock_auto_change.get(group, False)
            for i in range(len(self._pins)):
                if (i != idx and 
                    self._interlock_groups[i] == group and 
                    self._logical_state[i] == Relay.ON):
                    if auto_change:
                        self._set_relay_state(i, Relay.OFF)
                        continue
                    else:
                        return False
        return True

    def _set_relay_state(self, idx: int, state: int) -> bool:
        if not self._check_interlock(idx, state):
            return False
        
        self._logical_state[idx] = state
        self._update_physical_output(idx)
        return True

    def _update_physical_output(self, idx: int) -> None:
        logical_state = self._logical_state[idx]
        
        if self._contact_type[idx] == Relay.NORMALLY_OPEN:
            physical_state = logical_state
        else:  # NORMALLY_CLOSED
            physical_state = 1 - logical_state
        
        self._dout[idx].value = physical_state

    @staticmethod
    def _get_state_list(parent, indices: list[int]) -> list[int]:
        return [parent._logical_state[i] for i in indices]

    @staticmethod
    def _set_state_all(parent, state: int, indices: list[int]) -> None:
        for i in indices:
            parent._set_relay_state(i, state)

    @staticmethod
    def _get_contact_type_list(parent, indices: list[int]) -> list[bool]:
        return [parent._contact_type[i] for i in indices]

    @staticmethod
    def _set_contact_type_all(parent, contact_type: bool, indices: list[int]) -> None:
        """Set contact type for specified relays."""
        for i in indices:
            parent._contact_type[i] = contact_type
            parent._update_physical_output(i)

    @staticmethod
    def _get_interlock_group_list(parent, indices: list[int]) -> list[str]:
        return [parent._interlock_groups[i] for i in indices]

    @staticmethod
    def _set_interlock_group_all(parent, group: str, indices: list[int]) -> None:
        for i in indices:
            parent._interlock_groups[i] = group

    class _RelayView:
        def __init__(self, parent: "Relay", indices: list[int]):
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int | slice) -> "Relay._RelayView":
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Relay._RelayView(self._parent, selected_indices)
            else:
                return Relay._RelayView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            return len(self._indices)

        @property
        def state(self) -> list[int]:
            return Relay._get_state_list(self._parent, self._indices)

        @state.setter
        def state(self, value: int | list[int]):
            if isinstance(value, (list, tuple)):
                if len(value) != len(self._indices):
                    raise ValueError("list length must match number of relays")
                for i, state in zip(self._indices, value):
                    self._parent._set_relay_state(i, state)
            else:
                Relay._set_state_all(self._parent, value, self._indices)

        @property
        def contact_type(self) -> list[bool]:
            return Relay._get_contact_type_list(self._parent, self._indices)

        @contact_type.setter
        def contact_type(self, contact_type: bool):
            Relay._set_contact_type_all(self._parent, contact_type, self._indices)

        @property
        def interlock_group(self) -> list[str]:
            return Relay._get_interlock_group_list(self._parent, self._indices)

        @interlock_group.setter
        def interlock_group(self, group: str):
            Relay._set_interlock_group_all(self._parent, group, self._indices)

        def toggle(self):
            current_states = self.state
            new_states = [Relay.OFF if state == Relay.ON else Relay.ON for state in current_states]
            self.state = new_states

        def pulse(self, duration_ms: int, state: int = 1):
            opposite_state = Relay.OFF if state == Relay.ON else Relay.ON
            
            self.state = state
            utime.sleep_ms(duration_ms)            
            self.state = opposite_state

        def emergency_stop(self):
            original_groups = []
            for i in self._indices:
                original_groups.append(self._parent._interlock_groups[i])
                self._parent._interlock_groups[i] = None
            
            self.state = Relay.OFF

