from . import (
    utime,
    ticle
)


class Relay:
    """
    Relay control class with essential safety features and interlock protection.
    
    This class provides basic relay control functionality with interlock protection
    for safe operation in motor control, HVAC systems, and other applications where
    mutual exclusion is required. Features include basic ON/OFF control, contact
    type configuration, and interlock group management.
    
    Features:

        - Multiple relay support with unified interface
        - Basic ON/OFF control with safety initialization
        - Individual and group control via indexing/slicing
        - Contact type configuration (NO/NC)
        - Interlock protection for mutual exclusion
        - Force control for emergency situations
        
    Safety Features:

        - Interlock protection prevents conflicting operations
        - All relays start in OFF state for safety
        - Contact type configuration for proper wiring
        
    Constants:

        - Relay.ON (1): Relay energized state
        - Relay.OFF (0): Relay de-energized state
        - Relay.NORMALLY_OPEN: NO contact configuration
        - Relay.NORMALLY_CLOSED: NC contact configuration
    
    """
    ON = 1
    OFF = 0
    
    NORMALLY_OPEN = True
    NORMALLY_CLOSED = False

    
    def __init__(self, pins: list[int] | tuple[int, ...], *, contact_type: bool = NORMALLY_OPEN):
        """
        Initialize relay controller with interlock capability.
        
        Creates a relay controller for basic switching operations with interlock
        protection. All relays start in OFF state for safety.
        
        :param pins: list or tuple of GPIO pin numbers for relay control
        :param contact_type: Default contact configuration for all relays
        
            - Relay.NORMALLY_OPEN: NO contacts (default)
            - Relay.NORMALLY_CLOSED: NC contacts

        :raises ValueError: If pins list is empty
        :raises OSError: If GPIO pin initialization fails
        
        Example
        -------
        ```python
            >>> # Basic relay setup
            >>> relays = Relay([10, 11, 12])
            >>> 
            >>> # NC contact relays
            >>> nc_relays = Relay([20, 21], contact_type=Relay.NORMALLY_CLOSED)
            >>> 
            >>> # Motor control system
            >>> motor_relays = Relay([5, 6, 7, 8])  # Forward, Reverse, Enable, Brake
            >>> motor_relays[0:2].interlock_group = "DIRECTION"
        ```
        """
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
        """
        Safely deinitialize all relays.
        
        Turns off all relays, waits for contact settling, and releases resources.
        Should be called when relays are no longer needed.
        
        Example
        -------
        ```python
            >>> relays = Relay([10, 11, 12])
            >>> # ... use relays ...
            >>> relays.deinit()  # Safe shutdown
            >>> 
            >>> # Exception handling pattern
            >>> try:
            ...     relays = Relay([10, 11, 12])
            ...     relays[0].state = Relay.ON
            ...     # ... relay operations ...
            >>> finally:
            ...     relays.deinit()  # Ensures cleanup even if error occurs
        ```
        """
        try:
            for i in range(len(self._pins)):
                self._logical_state[i] = Relay.OFF
            self._dout[:].value = ticle.Dout.LOW
            
            utime.sleep_ms(50)
            
            self._dout.deinit()            
        except:
            pass

    def set_interlock_auto_change(self, group: str, value: bool):
        """
        Set auto change policy for a specific interlock group.

        :param group: Interlock group name
        :param value: True for auto change, False for block
        """
        if group is not None:
            self._interlock_auto_change[group] = bool(value)

    def get_interlock_auto_change(self, group: str) -> bool:
        """
        Get auto change policy for a specific interlock group.

        :param group: Interlock group name
        :return: True if auto change, False if block (default: False)
        """
        return self._interlock_auto_change.get(group, False)

    def __getitem__(self, idx: int | slice) -> "_RelayView":
        """
        Get a view for controlling specific relay(s).
        
        Supports both single indexing and slice notation for flexible
        relay selection and control.
        
        :param idx: Index (int) or slice for relay selection
        :return: _RelayView instance for selected relay(s)
        
        :raises IndexError: If relay index is out of range
        :raises TypeError: If index is not int or slice
        
        Example
        -------
        ```python
            >>> relays = Relay([10, 11, 12, 13])
            >>> 
            >>> # Single relay access
            >>> first_relay = relays[0]
            >>> last_relay = relays[-1]
            >>> 
            >>> # Multiple relay access
            >>> first_two = relays[0:2]        # Relays 0 and 1
            >>> last_two = relays[2:4]         # Relays 2 and 3
            >>> all_relays = relays[:]         # All relays
            >>> 
            >>> # Configure individual or groups
            >>> relays[0].contact_type = Relay.NORMALLY_OPEN
            >>> relays[1:3].interlock_group = "MOTOR_DIRECTION"
        ```
        """
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
        """
        Get the number of relays configured.
        
        Returns the total number of relays in this controller instance.
        
        :return: Number of relays configured
        
        Example
        -------
        ```python
            >>> relays = Relay([10, 11, 12, 13, 14])
            >>> print(len(relays))  # Output: 5
            >>> 
            >>> # Use in loops
            >>> for i in range(len(relays)):
            ...     relays[i].state = Relay.ON
            ...     utime.sleep_ms(500)  # Stagger relay activation
            >>> 
            >>> # Conditional operations based on count
            >>> if len(relays) > 4:
            ...     print("Large relay system detected")
        ```
        """
        return len(self._pins)

    def _check_interlock(self, idx: int, new_state: int) -> bool:
        """
        Check if relay state change violates interlock rules.
        
        Internal method that validates state changes against interlock
        group configurations to prevent conflicting operations.
        
        :param idx: Index of relay to check
        :param new_state: Proposed new state
        :return: True if change is allowed, False if blocked
        """
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
        """
        Set relay state with interlock checking.
        
        Internal method that updates relay state while respecting
        interlock rules and safety constraints.
        
        :param idx: Index of relay to control
        :param state: Desired state (ON/OFF)
        :return: True if state was changed, False if blocked
        """
        # Check interlock
        if not self._check_interlock(idx, state):
            return False
        
        # Update state
        self._logical_state[idx] = state
        self._update_physical_output(idx)
        return True

    def _update_physical_output(self, idx: int) -> None:
        """
        Update physical relay output based on logical state.
        
        Internal method to translate logical relay state to physical
        GPIO output considering contact type and polarity.
        
        :param idx: Index of relay to update
        """
        logical_state = self._logical_state[idx]
        
        # For normally open contacts: ON = energized, OFF = de-energized
        # For normally closed contacts: ON = de-energized, OFF = energized
        if self._contact_type[idx] == Relay.NORMALLY_OPEN:
            physical_state = logical_state
        else:  # NORMALLY_CLOSED
            physical_state = 1 - logical_state
        
        self._dout[idx].value = physical_state

    @staticmethod
    def _get_state_list(parent, indices: list[int]) -> list[int]:
        """Get current logical state for specified relays."""
        return [parent._logical_state[i] for i in indices]

    @staticmethod
    def _set_state_all(parent, state: int, indices: list[int]) -> None:
        """Set logical state for specified relays with interlock checking."""
        for i in indices:
            parent._set_relay_state(i, state)

    @staticmethod
    def _get_contact_type_list(parent, indices: list[int]) -> list[bool]:
        """Get contact type configuration for specified relays."""
        return [parent._contact_type[i] for i in indices]

    @staticmethod
    def _set_contact_type_all(parent, contact_type: bool, indices: list[int]) -> None:
        """Set contact type for specified relays."""
        for i in indices:
            parent._contact_type[i] = contact_type
            parent._update_physical_output(i)

    @staticmethod
    def _get_interlock_group_list(parent, indices: list[int]) -> list[str]:
        """Get interlock group assignments for specified relays."""
        return [parent._interlock_groups[i] for i in indices]

    @staticmethod
    def _set_interlock_group_all(parent, group: str, indices: list[int]) -> None:
        """Set interlock group for specified relays."""
        for i in indices:
            parent._interlock_groups[i] = group

    class _RelayView:
        """
        View class for controlling individual relays or groups of relays.
        
        This class provides a unified interface for controlling one or more relays
        through the same API. It's returned by Relay.__getitem__() and allows
        seamless control of single relays or groups using identical syntax.
        """

        def __init__(self, parent: "Relay", indices: list[int]):
            """
            Initialize relay view with parent reference and relay indices.
            
            :param parent: Parent Relay instance
            :param indices: list of relay indices this view controls
            
            Example
            -------
            ```python
                >>> # RelayView is typically created automatically
                >>> relays = Relay([10, 11, 12])
                >>> view = relays[0:2]  # Creates _RelayView for first two relays
                >>> print(type(view))    # <class '_RelayView'>
            ```
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int | slice) -> "Relay._RelayView":
            """
            Get a sub-view of this view for further relay selection.
            
            Enables nested selection and fine-grained control over relay groups.
            
            :param idx: Index or slice for sub-selection
            :return: New _RelayView with selected relays
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12, 13])
                >>> group = relays[0:3]        # First three relays
                >>> subgroup = group[1:3]      # Second and third from original
                >>> subgroup.state = Relay.ON  # Control sub-selection
                >>> 
                >>> # Chain selections
                >>> relays[1:4][0:2].state = Relay.ON  # Relays 1 and 2
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Relay._RelayView(self._parent, selected_indices)
            else:
                return Relay._RelayView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of relays in this view.
            
            Returns the count of relays controlled by this view instance.
            
            :return: Number of relays in this view
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12, 13])
                >>> group = relays[1:3]
                >>> print(len(group))  # Output: 2
                >>> 
                >>> # Use in operations
                >>> if len(group) > 1:
                ...     print("Multi-relay group")
                ...     group.state = Relay.ON
            ```
            """
            return len(self._indices)

        @property
        def state(self) -> list[int]:
            """
            Get current logical state of relays in this view.
            
            Returns the logical state (ON/OFF) regardless of contact type.
            
            :return: list of relay states (Relay.ON or Relay.OFF)
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12])
                >>> relays[0].state = Relay.ON
                >>> relays[1].state = Relay.OFF
                >>> 
                >>> # Check all states
                >>> states = relays[:].state
                >>> print(f"States: {states}")  # [1, 0, 0]
                >>> 
                >>> # Check individual state
                >>> if relays[0].state[0] == Relay.ON:
                ...     print("First relay is ON")
                >>> 
                >>> # Count active relays
                >>> active_count = sum(relays[:].state)
                >>> print(f"Active relays: {active_count}")
            ```
            """
            return Relay._get_state_list(self._parent, self._indices)

        @state.setter
        def state(self, value: int | list[int]):
            """
            Set logical state of relays in this view.
            
            Sets the desired logical state with interlock checking.
            Changes may be blocked by interlock protection.
            
            :param value: Single state for all relays or list of states
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12, 13])
                >>> 
                >>> # Turn on all relays (if interlocks allow)
                >>> relays[:].state = Relay.ON
                >>> 
                >>> # Individual relay control
                >>> relays[:].state = [Relay.ON, Relay.OFF, Relay.ON, Relay.OFF]
                >>> 
                >>> # Single relay
                >>> relays[0].state = Relay.OFF
                >>> 
                >>> # Sequential activation
                >>> for i in range(len(relays)):
                ...     relays[i].state = Relay.ON
                ...     utime.sleep_ms(200)  # 200ms delay between activations
                >>> 
                >>> # Pattern control
                >>> pattern = [Relay.ON, Relay.OFF] * 2  # Alternating pattern
                >>> relays[:].state = pattern
            ```
            """
            if isinstance(value, (list, tuple)):
                if len(value) != len(self._indices):
                    raise ValueError("list length must match number of relays")
                for i, state in zip(self._indices, value):
                    self._parent._set_relay_state(i, state)
            else:
                Relay._set_state_all(self._parent, value, self._indices)

        @property
        def contact_type(self) -> list[bool]:
            """
            Get contact type configuration for relays in this view.
            
            Returns the contact type configuration for each relay.
            
            :return: list of contact types (True=NO, False=NC)
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12])
                >>> 
                >>> # Check current contact types
                >>> types = relays[:].contact_type
                >>> print(types)  # [True, True, True] - all NO by default
                >>> 
                >>> # Check specific relay
                >>> if relays[0].contact_type[0] == Relay.NORMALLY_OPEN:
                ...     print("First relay has NO contacts")
                >>> 
                >>> # Verify configuration
                >>> for i, contact_type in enumerate(relays[:].contact_type):
                ...     type_str = "NO" if contact_type else "NC"
                ...     print(f"Relay {i}: {type_str} contacts")
            ```
            """
            return Relay._get_contact_type_list(self._parent, self._indices)

        @contact_type.setter
        def contact_type(self, contact_type: bool):
            """
            Set contact type for relays in this view.
            
            Configures the contact type and automatically updates the
            physical output to match the logical state.
            
            :param contact_type: Contact type for all relays
            
                - Relay.NORMALLY_OPEN (True): NO contacts
                - Relay.NORMALLY_CLOSED (False): NC contacts

            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12])
                >>> 
                >>> # Configure for NO contacts (typical)
                >>> relays[:].contact_type = Relay.NORMALLY_OPEN
                >>> 
                >>> # Configure specific relay for NC operation
                >>> relays[2].contact_type = Relay.NORMALLY_CLOSED
                >>> 
                >>> # Mixed configuration for different applications
                >>> relays[0].contact_type = Relay.NORMALLY_OPEN   # Motor control
                >>> relays[1].contact_type = Relay.NORMALLY_CLOSED # Safety circuit
                >>> 
                >>> # Heating system example
                >>> heating_relays = Relay([20, 21, 22])
                >>> heating_relays[:].contact_type = Relay.NORMALLY_OPEN  # Fail-safe off
            ```
            """
            Relay._set_contact_type_all(self._parent, contact_type, self._indices)

        @property
        def interlock_group(self) -> list[str]:
            """
            Get interlock group assignments for relays in this view.
            
            Returns the interlock group names for mutual exclusion control.
            
            :return: list of interlock group names or None values
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12])
                >>> groups = relays[:].interlock_group
                >>> print(groups)  # [None, None, None] - no groups assigned
                >>> 
                >>> # Check specific relay group
                >>> if relays[0].interlock_group[0] is not None:
                ...     print(f"Relay 0 is in group: {relays[0].interlock_group[0]}")
            ```
            """
            return Relay._get_interlock_group_list(self._parent, self._indices)

        @interlock_group.setter
        def interlock_group(self, group: str):
            """
            Set interlock group for mutual exclusion.
            
            Relays in the same interlock group cannot be ON simultaneously.
            This prevents dangerous conditions like motor short circuits.
            
            :param group: Interlock group name (None to remove from group)
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12, 13])
                >>> 
                >>> # Motor direction interlock
                >>> relays[0:2].interlock_group = "MOTOR_DIRECTION"
                >>> relays[0].state = Relay.ON   # Forward ON
                >>> relays[1].state = Relay.ON   # Blocked! (Reverse)
                >>> 
                >>> # HVAC system interlock
                >>> relays[2:4].interlock_group = "HEATING_COOLING"
                >>> relays[2].state = Relay.ON   # Heater ON
                >>> relays[3].state = Relay.ON   # Blocked! (Cooler)
                >>> 
                >>> # Remove from interlock group
                >>> relays[0].interlock_group = None
            ```
            """
            Relay._set_interlock_group_all(self._parent, group, self._indices)

        def toggle(self):
            """
            Toggle the state of relays in this view.
            
            Changes ON relays to OFF and OFF relays to ON, subject to
            interlock restrictions.
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12])
                >>> relays[0].state = Relay.ON
                >>> relays[1].state = Relay.OFF
                >>> 
                >>> # Toggle single relay
                >>> relays[0].toggle()  # Now OFF
                >>> relays[0].toggle()  # Now ON again
                >>> 
                >>> # Toggle multiple relays
                >>> relays[:].toggle()  # All relays toggle their state
                >>> 
                >>> # Blinking pattern
                >>> for _ in range(5):
                ...     relays[:].toggle()
                ...     utime.sleep_ms(500)  # 500ms on/off cycle
                >>> 
                >>> # Emergency signal
                >>> emergency_relay = Relay([25])
                >>> for _ in range(10):
                ...     emergency_relay[0].toggle()
                ...     utime.sleep_ms(100)  # Fast blinking
            ```
            """
            current_states = self.state
            new_states = [Relay.OFF if state == Relay.ON else Relay.ON for state in current_states]
            self.state = new_states

        def pulse(self, duration_ms: int, state: int = 1):
            """
            Generate a timed pulse on relays in this view.
            
            Turns relay to specified state for the given duration, then
            returns to opposite state. Useful for momentary operations.
            
            :param duration_ms: Pulse duration in milliseconds
            :param state: Pulse state (default: Relay.ON)
            
            Example
            -------
            ```python
                >>> relays = Relay([10, 11, 12])
                >>> 
                >>> # Momentary activation
                >>> relays[0].pulse(1000)  # 1-second ON pulse
                >>> 
                >>> # Momentary deactivation
                >>> relays[0].state = Relay.ON
                >>> relays[0].pulse(500, Relay.OFF)  # 500ms OFF pulse
                >>> 
                >>> # Door lock activation
                >>> door_lock = Relay([20])
                >>> door_lock[0].pulse(2000)  # 2-second unlock pulse
            ```
            """
            opposite_state = Relay.OFF if state == Relay.ON else Relay.ON
            
            self.state = state
            utime.sleep_ms(duration_ms)            
            self.state = opposite_state

        def emergency_stop(self):
            """
            Force all relays in this view to OFF state immediately.
            
            This method bypasses interlock checks and safety features,
            ensuring all relays are turned off instantly. Use with caution.
            This is typically used in emergency situations where im mediate
            shutdown is required, such as stopping motors or cutting power.
            
            EExample
            -------
            ```python
                >>> relays = Relay([10, 11, 12])
                >>>
                >>> # Emergency stop all relays
                >>> relays[:].emergency_stop()  # All relays OFF immediately
                >>> # Motor control system emergency
                >>> motor_relays = Relay([5, 6, 7, 8])
                >>> motor_relays[0:2].interlock_group = "MOTOR_DIRECTION"
                >>> motor_relays[0].state = Relay.ON  # Forward ON
                >>> motor_relays[1].state = Relay.ON  # Blocked! (Reverse)
                >>> # Emergency stop to prevent conflict
                >>> motor_relays[0:2].emergency_stop()
            """
            original_groups = []
            for i in self._indices:
                original_groups.append(self._parent._interlock_groups[i])
                self._parent._interlock_groups[i] = None
            
            self.state = Relay.OFF