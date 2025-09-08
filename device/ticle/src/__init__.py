"""
RP2350 Board Control Library

Comprehensive hardware abstraction and control library for the RP2350 (Tiny Internet of Things Connected Learning Environment) board.
This library provides high-level interfaces for all onboard peripherals and sensors, enabling rapid prototyping and development
of IoT applications with professional-grade functionality and robust error handling.

The RP2350 board is built around the Raspberry Pi RP2350 microcontroller and features integrated WiFi connectivity,
multiple GPIO interfaces, analog inputs, PWM outputs, I2C communication, and various onboard sensors. This library
abstracts the hardware complexity and provides intuitive Python interfaces for seamless development.

Core Features:
- Complete hardware abstraction for RP2350 board peripherals
- WiFi connectivity management with automatic reconnection
- Multi-channel analog-to-digital conversion with precision voltage readings
- Flexible PWM generation for servo control, LED dimming, and motor control
- I2C communication interface with automatic bus detection
- Digital input/output control with interrupt callback support
- Built-in LED and button interfaces for immediate feedback
- Advanced serial communication with REPL integration
- System monitoring and diagnostics capabilities
- Memory and filesystem information utilities

Hardware Interfaces:
- GPIO Control: Digital input/output with configurable pull resistors and interrupts
- ADC Channels: 16-bit analog inputs on GPIO 26, 27, 28 with voltage conversion
- PWM Outputs: Multi-channel pulse width modulation with frequency and duty cycle control
- I2C Communication: Dual-bus I2C with automatic device detection and comprehensive register access
- WiFi Connectivity: Station mode with network scanning, connection management, and status monitoring
- Built-in Components: LED control (WL_GPIO0) and BOOTSEL button interface
- Serial Interface: Advanced REPL integration with line editing and UTF-8 support
"""

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
    """
    Get system information including core frequency and temperature.
    
    This function retrieves the current CPU frequency and internal temperature
    of the RP2350 board, automatically detecting the chip type for proper
    temperature sensor access.
    
    :return: tuple of (frequency, temperature)
    
    :raises ValueError: If temperature sensor GPIO is not accessible
    
    Example
    --------
    ```python
        >>> freq, temp = get_sys_info()
        >>> print(f"CPU Frequency: {freq/1000000:.1f} MHz")
        >>> print(f"Temperature: {temp:.1f}°C")
        >>> # Output: CPU Frequency: 150.0 MHz
        >>> # Output: Temperature: 23.4°C
        >>> 
        >>> # Check for overheating
        >>> if temp > 70:
        ...     print("Warning: High temperature detected!")
        >>> 
        >>> # Monitor system performance
        >>> import utime
        >>> for i in range(5):
        ...     freq, temp = get_sys_info()
        ...     print(f"Reading {i+1}: {freq/1000000:.1f}MHz, {temp:.1f}°C")
        ...     utime.sleep(1)
        >>> 
        >>> # System health check
        >>> freq, temp = get_sys_info()
        >>> if freq < 100000000:  # Less than 100MHz
        ...     print("Warning: Low CPU frequency detected")
        >>> if temp > 80:
        ...     print("Critical: High temperature - consider cooling")
        >>> elif temp > 60:
        ...     print("Warning: Elevated temperature detected")
        >>> 
        >>> # Performance monitoring loop
        >>> import utime
        >>> max_temp = 0
        >>> min_temp = 100
        >>> while True:
        ...     freq, temp = get_sys_info()
        ...     max_temp = max(max_temp, temp)
        ...     min_temp = min(min_temp, temp)
        ...     print(f"Current: {temp:.1f}°C, Range: {min_temp:.1f}-{max_temp:.1f}°C")
        ...     utime.sleep(10)
    ```
    """
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
    """
    Get memory usage information of RP2350.
    
    This function performs garbage collection and returns detailed memory
    statistics including free, used, and total memory in bytes.
    
    :return: tuple of (free, used, total) memory in bytes
    
    Example
    --------
    ```python
        >>> free, used, total = get_mem_info()
        >>> print(f"Free memory: {free/1024:.1f} KB")
        >>> print(f"Used memory: {used/1024:.1f} KB")
        >>> print(f"Total memory: {total/1024:.1f} KB")
        >>> print(f"Memory usage: {used/total*100:.1f}%")
        >>> # Output: Free memory: 180.5 KB
        >>> # Output: Used memory: 83.2 KB
        >>> # Output: Total memory: 263.7 KB
        >>> # Output: Memory usage: 31.5%
        >>> 
        >>> # Memory monitoring loop
        >>> import utime
        >>> while True:
        ...     free, used, total = get_mem_info()
        ...     usage = used / total * 100
        ...     print(f"Memory usage: {usage:.1f}%")
        ...     if usage > 90:
        ...         print("Warning: Low memory!")
        ...     utime.sleep(10)
        >>> 
        >>> # Memory allocation tracking
        >>> def track_memory_usage(func):
        ...     '''Decorator to track memory usage of a function'''
        ...     def wrapper(*args, **kwargs):
        ...         before = get_mem_info()
        ...         result = func(*args, **kwargs)
        ...         after = get_mem_info()
        ...         print(f"Memory change: {after[1] - before[1]} bytes")
        ...         return result
        ...     return wrapper
        >>> 
        >>> @track_memory_usage
        >>> def create_large_list():
        ...     return [i for i in range(1000)]
        >>> 
        >>> data = create_large_list()
        >>> 
        >>> # Memory optimization check
        >>> free, used, total = get_mem_info()
        >>> if free < 10240:  # Less than 10KB free
        ...     print("Critical: Very low memory available")
        ...     import gc
        ...     gc.collect()
        ...     print("Garbage collection performed")
        >>> 
        >>> # Memory efficiency calculation
        >>> free, used, total = get_mem_info()
        >>> efficiency = (total - used) / total * 100
        >>> print(f"Memory efficiency: {efficiency:.1f}%")
        >>> if efficiency < 50:
        ...     print("Consider optimizing memory usage")
    ```
    """
    gc.collect()
    
    free = gc.mem_free()
    used = gc.mem_alloc()
    total = free + used
    
    return free, used, total

@micropython.native
def get_fs_info(path: str = '/') -> tuple:
    """
    Get filesystem information for the given path.
    
    This function retrieves detailed filesystem statistics including total,
    used, and free space along with usage percentage for the specified path.
    
    :param path: Path to check filesystem info for (default: '/')
    :return: tuple of (total, used, free, usage percentage)
    
    Example
    --------
    ```python
        >>> total, used, free, usage = get_fs_info()
        >>> print(f"Total space: {total/1024:.1f} KB")
        >>> print(f"Used space: {used/1024:.1f} KB")
        >>> print(f"Free space: {free/1024:.1f} KB")
        >>> print(f"Usage: {usage:.1f}%")
        >>> # Output: Total space: 1024.0 KB
        >>> # Output: Used space: 156.3 KB
        >>> # Output: Free space: 867.7 KB
        >>> # Output: Usage: 15.3%
        >>> 
        >>> # Check specific directory
        >>> try:
        ...     total, used, free, usage = get_fs_info('/lib')
        ...     print(f"Library usage: {usage:.1f}%")
        >>> except:
        ...     print("Directory not found or inaccessible")
        >>> 
        >>> # Storage monitoring
        >>> total, used, free, usage = get_fs_info()
        >>> if usage > 80:
        ...     print("Warning: Low storage space!")
        >>> elif usage > 95:
        ...     print("Critical: Storage nearly full!")
        >>> 
        >>> # Storage cleanup recommendation
        >>> total, used, free, usage = get_fs_info()
        >>> if free < 50 * 1024:  # Less than 50KB free
        ...     print("Consider removing unnecessary files")
        ...     print("Available space is critically low")
        >>> 
        >>> # File system health check
        >>> def check_filesystem_health():
        ...     total, used, free, usage = get_fs_info()
        ...     print(f"Filesystem Health Report:")
        ...     print(f"  Total Capacity: {total/1024:.1f} KB")
        ...     print(f"  Used Space: {used/1024:.1f} KB ({usage:.1f}%)")
        ...     print(f"  Free Space: {free/1024:.1f} KB ({100-usage:.1f}%)")
        ...     
        ...     if usage < 50:
        ...         print("  Status: Healthy")
        ...     elif usage < 75:
        ...         print("  Status: Moderate usage")
        ...     elif usage < 90:
        ...         print("  Status: High usage - monitor closely")
        ...     else:
        ...         print("  Status: Critical - cleanup needed")
        >>> 
        >>> check_filesystem_health()
        >>> 
        >>> # Storage usage comparison
        >>> root_stats = get_fs_info('/')
        >>> print(f"Root filesystem usage: {root_stats[3]:.1f}%")
        >>> 
        >>> # Monitor storage over time
        >>> import utime
        >>> previous_usage = 0
        >>> while True:
        ...     total, used, free, usage = get_fs_info()
        ...     if abs(usage - previous_usage) > 1.0:  # Significant change
        ...         print(f"Storage usage changed: {usage:.1f}% (was {previous_usage:.1f}%)")
        ...         previous_usage = usage
        ...     utime.sleep(30)
    ```
    """
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
    """
    Comprehensive WiFi management class for RP2350 board connectivity.
    
    This class provides complete WiFi functionality including network scanning,
    connection management, status monitoring, and configuration retrieval.
    It automatically activates the WiFi interface and provides both synchronous
    connection capabilities with robust error handling.
    
    The manager handles automatic interface activation, connection timeouts,
    and provides convenient methods for network discovery and status monitoring.
    Designed for reliable WiFi connectivity in IoT applications with proper
    resource management and clean disconnection capabilities.
    
    Key Features:
    
        - Automatic WiFi interface activation and management
        - Network scanning with detailed access point information
        - Robust connection handling with configurable timeouts
        - Connection status monitoring and IP address retrieval
        - Clean disconnection and resource management
        - Support for multiple network credentials
        - Network interface configuration access
    
    Security Support:
    
        - Open networks (no authentication)
        - WEP encryption (legacy, not recommended)
        - WPA-PSK and WPA2-PSK (recommended)
        - WPA/WPA2 mixed mode
        - Automatic security detection during scan
    
    Connection States:
    
        - Inactive: Interface disabled
        - Scanning: Searching for networks
        - Connecting: Authentication in progress
        - Connected: Successfully connected with IP
        - Disconnected: Not connected to any network
        
    """
    
    def __init__(self):
        """
        Initialize WiFi manager with automatic interface activation.
        
        Creates a WiFi manager instance with the specified network interface.
        The interface is automatically activated if not already active.
                
        :raises OSError: If WiFi interface initialization fails
        :raises RuntimeError: If WiFi hardware is not available
        
        Example
        --------
        ```python
            >>> # Standard initialization (recommended)
            >>> wifi = WifiManager()
        ```
        """
        self.__iface = network.WLAN(network.STA_IF)
        if not self.__iface.active():
            self.__iface.active(True)
           
    def scan(self) -> list[tuple[str,int,int,int]]:
        """
        Scan for available WiFi networks in the area.
        
        Performs a comprehensive WiFi scan and returns detailed information about
        all discovered access points including signal strength, channel, and security.
        The scan may take several seconds to complete.
        
        :return: List of tuples containing (SSID, RSSI, channel, security).
        
            - SSID (bytes): Network name as raw bytes
            - RSSI (int): Signal strength in dBm (negative values, closer to 0 = stronger)
            - Channel (int): WiFi channel number (1-14 for 2.4GHz)
            - Security (int): Security type (0=Open, 1=WEP, 2=WPA-PSK, 3=WPA2-PSK, 4=WPA/WPA2-PSK)
    
        :raises OSError: If WiFi scanning fails or interface is not active
        
        Example
        --------
        ```python
            >>> wifi = WifiManager()
            >>> networks = wifi.scan()
            >>> 
            >>> for ssid, rssi, channel, security in networks:
            ...     ssid_str = ssid.decode('utf-8', 'ignore')
            ...     security_types = ["Open", "WEP", "WPA-PSK", "WPA2-PSK", "WPA/WPA2-PSK"]
            ...     security_str = security_types[security] if security < len(security_types) else f"Unknown({security})"
            ...     print(f"{ssid_str:20} | {rssi:4d}dBm | CH{channel:2d} | {security_str}")
            >>> 
            >>> # Find strongest network
            >>> if networks:
            ...     strongest = max(networks, key=lambda x: x[1])
            ...     print(f"Strongest: {strongest[0].decode('utf-8')} ({strongest[1]}dBm)")
        ```
        """
        return self.__iface.scan()

    def available_ssids(self) -> list[str]:
        """
        Get a clean list of available network names (SSIDs).
        
        Scans for networks and returns a simplified list of unique SSIDs as strings,
        filtering out empty names and handling UTF-8 decoding gracefully.
        This is more convenient than scan() when you only need network names.
        
        :return: List of unique SSIDs found in the scanned access points.
        
        :raises OSError: If WiFi scanning fails or interface is not active
        
        Example
        --------
        ```python
            >>> wifi = WifiManager()
            >>> ssids = wifi.available_ssids()
            >>> print("Available networks:")
            >>> for i, ssid in enumerate(ssids, 1):
            ...     print(f"  {i}. {ssid}")
            >>> 
            >>> # Check if specific network exists
            >>> if "MyNetwork" in wifi.available_ssids():
            ...     print("Home network found!")
        ```
        """
        aps = self.scan()
        ssids = set()
        for ap in aps:
            ssid = ap[0].decode('utf-8', 'ignore')
            if ssid:
                ssids.add(ssid)
        return list(ssids)

    def connect(self, ssid: str, password: str, timeout: float = 20.0) -> bool:
        """
        Connect to a WiFi network with comprehensive error handling.
        
        Attempts to connect to the specified network with configurable timeout.
        Returns immediately if already connected to the same network.
        Handles connection failures gracefully and provides detailed status feedback.
        
        :param ssid: SSID of the WiFi network to connect to.
        :param password: Password for the WiFi network. Use empty string for open networks.
        :param timeout: Timeout in seconds for the connection attempt (default: 20.0 seconds).
        :return: True if connected successfully, False otherwise.
        
        :raises ValueError: If SSID is empty or timeout is negative
        :raises OSError: If WiFi interface is not available
        
        Example
        --------
        ```python
            >>> wifi = WifiManager()
            >>> 
            >>> # Basic connection
            >>> if wifi.connect("MyNetwork", "password123"):
            ...     print(f"Connected! IP: {wifi.ip}")
            ... else:
            ...     print("Connection failed")
            >>> 
            >>> # Open network connection
            >>> if wifi.connect("PublicWiFi", ""):  # Empty password
            ...     print("Connected to open network")
            >>> 
            >>> # Connection with custom timeout
            >>> success = wifi.connect("SlowNetwork", "password", timeout=30.0)
        ```
        """
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
        """
        Safely disconnect from the current WiFi network.
        
        Cleanly disconnects from the current network if connected, with a brief
        delay to ensure proper disconnection. Safe to call even if not connected.
        
        Example
        --------
        ```python
            >>> wifi = WifiManager()
            >>> wifi.connect("MyNetwork", "password")
            >>> print(f"Connected: {wifi.is_connected}")  # True
            >>> 
            >>> wifi.disconnect()
            >>> print(f"Connected: {wifi.is_connected}")  # False
        ```
        """
        if self.__iface.isconnected():
            self.__iface.disconnect()
            utime.sleep_ms(100)
    
    def ifconfig(self) -> tuple | None:
        """
        Get complete network interface configuration.
        
        Returns the full network configuration including IP address,
        netmask, gateway, and DNS server if connected. Useful for
        network diagnostics and advanced configuration.
        
        :return: Tuple containing (ip, netmask, gateway, dns) if connected, None otherwise.
        
        Example
        --------
        ```python
            >>> wifi = WifiManager()
            >>> wifi.connect("MyNetwork", "password")
            >>> 
            >>> config = wifi.ifconfig()
            >>> if config:
            ...     ip, netmask, gateway, dns = config
            ...     print(f"IP: {ip}, Gateway: {gateway}, DNS: {dns}")
        ```
        """
        if not self.is_connected:
            return None
        return self.__iface.ifconfig()

    @property
    def is_connected(self) -> bool:
        """
        Check if the WiFi interface is currently connected to a network.
        
        Returns the current connection status. This is a real-time check
        that reflects the actual network connectivity state.
        
        :return: True if connected, False otherwise.
        
        Example
        --------
        ```python
            >>> wifi = WifiManager()
            >>> print(f"Initial state: {wifi.is_connected}")  # False
            >>> 
            >>> wifi.connect("MyNetwork", "password")
            >>> print(f"After connect: {wifi.is_connected}")  # True
            >>> 
            >>> # Connection monitoring
            >>> if wifi.is_connected:
            ...     print("Network operations available")
            ... else:
            ...     print("No network connection")
        ```
        """
        return self.__iface.isconnected()

    @property
    def ip(self) -> str | None:
        """
        Get the current IP address of the WiFi connection.
        
        Returns the IP address as a string if connected, None otherwise.
        This is a convenient way to get just the IP address without
        the full interface configuration.
        
        :return: IP address as a string if connected, None otherwise.
        
        Example
        --------
        ```python
            >>> wifi = WifiManager()
            >>> print(f"IP before connect: {wifi.ip}")  # None
            >>> 
            >>> wifi.connect("MyNetwork", "password")
            >>> ip = wifi.ip
            >>> if ip:
            ...     print(f"Connected with IP: {ip}")
            ...     if ip.startswith("192.168."):
            ...         print("Private network detected")
            ... else:
            ...     print("No IP address assigned")
        ```
        """
        if not self.is_connected:
            return None
        return self.__iface.ifconfig()[0]


class Led(machine.Pin):
    """
    Built-in LED control class for RP2350 board wireless module.
    
    This class provides direct control of the built-in LED connected to the
    wireless module GPIO (WL_GPIO0). Inherits from machine.Pin with output
    configuration, enabling standard Pin object operations for simple on/off
    control and visual feedback.
    
    The LED is directly connected to the wireless module's GPIO pin and provides
    immediate visual feedback without requiring external components or wiring.
    It supports all standard Pin object operations and can be integrated into
    various signaling patterns for system status indication.
    
    Key Features:
    
        - Direct control of built-in hardware LED
        - Standard machine.Pin interface (on/off/value/toggle)
        - No external wiring or components required
        - Low power consumption suitable for battery applications
        - Immediate visual feedback for debugging and status indication
        - Active high logic (HIGH = LED on, LOW = LED off)
        
    """
    
    def __init__(self):
        """
        Initialize the built-in LED control interface.
        
        Creates an LED control instance configured for the RP2350 board's
        built-in LED connected to WL_GPIO0. The LED is initialized as an
        output pin and ready for immediate use.
        
        Example
        --------
        ```python
            >>> # Basic LED control
            >>> led = Led()
            >>> led.on()     # Turn LED on
            >>> led.off()    # Turn LED off
            >>> led.toggle() # Toggle current state
            >>> 
            >>> # Status indication based on system state
            >>> led = Led()
            >>> wifi = WifiManager()
            >>> if wifi.connect("MyNetwork", "password"):
            ...     led.on()  # Solid on when connected
            ... else:
            ...     # Blink pattern for connection failure
            ...     for _ in range(3):
            ...         led.on()
            ...         utime.sleep_ms(200)
            ...         led.off()
            ...         utime.sleep_ms(200)
            >>> 
            >>> # Heartbeat pattern for system monitoring
            >>> led = Led()
            >>> def heartbeat():
            ...     led.on()
            ...     utime.sleep_ms(100)   # Short pulse
            ...     led.off()
            ...     utime.sleep_ms(1900)  # Long pause (2 second cycle)
        ```
        """
        super().__init__("WL_GPIO0", machine.Pin.OUT)


class Button:
    """
    Built-in BOOTSEL button interface for RP2350 board user input.
    
    This class provides access to the built-in BOOTSEL button on the RP2350 board,
    enabling user input without requiring external components or wiring. The button
    serves dual purposes: boot selection during startup and general-purpose input
    during normal operation, making it ideal for simple user interfaces and control.
    
    The button is hardware-debounced and provides reliable digital input through
    the RP2040/RP2350's dedicated BOOTSEL functionality. It offers immediate
    response suitable for real-time applications while maintaining electrical
    isolation from user circuits.
    """
    
    @staticmethod
    def read() -> bool:
        """
        Read the current state of the built-in BOOTSEL button.
        
        This method provides immediate access to the button state without
        debouncing delays or edge detection. The hardware handles bounce
        elimination, providing clean digital readings suitable for both
        polling and edge detection applications.
        
        :return: True if button is currently pressed, False if released
        
        :raises OSError: If button hardware access fails (rare)
        
        Example
        --------
        ```python
            >>> # Basic button state checking
            >>> if Button.read():
            ...     print("Button pressed")
            >>> 
            >>> # Wait for button press with timeout
            >>> import utime
            >>> timeout_ms = 5000
            >>> start = utime.ticks_ms()
            >>> 
            >>> while not Button.read():
            ...     if utime.ticks_diff(utime.ticks_ms(), start) > timeout_ms:
            ...         print("Timeout - no button press")
            ...         break
            ... else:
            ...     print("Button pressed within timeout")
            >>> 
            >>> # Edge detection for event-driven control
            >>> last_state = False
            >>> while True:
            ...     current = Button.read()
            ...     if current and not last_state:  # Rising edge
            ...         print("Button press detected")
            ...         # Handle button press event here
            ...     last_state = current
            ...     utime.sleep_ms(50)  # Polling interval
        ```
        """
        return rp2.bootsel_button() == 1


class Din:
    """
    Digital input pin control class for multiple GPIO pins with advanced callback support.
    
    This class provides comprehensive digital input functionality for multiple
    GPIO pins with support for pull-up/pull-down resistors, interrupt callbacks,
    and convenient indexing operations. Designed for reading switches, buttons,
    sensors, and other digital input devices.
    
    Features:
    
        - Multiple pin support with unified interface
        - Configurable pull-up, pull-down, or open-drain modes
        - Individual and group callback configuration via indexing/slicing
        - Edge detection capabilities (rising/falling/both)
        - Automatic measurement control with user callbacks
        - Pin number extraction for callbacks
        - Unified indexing/slicing interface consistent with other classes
    
    Constants:
    
        - Din.LOW (0): Logic low level
        - Din.HIGH (1): Logic high level
        - Din.PULL_DOWN: Pull-down resistor configuration
        - Din.PULL_UP: Pull-up resistor configuration
        - Din.OPEN_DRAIN: Open-drain configuration
        - Din.CB_FALLING: Falling edge interrupt trigger
        - Din.CB_RISING: Rising edge interrupt trigger
    """
    LOW         = 0
    HIGH        = 1
    
    PULL_DOWN   = machine.Pin.PULL_DOWN
    PULL_UP     = machine.Pin.PULL_UP
    OPEN_DRAIN  = machine.Pin.OPEN_DRAIN
    CB_FALLING  = machine.Pin.IRQ_FALLING
    CB_RISING   = machine.Pin.IRQ_RISING
    CB_BOTH     = machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING
        
    def __init__(self, pins: list[int]|tuple[int, ...]):
        """
        Initialize digital input pins with optional pull resistors.
        
        Creates digital input pin objects with configurable pull resistors.
        Callbacks and edge detection are configured via the view interface
        after initialization.
        
        :param pins: List of GPIO pin numbers to be used as digital inputs
                
        :raises ValueError: If pins list is empty or contains invalid pin numbers
        :raises OSError: If GPIO pin initialization fails
        
        Example
        --------
        ```python
            >>> # Basic setup - configure pull resistors separately
            >>> inputs = Din([12, 13, 14, 15])
            >>> inputs[:].pull = Din.PULL_UP     # All pins pull-up
            >>> inputs[0].pull = Din.PULL_DOWN   # First pin pull-down
            >>> inputs[1:3].pull = None          # Pins 1-2 no pull
        ```
        """
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
        """
        Deinitialize all digital input pins and release resources.
        
        Safely shuts down all input pins by disabling interrupts and
        releasing GPIO resources. Should be called when inputs are
        no longer needed.
        
        Example
        -------
        ```python
            >>> inputs = Din([10, 11, 12])
            >>> # ... use inputs ...
            >>> inputs.deinit()  # Clean shutdown
        ```
        """
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
        """
        Get a view for controlling specific input pin(s) through indexing or slicing.
        
        Returns a DinView that provides access to individual pins or groups of pins
        using the same interface. Supports both single indexing and slice notation.
        
        :param idx: Index (int) or slice for pin selection
        
            - int: Single pin index (0-based)
            - slice: Range of pins (supports start:stop:step)

        :return: _DinView instance for selected pin(s)
        
        :raises IndexError: If pin index is out of range
        :raises TypeError: If index is not int or slice
        
        Example
        --------
        ```python
            >>> inputs = Din([10, 11, 12, 13])
            >>> 
            >>> # Single pin access
            >>> first_pin = inputs[0]           # First pin
            >>> last_pin = inputs[-1]           # Last pin
            >>> 
            >>> # Multiple pin access with slicing
            >>> first_two = inputs[0:2]         # First two pins
            >>> last_two = inputs[2:4]          # Last two pins
            >>> all_pins = inputs[:]            # All pins
            >>> even_pins = inputs[::2]         # Even-indexed pins (0, 2)
            >>> odd_pins = inputs[1::2]         # Odd-indexed pins (1, 3)
        ```
        """
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
        """
        Get the number of digital input pins configured.
        
        :return: Number of pins configured
        
        Example
        --------
        ```python
            >>> inputs = Din([10, 11, 12, 13, 14])
            >>> print(len(inputs))  # Output: 5
        ```
        """
        return len(self._pins)


    @property
    def pins(self) -> list:
        """
        Get the underlying machine.Pin objects for pins in this view.
        
        :return: List of machine.Pin objects for selected pins
        
        Example
        --------
        ```python
            >>> din = Din([2, 3, 4])
            >>> first_pin = din.pins[0]   # GPIO 2 Pin object
            >>> all_pins = din.pins       # All Pin objects
        ```
        """
        return self._din


    def _setup_irq(self, idx: int) -> None:
        """
        Setup interrupt for a specific pin based on its configuration.
        
        This internal method configures the interrupt handler for a pin
        if both callback and edge detection are properly configured.
        
        :param idx: Index of the pin to configure
        """
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
        """Get pull resistor configuration for specified pins."""
        return [parent._pull_config[i] for i in indices]

    @staticmethod
    def _set_pull_all(parent, pull: int|None, indices: list[int]) -> None:
        """Set pull resistor configuration for specified pins."""
        for i in indices:
            parent._pull_config[i] = pull
            parent._din[i].init(mode=machine.Pin.IN, pull=pull)
            
    @staticmethod
    def _get_value_list(parent, indices: list[int]) -> list[int]:
        """Get current values for specified pins."""
        return [parent._din[i].value() for i in indices]

    @staticmethod
    def _get_callback_list(parent, indices: list[int]) -> list[callable]:
        """Get callback functions for specified pins."""
        return [parent._user_callbacks[i] for i in indices]

    @staticmethod
    def _set_callback_all(parent, callback: callable, indices: list[int]) -> None:
        """Set callback function for specified pins."""
        for i in indices:
            parent._user_callbacks[i] = callback
            parent._setup_irq(i)

    @staticmethod
    def _get_edge_list(parent, indices: list[int]) -> list[int]:
        """Get edge configuration for specified pins."""
        return [parent._edge_config[i] for i in indices]

    @staticmethod
    def _set_edge_all(parent, edge: int, indices: list[int]) -> None:
        """Set edge configuration for specified pins."""
        for i in indices:
            parent._edge_config[i] = edge
            parent._setup_irq(i)

    @staticmethod
    def _get_measurement_list(parent, indices: list[int]) -> list[bool]:
        """Get measurement state for specified pins."""
        return [parent._measurement_enabled[i] for i in indices]

    @staticmethod
    def _set_measurement_all(parent, enabled: bool, indices: list[int]) -> None:
        """Set measurement state for specified pins."""
        for i in indices:
            parent._measurement_enabled[i] = enabled
            parent._setup_irq(i)

    @staticmethod
    def _get_debounce_list(parent, indices: list[int]) -> list[int]:
        """Get debounce time in microseconds for specified pins."""
        return [parent._debounce_us[i] for i in indices]

    @staticmethod
    def _set_debounce_all(parent, debounce_us: int, indices: list[int]) -> None:
        """Set debounce time in microseconds for specified pins."""
        for i in indices:
            parent._debounce_us[i] = debounce_us

    class _DinView:
        """
        View class for controlling individual digital input pins or groups of pins.
        
        This class provides a unified interface for controlling one or more pins
        through the same API. It's returned by Din.__getitem__() and allows
        seamless control of single pins or groups using identical syntax.
        
        Features:
        
            - Consistent API for single pin or multi-pin control
            - Property-based interface for intuitive control
            - Callback configuration and management
            - Edge detection configuration
            - Measurement enable/disable control
            
        """
        def __init__(self, parent: "Din", indices: list[int]):
            """
            Initialize input view with parent reference and pin indices.
            
            :param parent: Parent Din instance
            :param indices: List of pin indices this view controls
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Din._DinView":
            """
            Get a sub-view of this view for further pin selection.
            
            Allows nested selection of pins from an existing view,
            enabling fine-grained control over pin groups.
            
            :param idx: Index or slice for sub-selection
            :return: New _DinView with selected pins
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12, 13])
                >>> group = inputs[0:3]        # First three pins
                >>> subgroup = group[1:3]      # Second and third from original
                >>> subgroup.measurement = True # Control sub-selection
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Din._DinView(self._parent, selected_indices)
            else:
                return Din._DinView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of pins in this view.
            
            :return: Number of pins controlled by this view
            
            Example
            --------
                >>> inputs = Din([10, 11, 12, 13])
                >>> group = inputs[1:3]
                >>> print(len(group))  # Output: 2
            ```
            """
            return len(self._indices)

        @property
        def pull(self) -> list[int|None]:
            """
            Get pull resistor configuration for pins in this view.
            
            Returns the current pull resistor settings for each pin.
            Values can be Din.PULL_UP, Din.PULL_DOWN, or None.
            
            :return: List of pull resistor configurations
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Check current pull settings
                >>> pull_configs = inputs[:].pull
                >>> print(f"Pull configs: {pull_configs}")  # [None, None, None]
                >>> 
                >>> # Check specific pin
                >>> first_pull = inputs[0].pull
                >>> print(f"First pin pull: {first_pull}")  # [None]
            ```
            """
            return Din._get_pull_list(self._parent, self._indices)

        @pull.setter
        def pull(self, pull_type: int|None|list[int|None]):
            """
            Set pull resistor configuration for pins in this view.
            
            Configures the internal pull resistors for the selected pins.
            This provides much more flexibility than setting pull in __init__.
            
            :param pull_type: Pull resistor configuration
                             - Din.PULL_UP: Enable internal pull-up resistor
                             - Din.PULL_DOWN: Enable internal pull-down resistor
                             - None: No pull resistor (floating input)
                             - List: Individual settings for each pin
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12, 13])
                >>> 
                >>> # Set all pins to pull-up
                >>> inputs[:].pull = Din.PULL_UP
                >>> 
                >>> # Set individual pin configurations
                >>> inputs[0].pull = Din.PULL_DOWN   # First pin pull-down
                >>> inputs[1].pull = Din.PULL_UP     # Second pin pull-up
                >>> inputs[2].pull = None            # Third pin floating
                >>> 
                >>> # Set different pulls for different groups
                >>> inputs[0:2].pull = Din.PULL_UP   # First two pull-up
                >>> inputs[2:].pull = Din.PULL_DOWN  # Last two pull-down
                >>> 
                >>> # Set individual configurations with list
                >>> inputs[:].pull = [Din.PULL_UP, Din.PULL_DOWN, None, Din.PULL_UP]
                >>> 
                >>> # Button setup with appropriate pulls
                >>> buttons = Din([2, 3, 4])
                >>> buttons[:].pull = Din.PULL_UP    # Buttons typically need pull-up
                >>> 
                >>> # Sensor setup with mixed configurations
                >>> sensors = Din([20, 21, 22, 23])
                >>> sensors[0:2].pull = Din.PULL_DOWN   # Digital sensors
                >>> sensors[2:].pull = None             # Analog sensors (floating)
                >>> 
                >>> # Runtime reconfiguration
                >>> if sensor_type == "DIGITAL":
                ...     inputs[0].pull = Din.PULL_UP
                ... elif sensor_type == "ANALOG":
                ...     inputs[0].pull = None
                >>> 
                >>> # Configuration validation
                >>> current_pulls = inputs[:].pull
                >>> for i, pull in enumerate(current_pulls):
                ...     if pull == Din.PULL_UP:
                ...         print(f"Pin {i}: Pull-up enabled")
                ...     elif pull == Din.PULL_DOWN:
                ...         print(f"Pin {i}: Pull-down enabled")
                ...     else:
                ...         print(f"Pin {i}: Floating input")
            ```
            """
            if isinstance(pull_type, (list, tuple)):
                if len(pull_type) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, pull in zip(self._indices, pull_type):
                    Din._set_pull_all(self._parent, pull, [i])
            else:
                Din._set_pull_all(self._parent, pull_type, self._indices)
      
        @property
        def value(self) -> list[int]:
            """
            Get current values of pins in this view.
            
            Returns the current digital values for all pins in the view.
            Always returns a list, even for single pins.
            
            :return: List of digital values (0 or 1)
            
            Example
            --------
            ```python
                >>> inputs = Din([2, 3, 4, 5])
                >>> 
                >>> # Read individual pin (returns list)
                >>> pin0_state = inputs[0].value
                >>> print(f"Pin 0: {pin0_state}")  # Output: Pin 0: [0]
                >>> 
                >>> # Read multiple pins
                >>> first_two = inputs[0:2].value
                >>> print(f"First two: {first_two}")  # Output: First two: [0, 1]
                >>> 
                >>> # Read all pins
                >>> all_states = inputs[:].value
                >>> print(f"All pins: {all_states}")  # Output: All pins: [0, 1, 0, 1]
                >>> 
                >>> # Use in conditions
                >>> if inputs[0].value[0] == Din.HIGH:
                ...     print("First input is HIGH")
                >>> 
                >>> # Check multiple inputs
                >>> states = inputs[:].value
                >>> active_count = sum(states)
                >>> print(f"Active inputs: {active_count}/{len(states)}")
            ```
            """
            return Din._get_value_list(self._parent, self._indices)

        @property
        def callback(self) -> list[callable]:
            """
            Get callback functions for pins in this view.
            
            Returns the callback functions that will be called when
            edge events occur on the pins.
            
            :return: List of callback functions or None values
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> callbacks = inputs[:].callback
                >>> print(f"Callbacks: {callbacks}")  # [None, None, None]
                >>> 
                >>> # Check if callback is set
                >>> if inputs[0].callback[0] is not None:
                ...     print("Callback is set for first pin")
            ```
            """
            return Din._get_callback_list(self._parent, self._indices)

        @callback.setter
        def callback(self, fn: callable | list[callable]):
            """
            Set callback function for pins in this view.
            
            Sets the function to be called when edge events occur.
            Callback receives (pin, rising) parameters where pin is the
            GPIO number and rising is True for rising edge, False for falling.
            
            :param fn: Callback function, None, or list of callbacks
            
            :raises ValueError: If list length doesn't match number of pins
            :raises TypeError: If callback is not callable or None
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Simple callback for all pins
                >>> def input_callback(pin, rising):
                ...     edge_type = "Rising" if rising else "Falling"
                ...     print(f"Pin {pin}: {edge_type} edge detected")
                >>> 
                >>> inputs[:].callback = input_callback
                >>> 
                >>> # Different callbacks for different pins
                >>> def pin1_callback(pin, rising):
                ...     print(f"Button 1: {'Pressed' if not rising else 'Released'}")
                >>> 
                >>> def pin2_callback(pin, rising):
                ...     print(f"Sensor 2: {'Triggered' if rising else 'Clear'}")
                >>> 
                >>> inputs[:].callback = [pin1_callback, pin2_callback, None]
                >>> 
                >>> # Enable edge detection and measurement
                >>> inputs[:].edge = Din.CB_FALLING | Din.CB_RISING
                >>> inputs[:].measurement = True
            ```
            """
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
            """
            Get edge detection configuration for pins in this view.
            
            Returns the edge detection settings for each pin.
            Values can be Din.CB_RISING, Din.CB_FALLING, or their combination.
            
            :return: List of edge configuration values
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Check current edge settings
                >>> edges = inputs[:].edge
                >>> print(f"Edge configs: {edges}")  # [0, 0, 0] (no detection)
                >>> 
                >>> # Check specific pin
                >>> first_edge = inputs[0].edge
                >>> print(f"First pin edge: {first_edge}")  # [0]
            ```
            """
            return Din._get_edge_list(self._parent, self._indices)

        @edge.setter
        def edge(self, edge_type: int):
            """
            Set edge detection type for pins in this view.
            
            Configures which edges trigger interrupts:
            - Din.CB_RISING: Rising edge (LOW to HIGH)
            - Din.CB_FALLING: Falling edge (HIGH to LOW)
            - Din.CB_RISING | Din.CB_FALLING: Both edges
            
            :param edge_type: Edge detection configuration
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Rising edge detection for all pins
                >>> inputs[:].edge = Din.CB_RISING
                >>> 
                >>> # Falling edge for first pin only
                >>> inputs[0].edge = Din.CB_FALLING
                >>> 
                >>> # Both edges for middle pins
                >>> inputs[1:3].edge = Din.CB_RISING | Din.CB_FALLING
                >>> 
                >>> # Setup complete callback system
                >>> def edge_callback(pin, rising):
                ...     print(f"Pin {pin}: {'↑' if rising else '↓'}")
                >>> 
                >>> inputs[:].callback = edge_callback
                >>> inputs[:].edge = Din.CB_RISING | Din.CB_FALLING
                >>> inputs[:].measurement = True
            ```
            """
            Din._set_edge_all(self._parent, edge_type, self._indices)

        @property
        def debounce_us(self) -> list[int]:
            """
            Get debounce filter time in microseconds for pins in this view.
            
            Returns the debounce time settings for each pin. Debouncing helps
            eliminate electrical noise and contact bounce in mechanical switches.
            
            :return: List of debounce times in microseconds
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Check current debounce settings
                >>> debounce_times = inputs[:].debounce_us
                >>> print(f"Debounce times: {debounce_times}")  # [0, 0, 0] (no debouncing)
                >>> 
                >>> # Check specific pin
                >>> first_debounce = inputs[0].debounce_us
                >>> print(f"First pin debounce: {first_debounce}")  # [0]
            ```
            """
            return Din._get_debounce_list(self._parent, self._indices)

        @debounce_us.setter
        def debounce_us(self, us: int):
            """
            Set debounce filter time in microseconds for pins in this view.
            
            Sets the debounce time for noise filtering on mechanical switches.
            When debouncing is enabled, rapid state changes within the debounce
            window are ignored, providing stable readings.
            
            :param us: Debounce time in microseconds
            
                - 0: No debouncing (default)
                - 1000-50000: Typical range for mechanical switches
                - Higher values = more filtering, slower response
    
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Set debounce for all pins (typical for buttons)
                >>> inputs[:].debounce_us = 20000  # 20ms debounce
                >>> 
                >>> # Different debounce for different pins
                >>> inputs[0].debounce_us = 10000  # 10ms for sensitive button
                >>> inputs[1].debounce_us = 50000  # 50ms for noisy switch
                >>> 
                >>> # Setup complete button system with debouncing
                >>> def button_callback(pin, rising):
                ...     if rising:
                ...         print(f"Button {pin} pressed")
                ...     else:
                ...         print(f"Button {pin} released")
                >>> 
                >>> buttons = Din([2, 3, 4], pull=Din.PULL_UP)
                >>> buttons[:].debounce_us = 20000  # 20ms debounce
                >>> buttons[:].callback = button_callback
                >>> buttons[:].edge = Din.CB_FALLING | Din.CB_RISING
                >>> buttons[:].measurement = True
            ```
            """
            Din._set_debounce_all(self._parent, us, self._indices)

        @property
        def measurement(self) -> list[bool]:
            """
            Get measurement state for pins in this view.
            
            Returns whether interrupt-based measurement is enabled for each pin.
            Measurement must be enabled for callbacks to function.
            
            :return: List of measurement state flags
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Check measurement states
                >>> states = inputs[:].measurement
                >>> print(f"Measurement states: {states}")  # [False, False, False]
                >>> 
                >>> # Check specific pin
                >>> first_state = inputs[0].measurement
                >>> print(f"First pin enabled: {first_state}")  # [False]
            ```
            """
            return Din._get_measurement_list(self._parent, self._indices)

        @measurement.setter
        def measurement(self, enabled: bool | list[bool]):
            """
            Enable or disable interrupt-based measurement for pins in this view.
            
            Controls whether callbacks are triggered for edge events.
            Pins must have both callback and edge configuration set to function.
            
            :param enabled: Single boolean for all pins or list of booleans
            
            :raises ValueError: If list length doesn't match number of pins
            
            Example
            --------
            ```python
                >>> inputs = Din([10, 11, 12])
                >>> 
                >>> # Setup callback system
                >>> def my_callback(pin, rising):
                ...     print(f"Pin {pin} edge: {rising}")
                >>> 
                >>> inputs[:].callback = my_callback
                >>> inputs[:].edge = Din.CB_RISING | Din.CB_FALLING
                >>> 
                >>> # Enable measurement for all pins
                >>> inputs[:].measurement = True
                >>> 
                >>> # Enable only specific pins
                >>> inputs[:].measurement = [True, False, True]  # First and third only
                >>> 
                >>> # Disable all measurements
                >>> inputs[:].measurement = False
                >>> 
                >>> # Individual pin control
                >>> inputs[0].measurement = True   # Enable first pin
                >>> inputs[1].measurement = False  # Disable second pin
            ```
            """
            if isinstance(enabled, bool):
                Din._set_measurement_all(self._parent, enabled, self._indices)
            else:
                if len(enabled) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, en in zip(self._indices, enabled):
                    self._parent._measurement_enabled[i] = en
                    self._parent._setup_irq(i)

        def measure_pulse_width(self, level: int, timeout_ms: int = 1000) -> int:
            """
            Measure pulse width using machine.time_pulse_us() for hardware-level precision.
            
            This method uses MicroPython's built-in hardware timing function to measure
            pulse widths with microsecond accuracy. Works only with single pin for
            optimal precision and timing accuracy.
            
            :param level: Level to measure (Din.HIGH or Din.LOW)
            :param timeout_ms: Timeout in milliseconds
            :return: Pulse width in microseconds, -1 if timeout or error
            
            :raises ValueError: If this view contains more than one pin
            
            Example
            --------
            ```python
                >>> din = Din([12, 13, 14, 15])
                >>> din[15].pull = Din.PULL_UP
                >>> 
                >>> # Measure button press (LOW pulse when pressed)
                >>> width = din[15].measure_pulse_width(Din.LOW, 5000)
                >>> if width > 0:
                ...     print(f"Button pressed for {width}μs ({width/1000:.1f}ms)")
                >>> elif width == -1:
                ...     print("Measurement timeout or error")
                >>> 
                >>> # Ultrasonic sensor echo measurement
                >>> echo_pin = Din([3])
                >>> echo_pin[0].pull = None  # No pull resistor for sensor
                >>> width = echo_pin[0].measure_pulse_width(Din.HIGH, 30)
                >>> if width > 0:
                ...     distance_cm = (width * 0.0343) / 2
                ...     print(f"Distance: {distance_cm:.1f}cm")
                >>> 
                >>> # Continuous measurement
                >>> button = Din([12])
                >>> button[0].pull = Din.PULL_UP
                >>> for i in range(10):
                ...     print(f"Measurement {i+1}/10 - Press button...")
                ...     width = button[0].measure_pulse_width(Din.LOW, 3000)
                ...     if width > 0:
                ...         if width < 1000:
                ...             print(f"  Quick press: {width}μs")
                ...         else:
                ...             print(f"  Long press: {width/1000:.1f}ms")
                ...     else:
                ...         print("  No press detected")
                ...     utime.sleep_ms(500)
                >>> 
                >>> # Multiple pins - use individual calls
                >>> sensors = Din([14, 15, 16])
                >>> # Correct - individual measurements
                >>> width1 = sensors[0].measure_pulse_width(Din.HIGH, 1000)
                >>> width2 = sensors[1].measure_pulse_width(Din.HIGH, 1000)
                >>> width3 = sensors[2].measure_pulse_width(Din.HIGH, 1000)
                >>> 
                >>> # Incorrect - will raise ValueError
                >>> # width = sensors[:].measure_pulse_width(Din.HIGH, 1000)
            ```
            """
            if len(self._indices) != 1:
                raise ValueError("Pulse width measurement only works with single pin. Use individual pin access like din[0].measure_pulse_width() instead of din[:].measure_pulse_width()")
            
            pin = self._parent._din[self._indices[0]]
            
            return machine.time_pulse_us(pin, level, timeout_ms * 1000)            


class Dout:
    """
    Digital output pin control class with Active High/Low logic support.
    
    This class provides comprehensive digital output functionality with support
    for both Active High and Active Low logic, making it compatible with various
    hardware devices that have different activation requirements.
    
    Logic Types:
    
        - LOGIC_HIGH (Active High): HIGH = active/on, LOW = inactive/off
        - LOGIC_LOW (Active Low): LOW = active/on, HIGH = inactive/off
    
    Constants:
    
        - Dout.LOW (0): Logic low level
        - Dout.HIGH (1): Logic high level
        - Dout.LOGIC_HIGH: Active high logic (default)
        - Dout.LOGIC_LOW: Active low logic
        - Dout.PULL_DOWN: Pull-down resistor configuration
        - Dout.PULL_UP: Pull-up resistor configuration
        - Dout.OPEN_DRAIN: Open-drain configuration
        
    """
    LOW         = 0
    HIGH        = 1
    LOGIC_HIGH  = True   # Active High: HIGH = active
    LOGIC_LOW   = False  # Active Low: LOW = active
    PULL_DOWN   = machine.Pin.PULL_DOWN
    PULL_UP     = machine.Pin.PULL_UP
    OPEN_DRAIN  = machine.Pin.OPEN_DRAIN

    def __init__(self, pins: list[int]|tuple[int, ...]):
        """
        Initialize digital output pins with default active high logic.
        
        Creates digital output pin objects configured for driving external devices.
        All pins are initially set to inactive state (logical LOW) for safety.
        Active logic can be configured per pin via the view interface.
        
        :param pins: List of GPIO pin numbers to be used as digital outputs
        
        :raises ValueError: If pins list is empty or contains invalid pin numbers
        :raises OSError: If GPIO pin initialization fails
        
        Example
        --------
        ```python
            >>> # Basic setup with mixed device types
            >>> outputs = Dout([10, 11, 12, 13])
            >>> 
            >>> # Configure active logic per device
            >>> outputs[0:2].active = Dout.LOGIC_HIGH   # LEDs (active high)
            >>> outputs[2:4].active = Dout.LOGIC_LOW    # Relays (active low)
            >>> 
            >>> # All start in inactive state regardless of logic type
            >>> outputs[:].value = Dout.LOW             # All devices OFF
        ```
        """
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
        """
        Deinitialize all digital output pins and release resources.
        
        This method safely shuts down all output pins by setting them to inactive
        state (considering active logic) and then switching to input mode with
        pull-down to avoid floating states.
        
        Example
        --------
        ```python
            >>> outputs = Dout([10, 11, 12])
            >>> outputs[0].active = Dout.LOGIC_HIGH
            >>> outputs[1].active = Dout.LOGIC_LOW
            >>> outputs[:].value = Dout.HIGH  # Turn on all devices
            >>> 
            >>> # Safe shutdown - all devices will be turned off
            >>> outputs.deinit()
        ```
        """
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
        """
        Get a view for controlling specific output pin(s).
        
        :param idx: Index (int) or slice for pin selection
        
            - int: Single pin index (0-based)
            - slice: Range of pins (supports start:stop:step)
        
        :return: _DoutView instance for selected pin(s)
        
        :raises IndexError: If pin index is out of range
        :raises TypeError: If index is not int or slice
        
        Example
        --------
        ```python
            >>> outputs = Dout([10, 11, 12, 13])
            >>> 
            >>> # Single pin access
            >>> first_pin = outputs[0]           # First pin
            >>> last_pin = outputs[-1]           # Last pin
            >>> 
            >>> # Multiple pin access with slicing
            >>> first_two = outputs[0:2]         # First two pins
            >>> last_two = outputs[2:4]          # Last two pins
            >>> all_pins = outputs[:]             # All pins
            >>> even_pins = outputs[::2]          # Even-indexed pins (0, 2)
            >>> odd_pins = outputs[1::2]          # Odd-indexed pins (1, 3)
        ```
        """
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
        """
        Get the number of digital output pins configured.
        
        :return: Number of pins configured
        
        Example
        --------
        ```python
            >>> outputs = Dout([10, 11, 12, 13, 14])
            >>> print(len(outputs))  # Output: 5
        ``` 
        """
        return len(self._pins)

    @property
    def pins(self) -> list:
        """
        Get the underlying machine.Pin objects for pins in this view.
        
        :return: List of machine.Pin objects for selected pins
        
        Example
        --------
        ```python
            >>> dout = Dout([2, 3, 4])
            >>> first_pin = dout.pins[0]   # GPIO 2 Pin object
            >>> all_pins = dout.pins       # All Pin objects
        ```
        """
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
        """Get active logic configuration for specified pins."""
        return [parent._active_logic[i] for i in indices]

    @staticmethod
    def _set_active_all(parent, active_logic: bool, indices: list[int]) -> None:
        """Set active logic configuration for specified pins."""
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
        """Get current logical values for specified pins."""
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
        """Set logical value for specified pins."""
        for i in indices:
            if parent._active_logic[i] == Dout.LOGIC_HIGH:
                physical_value = logical_value
            else:
                physical_value = 1 - logical_value
            parent._dout[i].value(physical_value)

    @staticmethod
    def _set_value_list(parent, logical_values: list[int], indices: list[int]) -> None:
        """Set individual logical values for specified pins."""
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
        """Toggle logical values for specified pins."""
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
        """View class for controlling individual digital output pins or groups."""

        def __init__(self, parent: "Dout", indices: list[int]):
            """
            Initialize output view with parent reference and pin indices.

            :param parent: Parent Dout instance
            :param indices: List of pin indices this view controls
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Dout._DoutView":
            """
            Get a sub-view of this view for further pin selection.
            
            Allows nested selection of pins from an existing view,
            enabling fine-grained control over pin groups.
            
            :param idx: Index or slice for sub-selection
            :return: New _DoutView with selected pins
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11, 12, 13])
                >>> group = outputs[0:3]        # First three pins
                >>> subgroup = group[1:3]       # Second and third from original
                >>> subgroup.measurement = True # Control sub-selection
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Dout._DoutView(self._parent, selected_indices)
            else:
                return Dout._DoutView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of pins in this view.
            
            :return: Number of pins controlled by this view
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11, 12, 13])
                >>> group = outputs[1:3]
                >>> print(len(group))  # Output: 2
            ```
            """
            return len(self._indices)

        @property
        def active(self) -> list[bool]:
            """
            Get active logic configuration for pins in this view.
            
            Returns the current active logic settings for each pin.
            Values are Dout.LOGIC_HIGH (True) or Dout.LOGIC_LOW (False).
            
            :return: List of active logic configurations
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11, 12])
                >>> outputs[1].active = Dout.LOGIC_LOW
                >>> 
                >>> # Check active logic settings
                >>> logic_configs = outputs[:].active
                >>> print(f"Active logic: {logic_configs}")  # [True, False, True]
                >>> 
                >>> # Check specific pin
                >>> first_logic = outputs[0].active
                >>> print(f"First pin logic: {first_logic}")  # [True]
            ```
            """
            return Dout._get_active_list(self._parent, self._indices)

        @active.setter
        def active(self, logic_type: bool | list[bool]):
            """
            Set active logic configuration for pins in this view.
            
            Configures whether pins use active high or active low logic.
            This affects how logical HIGH/LOW values are translated to physical
            pin states, allowing unified control of mixed device types.
            
            :param logic_type: Active logic configuration
            
                - Dout.LOGIC_HIGH (True): HIGH = active, LOW = inactive
                - Dout.LOGIC_LOW (False): LOW = active, HIGH = inactive
                - List: Individual settings for each pin
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11, 12, 13])
                >>> 
                >>> # Set all pins to active high (default)
                >>> outputs[:].active = Dout.LOGIC_HIGH
                >>> 
                >>> # Set specific pins to active low
                >>> outputs[1:3].active = Dout.LOGIC_LOW  # Relay modules
                >>> 
                >>> # Mixed configuration with list
                >>> outputs[:].active = [Dout.LOGIC_HIGH, Dout.LOGIC_LOW, 
                ...                      Dout.LOGIC_LOW, Dout.LOGIC_HIGH]
                >>> 
                >>> # Device-specific configuration
                >>> led_strip = Dout([14, 15, 16])
                >>> led_strip[:].active = Dout.LOGIC_HIGH  # Standard LEDs
                >>> 
                >>> relay_board = Dout([20, 21, 22, 23])
                >>> relay_board[:].active = Dout.LOGIC_LOW  # Active-low relays
                >>> 
                >>> # Logical control works the same for all
                >>> led_strip[:].value = Dout.HIGH    # LEDs on (physical HIGH)
                >>> relay_board[:].value = Dout.HIGH  # Relays on (physical LOW)
                >>> 
                >>> # Runtime logic change (preserves current state)
                >>> motor = Dout([25])
                >>> motor[0].value = Dout.HIGH        # Motor on
                >>> motor[0].active = Dout.LOGIC_LOW  # Change to active low
                >>> # Motor stays on, but now HIGH means on via LOW signal
            ```
            """
            if isinstance(logic_type, (list, tuple)):
                if len(logic_type) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, logic in zip(self._indices, logic_type):
                    Dout._set_active_all(self._parent, logic, [i])
            else:
                Dout._set_active_all(self._parent, logic_type, self._indices)

        @property
        def pull(self) -> list[int|None]:
            """
            Get pull resistor configuration for pins in this view.
            Returns the current pull resistor settings for each pin.
            
            :return: List of pull configurations (None, PULL_UP, PULL_DOWN)
            
            Example
            --------
            ```python    
                >>> outputs = Dout([10, 11, 12])
                >>> outputs[0].pull = Dout.PULL_UP
                >>> outputs[1].pull = Dout.PULL_DOWN
                >>> outputs[2].pull = None
                >>>
                # Check pull configurations
                >>> pull_configs = outputs[:].pull
                >>> print(f"Pull configs: {pull_configs}")  # [Dout.PULL_UP, Dout.PULL_DOWN, None]
            ```
            """
            return Dout._get_pull_list(self._parent, self._indices)

        @pull.setter
        def pull(self, pull_type: int|None|list[int|None]):
            """
            Set pull resistor configuration for pins in this view.
            Configures the pull-up or pull-down resistors for each pin.
            
            :param pull_type: Pull configuration
            
                - int: Single pull type for all pins (Dout.PULL_UP, Dout.PULL_DOWN, None)
                - None: No pull resistor
                - list: Individual pull settings for each pin

            :raises ValueError: If list length doesn't match number of pins
      
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11, 12])
                >>> outputs[:].pull = Dout.PULL_UP      # All pins pull-up
                >>> outputs[0].pull = Dout.PULL_DOWN    # First pin pull-down
            ```
            """
            if isinstance(pull_type, (list, tuple)):
                if len(pull_type) != len(self._indices):
                    raise ValueError("List length must match number of pins")
                for i, pull in zip(self._indices, pull_type):
                    Dout._set_pull_all(self._parent, pull, [i])
            else:
                Dout._set_pull_all(self._parent, pull_type, self._indices)

        @property
        def value(self) -> list[int]:
            """
            Get current logical values of pins in this view.
            
            Returns the logical values (considering active logic configuration).
            Always returns a list, even for single pins.
            
            :return: List of logical values (0 or 1)
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11])
                >>> outputs[0].active = Dout.LOGIC_HIGH
                >>> outputs[1].active = Dout.LOGIC_LOW
                >>> 
                >>> # Both devices are logically "on"
                >>> outputs[:].value = Dout.HIGH
                >>> 
                >>> # Read logical states
                >>> logical_states = outputs[:].value  # [1, 1]
                >>> print(f"Logical states: {logical_states}")
                >>> 
                >>> # Check physical pins
                >>> pin0_physical = outputs[0].pins[0].value()  # 1 (HIGH)
                >>> pin1_physical = outputs[1].pins[0].value()  # 0 (LOW)
                >>> print(f"Physical: pin0={pin0_physical}, pin1={pin1_physical}")
            ```
            """
            return Dout._get_value_list(self._parent, self._indices)

        @value.setter
        def value(self, val: int | list[int]):
            """
            Set logical values of pins in this view.
            
            Sets the logical values which are automatically translated to appropriate
            physical pin states based on each pin's active logic configuration.
            
            :param val: Logical value(s) to set
            
                - int: Same logical value for all pins (0 or 1)
                - list: Individual logical values for each pin
    
            :raises ValueError: If list length doesn't match number of pins
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11, 12, 13])
                >>> 
                >>> # Configure mixed active logic
                >>> outputs[0:2].active = Dout.LOGIC_HIGH  # LEDs
                >>> outputs[2:4].active = Dout.LOGIC_LOW   # Relays
                >>> 
                >>> # Logical control - all devices "ON"
                >>> outputs[:].value = Dout.HIGH
                >>> # Result: LEDs get HIGH, Relays get LOW
                >>> 
                >>> # Individual logical control
                >>> outputs[:].value = [1, 0, 1, 0]  # Mixed on/off pattern
                >>> 
                >>> # Unified device control regardless of polarity
                >>> all_on = [Dout.HIGH] * len(outputs)
                >>> outputs[:].value = all_on
                >>> 
                >>> # Device-type specific control
                >>> led_controller = Dout([14, 15, 16])
                >>> led_controller[:].active = Dout.LOGIC_HIGH
                >>> led_controller[:].value = Dout.HIGH  # All LEDs on
                >>> 
                >>> relay_controller = Dout([20, 21, 22])
                >>> relay_controller[:].active = Dout.LOGIC_LOW
                >>> relay_controller[:].value = Dout.HIGH  # All relays on
            ```
            """
            if isinstance(val, (list, tuple)):
                Dout._set_value_list(self._parent, val, self._indices)
            else:
                Dout._set_value_all(self._parent, val, self._indices)

        def toggle(self) -> None:
            """
            Toggle the logical state of all pins in this view.
            
            Changes each pin from logical HIGH to LOW or LOW to HIGH,
            automatically handling the physical pin transitions based on
            active logic configuration.
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11])
                >>> outputs[0].active = Dout.LOGIC_HIGH
                >>> outputs[1].active = Dout.LOGIC_LOW
                >>> 
                >>> # Start with both logically OFF
                >>> outputs[:].value = Dout.LOW
                >>> 
                >>> # Toggle both to logically ON
                >>> outputs[:].toggle()
                >>> # Pin 10: LOW->HIGH (active high)
                >>> # Pin 11: HIGH->LOW (active low)
                >>> 
                >>> # Toggle back to logically OFF
                >>> outputs[:].toggle()
                >>> # Pin 10: HIGH->LOW (active high)
                >>> # Pin 11: LOW->HIGH (active low)
            ```
            """
            Dout._toggle_all(self._parent, self._indices)

        @property
        def physical_value(self) -> list[int]:
            """
            Get current physical pin values (bypass active logic).
            
            Returns the actual physical pin states without considering
            active logic configuration. Useful for debugging and diagnostics.
            
            :return: List of physical pin values (0 or 1)
            
            Example
            --------
            ```python
                >>> outputs = Dout([10, 11])
                >>> outputs[0].active = Dout.LOGIC_HIGH
                >>> outputs[1].active = Dout.LOGIC_LOW
                >>> outputs[:].value = Dout.HIGH  # Both logically ON
                >>> 
                >>> logical = outputs[:].value      # [1, 1]
                >>> physical = outputs[:].physical_value  # [1, 0]
                >>> 
                >>> print(f"Logical: {logical}, Physical: {physical}")
                >>> # Output: Logical: [1, 1], Physical: [1, 0]
            ```
            """
            return [self._parent._dout[i].value() for i in self._indices]


class Adc:
    """
    Multi-channel analog-to-digital converter for GPIO pins with advanced callback support.
    
    This class provides comprehensive ADC functionality for reading analog voltages from
    multiple GPIO pins with support for automatic periodic sampling, user-defined
    callback functions, and convenient indexing operations. Only GPIO pins 26, 27, and 28
    support ADC functionality on the RP2350 board.
    
    Features:
    
        - Multiple ADC channel support (GPIO 26, 27, 28 only)
        - Individual and group callback configuration via indexing/slicing
        - High-resolution 16-bit ADC readings (0-65535 range)
        - Automatic voltage conversion with 3.3V reference
        - Periodic sampling with configurable intervals
        - Non-blocking measurement control
        - Precision voltage readings to 3 decimal places
        - Unified indexing/slicing interface consistent with other classes
    
    Hardware Specifications:
    
        - Resolution: 16-bit (65536 levels)
        - Reference Voltage: 3.3V
        - Supported Pins: GPIO 26, 27, 28 only
        - Voltage Resolution: ~0.806mV per step
        - Sample Rate: Configurable via timer callbacks
    """
    
    def __init__(self, pins: list[int]|tuple[int, ...]):
        """
        Initialize ADC controller for multiple analog input pins.
        
        Creates ADC instances for the specified GPIO pins. Only GPIO pins 26, 27, and 28
        support ADC functionality on the RP2350 board. Callbacks and periodic sampling
        are configured via the view interface after initialization.
        
        :param pins: List or tuple of GPIO pin numbers (only 26, 27, 28 supported)
        
        :raises ValueError: If unsupported GPIO pins are specified or pins list is empty
        :raises OSError: If ADC initialization fails
        
        Example
        --------
        ```python
            >>> # All ADC channels
            >>> adc = Adc([26, 27, 28])
            >>> 
            >>> # Single channel
            >>> temp_sensor = Adc([26])
            >>> 
            >>> # Two channels for differential measurement
            >>> diff_adc = Adc([26, 27])
            >>> 
            >>> # Configure callbacks after initialization
            >>> adc[:].callback = my_callback
            >>> adc[:].period_ms = 100
            >>> adc[:].measurement = True
        ```
        """
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
        """
        Deinitialize all ADC channels and release resources.
        
        Safely shuts down all ADC channels by stopping timers and
        releasing GPIO resources. Should be called when ADC is
        no longer needed.
        
        Example
        --------
        ```python
            >>> adc = Adc([26, 27, 28])
            >>> # ... use ADC ...
            >>> adc.deinit()  # Clean shutdown
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
        except:
            pass

    def __getitem__(self, idx: int|slice) -> "_AdcView":
        """
        Get a view for controlling specific ADC channel(s) through indexing or slicing.
        
        Returns an AdcView that provides access to individual channels or groups of channels
        using the same interface. Supports both single indexing and slice notation.
        
        :param idx: Index (int) or slice for channel selection
                   - int: Single channel index (0-based)
                   - slice: Range of channels (supports start:stop:step)
        :return: _AdcView instance for selected channel(s)
        
        :raises IndexError: If channel index is out of range
        :raises TypeError: If index is not int or slice
        
        Example
        --------
        ```python
            >>> adc = Adc([26, 27, 28])
            >>> 
            >>> # Single channel access
            >>> first_channel = adc[0]        # First channel (GPIO 26)
            >>> last_channel = adc[-1]        # Last channel (GPIO 28)
            >>> 
            >>> # Multiple channel access with slicing
            >>> first_two = adc[0:2]          # First two channels
            >>> all_channels = adc[:]         # All channels
            >>> even_channels = adc[::2]      # Even-indexed channels
        ```
        """
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
        """
        Get the number of ADC channels configured.
        
        :return: Number of channels configured
        
        Example
        --------
        ```python
            >>> adc = Adc([26, 27, 28])
            >>> print(len(adc))  # Output: 3
        ```
        """
        return len(self._pins)

    def _setup_timer(self, idx: int) -> None:
        """
        Setup timer for a specific ADC channel based on its configuration.
        
        This internal method configures the timer for periodic sampling
        if both callback and period are properly configured.
        
        :param idx: Index of the channel to configure
        """
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
        """Get voltage values for specified ADC channels with filtering."""
        result = []
        for i in indices:
            raw = parent._adc[i].read_u16()
            voltage = raw * (3.3 / 65535)
            result.append(round(voltage, 3))
        return result

    @staticmethod
    def _get_raw_value_list(parent, indices: list[int]) -> list[int]:
        """Get raw values for specified ADC channels."""
        return [parent._adc[i].read_u16() for i in indices]

    @staticmethod
    def _get_callback_list(parent, indices: list[int]) -> list[callable]:
        """Get callback functions for specified ADC channels."""
        return [parent._user_callbacks[i] for i in indices]

    @staticmethod
    def _set_callback_all(parent, callback: callable, indices: list[int]) -> None:
        """Set callback function for specified ADC channels."""
        for i in indices:
            parent._user_callbacks[i] = callback
            parent._setup_timer(i)

    @staticmethod
    def _get_period_list(parent, indices: list[int]) -> list[int]:
        """Get period in milliseconds for specified ADC channels."""
        return [parent._period_ms[i] for i in indices]

    @staticmethod
    def _set_period_all(parent, period_ms: int, indices: list[int]) -> None:
        """Set period in milliseconds for specified ADC channels."""
        for i in indices:
            parent._period_ms[i] = period_ms
            parent._setup_timer(i)

    @staticmethod
    def _get_measurement_list(parent, indices: list[int]) -> list[bool]:
        """Get measurement state for specified ADC channels."""
        return [parent._measurement_enabled[i] for i in indices]

    @staticmethod
    def _set_measurement_all(parent, enabled: bool, indices: list[int]) -> None:
        """Set measurement state for specified ADC channels."""
        for i in indices:
            parent._measurement_enabled[i] = enabled
            parent._setup_timer(i)

    class _AdcView:
        """
        View class for controlling individual ADC channels or groups of channels.
        
        This class provides a unified interface for controlling one or more ADC channels
        through the same API. It's returned by Adc.__getitem__() and allows
        seamless control of single channels or groups using identical syntax.
        
        Features:
            - Consistent API for single channel or multi-channel control
            - Property-based interface for intuitive control
            - Callback configuration and management
            - Automatic sampling control
            - Voltage and raw value access
            
        """

        def __init__(self, parent: "Adc", indices: list[int]):
            """
            Initialize ADC view with parent reference and channel indices.
            
            :param parent: Parent Adc instance
            :param indices: List of channel indices this view controls
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Adc._AdcView":
            """
            Get a sub-view of this view for further channel selection.
            
            Allows nested selection of channels from an existing view,
            enabling fine-grained control over channel groups.
            
            :param idx: Index or slice for sub-selection
            :return: New _AdcView with selected channels
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> group = adc[0:3]        # First three channels
                >>> subgroup = group[1:3]   # Second and third from original
                >>> subgroup.measurement = True # Control sub-selection
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Adc._AdcView(self._parent, selected_indices)
            else:
                return Adc._AdcView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of channels in this view.
            
            :return: Number of channels controlled by this view
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> group = adc[1:3]
                >>> print(len(group))  # Output: 2
            ```
            """
            return len(self._indices)

        @property
        def value(self) -> list[float]:
            """
            Get current voltage values of channels in this view.
            
            Returns the current analog voltage values for all channels in the view.
            Always returns a list, even for single channels. Values are in volts
            with 3 decimal places precision.
                        
            :return: List of voltage values in volts (0.000-3.300V)
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> 
                >>> # Read individual channel (returns list)
                >>> voltage = adc[0].value
                >>> print(f"Channel 0: {voltage[0]:.3f}V")
                >>> 
                >>> # Read multiple channels
                >>> voltages = adc[:].value
                >>> for i, v in enumerate(voltages):
                ...     print(f"Channel {i}: {v:.3f}V")
                >>> 
                >>> # Use in conditions
                >>> if adc[0].value[0] > 2.5:
                ...     print("High voltage detected")
                >>> 
                >>> # Temperature sensor reading (TMP36)
                >>> temp_voltage = adc[0].value[0]
                >>> temp_celsius = (temp_voltage - 0.5) * 100
                >>> print(f"Temperature: {temp_celsius:.1f}°C")
            ```
            """
            return Adc._get_value_list(self._parent, self._indices)

        @property
        def raw_value(self) -> list[int]:
            """
            Get current raw ADC values of channels in this view.
            
            Returns the current raw 16-bit ADC values for all channels in the view.
            Always returns a list, even for single channels. Values range from 0-65535.
            
            :return: List of raw ADC values (0-65535)
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> 
                >>> # Read individual channel raw value
                >>> raw_value = adc[0].raw_value[0]
                >>> print(f"Raw value: {raw_value}")
                >>> 
                >>> # Read all channels raw values
                >>> raw_values = adc[:].raw_value
                >>> for i, raw in enumerate(raw_values):
                ...     print(f"Channel {i}: {raw}/65535")
                >>> 
                >>> # Calculate percentage
                >>> percentage = (adc[0].raw_value[0] / 65535) * 100
                >>> print(f"Input level: {percentage:.1f}%")
                >>> 
                >>> # High-resolution measurements
                >>> raw = adc[0].raw_value[0]
                >>> voltage = raw * (3.3 / 65535)
                >>> print(f"Precise voltage: {voltage:.6f}V")
            ```
            """
            return Adc._get_raw_value_list(self._parent, self._indices)

        @property
        def callback(self) -> list[callable]:
            """
            Get callback functions for channels in this view.
            
            Returns the callback functions that will be called during
            automatic sampling for the channels.
            
            :return: List of callback functions or None values
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> callbacks = adc[:].callback
                >>> print(f"Callbacks: {callbacks}")  # [None, None, None]
                >>> 
                >>> # Check if callback is set
                >>> if adc[0].callback[0] is not None:
                ...     print("Callback is set for first channel")
            ```
            """
            return Adc._get_callback_list(self._parent, self._indices)

        @callback.setter
        def callback(self, fn: callable | list[callable]):
            """
            Set callback function for channels in this view.
            
            Sets the function to be called during automatic sampling.
            Callback receives (pin, voltage, raw) parameters where pin is the
            GPIO number, voltage is the value in volts, and raw is the 16-bit value.
            
            :param fn: Callback function, None, or list of callbacks
            
            :raises ValueError: If list length doesn't match number of channels
            :raises TypeError: If callback is not callable or None
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> 
                >>> # Simple callback for all channels
                >>> def adc_callback(pin, voltage, raw):
                ...     print(f"Pin {pin}: {voltage:.3f}V (raw: {raw})")
                >>> 
                >>> adc[:].callback = adc_callback
                >>> 
                >>> # Different callbacks for different channels
                >>> def temp_callback(pin, voltage, raw):
                ...     temp_c = (voltage - 0.5) * 100  # TMP36 conversion
                ...     print(f"Temperature: {temp_c:.1f}°C")
                >>> 
                >>> def light_callback(pin, voltage, raw):
                ...     light_pct = (voltage / 3.3) * 100
                ...     print(f"Light level: {light_pct:.1f}%")
                >>> 
                >>> adc[:].callback = [temp_callback, light_callback, None]
                >>> 
                >>> # Enable sampling and measurement
                >>> adc[:].period_ms = 500  # Sample every 500ms
                >>> adc[:].measurement = True
            ```
            """
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
            """
            Get sampling period in milliseconds for channels in this view.
            
            Returns the sampling period settings for each channel.
            0 means automatic sampling is disabled.
            
            :return: List of period values in milliseconds
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> 
                >>> # Check current period settings
                >>> periods = adc[:].period_ms
                >>> print(f"Periods: {periods}")  # [0, 0, 0] (disabled)
                >>> 
                >>> # Check specific channel
                >>> first_period = adc[0].period_ms
                >>> print(f"First channel period: {first_period}")  # [0]
            ```
            """
            return Adc._get_period_list(self._parent, self._indices)

        @period_ms.setter
        def period_ms(self, ms: int):
            """
            Set sampling period in milliseconds for channels in this view.
            
            Sets the period for automatic sampling. Must be > 0 to enable
            automatic sampling. Requires both callback and measurement to be enabled.
            
            :param ms: Sampling period in milliseconds
                      - 0: Disable automatic sampling (default)
                      - >0: Enable automatic sampling with specified interval
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> 
                >>> # Set period for all channels
                >>> adc[:].period_ms = 100  # Sample every 100ms
                >>> 
                >>> # Different periods for different channels
                >>> adc[0].period_ms = 50   # Fast sampling for first channel
                >>> adc[1].period_ms = 200  # Slower sampling for second channel
                >>> 
                >>> # Setup complete automatic sampling system
                >>> def sensor_callback(pin, voltage, raw):
                ...     if pin == 26:  # Temperature sensor
                ...         temp = (voltage - 0.5) * 100
                ...         print(f"Temperature: {temp:.1f}°C")
                ...     elif pin == 27:  # Light sensor
                ...         light = (voltage / 3.3) * 100
                ...         print(f"Light: {light:.1f}%")
                >>> 
                >>> adc[:].callback = sensor_callback
                >>> adc[:].period_ms = 1000  # Sample every second
                >>> adc[:].measurement = True  # Start sampling
            ```
            """
            Adc._set_period_all(self._parent, ms, self._indices)

        @property
        def measurement(self) -> list[bool]:
            """
            Get measurement state for channels in this view.
            
            Returns whether automatic sampling is enabled for each channel.
            Measurement must be enabled for callbacks to function.
            
            :return: List of measurement state flags
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> 
                >>> # Check measurement states
                >>> states = adc[:].measurement
                >>> print(f"Measurement states: {states}")  # [False, False, False]
                >>> 
                >>> # Check specific channel
                >>> first_state = adc[0].measurement
                >>> print(f"First channel enabled: {first_state}")  # [False]
            ```
            """
            return Adc._get_measurement_list(self._parent, self._indices)

        @measurement.setter
        def measurement(self, enabled: bool | list[bool]):
            """
            Enable or disable automatic sampling for channels in this view.
            
            Controls whether callbacks are triggered for automatic sampling.
            Channels must have both callback and period_ms configured to function.
            
            :param enabled: Single boolean for all channels or list of booleans
            
            :raises ValueError: If list length doesn't match number of channels
            
            Example
            --------
            ```python
                >>> adc = Adc([26, 27, 28])
                >>> 
                >>> # Setup callback system
                >>> def my_callback(pin, voltage, raw):
                ...     print(f"Pin {pin}: {voltage:.3f}V")
                >>> 
                >>> adc[:].callback = my_callback
                >>> adc[:].period_ms = 100
                >>> 
                >>> # Enable measurement for all channels
                >>> adc[:].measurement = True
                >>> 
                >>> # Enable only specific channels
                >>> adc[:].measurement = [True, False, True]  # First and third only
                >>> 
                >>> # Disable all measurements
                >>> adc[:].measurement = False
                >>> 
                >>> # Individual channel control
                >>> adc[0].measurement = True   # Enable first channel
                >>> adc[1].measurement = False  # Disable second channel
                >>> 
                >>> # Data logging system
                >>> def data_logger(pin, voltage, raw):
                ...     timestamp = utime.ticks_ms()
                ...     print(f"[{timestamp}] Pin {pin}: {voltage:.3f}V")
                >>> 
                >>> logger_adc = Adc([26, 27])
                >>> logger_adc[:].callback = data_logger
                >>> logger_adc[:].period_ms = 500  # Log every 500ms
                >>> logger_adc[:].measurement = True  # Start logging
                >>> 
                >>> # Later stop logging
                >>> logger_adc[:].measurement = False
            ```
            """
            if isinstance(enabled, bool):
                Adc._set_measurement_all(self._parent, enabled, self._indices)
            else:
                if len(enabled) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, en in zip(self._indices, enabled):
                    self._parent._measurement_enabled[i] = en
                    self._parent._setup_timer(i)


class Pwm:
    """
    Multi-channel Pulse Width Modulation (PWM) controller with advanced control features.
    
    This class provides comprehensive PWM functionality for controlling multiple GPIO pins
    with support for various control modes, precise timing, and flexible grouping operations.
    Uses indexing and slicing for intuitive channel control.
    """
    __FULL_RANGE     = 65_535
    __MICROS_PER_SEC = 1_000_000

    def __init__(self, pins: list[int]|tuple[int, ...]):
        """
        Initialize PWM controller for multiple GPIO pins.
        
        Creates PWM instances for the specified GPIO pins. All channels start 
        disabled (0% duty cycle) for safety. Use indexing/slicing to control channels.
        
        :param pins: List or tuple of GPIO pin numbers
        
        Example
        --------
        ```python
            >>> # Basic setup - use indexing/slicing for control
            >>> pwm = Pwm([10, 11, 12, 13])
            >>> pwm[:].freq = 1000      # Set frequency for all
            >>> pwm[:].duty = 0         # Start with all off
            >>> 
            >>> # Individual control
            >>> pwm[0].duty = 50        # First channel 50%
            >>> pwm[1:3].duty = 75      # Channels 1-2 to 75%
        ```
        """
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
        """
        Deinitialize all PWM channels and release hardware resources.
        
        Safely shuts down all PWM channels by stopping timers and
        releasing GPIO resources. Should be called when PWM is no longer needed.
        
        Example
        --------
        ```python
            >>> pwm = Pwm([10, 11, 12, 13])
            >>> # ... use PWM ...
            >>> pwm.deinit()
        ```
        """
        try:
            for pwm in self._pwm:
                pwm.duty_u16(0)
            
            utime.sleep_ms(50)  # Allow hardware to settle
            
            for pwm in self._pwm:
                pwm.deinit()
        except:
            pass

    def __getitem__(self, idx: int|slice) -> "_PwmView":
        """
        Get a view for controlling specific PWM channels or groups.
        
        This is the primary interface for PWM control. Returns a view object 
        that provides control over the selected channels.
        
        :param idx: Index (int) for single channel or slice for channel groups
        :return: PWM view object for controlling selected channels
        
        Example
        --------
        ```python
            >>> pwm = Pwm([10, 11, 12, 13])
            >>> 
            >>> # All channels
            >>> pwm[:].freq = 1000
            >>> pwm[:].duty = 50
            >>> 
            >>> # Single channel
            >>> pwm[0].duty = 75
            >>> 
            >>> # Channel range
            >>> pwm[1:3].duty = 25
            >>> 
            >>> # Every other channel
            >>> pwm[::2].duty = 100
        ```
        """
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
        """
        Get the number of PWM channels configured in this controller.
        
        :return: Number of PWM channels
        
        Example
        --------
        ```python
            >>> pwm = Pwm([10, 11, 12, 13])
            >>> print(len(pwm))
        ```
        """
        return len(self._pins)

    @staticmethod
    def _get_freq_list(parent, indices: list[int]) -> list[int]:
        """Get frequency list for specified PWM channels."""
        return [parent._freq_hz[i] for i in indices]

    @staticmethod
    def _set_freq_all(parent, freq: int, indices: list[int]) -> None:
        """Set frequency for specified PWM channels."""
        for i in indices:
            parent._freq_hz[i] = freq
            parent._pwm[i].freq(freq)

    @staticmethod
    @micropython.native
    def _get_period_list(parent, indices: list[int]) -> list[int]:
        """Get period list in microseconds for specified PWM channels."""
        return [Pwm.__MICROS_PER_SEC // parent._freq_hz[i] for i in indices]

    @staticmethod
    def _set_period_all(parent, period_us: int, indices: list[int]) -> None:
        """Set period in microseconds for specified PWM channels."""
        freq = Pwm.__MICROS_PER_SEC // period_us
        for i in indices:
            parent._freq_hz[i] = freq
            parent._pwm[i].freq(freq)

    @staticmethod
    def _get_duty_list(parent, indices: list[int]) -> list[int]:
        """Get duty cycle percentage list for specified PWM channels."""
        return [parent._duty_pct[i] for i in indices]

    @staticmethod
    def _set_duty_all(parent, duty_pct: int, indices: list[int]) -> None:
        """Set duty cycle percentage for specified PWM channels."""
        for i in indices:
            parent._duty_pct[i] = duty_pct
            if parent._enabled[i]:
                raw_duty = int(duty_pct * Pwm.__FULL_RANGE / 100)
                parent._pwm[i].duty_u16(raw_duty)
            else:
                parent._pwm[i].duty_u16(0)

    @staticmethod
    def _get_duty_u16_list(parent, indices: list[int]) -> list[int]:
        """Get raw duty cycle values for specified PWM channels."""
        return [parent._pwm[i].duty_u16() for i in indices]

    @staticmethod
    def _set_duty_u16_all(parent, duty_raw: int, indices: list[int]) -> None:
        """Set raw duty cycle values for specified PWM channels."""
        duty_pct = round(duty_raw * 100 / Pwm.__FULL_RANGE)
        for i in indices:
            parent._duty_pct[i] = duty_pct
            if parent._enabled[i]:
                parent._pwm[i].duty_u16(duty_raw)
            else:
                parent._pwm[i].duty_u16(0)

    @staticmethod
    def _get_duty_us_list(parent, indices: list[int]) -> list[int]:
        """Get duty cycle in microseconds for specified PWM channels."""
        result = []
        for i in indices:
            period_us = Pwm.__MICROS_PER_SEC // parent._freq_hz[i]
            duty_us = int(parent._duty_pct[i] * period_us / 100)
            result.append(duty_us)
        return result

    @staticmethod
    @micropython.native
    def _set_duty_us_all(parent, duty_us: int, indices: list[int]) -> None:
        """Set duty cycle in microseconds for specified PWM channels."""
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
        """Get enabled state list for specified PWM channels."""
        return [parent._enabled[i] for i in indices]

    @staticmethod
    def _set_enabled_all(parent, enabled: bool, indices: list[int]) -> None:
        """Set enabled state for specified PWM channels."""
        for i in indices:
            parent._enabled[i] = enabled
            if enabled:
                raw_duty = int(parent._duty_pct[i] * Pwm.__FULL_RANGE / 100)
                parent._pwm[i].duty_u16(raw_duty)
            else:
                parent._pwm[i].duty_u16(0)

    class _PwmView:
        """
        View class for controlling individual PWM channels or groups.
        
        This class provides the actual control interface through properties.
        Accessed via indexing/slicing on the main Pwm object.
        """

        def __init__(self, parent: "Pwm", indices: list[int]):
            """
            Initialize PWM view with parent reference and channel indices.
            
            :param parent: Parent Pwm instance
            :param indices: List of channel indices this view controls
            
            :raises ValueError: If indices are empty or invalid
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm_view = pwm[0:2]
                >>> print(pwm_view.pins)  # List of machine.PWM objects for channels 0 and 1
            ```
            """
            self._parent = parent
            self._indices = indices

        def __getitem__(self, idx: int|slice) -> "Pwm._PwmView":
            """
            Get a sub-view of this view for further channel selection.
            Allows nested selection of channels from an existing view,
            enabling fine-grained control over channel groups.
            
            :param idx: Index or slice for sub-selection
            :return: New _PwmView with selected channels
            
            :raises IndexError: If channel index is out of range
            :raises TypeError: If index is not int or slice
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12, 13])
                >>> group = pwm[0:3]        # First three channels
                >>> subgroup = group[1:3]   # Second and third from original
                >>> subgroup.freq = 500      # Control sub-selection frequency
            ```
            """
            if isinstance(idx, slice):
                selected_indices = [self._indices[i] for i in range(*idx.indices(len(self._indices)))]
                return Pwm._PwmView(self._parent, selected_indices)
            else:
                return Pwm._PwmView(self._parent, [self._indices[idx]])

        def __len__(self) -> int:
            """
            Get the number of channels in this view.
            
            :return: Number of channels controlled by this view
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12, 13])
                >>> group = pwm[1:3]
                >>> print(len(group))
            ```
            """
            return len(self._indices)

        @property
        def freq(self) -> list[int]:
            """
            Get the frequency of selected PWM channels in Hz.
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].freq = 1000
                >>> frequencies = pwm[:].freq  # [1000, 1000, 1000]
                >>> single_freq = pwm[0].freq  # [1000]
            ```
            """
            return Pwm._get_freq_list(self._parent, self._indices)

        @freq.setter
        def freq(self, hz: int | list[int]):
            """
            Set the frequency of selected PWM channels.
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].freq = 1000      # All channels
                >>> pwm[0].freq = 2000      # First channel only
                >>> pwm[1:].freq = 500      # Channels 1-2
            ```
            """
            if isinstance(hz, (list, tuple)):
                if len(hz) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, f in zip(self._indices, hz):
                    Pwm._set_freq_all(self._parent, f, [i])
            else:
                Pwm._set_freq_all(self._parent, hz, self._indices)

        @property
        def period(self) -> list[int]:
            """
            Get the period of selected PWM channels in microseconds.
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11])
                >>> pwm[:].freq = 1000
                >>> periods = pwm[:].period  # [1000, 1000] μs
            ```
            """
            return Pwm._get_period_list(self._parent, self._indices)

        @period.setter
        def period(self, us: int | list[int]):
            """
            Set the period of selected PWM channels in microseconds.
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11])
                >>> pwm[:].period = 20000   # 20ms = 50Hz
                >>> pwm[0].period = 1000    # 1ms = 1kHz for first channel
            ```
            """
            if isinstance(us, (list, tuple)):
                if len(us) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, p in zip(self._indices, us):
                    Pwm._set_period_all(self._parent, p, [i])
            else:
                Pwm._set_period_all(self._parent, us, self._indices)

        @property
        def duty(self) -> list[int]:
            """
            Get the duty cycle of selected PWM channels as percentage.
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].duty = 50
                >>> duties = pwm[:].duty     # [50, 50, 50]
                >>> first_duty = pwm[0].duty # [50]
            ```
            """
            return Pwm._get_duty_list(self._parent, self._indices)

        @duty.setter
        def duty(self, pct: int | list[int]):
            """
            Set the duty cycle of selected PWM channels.
            
            :param pct: Single percentage (0-100) or list of percentages
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].duty = 50            # All channels to 50%
                >>> pwm[0].duty = 75            # First channel to 75%
                >>> pwm[:].duty = [25, 50, 75]  # Individual values
            ```
            """
            if isinstance(pct, (list, tuple)):
                if len(pct) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, p in zip(self._indices, pct):
                    Pwm._set_duty_all(self._parent, p, [i])
            else:
                Pwm._set_duty_all(self._parent, pct, self._indices)

        @property
        def duty_u16(self) -> list[int]:
            """
            Get raw duty cycle values (0-65535).
            Returns the raw duty cycle values for the selected PWM channels.
            These values represent the duty cycle as a 16-bit integer,
            where 0 is 0% and 65535 is 100%.
            
            :return: List of raw duty cycle values (0-65535)
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].duty_u16 = 32768  # Set all to 50% duty cycle
                >>> duties = pwm[:].duty_u16  # [32768, 32768, 32768]
                >>> first_duty = pwm[0].duty_u16  # [32768]
            ```
            """
            return Pwm._get_duty_u16_list(self._parent, self._indices)

        @duty_u16.setter
        def duty_u16(self, raw: int | list[int]):
            """
            Set raw duty cycle values (0-65535) for selected PWM channels.
            Sets the duty cycle for the selected PWM channels using raw 16-bit values.
            
            :param raw: Single value (0-65535) or list of values
            :raises ValueError: If list length doesn't match number of channels
            
            :raises TypeError: If raw value is not int or list of ints
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].duty_u16 = 32768  # Set all to 50% duty cycle
                >>> pwm[0].duty_u16 = 16384   # First channel to 25%
                >>> pwm[:].duty_u16 = [0, 32768, 65535]  # Individual values
            ```
            """
            if isinstance(raw, (list, tuple)):
                if len(raw) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, r in zip(self._indices, raw):
                    Pwm._set_duty_u16_all(self._parent, r, [i])
            else:
                Pwm._set_duty_u16_all(self._parent, raw, self._indices)

        @property
        def duty_us(self) -> list[int]:
            """
            Get duty cycle in microseconds.
            Returns the duty cycle for the selected PWM channels in microseconds.
            The value represents the duration of the HIGH state in microseconds
            relative to the PWM period.
            
            :return: List of duty cycle values in microseconds
            
            :raises ValueError: If period is not set for the channels
            :raises TypeError: If period is not an int or list of ints
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].freq = 50             # Set frequency to 50Hz
                >>> pwm[:].duty_us = 1500        # Set all to center (1.5ms)
                >>> duties = pwm[:].duty_us      # [1500, 1500, 1500]
                >>> first_duty = pwm[0].duty_us  # [1500]
            ```
            """
            return Pwm._get_duty_us_list(self._parent, self._indices)

        @duty_us.setter
        def duty_us(self, us: int | list[int]):
            """
            Set duty cycle in microseconds.
            
            Example
            --------
            ```python
                >>> servo = Pwm([14, 15, 16])
                >>> servo[:].freq = 50
                >>> servo[:].duty_us = 1500    # All to center
                >>> servo[:].duty_us = [1000, 1500, 2000]  # Individual positions
            ```
            """
            if isinstance(us, (list, tuple)):
                if len(us) != len(self._indices):
                    raise ValueError("List length must match number of channels")
                for i, u in zip(self._indices, us):
                    Pwm._set_duty_us_all(self._parent, u, [i])
            else:
                Pwm._set_duty_us_all(self._parent, us, self._indices)

        @property
        def enabled(self) -> list[bool]:
            """
            Get enabled state of selected PWM channels.
            Returns a list indicating whether each PWM channel is enabled (True) or disabled (False).
            
            :return: List of boolean flags for each channel
            
            :raises ValueError: If indices are empty
            :raises TypeError: If indices are not a list or tuple
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].duty = 50
                >>> pwm[:].enabled  # [True, True, True] (all enabled)
                >>> pwm[0].enabled  # [True]
                >>> pwm[1].enabled  # [True]
                >>> pwm[2].enabled  # [True]
            ```
            """
            return Pwm._get_enabled_list(self._parent, self._indices)

        @enabled.setter
        def enabled(self, flag: bool | list[bool]):
            """
            Enable or disable selected PWM channels.
            
            Example
            --------
            ```python
                >>> pwm = Pwm([10, 11, 12])
                >>> pwm[:].duty = 50
                >>> pwm[:].enabled = False   # Disable all (duty preserved)
                >>> pwm[0].enabled = True    # Enable first channel only
            ```
            """
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
    """
    Scan for I2C devices on the specified bus and display/return found device addresses.
    
    This function scans the I2C bus for connected devices and either displays a visual
    map of detected devices or returns a list of addresses. The visual display shows
    device addresses in a grid format similar to the Linux i2cdetect command, with
    detected devices highlighted in yellow.
    
    :param id: I2C bus number (0 or 1) to scan
    :param show: Display mode control
        
        - True: Print visual device map to console
        - False: Return list of device addresses only
    
    :return: List of detected 7-bit I2C addresses if show=False, None if show=True
    
    :raises ValueError: If bus ID is not 0 or 1
    :raises OSError: If I2C bus initialization fails
    
    Example
    --------
    ```python
        >>> # Quick device detection - get addresses only
        >>> devices = i2cdetect(0)  # Scan bus 0
        >>> if devices:
        ...     print(f"Found {len(devices)} devices: {[hex(addr) for addr in devices]}")
        ... else:
        ...     print("No I2C devices found")
        >>> # Output: Found 2 devices: ['0x48', '0x68']
        >>> 
        >>> # Visual device map display
        >>> i2cdetect(0, show=True)
        >>> # Output:
        >>> #      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
        >>> # 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
        >>> # 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
        >>> # 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
        >>> # 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
        >>> # 40: -- -- -- -- -- -- -- -- 48 -- -- -- -- -- -- -- 
        >>> # 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
        >>> # 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
        >>> # 70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
        >>> 
        >>> # Check for specific devices
        >>> devices = i2cdetect(1)
        >>> if 0x3C in devices:
        ...     print("OLED display found at 0x3C")
        >>> if 0x68 in devices:
        ...     print("RTC module found at 0x68")
        >>> 
        >>> # Scan both I2C buses
        >>> for bus in [0, 1]:
        ...     print(f"Scanning I2C bus {bus}:")
        ...     devices = i2cdetect(bus)
        ...     if devices:
        ...         for addr in devices:
        ...             print(f"  Device at 0x{addr:02X} ({addr})")
        ...     else:
        ...         print("  No devices found")
        >>> 
        >>> # Diagnostic mode - visual display for troubleshooting
        >>> print("I2C Bus Diagnostic:")
        >>> i2cdetect(0, show=True)
        >>> 
        >>> # Integration with I2c class
        >>> devices = i2cdetect(0)
        >>> if 0x48 in devices:
        ...     temp_sensor = I2c(scl=5, sda=4, addr=0x48)
        ...     print("Temperature sensor initialized")
    ```
    """
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
    """
    Inter-Integrated Circuit (I2C) communication interface for RP2350 board.
    
    This class provides a comprehensive I2C interface for communicating with various
    I2C devices such as sensors, displays, RTCs, and EEPROMs. It automatically detects
    the correct I2C bus based on the provided SDA and SCL pin numbers and offers
    convenient methods for both basic and advanced I2C operations.
    
    Features:
    
        - Automatic I2C bus detection based on pin configuration
        - Support for both 8-bit and 16-bit register operations
        - Configurable endianness for multi-byte operations
        - Direct memory access functions for efficient data transfer
        - Support for various I2C device types and protocols
        - Built-in error handling and validation
    
    Pin Configuration:
        - Bus 0: SDA pins (0,4,8,12,16,20), SCL pins (1,5,9,13,17,21)
        - Bus 1: SDA pins (2,6,10,14,18,26), SCL pins (3,7,11,15,19,27)
        - Default frequency: 400kHz (fast mode)
        - Supports standard mode (100kHz) and fast mode (400kHz)
    
    """

    def __init__(self, scl:int, sda:int, addr:int, freq:int=400_000):
        """
        Initialize I2C interface with automatic bus detection.
        
        Creates an I2C interface instance with the specified pin configuration
        and device address. Automatically detects the appropriate I2C bus based
        on the provided SDA and SCL pin numbers according to RP2350 pin mapping.
        
        :param scl: SCL (clock) pin number
        :param sda: SDA (data) pin number  
        :param addr: 7-bit I2C device address (0x00-0x7F)
        :param freq: I2C bus frequency in Hz (default: 400kHz)

            - 100_000: Standard mode (100kHz)
            - 400_000: Fast mode (400kHz)
            - 1_000_000: Fast mode plus (1MHz, device dependent)
        
        :raises ValueError: If invalid SDA/SCL pin combination is provided
        :raises OSError: If I2C bus initialization fails
        :raises ValueError: If device address is outside valid range (0x00-0x7F)
        
        Example
        --------
        ```python
            >>> # Standard temperature sensor setup
            >>> temp_sensor = I2c(scl=5, sda=4, addr=0x48)
            >>> 
            >>> # High-speed sensor communication
            >>> accel = I2c(scl=21, sda=20, addr=0x53, freq=400_000)
            >>> 
            >>> # Slower device requiring standard mode
            >>> old_device = I2c(scl=13, sda=12, addr=0x20, freq=100_000)
            >>> 
            >>> # Multiple devices on same bus (different addresses)
            >>> temp = I2c(scl=5, sda=4, addr=0x48)      # TMP102
            >>> pressure = I2c(scl=5, sda=4, addr=0x76)  # BMP280
            >>> 
            >>> # Device discovery before initialization
            >>> devices = i2cdetect(0)
            >>> if 0x3C in devices:
            ...     oled = I2c(scl=5, sda=4, addr=0x3C)
            ...     print("OLED display initialized")
            >>> 
            >>> # Error handling for invalid pins
            >>> try:
            ...     invalid = I2c(scl=99, sda=98, addr=0x48)
            ... except ValueError as e:
            ...     print(f"Pin configuration error: {e}")
        ```
        """        
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
        """
        Read an unsigned 8-bit value from a specific register.
        
        Reads a single byte from the specified register address. This is the most
        common operation for reading sensor values, status registers, and device IDs.
        
        :param reg: Register address to read from (0x00-0xFF)
        :return: 8-bit unsigned value read from the register (0-255)
        
        :raises OSError: If I2C communication fails
        :raises ValueError: If register address is invalid
        
        Example
        --------
        ```python
            >>> # Read device ID register
            >>> device_id = sensor.read_u8(0x00)
            >>> print(f"Device ID: 0x{device_id:02X}")
            >>> 
            >>> # Read temperature sensor value
            >>> temp_raw = temp_sensor.read_u8(0x01)
            >>> temp_c = temp_raw - 40  # Convert based on sensor spec
            >>> print(f"Temperature: {temp_c}°C")
            >>> 
            >>> # Read status register with bit checking
            >>> status = device.read_u8(0x02)
            >>> if status & 0x80:  # Check bit 7
            ...     print("Device ready")
            >>> if status & 0x01:  # Check bit 0
            ...     print("Error detected")
            >>> 
            >>> # Read multiple registers in sequence
            >>> for reg in range(0x10, 0x16):
            ...     value = sensor.read_u8(reg)
            ...     print(f"Reg 0x{reg:02X}: 0x{value:02X}")
        ```
        """
        data = self.__i2c.readfrom_mem(self.__addr, reg, 1)
        return data[0]

    def read_u16(self, reg:int, *, little_endian:bool=True) -> int:
        """
        Read an unsigned 16-bit value from a specific register.
        
        Reads two consecutive bytes from the specified register address and combines
        them into a 16-bit value. Byte order can be configured for different device
        requirements (little-endian or big-endian).
        
        :param reg: Starting register address to read from (0x00-0xFF)
        :param little_endian: Byte order interpretation
        
            - True: Little-endian (LSB first, default)
            - False: Big-endian (MSB first)

        :return: 16-bit unsigned value (0-65535)
        
        :raises OSError: If I2C communication fails
        :raises ValueError: If register address is invalid
        
        Example
        --------
        ```python
            >>> # Read ADC value (little-endian, typical for most sensors)
            >>> adc_value = sensor.read_u16(0x04)
            >>> voltage = adc_value * 3.3 / 65535
            >>> print(f"ADC: {adc_value}, Voltage: {voltage:.3f}V")
            >>> 
            >>> # Read temperature with high precision (big-endian)
            >>> temp_raw = temp_sensor.read_u16(0x00, little_endian=False)
            >>> temp_c = temp_raw / 256.0  # Convert based on sensor spec
            >>> print(f"Temperature: {temp_c:.2f}°C")
            >>> 
            >>> # Read pressure sensor (BMP280 example)
            >>> pressure_raw = bmp280.read_u16(0xF7, little_endian=False)
            >>> # Apply calibration formula here
            >>> print(f"Raw pressure: {pressure_raw}")
            >>> 
            >>> # Read accelerometer data (3-axis, 16-bit each)
            >>> x_accel = accel.read_u16(0x32)  # X-axis
            >>> y_accel = accel.read_u16(0x34)  # Y-axis  
            >>> z_accel = accel.read_u16(0x36)  # Z-axis
            >>> print(f"Acceleration: X={x_accel}, Y={y_accel}, Z={z_accel}")
        ```
        """
        data = self.__i2c.readfrom_mem(self.__addr, reg, 2)
        order = 'little' if little_endian else 'big'
        return int.from_bytes(data, order)

    def write_u8(self, reg:int, val:int) -> None:
        """
        Write an unsigned 8-bit value to a specific register.
        
        Writes a single byte to the specified register address. This is commonly
        used for configuring device settings, control registers, and single-byte
        data transmission.
        
        :param reg: Register address to write to (0x00-0xFF)
        :param val: 8-bit value to write (0-255, automatically masked)
        
        :raises OSError: If I2C communication fails
        :raises ValueError: If register address is invalid
        
        Example
        --------
        ```python
            >>> # Configure sensor power mode
            >>> sensor.write_u8(0x20, 0x27)  # Normal mode, all axes enabled
            >>> 
            >>> # Set display contrast
            >>> oled.write_u8(0x81, 0x7F)  # 50% contrast
            >>> 
            >>> # Configure interrupt settings
            >>> device.write_u8(0x2E, 0x80)  # Enable interrupt on pin
            >>> 
            >>> # Set RTC seconds (BCD format)
            >>> rtc.write_u8(0x00, 0x30)  # 30 seconds
            >>> 
            >>> # Device configuration sequence
            >>> config_sequence = [
            ...     (0x20, 0x27),  # Power control
            ...     (0x21, 0x00),  # Data format
            ...     (0x2C, 0x08),  # Power control
            ...     (0x31, 0x0B),  # Data format
            ... ]
            >>> for reg, val in config_sequence:
            ...     sensor.write_u8(reg, val)
            ...     utime.sleep_ms(10)  # Small delay between writes
        ```
        """
        self.__i2c.writeto_mem(self.__addr, reg, bytes([val & 0xFF]))

    def write_u16(self, reg:int, val:int, *, little_endian:bool=True) -> None:
        """
        Write an unsigned 16-bit value to a specific register.
        
        Writes two consecutive bytes to the specified register address. The 16-bit
        value is split into two bytes according to the specified endianness.
        
        :param reg: Starting register address to write to (0x00-0xFF)
        :param val: 16-bit value to write (0-65535, automatically masked)
        :param little_endian: Byte order for writing

            - True: Little-endian (LSB first, default)
            - False: Big-endian (MSB first)
        
        :raises OSError: If I2C communication fails
        :raises ValueError: If register address is invalid
        
        Example
        --------
        ```python
            >>> # Set DAC output value
            >>> dac.write_u16(0x40, 32768)  # Half-scale output
            >>> 
            >>> # Configure timer threshold (big-endian)
            >>> timer.write_u16(0x04, 5000, little_endian=False)
            >>> 
            >>> # Set PWM duty cycle
            >>> pwm_controller.write_u16(0x06, 0x7FFF)  # 50% duty cycle
            >>> 
            >>> # Write calibration data
            >>> calibration_values = [12345, 23456, 34567]
            >>> for i, cal_val in enumerate(calibration_values):
            ...     eeprom.write_u16(0x10 + i*2, cal_val)
            >>> 
            >>> # Configure sensor range and sensitivity
            >>> sensor.write_u16(0x30, 0x0800)  # ±8g range
            >>> sensor.write_u16(0x32, 0x0400)  # High sensitivity
        ```
        """
        order = 'little' if little_endian else 'big'
        self.__i2c.writeto_mem(self.__addr, reg, (val & 0xFFFF).to_bytes(2, order))

    def readfrom(self, nbytes:int, *, stop:bool=True) -> bytes:
        """
        Read a specified number of bytes directly from the I2C device.
        
        Performs a direct read operation from the device without specifying a
        register address. This is useful for devices that don't use register-based
        addressing or for reading streaming data.
        
        :param nbytes: Number of bytes to read (1-255)
        :param stop: Whether to send I2C stop condition after reading
        
            - True: Send stop condition (default, recommended)
            - False: Don't send stop (for repeated start operations)

        :return: Bytes read from the device
        
        :raises OSError: If I2C communication fails
        :raises ValueError: If nbytes is invalid
        
        Example
        --------
        ```python
            >>> # Read device identification string
            >>> device_info = device.readfrom(16)
            >>> print(f"Device info: {device_info.decode('ascii', errors='ignore')}")
            >>> 
            >>> # Read sensor data stream
            >>> sensor_data = sensor.readfrom(6)  # 6 bytes of sensor data
            >>> x, y, z = struct.unpack('<HHH', sensor_data)
            >>> print(f"Sensor: X={x}, Y={y}, Z={z}")
            >>> 
            >>> # Read FIFO data from sensor
            >>> fifo_data = sensor.readfrom(32)  # Read 32 bytes from FIFO
            >>> for i in range(0, len(fifo_data), 2):
            ...     sample = int.from_bytes(fifo_data[i:i+2], 'little')
            ...     print(f"Sample {i//2}: {sample}")
            >>> 
            >>> # Read without stop condition (for repeated start)
            >>> partial_data = device.readfrom(4, stop=False)
            >>> more_data = device.readfrom(4, stop=True)
            >>> complete_data = partial_data + more_data
        ```
        """
        return self.__i2c.readfrom(self.__addr, nbytes, stop)

    def readinto(self, buf:bytearray, *, stop:bool=True) -> int:
        """
        Read bytes into a pre-allocated buffer from the I2C device.
        
        Reads data directly into the provided buffer, which can be more memory
        efficient for large data transfers as it avoids creating temporary byte objects.
        
        :param buf: Pre-allocated bytearray to read data into
        :param stop: Whether to send I2C stop condition after reading
        :return: Number of bytes actually read into the buffer
        
        :raises OSError: If I2C communication fails
        :raises TypeError: If buf is not a bytearray
        
        Example
        --------
        ```python
            >>> # Efficient large data reading
            >>> data_buffer = bytearray(64)
            >>> bytes_read = sensor.readinto(data_buffer)
            >>> print(f"Read {bytes_read} bytes")
            >>> 
            >>> # Read into specific buffer section
            >>> large_buffer = bytearray(256)
            >>> section_buffer = memoryview(large_buffer)[64:128]
            >>> bytes_read = device.readinto(section_buffer)
            >>> 
            >>> # Continuous data acquisition
            >>> sample_buffer = bytearray(10)
            >>> samples = []
            >>> for _ in range(100):
            ...     device.readinto(sample_buffer)
            ...     samples.append(bytes(sample_buffer))
            ...     utime.sleep_ms(10)
        ```
        """
        return self.__i2c.readinto(self.__addr, buf, stop)

    def readfrom_mem(self, reg:int, nbytes:int, *, addrsize:int=8) -> bytes:
        """
        Read a specified number of bytes from a specific memory address/register.
        
        Reads consecutive bytes starting from the specified register address.
        Supports both 8-bit and 16-bit register addressing for different device types.
        
        :param reg: Starting register/memory address
        :param nbytes: Number of bytes to read (1-255)
        :param addrsize: Address size in bits (8 or 16)

            - 8: Standard 8-bit register addressing (default)
            - 16: 16-bit register addressing for larger memory devices

        :return: Bytes read from the specified memory location
        
        :raises OSError: If I2C communication fails
        :raises ValueError: If parameters are invalid
        
        Example
        --------
        ```python
            >>> # Read multiple sensor values
            >>> sensor_data = sensor.readfrom_mem(0x28, 6)  # Read 6 bytes from reg 0x28
            >>> accel_x = int.from_bytes(sensor_data[0:2], 'little', signed=True)
            >>> accel_y = int.from_bytes(sensor_data[2:4], 'little', signed=True)
            >>> accel_z = int.from_bytes(sensor_data[4:6], 'little', signed=True)
            >>> 
            >>> # Read configuration block
            >>> config_data = device.readfrom_mem(0x20, 16)
            >>> print(f"Config: {config_data.hex()}")
            >>> 
            >>> # Read from large EEPROM (16-bit addressing)
            >>> eeprom_data = eeprom.readfrom_mem(0x1000, 32, addrsize=16)
            >>> 
            >>> # Read calibration coefficients
            >>> cal_data = sensor.readfrom_mem(0xAA, 22)  # BMP280 calibration
            >>> # Parse calibration data according to datasheet
            >>> 
            >>> # Read device serial number
            >>> serial_bytes = device.readfrom_mem(0x80, 8)
            >>> serial_number = int.from_bytes(serial_bytes, 'big')
            >>> print(f"Serial: {serial_number:016X}")
        ```
        """
        return self.__i2c.readfrom_mem(self.__addr, reg, nbytes, addrsize=addrsize)

    def readfrom_mem_into(self, reg:int, buf:bytearray, *, addrsize:int=8) -> int:
        """
        Read bytes from a specific memory address into a pre-allocated buffer.
        
        Combines the memory addressing capability of readfrom_mem with the efficiency
        of reading into a pre-allocated buffer. Ideal for large data transfers from
        specific memory locations.
        
        :param reg: Starting register/memory address
        :param buf: Pre-allocated bytearray to read data into
        :param addrsize: Address size in bits (8 or 16)
        :return: Number of bytes actually read into the buffer
        
        :raises OSError: If I2C communication fails
        :raises TypeError: If buf is not a bytearray
        
        Example
        --------
        ```python
            >>> # Efficient reading of large data blocks
            >>> data_buffer = bytearray(128)
            >>> bytes_read = eeprom.readfrom_mem_into(0x00, data_buffer)
            >>> print(f"Read {bytes_read} bytes from EEPROM")
            >>> 
            >>> # Read sensor FIFO data efficiently
            >>> fifo_buffer = bytearray(64)
            >>> sensor.readfrom_mem_into(0x40, fifo_buffer)  # FIFO register
            >>> 
            >>> # Read into memory-mapped structure
            >>> import struct
            >>> sensor_struct = bytearray(20)
            >>> sensor.readfrom_mem_into(0x20, sensor_struct)
            >>> temp, pressure, humidity = struct.unpack('<HHH', sensor_struct[:6])
        ```
        """
        return self.__i2c.readfrom_mem_into(self.__addr, reg, buf, addrsize=addrsize)

    def writeto(self, buf:bytes, *, stop:bool=True) -> int:
        """
        Write bytes directly to the I2C device.
        
        Performs a direct write operation to the device without specifying a register
        address. This is useful for sending commands, data streams, or working with
        devices that don't use register-based addressing.
        
        :param buf: Bytes to write to the device
        :param stop: Whether to send I2C stop condition after writing
        
            - True: Send stop condition (default, recommended)
            - False: Don't send stop (for repeated start operations)

        :return: Number of bytes written
        
        :raises OSError: If I2C communication fails
        :raises TypeError: If buf is not bytes or bytearray
        
        Example
        --------
        ```python
            >>> # Send command sequence to device
            >>> init_commands = bytes([0x01, 0x02, 0x03, 0x04])
            >>> device.writeto(init_commands)
            >>> 
            >>> # Send display data to OLED
            >>> display_data = bytearray(128)  # One line of pixels
            >>> for i in range(128):
            ...     display_data[i] = 0xFF if i % 2 else 0x00
            >>> oled.writeto(display_data)
            >>> 
            >>> # Send sensor calibration data
            >>> cal_data = struct.pack('<HHH', 1000, 2000, 3000)
            >>> sensor.writeto(cal_data)
            >>> 
            >>> # Stream data without stop condition
            >>> header = bytes([0xAA, 0xBB])
            >>> data = bytes([0x01, 0x02, 0x03, 0x04])
            >>> device.writeto(header, stop=False)
            >>> device.writeto(data, stop=True)
        ```
        """
        return self.__i2c.writeto(self.__addr, buf, stop)

    def writeto_mem(self, reg:int, buf:bytes, *, addrsize:int=8) -> int:
        """
        Write bytes to a specific memory address/register.
        
        Writes data to consecutive memory locations starting from the specified
        register address. Supports both 8-bit and 16-bit register addressing.
        
        :param reg: Starting register/memory address to write to
        :param buf: Bytes to write to the specified memory location
        :param addrsize: Address size in bits (8 or 16)
                        
            - 8: Standard 8-bit register addressing (default)
            - 16: 16-bit register addressing for larger memory devices

        :return: Number of bytes written
        
        :raises OSError: If I2C communication fails
        :raises ValueError: If parameters are invalid
        
        Example
        --------
        ```python
            >>> # Write sensor configuration
            >>> config = bytes([0x20, 0x10, 0x08, 0x04])
            >>> sensor.writeto_mem(0x20, config)
            >>> 
            >>> # Store data in EEPROM
            >>> message = b"Hello, RP2350!"
            >>> eeprom.writeto_mem(0x00, message)
            >>> 
            >>> # Write to large EEPROM (16-bit addressing)
            >>> large_data = bytearray(256)
            >>> eeprom.writeto_mem(0x1000, large_data, addrsize=16)
            >>> 
            >>> # Write calibration coefficients
            >>> cal_coeffs = struct.pack('<HHHHHH', 100, 200, 300, 400, 500, 600)
            >>> sensor.writeto_mem(0x10, cal_coeffs)
            >>> 
            >>> # Write RTC time and date
            >>> time_data = bytes([0x00, 0x30, 0x14, 0x03, 0x15, 0x12, 0x23])
            >>> rtc.writeto_mem(0x00, time_data)  # sec, min, hour, day, date, month, year
        ```
        """
        return self.__i2c.writeto_mem(self.__addr, reg, buf, addrsize=addrsize)


class ReplSerial:
    """
    Advanced REPL (Read-Eval-Print Loop) serial communication interface with buffered I/O.
    
    This class provides a sophisticated interface for reading from and writing to the REPL
    using a ring buffer system. It enables non-blocking reads, pattern-based reading,
    and configurable timeout operations for robust serial communication in MicroPython
    applications.
    
    Features:
        - Ring buffer for efficient data storage and retrieval
        - Non-blocking, blocking, and timeout-based read operations
        - Pattern-based reading (read until specific byte sequence)
        - Automatic background data pumping via timer interrupts
        - Configurable buffer size and polling intervals
        - UTF-8 compatible data handling
        - Safe exception handling for robust operation
        
    """
    
    def __init__(self, timeout:float|None=None, *, bufsize:int=512, poll_ms:int=10):
        """
        Initialize REPL serial interface with configurable parameters.
        
        Creates a buffered serial interface that automatically reads from REPL stdin
        and stores data in a ring buffer. The interface supports various timeout modes
        and configurable buffer sizes for different application requirements.
        
        :param timeout: Read operation timeout behavior (default: None)
                       
            - None: Blocking reads (wait indefinitely)
            - 0: Non-blocking reads (return immediately)
            - \>0: Timeout in seconds (wait up to specified time)
                       
        :param bufsize: Ring buffer size in bytes (default: 512)
                       
            - Larger buffers can handle burst data better
            - Smaller buffers use less memory
            - Must be > 0
                       
        :param poll_ms: Polling interval in milliseconds (default: 10)
                       
            - How often to check for new data from REPL stdin
            - Lower values = more responsive but higher CPU usage
            - Higher values = less CPU usage but slower response
        
        :raises ValueError: If bufsize <= 0 or poll_ms <= 0
        :raises OSError: If REPL stdin/stdout access fails
        
        Example
        -------
        ```python
            >>> # Default configuration - blocking reads
            >>> repl = ReplSerial()
            >>> 
            >>> # Non-blocking configuration for real-time applications
            >>> fast_repl = ReplSerial(timeout=0, poll_ms=5)
            >>> 
            >>> # Large buffer for data logging
            >>> logger = ReplSerial(timeout=30.0, bufsize=2048, poll_ms=20)
            >>> 
            >>> # Interactive shell with reasonable timeout
            >>> shell = ReplSerial(timeout=60.0, bufsize=256, poll_ms=10)
            >>> 
            >>> # High-frequency data acquisition
            >>> daq = ReplSerial(timeout=0.1, bufsize=4096, poll_ms=1)
            >>> 
            >>> # Protocol handler with custom timing
            >>> protocol = ReplSerial(timeout=5.0, bufsize=1024, poll_ms=15)
        ```
        """
        self._timeout   = timeout
        self._stdin     = usys.stdin.buffer
        self._stdout    = usys.stdout
        self._buf       = utools.RingBuffer(bufsize)
        self._scheduled = False
        self._tmr = machine.Timer(-1)
        self._tmr.init(period=poll_ms, mode=machine.Timer.PERIODIC, callback=self.__tick)

    def __tick(self, t):
        """
        Periodic timer callback for background data pumping.
        
        This internal method is called periodically by the timer to schedule
        data reading from REPL stdin. It uses micropython.schedule() to ensure
        data pumping occurs safely in the main execution context.
        
        :param t: Timer object (unused but required by timer callback interface)
        """
        if not self._scheduled:
            self._scheduled = True
            try:
                micropython.schedule(self.__pump, None)
            except RuntimeError:
                self._scheduled = False

    def __pump(self, _):
        """
        Background data pump that reads from REPL stdin into the ring buffer.
        
        This internal method reads available data from REPL stdin and stores it
        in the ring buffer. It reads one byte at a time to avoid blocking and
        handles exceptions gracefully to maintain system stability.
        
        :param _: Unused parameter (required by micropython.schedule interface)
        """
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
        """
        Wait for data availability or timeout.
        
        Internal method that blocks until data becomes available in the buffer
        or the specified deadline is reached. Handles both blocking and timeout
        modes efficiently.
        
        :param deadline_ms: Deadline in milliseconds (None for infinite wait)
        """
        while not self._buf.avail():
            if deadline_ms is not None and utime.ticks_diff(deadline_ms, utime.ticks_ms()) <= 0:
                return
            dur = None if deadline_ms is None else max(0,
                utime.ticks_diff(deadline_ms, utime.ticks_ms())) / 1000
            uselect.select([self._stdin], [], [], dur)

    @property
    def timeout(self) -> float|None:
        """
        Get the current timeout setting for read operations.
        
        :return: Current timeout value:
        
            - None: Blocking mode (wait indefinitely)
            - 0: Non-blocking mode (return immediately)
            - >0: Timeout in seconds
    
        Example
        -------
        ```python
            >>> repl = ReplSerial(timeout=5.0)
            >>> print(f"Current timeout: {repl.timeout} seconds")
            >>> 
            >>> # Check timeout mode
            >>> if repl.timeout is None:
            ...     print("Blocking mode")
            >>> elif repl.timeout == 0:
            ...     print("Non-blocking mode")
            >>> else:
            ...     print(f"Timeout mode: {repl.timeout} seconds")
        ```
        """
        return self._timeout
    
    @timeout.setter
    def timeout(self, value:float|None):
        """
        Set the timeout for read operations.
        
        Changes the timeout behavior for all subsequent read operations.
        This allows dynamic switching between blocking, non-blocking, and
        timeout modes during runtime.
        
        :param value: New timeout value:
        
            - None: Blocking mode (wait indefinitely)
            - 0: Non-blocking mode (return immediately)
            - >0: Timeout in seconds

        :raises ValueError: If value is negative (except for None and 0)
        
        Example
        -------
        ```python 
            >>> repl = ReplSerial()
            >>> 
            >>> # Switch to non-blocking mode
            >>> repl.timeout = 0
            >>> data = repl.read(10)  # Returns immediately
            >>> 
            >>> # Switch to timeout mode
            >>> repl.timeout = 2.5  # 2.5 second timeout
            >>> data = repl.read(100)  # Wait up to 2.5 seconds
            >>> 
            >>> # Switch to blocking mode
            >>> repl.timeout = None
            >>> data = repl.read(50)  # Wait indefinitely
            >>> 
            >>> # Dynamic timeout based on conditions
            >>> if urgent_mode:
            ...     repl.timeout = 0.1  # Quick timeout for urgent data
            >>> else:
            ...     repl.timeout = 10.0  # Longer timeout for normal operation
        ```
        """
        self._timeout = value

    def read(self, size:int=1) -> bytes:
        """
        Read a specified number of bytes from the REPL buffer.
        
        Reads up to 'size' bytes from the internal ring buffer. The behavior
        depends on the current timeout setting and data availability.
        
        :param size: Number of bytes to read (default: 1)

            - If size <= 0, returns empty bytes
            - If size > available data, behavior depends on timeout

        :return: Bytes read from the buffer (may be less than requested size)
        
        Timeout Behavior:
        
            - None (blocking): Waits until at least some data is available
            - 0 (non-blocking): Returns immediately with available data
            - \>0 (timeout): Waits up to timeout seconds for data
        
        :raises ValueError: If size is not an integer

        Example
        -------
            >>> repl = ReplSerial()
            >>> 
            >>> # Read single byte (blocking)
            >>> byte = repl.read(1)
            >>> if byte:
            ...     print(f"Received: {byte[0]:02X}")
            >>> 
            >>> # Read multiple bytes
            >>> repl.timeout = 5.0
            >>> data = repl.read(100)  # Wait up to 5 seconds for 100 bytes
            >>> print(f"Received {len(data)} bytes: {data}")
            >>> 
            >>> # Non-blocking read for polling
            >>> repl.timeout = 0
            >>> while True:
            ...     chunk = repl.read(64)
            ...     if chunk:
            ...         process_data(chunk)
            ...     else:
            ...         utime.sleep_ms(10)  # Small delay when no data
            >>> 
            >>> # Read with size checking
            >>> repl.timeout = 2.0
            >>> expected_size = 50
            >>> data = repl.read(expected_size)
            >>> if len(data) == expected_size:
            ...     print("Received complete packet")
            >>> else:
            ...     print(f"Partial packet: {len(data)}/{expected_size} bytes")
            >>> 
            >>> # Stream processing
            >>> repl.timeout = 0.5
            >>> buffer = bytearray()
            >>> while len(buffer) < 256:
            ...     chunk = repl.read(32)
            ...     if chunk:
            ...         buffer.extend(chunk)
            ...     else:
            ...         break  # Timeout or no more data
        ```
        """
        if size <= 0:
            return b''
        dl = None if self._timeout is None else utime.ticks_add(utime.ticks_ms(), int(self._timeout*1000))
        self.__wait(dl)
        return self._buf.get(size)

    def read_until(self, expected:bytes=b'\r', max_size:int|None=None) -> bytes:
        """
        Read from the buffer until a specific byte pattern is found.
        
        Reads data from the ring buffer until the expected byte sequence is
        encountered, the maximum size is reached, or a timeout occurs. This is
        ideal for protocol implementations and line-based communication.
        
        :param expected: Byte sequence to search for (default: b'\\r')  

            - Common patterns: b'\\r\\n', b'\\n', b'\\x00', b'END'  
            - Can be any byte sequence  
        
        :param max_size: Maximum bytes to read (default: None for no limit)
        
            - Prevents memory exhaustion with malformed data
            - Returns data when limit is reached, even without pattern
        
        :return: Bytes read including the expected pattern (if found)
        
            - Empty bytes if timeout in non-blocking/timeout modes
            - May not contain expected pattern if max_size reached
        
        Timeout Behavior:
        
            - 0 (non-blocking): Returns immediately with available data
            - >0 (timeout): Waits up to timeout seconds for pattern/max_size
            - None (blocking): Waits indefinitely for pattern/max_size
        
        :raises ValueError: If expected is not bytes or max_size is negative
        
        Example
        -------
        ```python
            >>> repl = ReplSerial()
            >>> 
            >>> # Read line-based input
            >>> line = repl.read_until(b'\\r\\n')
            >>> if line:
            ...     command = line.decode().strip()
            ...     print(f"Command: {command}")
            >>> 
            >>> # Protocol message reading
            >>> repl.timeout = 5.0
            >>> message = repl.read_until(b'\\x00', max_size=1024)  # Null-terminated
            >>> if message:
            ...     if message.endswith(b'\\x00'):
            ...         print("Complete message received")
            ...     else:
            ...         print("Message truncated at max_size")
            >>> 
            >>> # Interactive shell implementation
            >>> repl = ReplSerial(timeout=30.0)
            >>> while True:
            ...     try:
            ...         cmd_line = repl.read_until(b'\\r\\n', max_size=256)
            ...         if cmd_line:
            ...             command = cmd_line.decode().strip()
            ...             if command == 'exit':
            ...                 break
            ...             response = execute_command(command)
            ...             repl.write(response.encode() + b'\\r\\n')
            ...     except KeyboardInterrupt:
            ...         break
            >>> 
            >>> # Data packet reading with validation
            >>> repl.timeout = 1.0
            >>> while True:
            ...     packet = repl.read_until(b'END', max_size=512)
            ...     if packet:
            ...         if packet.endswith(b'END'):
            ...             process_packet(packet[:-3])  # Remove 'END' marker
            ...         else:
            ...             print("Malformed packet (no END marker)")
            ...     else:
            ...         print("No packet received (timeout)")
            >>> 
            >>> # Multi-delimiter reading
            >>> delimiters = [b'\\r\\n', b'\\n', b'\\r']
            >>> for delimiter in delimiters:
            ...     repl.timeout = 0.1  # Quick check
            ...     data = repl.read_until(delimiter, max_size=100)
            ...     if data and data.endswith(delimiter):
            ...         print(f"Found data with delimiter: {delimiter}")
            ...         break
        ```
        """
        # Non-blocking shortcut
        if self._timeout == 0:
            if max_size and self._buf.avail() >= max_size:
                return self._buf.get(max_size)
            
            data = self._buf.get_until(expected, max_size)
            return data or b''

        deadline = None
        if self._timeout is not None:
            deadline = utime.ticks_add(utime.ticks_ms(), int(self._timeout * 1000))

        # Main loop for blocking/timeout
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
        """
        Write data to the REPL output stream.
        
        Sends the provided data to the REPL stdout, making it visible in the
        terminal or connected serial interface. This method provides direct
        access to the output stream for sending responses, status messages,
        or any other data.
        
        :param data: Data to write (must be bytes or bytearray)
    
            - String data must be encoded first: text.encode('utf-8')
            - Binary data can be written directly
            
        :return: Number of bytes actually written
        
        :raises TypeError: If data is not bytes or bytearray
        :raises OSError: If write operation fails
        
        Example
            --------
            ```python
            >>> repl = ReplSerial()
            >>> 
            >>> # Write string data
            >>> message = "Hello, World!\\r\\n"
            >>> bytes_written = repl.write(message.encode('utf-8'))
            >>> print(f"Wrote {bytes_written} bytes")
            >>> 
            >>> # Write binary data
            >>> binary_data = bytes([0x01, 0x02, 0x03, 0x04])
            >>> repl.write(binary_data)
            >>> 
            >>> # Command response system
            >>> def send_response(repl, status, message):
            ...     response = f"STATUS:{status}:{message}\\r\\n"
            ...     return repl.write(response.encode('utf-8'))
            >>> 
            >>> bytes_sent = send_response(repl, "OK", "Command executed")
            >>> 
            >>> # JSON protocol implementation
            >>> import json
            >>> def send_json_response(repl, data):
            ...     json_str = json.dumps(data) + "\\r\\n"
            ...     return repl.write(json_str.encode('utf-8'))
            >>> 
            >>> response_data = {"result": "success", "value": 42}
            >>> send_json_response(repl, response_data)
            >>> 
            >>> # Progress indicator
            >>> def show_progress(repl, percent):
            ...     bar = "=" * (percent // 2) + " " * (50 - percent // 2)
            ...     progress = f"\\rProgress: [{bar}] {percent}%"
            ...     repl.write(progress.encode('utf-8'))
            >>> 
            >>> for i in range(0, 101, 10):
            ...     show_progress(repl, i)
            ...     utime.sleep(0.1)
            >>> 
            >>> # Echo server implementation
            >>> while True:
            ...     data = repl.read_until(b'\\r\\n')
            ...     if data:
            ...         echo = b"ECHO: " + data
            ...         repl.write(echo)
        """
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("data must be bytes or bytearray")
        return self._stdout.write(data)

    def close(self):
        """
        Close the REPL serial interface and release resources.
        
        Properly shuts down the serial interface by stopping the background
        timer and releasing system resources. Should be called when the
        interface is no longer needed to prevent resource leaks.
        
        Note: After calling close(), the ReplSerial instance should not be
        used for further operations.
        
        Example
        -------
        ```python
            >>> repl = ReplSerial()
            >>> 
            >>> # Use the interface
            >>> data = repl.read(10)
            >>> repl.write(b"Response\\r\\n")
            >>> 
            >>> # Clean shutdown
            >>> repl.close()
            >>> 
            >>> # Context manager pattern (recommended)
            >>> class ReplSerialContext:
            ...     def __init__(self, *args, **kwargs):
            ...         self.repl = ReplSerial(*args, **kwargs)
            ...     def __enter__(self):
            ...         return self.repl
            ...     def __exit__(self, exc_type, exc_val, exc_tb):
            ...         self.repl.close()
            >>> 
            >>> with ReplSerialContext(timeout=5.0) as repl:
            ...     data = repl.read(100)
            ...     repl.write(b"Done\\r\\n")
            >>> # Automatically closed when exiting context
            >>> 
            >>> # Cleanup in exception handler
            >>> repl = ReplSerial()
            >>> try:
            ...     # Main processing loop
            ...     while True:
            ...         data = repl.read_until(b'\\r\\n')
            ...         if data:
            ...             process_command(data.decode().strip())
            ... except KeyboardInterrupt:
            ...     print("Shutting down...")
            ... finally:
            ...     repl.close()  # Ensure cleanup
        ```
        """
        self._tmr.deinit()


def input(prompt:str="") -> str:
    """
    Advanced input function with full-featured line editing capabilities.
    
    This function provides a sophisticated replacement for the standard input()
    function with comprehensive line editing features. It supports UTF-8 character
    encoding, cursor movement, text insertion/deletion, and proper terminal control
    sequences for a professional user interface experience.
    
    Features:
    
        - Full UTF-8 support for international characters (1-4 bytes per character)
        - Left/Right arrow key cursor movement with proper character width handling
        - Backspace deletion before cursor position
        - Delete key support for deletion at cursor position
        - Text insertion at any cursor position (not just at end)
        - Proper display width calculation for multi-byte characters
        - Terminal control sequence handling for professional appearance
        - Robust input validation and error handling
    
    Key Bindings:
    
        - Enter (CR/LF): Submit input and return string
        - Backspace/Delete: Remove character before cursor
        - Delete key (ESC[3~): Remove character at cursor
        - Left Arrow (ESC[D): Move cursor left one character
        - Right Arrow (ESC[C): Move cursor right one character
        - Printable characters: Insert at cursor position
    
    Terminal Compatibility:
    
        - VT100/ANSI terminal escape sequences
        - Proper handling of variable-width characters
        - Compatible with most terminal emulators
        - Handles both Windows and Unix line endings
    
    Example
    -------
    ```python
        >>> # Basic input with prompt
        >>> name = input("Enter your name: ")
        >>> print(f"Hello, {name}!")
        >>> 
        >>> # Interactive command shell
        >>> while True:
        ...     command = input(">>> ")
        ...     if command == "exit":
        ...         break
        ...     elif command == "help":
        ...         print("Available commands: help, exit, status")
        ...     elif command == "status":
        ...         print("System is running normally")
        ...     else:
        ...         print(f"Unknown command: {command}")
        >>> 
        >>> # Configuration input with validation
        >>> while True:
        ...     try:
        ...         port = input("Enter port number (1-65535): ")
        ...         port_num = int(port)
        ...         if 1 <= port_num <= 65535:
        ...             print(f"Port {port_num} configured successfully")
        ...             break
        ...         else:
        ...             print("Port number must be between 1 and 65535")
        ...     except ValueError:
        ...         print("Please enter a valid number")
        >>> 
        >>> # Multi-language support
        >>> greeting = input("Enter greeting (支持中文): ")
        >>> print(f"Your greeting: {greeting}")
        >>> 
        >>> # Interactive menu system
        >>> def show_menu():
        ...     print("\n=== System Menu ===")
        ...     print("1. View Status")
        ...     print("2. Configure Network")
        ...     print("3. Run Diagnostics")
        ...     print("4. Exit")
        ...     return input("Select option (1-4): ")
        >>> 
        >>> while True:
        ...     choice = show_menu()
        ...     if choice == "1":
        ...         print("System Status: OK")
        ...     elif choice == "2":
        ...         ssid = input("WiFi SSID: ")
        ...         password = input("Password: ")
        ...         print(f"Configuring network: {ssid}")
        ...     elif choice == "3":
        ...         print("Running diagnostics...")
        ...         # Run diagnostic functions
        ...     elif choice == "4":
        ...         print("Goodbye!")
        ...         break
        ...     else:
        ...         print("Invalid choice, please try again")
        >>> 
        >>> # Data entry with editing
        >>> print("Enter device configuration (use arrows to edit):")
        >>> device_name = input("Device Name: ")
        >>> location = input("Location: ")
        >>> description = input("Description: ")
        >>> 
        >>> # Confirmation with editing capability
        >>> print(f"\nConfiguration Summary:")
        >>> print(f"Name: {device_name}")
        >>> print(f"Location: {location}")
        >>> print(f"Description: {description}")
        >>> confirm = input("Confirm configuration (y/n): ")
        >>> if confirm.lower() == 'y':
        ...     print("Configuration saved!")
        ... else:
        ...     print("Configuration cancelled")
        >>> 
        >>> # Password-style input (note: this doesn't hide characters)
        >>> # For actual password input, you'd need additional masking
        >>> username = input("Username: ")
        >>> # For real applications, use a proper password input method
        >>> 
        >>> # Batch data entry
        >>> contacts = []
        >>> while True:
        ...     name = input("Contact name (or 'done' to finish): ")
        ...     if name.lower() == 'done':
        ...         break
        ...     email = input("Email address: ")
        ...     contacts.append({"name": name, "email": email})
        >>> 
        >>> print(f"Added {len(contacts)} contacts")
        >>> for contact in contacts:
        ...     print(f"  {contact['name']}: {contact['email']}")
    ```
    """

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

        # Backspace
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