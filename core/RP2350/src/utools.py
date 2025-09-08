"""
Utility Tools Library

Essential utility functions and classes for MicroPython embedded development.
Provides commonly used mathematical functions, data structures, and protocol
implementations optimized for resource-constrained environments.

Features:

- Mathematical utility functions (clamp, map, xrange)
- Random number generation with specified byte size
- Interval timing and scheduling utilities
- ANSI escape code formatting for terminal output
- SLIP protocol encoder/decoder for serial communication
- Ring buffer implementation for efficient data streaming
- Memory-efficient implementations for embedded systems
- Cross-platform compatibility for MicroPython devices

Mathematical Functions:

- clamp: Constrain values within specified bounds
- map: Linear interpolation between value ranges  
- xrange: Floating-point range generator with precision control
- rand: Cryptographically secure random number generation

Terminal Formatting:

- ANSI escape codes for foreground/background colors
- Text formatting attributes (bold, underline, reverse)
- Cursor positioning and movement commands
- RGB color support for enhanced terminal output
- Screen and line clearing operations

Communication Protocols:

- SLIP encoding/decoding for reliable serial data transmission
- Frame boundary detection and error recovery
- Escape sequence handling for binary data integrity

Data Structures:

- Ring buffer with overflow protection
- Pattern matching and data extraction
- Efficient byte-level operations
- Configurable buffer sizes for memory optimization

"""

import uos
import utime
import micropython


__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"


@micropython.native
def clamp(val: float, lo: float, hi: float) -> float:
    """
    Constrain a value within the inclusive range [lo, hi].
    
    This function ensures that the input value falls within the specified bounds
    by returning the lower bound if the value is too small, the upper bound if
    the value is too large, or the value itself if it's within range.
    
    :param val: Value to be clamped (numeric type)
    :param lo: Lower bound (inclusive)
    :param hi: Upper bound (inclusive)
    :return: Clamped value within [lo, hi] range
    
    :raises TypeError: If arguments are not numeric
    :raises ValueError: If lo > hi
    
    Example
    -------
    ```python
        >>> # Basic clamping operations
        >>> clamp(15, 0, 10)     # Returns 10 (clamped to upper bound)
        >>> clamp(-5, 0, 10)     # Returns 0 (clamped to lower bound)
        >>> clamp(7, 0, 10)      # Returns 7 (within range, unchanged)
        >>> 
        >>> # Practical application: Limiting sensor values
        >>> adc_value = adc.read_u16()
        >>> normalized = clamp(adc_value, 1000, 64000)  # Clamp to valid range
        >>> percentage = (normalized - 1000) / (64000 - 1000) * 100
        >>> 
        >>> # Input validation
        >>> try:
        ...     clamp(5, 10, 5)  # Invalid: lower bound > upper bound
        >>> except ValueError as e:
        ...     print(f"Error: {e}")
    ```
    """
    if lo > hi:
        raise ValueError("Lower bound must be <= upper bound")
    return lo if val < lo else hi if val > hi else val

@micropython.native
def map(x: float, min_i: float, max_i: float, min_o: float, max_o: float) -> float:
    """
    Map a value from one range to another using linear interpolation.
    
    This function performs linear interpolation to map a value from an input range
    [min_i, max_i] to an output range [min_o, max_o]. It's commonly used for
    scaling sensor readings, converting between units, and normalizing data.
    
    :param x: Input value to be mapped
    :param min_i: Minimum value of input range
    :param max_i: Maximum value of input range
    :param min_o: Minimum value of output range
    :param max_o: Maximum value of output range
    :return: Mapped value in the output range
    
    :raises ZeroDivisionError: If input range is zero (min_i == max_i)
    :raises TypeError: If arguments are not numeric
    
    Example
    -------
    ```python
        >>> # Basic range mapping
        >>> map(50, 0, 100, 0, 255)  # Returns 127.5 (50% of 255)
        >>> 
        >>> # ADC to voltage conversion
        >>> adc_value = adc.read_u16()  # 0-65535 range
        >>> voltage = map(adc_value, 0, 65535, 0, 3.3)
        >>> print(f"Voltage: {voltage:.3f}V")
        >>> 
        >>> # Servo control - angle to pulse width
        >>> angle = 45  # degrees (-90 to +90)
        >>> pulse_width = map(angle, -90, 90, 1000, 2000)  # microseconds
        >>> servo.duty_us(int(pulse_width))
    ```
    """
    if max_i == min_i:
        raise ZeroDivisionError("Input range cannot be zero")
    return (x - min_i) * (max_o - min_o) / (max_i - min_i) + min_o

@micropython.native
def xrange(start: float, stop: float | None = None, step: float | None = None) -> iter[float]:
    """
    Create a generator that yields floating-point numbers in a specified range.
    
    This function is a floating-point equivalent of Python's range() function,
    allowing precise control over decimal increments. It uses string formatting
    to maintain precision and avoid floating-point arithmetic errors.
    
    :param start: Starting value of the range
    :param stop: Ending value of the range (exclusive). If None, start becomes stop and start becomes 0.0
    :param step: Step size for the range. If None, defaults to 1.0 or -1.0 based on direction
    :return: Generator yielding floating-point values
    
    :raises ValueError: If step is zero
    :raises TypeError: If arguments are not numeric
    
    Example
    -------
    ```python
        >>> # Basic floating-point range
        >>> list(xrange(0.0, 1.0, 0.2))
        [0.0, 0.2, 0.4, 0.6, 0.8]
        >>> 
        >>> # Single argument (stop only)
        >>> list(xrange(3.0))  # Equivalent to xrange(0.0, 3.0, 1.0)
        [0.0, 1.0, 2.0]
        >>> 
        >>> # Descending range
        >>> list(xrange(5.0, 0.0, -1.0))
        [5.0, 4.0, 3.0, 2.0, 1.0]
        >>> 
        >>> # Practical application: PWM sweep
        >>> for duty in xrange(0, 100, 5):  # 0% to 100% in 5% steps
        ...     pwm.duty(duty)
        ...     utime.sleep_ms(100)  # Smooth transition
    ```
    """
    if stop is None:
        stop, start = start, 0.0

    if step is None:
        step = 1.0 if stop >= start else -1.0

    if step == 0.0:
        raise ValueError("step must not be zero")

    if (stop - start) * step <= 0.0:
        return

    s_step = "{:.16f}".format(abs(step)).rstrip('0').rstrip('.')
    decimals = len(s_step.split('.')[1]) if '.' in s_step else 0

    idx = 0
    while True:
        value = start + idx * step
        if (step > 0 and value >= stop) or (step < 0 and value <= stop):
            break
        yield round(value, decimals)
        idx += 1


def rand(size: int = 4) -> int:
    """
    Generate a cryptographically secure random number of specified byte size.
    
    This function uses the system's random number generator to create secure
    random numbers suitable for cryptographic applications, session tokens,
    and other security-sensitive uses.
    
    :param size: The size of the random number in bytes (default: 4 bytes)
    
        - 1 byte: 0 to 255
        - 2 bytes: 0 to 65,535  
        - 4 bytes: 0 to 4,294,967,295
        - 8 bytes: 0 to 18,446,744,073,709,551,615
    
    :return: Random number as integer within the range for specified byte size
    
    :raises ValueError: If size <= 0 or size > 8
    :raises OSError: If system random generator is not available
    
    Example
    -------
    ```python
        >>> # Generate random numbers of different sizes
        >>> rand(1)  # 8-bit random number (0-255)
        >>> rand(4)  # 32-bit random number (default)
        >>> 
        >>> # Generate random identifier
        >>> device_id = rand(4)
        >>> print(f"Device ID: 0x{device_id:08X}")
        >>> 
        >>> # Randomized timing for anti-collision
        >>> jitter_ms = rand(1) % 100  # Random delay 0-99ms
        >>> utime.sleep_ms(base_delay + jitter_ms)
        >>> 
        >>> # Input validation
        >>> try:
        ...     rand(10)  # Invalid: size > 8
        >>> except ValueError as e:
        ...     print(f"Error: {e}")
    ```
    """
    if size <= 0 or size > 8:
        raise ValueError("Size must be between 1 and 8 bytes")
    
    return int.from_bytes(uos.urandom(size), "big")

@micropython.native
def hsv_to_rgb(h: float, s: float, v: float) -> tuple[int,int,int]:
    """
    Convert HSV (Hue, Saturation, Value) color values to RGB format.
    
    This function converts HSV color space to RGB color space, commonly used
    for color manipulation, LED control, and graphics applications. HSV is
    often more intuitive for color selection as it separates color (hue),
    intensity (saturation), and brightness (value).
    
    :param h: Hue value in degrees (0-360)
        - 0° = Red
        - 60° = Yellow  
        - 120° = Green
        - 180° = Cyan
        - 240° = Blue
        - 300° = Magenta
    :param s: Saturation value (0.0-1.0)
        - 0.0 = Grayscale (no color)
        - 1.0 = Full color saturation
    :param v: Value/brightness (0.0-1.0)
        - 0.0 = Black (no brightness)
        - 1.0 = Full brightness
    :return: RGB values as tuple of integers (red, green, blue) in range 0-255
    
    :raises TypeError: If parameters are not numeric
    
    Example
    -------
    ```python
        >>> # Primary colors
        >>> hsv_to_rgb(0, 1.0, 1.0)    # Red: (255, 0, 0)
        >>> hsv_to_rgb(120, 1.0, 1.0)  # Green: (0, 255, 0)
        >>> hsv_to_rgb(240, 1.0, 1.0)  # Blue: (0, 0, 255)
        >>> 
        >>> # Color variations
        >>> hsv_to_rgb(60, 1.0, 1.0)   # Yellow: (255, 255, 0)
        >>> hsv_to_rgb(180, 1.0, 1.0)  # Cyan: (0, 255, 255)
        >>> hsv_to_rgb(300, 1.0, 1.0)  # Magenta: (255, 0, 255)
        >>> 
        >>> # Brightness control (varying value)
        >>> hsv_to_rgb(0, 1.0, 0.5)    # Dim red: (127, 0, 0)
        >>> hsv_to_rgb(0, 1.0, 0.25)   # Dark red: (63, 0, 0)
        >>> 
        >>> # Saturation control (color intensity)
        >>> hsv_to_rgb(0, 0.5, 1.0)    # Pale red: (255, 127, 127)
        >>> hsv_to_rgb(0, 0.0, 1.0)    # White: (255, 255, 255)
        >>> 
        >>> # Rainbow effect for RGB LEDs
        >>> import neopixel
        >>> np = neopixel.NeoPixel(machine.Pin(18), 8)
        >>> 
        >>> def rainbow_cycle():
        ...     for i in range(8):  # 8 LEDs
        ...         hue = (i * 360) / 8  # Distribute hues across LEDs
        ...         r, g, b = hsv_to_rgb(hue, 1.0, 0.5)  # Full saturation, half brightness
        ...         np[i] = (r, g, b)
        ...     np.write()
        >>> 
        >>> # Color breathing effect
        >>> def breathing_effect(hue, duration_ms=2000):
        ...     steps = 50
        ...     for i in range(steps):
        ...         # Sine wave for smooth breathing
        ...         import math
        ...         brightness = (math.sin(2 * math.pi * i / steps) + 1) / 2
        ...         r, g, b = hsv_to_rgb(hue, 1.0, brightness)
        ...         set_led_color(r, g, b)
        ...         utime.sleep_ms(duration_ms // steps)
        >>> 
        >>> # Temperature-based color indication
        >>> def temp_to_color(temp_celsius):
        ...     # Map temperature to hue: blue (cold) to red (hot)
        ...     if temp_celsius < 0:
        ...         hue = 240  # Blue for very cold
        ...     elif temp_celsius > 40:
        ...         hue = 0    # Red for very hot
        ...     else:
        ...         hue = 240 - (temp_celsius / 40) * 240  # Blue to red gradient
        ...     
        ...     return hsv_to_rgb(hue, 1.0, 1.0)
        >>> 
        >>> # Status indication with color coding
        >>> STATUS_COLORS = {
        ...     'idle': hsv_to_rgb(0, 0, 0.3),      # Dim white
        ...     'working': hsv_to_rgb(60, 1.0, 1.0), # Yellow
        ...     'success': hsv_to_rgb(120, 1.0, 1.0), # Green
        ...     'error': hsv_to_rgb(0, 1.0, 1.0),     # Red
        ...     'warning': hsv_to_rgb(30, 1.0, 1.0)   # Orange
        ... }
    ```
    """
    i = int(h // 60) % 6
    f = (h / 60) - i
    p = int(v * (1 - s) * 255)
    q = int(v * (1 - f*s) * 255)
    t = int(v * (1 - (1-f)*s) * 255)
    v = int(v * 255)
    if   i == 0: return v, t, p
    elif i == 1: return q, v, p
    elif i == 2: return p, v, t
    elif i == 3: return p, q, v
    elif i == 4: return t, p, v
    else:        return v, p, q


def intervalChecker(interval_ms: int) -> callable:
    """
    Create a function that checks if a specified time interval has elapsed.
    
    This function returns a closure that tracks time and indicates when the
    specified interval has passed since the last positive check. It's useful
    for implementing periodic operations without blocking execution.
    
    :param interval_ms: The interval in milliseconds to check
    :return: Function that returns True when interval has elapsed, False otherwise
    
    :raises ValueError: If interval_ms <= 0
    :raises TypeError: If interval_ms is not an integer
    
    Example
    -------
    ```python
        >>> # Basic periodic operation without blocking
        >>> check_sensor = intervalChecker(1000)  # Check every 1 second
        >>> 
        >>> # Main loop with non-blocking timing
        >>> while True:
        ...     if check_sensor():
        ...         # This executes approximately every 1 second
        ...         sensor_value = read_sensor()
        ...         print(f"Sensor value: {sensor_value}")
        ...     
        ...     # Other operations continue without delay
        ...     process_input()
        ...     update_display()
        ...     utime.sleep_ms(10)
        >>> 
        >>> # Multiple independent timers
        >>> fast_check = intervalChecker(100)    # Fast operations (100ms)
        >>> slow_check = intervalChecker(5000)   # Slow operations (5s)
        >>> 
        >>> while True:
        ...     if fast_check():
        ...         update_leds()
        ...     
        ...     if slow_check():
        ...         log_data()
        ...     
        ...     # No blocking delays needed
        ...     utime.sleep_ms(10)
    ```
    """
    if not isinstance(interval_ms, int) or interval_ms <= 0:
        raise ValueError("Interval must be a positive integer")
    
    current_tick = utime.ticks_us()   
    
    def check_interval():
        nonlocal current_tick
        
        if utime.ticks_diff(utime.ticks_us(), current_tick) >= interval_ms * 1000:
            current_tick = utime.ticks_us()
            return True
        return False
    
    return check_interval


class ANSIEC:
    """
    ANSI Escape Code utilities for terminal text formatting and cursor control.
    
    This class provides comprehensive ANSI escape code support for creating
    colorful, formatted terminal output with cursor positioning capabilities.
    Compatible with most modern terminals and useful for creating professional
    command-line interfaces and debugging output.
    
    Features:
    
        - Foreground and background color control (standard and bright variants)
        - RGB color support for 24-bit color terminals
        - Text formatting attributes (bold, underline, reverse)
        - Cursor positioning and movement commands
        - Screen and line clearing operations
        - Cross-platform terminal compatibility
    
    """
    
    class FG:
        """
        ANSI escape codes for foreground (text) colors.
        
        This class provides both standard and bright color variants for text
        coloring. Standard colors work on all ANSI-compatible terminals,
        while bright colors provide enhanced visibility.
        """
        BLACK = "\u001b[30m"
        RED = "\u001b[31m"
        GREEN = "\u001b[32m"
        YELLOW = "\u001b[33m"
        BLUE = "\u001b[34m"
        MAGENTA = "\u001b[35m"
        CYAN = "\u001b[36m"
        WHITE = "\u001b[37m"
        BRIGHT_BLACK= "\u001b[30;1m"
        BRIGHT_RED = "\u001b[31;1m"
        BRIGHT_GREEN = "\u001b[32;1m"
        BRIGHT_YELLOW = "\u001b[33;1m"
        BRIGHT_BLUE = "\u001b[34;1m"
        BRIGHT_MAGENTA = "\u001b[35;1m"
        BRIGHT_CYAN = "\u001b[36;1m"
        BRIGHT_WHITE = "\u001b[37;1m"
                
        @classmethod
        def rgb(cls, r: int, g: int, b: int) -> str:
            """
            Generate ANSI escape code for RGB foreground color.
            
            Creates a 24-bit color escape sequence for terminals that support
            true color mode. Allows precise color specification using RGB values.
            
            :param r: Red component (0-255)
            :param g: Green component (0-255)
            :param b: Blue component (0-255)
            :return: ANSI escape sequence for RGB foreground color
            
            :raises ValueError: If color components are outside 0-255 range
            
            Example
            -------
            ```python
                >>> # Primary colors
                >>> red = ANSIEC.FG.rgb(255, 0, 0)
                >>> green = ANSIEC.FG.rgb(0, 255, 0)
                >>> blue = ANSIEC.FG.rgb(0, 0, 255)
                >>> 
                >>> # Custom theme colors
                >>> header_color = ANSIEC.FG.rgb(64, 128, 255)  # Light blue
                >>> warning_color = ANSIEC.FG.rgb(255, 140, 0)  # Orange
                >>> success_color = ANSIEC.FG.rgb(50, 205, 50)  # Lime green
                >>> 
                >>> print(header_color + "System Information" + ANSIEC.OP.RESET)
                >>> print(warning_color + "Warning message" + ANSIEC.OP.RESET)
                >>> print(success_color + "Operation successful" + ANSIEC.OP.RESET)
                >>> 
                >>> # Gradient effect (simple example)
                >>> for i in range(256):
                ...     color = ANSIEC.FG.rgb(i, 0, 255-i)  # Red to blue gradient
                ...     print(color + "█" + ANSIEC.OP.RESET, end="")
                >>> print()
                >>> 
                >>> # Status indicators with custom colors
                >>> def show_sensor_status(temp, humidity, pressure):
                ...     # Temperature: blue (cold) to red (hot)
                ...     temp_color = ANSIEC.FG.rgb(
                ...         min(255, int(temp * 10)), 
                ...         0, 
                ...         max(0, 255 - int(temp * 10))
                ...     )
                ...     
                ...     print(f"Temperature: {temp_color}{temp:.1f}°C{ANSIEC.OP.RESET}")
                ...     print(f"Humidity: {ANSIEC.FG.rgb(0, 150, 255)}{humidity:.1f}%{ANSIEC.OP.RESET}")
                ...     print(f"Pressure: {ANSIEC.FG.rgb(128, 255, 128)}{pressure:.1f}hPa{ANSIEC.OP.RESET}")
            ```
            """
            if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                raise ValueError("RGB components must be in range 0-255")
            return "\u001b[38;2;{};{};{}m".format(r, g, b)

    class BG:
        """
        ANSI escape codes for background colors.
        
        This class provides background color control for creating highlighted
        text, status indicators, and visual separation in terminal output.
        """
        BLACK = "\u001b[40m"
        RED = "\u001b[41m"
        GREEN = "\u001b[42m"
        YELLOW = "\u001b[43m"
        BLUE = "\u001b[44m"
        MAGENTA = "\u001b[45m"
        CYAN = "\u001b[46m"
        WHITE = "\u001b[47m"
        BRIGHT_BLACK= "\u001b[40;1m"
        BRIGHT_RED = "\u001b[41;1m"
        BRIGHT_GREEN = "\u001b[42;1m"
        BRIGHT_YELLOW = "\u001b[43;1m"
        BRIGHT_BLUE = "\u001b[44;1m"
        BRIGHT_MAGENTA = "\u001b[45;1m"
        BRIGHT_CYAN = "\u001b[46;1m"
        BRIGHT_WHITE = "\u001b[47;1m"
                
        @classmethod
        def rgb(cls, r: int, g: int, b: int) -> str:
            """
            Generate ANSI escape code for RGB background color.
            
            Creates a 24-bit background color escape sequence for terminals
            that support true color mode.
            
            :param r: Red component (0-255)
            :param g: Green component (0-255)
            :param b: Blue component (0-255)
            :return: ANSI escape sequence for RGB background color
            
            :raises ValueError: If color components are outside 0-255 range
            
            Example
            -------
            ```python
                >>> # Custom background highlights
                >>> header_bg = ANSIEC.BG.rgb(25, 25, 112)  # Midnight blue
                >>> success_bg = ANSIEC.BG.rgb(34, 139, 34)  # Forest green
                >>> error_bg = ANSIEC.BG.rgb(178, 34, 34)    # Fire brick red
                >>> 
                >>> print(header_bg + ANSIEC.FG.WHITE + " SYSTEM STATUS " + ANSIEC.OP.RESET)
                >>> print(success_bg + ANSIEC.FG.WHITE + " CONNECTED " + ANSIEC.OP.RESET)
                >>> print(error_bg + ANSIEC.FG.WHITE + " FAILED " + ANSIEC.OP.RESET)
                >>> 
                >>> # Progress bar with gradient background
                >>> def progress_bar_gradient(percent):
                ...     bar_length = 30
                ...     filled = int(bar_length * percent / 100)
                ...     
                ...     result = "["
                ...     for i in range(bar_length):
                ...         if i < filled:
                ...             # Gradient from green to red based on position
                ...             red = int(255 * i / bar_length)
                ...             green = 255 - red
                ...             bg_color = ANSIEC.BG.rgb(red, green, 0)
                ...             result += bg_color + " " + ANSIEC.OP.RESET
                ...         else:
                ...             result += " "
                ...     result += f"] {percent}%"
                ...     return result
            ```
            """
            if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                raise ValueError("RGB components must be in range 0-255")
            return "\u001b[48;2;{};{};{}m".format(r, g, b)

    class OP:
        """
        ANSI escape codes for text formatting and cursor operations.
        
        This class provides text formatting attributes and cursor positioning
        commands for creating sophisticated terminal interfaces with proper
        layout control and text emphasis.
        """
        RESET = "\u001b[0m"
        BOLD = "\u001b[1m"
        UNDER_LINE = "\u001b[4m"
        REVERSE = "\u001b[7m"
        CLEAR = "\u001b[2J"
        CLEAR_LINE = "\u001b[2K"
        TOP = "\u001b[0;0H"

        @classmethod
        def up(cls, n: int) -> str:
            """
            Move cursor up by n lines.
            
            :param n: Number of lines to move up
            :return: ANSI escape sequence for cursor movement
            
            Example
            -------
            ```python
                >>> # Move up and overwrite previous line
                >>> print("Line 1")
                >>> print("Line 2")
                >>> print(ANSIEC.OP.up(1) + "Modified Line 2")
                >>> 
                >>> # Create updating status display
                >>> for i in range(10):
                ...     print(f"Progress: {i*10}%")
                ...     utime.sleep(0.5)
                ...     if i < 9:  # Don't move up on last iteration
                ...         print(ANSIEC.OP.up(1), end="")
            ```
            """
            return "\u001b[{}A".format(n)

        @classmethod
        def down(cls, n: int) -> str:
            """
            Move cursor down by n lines.
            
            :param n: Number of lines to move down
            :return: ANSI escape sequence for cursor movement
            
            Example
            -------
            ```python
                >>> # Skip lines for formatting
                >>> print("Header")
                >>> print(ANSIEC.OP.down(2) + "Content after gap")
                >>> 
                >>> # Create spaced output
                >>> items = ["Item 1", "Item 2", "Item 3"]
                >>> for i, item in enumerate(items):
                ...     print(item)
                ...     if i < len(items) - 1:
                ...         print(ANSIEC.OP.down(1), end="")  # Extra spacing
            ```
            """
            return "\u001b[{}B".format(n)

        @classmethod
        def right(cls, n: int) -> str:
            """
            Move cursor right by n columns.
            
            :param n: Number of columns to move right
            :return: ANSI escape sequence for cursor movement
            
            Example
            -------
            ```python
                >>> # Create indented output
                >>> print("Main item")
                >>> print(ANSIEC.OP.right(4) + "Sub-item 1")
                >>> print(ANSIEC.OP.right(4) + "Sub-item 2")
                >>> 
                >>> # Align text in columns
                >>> print("Name" + ANSIEC.OP.right(10) + "Value")
                >>> print("Temperature" + ANSIEC.OP.right(1) + "25.3°C")
                >>> print("Humidity" + ANSIEC.OP.right(4) + "67%")
            ```
            """
            return "\u001b[{}C".format(n)

        @classmethod
        def left(cls, n: int) -> str:
            """
            Move cursor left by n columns.
            
            :param n: Number of columns to move left
            :return: ANSI escape sequence for cursor movement
            
            Example
            -------
            ```python
                >>> # Overwrite part of a line
                >>> print("Original text", end="")
                >>> print(ANSIEC.OP.left(4) + "NEW")  # Overwrites "text"
                >>> 
                >>> # Create backspace effect
                >>> print("Typing...", end="")
                >>> utime.sleep(1)
                >>> print(ANSIEC.OP.left(3) + "   ")  # Clear last 3 chars
                >>> print(ANSIEC.OP.left(3) + "DONE")
            ```
            """
            return "\u001b[{}D".format(n)
        
        @classmethod
        def next_line(cls, n: int) -> str:
            """
            Move cursor to beginning of next n-th line.
            
            :param n: Number of lines down to move
            :return: ANSI escape sequence for cursor movement
            
            Example
            -------
            ```python
                >>> # Skip to next line with proper positioning
                >>> print("Header line", end="")
                >>> print(ANSIEC.OP.next_line(2) + "Content after blank line")
                >>> 
                >>> # Create separated sections
                >>> print("Section 1")
                >>> print("Details...")
                >>> print(ANSIEC.OP.next_line(2) + "Section 2")
            ```
            """
            return "\u001b[{}E".format(n)

        @classmethod
        def prev_line(cls, n: int) -> str:
            """
            Move cursor to beginning of previous n-th line.
            
            :param n: Number of lines up to move
            :return: ANSI escape sequence for cursor movement
            
            Example
            -------
            ```python
                >>> # Update previous lines
                >>> print("Line 1")
                >>> print("Line 2")
                >>> print("Line 3")
                >>> print(ANSIEC.OP.prev_line(2) + "Updated Line 2")
                >>> 
                >>> # Create real-time log updates
                >>> for i in range(5):
                ...     print(f"Processing step {i+1}...")
                ...     utime.sleep(0.5)
                ...     if i > 0:
                ...         print(ANSIEC.OP.prev_line(1) + f"Step {i+1} complete ✓")
            ```
            """
            return "\u001b[{}F".format(n)
                
        @classmethod
        def to(cls, row: int, column: int) -> str:
            """
            Move cursor to specific row and column position.
            
            :param row: Row number (1-based indexing)
            :param column: Column number (1-based indexing)
            :return: ANSI escape sequence for absolute cursor positioning
            
            Example
            -------
            ```python
                >>> # Create a simple dashboard layout
                >>> print(ANSIEC.OP.CLEAR)
                >>> print(ANSIEC.OP.to(1, 1) + "=== System Dashboard ===")
                >>> print(ANSIEC.OP.to(3, 5) + "CPU Temperature: 42.5°C")
                >>> print(ANSIEC.OP.to(4, 5) + "Memory Usage:    78%")
                >>> print(ANSIEC.OP.to(5, 5) + "Network Status:  Connected")
                >>> print(ANSIEC.OP.to(10, 1) + "Press any key to continue...")
                >>> 
                >>> # Create a data table
                >>> headers = ["Sensor", "Value", "Status"]
                >>> data = [
                ...     ("Temperature", "25.3°C", "OK"),
                ...     ("Humidity", "67%", "OK"),
                ...     ("Pressure", "1013hPa", "WARN")
                ... ]
                >>> 
                >>> # Print headers
                >>> for i, header in enumerate(headers):
                ...     print(ANSIEC.OP.to(1, 1 + i*15) + header)
                >>> 
                >>> # Print data rows
                >>> for row_idx, row_data in enumerate(data):
                ...     for col_idx, cell in enumerate(row_data):
                ...         print(ANSIEC.OP.to(3 + row_idx, 1 + col_idx*15) + cell)
                >>> 
                >>> # Status display with fixed positions
                >>> def update_status_display(cpu_temp, memory, network):
                ...     print(ANSIEC.OP.to(3, 20) + f"{cpu_temp:.1f}°C")
                ...     print(ANSIEC.OP.to(4, 20) + f"{memory}%")
                ...     print(ANSIEC.OP.to(5, 20) + network)
                >>> 
                >>> # Update display without clearing screen
                >>> while True:
                ...     cpu_temp = read_cpu_temperature()
                ...     memory = get_memory_usage()
                ...     network = get_network_status()
                ...     update_status_display(cpu_temp, memory, network)
                ...     utime.sleep(1)
            ```
            """
            return "\u001b[{};{}H".format(row, column)

@micropython.native
def __slip_encode_core(payload, payload_len, out_buf) -> int:
    i: int = 0
    j: int = 1  # Skip first END byte
    
    while i < payload_len:
        b: int = payload[i]
        if b == 0xC0:  # END
            out_buf[j] = 0xDB  # ESC
            out_buf[j + 1] = 0xDC  # ESC_END
            j += 2
        elif b == 0xDB:  # ESC
            out_buf[j] = 0xDB  # ESC
            out_buf[j + 1] = 0xDD  # ESC_ESC
            j += 2
        else:
            out_buf[j] = b
            j += 1
        i += 1
    
    return j

class SlipEncoder:
    """
    SLIP (Serial Line Internet Protocol) encoder for reliable data transmission.
    
    This class implements the SLIP protocol encoding which allows reliable
    transmission of binary data over serial connections by using special
    escape sequences to frame data packets and handle control characters.
    
    SLIP Protocol Details:
        
        - END byte (0xC0): Marks frame boundaries
        - ESC byte (0xDB): Escape character for special bytes
        - ESC_END (0xDC): Escaped version of END byte
        - ESC_ESC (0xDD): Escaped version of ESC byte
    
    Features:
        
        - Reliable binary data transmission
        - Frame boundary detection
        - Escape sequence handling
        - Compatible with SLIP decoders
        - Stateless encoding (static methods)
    
    """
    END      = 0xC0
    ESC      = 0xDB
    ESC_END  = 0xDC
    ESC_ESC  = 0xDD

    @staticmethod
    def encode(payload: bytes) -> bytes:
        """
        Encode a byte array into a SLIP frame.
        
        Converts arbitrary binary data into a SLIP-encoded frame by adding
        frame delimiters and escaping special characters. The resulting
        frame can be safely transmitted over serial connections.
        
        :param payload: The byte array to encode (any binary data)
        :return: SLIP-encoded frame as bytes with delimiters and escaping
        
        :raises TypeError: If payload is not bytes or bytearray
        
        Encoding Rules:
        
            - Frames are surrounded by END bytes (0xC0)
            - END bytes (0xC0) in data are escaped as ESC + ESC_END (0xDB 0xDC)
            - ESC bytes (0xDB) in data are escaped as ESC + ESC_ESC (0xDB 0xDD)
        
        Example
        -------
        ```python
            >>> # Simple text encoding
            >>> text = b"Hello"
            >>> encoded = SlipEncoder.encode(text)
            >>> print(f"Encoded: {encoded.hex()}")  # c048656c6c6fc0
            >>> # Breakdown: C0 (END) + "Hello" + C0 (END)
            >>> 
            >>> # Binary data with special bytes
            >>> data = bytes([0x01, 0xC0, 0x02, 0xDB, 0x03])
            >>> encoded = SlipEncoder.encode(data)
            >>> print(f"Original: {data.hex()}")    # 01c002db03
            >>> print(f"Encoded:  {encoded.hex()}")  # c001dbdc02dbdd03c0
            >>> # Breakdown: C0 + 01 + DB DC (escaped C0) + 02 + DB DD (escaped DB) + 03 + C0
            >>> 
            >>> # JSON data encoding
            >>> import json
            >>> json_str = json.dumps({"temp": 25.5, "humidity": 67})
            >>> json_bytes = json_str.encode('utf-8')
            >>> encoded = SlipEncoder.encode(json_bytes)
            >>> 
            >>> # Struct data encoding  
            >>> import struct
            >>> # Pack sensor readings: temp(float), humidity(int), timestamp(int)
            >>> sensor_data = struct.pack('<fII', 25.5, 67, 1234567890)
            >>> encoded = SlipEncoder.encode(sensor_data)
            >>> 
            >>> # Multiple packet encoding
            >>> packets = [
            ...     b"PING",
            ...     b"DATA:12345", 
            ...     bytes([0x00, 0xFF, 0xC0, 0xDB])  # Test with special chars
            ... ]
            >>> 
            >>> encoded_packets = []
            >>> for packet in packets:
            ...     encoded = SlipEncoder.encode(packet)
            ...     encoded_packets.append(encoded)
            ...     print(f"Packet: {packet.hex() if isinstance(packet, bytes) else packet}")
            ...     print(f"Encoded: {encoded.hex()}")
            >>> 
            >>> # Protocol message encoding
            >>> def create_message(msg_type, msg_id, payload):
            ...     # Message format: [TYPE:1][ID:2][PAYLOAD_LEN:2][PAYLOAD]
            ...     header = struct.pack('<BHH', msg_type, msg_id, len(payload))
            ...     message = header + payload
            ...     return SlipEncoder.encode(message)
            >>> 
            >>> # Create different message types
            >>> ping_msg = create_message(0x01, 1, b"")
            >>> data_msg = create_message(0x02, 2, b"sensor_data")
            >>> config_msg = create_message(0x03, 3, json.dumps({"rate": 100}).encode())
        ```
        """
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError("Payload must be bytes or bytearray")
        
        max_size = len(payload) * 2 + 2  # 2x for worst case + 2 END bytes
        out_buf = bytearray(max_size)
        
        out_buf[0] = 0xC0
        
        if isinstance(payload, bytes):
            payload_array = bytearray(payload)
        else:
            payload_array = payload
            
        actual_len = __slip_encode_core(payload_array, len(payload_array), out_buf)
        
        out_buf[actual_len] = 0xC0
        
        return bytes(out_buf[:actual_len + 1])


class SlipDecoder:
    """
    SLIP (Serial Line Internet Protocol) decoder for reliable data reception.
    
    This class implements the SLIP protocol decoding which allows reliable
    reception and reconstruction of binary data packets from serial connections.
    It handles frame boundary detection, escape sequence processing, and
    automatic error recovery from corrupted data streams.
    
    SLIP Protocol Details:
        
        - END byte (0xC0): Marks frame boundaries
        - ESC byte (0xDB): Escape character for special bytes
        - ESC_END (0xDC): Escaped version of END byte
        - ESC_ESC (0xDD): Escaped version of ESC byte
    
    Features:
        
        - Stateful decoding with internal buffering
        - Automatic frame boundary detection
        - Escape sequence processing
        - Error recovery from invalid sequences
        - Multiple frame extraction from single data chunk
        - Compatible with SlipEncoder output
    
    State Management:
        
        - Maintains internal buffer for incomplete frames
        - Tracks escape sequence state
        - Handles frame boundary detection
        - Provides reset capability for error recovery
    
    """    
    END      = 0xC0
    ESC      = 0xDB
    ESC_END  = 0xDC
    ESC_ESC  = 0xDD

    def __init__(self) -> None:
        """
        Initialize the SLIP decoder with empty state.
        
        Sets up the internal buffer and resets all state flags for processing
        SLIP frames. The decoder is ready to receive data immediately after
        initialization.
        
        Example
        -------
        ```python
            >>> # Create decoder for serial communication
            >>> decoder = SlipDecoder()
            >>> 
            >>> # Multiple decoders for different channels
            >>> control_decoder = SlipDecoder()  # For control messages
            >>> data_decoder = SlipDecoder()     # For data streams
        ```
        """
        self._buf = bytearray()
        self._escaped = False
        self._in_frame = False

    def reset(self) -> None:
        """
        Reset the decoder state to initial conditions.
        
        Clears the internal buffer and resets all state flags. Use this method
        when you need to restart decoding after detecting errors or when
        switching to a new data stream.
        
        Example
        -------
        ```python
            >>> decoder = SlipDecoder()
            >>> 
            >>> # Process some data
            >>> frames = decoder.feed(b'\xc0Hello\xc0')
            >>> print(f"Frames received: {len(frames)}")
            >>> 
            >>> # Reset clears all internal state
            >>> decoder.reset()
            >>> 
            >>> # Error recovery scenario
            >>> def handle_communication_error():
            ...     print("Communication error detected")
            ...     decoder.reset()  # Clear any partial data
            ...     uart.flush()     # Clear hardware buffers
        ```
        """
        self._buf[:] = b''
        self._escaped = False
        self._in_frame = False

    def feed(self, chunk: bytes) -> list[bytes]:
        """
        Feed a chunk of bytes to the decoder and extract complete frames.
        
        Processes incoming data and returns a list of complete SLIP frames.
        Incomplete frames are buffered internally until the next call.
        The method handles escape sequences, frame boundaries, and error
        recovery automatically.
        
        :param chunk: The chunk of bytes to process (any length)
        :return: List of complete SLIP frames as byte arrays
        
        :raises TypeError: If chunk is not bytes or bytearray
        
        Processing Rules:
        
            - END bytes mark frame boundaries
            - ESC+ESC_END sequences become END bytes in output
            - ESC+ESC_ESC sequences become ESC bytes in output
            - Invalid escape sequences cause frame reset
            - Junk data before first frame is ignored
        
        Example
        -------
        ```python
            >>> decoder = SlipDecoder()
            >>> 
            >>> # Single complete frame
            >>> chunk1 = b'\xc0Hello\xc0'
            >>> frames = decoder.feed(chunk1)
            >>> print(f"Frames: {frames}")  # [b'Hello']
            >>> 
            >>> # Partial frame (buffered internally)
            >>> chunk2 = b'\xc0Partial'
            >>> frames = decoder.feed(chunk2)
            >>> print(f"Frames: {frames}")  # [] - no complete frames yet
            >>> 
            >>> # Complete the partial frame
            >>> chunk3 = b' frame\xc0'
            >>> frames = decoder.feed(chunk3)
            >>> print(f"Frames: {frames}")  # [b'Partial frame']
            >>> 
            >>> # Multiple frames in one chunk
            >>> chunk4 = b'\xc0Frame1\xc0\xc0Frame2\xc0\xc0Frame3\xc0'
            >>> frames = decoder.feed(chunk4)
            >>> print(f"Frames: {frames}")  # [b'Frame1', b'Frame2', b'Frame3']
            >>> 
            >>> # Real-world serial receiver
            >>> def serial_receiver():
            ...     decoder = SlipDecoder()
            ...     while True:
            ...         if uart.any():
            ...             chunk = uart.read()
            ...             frames = decoder.feed(chunk)
            ...             for frame in frames:
            ...                 process_message(frame)
            ...         utime.sleep_ms(10)
        ```
        """
        if not isinstance(chunk, (bytes, bytearray)):
            raise TypeError("Chunk must be bytes or bytearray")
            
        frames = []
        for b in chunk:
            if self._escaped:
                if b == self.ESC_END:
                    self._buf.append(self.END)
                elif b == self.ESC_ESC:
                    self._buf.append(self.ESC)
                else:          # invalid escape
                    self.reset()
                    continue
                self._escaped = False
                continue

            if b == self.ESC:
                self._escaped = True
            elif b == self.END:
                if self._in_frame:
                    # END frame 
                    frames.append(bytes(self._buf))
                    self._buf[:] = b''
                    self._in_frame = False
                else:
                    # junk end. start frame
                    self._in_frame = True
            else:
                if self._in_frame:
                    self._buf.append(b)
                # else: junk, byte invalid
        return frames


#@micropython.native
def __ring_buffer_find_pattern(buf, buf_size, head, tail, pattern, pattern_len, max_search):
    available: int = (head - tail) % buf_size
    search_len: int = min(available, max_search)
    
    if search_len < pattern_len:
        return -1
    
    pos: int = tail
    matches: int = 0
    start_pos: int = -1
    
    i: int = 0
    while i < search_len:
        current_byte: int = buf[pos]
        
        if current_byte == pattern[matches]:
            if matches == 0:
                start_pos = i
            matches += 1
            if matches == pattern_len:
                return start_pos
        else:
            if matches > 0:
                matches = 0
                if current_byte == pattern[0]:
                    matches = 1
                    start_pos = i
        
        pos = (pos + 1) % buf_size
        i += 1
    
    return -1

class RingBuffer:
    """
    High-performance circular buffer implementation for byte data streaming.
    
    This class provides an efficient ring buffer (circular buffer) for managing
    byte streams with automatic overflow handling. It's designed for real-time
    data processing, protocol parsing, and buffering serial communication data.
    The buffer uses a fixed-size array with head and tail pointers for O(1)
    operations.
    
    Features:
    
        - Fixed-size circular buffer with automatic wraparound
        - Overflow protection (overwrites oldest data when full)
        - Efficient O(1) put and get operations
        - Pattern searching and extraction
        - Peek operations without data removal
        - Memory-efficient implementation using bytearray
        - Thread-safe for single producer/single consumer
    
    Buffer Management:
    
        - Automatic wraparound when reaching buffer end
        - Oldest data is overwritten when buffer is full
        - Maintains data ordering (FIFO behavior)
        - Efficient memory usage with pre-allocated buffer
    
    Use Cases:
    
        - Serial communication buffering
        - Protocol frame parsing
        - Data stream processing
        - Sensor data collection
        - Real-time data logging
        - Circular logging systems
    
    """
    
    def __init__(self, size: int) -> None:
        """
        Initialize the ring buffer with a specified size.
        
        Creates a circular buffer with the given capacity. The actual usable
        space is (size-1) bytes due to the implementation requiring one slot
        to distinguish between full and empty states.
        
        :param size: The total size of the ring buffer in bytes (minimum 2)
        
        :raises ValueError: If size < 2 (minimum required for ring buffer)
        :raises TypeError: If size is not an integer
        
        Example
        -------
        ```python
            >>> # Create buffers of different sizes for different purposes
            >>> cmd_buffer = RingBuffer(64)    # Small buffer for commands
            >>> data_buffer = RingBuffer(256)  # Medium buffer for sensor data
            >>> log_buffer = RingBuffer(2048)  # Large buffer for logging
            >>> 
            >>> # Buffer sizing based on data rate and latency
            >>> def calculate_buffer_size(data_rate_bps, latency_ms):
            ...     bytes_per_ms = data_rate_bps / 8 / 1000
            ...     min_size = int(bytes_per_ms * latency_ms * 2)  # 2x safety margin
            ...     return max(64, min_size)  # Minimum 64 bytes
            >>> 
            >>> # Create UART buffer based on baud rate
            >>> uart_buffer = RingBuffer(calculate_buffer_size(115200, 100))
        ```
        """
        if not isinstance(size, int):
            raise TypeError("Size must be an integer")
        if size < 2:
            raise ValueError("Buffer size must be at least 2 bytes")
            
        self._buf = bytearray(size)
        self._size = size
        self._head = 0
        self._tail = 0
    
    @micropython.native
    def put(self, data: bytes) -> None:
        """
        Put data into the ring buffer with automatic overflow handling.
        
        Adds data to the buffer, automatically wrapping around when the end
        is reached. If the buffer becomes full, the oldest data is overwritten
        to make room for new data (FIFO with overflow).
        
        :param data: The byte data to put into the buffer
        
        :raises TypeError: If data is not bytes or bytearray
        
        Behavior:
        
            - Data is added byte by byte to the buffer
            - Buffer wraps around at the end (circular behavior)
            - Oldest data is overwritten when buffer is full
            - Operation is atomic for single bytes
        
        Example
        -------
        ```python
            >>> buffer = RingBuffer(10)
            >>> 
            >>> # Basic data insertion
            >>> buffer.put(b"Hello")
            >>> print(f"Available: {buffer.avail()}")  # 5 bytes
            >>> 
            >>> # Add more data (demonstrates overflow)
            >>> buffer.put(b" World")
            >>> print(f"Available: {buffer.avail()}")  # 9 bytes (limited by buffer size)
            >>> 
            >>> # Serial data buffering in an interrupt handler
            >>> def uart_irq_handler(pin):
            ...     # Buffer incoming data quickly
            ...     while uart.any():
            ...         chunk = uart.read()
            ...         if chunk:
            ...             buffer.put(chunk)  # Handles overflow automatically
        ```
        """
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("Data must be bytes or bytearray")
            
        for b in data:
            nxt = (self._head + 1) % self._size
            if nxt == self._tail:
                self._tail = (self._tail + 1) % self._size
            self._buf[self._head] = b
            self._head = nxt

    @micropython.native
    def avail(self) -> int:
        """
        Get the number of bytes currently available in the buffer.
        
        Returns the count of bytes that can be read from the buffer without
        blocking. This represents the amount of data currently stored in
        the circular buffer.
        
        :return: Number of bytes available for reading (0 to size-1)
        
        Example
        -------
        ```python
            >>> buffer = RingBuffer(10)
            >>> print(f"Initial: {buffer.avail()}")  # 0
            >>> 
            >>> buffer.put(b"Hello")
            >>> print(f"After put: {buffer.avail()}")  # 5
            >>> 
            >>> data = buffer.get(2)
            >>> print(f"After get: {buffer.avail()}")  # 3
            >>> 
            >>> # Batch processing based on available data
            >>> def process_when_ready():
            ...     if buffer.avail() >= MIN_BATCH_SIZE:
            ...         batch = buffer.get(MIN_BATCH_SIZE)
            ...         process_data(batch)
            ...     elif buffer.avail() > 0 and timeout_occurred():
            ...         # Process whatever is available after timeout
            ...         process_data(buffer.get(buffer.avail()))
        ```
        """
        return (self._head - self._tail) % self._size

    @micropython.native
    def get(self, n: int = 1) -> bytes:
        """
        Get and remove n bytes from the ring buffer.
        
        Retrieves up to n bytes from the buffer and removes them. If fewer
        than n bytes are available, returns only the available bytes.
        Data is returned in FIFO order (oldest data first).
        
        :param n: Number of bytes to retrieve (default: 1, minimum: 1)
        :return: Retrieved bytes (may be shorter than requested)
        
        :raises ValueError: If n < 1
        :raises TypeError: If n is not an integer
        
        Example
        -------
        ```python
            >>> buffer = RingBuffer(10)
            >>> buffer.put(b"Hello World")  # Note: only 9 bytes fit in a size-10 buffer
            >>> 
            >>> # Get specific number of bytes
            >>> data1 = buffer.get(5)
            >>> print(data1)  # b"Hello"
            >>> 
            >>> # Get remaining bytes
            >>> data2 = buffer.get(buffer.avail())
            >>> print(data2)  # b" Worl" (only what's available)
            >>> 
            >>> # Protocol parsing example
            >>> def parse_packet_header():
            ...     if buffer.avail() >= 4:  # Header size is 4 bytes
            ...         header = buffer.get(4)
            ...         msg_type = header[0]
            ...         msg_len = int.from_bytes(header[1:3], 'big')
            ...         checksum = header[3]
            ...         return msg_type, msg_len, checksum
            ...     return None, None, None
        ```
        """
        if not isinstance(n, int):
            raise TypeError("n must be an integer")
        if n < 1:
            raise ValueError("n must be at least 1")
            
        n = min(n, self.avail())
        if n == 0:
            return b''
            
        out = self._buf[self._tail:self._tail + n] \
            if self._tail + n <= self._size else \
            self._buf[self._tail:] + self._buf[:(self._tail+n)%self._size]
        self._tail = (self._tail + n) % self._size
        return bytes(out)
    
    @micropython.native
    def peek(self, n: int = 1) -> bytes:
        """
        Peek at n bytes from the buffer without removing them.
        
        Returns up to n bytes from the buffer without modifying the buffer
        state. This allows inspection of data before deciding whether to
        consume it. Useful for protocol parsing and lookahead operations.
        
        :param n: Number of bytes to peek at (default: 1, minimum: 1)
        :return: Peeked bytes (may be shorter than requested)
        
        :raises ValueError: If n < 1
        :raises TypeError: If n is not an integer
        
        Example
        -------
        ```python
            >>> buffer = RingBuffer(10)
            >>> buffer.put(b"Hello")
            >>> 
            >>> # Peek without consuming
            >>> peeked = buffer.peek(3)
            >>> print(f"Peeked: {peeked}")  # b"Hel"
            >>> print(f"Available: {buffer.avail()}")  # Still 5 bytes
            >>> 
            >>> # Protocol header inspection
            >>> def inspect_header():
            ...     if buffer.avail() >= 2:
            ...         header = buffer.peek(2)
            ...         if header[0] == 0xAA and header[1] == 0x55:
            ...             # Valid header, consume it and process packet
            ...             buffer.get(2)  # Remove header
            ...             process_packet()
            ...         else:
            ...             # Invalid header, skip one byte and try again
            ...             buffer.get(1)
        ```
        """
        if not isinstance(n, int):
            raise TypeError("n must be an integer")
        if n < 1:
            raise ValueError("n must be at least 1")
            
        n = min(n, self.avail())
        if n == 0:
            return b''

        if self._tail + n <= self._size:
            return bytes(self._buf[self._tail:self._tail + n])

        part1 = self._buf[self._tail:]
        part2 = self._buf[:(self._tail + n) % self._size]
        return bytes(part1 + part2)
    
    def get_until(self, pattern: bytes, max_size: int | None = None) -> bytes | None:
        """
        Extract data from buffer up to and including a specific pattern.
        
        Searches for the first occurrence of the specified pattern in the buffer
        and returns all data from the beginning up to and including the pattern.
        The returned data is removed from the buffer. This is useful for
        extracting complete messages or packets with known terminators.
        
        :param pattern: The byte sequence to search for (delimiter/terminator)
        :param max_size: Maximum bytes to search/return (None for no limit)
        :return: Data up to and including pattern, or None if pattern not found
        
        :raises TypeError: If pattern is not bytes or bytearray
        :raises ValueError: If pattern is empty or max_size < pattern length
        
        Example
        -------
        ```python
            >>> buffer = RingBuffer(256)
            >>> buffer.put(b"Hello\nWorld\nTest\n")
            >>> 
            >>> # Extract line by line
            >>> line1 = buffer.get_until(b'\n')
            >>> print(line1)  # b"Hello\n"
            >>> 
            >>> line2 = buffer.get_until(b'\n')
            >>> print(line2)  # b"World\n"
            >>> 
            >>> # Message protocol parsing
            >>> def parse_messages():
            ...     while True:
            ...         message = buffer.get_until(b'\r\n', max_size=200)
            ...         if message:
            ...             # Remove delimiter and process
            ...             content = message[:-2].decode('utf-8')
            ...             process_message(content)
            ...         else:
            ...             break  # No complete message available
        ```
        """
        if not isinstance(pattern, (bytes, bytearray)):
            raise TypeError("Pattern must be bytes or bytearray")
        if len(pattern) == 0:
            raise ValueError("Pattern cannot be empty")
        
        max_search = min(self.avail(), max_size) if max_size else self.avail()
        
        if max_search < len(pattern):
            return None

        pattern_start = __ring_buffer_find_pattern(self._buf, self._size, self._head, self._tail, pattern, len(pattern), max_search)
        
        if pattern_start == -1:
            return None
        
        length = pattern_start + len(pattern)
        return self.get(length)