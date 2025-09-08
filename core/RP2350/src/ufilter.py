"""
Signal Processing Filter Library

Advanced signal processing filters for sensor data processing, noise reduction,
and real-time signal conditioning. Designed for MicroPython with performance
optimization and memory efficiency for embedded systems.

Features:

- Base filter interface for consistent API across all filter types
- Low-pass, high-pass, and band-pass filters with frequency specifications
- Kalman filter for optimal state estimation and noise reduction
- Moving average and median filters for robust signal smoothing
- Adaptive filters with automatic parameter adjustment
- Cascaded and parallel filter architectures for complex processing
- Memory-efficient implementations optimized for embedded systems
- Real-time performance with @micropython.native optimization
- Comprehensive error handling and parameter validation

Supported Filter Types:

- AlphaFilter: Direct coefficient control for exponential smoothing
- LowPassFilter/HighPassFilter: Frequency-domain filtering with automatic design
- MovingAverageFilter: Linear phase smoothing with circular buffer
- MedianFilter: Non-linear outlier rejection and impulse noise removal
- RMSFilter: Power estimation and signal amplitude monitoring
- KalmanFilter: Optimal estimation with process and measurement noise modeling
- AdaptiveFilter: Self-adjusting smoothing based on signal dynamics
- BiquadFilter: Second-order IIR filters with direct coefficient specification
- ButterworthFilter: Maximally flat frequency response with automatic design
- FIRFilter: Finite impulse response with custom tap coefficients
- FilterChain: Serial connection of multiple filters for complex processing

Performance:

- Optimized for low-power embedded devices with limited memory
- Configurable parameters for different noise levels and response times
- Efficient algorithms with O(1) or O(N) complexity per sample
- Minimal memory footprint with intelligent buffer management
- Non-blocking operation modes for real-time applications
- Automatic numerical stability protection and overflow handling

Applications:

- Sensor data smoothing and noise reduction
- Biomedical signal processing (EMG, ECG, EEG)
- Audio processing and digital signal conditioning
- Motion tracking and accelerometer filtering
- Environmental monitoring and data logging
- Industrial measurement and control systems
- Communication signal processing and detection

Mathematical Foundation:

- Proven filter designs based on established signal processing theory
- Support for both IIR (Infinite Impulse Response) and FIR (Finite Impulse Response) structures
- Linear and non-linear filtering techniques for different noise characteristics
- Frequency-domain and time-domain filter specifications
- Optimal estimation theory implementation (Kalman filtering)
- Statistical signal processing methods for robust performance

"""

from array import array
from math import exp, pi, sqrt, tan, fabs
import micropython

__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"

class FilterError(Exception):
    """Base exception for filter-related errors"""
    pass

class FilterConfigurationError(FilterError):
    """Raised when filter parameters are invalid"""
    pass

class FilterOperationError(FilterError):
    """Raised when filter operation fails"""
    pass


class BaseFilter:
    """
    Base class for all signal processing filters.
    
    This class provides a consistent interface for all filter implementations with
    standardized methods for processing, resetting, and configuration. All filter
    subclasses inherit from this base class to ensure consistent behavior and API
    across different filter types.
    
    The BaseFilter establishes the fundamental contract that all filters must follow:

    - Process samples one at a time through update()
    - Support batch processing for efficiency
    - Maintain internal state with reset capability
    - Track processing statistics
    - Provide parameter validation utilities
    
    Features:
    
        - Consistent interface for all filter implementations
        - Sample counting and processing statistics
        - Built-in parameter validation methods
        - Batch processing capabilities
        - Function call interface support (__call__)
        - Reset functionality for filter state management
        - Memory-efficient implementations
        - Real-time performance optimization
    
    Common Operations:

        - update(): Process single sample through filter
        - process_batch(): Process multiple samples efficiently
        - reset(): Reset filter to initial state
        - __call__(): Allow filter to be called as function
        - sample_count: Get number of processed samples
    
    Parameter Validation:

        - Numeric range validation with min/max bounds
        - Type checking with meaningful error messages
        - Configuration error reporting with detailed messages
        - Consistent error handling across all filter types
    
    Inheritance Pattern:
        All concrete filter classes should inherit from BaseFilter and implement
        the update() method. The base class provides common functionality while
        derived classes implement specific filtering algorithms.
    """
    
    def __init__(self) -> None:
        """
        Initialize base filter with default state.
        
        Sets up internal state tracking and prepares filter for operation.
        All filter implementations should call this base constructor to ensure
        proper initialization of common state variables.
        
        Initializes:
        
            - Sample counter to zero
            - Initialization flag to False
            - Internal state variables for tracking
        
        Example
        -------
        ```python
            >>> # Basic subclass implementation pattern
            >>> class CustomFilter(BaseFilter):
            ...     def __init__(self, custom_param):
            ...         super().__init__()  # Call base constructor first
            ...         self.custom_param = custom_param
            ...         self.internal_state = 0.0
            ...     
            ...     def update(self, x):
            ...         self._sample_count += 1  # Track sample count
            ...         self.internal_state = self.internal_state * 0.9 + x * 0.1
            ...         return self.internal_state
            >>> 
            >>> # Create filter instance
            >>> my_filter = CustomFilter(custom_param=5.0)
            >>> print(f"Initial sample count: {my_filter.sample_count}")  # 0
        ```
        """
        self._initialized = False
        self._sample_count = 0

    def update(self, x: float) -> float:
        """
        Process a single sample through the filter.
        
        This is the core method that all filter subclasses must implement.
        It takes an input sample and returns the filtered output sample.
        The method should update internal filter state and increment the
        sample counter for proper statistics tracking.
        
        :param x: Input sample value (numeric type, will be converted to float)
        :return: Filtered output sample as float
        
        :raises NotImplementedError: This base method must be overridden by subclasses
        
        Implementation Guidelines:
            - Always increment self._sample_count
            - Convert input to float for consistency
            - Update internal filter state
            - Return filtered result as float
            - Handle edge cases (NaN, infinity) appropriately
        
        Example
        -------
        ```python
            >>> # Real-time processing example
            >>> filter_obj = LowPassFilter(fc=1.0, fs=10.0)
            >>> 
            >>> # Read sensor data (simulated here)
            >>> raw_value = 32768  # Simulated ADC reading
            >>> voltage = raw_value * 3.3 / 65535  # Convert to voltage
            >>> 
            >>> # Filter the signal
            >>> filtered_voltage = filter_obj.update(voltage)
            >>> print(f"Raw: {voltage:.3f}V → Filtered: {filtered_voltage:.3f}V")
            >>> 
            >>> # Process a sequence of samples
            >>> sequence = [1.0, 2.0, 1.5, 0.5, 1.0]
            >>> results = []
            >>> for value in sequence:
            ...     results.append(filter_obj.update(value))
            >>> print(f"Processed {filter_obj.sample_count} samples")
        ```
        """
        raise NotImplementedError("Subclasses must implement update method")
    
    def reset(self) -> None:
        """
        Reset filter to initial state.
        
        Clears all internal state and prepares the filter for fresh processing.
        This method resets the sample counter and initialization flag, and
        subclasses should override this method to reset their specific state
        while calling the base implementation.
        
        Base Reset Operations:
            - Resets sample counter to zero
            - Clears initialization flag
            - Prepares filter for new data stream
        
        Subclass Override Pattern:
            Subclasses should call super().reset() first, then reset their
            own internal state variables to initial values.
        
        Example
        -------
        ```python
            >>> # Reset behavior demonstration
            >>> filter_obj = MovingAverageFilter(window_size=5)
            >>> 
            >>> # Process a few samples
            >>> for i in range(10):
            ...     filter_obj.update(i)
            >>> 
            >>> print(f"Before reset: {filter_obj.sample_count} samples processed")
            >>> 
            >>> # Reset the filter
            >>> filter_obj.reset()
            >>> print(f"After reset: {filter_obj.sample_count} samples processed")
            >>> 
            >>> # Start fresh processing
            >>> new_result = filter_obj.update(20.0)
            >>> print(f"First new sample result: {new_result}")
        ```
        """
        self._initialized = False
        self._sample_count = 0
    
    def __call__(self, x: float) -> float:
        """
        Allow filter to be called as a function.
        
        This enables convenient functional-style usage of filters by making
        the filter object callable. It's equivalent to calling the update()
        method directly but provides a more concise syntax for functional
        programming patterns.
        
        :param x: Input sample value (numeric type)
        :return: Filtered output sample as float
        
        Functional Programming Benefits:
        
            - Cleaner syntax for single-sample processing
            - Compatible with higher-order functions
            - Enables filter chaining and composition
            - Supports lambda-like usage patterns
        
        Example
        -------
        ```python
            >>> filter_obj = LowPassFilter(fc=1.0, fs=10.0)
            >>> 
            >>> # These are equivalent:
            >>> result1 = filter_obj.update(5.0)
            >>> result2 = filter_obj(5.0)
            >>> print(f"update(): {result1:.3f}, __call__(): {result2:.3f}")
            >>> 
            >>> # Functional programming style
            >>> data = [1.0, 2.0, 3.0, 4.0, 5.0]
            >>> filtered_data = [filter_obj(x) for x in data]
            >>> print(f"Filtered: {[f'{x:.2f}' for x in filtered_data]}")
            >>> 
            >>> # Filter composition
            >>> def create_filter_pipeline(data):
            ...     noise_filter = MedianFilter(window_size=3)
            ...     smooth_filter = LowPassFilter(fc=2.0, fs=20.0)
            ...     
            ...     # Chain filters using __call__
            ...     return [smooth_filter(noise_filter(x)) for x in data]
        ```
        """
        return self.update(x)
    
    def process_batch(self, samples: list) -> list:
        """
        Process multiple samples efficiently.
        
        Processes a list or sequence of input samples and returns a list of
        filtered outputs. This method provides a convenient way to process
        multiple samples at once and can be overridden by subclasses for
        more efficient batch processing if the filter algorithm allows it.
        
        :param samples: Iterable of input sample values (list, tuple, etc.)
        :return: List of filtered output samples in the same order
        
        :raises TypeError: If samples is not iterable
        
        Batch Processing Benefits:
        
            - Convenient for processing stored data
            - Can be optimized by subclasses for better performance
            - Maintains sample order (FIFO processing)
            - Reduces function call overhead for large datasets
            - Suitable for offline processing and analysis
        
        Example
        -------
        ```python
            >>> filter_obj = MovingAverageFilter(window_size=3)
            >>> 
            >>> # Process batch of samples
            >>> input_data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
            >>> output_data = filter_obj.process_batch(input_data)
            >>> 
            >>> print("Input: ", input_data)
            >>> print("Output:", [f"{x:.2f}" for x in output_data])
            >>> # Output: ['1.00', '1.50', '2.00', '3.00', '4.00', '5.00']
            >>> 
            >>> # File processing example
            >>> def process_data_file(filename, filter_obj):
            ...     # Read data from file (simulated)
            ...     raw_data = [1.2, 1.5, 1.1, 1.8, 1.3]  # Simulated file data
            ...     
            ...     # Process entire dataset at once
            ...     filtered_data = filter_obj.process_batch(raw_data)
            ...     
            ...     print(f"Processed {len(filtered_data)} samples")
            ...     return filtered_data
        ```
        """
        if not hasattr(samples, '__iter__'):
            raise TypeError("samples must be iterable")
        
        return [self.update(sample) for sample in samples]
    
    @property
    def sample_count(self) -> int:
        """
        Get number of samples processed since reset.
        
        Returns the total count of samples that have been processed through
        this filter instance since the last reset() call. This provides
        useful statistics for monitoring filter usage and performance.
        
        :return: Number of samples processed (non-negative integer)
        
        Statistics and Monitoring:
        
            - Tracks total throughput since last reset
            - Useful for performance analysis and debugging
            - Enables periodic operations based on sample count
            - Helps with filter state management
        
        Example
        -------
        ```python
            >>> filter_obj = MedianFilter(window_size=5)
            >>> 
            >>> print(f"Initial count: {filter_obj.sample_count}")  # 0
            >>> 
            >>> # Process some samples
            >>> for i in range(10):
            ...     result = filter_obj.update(i)
            >>> 
            >>> print(f"Processed {filter_obj.sample_count} samples")  # 10
            >>> 
            >>> # Periodic logging based on sample count
            >>> def process_with_logging(filter_obj, input_data, log_interval=5):
            ...     for sample in input_data:
            ...         result = filter_obj.update(sample)
            ...         
            ...         # Log periodically based on sample count
            ...         if filter_obj.sample_count % log_interval == 0:
            ...             print(f"Processed {filter_obj.sample_count} samples")
        ```
        """
        return self._sample_count
    
    def _validate_numeric(self, value: float, name: str, min_val: float = None, max_val: float = None) -> float:
        """
        Validate numeric parameters with optional range checking.
        
        Ensures that a parameter is numeric and optionally within specified bounds.
        Used internally by filter implementations for parameter validation during
        initialization and configuration updates.
        
        :param value: Value to validate (will be converted to float)
        :param name: Parameter name for error messages (descriptive string)
        :param min_val: Minimum allowed value (inclusive, optional)
        :param max_val: Maximum allowed value (inclusive, optional)
        :return: Validated numeric value as float
        
        :raises FilterConfigurationError: If value is invalid or out of range
        :raises TypeError: If value cannot be converted to float
        
        Validation Rules:
            - Value must be convertible to float
            - If min_val specified, value must be >= min_val
            - If max_val specified, value must be <= max_val
            - Provides clear error messages with parameter names
        
        Example
        -------
        ```python
            >>> # In a filter subclass implementation
            >>> class LowPassFilter(BaseFilter):
            ...     def __init__(self, fc, fs):
            ...         super().__init__()
            ...         
            ...         # Validate sampling frequency
            ...         self.fs = self._validate_numeric(fs, "sampling frequency", min_val=0.0)
            ...         
            ...         # Validate cutoff frequency with both min and max bounds
            ...         self.fc = self._validate_numeric(
            ...             fc, "cutoff frequency", min_val=0.0, max_val=self.fs/2
            ...         )
            ...         
            ...         # Calculate filter coefficient
            ...         self.alpha = 1.0 - exp(-2.0 * pi * self.fc / self.fs)
            >>> 
            >>> # Valid parameter usage
            >>> filter1 = LowPassFilter(fc=10.0, fs=100.0)
            >>> print(f"Valid filter created: fc={filter1.fc}, fs={filter1.fs}")
            >>> 
            >>> # Invalid parameter usage (will raise error)
            >>> try:
            ...     filter2 = LowPassFilter(fc=60.0, fs=100.0)  # fc >= fs/2 (Nyquist violation)
            >>> except FilterConfigurationError as e:
            ...     print(f"Configuration error: {e}")
            >>> # Output: "cutoff frequency must be <= 50.0, got 60.0"
        ```
        """
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
    """
    Single-pole IIR filter with direct alpha coefficient specification.
    
    A simple first-order infinite impulse response (IIR) filter that implements
    exponential smoothing with a user-specified alpha coefficient. This filter
    provides direct control over the filtering strength without requiring frequency
    domain calculations, making it ideal for rapid prototyping, real-time tuning,
    and applications where simplicity is preferred over frequency-domain precision.
    
    The filter implements the classic exponential smoothing equation:
        y[n] = α·x[n] + (1-α)·y[n-1]
    
    Where:
    
        - α (alpha) is the smoothing factor (0 < α ≤ 1)
        - Higher α values → faster response, less smoothing
        - Lower α values → slower response, more smoothing
    
    Key Characteristics:
    
        - Direct coefficient control without frequency calculations
        - Real-time adjustable alpha parameter
        - Memory-efficient single-delay implementation
        - Predictable exponential decay response
        - Always stable for valid alpha range
        - Zero-phase initialization capability
    
    Applications:
    
        - Sensor data smoothing with adjustable responsiveness
        - Real-time signal conditioning with dynamic tuning
        - Rapid prototyping and algorithm development
        - Educational demonstrations of IIR filtering
        - Simple noise reduction for embedded systems
        - Adaptive filtering with coefficient modulation
    
    Mathematical Properties:
    
        - Impulse response: h[n] = α·(1-α)ⁿ for n ≥ 0
        - Step response: y[n] = 1 - (1-α)ⁿ⁺¹ for unit step
        - Time constant: τ = -1/ln(1-α) samples
        - 3dB frequency: f₃dB ≈ -fs·ln(1-α)/(2π) Hz
        - DC gain: 0dB (unity gain)
        - Stability: Always stable for 0 < α ≤ 1
    
    Alpha Selection Guidelines:
    
        - α = 0.1: Heavy smoothing, slow response (τ ≈ 9.5 samples)
        - α = 0.2: Moderate smoothing (τ ≈ 4.5 samples)
        - α = 0.5: Balanced response (τ ≈ 1.4 samples)
        - α = 0.8: Light smoothing, fast response (τ ≈ 0.7 samples)
        - α = 1.0: No filtering (pass-through)
    
    """
    
    def __init__(self, alpha: float, initial: float = 0.0) -> None:
        """
        Initialize alpha filter with specified smoothing coefficient.
        
        Creates a first-order IIR filter with direct alpha coefficient control,
        providing exponential smoothing without requiring frequency domain
        calculations. The alpha parameter directly controls the trade-off
        between responsiveness and smoothing strength.
        
        :param alpha: Smoothing coefficient in range (0, 1]

            - Higher values (→1.0): Less smoothing, faster response
            - Lower values (→0.0): More smoothing, slower response
            - Typical range: 0.01 to 0.9 for practical applications

        :param initial: Initial output value for filter state (default: 0.0)
                       Useful for avoiding startup transients when expected
                       signal level is known
        
        :raises FilterConfigurationError: If alpha is not in valid range (0, 1]
        :raises TypeError: If parameters cannot be converted to numeric types
        
        Alpha Selection Guidelines:
            Choose alpha based on your application requirements:
            
            Conservative Filtering (α = 0.01 to 0.1):

                - Heavy noise suppression
                - Slow response to signal changes
                - Good for: Temperature monitoring, battery voltage
                - Time constant: ~10-100 samples
            
            Moderate Filtering (α = 0.1 to 0.3):

                - Balanced noise reduction and responsiveness
                - General-purpose applications
                - Good for: Sensor smoothing, control systems
                - Time constant: ~3-10 samples
            
            Light Filtering (α = 0.3 to 0.7):

                - Minimal smoothing, fast response
                - Preserves signal dynamics
                - Good for: Audio processing, vibration monitoring
                - Time constant: ~0.5-3 samples
            
            Minimal Filtering (α = 0.7 to 1.0):

                - Very light smoothing
                - Near real-time response
                - Good for: Control feedback, rapid prototyping
                - Time constant: <1 sample
        
        Mathematical Relationships:
            Time Constant: τ = -1/ln(1-α) samples
            3dB Frequency: f₃dB ≈ -fs·ln(1-α)/(2π) Hz (approximate)
            Step Response: y[n] = initial + (target-initial)·(1-(1-α)ⁿ⁺¹)
            Settling Time (95%): n₉₅% ≈ 3·τ = -3/ln(1-α) samples
        
        Example
        -------
        ```python
            >>> # Creating filters for different applications
            >>> # Conservative filtering for temperature sensor
            >>> temp_filter = AlphaFilter(alpha=0.05, initial=25.0)
            >>> print(f"Time constant: {-1/log(1-0.05):.1f} samples")  # ~19.5
            >>> 
            >>> # Moderate filtering for general sensor data
            >>> sensor_filter = AlphaFilter(alpha=0.2, initial=0.0)
            >>> print(f"Time constant: {-1/log(1-0.2):.1f} samples")  # ~4.5
            >>> 
            >>> # Light filtering for responsive control
            >>> control_filter = AlphaFilter(alpha=0.6, initial=5.0)
            >>> print(f"Time constant: {-1/log(1-0.6):.1f} samples")  # ~1.1
            >>> 
            >>> # Parameter validation examples
            >>> try:
            ...     invalid_filter = AlphaFilter(alpha=0.0)  # Too low
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> try:
            ...     invalid_filter = AlphaFilter(alpha=1.5)  # Too high
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
        ```
        """
        super().__init__()
        if alpha <= 0.0 or alpha > 1.0:
            raise FilterConfigurationError("alpha must be in range (0, 1]")
        self._alpha = float(alpha)
        self.y = float(initial)
        self._initial_value = float(initial)

    @property
    def alpha(self) -> float:
        """
        Get current alpha coefficient.
        
        Returns the current smoothing coefficient value. This property
        allows runtime inspection of the filter's responsiveness setting.
        
        :return: Current alpha value (0 < α ≤ 1)
        
        The alpha value determines the filter's behavior:
            
            - α → 1.0: Minimal filtering, maximum responsiveness
            - α → 0.0: Maximum filtering, minimal responsiveness
        
        Example
        -------
        ```python
            >>> # Inspecting and analyzing filter characteristics
            >>> filter_obj = AlphaFilter(alpha=0.3)
            >>> print(f"Current alpha: {filter_obj.alpha}")  # 0.3
            >>> 
            >>> # Check filter characteristics
            >>> time_constant = -1 / log(1 - filter_obj.alpha)
            >>> print(f"Time constant: {time_constant:.2f} samples")
            >>> 
            >>> # Calculate settling time
            >>> settling_time = -3 / log(1 - filter_obj.alpha)
            >>> print(f"95% settling time: {settling_time:.2f} samples")
            >>> 
            >>> # Estimate frequency response (with 100 Hz sampling)
            >>> fs = 100.0  # Hz
            >>> cutoff_freq = -fs * log(1 - filter_obj.alpha) / (2 * pi)
            >>> print(f"Approximate 3dB cutoff: {cutoff_freq:.2f} Hz")
        ```
        """
        return self._alpha

    @alpha.setter
    def alpha(self, value: float) -> None:
        """
        Set alpha coefficient with validation.
        
        Updates the smoothing coefficient while ensuring it remains within
        the valid range. This allows real-time adjustment of filter
        characteristics without recreating the filter object.
        
        :param value: New alpha value (0 < α ≤ 1)
        
        :raises FilterConfigurationError: If value is not in valid range (0, 1]
        :raises TypeError: If value cannot be converted to float
        
        Real-time Tuning Applications:
            
            - Adaptive filtering based on signal conditions
            - User-adjustable smoothing controls
            - Automatic noise level compensation
            - Dynamic system response optimization
        
        Example
        -------
        ```python
            >>> # Real-time filter adjustment based on conditions
            >>> filter_obj = AlphaFilter(alpha=0.2)
            >>> 
            >>> # Adjust filtering strength in real-time
            >>> for noise_level in [0.1, 0.5, 0.9, 0.3]:
            ...     if noise_level > 0.7:
            ...         filter_obj.alpha = 0.05  # Heavy filtering for high noise
            ...     elif noise_level < 0.2:
            ...         filter_obj.alpha = 0.6   # Light filtering for low noise
            ...     else:
            ...         filter_obj.alpha = 0.2   # Moderate filtering
            ...     
            ...     print(f"Noise: {noise_level:.1f} → Alpha: {filter_obj.alpha:.2f}")
            >>> # Noise: 0.1 → Alpha: 0.60
            >>> # Noise: 0.5 → Alpha: 0.20
            >>> # Noise: 0.9 → Alpha: 0.05
            >>> # Noise: 0.3 → Alpha: 0.20
            >>> 
            >>> # User interface for smoothing adjustment
            >>> def set_smoothing_level(filter_obj, level):
            ...     '''Set smoothing from user-friendly 1-10 scale.'''
            ...     if not (1 <= level <= 10):
            ...         raise FilterConfigurationError("Smoothing level must be 1-10")
            ...     
            ...     # Convert 1-10 scale to alpha (1=heavy smoothing, 10=light)
            ...     filter_obj.alpha = 0.02 + (level - 1) * 0.1
            ...     print(f"Smoothing level {level} → α={filter_obj.alpha:.3f}")
            >>> 
            >>> user_filter = AlphaFilter(alpha=0.2)
            >>> set_smoothing_level(user_filter, 3)  # Smoothing level 3 → α=0.220
            >>> set_smoothing_level(user_filter, 8)  # Smoothing level 8 → α=0.720
        ```
        """
        if value <= 0.0 or value > 1.0:
            raise FilterConfigurationError("alpha must be in range (0, 1]")
        self._alpha = float(value)

    @micropython.native
    def update(self, x: float) -> float:
        """
        Update filter with new sample and return filtered output.
        
        Applies exponential smoothing to the input sample using the current
        alpha coefficient. This is the core filtering operation that implements
        the first-order IIR difference equation.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Filtered output sample as float
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
            The filter implements: y[n] = α·x[n] + (1-α)·y[n-1]
            
            Where:
            
            - y[n] is the current output
            - x[n] is the current input
            - y[n-1] is the previous output (stored in self.y)
            - α is the smoothing coefficient (self._alpha)
        
        Performance Characteristics:
            
            - O(1) computational complexity
            - Single multiply-accumulate operation
            - Minimal memory usage (one float for previous output)
            - Optimized with @micropython.native decorator
            - Suitable for high-frequency real-time processing
        
        Numerical Properties:
            
            - Always stable for 0 < α ≤ 1
            - No risk of overflow for finite inputs
            - Maintains precision for typical signal ranges
            - Graceful handling of extreme input values
        
        Example
        -------
        ```python
            >>> # Processing samples through the filter
            >>> filter_obj = AlphaFilter(alpha=0.3, initial=0.0)
            >>> 
            >>> # Process a sequence of samples
            >>> test_sequence = [1.0, 2.0, 1.5, 3.0, 2.5, 1.0, 0.5]
            >>> 
            >>> print("Input  | Output | Change")
            >>> print("-" * 25)
            >>> 
            >>> for i, sample in enumerate(test_sequence):
            ...     previous_output = filter_obj.y
            ...     current_output = filter_obj.update(sample)
            ...     change = current_output - previous_output
            ...     
            ...     print(f"{sample:5.1f} | {current_output:6.3f} | {change:+6.3f}")
            >>> # Input  | Output | Change
            >>> # -------------------------
            >>> #   1.0 |  0.300 | +0.300
            >>> #   2.0 |  0.810 | +0.510
            >>> #   1.5 |  1.017 | +0.207
            >>> #   3.0 |  1.612 | +0.595
            >>> #   2.5 |  1.878 | +0.267
            >>> #   1.0 |  1.615 | -0.263
            >>> #   0.5 |  1.330 | -0.285
            >>> 
            >>> # Real-time filtering for sensor data
            >>> def process_sensor_data():
            ...     accel_filter = AlphaFilter(alpha=0.1)  # Heavy smoothing
            ...     
            ...     # Simulate 10 sensor readings with noise
            ...     true_values = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
            ...     noise = [0.2, -0.3, 0.1, 0.4, -0.2, 0.3, -0.4, 0.2, -0.1, 0.3]
            ...     
            ...     print("Raw    | Filtered | Error")
            ...     print("-" * 30)
            ...     
            ...     for i, (true, n) in enumerate(zip(true_values, noise)):
            ...         raw = true + n  # Simulated noisy reading
            ...         filtered = accel_filter.update(raw)
            ...         error = abs(filtered - true)
            ...         
            ...         print(f"{raw:6.2f} | {filtered:8.2f} | {error:5.2f}")
        ```
        """
        self._sample_count += 1
        x = float(x)
        self.y = self._alpha * x + (1.0 - self._alpha) * self.y
        return self.y
    
    def reset(self) -> None:
        """
        Reset filter to initial state while preserving configuration.
        
        Clears the filter's internal state (previous output value) and resets
        the sample counter, but maintains the alpha coefficient and initial
        value settings. This allows the filter to be reused for new data
        streams without reconfiguration.
        
        Reset Operations:
           
            - Restores output value to initial setting
            - Resets sample counter to zero
            - Preserves alpha coefficient
            - Preserves initial value setting
            - Prepares filter for new input sequence
        
        Use Cases:
           
            - Starting new measurement session
            - Switching between different signal sources
            - Clearing filter memory after transient events
            - Batch processing of multiple datasets
            - A/B testing with consistent initial conditions
        
        Example
        -------
        ```python
            >>> # Demonstrating filter reset for batch processing
            >>> filter_obj = AlphaFilter(alpha=0.3, initial=5.0)
            >>> 
            >>> # Process first batch of data
            >>> first_batch = [6.0, 7.0, 8.0, 6.5, 7.5]
            >>> first_results = []
            >>> 
            >>> for sample in first_batch:
            ...     result = filter_obj.update(sample)
            ...     first_results.append(result)
            >>> 
            >>> print("First batch results:", [f"{x:.2f}" for x in first_results])
            >>> print(f"Final state: {filter_obj.y:.2f}, Samples: {filter_obj.sample_count}")
            >>> 
            >>> # Reset filter for second batch
            >>> filter_obj.reset()
            >>> print(f"After reset: {filter_obj.y:.2f}, Samples: {filter_obj.sample_count}")
            >>> 
            >>> # Process second batch with same filter
            >>> second_batch = [10.0, 9.5, 11.0, 10.5, 9.0]
            >>> second_results = []
            >>> 
            >>> for sample in second_batch:
            ...     result = filter_obj.update(sample)
            ...     second_results.append(result)
            >>> 
            >>> print("Second batch results:", [f"{x:.2f}" for x in second_results])
            >>> 
            >>> # Compare how each batch started from the same initial conditions
            >>> print(f"First sample: Batch 1={first_results[0]:.2f}, Batch 2={second_results[0]:.2f}")
        ```
        """
        super().reset()
        self.y = self._initial_value


class LowPassFilter(BaseFilter):
    """
    First-order low-pass filter with cutoff frequency specification.
    
    A digital low-pass filter that allows low-frequency components to pass through
    while attenuating high-frequency components above the cutoff frequency. This
    implementation uses a first-order IIR (Infinite Impulse Response) structure
    with exponential decay characteristics, making it ideal for noise reduction
    and signal smoothing applications.
    
    The filter implements the difference equation:
        
        y[n] = α·x[n] + (1-α)·y[n-1]
    
    Where α (alpha) is automatically calculated from the cutoff and sampling frequencies:
        
        α = 1 - exp(-2π·fc/fs)
    
    Features:
        
        - Automatic coefficient calculation from frequency specifications
        - Standard first-order low-pass response (-20dB/decade rolloff)
        - Configurable cutoff frequency and sampling rate
        - Memory-efficient single-delay implementation
        - Real-time processing with minimal computational overhead
        - Numerical stability through proper coefficient bounds
    
    Applications:
        
        - Sensor data smoothing and noise reduction
        - Anti-aliasing before downsampling
        - Signal conditioning for control systems
        - EMG/ECG signal preprocessing
        - Audio processing and equalization
        - Vibration analysis and filtering
    
    Mathematical Properties:
        
        - 3dB cutoff at specified frequency fc
        - Phase lag increases with frequency
        - Group delay: τ = (1-α)/(2π·fc·α)
        - Stability: Always stable for 0 < α ≤ 1
        - DC gain: 0dB (unity)
        - High-frequency attenuation: -20dB/decade
    
    """
    
    def __init__(self, fc: float, fs: float, initial: float = 0.0) -> None:
        """
        Initialize low-pass filter with frequency specifications.
        
        Creates a first-order digital low-pass filter by calculating the appropriate
        filter coefficient (alpha) from the specified cutoff and sampling frequencies.
        The filter uses bilinear transform equivalent for first-order analog prototype.
        
        :param fc: Cutoff frequency in Hz (3dB point, must be > 0 and < fs/2)
        :param fs: Sampling frequency in Hz (must be > 0 and > 2*fc)
        :param initial: Initial output value for filter state (default: 0.0)
        
        :raises FilterConfigurationError: If frequencies are invalid or violate Nyquist criterion
        :raises TypeError: If parameters cannot be converted to float
        
        Filter Coefficient Calculation:
            The alpha coefficient is calculated using the formula:
        
            α = 1 - exp(-2π·fc/fs)
            
            This ensures:
        
            - Correct 3dB cutoff frequency at fc
            - Proper frequency response characteristics  
            - Numerical stability for all valid frequency ratios
            - Smooth transition between passband and stopband
        
        Frequency Constraints:
        
            - fc must be positive (fc > 0)
            - fc must be less than Nyquist frequency (fc < fs/2)
            - fs must be positive (fs > 0)
            - Recommended: fc should be at least 10x smaller than fs for good approximation
        
        Example
        -------
        ```python
            >>> # Audio processing filter
            >>> audio_filter = LowPassFilter(fc=5000.0, fs=44100.0, initial=0.0)
            >>> print(f"Filter alpha: {audio_filter._alpha:.6f}")
            >>> 
            >>> # Sensor data filter  
            >>> sensor_filter = LowPassFilter(fc=2.0, fs=100.0)
            >>> print(f"Cutoff: {sensor_filter.fc} Hz, Sampling: {sensor_filter.fs} Hz")
            >>> 
            >>> # Control system filter
            >>> control_filter = LowPassFilter(fc=0.1, fs=10.0, initial=5.0)
            >>> print(f"Initial output: {control_filter.y}")
            >>> 
            >>> # Invalid configurations (will raise FilterConfigurationError)
            >>> try:
            ...     invalid_filter = LowPassFilter(fc=60.0, fs=100.0)  # fc >= fs/2
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> try:
            ...     invalid_filter = LowPassFilter(fc=-5.0, fs=100.0)  # Negative fc
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> # Coefficient analysis
            >>> def analyze_coefficients():
            ...     fs = 100.0
            ...     cutoffs = [0.1, 0.5, 1.0, 5.0, 10.0, 20.0, 40.0]
            ...     
            ...     print("Cutoff (Hz) | Alpha     | -3dB Freq | Time Const")
            ...     print("-" * 50)
            ...     
            ...     for fc in cutoffs:
            ...         if fc < fs / 2:
            ...             filter_obj = LowPassFilter(fc=fc, fs=fs)
            ...             alpha = filter_obj._alpha
            ...             time_constant = (1 - alpha) / (2 * pi * fc * alpha)
            ...             
            ...             print(f"{fc:10.1f} | {alpha:8.6f} | {fc:8.1f} | {time_constant:9.3f}")
        ```
        """
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
        """
        Update filter with new sample and return filtered output.
        
        Processes a single input sample through the first-order IIR low-pass filter
        using the difference equation: y[n] = α·x[n] + (1-α)·y[n-1]
        
        This method applies exponential smoothing to the input signal, where the
        filter coefficient α determines the balance between responsiveness to new
        samples and smoothness of the output.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Filtered output sample as float
        
        :raises TypeError: If input cannot be converted to float
        
        Performance Characteristics:

            - O(1) computational complexity
            - Single multiply-add operation per sample
            - Minimal memory usage (single delay element)
            - Optimized with @micropython.native decorator
            - Suitable for real-time applications
        
        Signal Processing Properties:

            - Amplitude response: |H(f)| = α / sqrt((α²) + (1-α)²)
            - Phase response: φ(f) = -arctan((1-α)/α * sin(2πf/fs) / (1 + (1-α)cos(2πf/fs)))
            - Group delay: τ = (1-α)/(2πfc·α) at low frequencies
            - Step response: y(n) = α·(1-(1-α)ⁿ) for unit step input
        
        Example
        -------
        ```python
            >>> # Basic filtering operation
            >>> filter_obj = LowPassFilter(fc=5.0, fs=50.0)
            >>> 
            >>> # Process single sample
            >>> output = filter_obj.update(10.0)
            >>> print(f"First sample: {output:.4f}")  # Shows initial response
            >>> 
            >>> # Process sequence of samples
            >>> input_sequence = [1.0, 2.0, 1.5, 3.0, 2.5, 1.0, 0.5]
            >>> output_sequence = []
            >>> 
            >>> for i, sample in enumerate(input_sequence):
            ...     filtered = filter_obj.update(sample)
            ...     output_sequence.append(filtered)
            ...     print(f"Sample {i}: {sample:4.1f} → {filtered:6.4f}")
            >>> 
            >>> # Analyze step response
            >>> def step_response_analysis():
            ...     step_filter = LowPassFilter(fc=2.0, fs=20.0)
            ...     
            ...     print("Step Response Analysis:")
            ...     print("Sample | Input | Output | Error")
            ...     print("-" * 35)
            ...     
            ...     for n in range(20):
            ...         step_input = 1.0  # Unit step
            ...         output = step_filter.update(step_input)
            ...         
            ...         # Theoretical step response: 1 - (1-α)^n
            ...         alpha = step_filter._alpha
            ...         theoretical = 1.0 - (1.0 - alpha) ** (n + 1)
            ...         error = abs(output - theoretical)
            ...         
            ...         print(f"{n:6d} | {step_input:5.1f} | {output:6.4f} | {error:6.5f}")
            ...         
            ...         if output > 0.95:  # 95% of final value
            ...             print(f"95% settling time: {n} samples ({n/20:.2f} seconds)")
            ...             break
            >>> 
            >>> # Frequency response visualization
            >>> def visualize_frequency_response():
            ...     filter_obj = LowPassFilter(fc=10.0, fs=100.0)
            ...     
            ...     # Test at various frequencies
            ...     test_freqs = [1, 2, 5, 10, 15, 20, 30, 40]
            ...     
            ...     print("Frequency Response:")
            ...     print("Freq (Hz) | Gain (dB) | Phase (deg)")
            ...     print("-" * 40)
            ...     
            ...     for freq in test_freqs:
            ...         # Calculate theoretical response
            ...         alpha = filter_obj._alpha
            ...         w = 2 * pi * freq / filter_obj.fs
            ...         
            ...         # Magnitude response
            ...         numerator = alpha
            ...         denominator = sqrt(alpha**2 + (1-alpha)**2 - 2*alpha*(1-alpha)*cos(w))
            ...         gain = numerator / denominator if denominator > 0 else 1.0
            ...         gain_db = 20 * log10(gain) if gain > 0 else -60
            ...         
            ...         # Phase response (simplified)
            ...         phase_rad = -atan2((1-alpha) * sin(w), alpha + (1-alpha) * cos(w))
            ...         phase_deg = phase_rad * 180 / pi
            ...         
            ...         print(f"{freq:8.1f} | {gain_db:9.2f} | {phase_deg:11.1f}")
        ```
        """
        self._sample_count += 1
        x = float(x)
        self.y = self._alpha * x + (1.0 - self._alpha) * self.y
        return self.y
    
    def reset(self) -> None:
        """
        Reset filter to initial state while preserving configuration.
        
        Clears the filter's internal state (output history) and resets the sample
        counter, but maintains all configuration parameters like cutoff frequency,
        sampling frequency, and calculated coefficients. This allows the filter
        to be reused for new data streams without reconfiguration.
        
        Reset Operations:
        
            - Clears output history (y[n-1])
            - Resets sample counter to zero
            - Preserves filter coefficients and parameters
            - Restores initial output value
            - Prepares filter for new input sequence
        
        Use Cases:
        
            - Starting new measurement session
            - Switching between different signal sources
            - Clearing filter memory after transient events
            - Periodic reset to prevent numerical drift
            - Batch processing of multiple datasets
        
        Example
        -------
        ```python
            >>> filter_obj = LowPassFilter(fc=5.0, fs=50.0, initial=2.0)
            >>> 
            >>> # Process some data
            >>> for i in range(10):
            ...     result = filter_obj.update(i)
            ...     print(f"Sample {i}: {result:.3f}")
            >>> 
            >>> print(f"Samples processed: {filter_obj.sample_count}")  # 10
            >>> print(f"Current output: {filter_obj.y:.3f}")
            >>> 
            >>> # Reset filter
            >>> filter_obj.reset()
            >>> print(f"After reset - Samples: {filter_obj.sample_count}")  # 0
            >>> print(f"After reset - Output: {filter_obj.y:.3f}")  # 2.0 (initial value)
            >>> 
            >>> # Filter is ready for new data with same configuration
            >>> result = filter_obj.update(100.0)
            >>> print(f"First new sample: {result:.3f}")
            >>> 
            >>> # Batch processing example
            >>> def process_multiple_datasets():
            ...     data_filter = LowPassFilter(fc=2.0, fs=20.0)
            ...     
            ...     datasets = [
            ...         [1, 2, 3, 4, 5],
            ...         [10, 20, 30, 40, 50],
            ...         [0.1, 0.2, 0.3, 0.4, 0.5]
            ...     ]
            ...     
            ...     for i, dataset in enumerate(datasets):
            ...         print(f"Processing dataset {i+1}:")
            ...         
            ...         # Reset filter for each new dataset
            ...         data_filter.reset()
            ...         
            ...         for sample in dataset:
            ...             filtered = data_filter.update(sample)
            ...             print(f"  {sample:4.1f} → {filtered:6.3f}")
            ...         
            ...         print(f"  Dataset complete: {data_filter.sample_count} samples\n")
        ```
        """
        super().reset()
        self.y = self._initial_value


class HighPassFilter(BaseFilter):
    """
    First-order high-pass filter with cutoff frequency specification.
    
    A digital high-pass filter that allows high-frequency components to pass through
    while attenuating low-frequency components below the cutoff frequency. This
    implementation uses a first-order IIR (Infinite Impulse Response) structure
    with exponential decay characteristics, making it ideal for DC removal,
    baseline correction, and extracting dynamic signal components.
    
    The filter implements the difference equation:
    
        y[n] = a·(y[n-1] + x[n] - x[n-1])
    
    Where the coefficient 'a' is automatically calculated from the cutoff and sampling frequencies:
    
        a = exp(-2π·fc/fs)
    
    Features:
    
        - Automatic coefficient calculation from frequency specifications
        - Standard first-order high-pass response (+20dB/decade rolloff)
        - Configurable cutoff frequency and sampling rate
        - Memory-efficient dual-delay implementation (input and output)
        - Real-time processing with minimal computational overhead
        - Numerical stability through proper coefficient bounds
        - DC blocking with zero steady-state response to constant inputs
    
    Applications:
    
        - DC offset removal from sensor signals
        - Baseline drift correction in biomedical signals
        - Edge detection in signal processing
        - Motion detection in accelerometer data
        - Audio coupling (removing DC bias)
        - Derivative-like signal enhancement
        - Trend removal from time series data
        - EMG/ECG baseline correction
        - Vibration monitoring (isolating dynamic components)
    
    Mathematical Properties:
    
        - 3dB cutoff at specified frequency fc
        - Phase lead increases with frequency
        - High-frequency gain: 0dB (unity)
        - Low-frequency attenuation: -20dB/decade
        - DC gain: -∞dB (complete DC rejection)
        - Stability: Always stable for 0 < a < 1
        - Group delay: varies with frequency
    
    Signal Characteristics:
    
        - Removes slow-varying components (trends, drift)
        - Emphasizes rapid changes and edges
        - Introduces phase lead at low frequencies
        - Preserves high-frequency content
        - Zero response to constant (DC) inputs
        - Differentiation-like behavior at low frequencies
    """
    
    def __init__(self, fc: float, fs: float, initial: float = 0.0) -> None:
        """
        Initialize high-pass filter with frequency specifications.
        
        Creates a first-order digital high-pass filter by calculating the appropriate
        filter coefficient from the specified cutoff and sampling frequencies.
        The filter removes low-frequency components while preserving high-frequency
        content, making it ideal for DC removal and baseline correction.
        
        :param fc: Cutoff frequency in Hz (3dB point, must be > 0 and < fs/2)
        :param fs: Sampling frequency in Hz (must be > 0 and > 2*fc)
        :param initial: Initial state value for both input and output delays (default: 0.0)
                       Should typically be 0.0 for high-pass filters to avoid startup transients
        
        :raises FilterConfigurationError: If frequencies are invalid or violate Nyquist criterion
        :raises TypeError: If parameters cannot be converted to float
        
        Filter Coefficient Calculation:
            The coefficient 'a' is calculated using the formula:
    
            a = exp(-2π·fc/fs)
            
            This ensures:
    
            - Correct 3dB cutoff frequency at fc
            - Proper high-pass frequency response characteristics
            - Numerical stability for all valid frequency ratios
            - Zero steady-state response to DC inputs
        
        Frequency Constraints:
    
            - fc must be positive (fc > 0)
            - fc must be less than Nyquist frequency (fc < fs/2)
            - fs must be positive (fs > 0)
            - Recommended: fc should be significantly smaller than fs for good performance
        
        Example
        -------
        ```python
            >>> # DC removal from sensor data
            >>> dc_filter = HighPassFilter(fc=0.1, fs=100.0, initial=0.0)
            >>> print(f"Filter coefficient a: {dc_filter.a:.6f}")
            >>> 
            >>> # Audio DC blocking
            >>> audio_filter = HighPassFilter(fc=20.0, fs=44100.0)
            >>> print(f"Cutoff: {audio_filter.fc} Hz, Sampling: {audio_filter.fs} Hz")
            >>> 
            >>> # Motion detection (remove gravity)
            >>> motion_filter = HighPassFilter(fc=0.5, fs=100.0, initial=0.0)
            >>> print(f"Initial states: y={motion_filter.y}, x_prev={motion_filter.x_prev}")
            >>> 
            >>> # Invalid configurations (will raise FilterConfigurationError)
            >>> try:
            ...     invalid_filter = HighPassFilter(fc=60.0, fs=100.0)  # fc >= fs/2
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> # Application-specific filter design
            >>> def design_application_filters():
            ...     '''Design filters for specific applications.'''
            ...     
            ...     # EMG baseline correction (remove drift < 5 Hz)
            ...     emg_filter = HighPassFilter(fc=5.0, fs=1000.0)
            ...     print(f"EMG filter: fc={emg_filter.fc} Hz, a={emg_filter.a:.4f}")
            ...     
            ...     # ECG baseline wander removal (remove < 0.5 Hz)
            ...     ecg_filter = HighPassFilter(fc=0.5, fs=250.0)
            ...     print(f"ECG filter: fc={ecg_filter.fc} Hz, a={ecg_filter.a:.4f}")
            ...     
            ...     # Accelerometer motion detection (remove gravity)
            ...     motion_filter = HighPassFilter(fc=0.1, fs=100.0)
            ...     print(f"Motion filter: fc={motion_filter.fc} Hz, a={motion_filter.a:.4f}")
            ...     
            ...     # Audio DC blocking (remove < 20 Hz)
            ...     audio_filter = HighPassFilter(fc=20.0, fs=44100.0)
            ...     print(f"Audio filter: fc={audio_filter.fc} Hz, a={audio_filter.a:.4f}")
        ```
        """
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
        """
        Update filter with new sample and return filtered output.
        
        Processes a single input sample through the first-order IIR high-pass filter
        using the difference equation: y[n] = a·(y[n-1] + x[n] - x[n-1])
        
        This method implements high-pass filtering by maintaining both input and output
        history. The filter emphasizes changes in the input signal while suppressing
        constant (DC) components.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Filtered output sample as float
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
            The filter implements: y[n] = a·(y[n-1] + x[n] - x[n-1])
            
            Where:
        
            - y[n] is the current output
            - y[n-1] is the previous output (stored in self.y)
            - x[n] is the current input
            - x[n-1] is the previous input (stored in self.x_prev)
            - a is the filter coefficient (self.a)
        
        Performance Characteristics:
        
            - O(1) computational complexity
            - Two multiply-add operations per sample
            - Minimal memory usage (two float values for state)
            - Optimized with @micropython.native decorator
            - Suitable for real-time applications
        
        Signal Processing Properties:
        
            - High-frequency gain: 0dB (unity gain at high frequencies)
            - Low-frequency attenuation: -20dB/decade below cutoff
            - DC gain: -∞dB (complete DC rejection)
            - Phase response: +90° at low frequencies, 0° at high frequencies
            - Steady-state response to constants: 0.0
        
        Example
        -------
        ```python
            >>> # Basic high-pass filtering
            >>> filter_obj = HighPassFilter(fc=1.0, fs=10.0)
            >>> 
            >>> # Test with DC + AC signal
            >>> test_signal = [5.0, 5.1, 5.0, 4.9, 5.0, 5.2, 5.0, 4.8]  # DC=5, AC varies
            >>> 
            >>> print("Input | Output | Description")
            >>> print("-" * 35)
            >>> 
            >>> for i, sample in enumerate(test_signal):
            ...     output = filter_obj.update(sample)
            ...     
            ...     if i == 0:
            ...         desc = "First sample"
            ...     elif sample == 5.0:
            ...         desc = "DC component"
            ...     else:
            ...         desc = f"AC: {sample-5.0:+.1f}"
            ...     
            ...     print(f"{sample:5.1f} | {output:+6.3f} | {desc}")
            >>> # Input | Output | Description
            >>> # -------------------------
            >>> #   5.0 | +0.000 | First sample
            >>> #   5.1 | +0.095 | AC: +0.1
            >>> #   5.0 | -0.005 | DC component
            >>> #   4.9 | -0.095 | AC: -0.1
            >>> #   5.0 | +0.005 | DC component
            >>> #   5.2 | +0.190 | AC: +0.2
            >>> #   5.0 | -0.010 | DC component
            >>> #   4.8 | -0.191 | AC: -0.2
            >>> 
            >>> # DC rejection demonstration
            >>> def demonstrate_dc_rejection():
            ...     dc_filter = HighPassFilter(fc=0.1, fs=10.0)
            ...     
            ...     print("DC Rejection Test:")
            ...     print("Sample | Input | Output | Running Avg")
            ...     print("-" * 40)
            ...     
            ...     dc_value = 3.0
            ...     running_sum = 0.0
            ...     
            ...     for n in range(50):
            ...         # Constant DC input
            ...         output = dc_filter.update(dc_value)
            ...         running_sum += output
            ...         running_avg = running_sum / (n + 1)
            ...         
            ...         if n % 10 == 0:  # Print every 10th sample
            ...             print(f"{n:6d} | {dc_value:5.1f} | {output:+6.4f} | {running_avg:+8.5f}")
            ...     
            ...     print(f"Final average output: {running_avg:+.6f} (should approach 0)")
            >>> 
            >>> # Edge detection example
            >>> def edge_detection_example():
            ...     edge_detector = HighPassFilter(fc=2.0, fs=20.0)
            ...     
            ...     # Create signal with step changes
            ...     signal = [1.0]*10 + [2.0]*10 + [1.5]*10 + [3.0]*10
            ...     
            ...     print("Edge Detection Example:")
            ...     print("Sample | Input | Edge Signal | Event")
            ...     print("-" * 45)
            ...     
            ...     threshold = 0.1
            ...     
            ...     for i, sample in enumerate(signal):
            ...         edge_signal = edge_detector.update(sample)
            ...         
            ...         # Detect edges
            ...         if abs(edge_signal) > threshold:
            ...             if edge_signal > 0:
            ...                 event = "Rising edge"
            ...             else:
            ...                 event = "Falling edge"
            ...         else:
            ...             event = "-"
            ...         
            ...         if i % 5 == 0 or event != "-":  # Print on events or periodically
            ...             print(f"{i:6d} | {sample:5.1f} | {edge_signal:+10.4f} | {event}")
        ```
        """
        self._sample_count += 1
        x = float(x)
        self.y = self.a * (self.y + x - self.x_prev)
        self.x_prev = x
        return self.y
    
    def reset(self) -> None:
        """
        Reset filter to initial state while preserving configuration.
        
        Clears the filter's internal state (both input and output history) and resets
        the sample counter, but maintains all configuration parameters like cutoff
        frequency, sampling frequency, and calculated coefficients. This allows the
        filter to be reused for new data streams without reconfiguration.
        
        Reset Operations:
        
            - Clears output history (y[n-1])
            - Clears input history (x[n-1])
            - Resets sample counter to zero
            - Preserves filter coefficients and parameters
            - Restores initial state values
            - Prepares filter for new input sequence
        
        State Variables Reset:
        
            - self.y (output history) → initial value
            - self.x_prev (input history) → initial value
            - self._sample_count → 0
            - Filter coefficients (a, fc, fs) preserved
        
        Example
        -------
        ```python
            >>> filter_obj = HighPassFilter(fc=1.0, fs=10.0, initial=0.0)
            >>> 
            >>> # Process some data with DC offset
            >>> test_data = [5.0, 5.1, 5.0, 4.9, 5.0]
            >>> for sample in test_data:
            ...     result = filter_obj.update(sample)
            ...     print(f"Sample: {sample:.1f} → Output: {result:+.4f}")
            >>> # Sample: 5.0 → Output: +0.0000
            >>> # Sample: 5.1 → Output: +0.0905
            >>> # Sample: 5.0 → Output: -0.0082
            >>> # Sample: 4.9 → Output: -0.0905
            >>> # Sample: 5.0 → Output: +0.0082
            >>> 
            >>> print(f"Before reset: samples={filter_obj.sample_count}, "
            ...       f"y={filter_obj.y:.4f}, x_prev={filter_obj.x_prev:.1f}")
            >>> # Before reset: samples=5, y=0.0082, x_prev=5.0
            >>> 
            >>> # Reset filter
            >>> filter_obj.reset()
            >>> print(f"After reset: samples={filter_obj.sample_count}, "
            ...       f"y={filter_obj.y:.4f}, x_prev={filter_obj.x_prev:.1f}")
            >>> # After reset: samples=0, y=0.0000, x_prev=0.0
            >>> 
            >>> # Verify coefficients preserved
            >>> print(f"Coefficients preserved: fc={filter_obj.fc} Hz, a={filter_obj.a:.6f}")
            >>> 
            >>> # Multi-channel processing with synchronized reset
            >>> def synchronized_multichannel():
            ...     # Create filters for 3-axis accelerometer
            ...     accel_x = HighPassFilter(fc=0.5, fs=100.0)
            ...     accel_y = HighPassFilter(fc=0.5, fs=100.0)
            ...     accel_z = HighPassFilter(fc=0.5, fs=100.0)
            ...     
            ...     # Process first batch of data
            ...     batch1 = [(1.0, 0.1, 9.8), (0.9, 0.2, 9.7), (1.1, 0.0, 9.9)]
            ...     
            ...     print("Batch 1 processing:")
            ...     for i, (x, y, z) in enumerate(batch1):
            ...         x_filt = accel_x.update(x)
            ...         y_filt = accel_y.update(y)
            ...         z_filt = accel_z.update(z)
            ...         print(f"Sample {i+1}: Gravity removed - X:{x_filt:.3f} Y:{y_filt:.3f} Z:{z_filt:.3f}")
            ...     
            ...     # Reset all filters simultaneously for new session
            ...     for filter_obj in [accel_x, accel_y, accel_z]:
            ...         filter_obj.reset()
            ...     
            ...     print("\nAfter reset - all filters ready for new data")
            ...     
            ...     # Process second batch with fresh state
            ...     batch2 = [(0.5, -0.1, 9.7), (0.6, -0.2, 9.8)]
            ...     
            ...     print("\nBatch 2 processing:")
            ...     for i, (x, y, z) in enumerate(batch2):
            ...         x_filt = accel_x.update(x)
            ...         y_filt = accel_y.update(y)
            ...         z_filt = accel_z.update(z)
            ...         print(f"Sample {i+1}: Gravity removed - X:{x_filt:.3f} Y:{y_filt:.3f} Z:{z_filt:.3f}")
        ```
        """
        super().reset()
        self.y = self._initial_value
        self.x_prev = self._initial_value

from math import exp, pi
import micropython

# 모든 주석과 docstirng은 영문으로 작성할 것!
class TauLowPass(BaseFilter):
    """
    Low-pass filter with time constant (tau) specification.
    """
    def __init__(self, tau_s: float, initial: float = 0.0, fs: float = None) -> None:
        """
        Initialize the TauLowPass filter.

        :param tau_s: Time constant τ [s], τ > 0.
        :param initial: Initial output value (starting state), default 0.0.
        :param fs: Fixed sampling frequency [Hz]. If provided, update() can be used.
            If not provided, only update_with_dt() can be used.
        """
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
        """
        Get the time constant τ [s].

        :return: Time constant τ [s].
        """
        return self._tau

    @tau.setter
    def tau(self, value: float) -> None:
        """
        Set the time constant τ [s].

        :param value: Time constant τ [s], τ > 0.
        """
        if value <= 0.0:
            raise ValueError("tau must be > 0")
        self._tau = float(value)
        if self.fs is not None:
            self._alpha_fixed = self._compute_alpha_fixed()

    def set_cutoff(self, fc_hz: float) -> None:
        """
        Set the cutoff frequency for the filter.

        :param fc_hz: Cutoff frequency [Hz], fc > 0.
        """
        if fc_hz <= 0.0:
            raise ValueError("fc must be > 0")
        self.tau = 1.0 / (2.0 * pi * float(fc_hz))

    def _compute_alpha_fixed(self) -> float:
        return 1.0 - exp(-(1.0 / self.fs) / self._tau)

    @micropython.native
    def update_with_dt(self, x: float, dt_s: float) -> float:
        """
        Update the filter output with new input and time delta.

        :param x: New input sample.
        :param dt_s: Time delta since last update [s].
        :return: Filtered output sample.
        """
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
        """
        Update the filter output with new input sample.

        :return: Filtered output sample.
        """
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
        """
        Reset the filter state to initial values.
        """
        self._sample_count = 0
        super().reset()
        self.y = self._initial_value


class SlewRateLimiter(BaseFilter):
    """
    Slew rate limiter filter with separate rise and fall rates.
    """
    
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
    """
    Moving average filter with efficient circular buffer implementation.
    
    A finite impulse response (FIR) filter that computes the arithmetic mean of
    the most recent N samples using a memory-efficient circular buffer. This
    filter provides excellent noise reduction while preserving signal trends,
    making it ideal for smoothing noisy sensor data and extracting signal baselines.
    
    The filter implements the moving average equation:
        
        y[n] = (1/N) * Σ(x[n-k]) for k = 0 to N-1
    
    Key Features:
        
        - O(1) computational complexity per sample (constant time updates)
        - Memory-efficient circular buffer implementation
        - Optimized with @micropython.viper decorator
        - Linear phase response (no phase distortion)
        - Excellent for noise reduction and baseline extraction
        - Configurable window size for different smoothing levels
    
    Performance Characteristics:
        
        - Zero phase delay for centered implementation
        - Group delay: (N-1)/2 samples
        - Frequency response: sinc function with nulls at fs/N, 2*fs/N, etc.
        - DC gain: 0dB (unity gain)
        - Noise reduction: ~10*log10(N) dB for white noise
    
    Applications:
        
        - Sensor data smoothing and noise reduction
        - Signal baseline estimation
        - Trend extraction from noisy measurements
        - Data preprocessing for control systems
        - Real-time signal conditioning
        - Peak detection preprocessing
    
    Mathematical Properties:
        
        - Linear phase response (symmetric impulse response)
        - First null at frequency fs/N
        - Stopband attenuation: -13.3dB at fs/N for rectangular window
        - Transition bandwidth inversely proportional to window size
        - Always stable (FIR filter)
    
    """
    
    def __init__(self, window_size: int, initial: float = 0.0) -> None:
        """
        Initialize moving average filter with specified window size.
        
        Creates a moving average filter using an efficient circular buffer
        implementation. The filter maintains a rolling average of the most
        recent window_size samples, providing smooth output with minimal
        computational overhead.
        
        :param window_size: Number of samples to include in moving average
                           Must be a positive integer. Larger values provide
                           more smoothing but increase delay.
        :param initial: Initial value for all buffer positions (default: 0.0)
                       Use expected signal level to minimize startup transients
        
        :raises FilterConfigurationError: If window_size is not a positive integer
        :raises TypeError: If initial cannot be converted to float
        
        Window Size Selection Guidelines:
            Small windows (3-10 samples):
        
                - Fast response to signal changes
                - Minimal delay (good for control systems)
                - Less noise reduction
                - Good for: Real-time control, peak detection
            
            Medium windows (10-50 samples):
        
                - Balanced smoothing and responsiveness
                - Moderate delay
                - Good noise reduction
                - Good for: General sensor smoothing, trend detection
            
            Large windows (50+ samples):
        
                - Heavy smoothing, excellent noise reduction
                - Significant delay (not suitable for fast control)
                - Stable baseline estimation
                - Good for: Baseline estimation, very noisy signals
        
        Memory Usage:
        
            - Buffer memory: window_size × 4 bytes (float array)
            - Additional overhead: ~32 bytes for object structure
            - Total memory: approximately (window_size × 4) + 32 bytes
        
        Example
        -------
        ```python
            >>> # Temperature sensor with moderate smoothing
            >>> temp_filter = MovingAverageFilter(window_size=10, initial=22.0)
            >>> print(f"Memory usage: ~{10*4 + 32} bytes")
            >>> 
            >>> # Pressure sensor with heavy smoothing
            >>> pressure_filter = MovingAverageFilter(window_size=100, initial=1013.25)
            >>> 
            >>> # Very responsive filter for control applications
            >>> control_filter = MovingAverageFilter(window_size=3, initial=0.0)
            >>> 
            >>> # Invalid configurations
            >>> try:
            ...     bad_filter = MovingAverageFilter(window_size=0)
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> # Performance consideration for large windows
            >>> def choose_window_size_for_application():
            ...     # For 1 kHz sampling rate
            ...     fs = 1000  # Hz
            ...     
            ...     # Different applications
            ...     applications = {
            ...         'Motor control': 5,      # 5ms delay, fast response
            ...         'Temperature': 60,       # 60ms delay, good smoothing
            ...         'Baseline estimation': 500  # 500ms delay, heavy smoothing
            ...     }
            ...     
            ...     for app, window_size in applications.items():
            ...         delay_ms = window_size / fs * 1000
            ...         memory_bytes = window_size * 4 + 32
            ...         print(f"{app}: {window_size} samples, {delay_ms:.1f}ms delay, {memory_bytes} bytes")
        ```
        """
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
        """
        Update filter with new sample and return moving average.
        
        Efficiently computes the moving average using a circular buffer
        implementation. This method updates the internal state and returns
        the current average of the most recent samples.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Current moving average as float
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
        
            1. Replace oldest sample in circular buffer with new sample
            2. Update running sum (subtract old value, add new value)
            3. Advance circular buffer index
            4. Return sum divided by number of samples
        
        Performance Characteristics:
        
            - O(1) time complexity (constant time per sample)
            - Single division operation per sample
            - Optimized with @micropython.viper decorator
            - Memory efficient (no data copying)
            - Suitable for high-frequency real-time processing
        
        Circular Buffer Behavior:
        
            - Initial phase: Buffer fills gradually (average of 1, 2, ..., N samples)
            - Steady state: Always averages exactly N most recent samples
            - No memory allocation during operation (fixed buffer size)
            - Automatic wraparound when buffer is full
        
        Example
        -------
        ```python
            >>> # Demonstrate circular buffer behavior
            >>> filter_obj = MovingAverageFilter(window_size=4, initial=0.0)
            >>> 
            >>> test_sequence = [1, 2, 3, 4, 5, 6, 7, 8]
            >>> 
            >>> print("Sample | Input | Buffer State | Average | Samples Used")
            >>> print("-" * 55)
            >>> 
            >>> for i, sample in enumerate(test_sequence):
            ...     avg = filter_obj.update(sample)
            ...     
            ...     # Show internal buffer state (for illustration)
            ...     buffer_contents = list(filter_obj._buf)
            ...     samples_used = min(i + 1, filter_obj.window_size)
            ...     
            ...     print(f"{i+1:6d} | {sample:5d} | {buffer_contents} | {avg:7.2f} | {samples_used:11d}")
            >>> # Sample | Input | Buffer State | Average | Samples Used
            >>> # -------------------------------------------------------
            >>> #      1 |     1 | [1.0, 0.0, 0.0, 0.0] |    1.00 |           1
            >>> #      2 |     2 | [1.0, 2.0, 0.0, 0.0] |    1.50 |           2
            >>> #      3 |     3 | [1.0, 2.0, 3.0, 0.0] |    2.00 |           3
            >>> #      4 |     4 | [1.0, 2.0, 3.0, 4.0] |    2.50 |           4
            >>> #      5 |     5 | [5.0, 2.0, 3.0, 4.0] |    3.50 |           4
            >>> #      6 |     6 | [5.0, 6.0, 3.0, 4.0] |    4.50 |           4
            >>> #      7 |     7 | [5.0, 6.0, 7.0, 4.0] |    5.50 |           4
            >>> #      8 |     8 | [5.0, 6.0, 7.0, 8.0] |    6.50 |           4
            >>> 
            >>> # Noise reduction measurement
            >>> def measure_noise_reduction():
            ...     import random
            ...     
            ...     # Generate noisy signal
            ...     true_signal = 5.0
            ...     noise_level = 0.5
            ...     window_sizes = [1, 5, 10, 20, 50]
            ...     
            ...     print("Noise Reduction Analysis:")
            ...     print("Window Size | Output Std Dev | Noise Reduction (dB)")
            ...     print("-" * 50)
            ...     
            ...     for window_size in window_sizes:
            ...         filter_obj = MovingAverageFilter(window_size=window_size, initial=true_signal)
            ...         
            ...         # Process 1000 noisy samples
            ...         outputs = []
            ...         for _ in range(1000):
            ...             noisy_sample = true_signal + random.uniform(-noise_level, noise_level)
            ...             filtered = filter_obj.update(noisy_sample)
            ...             outputs.append(filtered)
            ...         
            ...         # Calculate output noise level
            ...         mean_output = sum(outputs) / len(outputs)
            ...         variance = sum((x - mean_output)**2 for x in outputs) / len(outputs)
            ...         std_dev = variance ** 0.5
            ...         
            ...         # Theoretical noise reduction for white noise
            ...         theoretical_reduction_db = 10 * log10(window_size)
            ...         actual_reduction_db = 20 * log10(noise_level / std_dev) if std_dev > 0 else 60
            ...         
            ...         print(f"{window_size:11d} | {std_dev:14.4f} | {actual_reduction_db:17.1f}")
        ```
        """
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
        """
        Reset filter to initial state while preserving configuration.
        
        Clears all samples from the circular buffer and resets internal state
        variables, but maintains the window size and initial value settings.
        This allows the filter to be reused for new data streams without
        reconfiguration.
        
        Reset Operations:
        
            - Fills buffer with initial values
            - Resets sum to initial_value × window_size
            - Resets buffer index to 0
            - Resets sample count to 0
            - Preserves window_size and initial_value settings
        
        Use Cases:
        
            - Starting new measurement session
            - Switching between different signal sources
            - Clearing filter memory after transient events
            - Batch processing of multiple datasets
            - Removing effects of outliers or bad data
        
        Example
        -------
        ```python
            >>> filter_obj = MovingAverageFilter(window_size=5, initial=10.0)
            >>> 
            >>> # Process some data
            >>> test_data = [12, 14, 11, 13, 15, 16, 12, 14]
            >>> for sample in test_data:
            ...     result = filter_obj.update(sample)
            ...     print(f"Sample: {sample:2d} → Average: {result:5.2f}")
            >>> 
            >>> print(f"Before reset: {filter_obj.sample_count} samples processed")
            >>> print(f"Current average: {filter_obj._sum / filter_obj._count:.2f}")
            >>> 
            >>> # Reset filter
            >>> filter_obj.reset()
            >>> print(f"After reset: {filter_obj.sample_count} samples processed")
            >>> print(f"Buffer restored to initial value: {filter_obj._initial_value}")
            >>> 
            >>> # Filter ready for new data
            >>> first_new_result = filter_obj.update(20.0)
            >>> print(f"First new sample result: {first_new_result:.2f}")
            >>> 
            >>> # Error recovery with reset
            >>> def robust_data_processing():
            ...     signal_filter = MovingAverageFilter(window_size=10, initial=5.0)
            ...     error_count = 0
            ...     max_errors = 5
            ...     
            ...     while True:
            ...         try:
            ...             sensor_reading = read_sensor()
            ...             
            ...             # Validate reading
            ...             if not (0 <= sensor_reading <= 100):
            ...                 raise FilterConfigurationError(f"Sensor reading out of range: {sensor_reading}")
            ...             
            ...             # Process valid reading
            ...             filtered_value = signal_filter.update(sensor_reading)
            ...             error_count = 0  # Reset error count on success
            ...             
            ...             use_filtered_value(filtered_value)
            ...             
            ...         except Exception as e:
            ...             error_count += 1
            ...             print(f"Error {error_count}: {e}")
            ...             
            ...             if error_count >= max_errors:
            ...                 print("Too many errors, resetting filter")
            ...                 signal_filter.reset()
            ...                 error_count = 0
            ...         
            ...         utime.sleep_ms(100)
        ```
        """
        super().reset()
        for i in range(self._window_size):
            self._buf[i] = self._initial_value
        self._sum = self._initial_value * self._window_size
        self._idx = 0
        self._count = 0


class MedianFilter(BaseFilter):
    """
    Median filter for impulse noise removal and outlier rejection.
    
    A non-linear filter that computes the median value of the most recent N samples
    using a sliding window approach. This filter excels at removing impulse noise,
    spikes, and outliers while preserving signal edges and sharp transitions,
    making it ideal for robust signal conditioning in noisy environments.
    
    The filter implements the median operation:
    
        y[n] = median(x[n], x[n-1], ..., x[n-N+1])
    
    Key Features:
    
        - Excellent impulse noise and spike removal
        - Preserves edges and sharp signal transitions
        - Non-linear operation (order statistics)
        - Robust against outliers and artifacts
        - No overshoot or undershoot in step responses
        - Effective for salt-and-pepper type noise
    
    Performance Characteristics:
    
        - Computational complexity: O(N log N) per sample (sorting)
        - Memory usage: N samples in circular buffer
        - Group delay: (N-1)/2 samples
        - Non-linear phase response
        - DC preservation for constant signals
        - Edge-preserving properties
    
    Applications:
    
        - Sensor spike removal (ADC glitches, EMI)
        - Image processing (salt-and-pepper noise)
        - Biomedical signal preprocessing
        - Industrial measurement outlier rejection
        - Communication system impulse noise removal
        - Robust baseline estimation
    
    Mathematical Properties:
    
        - Output always within input range (no overshoot)
        - Preserves monotonic signal trends
        - Robust breakdown point: 50% for odd window sizes
        - Non-linear operation (not amenable to frequency analysis)
        - Idempotent: median(median(x)) = median(x)
    
    """
    
    def __init__(self, window_size: int, initial: float = 0.0) -> None:
        """
        Initialize median filter with specified window size.
        
        Creates a median filter using a circular buffer to maintain the most
        recent samples. The filter computes the median of these samples to
        provide robust outlier rejection and impulse noise removal.
        
        :param window_size: Number of samples in sliding window (positive integer)
                           Larger windows provide stronger outlier rejection but
                           increase delay and computational cost. Odd values
                           recommended for symmetric median operation.
        :param initial: Initial value for all buffer positions (default: 0.0)
                       Should be set to expected signal level to minimize
                       startup transients.
        
        :raises FilterConfigurationError: If window_size is not a positive integer
        :raises TypeError: If initial value cannot be converted to float
        
        Window Size Selection Guidelines:
            Small windows (3-5 samples):
    
                - Fast response, minimal delay
                - Light outlier rejection
                - Preserves rapid signal changes
                - Good for: Real-time control, low-latency applications
            
            Medium windows (5-15 samples):
    
                - Balanced performance
                - Good outlier rejection
                - Moderate delay
                - Good for: General sensor conditioning, measurement systems
            
            Large windows (15+ samples):
    
                - Strong outlier rejection
                - High delay (not suitable for fast control)
                - May blur rapid signal changes
                - Good for: Offline processing, heavy noise environments
        
        Computational Considerations:
    
            - CPU usage: O(N log N) per sample due to sorting
            - Memory usage: N × 4 bytes for buffer storage
            - Real-time suitability decreases with window size
            - Consider MovingAverageFilter for lower CPU usage
        
        Example
        -------
        ```python
            >>> # Light filtering for control applications
            >>> control_filter = MedianFilter(window_size=3, initial=0.0)
            >>> 
            >>> # Moderate filtering for sensor data
            >>> sensor_filter = MedianFilter(window_size=7, initial=25.0)
            >>> 
            >>> # Heavy filtering for noisy environments
            >>> noise_filter = MedianFilter(window_size=15, initial=0.0)
            >>> 
            >>> # Invalid configurations
            >>> try:
            ...     bad_filter = MedianFilter(window_size=0)
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> try:
            ...     bad_filter = MedianFilter(window_size=-3)
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> # Performance analysis for different window sizes
            >>> def analyze_performance():
            ...     import utime
            ...     
            ...     window_sizes = [3, 5, 7, 11, 15]
            ...     test_samples = 1000
            ...     
            ...     print("Performance Analysis:")
            ...     print("Window Size | Processing Time (ms) | CPU Usage")
            ...     print("-" * 50)
            ...     
            ...     for size in window_sizes:
            ...         filter_obj = MedianFilter(window_size=size)
            ...         
            ...         start_time = utime.ticks_ms()
            ...         for i in range(test_samples):
            ...             filter_obj.update(i * 0.1)
            ...         end_time = utime.ticks_ms()
            ...         
            ...         duration = utime.ticks_diff(end_time, start_time)
            ...         cpu_usage = "Low" if duration < 50 else "Medium" if duration < 200 else "High"
            ...         
            ...         print(f"{size:11d} | {duration:19d} | {cpu_usage}")
        ```
        """
        super().__init__()
        if not isinstance(window_size, int) or window_size <= 0:
            raise FilterConfigurationError("window_size must be positive int")

        self._window_size = window_size        
        self.reset()

    @micropython.native
    def update(self, x: float) -> float:
        """
        Update filter with new sample and return median value.
        
        Adds the new sample to the circular buffer and computes the median
        of all samples currently in the buffer. This provides robust
        outlier rejection while preserving signal characteristics.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Median of current window samples as float
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
        
            1. Add new sample to circular buffer (replacing oldest)
            2. Create sorted copy of active buffer contents
            3. Return middle value (or average of two middle values)
            4. Advance buffer index for next sample
        
        Performance Characteristics:
        
            - Time complexity: O(N log N) due to sorting operation
            - Space complexity: O(N) for buffer and temporary sort array
            - Optimized with @micropython.native decorator
            - CPU usage scales with window size
        
        Median Calculation:
        
            - Odd window size: Returns middle element of sorted array
            - Even window size: Returns average of two middle elements
            - Always returns value within input data range
            - Robust against outliers (up to 50% contamination)
        
        Example
        -------
        ```python
            >>> # Basic median filtering
            >>> filter_obj = MedianFilter(window_size=5, initial=0.0)
            >>> 
            >>> # Process sequence with outliers
            >>> test_sequence = [1.0, 1.1, 10.0, 1.2, 0.9, 1.0, 15.0, 1.1]
            >>> 
            >>> print("Sample | Input | Median | Buffer Contents")
            >>> print("-" * 45)
            >>> 
            >>> for i, sample in enumerate(test_sequence):
            ...     median_val = filter_obj.update(sample)
            ...     # Show buffer state (first few samples)
            ...     active_samples = min(i + 1, filter_obj.window_size)
            ...     buffer_contents = list(filter_obj._buffer[:active_samples])
            ...     
            ...     print(f"{i+1:6d} | {sample:5.1f} | {median_val:6.1f} | {buffer_contents}")
            >>> # Sample | Input | Median | Buffer Contents
            >>> # ---------------------------------------------
            >>> #      1 |   1.0 |    1.0 | [1.0]
            >>> #      2 |   1.1 |    1.0 | [1.0, 1.1]
            >>> #      3 |  10.0 |    1.1 | [1.0, 1.1, 10.0]
            >>> #      4 |   1.2 |    1.1 | [1.0, 1.1, 10.0, 1.2]
            >>> #      5 |   0.9 |    1.1 | [1.0, 1.1, 10.0, 1.2, 0.9]
            >>> #      6 |   1.0 |    1.0 | [1.0, 1.1, 1.0, 1.2, 0.9]
            >>> #      7 |  15.0 |    1.0 | [1.0, 1.1, 15.0, 1.2, 0.9]
            >>> #      8 |   1.1 |    1.1 | [1.0, 1.1, 15.0, 1.1, 0.9]
            >>> 
            >>> # Comparing median with average for outlier rejection
            >>> def compare_median_vs_average():
            ...     data = [5.0, 5.2, 4.8, 5.3, 20.0, 5.1, 4.9]
            ...     
            ...     # Calculate standard average
            ...     avg = sum(data) / len(data)
            ...     
            ...     # Calculate median
            ...     sorted_data = sorted(data)
            ...     if len(data) % 2 == 1:
            ...         median = sorted_data[len(data) // 2]
            ...     else:
            ...         median = (sorted_data[len(data) // 2 - 1] + sorted_data[len(data) // 2]) / 2
            ...     
            ...     print(f"Data: {data}")
            ...     print(f"Average: {avg:.2f}")
            ...     print(f"Median: {median:.2f}")
            ...     print(f"Outlier effect: {avg - median:.2f} units")
        ```
        """
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
        """
        Reset filter to initial state while preserving configuration.
        
        Clears all samples from the buffer and resets internal state variables,
        but maintains the window size and initial value settings. This allows
        the filter to be reused for new data streams without reconfiguration.
        
        Reset Operations:
        
            - Fills buffer with initial values
            - Resets buffer index to 0
            - Resets sample count to 0
            - Preserves window_size and initial_value settings
            - Prepares filter for new input sequence
        
        Use Cases:
        
            - Starting new measurement session
            - Switching between different signal sources
            - Clearing filter memory after data corruption
            - Batch processing of multiple datasets
            - Removing effects of previous outliers
        
        Example
        -------
        ```python
            >>> filter_obj = MedianFilter(window_size=5, initial=10.0)
            >>> 
            >>> # Process some data with outliers
            >>> test_data = [12, 11, 50, 13, 10, 100, 12, 11]
            >>> for sample in test_data:
            ...     result = filter_obj.update(sample)
            ...     print(f"Sample: {sample:3d} → Median: {result:5.1f}")
            >>> 
            >>> print(f"Before reset: {filter_obj.sample_count} samples processed")
            >>> print(f"Buffer contents: {list(filter_obj._buffer)}")
            >>> 
            >>> # Reset filter
            >>> filter_obj.reset()
            >>> print(f"After reset: {filter_obj.sample_count} samples processed")
            >>> print(f"Buffer restored: {list(filter_obj._buffer)}")
            >>> 
            >>> # Filter ready for new data
            >>> first_new_result = filter_obj.update(20.0)
            >>> print(f"First new sample: {first_new_result:.1f}")
            >>> 
            >>> # Segmented data processing
            >>> def process_segmented_data():
            ...     '''Process multiple data segments with clean filter state.'''
            ...     segments = [
            ...         [1.0, 1.2, 5.0, 1.1, 0.9],  # Normal data with outlier
            ...         [5.0, 5.2, 5.1, 20.0, 4.9],  # Different baseline with outlier
            ...         [10.0, 9.8, 10.2, 10.1, 9.9]  # Third segment
            ...     ]
            ...     
            ...     outlier_filter = MedianFilter(window_size=3, initial=0.0)
            ...     
            ...     for i, segment in enumerate(segments):
            ...         print(f"Processing segment {i+1}:")
            ...         
            ...         # Reset filter for clean start on each segment
            ...         outlier_filter.reset()
            ...         
            ...         # Process segment data
            ...         segment_results = []
            ...         for sample in segment:
            ...             clean_value = outlier_filter.update(sample)
            ...             segment_results.append(clean_value)
            ...         
            ...         # Report segment statistics
            ...         avg_raw = sum(segment) / len(segment)
            ...         avg_clean = sum(segment_results) / len(segment_results)
            ...         
            ...         print(f"  Raw data: {segment}")
            ...         print(f"  Filtered: {[f'{x:.1f}' for x in segment_results]}")
            ...         print(f"  Raw average: {avg_raw:.2f}, Clean average: {avg_clean:.2f}")
            ...         print()
        ```
        """
        super().reset()
        self._ring = array("f", [float(initial)] * self._window_size)
        self._sorted = [float(initial)] * self._window_size
        self._idx = 0
        self._count = 0


class RMSFilter(BaseFilter):
    """
    Root Mean Square (RMS) filter for signal power estimation and analysis.
    
    A specialized filter that computes the RMS (Root Mean Square) value of the most
    recent N samples using an efficient circular buffer implementation. This filter
    is essential for power analysis, signal level monitoring, and amplitude detection
    in real-time applications where understanding signal strength is critical.
    
    The filter implements the RMS equation:
        
        RMS[n] = sqrt((1/N) * Σ(x[n-k]²)) for k = 0 to N-1
    
    Key Features:
        
        - O(1) computational complexity per sample (constant time updates)
        - Efficient circular buffer with running sum of squares
        - Optimized with @micropython.viper decorator
        - Real-time power and amplitude monitoring
        - Suitable for audio, vibration, and signal analysis
        - Automatic numerical stability protection
    
    Performance Characteristics:
        
        - Time complexity: O(1) per sample (no sorting required)
        - Memory usage: N × 4 bytes for sample buffer
        - Group delay: (N-1)/2 samples
        - Always positive output (magnitude only)
        - Responsive to both positive and negative signal changes
        - Excellent for detecting signal presence and amplitude variations
    
    Applications:
        
        - Audio level monitoring and AGC (Automatic Gain Control)
        - Vibration analysis and machinery monitoring
        - Signal quality assessment and SNR estimation
        - Power consumption monitoring in RF systems
        - Biomedical signal amplitude detection (EMG, ECG)
        - Motor current monitoring and load detection
        - Environmental noise level measurement
        - Communication signal strength indication
    
    Mathematical Properties:
        
        - Output represents signal power (energy per sample)
        - Sensitive to outliers and spikes (squared terms amplify)
        - Linear response to signal amplitude changes
        - Zero output for zero-mean noise approaches sqrt(variance)
        - Suitable for detecting both AC and DC signal components
        - Natural integration with power-based signal processing
    
    """
    
    def __init__(self, window_size: int) -> None:
        """
        Initialize RMS filter with specified window size.
        
        Creates an RMS filter using an efficient circular buffer implementation
        that maintains a running sum of squares for constant-time RMS calculation.
        The filter provides real-time power estimation suitable for signal
        monitoring and analysis applications.
        
        :param window_size: Number of samples to include in RMS calculation
                           Must be a positive integer. Larger values provide
                           more stable readings but increase response delay.
        
        :raises FilterConfigurationError: If window_size is not a positive integer
        
        Window Size Selection Guidelines:
            Short windows (10-100 samples):
        
                - Fast response to amplitude changes
                - Good for transient detection
                - Higher variability in readings
                - Good for: Real-time level meters, AGC systems
            
            Medium windows (100-1000 samples):
        
                - Balanced stability and responsiveness
                - Suitable for most monitoring applications
                - Good for: Audio processing, vibration analysis
            
            Long windows (1000+ samples):
        
                - Very stable readings
                - Slow response to changes
                - Good for: Long-term trending, baseline estimation
        
        Memory and Performance:
        
            - Memory usage: window_size × 4 bytes (float array)
            - Computational cost: O(1) per sample
            - Suitable for high-frequency real-time processing
        
        Example
        -------
        ```python
            >>> # Audio level meter (fast response)
            >>> audio_meter = RMSFilter(window_size=256)  # ~6ms at 44.1kHz
            >>> print(f"Audio meter window: {audio_meter.window_size} samples")
            >>> 
            >>> # Vibration monitor (medium response)
            >>> vibration_monitor = RMSFilter(window_size=500)  # 5s at 100Hz
            >>> print(f"Vibration monitor delay: {500/100:.1f}s at 100Hz")
            >>> 
            >>> # Power trend analysis (slow response)
            >>> power_trend = RMSFilter(window_size=5000)  # 50s at 100Hz
            >>> print(f"Power trend memory usage: ~{5000*4} bytes")
            >>> 
            >>> # Invalid configurations
            >>> try:
            ...     bad_filter = RMSFilter(window_size=0)
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> # Memory consideration for embedded systems
            >>> def estimate_memory_requirements(sample_rate, response_time):
            ...     '''Calculate window size and memory needs for target response.'''
            ...     window_size = int(sample_rate * response_time)
            ...     memory_bytes = window_size * 4
            ...     
            ...     print(f"For {response_time:.1f}s response at {sample_rate}Hz:")
            ...     print(f"  Window size: {window_size} samples")
            ...     print(f"  Memory usage: {memory_bytes} bytes")
            ...     
            ...     return window_size
            >>> 
            >>> # Example calculations for different applications
            >>> audio_window = estimate_memory_requirements(44100, 0.1)  # 100ms audio
            >>> vibration_window = estimate_memory_requirements(100, 2.0)  # 2s vibration
        ```
        """
        super().__init__()
        
        if not isinstance(window_size, int) or window_size <= 0:
            raise FilterConfigurationError("window_size must be a positive integer")
        
        self._window_size = window_size
        self.reset()

    @micropython.native
    def update(self, x: float) -> float:
        """
        Update filter with new sample and return current RMS value.
        
        Efficiently computes the RMS value using a circular buffer and running
        sum of squares. This method provides constant-time RMS calculation
        regardless of window size, making it suitable for real-time applications.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Current RMS value as float (always non-negative)
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
        
            1. Square the new input sample
            2. Add new squared value to running sum
            3. Subtract old squared value from running sum
            4. Update circular buffer with new sample
            5. Return sqrt(sum_of_squares / sample_count)
        
        Performance Characteristics:
        
            - O(1) computational complexity per sample
            - Single square root operation per sample
            - Optimized with @micropython.viper decorator
            - Numerically stable with automatic bounds checking
            - Suitable for high-frequency processing
        
        RMS Properties:
        
            - Always returns non-negative values
            - Sensitive to signal amplitude (linear response)
            - Emphasizes outliers due to squaring operation
            - Provides power-based signal characterization
            - Zero output only for all-zero input window
        
        Example
        -------
        ```python
            >>> # Basic RMS calculation demonstration
            >>> rms_filter = RMSFilter(window_size=4)
            >>> 
            >>> # Test with sine wave samples
            >>> import math
            >>> test_samples = [math.sin(2 * math.pi * i / 8) for i in range(12)]
            >>> 
            >>> print("Sample | Input  | RMS    | Comment")
            >>> print("-" * 35)
            >>> 
            >>> # Process samples and show progression toward true RMS
            >>> for i, sample in enumerate(test_samples):
            ...     rms_value = rms_filter.update(sample)
            ...     # For sine wave, RMS approaches 1/sqrt(2) ≈ 0.707
            ...     comment = "Building" if i < 4 else "Steady state"
            ...     print(f"{i+1:6d} | {sample:+6.3f} | {rms_value:6.3f} | {comment}")
            >>> 
            >>> # Simple audio level calculation
            >>> def analyze_audio_levels():
            ...     '''Demonstrate RMS for audio level analysis.'''
            ...     # Create sine wave at different amplitudes
            ...     level_filter = RMSFilter(window_size=100)
            ...     
            ...     amplitudes = [0.1, 0.5, 1.0, 0.2]
            ...     sample_count = 200
            ...     
            ...     print("\nAudio Level Analysis:")
            ...     print("Amplitude | RMS Level | dB FS")
            ...     print("-" * 35)
            ...     
            ...     for amplitude in amplitudes:
            ...         # Generate sine wave at this amplitude
            ...         level_filter.reset()
            ...         
            ...         # Process full sine wave cycle
            ...         for i in range(sample_count):
            ...             angle = 2 * math.pi * i / 50  # 50 samples per cycle
            ...             sample = amplitude * math.sin(angle)
            ...             level = level_filter.update(sample)
            ...         
            ...         # Calculate dB relative to full scale
            ...         true_rms = amplitude / math.sqrt(2)
            ...         db_level = 20 * math.log10(level) if level > 0 else -100
            ...         
            ...         print(f"{amplitude:9.2f} | {level:9.4f} | {db_level:6.1f}")
        ```
        """
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
        """
        Reset filter to initial state while preserving configuration.
        
        Clears all samples from the circular buffer and resets internal state
        variables, but maintains the window size setting. This allows the filter
        to be reused for new data streams without reconfiguration.
        
        Reset Operations:
        
            - Clears all buffer samples to zero
            - Resets sum of squares to zero
            - Resets buffer index to zero
            - Resets sample count to zero
            - Preserves window_size setting
        
        Use Cases:
        
            - Starting new measurement session
            - Switching between different signal sources
            - Clearing filter memory after data corruption
            - Initializing for baseline measurements
            - Removing effects of previous transients
        
        Example
        -------
        ```python
            >>> # Demonstrate reset functionality
            >>> rms_filter = RMSFilter(window_size=100)
            >>> 
            >>> # Process some test data
            >>> test_data = [1.0, -2.0, 1.5, -1.8, 2.2, -1.2, 0.8]
            >>> for sample in test_data:
            ...     rms_value = rms_filter.update(sample)
            >>> 
            >>> # Check state before reset
            >>> print(f"Before reset: {rms_filter.sample_count} samples processed")
            >>> current_rms = sqrt(max(0.0, rms_filter._sum_of_squares / rms_filter._count))
            >>> print(f"Current RMS value: {current_rms:.4f}")
            >>> 
            >>> # Reset filter
            >>> rms_filter.reset()
            >>> print(f"After reset: {rms_filter.sample_count} samples processed")
            >>> print(f"Buffer sum of squares: {rms_filter._sum_of_squares:.4f}")
            >>> 
            >>> # Process new data after reset
            >>> rms_filter.update(5.0)
            >>> print(f"After first new sample: {rms_filter.sample_count} sample processed")
            >>> new_rms = sqrt(max(0.0, rms_filter._sum_of_squares / rms_filter._count))
            >>> print(f"New RMS value: {new_rms:.4f}")  # Should be 5.0
            >>> 
            >>> # Real-world Example: Calibration and monitoring sequence
            >>> def calibration_sequence():
            ...     '''Demonstrate reset in a calibration workflow.'''
            ...     # Create vibration monitor
            ...     vibration_filter = RMSFilter(window_size=200)
            ...     
            ...     # Step 1: Measure baseline noise floor
            ...     print("\nStep 1: Measuring system noise floor (motor off)")
            ...     # Simulate measuring ambient noise for 2 seconds
            ...     for _ in range(200):  # 200 samples
            ...         noise_sample = 0.05 * (random() - 0.5)  # Small random noise
            ...         vibration_filter.update(noise_sample)
            ...     
            ...     noise_floor = sqrt(vibration_filter._sum_of_squares / vibration_filter._count)
            ...     print(f"Noise floor: {noise_floor:.4f}g RMS")
            ...     
            ...     # Step 2: Reset and measure with motor running
            ...     vibration_filter.reset()
            ...     print("\nStep 2: Measuring normal operation (motor on)")
            ...     
            ...     # Simulate motor vibration for 2 seconds
            ...     for _ in range(200):  # 200 samples
            ...         # Vibration signal: 0.5g base + harmonics + noise
            ...         vibration = 0.5 * sin(2*pi*random()) + 0.05 * (random() - 0.5)
            ...         vibration_filter.update(vibration)
            ...     
            ...     normal_level = sqrt(vibration_filter._sum_of_squares / vibration_filter._count)
            ...     print(f"Normal operation: {normal_level:.4f}g RMS")
            ...     
            ...     # Step 3: Reset and measure with simulated fault
            ...     vibration_filter.reset()
            ...     print("\nStep 3: Measuring fault condition (bearing wear)")
            ...     
            ...     # Simulate faulty bearing for 2 seconds
            ...     for _ in range(200):  # 200 samples
            ...         # Fault adds high peaks and increased baseline
            ...         fault = 0.8 * sin(2*pi*random()) + 0.2 * sin(8*pi*random()) + 0.1 * random()
            ...         vibration_filter.update(fault)
            ...     
            ...     fault_level = sqrt(vibration_filter._sum_of_squares / vibration_filter._count)
            ...     print(f"Fault condition: {fault_level:.4f}g RMS")
            ...     
            ...     # Compare results
            ...     print("\nResults Summary:")
            ...     print(f"Signal-to-noise ratio: {20*log10(normal_level/noise_floor):.1f} dB")
            ...     print(f"Fault vs normal ratio: {fault_level/normal_level:.2f}x")
            ...     
            ...     # Make maintenance decision
            ...     if fault_level > 1.5 * normal_level:
            ...         print("ALERT: Maintenance required - abnormal vibration detected")
        ```
        """
        super().reset()
        self._buf = array("f", [0.0] * self._window_size)
        self._sum_of_squares = 0.0
        self._idx = 0
        self._count = 0


class KalmanFilter(BaseFilter):
    """
    One-dimensional Kalman filter for optimal state estimation.
    
    A recursive optimal estimator that produces statistically optimal estimates of
    noisy signals by combining measurements with predictions from a system model.
    This implementation uses a simplified scalar (1D) Kalman filter suitable for
    tracking a single variable in the presence of both measurement noise and
    process dynamics.
    
    The filter implements the standard Kalman filter equations simplified for
    the one-dimensional case:
        Prediction:
        
            x_pred = x
            p_pred = p + q
        
        Update:
        
            k = p_pred / (p_pred + r)
            x = x_pred + k * (z - x_pred)
            p = (1 - k) * p_pred
    
    Key Features:
        
        - Recursive Bayesian estimation for optimal tracking
        - Handles both process and measurement uncertainty
        - Automatic adaptation to noise characteristics
        - Outlier rejection with innovation-based gain adjustment
        - Numerically stable implementation with safeguards
        - Optimized with @micropython.viper decorator
        - Minimal memory footprint with efficient state representation
    
    Performance Characteristics:
        
        - O(1) computational complexity (constant time updates)
        - Optimal for Gaussian noise distributions
        - Automatic gain adjustment based on uncertainty
        - Graceful handling of temporary measurement failures
        - Self-stabilizing error covariance
        - Responsive to changing signal dynamics
    
    Applications:
        
        - Sensor fusion and data integration
        - Noisy sensor reading stabilization
        - Position and velocity estimation
        - Signal tracking with varying noise levels
        - Predictive filtering for control systems
        - Motion prediction and smoothing
        - Financial time series estimation
    
    Mathematical Properties:
        
        - Optimal for linear systems with Gaussian noise
        - Minimizes mean squared error
        - Combines prior knowledge with new measurements
        - Automatically balances smoothing vs. responsiveness
        - Uses uncertainty (covariance) for optimal weighting
        - Converges to steady-state behavior for constant systems
    
    """
    
    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1, 
                 initial_estimate: float = 0.0, initial_error: float = 1.0) -> None:
        """
        Initialize one-dimensional Kalman filter.
        
        Creates a Kalman filter for tracking a single variable with specified
        noise characteristics and initial state. The filter provides optimal
        estimation by balancing between system model predictions and measurements.
        
        :param process_noise: Process noise variance (q) - how quickly the true state can change
                             Higher values make filter more responsive to measurements
                             Lower values make filter rely more on its internal model
                             Must be non-negative (default: 0.01)
        :param measurement_noise: Measurement noise variance (r) - how noisy the measurements are
                                 Higher values make filter more skeptical of measurements
                                 Lower values make filter follow measurements more closely
                                 Must be positive (default: 0.1)
        :param initial_estimate: Initial state estimate (x) - starting value for the filter
                                (default: 0.0)
        :param initial_error: Initial error covariance (p) - starting uncertainty
                             Higher values make filter adapt more quickly at start
                             Lower values indicate confidence in initial estimate
                             Must be positive (default: 1.0)
        
        :raises FilterConfigurationError: If process_noise is negative or measurement_noise is not positive
        
        Parameter Selection Guidelines:
            Process Noise (q):
        
                - Low (0.0001-0.001): For very stable variables (temperature, pressure)
                - Medium (0.01-0.1): For moderately changing variables (position)
                - High (0.1-1.0): For rapidly changing variables (acceleration)
            
            Measurement Noise (r):
        
                - Low (0.01-0.1): For precise sensors (high-end IMUs, calibrated instruments)
                - Medium (0.1-1.0): For typical sensors (consumer-grade sensors)
                - High (1.0-10.0): For very noisy sensors (low-cost or compromised sensors)
            
            Initial Error (p):
        
                - Low (0.1): High confidence in initial estimate
                - Medium (1.0): Moderate confidence
                - High (10.0): Low confidence, adapt quickly
        
        Filter Behavior:
        
            - q/r ratio determines filter responsiveness vs. smoothness
            - q/r >> 1: Very responsive, follows measurements closely
            - q/r << 1: Very smooth, rejects measurement noise but responds slowly
        
        Example
        -------
        ```python
            >>> # Position tracking with moderate noise
            >>> position_filter = KalmanFilter(
            ...     process_noise=0.01,      # Position can change moderately
            ...     measurement_noise=0.5,   # Moderate sensor noise
            ...     initial_estimate=0.0,    # Start at origin
            ...     initial_error=1.0        # Moderate initial uncertainty
            ... )
            >>> 
            >>> # Temperature monitoring with high stability
            >>> temp_filter = KalmanFilter(
            ...     process_noise=0.0001,    # Temperature changes very slowly
            ...     measurement_noise=0.2,   # Moderate sensor noise
            ...     initial_estimate=22.0,   # Start at room temperature
            ...     initial_error=5.0        # High initial uncertainty
            ... )
            >>> 
            >>> # Parameter validation
            >>> try:
            ...     invalid_filter = KalmanFilter(process_noise=-0.5)  # Negative value
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
        ```
        """
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
        """
        Update filter with new measurement and return optimal estimate.
        
        Processes a single measurement through the Kalman filter using the
        prediction-update cycle. This method produces an optimal estimate
        that balances between the system model and the new measurement.
        
        :param measurement: New measurement value (any numeric type, converted to float)
        :return: Current optimal state estimate as float
        
        :raises TypeError: If measurement cannot be converted to float
        
        Algorithm Steps:
        
            1. Prediction step: Project state and error covariance forward
        
               - x_pred = x (assumes constant state)
               - p_pred = p + q (state uncertainty grows by process noise)
            
            2. Update step: Incorporate new measurement
        
               - Calculate Kalman gain: k = p_pred / (p_pred + r)
               - Update state: x = x_pred + k * (z - x_pred)
               - Update error covariance: p = (1 - k) * p_pred
            
            3. Outlier detection: Reduce gain for measurements far from prediction
        
               - If |z - x_pred| > 3*sqrt(p_pred), reduce Kalman gain by 90%
        
        Performance Characteristics:
        
            - O(1) computational complexity
            - Optimized with @micropython.viper decorator
            - Minimal memory usage (four float values for state)
            - Automatic stability protection for covariance
            - Outlier rejection for large innovations
        
        Example
        -------
        ```python
            >>> # Basic filtering demonstration
            >>> filter_obj = KalmanFilter(process_noise=0.01, measurement_noise=1.0)
            >>> 
            >>> # Process a series of noisy measurements
            >>> true_value = 10.0
            >>> noisy_measurements = [10.2, 9.7, 10.3, 9.9, 11.2, 9.8, 10.1, 9.6]
            >>> 
            >>> print("Measurement | Estimate | Error")
            >>> print("-" * 35)
            >>> 
            >>> for i, measurement in enumerate(noisy_measurements):
            ...     estimate = filter_obj.update(measurement)
            ...     error = abs(estimate - true_value)
            ...     
            ...     print(f"{measurement:11.2f} | {estimate:8.2f} | {error:5.2f}")
            >>> 
            >>> # Outlier rejection demonstration
            >>> def demonstrate_outlier_rejection():
            ...     '''Show how the filter handles outliers.'''
            ...     robust_filter = KalmanFilter(process_noise=0.01, measurement_noise=0.1)
            ...     
            ...     # Normal measurements with outliers
            ...     data = [5.1, 5.0, 5.2, 15.0, 5.1, 4.9, 5.0, -5.0, 5.2, 5.0]
            ...     
            ...     print("\nOutlier Rejection Test:")
            ...     print("Measurement | Estimate | Innovation | Kalman Gain")
            ...     print("-" * 55)
            ...     
            ...     for i, measurement in enumerate(data):
            ...         # Store state before update
            ...         x_prev = robust_filter.x
            ...         p_pred = robust_filter.p + robust_filter.q
            ...         
            ...         # Calculate theoretical Kalman gain
            ...         k_normal = p_pred / (p_pred + robust_filter.r)
            ...         
            ...         # Innovation (measurement residual)
            ...         innovation = measurement - x_prev
            ...         
            ...         # Update filter
            ...         estimate = robust_filter.update(measurement)
            ...         
            ...         # Calculate actual Kalman gain from results
            ...         if abs(innovation) > 0.001:  # Avoid division by zero
            ...             k_actual = (estimate - x_prev) / innovation
            ...         else:
            ...             k_actual = k_normal
            ...         
            ...         # Determine if this was detected as an outlier
            ...         is_outlier = abs(innovation) > 3.0 * sqrt(p_pred)
            ...         
            ...         print(f"{measurement:11.1f} | {estimate:8.2f} | {innovation:+10.2f} | {k_actual:.4f}" +
            ...               (" (outlier)" if is_outlier else ""))
        ```
        """
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
        """
        Reset filter to initial state while preserving configuration.
        
        Clears the filter's internal state (estimate and error covariance) and
        resets the sample counter, but maintains the noise parameters. This allows
        the filter to be reused for new data streams without reconfiguration.
        
        Reset Operations:
        
            - Restores state estimate to initial value
            - Restores error covariance to initial value
            - Resets sample counter to zero
            - Preserves process and measurement noise settings
            - Prepares filter for new input sequence
        
        Use Cases:
        
            - Starting new measurement session
            - Switching between different signal sources
            - Recovering from divergence or numerical issues
            - Batch processing of multiple datasets
            - Reinitializing after system mode changes
        
        Example
        -------
        ```python
            >>> # Demonstrate filter reset for multiple data sessions
            >>> filter_obj = KalmanFilter(
            ...     process_noise=0.01, 
            ...     measurement_noise=0.5,
            ...     initial_estimate=0.0,
            ...     initial_error=1.0
            ... )
            >>> 
            >>> # First data session (tracking position)
            >>> position_data = [0.1, 0.3, 0.6, 0.9, 1.2, 1.5, 1.7, 2.0]
            >>> position_estimates = []
            >>> 
            >>> for measurement in position_data:
            ...     estimate = filter_obj.update(measurement)
            ...     position_estimates.append(estimate)
            >>> 
            >>> print("Position tracking results:")
            >>> print(f"Final estimate: {filter_obj.x:.2f}")
            >>> print(f"Final uncertainty: {filter_obj.p:.4f}")
            >>> print(f"Samples processed: {filter_obj.sample_count}")
            >>> 
            >>> # Reset filter for new data session
            >>> filter_obj.reset()
            >>> print("\nAfter reset:")
            >>> print(f"State estimate: {filter_obj.x:.2f}")
            >>> print(f"Error covariance: {filter_obj.p:.4f}")
            >>> print(f"Samples processed: {filter_obj.sample_count}")
            >>> 
            >>> # Multiple sensor fusion with reset
            >>> def sensor_fusion_example():
            ...     '''Demonstrate switching between sensors with reset.'''
            ...     kalman = KalmanFilter(process_noise=0.01, measurement_noise=0.5)
            ...     
            ...     # Simulate primary sensor data (accurate but occasional dropouts)
            ...     primary_data = [10.1, 10.2, None, 10.0, 9.9, None, None, 10.1]
            ...     
            ...     # Simulate backup sensor data (less accurate but always available)
            ...     backup_data = [10.5, 9.8, 10.4, 9.7, 9.5, 10.3, 9.6, 10.2]
            ...     
            ...     print("\nSensor Fusion with Primary/Backup:")
            ...     print("Sample | Primary | Backup | Selected | Estimate")
            ...     print("-" * 55)
            ...     
            ...     for i, (primary, backup) in enumerate(zip(primary_data, backup_data)):
            ...         if primary is not None:
            ...             # Use primary sensor with regular noise model
            ...             kalman.r = 0.5  # Lower noise for primary sensor
            ...             selected = primary
            ...             sensor = "Primary"
            ...         else:
            ...             # Primary sensor dropout, switch to backup
            ...             kalman.r = 2.0  # Higher noise for backup sensor
            ...             selected = backup
            ...             sensor = "Backup"
            ...         
            ...         estimate = kalman.update(selected)
            ...         
            ...         primary_val = f"{primary:.1f}" if primary is not None else "---"
            ...         print(f"{i:6d} | {primary_val:7s} | {backup:6.1f} | {sensor:8s} | {estimate:8.2f}")
        ```
        """
        super().reset()
        self.x = self._initial_estimate
        self.p = self._initial_error


class AdaptiveFilter(BaseFilter):
    """
    Adaptive alpha filter that automatically adjusts response based on signal dynamics.
    
    A smart first-order IIR filter that dynamically changes its alpha coefficient
    based on detected signal changes. The filter transitions between slow response
    (for steady signals) and fast response (for rapidly changing signals) by monitoring
    signal deltas between consecutive samples and comparing against a threshold.
    
    The filter implements a variable alpha smoothing equation:
        
        y[n] = α(n)·x[n] + (1-α(n))·y[n-1]
    
    Where α(n) is dynamically calculated based on input signal changes:
        
        α(n) = α_min + ratio·(α_max - α_min)
        ratio = min(|x[n] - x[n-1]| / threshold, 1.0)
    
    Key Features:
        
        - Self-adjusting smoothing coefficient for optimal response
        - Balances noise rejection and transient response automatically
        - Configurable sensitivity through threshold parameter
        - Customizable response range via alpha_min and alpha_max
        - Memory-efficient implementation with minimal state variables
        - Adaptive to changing signal characteristics in real-time
        - No prior signal statistics required for operation
    
    Performance Characteristics:
        
        - O(1) computational complexity per sample
        - Preserves sharp transitions while smoothing steady regions
        - Fast convergence on signal changes
        - Gradual smoothing on stable signals
        - Continuous adaptation without mode switching
        - No overshoot during transition between coefficients
    
    Applications:
        
        - Sensor fusion with varying noise conditions
        - Motion tracking with rapid direction changes
        - User interface input smoothing
        - Biomedical signal processing
        - Embedded systems with limited processing power
        - Signal conditioning for event detection
        - Mixed steady-state and transient monitoring
    
    Mathematical Properties:
        
        - Time constant varies with signal dynamics
        - Automatic adjustment between first-order response modes
        - Continuous operation without discontinuities
        - Self-stabilizing for both steady and changing signals
        - Noise rejection proportional to signal stability
        - Step response speed proportional to step magnitude
    
    """
    
    def __init__(self, alpha_min: float = 0.01, alpha_max: float = 0.9, 
                 threshold: float = 0.1, initial: float = 0.0) -> None:
        """
        Initialize adaptive filter with customizable response range.
        
        Creates an adaptive filter that automatically adjusts its smoothing coefficient
        based on detected signal changes. The filter provides variable smoothing that
        adapts to both steady-state and transient signal conditions.
        
        :param alpha_min: Minimum alpha value for steady signals (0 < alpha_min < 1)
                         Lower values provide stronger smoothing for stable signals
                         (default: 0.01)
        :param alpha_max: Maximum alpha value for rapidly changing signals (0 < alpha_max ≤ 1)
                         Higher values provide faster response to signal changes
                         (default: 0.9)
        :param threshold: Signal change threshold that triggers maximum response
                         Changes larger than this trigger alpha_max response
                         Changes smaller than this scale alpha proportionally
                         Must be non-negative (default: 0.1)
        :param initial: Initial output value for filter state (default: 0.0)
                       Used to minimize startup transients when expected signal 
                       level is known
        
        :raises FilterConfigurationError: If alpha_min, alpha_max are outside valid ranges or if
                           alpha_min ≥ alpha_max, or if threshold is negative
        
        Parameter Selection Guidelines:
        
            alpha_min:
        
                - Very low (0.001-0.01): Heavy smoothing for steady signals
                - Low (0.01-0.05): Moderate smoothing
                - Medium (0.05-0.2): Light smoothing for noisy environments
            
            alpha_max:
        
                - Medium (0.5-0.7): Balanced transient response
                - High (0.7-0.9): Fast transient response
                - Maximum (1.0): Immediate response to changes above threshold
            
            threshold:
        
                - Low (< 0.1): Very sensitive to small changes
                - Medium (0.1-1.0): Moderate sensitivity
                - High (> 1.0): Only responds to large signal transitions
        
        Example
        -------
        ```python
            >>> # Create adaptive filter for sensor fusion
            >>> sensor_filter = AdaptiveFilter(
            ...     alpha_min=0.02,   # Strong smoothing for steady signals
            ...     alpha_max=0.8,    # Fast response to changes
            ...     threshold=0.5,    # Transition threshold
            ...     initial=0.0       # Start at zero
            ... )
            >>> 
            >>> # Parameter validation examples
            >>> try:
            ...     # Invalid: alpha_min must be > 0
            ...     invalid_filter = AdaptiveFilter(alpha_min=0, alpha_max=0.9)
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
            >>> 
            >>> try:
            ...     # Invalid: alpha_min must be < alpha_max
            ...     invalid_filter = AdaptiveFilter(alpha_min=0.5, alpha_max=0.3)
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
        ```
        """
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
        """
        Update filter with new sample using adaptive coefficient.
        
        Processes a single input sample through the adaptive filter, dynamically
        adjusting the alpha coefficient based on detected signal changes. This
        provides optimal smoothing that adapts to signal dynamics.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Filtered output sample as float
        
        :raises TypeError: If input cannot be converted to float
        
        Adaptation Algorithm:
        
            1. Calculate absolute change between current and previous input
            2. If change > threshold: Use alpha_max (fast response)
            3. If change ≤ threshold: Scale alpha proportionally between
               alpha_min and alpha_max
            4. Apply standard first-order filter with calculated alpha
            5. Store current input for next comparison
        
        Performance Characteristics:
        
            - O(1) computational complexity per sample
            - Adaptive smoothing with no parameter tuning needed
            - Optimized with @micropython.native decorator
            - Suitable for real-time applications
        
        Example
        -------
        ```python
            >>> # Basic adaptive filtering demonstration
            >>> filter_obj = AdaptiveFilter(alpha_min=0.1, alpha_max=0.9, threshold=0.5)
            >>> 
            >>> # Test with steady signal followed by step change
            >>> test_sequence = [5.0, 5.1, 5.0, 5.2, 5.0, 8.0, 8.1, 8.0, 7.9]
            >>> 
            >>> print("Input | Output | Change | Alpha")
            >>> print("-" * 40)
            >>> 
            >>> for i, sample in enumerate(test_sequence):
            ...     # Store previous output for comparison
            ...     prev_output = filter_obj.y if i > 0 else 0
            ...     
            ...     # Calculate change for alpha selection
            ...     change = abs(sample - filter_obj.prev_x) if i > 0 else 0
            ...     
            ...     # Calculate what alpha will be used
            ...     if change > filter_obj.threshold:
            ...         alpha = filter_obj.alpha_max
            ...     else:
            ...         ratio = change / filter_obj.threshold
            ...         alpha = filter_obj.alpha_min + ratio * (filter_obj.alpha_max - filter_obj.alpha_min)
            ...     
            ...     # Update filter
            ...     output = filter_obj.update(sample)
            ...     
            ...     print(f"{sample:5.1f} | {output:6.3f} | {change:6.3f} | {alpha:.3f}")
        ```
        """
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
        """
        Reset filter to initial state while preserving configuration.
        
        Clears the filter's internal state (output and previous input) and resets
        the sample counter, but maintains all configuration parameters. This allows
        the filter to be reused for new data streams without reconfiguration.
        
        Reset Operations:
        
            - Restores output value to initial value
            - Restores previous input to initial value
            - Resets sample counter to zero
            - Preserves alpha_min, alpha_max, and threshold parameters
            - Prepares filter for new input sequence
        
        Use Cases:
        
            - Starting new measurement session
            - Switching between different signal sources
            - Recovering from signal glitches or dropouts
            - Batch processing of multiple datasets
        
        Example
        -------
        ```python
            >>> # Demonstrate filter reset
            >>> filter_obj = AdaptiveFilter(alpha_min=0.1, alpha_max=0.9, threshold=0.5, initial=1.0)
            >>> 
            >>> # Process some data
            >>> for sample in [1.2, 1.5, 3.0, 3.2, 3.1]:
            ...     result = filter_obj.update(sample)
            >>> 
            >>> print(f"Before reset: output={filter_obj.y:.2f}, prev_x={filter_obj.prev_x:.2f}")
            >>> 
            >>> # Reset filter
            >>> filter_obj.reset()
            >>> print(f"After reset: output={filter_obj.y:.2f}, prev_x={filter_obj.prev_x:.2f}")
            >>> 
            >>> # Process new data with reset filter
            >>> new_result = filter_obj.update(10.0)
            >>> print(f"First new result: {new_result:.2f}")
        ```
        """
        super().reset()
        self.y = self._initial_value
        self.prev_x = self._initial_value


class BiquadFilter(BaseFilter):
    """
    Generic biquad (second-order IIR) filter with configurable coefficients.
    
    A versatile second-order Infinite Impulse Response (IIR) filter that directly
    implements the biquadratic transfer function with configurable coefficients.
    This filter provides a flexible building block for creating various filter types
    including lowpass, highpass, bandpass, notch, and peaking filters, offering
    superior frequency response control compared to first-order filters.
    
    The filter implements the difference equation:
        
        y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
    
    Where:
        
        - b0, b1, b2: Feedforward (numerator) coefficients
        - a1, a2: Feedback (denominator) coefficients
        - x[n]: Current and previous input samples
        - y[n]: Current and previous output samples
    
    Key Features:
        
        - Flexible second-order transfer function implementation
        - Direct coefficient specification for custom responses
        - Memory-efficient four-delay implementation
        - Configurable for all standard filter types
        - -40dB/decade roll-off for lowpass/highpass configurations
        - Optimized with @micropython.viper decorator
        - High precision with minimal computational overhead
    
    Performance Characteristics:
        
        - O(1) computational complexity (constant time updates)
        - Steeper roll-off than first-order filters
        - Better frequency selectivity than first-order filters
        - Capability for resonance and peaked responses
        - Efficient implementation for embedded systems
        - Low memory footprint (6 float values)
    
    Applications:
        
        - Audio equalizers and crossovers
        - Precise sensor data filtering
        - Scientific signal analysis
        - Anti-aliasing and reconstruction filters
        - Biomedical signal processing
        - Frequency-selective noise reduction
        - Communication systems filtering
        - Resonator and oscillator implementation
    
    Mathematical Properties:
        
        - Second-order transfer function: H(z) = (b0 + b1·z⁻¹ + b2·z⁻²)/(1 + a1·z⁻¹ + a2·z⁻²)
        - Two poles and two zeros in z-plane
        - Roll-off: -40dB/decade for lowpass/highpass
        - Q factor controls resonance/bandwidth
        - Stability requires poles inside unit circle (care needed with coefficients)
        - Amplitude and phase can be independently controlled
        - Can implement complex-conjugate pole pairs
    """
    
    def __init__(self, b0: float, b1: float, b2: float, a1: float, a2: float) -> None:
        """
        Initialize biquad filter with direct coefficient specification.
        
        Creates a second-order IIR filter with explicitly specified coefficients,
        providing complete control over the filter's transfer function. The 
        coefficients directly determine the filter's frequency response, stability,
        and time-domain characteristics.
        
        :param b0: Feedforward coefficient for current input x[n]
        :param b1: Feedforward coefficient for first delayed input x[n-1]
        :param b2: Feedforward coefficient for second delayed input x[n-2]
        :param a1: Feedback coefficient for first delayed output y[n-1] (negative in equation)
        :param a2: Feedback coefficient for second delayed output y[n-2] (negative in equation)
        
        :raises TypeError: If coefficients cannot be converted to float
        
        Coefficient Guidelines:
        
            - Stability requires: |a2| < 1, |a1| < 1+a2
            - Unity DC gain (lowpass): b0 + b1 + b2 = 1 + a1 + a2
            - Unity Nyquist gain (highpass): b0 - b1 + b2 = 1 - a1 + a2
            - For coefficient design, consider using derived classes like 
              ButterworthFilter or NotchFilter instead of manual calculation
        
        Transfer Function:
        
            H(z) = (b0 + b1·z⁻¹ + b2·z⁻²)/(1 + a1·z⁻¹ + a2·z⁻²)
        
        Example
        -------
        ```python
            >>> # Create a simple lowpass biquad filter (manually calculated coefficients)
            >>> # Cutoff: 100Hz, Sampling: 1000Hz, Q: 0.7071
            >>> lowpass = BiquadFilter(
            ...     b0=0.0718, b1=0.1436, b2=0.0718, 
            ...     a1=-1.1430, a2=0.4304
            ... )
            >>> 
            >>> # Create a notch filter (manually calculated coefficients)
            >>> # Notch at 60Hz, Sampling: 1000Hz, Q: 10
            >>> notch = BiquadFilter(
            ...     b0=0.9889, b1=-1.9558, b2=0.9889,
            ...     a1=-1.9558, a2=0.9778
            ... )
            >>> 
            >>> # Coefficient validation (stability check)
            >>> def check_stability(a1, a2):
            ...     '''Check if filter is stable based on feedback coefficients.'''
            ...     if abs(a2) >= 1:
            ...         return False  # Unstable
            ...     if abs(a1) >= 1 + a2:
            ...         return False  # Unstable
            ...     return True
            >>> 
            >>> # Check our filter stability
            >>> check_stability(-1.1430, 0.4304)  # Should be True
        ```
        """
        super().__init__()
        self.set_coefficients(b0, b1, b2, a1, a2)
        self.reset()

    def set_coefficients(self, b0: float, b1: float, b2: float, a1: float, a2: float) -> None:
        """
        Set filter coefficients to new values.
        
        Updates the biquad filter coefficients without needing to create a new filter
        instance. This allows dynamic modification of the filter's frequency response
        characteristics during operation.
        
        :param b0: Feedforward coefficient for current input x[n]
        :param b1: Feedforward coefficient for first delayed input x[n-1]
        :param b2: Feedforward coefficient for second delayed input x[n-2]
        :param a1: Feedback coefficient for first delayed output y[n-1] (negative in equation)
        :param a2: Feedback coefficient for second delayed output y[n-2] (negative in equation)
        
        :raises TypeError: If coefficients cannot be converted to float
        
        Dynamic Configuration Applications:
        
            - Real-time filter tuning for adaptive filtering
            - Parameter sweeping for filter design
            - Frequency response morphing between different filter types
            - User-adjustable filter characteristics
        
        Example
        -------
        ```python
            >>> # Create a filter and then modify it
            >>> filter_obj = BiquadFilter(1.0, 0.0, 0.0, 0.0, 0.0)  # Initially a pass-through
            >>> 
            >>> # Convert to lowpass filter (manually calculated coefficients)
            >>> filter_obj.set_coefficients(
            ...     b0=0.0718, b1=0.1436, b2=0.0718, 
            ...     a1=-1.1430, a2=0.4304
            ... )
            >>> 
            >>> # Dynamic frequency adjustment
            >>> def change_filter_frequency(fc, fs, Q=0.7071):
            ...     '''Calculate and set coefficients for a lowpass filter.'''
            ...     w0 = 2 * pi * fc / fs
            ...     alpha = sin(w0) / (2 * Q)
            ...     cosw0 = cos(w0)
            ...     
            ...     # Calculate normalized coefficients
            ...     b0 = (1 - cosw0) / 2
            ...     b1 = 1 - cosw0
            ...     b2 = (1 - cosw0) / 2
            ...     a0 = 1 + alpha
            ...     a1 = -2 * cosw0
            ...     a2 = 1 - alpha
            ...     
            ...     # Normalize by a0
            ...     filter_obj.set_coefficients(
            ...         b0/a0, b1/a0, b2/a0, 
            ...         a1/a0, a2/a0
            ...     )
            ...     
            ...     print(f"Updated to {fc}Hz lowpass filter")
            >>> 
            >>> # Change filter cutoff frequency
            >>> change_filter_frequency(fc=200, fs=1000)  # 200Hz cutoff
            >>> change_filter_frequency(fc=500, fs=1000)  # 500Hz cutoff
        ```
        """
        self.b0 = float(b0); self.b1 = float(b1); self.b2 = float(b2)
        self.a1 = float(a1); self.a2 = float(a2)

    @micropython.native
    def update(self, x: float) -> float:
        """
        Update filter with new sample and return filtered output.
        
        Processes a single input sample through the second-order IIR biquad filter
        using the direct form I structure. This method implements the difference equation:
        y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Filtered output sample as float
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
        
            - Direct Form I implementation (separate input and output delays)
            - Updates internal state with new input and output values
            - Maintains two input (x1, x2) and two output (y1, y2) delay elements
            - Optimized with @micropython.viper decorator for maximum performance
        
        Performance Characteristics:
        
            - O(1) computational complexity
            - Five multiply-add operations per sample
            - Minimal memory usage (four float values for delay elements)
            - Suitable for real-time processing of high-frequency signals
        
        Example
        -------
        ```python
            >>> # Basic biquad filtering operation
            >>> # Create a simple lowpass filter (fc=0.1*fs)
            >>> filter_obj = BiquadFilter(
            ...     b0=0.0201, b1=0.0402, b2=0.0201,
            ...     a1=-1.5610, a2=0.6414
            ... )
            >>> 
            >>> # Process a simple step input
            >>> output_sequence = []
            >>> for i in range(10):
            ...     # Input is 0 for i<5, then jumps to 1.0
            ...     input_value = 0.0 if i < 5 else 1.0
            ...     output = filter_obj.update(input_value)
            ...     output_sequence.append(output)
            ...     print(f"Sample {i}: Input={input_value:.1f}, Output={output:.6f}")
            >>> 
            >>> # Verify the filter's step response is as expected:
            >>> # - Initially zero
            >>> # - After step, gradually approaches 1.0
            >>> # - Smooth transition without excessive ringing
            >>> print(f"Final output: {output_sequence[-1]:.6f}")
        ```
        """
        self._sample_count += 1
        x = float(x)
        # DF-II Transposed states
        w0 = x - self.a1*self.z1 - self.a2*self.z2
        y  = self.b0*w0 + self.b1*self.z1 + self.b2*self.z2
        self.z2 = self.z1
        self.z1 = w0
        return y

    def reset(self) -> None:
        """
        Reset filter to initial state while preserving coefficients.
        
        Clears the filter's internal delay line (both input and output history)
        and resets the sample counter, but maintains all coefficient settings.
        This allows the filter to be reused for new data streams without
        reconfiguration.
        
        Reset Operations:
        
            - Clears input history (x1, x2) to zero
            - Clears output history (y1, y2) to zero
            - Resets sample counter to zero
            - Preserves all filter coefficients (b0, b1, b2, a1, a2)
        
        Use Cases:
        
            - Starting new filtering session
            - Switching between different signal sources
            - Clearing filter memory after transient events
            - Preventing startup transients when processing new signals
            - Batch processing of multiple datasets
        
        Example
        -------
        ```python
            >>> # Demonstrate filter reset for batch processing
            >>> filter_obj = BiquadFilter(
            ...     b0=0.0201, b1=0.0402, b2=0.0201,
            ...     a1=-1.5610, a2=0.6414
            ... )
            >>> 
            >>> # Process first batch (step response)
            >>> for i in range(10):
            ...     input_value = 0.0 if i < 5 else 1.0
            ...     output = filter_obj.update(input_value)
            >>> 
            >>> print(f"Before reset: Output={filter_obj.y1:.4f}, Count={filter_obj.sample_count}")
            >>> 
            >>> # Reset filter for second batch
            >>> filter_obj.reset()
            >>> print(f"After reset: Output={filter_obj.y1:.4f}, Count={filter_obj.sample_count}")
            >>> 
            >>> # Process second batch (sine wave)
            >>> import math
            >>> for i in range(10):
            ...     # Create sine wave input
            ...     input_value = math.sin(2 * math.pi * i / 10)
            ...     output = filter_obj.update(input_value)
            ...     print(f"Sample {i}: Input={input_value:.4f}, Output={output:.4f}")
            >>> 
            >>> # Verify filter starts with fresh state for each batch
            >>> print(f"Final count: {filter_obj.sample_count}")  # Should be 10
        ```
        """
        super().reset()
        self.z1 = 0.0
        self.z2 = 0.0


class ButterworthFilter(BiquadFilter):
    """
    Second-order Butterworth filter with maximally flat frequency response.
    
    A specialized biquad filter implementation that provides a Butterworth response
    characteristic, known for its maximally flat magnitude response in the passband.
    This filter offers an optimal balance between time and frequency domain performance,
    with no ripple in the passband and a -40dB/decade (12dB/octave) roll-off in the
    stopband, making it a popular choice for general filtering applications.
    
    The filter supports both lowpass and highpass configurations with automatic
    coefficient calculation from basic frequency specifications, eliminating the need
    for manual coefficient design.
    
    Key Features:
        
        - Maximally flat magnitude response in passband (no ripple)
        - Smooth rolloff transition (-40dB/decade)
        - Automatic coefficient calculation from frequency specifications
        - Both lowpass and highpass configurations
        - Excellent general-purpose filter characteristics
        - Inherited efficiency from BiquadFilter implementation
        - Optimal phase response for given magnitude constraint
    
    Performance Characteristics:
        
        - Standard second-order response (-40dB/decade)
        - Slightly less steep rolloff than Chebyshev filters
        - Slightly better stopband attenuation than Bessel filters
        - Moderate group delay variation near cutoff
        - No passband ripple (unlike Chebyshev filters)
        - Phase response optimized for given constraints
    
    Applications:
        
        - General-purpose noise filtering
        - Anti-aliasing before sampling
        - Audio processing and crossovers
        - Biomedical signal preprocessing
        - Sensor data conditioning
        - Baseline removal (highpass mode)
        - Communication systems
        - Smooth signal reconstruction
    
    Mathematical Properties:
        
        - Magnitude squared response: |H(ω)|² = 1/(1 + (ω/ωc)²ⁿ)
          where n is the filter order (2 for this implementation)
        - Maximally flat at ω = 0 (all derivatives zero)
        - No ripple in passband or stopband
        - -3dB attenuation exactly at the cutoff frequency
        - Monotonically decreasing magnitude response
        - Optimal phase response for given magnitude constraint
   
    """
    
    def __init__(self, fc: float, fs: float, filter_type: str = 'lowpass') -> None:
        """
        Initialize second-order Butterworth filter with frequency specifications.
        
        Creates a Butterworth filter with automatic coefficient calculation from
        cutoff frequency, sampling frequency, and filter type. The coefficients are
        calculated to provide the characteristic maximally flat response.
        
        :param fc: Cutoff frequency in Hz (3dB point, must be > 0 and < fs/2)
        :param fs: Sampling frequency in Hz (must be > 0 and > 2*fc)
        :param filter_type: Filter mode - 'lowpass' or 'highpass' (default: 'lowpass')
                          'lowpass': Passes frequencies below fc, attenuates above
                          'highpass': Passes frequencies above fc, attenuates below
        
        :raises FilterConfigurationError: If frequencies are invalid, violate Nyquist criterion,
                          or if filter_type is not recognized
        :raises TypeError: If parameters cannot be converted to appropriate types
        
        Filter Design:
        
            The filter implements a second-order Butterworth response with:
        
            - Maximally flat passband (binomial coefficients)
            - -40dB/decade rolloff rate
            - -3dB attenuation exactly at fc
            - Optimal phase response for given magnitude constraints
        
        Example
        -------
        ```python
            >>> # Create lowpass Butterworth filter at 100Hz (sampled at 1kHz)
            >>> lowpass = ButterworthFilter(fc=100.0, fs=1000.0, filter_type='lowpass')
            >>> print(f"Lowpass cutoff: {lowpass.fc} Hz")
            >>> 
            >>> # Create highpass Butterworth filter (DC blocker)
            >>> highpass = ButterworthFilter(fc=20.0, fs=1000.0, filter_type='highpass')
            >>> print(f"Highpass cutoff: {highpass.fc} Hz")
            >>> 
            >>> # Invalid configuration
            >>> try:
            ...     # Cutoff too high (violates Nyquist)
            ...     invalid = ButterworthFilter(fc=600.0, fs=1000.0)
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
        ```
        """
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
        """
        Calculate Butterworth filter coefficients for given specifications.
        
        Computes the biquad filter coefficients that implement a second-order
        Butterworth response with the specified cutoff frequency, sampling rate,
        and filter type (lowpass or highpass).
        
        :param fc: Cutoff frequency in Hz
        :param fs: Sampling frequency in Hz
        :param filter_type: 'lowpass' or 'highpass'
        :return: Tuple of five coefficients (b0, b1, b2, a1, a2)
        
        Implementation Details:
        
            - Uses bilinear transform to convert from analog to digital domain
            - Properly warps the cutoff frequency to account for transform distortion
            - Normalizes coefficients for correct cutoff frequency
            - Implements standard form for second-order section
        
        Example
        -------
        ```python
            >>> # Calculate coefficients for 100Hz lowpass at 1kHz sampling
            >>> filter_obj = ButterworthFilter(fc=0, fs=0)  # Temporary instance
            >>> coeffs = filter_obj._design_filter(100.0, 1000.0, 'lowpass')
            >>> print(f"Biquad coefficients: {[f'{c:.6f}' for c in coeffs]}")
            >>> 
            >>> # Verify cutoff by calculating response
            >>> def calculate_response(coeffs, freq, fs):
            ...     '''Calculate filter magnitude response at specific frequency.'''
            ...     b0, b1, b2, a1, a2 = coeffs
            ...     w = 2 * pi * freq / fs
            ...     z_real = cos(w)
            ...     z_imag = sin(w)
            ...     
            ...     # Calculate numerator
            ...     num_real = b0 + b1 * z_real + b2 * z_real * z_real
            ...     num_imag = -b1 * z_imag - 2 * b2 * z_real * z_imag
            ...     
            ...     # Calculate denominator
            ...     den_real = 1 + a1 * z_real + a2 * z_real * z_real
            ...     den_imag = -a1 * z_imag - 2 * a2 * z_real * z_imag
            ...     
            ...     # Calculate magnitude
            ...     num = sqrt(num_real**2 + num_imag**2)
            ...     den = sqrt(den_real**2 + den_imag**2)
            ...     gain = num / den if den != 0 else 0
            ...     
            ...     return 20 * log10(gain) if gain > 0 else -100
            >>> 
            >>> # Response at cutoff should be -3dB
            >>> response = calculate_response(coeffs, 100.0, 1000.0)
            >>> print(f"Response at cutoff: {response:.2f} dB (should be -3dB)")
        ```
        """
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
        """
        Update filter with new sample and return filtered output.
        
        Processes a single input sample through the second-order Butterworth filter.
        This method inherits the efficient implementation from the BiquadFilter parent class.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Filtered output sample as float
        
        :raises TypeError: If input cannot be converted to float
        
        Example
        -------
        ```python
            >>> # Process a simple step response
            >>> filter_obj = ButterworthFilter(fc=10.0, fs=100.0, filter_type='lowpass')
            >>> 
            >>> # Create and process a step input
            >>> outputs = []
            >>> for i in range(20):
            ...     # Step occurs at i=10
            ...     input_value = 0.0 if i < 10 else 1.0
            ...     output = filter_obj.update(input_value)
            ...     outputs.append(output)
            >>> 
            >>> # Print key points in the response
            >>> print(f"Step applied at sample 10")
            >>> print(f"Initial output (0): {outputs[0]:.6f}")
            >>> print(f"At step (10): {outputs[10]:.6f}")
            >>> print(f"Final value (19): {outputs[19]:.6f}")
        ```
        """
        return super().update(x)
    
    def reset(self) -> None:
        """
        Reset filter to initial state while preserving configuration.
        
        Clears the filter's internal state (delay line) and resets the sample counter,
        but maintains all configuration parameters and coefficients. This method
        inherits its implementation from the BiquadFilter parent class.
        
        Example
        -------
        ```python
            >>> # Create filter and process some data
            >>> filter_obj = ButterworthFilter(fc=30.0, fs=1000.0, filter_type='lowpass')
            >>> 
            >>> # Process first dataset
            >>> for x in [1.0, 0.5, 0.2, 0.7, 0.9]:
            ...     y = filter_obj.update(x)
            >>> 
            >>> print(f"Before reset: {filter_obj.sample_count} samples processed")
            >>> 
            >>> # Reset filter for new dataset
            >>> filter_obj.reset()
            >>> print(f"After reset: {filter_obj.sample_count} samples processed")
            >>> 
            >>> # Filter is now ready for new dataset with same configuration
            >>> new_output = filter_obj.update(1.0)
            >>> print(f"First output after reset: {new_output:.6f}")
        ```
        """
        super().reset()


class FIRFilter(BaseFilter):
    """
    Finite Impulse Response (FIR) filter with configurable tap coefficients.
    
    A versatile FIR filter implementation that processes signals using a direct
    convolution structure with user-specified tap coefficients. This filter offers
    linear phase response, guaranteed stability, and precise frequency response
    control at the cost of higher memory usage compared to IIR filters.
    
    The filter implements the standard FIR convolution equation:
        
        y[n] = Σ(h[k]·x[n-k]) for k = 0 to N-1
    
    Where:
        
        - h[k]: Tap coefficients (impulse response)
        - x[n-k]: Delayed input samples
        - N: Filter order (number of taps)
    
    Key Features:
        
        - Linear phase response (when using symmetric coefficients)
        - Guaranteed stability (no feedback paths)
        - Precise frequency response control
        - Efficient circular buffer implementation
        - Customizable impulse response via tap coefficients
        - Zero phase distortion for symmetric coefficients
        - Runtime coefficient adjustment capability
    
    Performance Characteristics:
        
        - Computational complexity: O(N) per sample
        - Memory usage: 2N float values (taps + buffer)
        - Group delay: (N-1)/2 samples for symmetric coefficients
        - Frequency response exactly matches coefficient design
        - Steeper roll-off requires more taps (memory/CPU tradeoff)
        - Optimized with @micropython.native decorator
    
    Applications:
        
        - Custom frequency response filtering
        - Linear phase signal processing
        - Matched filtering and correlation
        - Hilbert transformers
        - Pulse shaping filters
        - Audio equalization with minimal phase distortion
        - Communication system filters
        - Signal detection and extraction
    
    Mathematical Properties:
        
        - Linear phase when taps are symmetric
        - Frequency response is Fourier transform of tap coefficients
        - Always stable (no feedback, no poles)
        - Finite response duration equal to tap count
        - No recursive behavior (output always settles)
        - Adjustable frequency selectivity based on tap count
    
    """
    
    def __init__(self, taps: list) -> None:
        """
        Initialize FIR filter with specified tap coefficients.
        
        Creates a Finite Impulse Response filter with user-supplied tap coefficients.
        The coefficients directly determine the filter's impulse response and
        frequency characteristics.
        
        :param taps: List or sequence of filter coefficients (must be non-empty)
                    These define the impulse response of the filter
                    Length of taps determines filter order
        
        :raises FilterConfigurationError: If taps list is empty
        :raises TypeError: If taps cannot be converted to float values
        
        Coefficient Design Guidelines:
        
            - Symmetric coefficients (h[i] = h[N-1-i]) → Linear phase
            - Number of taps determines frequency selectivity
            - More taps → Steeper roll-off, narrower transition band
            - Window functions reduce Gibbs phenomenon (ripple)
            - For specific responses, use windowed-sinc or filter design software
        
        Common FIR Types:
        
            - Lowpass: Central coefficient largest, decaying symmetrically
            - Highpass: Alternating signs, central region strongest
            - Bandpass: Sinusoidal envelope modulated by lowpass shape
            - Bandstop: Lowpass with subtracted bandpass component
        
        Example
        -------
        ```python
            >>> # Simple 5-tap moving average filter (symmetric)
            >>> moving_avg = FIRFilter(taps=[0.2, 0.2, 0.2, 0.2, 0.2])
            >>> print(f"Filter order: {len(moving_avg.taps)-1}")
            >>> print(f"Group delay: {(len(moving_avg.taps)-1)/2} samples")
            >>> 
            >>> # Highpass filter (symmetric with alternating signs)
            >>> highpass = FIRFilter(taps=[-0.1, 0.2, -0.5, 0.2, -0.1])
            >>> 
            >>> # Parameter validation
            >>> try:
            ...     invalid_filter = FIRFilter(taps=[])  # Empty taps list
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
        ```
        """
        super().__init__()
        self.taps = taps
        self.reset()

    @property
    def taps(self) -> list:
        """
        Get current tap coefficients.
        
        Returns a copy of the current filter coefficients as a list.
        This allows inspection of the filter configuration without
        modifying internal state.
        
        :return: List of current tap coefficients
        
        Example
        -------
        ```python
            >>> # Inspect filter taps
            >>> filter_obj = FIRFilter(taps=[0.1, 0.2, 0.4, 0.2, 0.1])
            >>> 
            >>> # Get current taps
            >>> current_taps = filter_obj.taps
            >>> print(f"Filter taps: {current_taps}")
            >>> 
            >>> # Verify if filter is symmetric (linear phase)
            >>> def is_symmetric(taps):
            ...     n = len(taps)
            ...     for i in range(n // 2):
            ...         if abs(taps[i] - taps[n-i-1]) > 1e-6:
            ...             return False
            ...     return True
            >>> 
            >>> print(f"Linear phase: {is_symmetric(filter_obj.taps)}")
        ```
        """
        return list(self._taps)

    @taps.setter
    def taps(self, taps: list) -> None:
        """
        Set filter tap coefficients.
        
        Updates the filter's tap coefficients without creating a new filter instance.
        This allows dynamic modification of the filter's frequency response during
        operation.
        
        :param taps: List or sequence of filter coefficients (must be non-empty)
        
        :raises FilterConfigurationError: If taps list is empty
        :raises TypeError: If taps cannot be converted to float values
        
        Dynamic Configuration Applications:
        
            - Adaptive filtering based on signal conditions
            - Filter morphing between different responses
            - Real-time frequency response adjustment
            - A/B testing of different coefficient sets
        
        Example
        -------
        ```python
            >>> # Create filter with initial coefficients
            >>> filter_obj = FIRFilter(taps=[1.0, 0.0, 0.0, 0.0, 0.0])  # Unit impulse
            >>> 
            >>> # Later update to moving average coefficients
            >>> filter_obj.taps = [0.2, 0.2, 0.2, 0.2, 0.2]
            >>> print(f"New taps: {filter_obj.taps}")
            >>> 
            >>> # Dynamic coefficient switching
            >>> def switch_filter_mode(filter_obj, mode='lowpass'):
            ...     '''Switch between different filter responses.'''
            ...     if mode == 'lowpass':
            ...         filter_obj.taps = [0.2, 0.2, 0.2, 0.2, 0.2]
            ...     elif mode == 'highpass':
            ...         filter_obj.taps = [-0.1, 0.2, -0.5, 0.2, -0.1]
            ...     else:
            ...         filter_obj.taps = [0.0, 0.0, 1.0, 0.0, 0.0]  # All-pass
            ...     
            ...     print(f"Switched to {mode} mode, filter length: {len(filter_obj.taps)}")
        ```
        """
        if not taps:
            raise FilterConfigurationError("taps list cannot be empty")
        self._taps = array("f", [float(t) for t in taps])
        self.n = len(self._taps)
        if hasattr(self, '_buf'):
            self._buf = array("f", [0.0] * self.n)
            self._idx = 0

    @micropython.native
    def update(self, x: float) -> float:
        """
        Update filter with new sample and return filtered output.
        
        Processes a single input sample through the FIR filter using
        direct convolution with the tap coefficients. This implements
        the standard FIR filtering operation.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Filtered output sample as float
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
        
            1. Store new input in circular buffer
            2. Compute dot product of buffer and tap coefficients
            3. Advance circular buffer index
            4. Return computed result
        
        Performance Characteristics:
        
            - Computational complexity: O(N) operations per sample
            - Memory access: 2N reads, 1 write per sample
            - Optimized with @micropython.native decorator
            - Linear scaling with number of taps
        
        Example
        -------
        ```python
            >>> # Basic FIR filtering operation
            >>> # 5-tap moving average filter
            >>> filter_obj = FIRFilter(taps=[0.2, 0.2, 0.2, 0.2, 0.2])
            >>> 
            >>> # Process a simple impulse input
            >>> inputs = [0, 0, 1, 0, 0, 0, 0, 0]
            >>> outputs = []
            >>> 
            >>> for sample in inputs:
            ...     output = filter_obj.update(sample)
            ...     outputs.append(output)
            >>> 
            >>> print("Impulse Response:")
            >>> for i, out in enumerate(outputs):
            ...     print(f"Sample {i}: {out:.1f}")
            >>> 
            >>> # Response should match tap coefficients as impulse propagates
            >>> # through the filter's delay line
        ```
        """
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
        """
        Reset filter to initial state while preserving coefficients.
        
        Clears the filter's delay line (buffer) and resets the sample counter,
        but maintains the tap coefficients. This allows the filter to be reused
        for new data streams without reconfiguration.
        
        Reset Operations:
        
            - Clears input buffer (delay line) to zeros
            - Resets buffer index to 0
            - Resets sample counter to 0
            - Preserves tap coefficients
        
        Use Cases:
        
            - Starting new filtering session
            - Removing previous signal history
            - Batch processing of multiple signals
            - Clearing filter state after transients
        
        Example
        -------
        ```python
            >>> # Demonstrate filter reset
            >>> filter_obj = FIRFilter(taps=[0.1, 0.2, 0.4, 0.2, 0.1])
            >>> 
            >>> # Process some data
            >>> for sample in [1.0, 2.0, 3.0, 4.0, 5.0]:
            ...     result = filter_obj.update(sample)
            >>> 
            >>> print(f"Before reset: {filter_obj.sample_count} samples processed")
            >>> 
            >>> # Reset filter
            >>> filter_obj.reset()
            >>> print(f"After reset: {filter_obj.sample_count} samples processed")
            >>> 
            >>> # Process new data with clean state
            >>> new_result = filter_obj.update(10.0)
            >>> print(f"First result after reset: {new_result:.1f}")
        ```
        """
        super().reset()
        self._buf = array("f", [0.0] * self.n)
        self._idx = 0


class FilterChain(BaseFilter):
    """
    Chain multiple filters in series for complex signal processing.
    
    A flexible filter composition system that connects multiple filter objects
    in series, with each filter's output feeding into the next filter's input.
    This enables creation of sophisticated signal processing pipelines by
    combining simpler filter building blocks.
    
    The filter chain implements sequential processing:
        
        y[n] = fₙ(... f₂(f₁(x[n])))
    
    Where:
        
        - f₁, f₂, ..., fₙ: Individual filter objects in the chain
        - x[n]: Input sample
        - y[n]: Output after processing through all filters
    
    Key Features:
        
        - Flexible composition of filter elements
        - Dynamic chain reconfiguration during operation
        - Unified interface through BaseFilter compatibility
        - Memory-efficient implementation
        - Runtime chain modification (add/remove filters)
        - Simple signal flow - sequential processing
        - Unified reset capability for entire chain
    
    Performance Characteristics:
        
        - Computational complexity: Sum of individual filter complexities
        - Memory usage: Sum of individual filter memory requirements
        - Overall delay: Sum of individual filter delays
        - Frequency response: Multiplication of individual responses
        - Phase response: Sum of individual phase responses
        - Group delay: Sum of individual group delays
    
    Applications:
        
        - Multi-stage signal conditioning
        - Audio processing chains (noise reduction → EQ → compression)
        - Sensor data preprocessing pipelines
        - Complex filter design via simpler building blocks
        - Signal pre/post processing combinations
        - Adaptive processing with reconfigurable stages
        - Test harnesses for filter component evaluation
    
    Chain Properties:
        
        - Overall transfer function: Product of individual transfer functions
        - Overall impulse response: Convolution of individual impulse responses
        - Overall stability: Chain is stable if all components are stable
        - Order of filters affects overall response
        - Can combine different filter types (IIR, FIR, etc.)
    """
    
    def __init__(self, *filters: BaseFilter) -> None:
        """
        Initialize filter chain with a sequence of filters.
        
        Creates a filter chain by connecting multiple filter objects in series.
        Each filter's output feeds into the next filter's input, creating a
        processing pipeline.
        
        :param *filters: One or more filter objects (variable argument list)
                        All must be instances of BaseFilter subclasses
                        Order determines processing sequence
        
        :raises FilterConfigurationError: If no filters are provided or if any argument is not a BaseFilter
        
        Chain Design Guidelines:
        
            - Order matters: results can differ based on filter sequence
            - Consider placing computation-heavy filters later in chain when possible
            - Low-pass filtering early can reduce computational load for later stages
            - Consider signal levels between stages to avoid clipping/underflow
        
        Example
        -------
        ```python
            >>> # Create a multi-stage processing chain
            >>> # Stage 1: Median filter for outlier rejection
            >>> outlier_filter = MedianFilter(window_size=3)
            >>> 
            >>> # Stage 2: Low-pass filter for noise reduction
            >>> noise_filter = LowPassFilter(fc=10.0, fs=100.0)
            >>> 
            >>> # Stage 3: Kalman filter for optimal tracking
            >>> tracker = KalmanFilter(process_noise=0.01, measurement_noise=0.1)
            >>> 
            >>> # Create the chain connecting all three filters
            >>> processing_chain = FilterChain(outlier_filter, noise_filter, tracker)
            >>> print(f"Chain length: {len(processing_chain.filters)} filters")
            >>> 
            >>> # Parameter validation
            >>> try:
            ...     invalid_chain = FilterChain()  # No filters
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
        ```
        """
        super().__init__()
        if not filters:
            raise FilterConfigurationError("At least one filter required")
        
        for f in filters:
            if not isinstance(f, BaseFilter):
                raise FilterConfigurationError("All items must be BaseFilter instances, got {}".format(type(f)))
        
        self.filters = list(filters)

    @micropython.native
    def update(self, x: float) -> float:
        """
        Process sample through filter chain.
        
        Processes a single input sample sequentially through each filter in the chain,
        where each filter's output becomes the input to the next filter.
        
        :param x: Input sample value (any numeric type, converted to float)
        :return: Output from last filter in chain as float
        
        :raises TypeError: If input cannot be converted to float
        
        Algorithm Details:
        
            1. Convert input to float
            2. Pass input through first filter
            3. Pass that filter's output to next filter
            4. Continue through chain sequentially
            5. Return output from final filter
        
        Example
        -------
        ```python
            >>> # Create a simple processing chain
            >>> # Median filter followed by low-pass filter
            >>> chain = FilterChain(
            ...     MedianFilter(window_size=3),
            ...     LowPassFilter(fc=10.0, fs=100.0)
            ... )
            >>> 
            >>> # Process a sample with outlier followed by valid readings
            >>> test_data = [1.0, 10.0, 2.0, 1.5, 1.8]
            >>> 
            >>> print("Input | Output")
            >>> print("-" * 15)
            >>> 
            >>> for sample in test_data:
            ...     result = chain.update(sample)
            ...     print(f"{sample:5.1f} | {result:6.3f}")
            >>> 
            >>> # Output should show outlier rejection and smoothing
        ```
        """
        self._sample_count += 1
        result = float(x)
        for filter_obj in self.filters:
            result = filter_obj.update(result)
        return result

    def reset(self) -> None:
        """
        Reset all filters in chain.
        
        Resets the state of all filters in the chain and the chain's own sample
        counter. Each filter maintains its configuration but clears its internal state.
        
        Reset Operations:
        
            - Resets each filter in the chain
            - Resets chain's sample counter
            - Preserves chain structure and filter configurations
            - Prepares entire chain for new input sequence
        
        Example
        -------
        ```python
            >>> # Create a filter chain
            >>> chain = FilterChain(
            ...     MovingAverageFilter(window_size=5),
            ...     LowPassFilter(fc=10.0, fs=100.0)
            ... )
            >>> 
            >>> # Process some data
            >>> for i in range(10):
            ...     result = chain.update(i)
            >>> 
            >>> print(f"Before reset: {chain.sample_count} samples processed")
            >>> 
            >>> # Reset entire chain
            >>> chain.reset()
            >>> print(f"After reset: {chain.sample_count} samples processed")
            >>> 
            >>> # Verify filters in chain were reset
            >>> all_reset = all(f.sample_count == 0 for f in chain.filters)
            >>> print(f"All filters reset: {all_reset}")
        ```
        """
        super().reset()
        for filter_obj in self.filters:
            filter_obj.reset()

    def add_filter(self, filter_obj: BaseFilter) -> None:
        """
        Add filter to end of chain.
        
        Appends a new filter to the end of the filter chain. The new filter will 
        receive input from the previously last filter in the chain.
        
        :param filter_obj: Filter to add (must be BaseFilter instance)
        
        :raises FilterConfigurationError: If filter_obj is not a BaseFilter instance
        
        Dynamic Chain Modification:
        
            - Allows runtime extension of processing pipeline
            - New filter becomes the new final stage
            - Existing chain state is preserved
            - Useful for adaptive processing systems
        
        Example
        -------
        ```python
            >>> # Create initial chain
            >>> chain = FilterChain(MovingAverageFilter(window_size=3))
            >>> print(f"Initial chain length: {len(chain.filters)}")
            >>> 
            >>> # Add another filter dynamically
            >>> chain.add_filter(LowPassFilter(fc=10.0, fs=100.0))
            >>> print(f"Updated chain length: {len(chain.filters)}")
            >>> 
            >>> # Process data through extended chain
            >>> result = chain.update(5.0)
            >>> print(f"Result after adding filter: {result:.3f}")
            >>> 
            >>> # Conditional filter addition
            >>> def adaptive_chain(signal_level):
            ...     '''Add appropriate filter based on signal conditions.'''
            ...     chain = FilterChain(MedianFilter(window_size=3))
            ...     
            ...     if signal_level > 10.0:
            ...         # Add high noise filtering for strong signals
            ...         chain.add_filter(LowPassFilter(fc=5.0, fs=100.0))
            ...         print("Added strong noise filter")
            ...     else:
            ...         # Add light filtering for weaker signals
            ...         chain.add_filter(LowPassFilter(fc=20.0, fs=100.0))
            ...         print("Added light noise filter")
            ...     
            ...     return chain
        ```
        """
        if not isinstance(filter_obj, BaseFilter):
            raise FilterConfigurationError("filter_obj must be BaseFilter instance")
        self.filters.append(filter_obj)

    def remove_filter(self, index: int) -> BaseFilter:
        """
        Remove filter from chain.
        
        Removes a filter at the specified index from the chain. This allows
        dynamic reconfiguration of the filter chain during operation.
        
        :param index: Zero-based index of filter to remove
        :return: The removed filter object
        
        :raises FilterConfigurationError: If index is out of range
        
        Chain Modification Applications:
        
            - Dynamic processing pipeline adaptation
            - A/B testing of filter configurations
            - Fault tolerance (removing problematic filters)
            - Resource management (removing unneeded stages)
        
        Example
        -------
        ```python
            >>> # Create a three-stage filter chain
            >>> chain = FilterChain(
            ...     MedianFilter(window_size=3),
            ...     LowPassFilter(fc=10.0, fs=100.0),
            ...     MovingAverageFilter(window_size=5)
            ... )
            >>> print(f"Initial chain length: {len(chain.filters)}")
            >>> 
            >>> # Remove the middle filter
            >>> removed = chain.remove_filter(1)
            >>> print(f"Removed: {type(removed).__name__}")
            >>> print(f"Updated chain length: {len(chain.filters)}")
            >>> 
            >>> # Invalid index
            >>> try:
            ...     chain.remove_filter(5)  # Out of range
            >>> except FilterConfigurationError as e:
            ...     print(f"Error: {e}")
        ```
        """
        if not (0 <= index < len(self.filters)):
            raise FilterConfigurationError("Filter index out of range")
        return self.filters.pop(index)
