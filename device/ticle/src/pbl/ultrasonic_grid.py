from . import (
    utime,
    utools, ext
)

class UltrasonicGrid:
    """
    A class to manage a grid of ultrasonic sensors (SR04) for distance measurement.
    This class provides core methods to read distances and manage sensor coordinates.
    """
    def __init__(self, sensors:ext.SR04, width:int, height:int):
        """
        Initialize the UltrasonicGrid with a SR04 instance and grid dimensions.
        
        :param sensors: An instance of ext.SR04 configured with the desired sensors.
        :param width: The number of sensors in each row of  the grid.
        :param height: The number of sensors in each column of the grid.
        """
        expected_sensor_count = width * height
        actual_sensor_count = len(sensors)
        
        if actual_sensor_count != expected_sensor_count:
            raise ValueError(f"Expected {expected_sensor_count} sensors, but got {actual_sensor_count}.")

        self.sensors = sensors
        self.width = width
        self.height = height
        self.total_sensors = expected_sensor_count

    def __repr__(self) -> str:
        return f"UltrasonicGrid({self.width}x{self.height}, {self.total_sensors} sensors)"

    def index_to_coord(self, sensor_index: int) -> tuple[int, int]:
        """
        Convert a sensor index to its corresponding (row, column) coordinates in the grid.
        
        :param sensor_index: The index of the sensor in the flat list.
        :return: A tuple (row, column) representing the sensor's position in the grid
        """
        if not (0 <= sensor_index < self.total_sensors):
            raise IndexError(f"Sensor index {sensor_index} is out of range (0-{self.total_sensors-1})")
        
        row = sensor_index // self.width
        col = sensor_index % self.width
        return (row, col)

    def coord_to_index(self, row: int, col: int) -> int:
        """
        Convert (row, column) coordinates to the corresponding sensor index in the flat list.
        
        :param row: The row index of the sensor.
        :param col: The column index of the sensor.
        :return: The index of the sensor in the flat list.
        """
        if not (0 <= row < self.height):
            raise IndexError(f"Row index {row} is out of range (0-{self.height-1})")
        if not (0 <= col < self.width):
            raise IndexError(f"Column index {col} is out of range (0-{self.width-1})")
        
        return row * self.width + col

    def read_grid(self, delay_ms: int = 8) -> list[list[int | None]]:
        """
        Read the distances from all sensors in an optimized order to minimize measurement time.
        This method alternates between black and white squares in a checkerboard pattern,
        allowing for faster measurements by reducing the number of active sensors at any time.
        
        :param delay_ms: Delay in milliseconds between measurements to allow sensors to stabilize.
        :return: A 2D list where each element is the distance measured by the corresponding sensor.
        If a sensor fails to measure, its value will be None.
        """
        measurement_order = self._generate_optimized_order()
        
        distances = [None] * self.total_sensors
        
        for i, sensor_idx in enumerate(measurement_order):
            distances[sensor_idx] = self.sensors[sensor_idx].value
            
            if i < len(measurement_order) - 1:
                utime.sleep_ms(delay_ms)
        
        grid = []
        for row in range(self.height):
            grid_row = []
            for col in range(self.width):
                idx = self.coord_to_index(row, col)
                grid_row.append(distances[idx])
            grid.append(grid_row)
        
        return grid

    def _generate_optimized_order(self) -> list[int]:
        """
        Generate an optimized measurement order for the sensors in a checkerboard pattern.
        This method creates two lists: one for black squares and one for white squares,
        and then combines them in an alternating fashion to minimize measurement time.
        
        :return: A list of sensor indices in the optimized measurement order.
        """
        black_squares = []  # (row + col) % 2 == 0
        white_squares = []  # (row + col) % 2 == 1
        
        for row in range(self.height):
            for col in range(self.width):
                sensor_idx = self.coord_to_index(row, col)
                if (row + col) % 2 == 0:
                    black_squares.append(sensor_idx)
                else:
                    white_squares.append(sensor_idx)
        
        measurement_order = []
        max_len = max(len(black_squares), len(white_squares))
        
        for i in range(max_len):
            if i < len(black_squares):
                measurement_order.append(black_squares[i])
            if i < len(white_squares):
                measurement_order.append(white_squares[i])
        
        return measurement_order

    def setup_nonblocking_callbacks(self, enable: bool = True):
        """Setup non-blocking callbacks for all sensors"""
        if enable:
            self.sensors[:].nonblocking = True
            self.sensors[:].callback = self._sensor_callback
            self.sensors.measurement = True
        else:
            self.sensors.measurement = False
            self.sensors[:].nonblocking = False
            self.sensors[:].callback = None

    def _sensor_callback(self, pin, distance):
        """Internal callback function for sensor readings"""
        sensor_idx = None
        for i in range(self.total_sensors):
            if hasattr(self.sensors, '_trig_pins') and self.sensors._trig_pins[i] == pin:
                sensor_idx = i
                break
        
        if sensor_idx is not None:
            row, col = self.index_to_coord(sensor_idx)
            self.latest_readings[row][col] = distance

    def get_sensor_info(self, row: int, col: int) -> dict:
        sensor_idx = self.coord_to_index(row, col)
        distance = self.sensors[sensor_idx].value
        
        return {
            'row': row,
            'col': col,
            'sensor_index': sensor_idx,
            'distance_cm': distance,
            'color': self.distance_to_color(distance),
            'trig_pin': self.sensors._trig_pins[sensor_idx],
            'echo_pin': self.sensors._echo_pins[sensor_idx]
        }

    def get_color_config(self):
        """Get color configuration for distance visualization"""
        colors = {
            'very_close': utools.ANSIEC.FG.BRIGHT_RED + '██' + utools.ANSIEC.OP.RESET,
            'close':      utools.ANSIEC.FG.BRIGHT_YELLOW + '██' + utools.ANSIEC.OP.RESET,
            'medium':     utools.ANSIEC.FG.BRIGHT_GREEN + '██' + utools.ANSIEC.OP.RESET,
            'far':        utools.ANSIEC.FG.BRIGHT_BLUE + '██' + utools.ANSIEC.OP.RESET,
            'very_far':   utools.ANSIEC.FG.BRIGHT_WHITE + '██' + utools.ANSIEC.OP.RESET,
            'no_signal':  utools.ANSIEC.FG.BRIGHT_BLACK + '▓▓' + utools.ANSIEC.OP.RESET
        }
        
        distance_ranges = [
            (0, 20, 'very_close'),
            (20, 40, 'close'),
            (40, 60, 'medium'),
            (60, 100, 'far'),
            (100, 400, 'very_far')
        ]
        
        return colors, distance_ranges


    def distance_to_color(self, distance: int | None) -> str:
        """
        Convert a distance measurement to a color block representation.
        
        :param distance: The distance in centimeters, or None if no signal.
        :return: A string representing the color block for the given distance.
        """
        colors, distance_ranges = self.get_color_config()
        
        if distance is None:
            return colors['no_signal']
        
        for min_dist, max_dist, color_key in distance_ranges:
            if min_dist <= distance < max_dist:
                return colors[color_key]
        
        return colors['very_far']