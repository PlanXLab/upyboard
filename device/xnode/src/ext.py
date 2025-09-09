import utime
import ustruct
import machine

from micropython import const


class Pir:
    NONE = 0
    DETECT = 1
    ENTER = 2
    LEAVE = 3
    
    def __init__(self):
        self.__pin = machine.Pin('P2', machine.Pin.IN)
        self.__t0 = 0
        
    def read(self):
        return self.__pin.value()    

    def state(self, timeout=1):
        old_state = self.read()
        utime.sleep_ms(timeout)
        curr_state = self.read()
        if old_state != curr_state:
            return self.ENTER if curr_state else self.LEAVE
        else:
            return curr_state
    
    def detect(self, timeout=1500):
        n = self.read()
        if n == self.DETECT and not self.__t0:
            self.__t0 = utime.ticks_ms()
        elif self.__t0:
            if utime.ticks_ms() - self.__t0 >= timeout:
                self.__t0 = 0
            n = not self.DETECT
        return n == self.DETECT

class IRThermometer:
    MLX90614_ADDR = const(0x5A)

    EEPROM_PWMCTRL = const(0x02)
    EEPROM_CONFIG_REGISTER1 = const(0x05)

    def __init__(self):
        self.__i2c = machine.I2C(1, freq=50000) #The maximum frequency of the MLX90614 is 100 KHz and the minimum is 10 KHz. 

    def read(self, object=True, eeprom=False):
        reg = const(0x07) if object else const(0x06)
        if eeprom:
            reg = 0x20 | reg
            
        data = self.__i2c.readfrom_mem(IRThermometer.MLX90614_ADDR, reg, 2)
        return ustruct.unpack('<H', data)[0]

    def ambient(self):
        data = self.read(False)
        return round(data * 0.02 - 273.15, 1)

    def object(self):
        data = self.read()
        return round(data * 0.02 - 273.15, 1)


class IMU:
    BNO055_ADDR = const(0x28)

    ACCELERATION = const(0x08)
    MAGNETIC = const(0x0E)
    GYROSCOPE = const(0x14)
    EULER = const(0x1A)
    QUATERNION = const(0x20)
    ACCEL_LINEAR = const(0x28)
    ACCEL_GRAVITY = const(0x2E)
    TEMPERATURE = const(0x34)
    
    def __init__(self):
        self.__i2c = machine.I2C(1)
        self.__scale = {self.ACCELERATION:1/100, self.MAGNETIC:1/16, self.GYROSCOPE:0.001090830782496456, self.EULER:1/16,  self.QUATERNION:1/(1<<14), self.ACCEL_LINEAR:1/100, self.ACCEL_GRAVITY:1/100}
        self.__call = {self.ACCELERATION:self.__read_other, self.MAGNETIC:self.__read_other, self.GYROSCOPE:self.__read_other, self.EULER:self.__read_other,  self.QUATERNION:self.__read_quaternion, self.ACCEL_LINEAR:self.__read_other, self.ACCEL_GRAVITY:self.__read_other, self.TEMPERATURE:self.__read_temperature}

        self.__i2c.writeto_mem(IMU.BNO055_ADDR, 0X3D, bytes([0x00])) #Mode Register, Enter configuration.
        utime.sleep_ms(20)
        self.__i2c.writeto_mem(IMU.BNO055_ADDR, 0x3F, bytes([0x20])) #Trigger Register, Reset
        utime.sleep_ms(650)
        self.__i2c.writeto_mem(IMU.BNO055_ADDR, 0X3E, bytes([0x00])) #Power Register, Set to normal power. cf) low power is 0x01
        self.__i2c.writeto_mem(IMU.BNO055_ADDR, 0X07, bytes([0x00])) #Page Register, Make sure we're in config mode and on page0(param, data), page1(conf)
        self.__i2c.writeto_mem(IMU.BNO055_ADDR, 0X3F, bytes([0x80])) #Trigger Register, External oscillator
        self.__i2c.writeto_mem(IMU.BNO055_ADDR, 0X3F, bytes([0x00])) #Trigger Register,
        utime.sleep_ms(10)
        self.__i2c.writeto_mem(IMU.BNO055_ADDR, 0X3D, bytes([0x0C])) #Mode Register, Enter normal operation (NDOF)
        utime.sleep_ms(200)

    def __read_temperature(self, addr):
        t = self.__i2c.readfrom_mem(IMU.BNO055_ADDR, addr, 1)[0]
        return t - 256 if t > 127 else t

    def __read_quaternion(self, addr):
        t = self.__i2c.readfrom_mem(IMU.BNO055_ADDR, addr, 8)  
        return tuple(v * self.__scale[self.QUATERNION] for v in ustruct.unpack('hhhh', t))

    def __read_other(self, addr):
        if addr not in self.__scale:
            raise ValueError(f"Address {addr} not in scale mapping")
        t = self.__i2c.readfrom_mem(IMU.BNO055_ADDR, addr, 6)
        return tuple(v * self.__scale[addr] for v in ustruct.unpack('hhh', t))

    def calibration(self):
        data = self.__i2c.readfrom_mem(IMU.BNO055_ADDR, 0x35, 1)[0] #Calibration Resiger, Read        
        return (data >> 6) & 0x03, (data >> 4) & 0x03, (data >> 2) & 0x03, data & 0x03  #Sys, Gyro, Accel, Mag

    def read(self, addr):
        return self.__call[addr](addr)


class _I2CtoUart:
    SC16IS750_ADDR = const(0x4D)
    
    # Register addresses (using datasheet table 21 as a reference)
    RHR = const(0x00)  # Receive Holding Register (read)
    THR = const(0x00)  # Transmit Holding Register (write)
    IER = const(0x01)  # Interrupt Enable Register
    FCR = const(0x02)  # FIFO Control Register
    IIR = const(0x02)  # Interrupt Identification Register
    LCR = const(0x03)  # Line Control Register
    MCR = const(0x04)  # Modem Control Register
    LSR = const(0x05)  # Line Status Register
    MSR = const(0x06)  # Modem Status Register
    SPR = const(0x07)  # Scratchpad Register
    TXLVL = const(0x08)  # Transmit FIFO Level
    RXLVL = const(0x09)  # Receive FIFO Level
    IODIR = const(0x0A)  # IO pin direction control
    IOSTATE = const(0x0B) # IO pin state control
    IOINTENA = const(0x0C) # IO interrupt enable
    IOCTRL = const(0x0E)  # IO control configuration
    EFCR = const(0x0F)  # Extra features control register

    DLL = const(0x00)  # Divisor Latch LSB (when LCR[7] = 1)
    DLH = const(0x01)  # Divisor Latch MSB (when LCR[7] = 1)

    # Line Status Register (LSR) bits
    LSR_DR = const(0x01)  # Data Ready
    LSR_OE = const(0x02)  # Overrun Error
    LSR_PE = const(0x04)  # Parity Error
    LSR_FE = const(0x08)  # Framing Error
    LSR_BI = const(0x10)  # Break Interrupt
    LSR_THRE = const(0x20)  # Transmitter Holding Register Empty
    LSR_TEMT = const(0x40)  # Transmitter Empty
    LSR_FIFOE = const(0x80) # FIFO Error

    # FIFO Control Register (FCR) bits
    FCR_FIFO_ENABLE = const(0x01)  # FIFO enable
    FCR_FIFO_RCVR_RESET = const(0x02) # Reset Receiver FIFO
    FCR_FIFO_XMIT_RESET = const(0x04) # Reset Transmitter FIFO
    FCR_FIFO_DMA_MODE = const(0x08)  # DMA mode
    FCR_FIFO_64 = const(0x20) # Enable 64 byte FIFO (SC16IS750/760)
    FCR_FIFO_RCVR_TRIGGER_MASK = const(0xC0)  # Receiver FIFO trigger level
    FCR_FIFO_RCVR_TRIGGER_8 = const(0x00)  # 8 bytes
    FCR_FIFO_RCVR_TRIGGER_16 = const(0x40) # 16 bytes
    FCR_FIFO_RCVR_TRIGGER_32 = const(0x80) # 32 bytes
    FCR_FIFO_RCVR_TRIGGER_60 = const(0xC0) # 60 bytes

    # Modem Control Register (MCR) bits
    MCR_DTR = const(0x01)  # DTR output
    MCR_RTS = const(0x02)  # RTS output
    MCR_LOOP = const(0x10) # Loop back mode
        
    def __init__(self, baudrate=9600, data_bits=8, stop_bits=1, parity='N'):
        self._i2c = machine.I2C(1)
        
        """
        Initializes the UART communication parameters.
        :param baudrate: The desired baud rate (default: self.baudrate).
        :param data_bits: The number of data bits (default: 8).
        :param stop_bits: The number of stop bits (default: 1).
        :param parity: The parity (default: 'N' - None, 'E' - Even, 'O' - Odd).
        """

        crystal_freq = 12_000_000   # XNode crystal frequency: 12MHz
        
        # Calculate divisor
        divisor = int(crystal_freq / (baudrate * 16))
        
        self.reset()
        
        # Enable access to divisor registers
        lcr = self.__read(self.LCR)
        self.__write(self.LCR, lcr | 0x80)

        # Write divisor
        self.__write(self.DLL, divisor & 0xFF)           # Division Register LSB
        self.__write(self.DLH, (divisor >> 8) & 0xFF)    

        # Set data bits, stop bits, and parity
        lcr = 0
        if data_bits == 8:
            lcr |= 0x03
        elif data_bits == 7:
            lcr |= 0x02
        elif data_bits == 6:
            lcr |= 0x01
        elif data_bits == 5:
            lcr |= 0x00
        else:
            raise ValueError("Invalid number of data bits")

        if stop_bits == 2:
            lcr |= 0x04

        if parity == 'E':
            lcr |= 0x18
        elif parity == 'O':
            lcr |= 0x08        

        # Disable divisor access and set communication parameters
        self.__write(self.LCR, lcr & 0x7F)

        # Configure FIFO
        self.__write(self.FCR, self.FCR_FIFO_ENABLE | \
                                  self.FCR_FIFO_RCVR_RESET | \
                                  self.FCR_FIFO_XMIT_RESET | \
                                  self.FCR_FIFO_RCVR_TRIGGER_16)

        # Enable RX interrupts
        self.__write(self.IER, 0x01)

    def __read(self, reg):
        return self._i2c.readfrom_mem(self.SC16IS750_ADDR, reg, 1)[0]

    def __write(self, reg, data):
        self._i2c.writeto_mem(self.SC16IS750_ADDR, reg, bytes([data]))

    def reset(self):
        """Resets the SC16IS750."""
        # Software reset via IOControl register
        self.__write(self.IOCTRL, 0x02)
        utime.sleep_ms(1)
    
    def available(self):
        """Returns the number of bytes available to read."""
        return self.__read(self.RXLVL)
            
    def read(self, size=1):
        """Reads data from the RX FIFO.
        size: The number of bytes to read (default: 1).

        :return: The bytes type data read from the RX FIFO.
        """
        
        if size == 1:
            if self.available() > 0:
                return bytes([self.__read(self.RHR)])
            else:
                return bytes()
        else:
            data = bytearray()
            for _ in range(size):
                if self.available() > 0:
                    data.append(self.__read(self.RHR))
                else:
                    break
            return bytes(data)
        
    def write(self, data):
        """Writes data to the TX FIFO.

        param data: The data to write (bytes or str).
        """
        if isinstance(data, str):
            data = data.encode('utf-8')

        for byte in data:
            while not (self._read(self.LSR) & self.LSR_THRE):
                pass  # Wait until TX FIFO is not full
            self.__write(self.THR, byte)

class Gps:
    """
    GPS module control class.
    """
    # FGPMMOPA6H(MediaTek MT3339) GPS Module
    # NMEA Checksum Calculator: https://nmeachecksum.eqth.net/
    
    UPDATE_1HZ = "$PMTK220,1000*1F"
    UPDATE_2HZ = "$PMTK220,200*2C"
    UPDATE_10HZ = "$PMTK220,100*2F"

    GPGGA = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
    GPVTG = "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
    GPRMC = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"    
    
    BAUD_9600   = "$PMTK251,9600*17"
    BAUD_19200  = "$PMTK251,19200*22"
    BAUD_38400  = "$PMTK251,38400*27"   #Do Not!!!
    BAUD_115200 = "$PMTK251,115200*1F"  #Do Not!!!
        
    def __init__(self, first_update=True, gps_mode=GPGGA, baudrate=BAUD_9600):
        """
        Initializes the Gps object.

        Args:
            i2c: The I2C object from the machine module.
            i2c_address: The I2C address of the SC16IS750 (default: 0x4D).
            sc16is750_crystal_freq: The crystal oscillator frequency of the SC16IS750 (default: 11059200).
            baudrate: The baud rate for communication with the GPS module (default: 9600).
        """
        if baudrate == self.BAUD_9600:
            i2c_uart_bourate = 9600
        elif baudrate == self.BAUD_19200:
            i2c_uart_bourate = 19200
        elif i2c_uart_bourate == self.BAUD_38400:
            i2c_uart_bourate = 38400
        elif i2c_uart_bourate == self.BAUD_115200:
            i2c_uart_bourate = 115200
        else:
            raise ValueError("Invalid baudrate")
         
        self.__uart = _I2CtoUart(i2c_uart_bourate)
        self.setBaudrate(baudrate)
                
        self.setFastUpdate(first_update)
        self.setGpsMode(gps_mode)
        
        self.latest_nmea = ""
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.fix_quality = 0
        self.satellites_in_use = 0
        self.timestamp = None
        self.speed_over_ground_knots = 0.0
        self.speed_over_ground_kmh = 0.0
        self.course_over_ground = 0.0
        self.fix_status = 'V' # Default to 'V' (void/invalid)
        
    def __sendCommand(self, command):
        """
        Sends a command to the GPS module.
        :param command: The command string to send.
        """
        self.__uart.write(command + '\r\n')
        utime.sleep_ms(50)

    def setBaudrate(self, baudrate):
        self.__sendCommand(baudrate)

    def setFastUpdate(self, first_update):
        self.__sendCommand(self.UPDATE_2HZ if first_update else self.UPDATE_1HZ) 
    
    def setGpsMode(self, gps_mode):
        self.gps_mode = gps_mode
        self.__sendCommand(gps_mode) 

    def update(self, timeout_ms = 2000):
        """
        Reads NMEA sentences from the GPS module and parses them.
        :param timeout_ms: Maximum time in milliseconds to wait for a complete NMEA sentence.

        :return: True if a valid NMEA sentence has been parsed, False otherwise
        """

        start_time = utime.ticks_ms()
        while (utime.ticks_ms() - start_time) < timeout_ms:
            if self.__uart.available():
                nmea_char = self.__uart.read().decode("ascii")
                self.latest_nmea += nmea_char
                if nmea_char == "\n":
                    if self._parse_nmea(self.latest_nmea):
                        self.latest_nmea = ""
                        return True
                    else:
                        self.latest_nmea = ""
                        return False
        return False

    def __parse_nmea_gpgga(self, nmea_sentence):
        if nmea_sentence.startswith("$GPGGA"):
            parts = nmea_sentence.split(",")
            if len(parts) >= 10 and all(parts[i] for i in [1, 2, 3, 4, 5, 6, 9]):  # Check for empty fields
                try:
                    # Time (UTC)
                    self.timestamp = parts[1]

                    # Latitude
                    lat_degrees = int(parts[2][:2])
                    lat_minutes = float(parts[2][2:])
                    self.latitude = lat_degrees + lat_minutes / 60.0
                    if parts[3] == "S":
                        self.latitude = -self.latitude

                    # Longitude
                    lon_degrees = int(parts[4][:3])
                    lon_minutes = float(parts[4][3:])
                    self.longitude = lon_degrees + lon_minutes / 60.0
                    if parts[5] == "W":
                        self.longitude = -self.longitude

                    # Fix quality
                    self.fix_quality = int(parts[6])

                    # Satellites in use
                    self.satellites_in_use = int(parts[7])
                    
                    # Altitude
                    self.altitude = float(parts[9])

                    return True
                except ValueError:
                    return False
            else:
                return False
        return False
    
    
    def ___parse_nmea_gpvtg(self, nmea_sentence):
        if nmea_sentence.startswith("$GPRMC"):
            parts = nmea_sentence.split(",")
            if len(parts) >= 10 and all(parts[i] for i in [1, 2, 3, 4, 5, 7, 8, 9]): # Check for empty fields
                try:
                    # Time (UTC)
                    self.timestamp = parts[1]

                    # Fix status (A: active, V: void)
                    self.fix_status = parts[2]

                    # Latitude
                    lat_degrees = int(parts[3][:2])
                    lat_minutes = float(parts[3][2:])
                    self.latitude = lat_degrees + lat_minutes / 60.0
                    if parts[4] == "S":
                        self.latitude = -self.latitude

                    # Longitude
                    lon_degrees = int(parts[5][:3])
                    lon_minutes = float(parts[5][3:])
                    self.longitude = lon_degrees + lon_minutes / 60.0
                    if parts[6] == "W":
                        self.longitude = -self.longitude

                    # Speed over ground (knots)
                    self.speed_over_ground_knots = float(parts[7])

                    # Course over ground (degrees)
                    self.course_over_ground = float(parts[8])

                    # Date (DDMMYY)
                    self.date = parts[9] # You might want to store and use the date

                    return True

                except ValueError:
                    return False
            else:
                return False
        return False
    
    def ___parse_nmea_gprmc(self, nmea_sentence):
        if nmea_sentence.startswith("$GPVTG"):
            parts = nmea_sentence.split(",")
            if len(parts) >= 8 and all(parts[i] for i in [1, 5, 7]): # Check for empty fields
                try:
                    # Course over ground (degrees) - True
                    if parts[1]:
                        self.course_over_ground = float(parts[1])
                    # Course over ground (degrees) - Magnetic
                    # if parts[3]:
                    #     self.course_over_ground_magnetic = float(parts[3])
                    # Speed over ground (knots)
                    if parts[5]:
                        self.speed_over_ground_knots = float(parts[5])

                    # Speed over ground (km/h)
                    if parts[7]:
                        self.speed_over_ground_kmh = float(parts[7])
                        
                    return True
                
                except ValueError:
                    return False
            else:
                return False
        return False
    
    def _parse_nmea(self, nmea_sentence):
        """
        Parses an NMEA sentence and extracts relevant data.

        Args:
            nmea_sentence: The NMEA sentence string.

        Returns:
            True if the sentence was parsed successfully, False otherwise.
        """
        if self.gps_mode == self.GPGGA:
            return self.__parse_nmea_gpgga(nmea_sentence)
        elif self.gps_mode == self.GPVTG:
            return self.___parse_nmea_gpvtg(nmea_sentence)
        elif self.gps_mode == self.GPRMC:
            return self.___parse_nmea_gprmc(nmea_sentence)
        return False

    @property
    def latitude(self):
        """
        Get the latitude.

        :return: float: The latitude.
        """
        return self.latitude

    @property
    def longitude(self):
        """
        Get the longitude.

        :return: The longitude (float).
        """
        return self.longitude
    
    @property
    def altitude(self):
        """
        Get the altitude.

        :return: The altitude (float).
        """
        return self.altitude

    @property
    def fix_quality(self):
        """
        Get the fix quality.

        :return: The fix quality (0: invalid, 1: GPS fix, 2: DGPS fix, etc.).
        """
        return self.fix_quality

    @property
    def satellites_in_use(self):
        """
        Get the number of satellites in use.

        :return: The number of satellites in use.
        """
        return self.satellites_in_use

    @property
    def timestamp(self):
        """
        Get the timestamp of the last fix.

        :return: The timestamp in HHMMSS.sss format.
        """
        return self.timestamp
    
    @property
    def fix_status(self):
        """
        Get the fix status from GPRMC sentence.

        :return: 'A' for Active, 'V' for Void (invalid).
        """
        return self.fix_status

    @property
    def speed_knots(self):
        """
        Get the speed over ground in knots.

        :return: The speed over ground in knots (float).
        """
        return self.speed_over_ground_knots

    @property
    def speed_kmh(self):
        """
        Get the speed over ground in km/h.

        :return: The speed over ground in km/h (float).
        """
        return self.speed_over_ground_kmh

    @property
    def course(self):
        """
        Get the course over ground in degrees.

        :return: The course over ground in degrees (float).
        """
        return self.course_over_ground

class Basic:
    class Buzzer:
        def __init__(self):
            self._pin = machine.Pin('P0', machine.Pin.OUT, value=0)

        def on(self):
            self._pin.value(1)

        def off(self):
            self._pin.value(0)

        def beep(self, delay, on=50, off=10):            
            self.on()
            t_on = utime.ticks_ms()
            t_off = 0
            
            while delay:               
                if t_on and utime.ticks_ms() - t_on > on:        
                    self.off()
                    t_on = 0
                    t_off = utime.ticks_ms()
                    
                if t_off and utime.ticks_ms() - t_off > off:
                    t_on = utime.ticks_ms()
                    t_off = 0
                                
                    self.on()  
                    delay -= 1
                        
            self.off()      

    class _I2CtoGPIO:
        """
        MicroPython driver for PCA9535 and PCA9535C 16-bit I2C I/O expanders.
        """
        PCA9535_ADDR = const(0x24)
        
        def __init__(self):
            self._i2c = machine.I2C(1)
            self._i2c.writeto_mem(Basic._I2CtoGPIO.PCA9535_ADDR, 0x06, bytes([0xFF]))
            self._i2c.writeto_mem(Basic._I2CtoGPIO.PCA9535_ADDR, 0x07, bytes([0x00]))
        
        def read(self):
            """
            Reads a register or registers.
            
            :reeturn: A bytes object containing the register value(s).
            """
            return ~self._i2c.readfrom_mem(Basic._I2CtoGPIO.PCA9535_ADDR, 0x00, 1)[0] & 0x03

        def write(self, n):
            """
            Writes to a register or registers.
            :param n: The value(s) to write (int or bytes).
            """
            self._i2c.writeto_mem(Basic._I2CtoGPIO.PCA9535_ADDR, 0x03, bytes([0xFF & ~n]))
        
    class _LedIter():
        def __init__(self, leds):
            self._index = 0
            self._leds = leds
            
        def on(self):
            self._leds[self._index-1] = True
        
        def off(self):
            self._leds[self._index-1] = False
            
    class Leds(_I2CtoGPIO):
        def __init__(self):
            super().__init__()
            self._stat = 0x00
            self._count = 8
    
        def __call__(self):
            return tuple([((self._stat >> i) & 0x1) == 1 for i in range(7+1)])
        
        def __getitem__(self, index):
            return ((self._stat >> index) & 0x01) == 1
        
        def __setitem__(self, index, value):
            if value:
                self._stat |= (1 << index)
            else:
                self._stat &= ~(1 << index)
            self.write(self._stat)

        def __iter__(self):
            self._led = Basic._LedIter(self)
            return self
        
        def __next__(self):
            if self._led._index < self._count:
                self._led._index += 1
                return self._led
            else:
                raise StopIteration

        def write(self, n):
            self._stat = n
            super().write(self._stat)
            
        def clear(self):
            self._stat = 0x00
            self.write(self._stat)

    class Buttons(_I2CtoGPIO):
        def __init__(self):
            super().__init__()

        def __getitem__(self, index):
            bits = self.read()
            return (bits & 0x01) if index == 0 else ((bits >> 1) & 0x01) if index == 1 else None
        
        def __call__(self):
            bits = self.read()        
            return bits & 0x01, (bits >> 1) & 0x01

