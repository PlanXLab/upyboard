from . import (
    utime,
    ext
)

class ServoFnd:
    """
    7-segment FND display using 7 servo motors and ServoMotor class.
    """
    _SEG_ANGLE = [
        (170, 96),   # A (OFF, ON)
        (0, 82),     # B
        (170, 90),   # C
        (170, 100),  # D
        (0, 90),     # E
        (170, 86),   # F
        (0, 90),     # G
    ]

    _CHAR_PATTERN = {
        '0': [1,1,1,1,1,1,0],
        '1': [0,1,1,0,0,0,0],
        '2': [1,1,0,1,1,0,1],
        '3': [1,1,1,1,0,0,1],
        '4': [0,1,1,0,0,1,1],
        '5': [1,0,1,1,0,1,1],
        '6': [1,0,1,1,1,1,1],
        '7': [1,1,1,0,0,0,0],
        '8': [1,1,1,1,1,1,1],
        '9': [1,1,1,1,0,1,1],
        'A': [1,1,1,0,1,1,1],
        'B': [0,0,1,1,1,1,1],  # b
        'C': [1,0,0,1,1,1,0],
        'D': [0,1,1,1,1,0,1],  # d
        'E': [1,0,0,1,1,1,1],
        'F': [1,0,0,0,1,1,1],        
        'G': [1,0,1,1,1,1,0],
        'H': [0,1,1,0,1,1,1],  # h
        'I': [0,0,0,0,1,1,0],
        'J': [0,1,1,1,0,0,0],
        'K': [1,0,1,0,1,1,1],
        'L': [0,0,0,1,1,1,0],
        'M': [1,0,1,0,1,0,1],  
        'N': [0,0,1,0,1,0,1],  # n
        'O': [1,1,1,1,1,1,0],
        'P': [1,1,0,0,1,1,1],
        'Q': [1,1,1,0,0,1,1],
        'R': [0,0,0,0,1,0,1],  # r
        'S': [1,0,1,1,0,1,1],
        'T': [0,0,0,1,1,1,1],
        'U': [0,1,1,1,1,1,0],
        'V': [0,1,0,1,0,1,0],  
        'W': [0,1,0,1,0,1,1],  
        'X': [0,0,1,0,1,0,0],  #
        'Y': [0,1,1,1,0,1,1],
        'Z': [1,1,0,1,1,0,0], 
        '_': [0,0,0,1,0,0,0],
        ' ': [0,0,0,0,0,0,0],
    }

    def __init__(self, a, b, c, d, e, f, g):
        """
        Initialize the ServoFnd with 7 servo motors for each segment.
        
        :param a: Pin for segment A
        :param b: Pin for segment B
        :param c: Pin for segment C
        :param d: Pin for segment D 
        :param e: Pin for segment E
        :param f: Pin for segment F
        :param g: Pin for segment G

        Segment order
        -------------
             A
            ___
        E |  F  | B
            ___
        D |     | C
            ___
             D
        """
        self.servo = ext.ServoMotor([a, b, c, d, e, f, g])
        self.chr_delay_ms = 300
        
    def __set_segment(self, idx, on):
        off_angle, on_angle = self._SEG_ANGLE[idx]
        angle = on_angle if on else off_angle
        self.servo[idx].angle = angle
        utime.sleep_ms(20)

    def calibrate(self, segment_idx, off_angle=170, on_angle=96):
        """
        Calibrate a specific segment of the FND.
        
        :param segment_idx: Index of the segment to calibrate (0-6)
        :param off_angle: Angle for the segment when OFF
        :param on_angle: Angle for the segment when ON
        """
        if segment_idx < 0 or segment_idx >= len(self._SEG_ANGLE):
            raise ValueError("Segment index must be between 0 and 6.")
        self._SEG_ANGLE[segment_idx] = (off_angle, on_angle)

    def display(self, char):
        """
        Display a digit (0~9) or supported letter (A-Z, -, ' ') on the servo FND.
        
        :param char: Character to display (0-9, A-Z, -, ' ')
        :raises ValueError: If the character is not supported.
        """
        c = str(char).upper()
        pattern = self._CHAR_PATTERN.get(c)
        if pattern is None:
            pattern = self._CHAR_PATTERN[' ']
        for idx in range(7):
            self.__set_segment(idx, bool(pattern[idx]))
        utime.sleep_ms(self.chr_delay_ms)
