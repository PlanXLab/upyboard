import uos
import utime
import micropython


__version__ = "1.0.0"
__author__ = "PlanXLab Development Team"


@micropython.native
def clamp(val: float, lo: float, hi: float) -> float:
    if lo > hi:
        raise ValueError("Lower bound must be <= upper bound")
    return lo if val < lo else hi if val > hi else val

@micropython.native
def map(x: float, min_i: float, max_i: float, min_o: float, max_o: float) -> float:
    if max_i == min_i:
        raise ZeroDivisionError("Input range cannot be zero")
    return (x - min_i) * (max_o - min_o) / (max_i - min_i) + min_o

@micropython.native
def xrange(start: float, stop: float | None = None, step: float | None = None) -> iter[float]:
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
    if size <= 0 or size > 8:
        raise ValueError("Size must be between 1 and 8 bytes")
    
    return int.from_bytes(uos.urandom(size), "big")

@micropython.native
def hsv_to_rgb(h: float, s: float, v: float) -> tuple[int,int,int]:
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
    class FG:
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
            if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                raise ValueError("RGB components must be in range 0-255")
            return "\u001b[38;2;{};{};{}m".format(r, g, b)

    class BG:
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
            if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                raise ValueError("RGB components must be in range 0-255")
            return "\u001b[48;2;{};{};{}m".format(r, g, b)

    class OP:
        RESET = "\u001b[0m"
        BOLD = "\u001b[1m"
        UNDER_LINE = "\u001b[4m"
        REVERSE = "\u001b[7m"
        CLEAR = "\u001b[2J"
        CLEAR_LINE = "\u001b[2K"
        TOP = "\u001b[0;0H"

        @classmethod
        def up(cls, n: int) -> str:
            return "\u001b[{}A".format(n)

        @classmethod
        def down(cls, n: int) -> str:
            return "\u001b[{}B".format(n)

        @classmethod
        def right(cls, n: int) -> str:
            return "\u001b[{}C".format(n)

        @classmethod
        def left(cls, n: int) -> str:
            return "\u001b[{}D".format(n)
        
        @classmethod
        def next_line(cls, n: int) -> str:
            return "\u001b[{}E".format(n)

        @classmethod
        def prev_line(cls, n: int) -> str:
            return "\u001b[{}F".format(n)
                
        @classmethod
        def to(cls, row: int, column: int) -> str:
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
    END      = 0xC0
    ESC      = 0xDB
    ESC_END  = 0xDC
    ESC_ESC  = 0xDD

    @staticmethod
    def encode(payload: bytes) -> bytes:
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
    END      = 0xC0
    ESC      = 0xDB
    ESC_END  = 0xDC
    ESC_ESC  = 0xDD

    def __init__(self) -> None:
        self._buf = bytearray()
        self._escaped = False
        self._in_frame = False

    def reset(self) -> None:
        self._buf[:] = b''
        self._escaped = False
        self._in_frame = False

    def feed(self, chunk: bytes) -> list[bytes]:
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


@micropython.native
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
    def __init__(self, size: int) -> None:
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
        return (self._head - self._tail) % self._size

    @micropython.native
    def get(self, n: int = 1) -> bytes:
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
