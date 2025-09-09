import uos
import utime


clamp = lambda val, lo, hi: lo if val < lo else hi if val > hi else val

map = lambda x, min_i, max_i, min_o, max_o: (x - min_i) * (max_o - min_o) / (max_i - min_i) + min_o

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

def rand(size:int=4) -> int:
    return int.from_bytes(uos.urandom(size), "big")

def intervalChecker(interval_ms:int) -> callable:
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
        def rgb(cls, r:int, g:int, b:int) -> str:
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
        def rgb(cls, r:int, g:int, b:int) -> str:             
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
        def up(cls, n:int) -> str:
            return "\u001b[{}A".format(n)

        @classmethod
        def down(cls, n:int) -> str:
            return "\u001b[{}B".format(n)

        @classmethod
        def right(cls, n:int) -> str:
            return "\u001b[{}C".format(n)

        @classmethod
        def left(cls, n:int) -> str:
            return "\u001b[{}D".format(n)
        
        @classmethod
        def next_line(cls, n:int) -> str:
            return "\u001b[{}E".format(n)

        @classmethod
        def prev_line(cls, n:int) -> str:
            return "\u001b[{}F".format(n)
                
        @classmethod
        def to(cls, row:int, colum:int) -> str:
            return "\u001b[{};{}H".format(row, colum)


class SlipEncoder:
    END      = 0xC0
    ESC      = 0xDB
    ESC_END  = 0xDC
    ESC_ESC  = 0xDD

    @staticmethod
    def encode(payload: bytes) -> bytes:
        out = bytearray([SlipEncoder.END])       # leading END
        for b in payload:
            if b == SlipEncoder.END:
                out += bytes([SlipEncoder.ESC, SlipEncoder.ESC_END])
            elif b == SlipEncoder.ESC:
                out += bytes([SlipEncoder.ESC, SlipEncoder.ESC_ESC])
            else:
                out.append(b)
        out.append(SlipEncoder.END)              # trailing END
        return bytes(out)


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
                    # END  frame 
                    frames.append(bytes(self._buf))
                    self._buf[:] = b''
                    self._in_frame = False
                else:
                    # junk end. start frame
                    self._in_frame = True
            else:
                if self._in_frame:
                    self._buf.append(b)
                # else: junk, byte invalied
        return frames


class RingBuffer:
    def __init__(self, size:int):
        self._buf = bytearray(size)
        self._size = size
        self._head = 0
        self._tail = 0

    def put(self, data:bytes):
        for b in data:
            nxt = (self._head + 1) % self._size
            if nxt == self._tail:
                self._tail = (self._tail + 1) % self._size
            self._buf[self._head] = b
            self._head = nxt

    def avail(self) -> int:
        return (self._head - self._tail) % self._size

    def get(self, n:int=1) -> bytes:
        n = min(n, self.avail())
        out = self._buf[self._tail:self._tail + n] \
            if self._tail + n <= self._size else \
            self._buf[self._tail:] + self._buf[:(self._tail+n)%self._size]
        self._tail = (self._tail + n) % self._size
        return bytes(out)

    def peek(self, n:int = 1) -> bytes:
        n = min(n, self.avail())
        if n == 0:
            return b''

        if self._tail + n <= self._size:
            return bytes(self._buf[self._tail:self._tail + n])

        part1 = self._buf[self._tail:]
        part2 = self._buf[:(self._tail + n) % self._size]
        return bytes(part1 + part2)
    
    def get_until(self, pattern:bytes, max_size:int|None=None) -> bytes|None:
        plen = len(pattern)
        total = self.avail()
        if total < plen:
            return None

        view = self.peek(total) 
        idx  = view.find(pattern)
        if idx == -1:
            return None 

        length = idx + plen
        if max_size and length > max_size:
            length = max_size

        return self.get(length)
