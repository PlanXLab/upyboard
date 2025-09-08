from . import (
    sys, math, utime, array,
    gc, machine, micropython, rp2
)


gc.threshold(20480)

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def __ws2812_pio():
    T1, T2, T3 = 2, 5, 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

class WS2812Matrix:
    DEFAULT_FONT: str = "lib/ticle/vga2_bold_16x16"

    def __init__(self, pin_sm_pairs: list[tuple[int, int]], *,
                 panel_width: int = 16, panel_height: int = 16,
                 grid_width: int = 1, grid_height: int = 1,
                 zigzag: bool = False, origin: str = "top_left",
                 brightness: float = 0.25, font: str = DEFAULT_FONT) -> None:

        if origin not in ("top_left", "top_right", "bottom_left", "bottom_right"):
            raise ValueError("origin must be top_left/top_right/bottom_left/bottom_right")
        
        for _, sm_id in pin_sm_pairs:
            if not (0 <= sm_id <= 11):
                raise ValueError("State machine ID out of range (0-11)")
 
        self.__panel_width = panel_width
        self.__panel_height = panel_height
        self.__grid_width = grid_width
        
        self._fb_width = panel_width * grid_width
        self._fb_height = panel_height * grid_height 
        self._fb_length = self._fb_width * self._fb_height
        self._fb = array.array("I", [0] * (self._fb_length))
        self._fb_dirty = False

        self.__load_font(font)

        self.__zigzag = zigzag
        self.__origin: str = origin

        self.brightness = brightness
  
        self.__sms: list[rp2.StateMachine] = []
        self.__tx_bufs: list[array.array] = []

        self.__panels_per_sm: int = math.ceil((grid_width * grid_height) / len(pin_sm_pairs))
        self.__pixels_per_panel: int = panel_width * panel_height

        for pin_no, sm_id in pin_sm_pairs:
            try:
                pin = machine.Pin(pin_no, machine.Pin.OUT, machine.Pin.PULL_DOWN)
                sm = rp2.StateMachine(sm_id, __ws2812_pio, freq=8_000_000, sideset_base=pin)
                sm.active(1)
                self.__sms.append(sm)
                buf_len = self.__pixels_per_panel * self.__panels_per_sm
                self.__tx_bufs.append(array.array("I", [0] * buf_len))
            except OSError as e:
                for sm2 in self.__sms:
                    try:
                        sm2.active(0)
                    except:
                        pass
                
                raise OSError(f"Failed init pin {pin_no} / SM {sm_id}: {e}")

        self.__sm_ids: list[int] = [sm_id for _, sm_id in pin_sm_pairs]
        self.__dmas: list[rp2.DMA] = [rp2.DMA() for _ in self.__sms]
        self.__build_pix_maps()
        
        self.__ink_width_cache: dict[tuple, int] = {}
        
    class _PixelView:
        def __init__(self, parent, x, y):
            self._parent = parent
            self._x = x
            self._y = y

        @property
        def value(self):
            fb = self._parent._fb
            w = self._parent._fb_width
            packed = fb[self._y * w + self._x]
            g = (packed >> 24) & 0xFF
            r = (packed >> 16) & 0xFF
            b = (packed >> 8) & 0xFF
            return r, g, b

        @value.setter
        def value(self, color):
            self._parent._set_pixel(self._x, self._y, color)

    def __getitem__(self, pos: tuple[int, int]) -> "_PixelView":
        if isinstance(pos, tuple) and len(pos) == 2:
            x, y = pos
            
            if not (0 <= x < self._fb_width and 0 <= y < self._fb_height):
                raise IndexError("Pixel coordinates out of bounds")
            
            return WS2812Matrix._PixelView(self, x, y)
        else:
            raise TypeError("Index must be a tuple of (x, y) coordinates")


    @micropython.native
    def _set_pixel(self, x: int, y: int, color: int | tuple[int, int, int]) -> None:
        r, g, b = self.__normalize_color(color)
        packed = self.__pack_grb(r, g, b)
        
        if 0 <= x < self._fb_width and 0 <= y < self._fb_height:
            self._fb[y * self._fb_width + x] = packed
            self._fb_dirty = True

    @property
    def width(self) -> int: 
        return self._fb_width

    @property
    def height(self) -> int: 
        return self._fb_height

    @property
    def brightness(self) -> float: 
        return self.__brightness

    @brightness.setter
    def brightness(self, value: float) -> None:
        self.__brightness = max(0.0, min(value, 1.0))
        self.__btab = bytes(int(i * self.__brightness + 0.5) for i in range(256))

    @micropython.native
    def update(self, wait: bool = True) -> None:
        dma_busy = any(dma.active() for dma in self.__dmas)

        if dma_busy and not wait:
            return
        
        if dma_busy and wait:
            while any(dma.active() for dma in self.__dmas):
                pass

        if self._fb_dirty:
            self.__flush_fb_to_txb()
            self._fb_dirty = False

        for sm_id, sm, src, dma in zip(self.__sm_ids, self.__sms, self.__tx_bufs, self.__dmas):
            if not dma.active():
                ctrl = dma.pack_ctrl(size=2, inc_write=False, treq_sel=self.__pio_dreq(sm_id), bswap=False)
                dma.config(read=src, write=sm, count=len(src), ctrl=ctrl, trigger=True)

        if wait:
            while any(dma.active() for dma in self.__dmas):
                pass

    def clear(self) -> None:
        self.fill(0)
        self.update(True)
    
    @micropython.native
    def fill(self, color: int | tuple[int, int, int]) -> None:
        r, g, b = self.__normalize_color(color)
        packed = self.__pack_grb(r, g, b)
        self.__fill32(self._fb, 0, packed, self._fb_length)
        self._fb_dirty = True

    def set_font(self, font_src: str | object) -> None:
        self.__load_font(font_src)
        
    def deinit(self) -> None:
        self.clear()
        utime.sleep_ms(50)
        
        for sm in self.__sms:
            sm.active(0)

    @micropython.native
    def draw_line(self, x0:int, y0:int, x1:int, y1:int, color):
        r, g, b = self.__normalize_color(color)
        packed = self.__pack_grb(r, g, b)
        w = self._fb_width
        h = self._fb_height
        fb = self._fb

        if y0 == y1:
            y = y0
            
            if 0 <= y < h:
                if x0 > x1:
                    x0, x1 = x1, x0
            
                x0 = max(0, x0)
                x1 = min(w - 1, x1)
            
                if x0 <= x1:
                    span = x1 - x0 + 1
                    base = y * w + x0
                    self.__fill32(fb, base, packed, span)
                    self._fb_dirty = True
            return

        if x0 == x1:
            x = x0
          
            if not (0 <= x < w):
                return

            if y0 > y1:
                y0, y1 = y1, y0
          
            y0 = max(0, y0)
            y1 = min(h - 1, y1)
          
            if y0 > y1:
                return

            span   = y1 - y0 + 1
            offset = y0 * w + x
            self.__fill32(fb, offset, packed, span)
            self._fb_dirty = True
            return

        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            if 0 <= x0 < w and 0 <= y0 < h:
                fb[y0 * w + x0] = packed
            
            if x0 == x1 and y0 == y1:
                break
            
            e2 = err << 1
            
            if e2 >= dy:
                err += dy
                x0  += sx
            
            if e2 <= dx:
                err += dx
                y0  += sy

        self._fb_dirty = True


    @micropython.native
    def draw_line_polar(self, cx:int, cy:int, length:int, angle_deg:int, color):
        w, h = self._fb_width, self._fb_height
        fb = self._fb

        half = length >> 1

        if half <= 0:
            return

        angle = angle_deg % 360

        if angle % 90 == 0:
            r, g, b = self.__normalize_color(color)
            #packed  = self.__pack_grb(r, g, b)
            packed  = self.__pack_grb_v(r, g, b)

            if angle == 0 or angle == 180:  # 가로
                y = cy
                
                if not (0 <= y < h):
                    return
                
                x0 = cx - half
                x1 = cx + half
                
                if angle == 180:
                    x0, x1 = x1, x0
                
                if x0 > x1:
                    x0, x1 = x1, x0
                
                x0 = max(0, x0)
                x1 = min(w - 1, x1)
                
                if x0 <= x1:
                    span = x1 - x0 + 1
                    base = y * w + x0
                    fb[base: base + span] = self.__get_row_block(span, packed)
                    self._fb_dirty = True
                
                return
            else:
                x = cx
                
                if not (0 <= x < w):
                    return
                
                y0 = cy - half
                y1 = cy + half
                
                if angle == 270:
                    y0, y1 = y1, y0
                
                if y0 > y1:
                    y0, y1 = y1, y0
                
                y0 = max(0, y0)
                y1 = min(h - 1, y1)
                
                if y0 <= y1:
                    r, g, b = self.__normalize_color(color)
                    #packed  = self.__pack_grb(r, g, b)
                    packed  = self.__pack_grb_v(r, g, b)

                    for py in range(y0, y1 + 1):
                        fb[py * w + x] = packed
                
                    self._fb_dirty = True
                
                return

        rad  = math.radians(angle)
        dxh  = int(round(math.cos(rad) * half))
        dyh  = int(round(math.sin(rad) * half))

        x0 = cx - dxh
        y0 = cy - dyh
        x1 = cx + dxh
        y1 = cy + dyh

        if (x0 < 0 and x1 < 0) or (x0 >= w and x1 >= w):
            return
        
        if (y0 < 0 and y1 < 0) or (y0 >= h and y1 >= h):
            return
        
        self.draw_line(x0, y0, x1, y1, color)

    @micropython.native
    def draw_rect(self, x: int, y: int, w: int, h: int, outline: int | tuple[int, int, int], *, fill: tuple[int | tuple[int, int, int]] = None) -> None:
        if w <= 0 or h <= 0:
            return

        x1, y1 = x, y
        x2, y2 = x + w - 1, y + h - 1
        dw, dh = self._fb_width, self._fb_height
        fb = self._fb

        pix_out = self.__pack_grb(*self.__normalize_color(outline))
        pix_fill = (self.__pack_grb(*self.__normalize_color(fill)) if fill is not None else None)

        def hspan(y_row: int, xa: int, xb: int, packed: int):
            if not (0 <= y_row < dh):
                return
            
            if xb < 0 or xa >= dw:
                return
            
            if xa < 0:  
                xa = 0
        
            if xb >= dw: 
                xb = dw - 1
     
            if xa > xb:
                return
     
            self.__fill32(fb, y_row*dw + xa, packed, xb - xa + 1)

        hspan(y1, x1, x2, pix_out)
     
        if y2 != y1:
            hspan(y2, x1, x2, pix_out)

        if w >= 2:
            xa, xb = x1, x2
            
            for py in range(max(y1, 0)+1, min(y2, dh-1)):
                row = py * dw
            
                if 0 <= xa < dw:
                    fb[row + xa] = pix_out
            
                if 0 <= xb < dw and xb != xa:
                    fb[row + xb] = pix_out

        if pix_fill is not None and w > 2 and h > 2:
            fill_xa = x1 + 1
            fill_xb = x2 - 1
            
            for py in range(max(y1+1, 0), min(y2, dh-1)):
                hspan(py, fill_xa, fill_xb, pix_fill)

        self._fb_dirty = True


    @micropython.native
    def draw_rect_polar(self, cx: int, cy: int, w: int, h: int, angle_deg: float, outline, *, fill=None) -> None:
        if w <= 0 or h <= 0:
            return

        if angle_deg % 90 == 0:
            self.draw_rect(cx - (w >> 1), cy - (h >> 1), w, h, outline, fill=fill)
            return

        half_w = (w - 1) / 2.0
        half_h = (h - 1) / 2.0
        vx = (-half_w, +half_w, +half_w, -half_w)
        vy = (-half_h, -half_h, +half_h, +half_h)

        rad = math.radians(angle_deg)
        ca, sa = math.cos(rad), math.sin(rad)

        def rot(ix, iy):
            return cx + ix * ca - iy * sa, cy + ix * sa + iy * ca

        pts = [rot(vx[i], vy[i]) for i in range(4)]

        fb = self._fb
        dw, dh = self._fb_width, self._fb_height
        p_out = self.__pack_grb(*self.__normalize_color(outline))
        p_fill = (self.__pack_grb(*self.__normalize_color(fill)) if fill is not None else None)

        tris = [(pts[0], pts[1], pts[2]), (pts[0], pts[2], pts[3])]

        def edge_x(y_scan, v0, v1):
            x0, y0 = v0;  x1, y1 = v1
   
            if y0 == y1:
                return None
   
            if y0 > y1:
                x0, y0, x1, y1 = x1, y1, x0, y0
   
            if not (y0 <= y_scan < y1):
                return None
   
            t = (y_scan - y0) / (y1 - y0)
   
            return x0 + t * (x1 - x0)

        y_min = max(0, int(math.ceil(min(p[1] for p in pts))))
        y_max = min(dh - 1, int(math.floor(max(p[1] for p in pts))))

        if p_fill is not None:
            for py in range(y_min, y_max + 1):
                xs = []
                
                for v0, v1, v2 in tris:
                    for e0, e1 in ((v0, v1), (v1, v2), (v2, v0)):
                        xx = edge_x(py + 0.5, e0, e1)
                
                        if xx is not None:
                            xs.append(xx)
                
                if len(xs) < 2:
                    continue
                
                xs.sort()
                xa = int(math.ceil(xs[0])) + 1
                xb = int(math.floor(xs[-1])) - 1
            
                if xa <= xb:
                    if xa < 0: 
                        xa = 0
                  
                    if xb >= dw: 
                        xb = dw - 1
                  
                    if xa <= xb:
                        self.__fill32(fb, py * dw + xa, p_fill, xb - xa + 1)

        def put(px, py):
            if 0 <= px < dw and 0 <= py < dh:
                fb[py * dw + px] = p_out

        for py in range(y_min, y_max + 1):
            xs = []
            
            for v0, v1, v2 in tris:
                for e0, e1 in ((v0, v1), (v1, v2), (v2, v0)):
                    xx = edge_x(py + 0.5, e0, e1)
            
                    if xx is not None:
                        xs.append(xx)
            
            if len(xs) < 2:
                continue
            
            xs.sort()
            xl = int(math.floor(xs[0] + 0.5))
            xr = int(math.floor(xs[-1] + 0.5))
            put(xl, py)
            
            if xr != xl:
                put(xr, py)

        self._fb_dirty = True


    @micropython.native
    def draw_ellipse(self, cx: int, cy: int, rx: int, ry: int | None = None, outline=(255, 255, 255), *, fill=None, angle_deg: float = 0.0):
        if ry is None:
            ry = rx
            
        if rx <= 0 or ry <= 0:
            return

        dw, dh = self._fb_width, self._fb_height
        fb = self._fb
        p_edge = self.__pack_grb(*self.__normalize_color(outline))
        p_fill = (self.__pack_grb(*self.__normalize_color(fill)) if fill is not None else None)

        if angle_deg == 0.0:
            rx2, ry2 = rx * rx, ry * ry
            two_rx2  = rx2 << 1
            two_ry2  = ry2 << 1

            x, y = 0, ry
            dx, dy = 0, two_rx2 * y
            d1 = ry2 - rx2 * ry + (rx2 >> 2)

            def pset(px: int, py: int):
                if 0 <= px < dw and 0 <= py < dh:
                    fb[py * dw + px] = p_edge

            while dx < dy:
                if p_fill is not None:
                    xa, xb = cx - x, cx + x
                    
                    if xa < 0: 
                        xa = 0
                    
                    if xb >= dw: 
                        xb = dw - 1
                    
                    if xa <= xb:
                        span = xb - xa + 1
                        self.__fill32(fb, (cy + y) * dw + xa, p_fill, span)
                        self.__fill32(fb, (cy - y) * dw + xa, p_fill, span)

                pset(cx + x, cy + y);  pset(cx - x, cy + y)
                pset(cx + x, cy - y);  pset(cx - x, cy - y)
                x  += 1
                dx += two_ry2
         
                if d1 < 0:
                    d1 += dx + ry2
                else:
                    y  -= 1
                    dy -= two_rx2
                    d1 += dx - dy + ry2

            d2 = (ry2 * (x + 0.5)**2) + (rx2 * (y - 1)**2) - rx2 * ry2
         
            while y >= 0:
                if p_fill is not None:
                    xa, xb = cx - x, cx + x
                
                    if xa < 0: 
                        xa = 0
                    
                    if xb >= dw: 
                        xb = dw - 1
                    
                    if xa <= xb:
                        span = xb - xa + 1
                        self.__fill32(fb, (cy + y) * dw + xa, p_fill, span)
                        self.__fill32(fb, (cy - y) * dw + xa, p_fill, span)

                pset(cx + x, cy + y);  pset(cx - x, cy + y)
                pset(cx + x, cy - y);  pset(cx - x, cy - y)

                y -= 1
                
                if d2 > 0:
                    d2 += rx2 - (two_rx2 * y)
                else:
                    x += 1
                    d2 += two_ry2 * x + rx2 - two_rx2 * y

            self._fb_dirty = True
            return

        th  = math.radians(angle_deg)
        ct, st = math.cos(th), math.sin(th)
        inv_rx2, inv_ry2 = 1 / (rx * rx), 1 / (ry * ry)
        A = ct * ct * inv_rx2 + st * st * inv_ry2
        B = 2 * ct * st * (inv_rx2 - inv_ry2)
        C = st * st * inv_rx2 + ct * ct * inv_ry2
        h_bound = abs(rx * st) + abs(ry * ct)
        y0 = max(0, int(cy - h_bound) - 1)
        y1 = min(dh - 1, int(cy + h_bound) + 1)

        for py in range(y0, y1 + 1):
            dy = py - cy
            By = B * dy
            K  = C * dy * dy - 1
            D  = By * By - 4 * A * K
            
            if D < 0:
                continue

            sqrtD = math.sqrt(D)
            xL = (-By - sqrtD) / (2 * A)
            xR = (-By + sqrtD) / (2 * A)
            oxl = int(math.ceil(cx + xL))
            oxr = int(math.floor(cx + xR))
            
            if oxr < 0 or oxl >= dw:
                continue

            xl = max(0, oxl)
            xr = min(dw - 1, oxr)
            
            if xl > xr:
                continue

            if p_fill is not None:
                self.__fill32(fb, py * dw + xl, p_fill, xr - xl + 1)

            if 0 <= oxl < dw:
                fb[py * dw + oxl] = p_edge
            
            if oxr != oxl and 0 <= oxr < dw:
                fb[py * dw + oxr] = p_edge

        self._fb_dirty = True


    def draw_circle(self, cx: int, cy: int, r: int, color: int | tuple[int, int, int], *, fill: tuple[int | tuple[int, int, int]] = None) -> None:
        self.draw_ellipse(cx, cy, r, r, color, fill=fill, angle_deg=0.0)

    @micropython.native
    def draw_bitmap_1bit(self, data:bytes|bytearray|memoryview, 
                         width:int, height:int, 
                         x:int=0, y:int=0, 
                         outline=(255,255,255),
                         customize: str | None = None):
        fb, W, H = self._fb, self._fb_width, self._fb_height
        if not outline:
            return

        stride = (width + 7) // 8
        r, g, b = self.__normalize_color(outline)
        px_outline = self.__pack_grb(r, g, b)

        def _bit_on(ix: int, iy: int) -> int:
            if ix < 0 or ix >= width or iy < 0 or iy >= height:
                return 0
            off = iy * stride + (ix >> 3)
            return 1 if (data[off] & (0x80 >> (ix & 7))) else 0

        if customize == 'tile':
            for py in range(H):
                row_base = py * W
                sy = (py - y) % height
                for px in range(W):
                    sx = (px - x) % width
                    if _bit_on(sx, sy):
                        if (_bit_on(sx - 1, sy) == 0 or
                            _bit_on(sx + 1, sy) == 0 or
                            _bit_on(sx, sy - 1) == 0 or
                            _bit_on(sx, sy + 1) == 0 or
                            _bit_on(sx - 1, sy - 1) == 0 or
                            _bit_on(sx + 1, sy - 1) == 0 or
                            _bit_on(sx - 1, sy + 1) == 0 or
                            _bit_on(sx + 1, sy + 1) == 0):
                            fb[row_base + px] = px_outline
            self._fb_dirty = True
            return

        if customize == 'stretch':
            for py in range(H):
                row_base = py * W
                sy = (py * height) // H
                for px in range(W):
                    sx = (px * width) // W
                    if _bit_on(sx, sy):
                        sx_l = ((px - 1) * width) // W
                        sx_r = ((px + 1) * width) // W
                        sy_u = ((py - 1) * height) // H
                        sy_d = ((py + 1) * height) // H
                        if (_bit_on(sx_l, sy) == 0 or
                            _bit_on(sx_r, sy) == 0 or
                            _bit_on(sx, sy_u) == 0 or
                            _bit_on(sx, sy_d) == 0 or
                            _bit_on(sx_l, sy_u) == 0 or
                            _bit_on(sx_r, sy_u) == 0 or
                            _bit_on(sx_l, sy_d) == 0 or
                            _bit_on(sx_r, sy_d) == 0):
                            fb[row_base + px] = px_outline
            self._fb_dirty = True
            return

        for yy in range(height):
            py = y + yy
            if py < 0 or py >= H:
                continue
            row_base = py * W
            byte_index = yy * stride
            bits = 0
            mask = 0
            for xx in range(width):
                px = x + xx
                if (xx & 7) == 0:
                    bits = data[byte_index]
                    byte_index += 1
                    mask = 0x80
                else:
                    mask >>= 1
                if px < 0 or px >= W:
                    continue
                if bits & mask:
                    if (_bit_on(xx - 1, yy) == 0 or
                        _bit_on(xx + 1, yy) == 0 or
                        _bit_on(xx, yy - 1) == 0 or
                        _bit_on(xx, yy + 1) == 0 or
                        _bit_on(xx - 1, yy - 1) == 0 or
                        _bit_on(xx + 1, yy - 1) == 0 or
                        _bit_on(xx - 1, yy + 1) == 0 or
                        _bit_on(xx + 1, yy + 1) == 0):
                        fb[row_base + px] = px_outline

        self._fb_dirty = True

    @micropython.native
    def draw_bitmap_color(self, data: bytes | bytearray | memoryview, 
                          width: int, height: int, 
                          x: int = 0, y: int = 0,
                          customize: str | None = None) -> None:
        W, H = self._fb_width, self._fb_height
        fb = self._fb
        stride = width * 3

        if customize == 'tile':
            for py in range(H):
                dst_row = py * W
                sy = (py - y) % height
                src_row_b = sy * stride

                for px in range(W):
                    sx = (px - x) % width
                    si = src_row_b + sx * 3
                    r = data[si]
                    g = data[si + 1]
                    b = data[si + 2]
                    fb[dst_row + px] = self.__pack_grb(r, g, b)

            self._fb_dirty = True
            return

        if customize == 'stretch':
            for py in range(H):
                dst_row = py * W
                sy = (py * height) // H
                src_row_b = sy * stride

                for px in range(W):
                    sx = (px * width) // W
                    si = src_row_b + sx * 3
                    fb[dst_row + px] = self.__pack_grb(data[si], data[si + 1], data[si + 2])

            self._fb_dirty = True
            return

        for yy in range(height):
            py = y + yy

            if py < 0 or py >= H:
                continue

            src_off_b = yy * stride
            dst_off_i = py * W + x

            vis_x0 = 0
            vis_x1 = width

            if x + vis_x1 <= 0 or x + vis_x0 >= W:
                continue

            if x + vis_x0 < 0:
                vis_x0 = -x

            if x + vis_x1 > W:
                vis_x1 = W - x

            count = vis_x1 - vis_x0

            if count <= 0:
                continue

            src = src_off_b + vis_x0 * 3
            dst = dst_off_i + vis_x0

            for i in range(count):
                r = data[src]
                g = data[src + 1]
                b = data[src + 2]
                fb[dst + i] = self.__pack_grb(r, g, b)
                src += 3

        self._fb_dirty = True

    _SH_SPLIT_LR = 1
    _SH_SPLIT_TB = 2
    _SH_CHECKER  = 3
    _SH_SNOW     = 4

    @micropython.native
    def shader_split_lr(self, c_left, c_right, ratio_num=1, ratio_den=2):
        r1,g1,b1 = self.__normalize_color(c_left)
        r2,g2,b2 = self.__normalize_color(c_right)
        return (self._SH_SPLIT_LR, self.__pack_grb(r1,g1,b1), self.__pack_grb(r2,g2,b2), int(ratio_num), int(ratio_den))
    
    @micropython.native
    def shader_split_tb(self, c_top, c_bottom, ratio_num=1, ratio_den=2):
        r1,g1,b1 = self.__normalize_color(c_top)
        r2,g2,b2 = self.__normalize_color(c_bottom)
        return (self._SH_SPLIT_TB, self.__pack_grb(r1,g1,b1), self.__pack_grb(r2,g2,b2), int(ratio_num), int(ratio_den))
        
    @micropython.native
    def shader_checker(self, c1, c2, cell_w=2, cell_h=2):
        r1,g1,b1 = self.__normalize_color(c1)
        r2,g2,b2 = self.__normalize_color(c2)
        return (self._SH_CHECKER, self.__pack_grb(r1,g1,b1), self.__pack_grb(r2,g2,b2), int(cell_w), int(cell_h))
        
    @micropython.native
    def shader_snowflake(self,c1, c2, arm=1):
        r1,g1,b1 = self.__normalize_color(c1)
        r2,g2,b2 = self.__normalize_color(c2)
        return (self._SH_SNOW, self.__pack_grb(r1,g1,b1), self.__pack_grb(r2,g2,b2), int(arm))

    @micropython.native
    def draw_text(self, text: str, 
                  *, 
                  x: int = 0, y: int = 0,
                  fg: int | tuple[int, int, int] | callable = (255, 255, 255),
                  bg: int | tuple[int, int, int] = (0, 0, 0),
                  space_scale: float = 0.3,
                  right_margin: int = 1, 
                  left_margin: int = 0) -> None:
        fw, fh = self.font_width, self.font_height
        fb_w, fb_h = self._fb_width, self._fb_height
        fb = self._fb
        mask_base = fw - 1

        skip_drawing = (y + fh < 0) or (y >= fb_h)

        chars = list(text)
        mode, fg_tbl, packed_fg, fg_shader = self.__fg_mode(fg, text, chars)

        draw_bg = (bg is not None) and (not skip_drawing)
        packed_bg = 0 if not draw_bg else self.__pack_grb(*self.__normalize_color(bg))

        pen_x = x
        total_advance = 0

        for idx, ch in enumerate(chars):
            if ch == " ":
                adv = int(fw * space_scale)
                total_advance += adv
                pen_x += adv
                continue

            gi = self.__glyph_offset(ord(ch))
            blank_L, blank_R = self.__glyph_lr(gi)
            ink_l = blank_L
            ink_r = fw - 1 - blank_R
            ink_w = ink_r - ink_l + 1
            if ink_w <= 0:
                adv = left_margin + right_margin
                total_advance += adv
                pen_x += adv
                continue

            block_w = left_margin + ink_w + right_margin

            total_advance += block_w

            if not skip_drawing:
                if (pen_x + block_w) > 0 and (pen_x < fb_w):
                    if draw_bg:
                        bg_x = pen_x if pen_x >= 0 else 0
                        bg_w = (pen_x + block_w if (pen_x + block_w) < fb_w else fb_w) - bg_x
                        if bg_w > 0:
                            rr = 0
                            while rr < fh:
                                gy = y + rr
                                if 0 <= gy < fb_h:
                                    self.__fill32(fb, gy * fb_w + bg_x, packed_bg, bg_w)
                                rr += 1

                    dst_x = pen_x + left_margin
                    vis_l = 0 if dst_x >= 0 else -dst_x
                    vis_r = 0 if (dst_x + ink_w) <= fb_w else (dst_x + ink_w) - fb_w

                    if vis_l < (ink_w - vis_r):
                        cc_from = ink_l + vis_l
                        cc_to = ink_r - vis_r
                        px0 = dst_x + vis_l

                        if mode != 'shader':
                            pix = packed_fg if fg_tbl is None else fg_tbl[idx]
                            rr = 0
                            while rr < fh:
                                gy = y + rr
                                if 0 <= gy < fb_h:
                                    row_bits = self.__glyph_row_bits(gi, rr)
                                    self.__blit_row(memoryview(fb), gy * fb_w, int(row_bits),
                                                    mask_base, cc_from, cc_to, px0, pix)
                                rr += 1
                        else:
                            is_spec = (isinstance(fg_shader, (tuple, list)) and len(fg_shader) >= 1
                                    and isinstance(fg_shader[0], int) and 1 <= fg_shader[0] <= 4)
                            if is_spec:
                                m = int(fg_shader[0])
                                pa = int(fg_shader[1]); pb = int(fg_shader[2])
                                if m == int(self._SH_SPLIT_LR):
                                    rnum = int(fg_shader[3]); rden = int(fg_shader[4])
                                    p1 = (fw * rnum) // rden; p2 = 0
                                elif m == int(self._SH_SPLIT_TB):
                                    rnum = int(fg_shader[3]); rden = int(fg_shader[4])
                                    p1 = (fh * rnum) // rden; p2 = 0
                                elif m == int(self._SH_CHECKER):
                                    p1 = int(fg_shader[3]); p2 = int(fg_shader[4])
                                else:  # _SH_SNOW
                                    p1 = int(fg_shader[3]); p2 = 0

                                rr = 0
                                while rr < fh:
                                    gy = y + rr
                                    if 0 <= gy < fb_h:
                                        row_bits = self.__glyph_row_bits(gi, rr)
                                        self.__blit_row_shader_spec(memoryview(fb), gy * fb_w, int(row_bits),
                                            mask_base, cc_from, cc_to, px0, int(m), pa, pb, p1, p2,
                                            int(fw), int(fh), int(rr))
                                    rr += 1
                            else:
                                rr = 0
                                while rr < fh:
                                    gy = y + rr
                                    if 0 <= gy < fb_h:
                                        row_bits = self.__glyph_row_bits(gi, rr)
                                        fb_row_off = gy * fb_w
                                        xx = cc_from
                                        while xx <= cc_to:
                                            if ((int(row_bits) >> (mask_base - xx)) & 1) != 0:
                                                try:
                                                    col = fg(xx, rr, fw, fh, idx, ch)
                                                except TypeError:
                                                    xx += 1
                                                    continue
                                                r, g, b = self.__normalize_color(col)
                                                fb[fb_row_off + (px0 + (xx - cc_from))] = self.__pack_grb(r, g, b)
                                            xx += 1
                                    rr += 1

            pen_x += block_w

        if not skip_drawing:
            self._fb_dirty = True

        return int(total_advance)

    def draw_text_scroll(self, text: str, 
                         *, 
                         direction: str = "left", x: int = 0, y: int = 0,
                         fg: int | tuple[int, int, int] | callable,
                         bg: tuple[int | tuple[int, int, int]] = (0, 0, 0),
                         step_px: int = 1, space_scale: float = 0.3,
                         right_margin: int = 1, left_margin: int = 0, up_margin: int = 1, down_margin: int = 0,
                         speed_ms: int = 0,
                         update_flag: bool = False, 
                         right_mirrored: bool = False) -> None:
                             
        chars = list(text)
        mode, _, _, fg_shader = self.__fg_mode(fg, text, chars)

        if direction in ("left", "right"):
            if mode == 'shader':
                self.__scroll_text_h_shader(text, direction, fg_shader, bg, step_px, space_scale, right_margin, left_margin, speed_ms, update_flag, right_mirrored)
            else:
                self.__scroll_text_h(text, direction, fg, bg, step_px, space_scale, right_margin, left_margin, speed_ms, update_flag, right_mirrored)
        elif direction in ("up", "down"):
                if mode == 'shader':
                    self.__scroll_text_v_shader(text, direction, x, y, fg_shader, bg, step_px, space_scale, left_margin, right_margin, up_margin, down_margin, speed_ms, update_flag)
                else:
                    self.__scroll_text_v(text, direction, x, y, fg, bg, step_px, space_scale, left_margin, right_margin, up_margin, down_margin, speed_ms, update_flag)
        else:
            raise ValueError("direction must be 'left','right','up','down'")

    @micropython.viper
    def __fill32(self, buf: ptr32, off: int, value: uint, count: int):
        i: int = 0
        while i < count:
            buf[off + i] = value
            i += 1

    @micropython.viper
    def __vb_fill_row(self, row:int, pix:uint):
        vb = ptr32(self._vb)
        vb_w: int = int(self._vb_w)
        vb_h: int = int(self._vb_h)

        if row >= vb_h:
            row -= vb_h
        elif row < 0:
            row += vb_h

        off: int = row * vb_w
        i: int = 0
        while i < vb_w:
            vb[off + i] = pix
            i += 1

    @micropython.viper
    def __blit_row(self, fb: ptr32, off: int, row_bits: int, mask_base: int, cc_from: int, cc_to: int, px0: int, packed_fg: int):
        while cc_from <= cc_to:
            if (row_bits >> (mask_base - cc_from)) & 1:
                fb[off + px0] = packed_fg

            cc_from += 1
            px0 += 1

    @micropython.viper
    def __blit_row_shader_spec(self, fb: ptr32, off_in:int, row_bits_in:int, mask_base_in:int,
        cc_from_in:int, cc_to_in:int, px0_in:int, mode_in:int, pix_a_in:int, pix_b_in:int,
        p1_in:int, p2_in:int, fw_in:int, fh_in:int, rr_in:int):
        off: int = int(off_in)
        rbits: int = int(row_bits_in)
        mask_base: int = int(mask_base_in)
        cc_from: int = int(cc_from_in)
        cc_to: int = int(cc_to_in)
        px0: int = int(px0_in)
        mode: int = int(mode_in)
        pix_a: int = int(pix_a_in)
        pix_b: int = int(pix_b_in)
        p1: int = int(p1_in)
        p2: int = int(p2_in)
        fw: int = int(fw_in)
        fh: int = int(fh_in)
        rr: int = int(rr_in)

        thr_lr: int = 0
        thr_tb: int = 0
        cw: int = 0
        ch: int = 0
        arm: int = 0
        cx: int = 0
        cy: int = 0

        if mode == 1:               # split_lr
            thr_lr = p1
        elif mode == 2:             # split_tb
            thr_tb = p1
        elif mode == 3:             # checker
            cw = p1
            ch = p2
        else:                       # snowflake
            arm = p1
            cx = fw >> 1
            cy = fh >> 1

        while cc_from <= cc_to:
            sh: int = mask_base - cc_from
            on: int = (rbits >> sh) & 1
            if on != 0:
                pix: int = pix_a

                if mode == 1:
                    if cc_from >= thr_lr:
                        pix = pix_b
                elif mode == 2:
                    if rr >= thr_tb:
                        pix = pix_b
                elif mode == 3:
                    tx: int = cc_from // cw
                    ty: int = rr // ch
                    v: int = (tx + ty) & 1
                    if v != 0:
                        pix = pix_b
                else:
                    dx: int = cc_from - cx
                    if dx < 0:
                        adx: int = -dx
                    else:
                        adx = dx

                    dy: int = rr - cy
                    if dy < 0:
                        ady: int = -dy
                    else:
                        ady = dy

                    on_cross: int = 0
                    if (adx <= arm) or (ady <= arm):
                        on_cross = 1

                    on_diag: int = 0
                    d1: int = dx - dy
                    if d1 < 0:
                        d1 = -d1

                    if d1 <= arm:
                        on_diag = 1
                    else:
                        d2: int = dx + dy
                        if d2 < 0:
                            d2 = -d2
                        if d2 <= arm:
                            on_diag = 1

                    if (on_cross == 0) and (on_diag == 0):
                        pix = pix_b

                fb[off + px0] = pix

            cc_from += 1
            px0     += 1



    @micropython.viper
    def __rb_fill_col(self, col:int, pix:uint):
        rb = ptr32(self._rb)
        rb_w = int(self._rb_w)
        h = int(self._rb_h)
        off = col

        for _ in range(h):
            rb[off] = pix
            off += rb_w

    @micropython.viper
    def __rb_copy_to_fb(self):
        rb = ptr32(self._rb)
        fb = ptr32(self._fb)
        rb_w = int(self._rb_w)
        w = int(self._fb_width)
        h = int(self._fb_height)
        head = int(self._rb_head_px)

        for row in range(h):
            src = head + row * rb_w
            dst = row * w
            left = rb_w - head if head + w > rb_w else w
           
            for i in range(left):
                fb[dst + i] = rb[src + i]
           
            if left < w:
                src2 = row * rb_w
                for i in range(w - left):
                    fb[dst + left + i] = rb[src2 + i]

    @micropython.viper
    def __glyph_row_bits(self, gi: int, rr: int) -> int:
        bpr: int = int(self._glyph_bpr)
        fw: int = int(self.font_width)
        p = ptr8(self.font)

        base: int = int(gi) + int(rr) * bpr
        mask_shift: int = (bpr << 3) - fw

        if bpr == 1:
            bits: int = int(p[base])
        elif bpr == 2:
            bits = (int(p[base]) << 8) | int(p[base + 1])
        elif bpr == 3:
            bits = (int(p[base]) << 16) | (int(p[base + 1]) << 8) | int(p[base + 2])
        else:  # bpr >= 4
            bits = ((int(p[base]) << 24) | (int(p[base + 1]) << 16) | (int(p[base + 2]) << 8) | int(p[base + 3]))

        mask: int = -1 if fw >= 32 else (1 << fw) - 1
        return (bits >> mask_shift) & mask

    @micropython.viper
    def __flush_fb_to_txb(self):
        fb = ptr32(self._fb)
        map_sm = ptr8(self.__pix_map_sm)
        map_idx = ptr16(self.__pix_map_idx)
        bufs = self.__tx_bufs      
        length = int(self._fb_length)
        
        for i in range(length):
            sm = int(map_sm[i])
            idx = map_idx[i]
            bufs[sm][idx] = fb[i]  

    @micropython.viper
    def __glyph_lr_compute_packed(self, off: int) -> int:
        fw: int = int(self.font_width)
        fh: int = int(self.font_height)
        bpr: int = int(self._glyph_bpr)
        p = ptr8(self.font)

        base: int = int(off)
        mask_shift: int = (bpr << 3) - fw
        mask: int = -1 if fw >= 32 else (1 << fw) - 1
        topmask: int = 1 << (fw - 1)

        left: int = fw
        right: int = -1

        rr: int = 0
        while rr < fh:
            bits: int = 0
            bi: int = 0
            bb: int = base
            while bi < bpr:
                bits = (bits << 8) | int(p[bb])
                bb += 1
                bi += 1

            row: int = (bits >> mask_shift) & mask
            if row != 0:
                t: int = row
                c: int = 0
                while (t & topmask) == 0:
                    c += 1
                    t <<= 1
                    if c == fw:
                        break
                
                if c < left:
                    left = c

                t = row
                cz: int = 0
                while (t & 1) == 0:
                    cz += 1
                    t >>= 1
                rpos: int = fw - 1 - cz
                
                if rpos > right:
                    right = rpos

                if left == 0 and right == (fw - 1):
                    rr = fh 
                    continue

            base += bpr
            rr += 1

        blank_L: int = fw if right < 0 else left
        blank_R: int = fw if right < 0 else (fw - 1 - right)
        return (blank_L << 16) | (blank_R & 0xFFFF)

    @micropython.viper
    def __present_from_vb(self, head: int, wait_complete: int):
        vb   = ptr32(self._vb)
        vb_w: int = int(self._vb_w)
        vb_h: int = int(self._vb_h)

        fb_w: int = int(self._fb_width)
        fb_h: int = int(self._fb_height)

        map_sm = ptr8(self.__pix_map_sm)
        map_idx = ptr16(self.__pix_map_idx)

        bufs = self.__tx_bufs   # list[array('I')]
        dmas = self.__dmas      # list[DMA]
        sms = self.__sms        # list[StateMachine]
        sm_ids = self.__sm_ids  # list[int]

        y: int = 0
        while y < fb_h:
            src_row: int = head + y
            if src_row >= vb_h:
                src_row -= vb_h
            vb_off: int = src_row * vb_w
            base: int   = y * fb_w

            x: int = 0
            while x < fb_w:
                i_pix: int  = base + x
                sm_i: int   = int(map_sm[i_pix])
                idx: int    = int(map_idx[i_pix])
                dst = bufs[sm_i]
                dst[idx] = vb[vb_off + x]
                x += 1
            y += 1

        busy: int = 0
        i: int = 0
        n_dmas: int = int(len(dmas))
        while i < n_dmas:
            if int(dmas[i].active()):
                busy = 1
                break
            i += 1

        if busy == 0:
            i = 0
            n_sms: int = int(len(sms))
            while i < n_sms:
                dma = dmas[i]
                if int(dma.active()) == 0:
                    sm_id = sm_ids[i]
                    sm_obj = sms[i]
                    src_arr = bufs[i]
                    ctrl = dma.pack_ctrl(size=2, inc_write=False, treq_sel=self.__pio_dreq(int(sm_id)), bswap=False)
                    dma.config(read=src_arr, write=sm_obj, count=int(len(src_arr)), ctrl=ctrl, trigger=True)
                i += 1

        if wait_complete:
            while True:
                any_active: int = 0
                i = 0
                while i < n_dmas:
                    if int(dmas[i].active()):
                        any_active = 1
                        break
                    i += 1
                if any_active == 0:
                    break

    @micropython.viper
    def __present_from_rb(self, head: int, wait_complete: int):
        rb = ptr32(self._rb)
        rb_w: int = int(self._rb_w)
        fb_w: int = int(self._fb_width)
        fb_h: int = int(self._fb_height)
        map_sm = ptr8(self.__pix_map_sm)
        map_idx = ptr16(self.__pix_map_idx)
        bufs = self.__tx_bufs
        dmas = self.__dmas
        sms = self.__sms
        sm_ids = self.__sm_ids

        y: int = 0
        while y < fb_h:
            base: int    = y * fb_w
            row_off: int = y * rb_w

            x: int = 0
            while x < fb_w:
                i_pix: int = base + x
                sm_i: int = int(map_sm[i_pix]) 
                idx: int = int(map_idx[i_pix])

                src_col: int = head + x
                if src_col >= rb_w:
                    src_col -= rb_w

                dst = bufs[sm_i] 
                dst[idx] = rb[row_off + src_col]
                x += 1
            y += 1

        busy: int = 0
        i: int = 0
        n_dmas: int = int(len(dmas))
        while i < n_dmas:
            if int(dmas[i].active()):
                busy = 1
                break
            i += 1

        if busy == 0:
            i = 0
            n_sms: int = int(len(sms))
            while i < n_sms:
                dma = dmas[i]
                if int(dma.active()) == 0:
                    sm_id = sm_ids[i]
                    sm_obj = sms[i]
                    src_arr = bufs[i]
                    ctrl = dma.pack_ctrl(size=2, inc_write=False, treq_sel=self.__pio_dreq(int(sm_id)), bswap=False)
                    dma.config(read=src_arr, write=sm_obj, count=int(len(src_arr)), ctrl=ctrl, trigger=True)
                i += 1

        if wait_complete:
            while True:
                any_active: int = 0
                i = 0
                while i < n_dmas:
                    if int(dmas[i].active()):
                        any_active = 1
                        break
                    i += 1
                if any_active == 0:
                    break

    @micropython.native
    def __get_row_block(self, span:int, packed:int):
        try:
            cache = self.__row_cache
        except AttributeError:
            cache = self.__row_cache = {}

        key = (span, packed)
        blk = cache.get(key)
        if blk is None:
            blk = array.array('I', [packed] * span)
            cache[key] = blk

        return blk

    @micropython.native
    def __glyph_lr(self, off: int) -> tuple[int, int]:
        t = self.__blank_cache.get(off)
        if t is not None:
            return t

        packed = self.__glyph_lr_compute_packed(off)
        blank_L = packed >> 16
        blank_R = packed & 0xFFFF
        t = (blank_L, blank_R)
        self.__blank_cache[off] = t
        return t

    @micropython.native
    def __build_pix_maps(self) -> None:
        dw = self._fb_width
        dh = self._fb_height
        n = dw * dh

        sm_t = array.array("B", [0] * n)
        idx_t = array.array("H", [0] * n)

        for y in range(dh):
            base = y * dw
            for x in range(dw):
                sm_i, bi = self.__coord_to_index(x, y)
                i = base + x
                sm_t[i] = sm_i
                idx_t[i] = bi

        self.__pix_map_sm = sm_t
        self.__pix_map_idx = idx_t

    @micropython.native
    def __coord_to_index(self, x: int, y: int) -> tuple[int, int]:
        if not (0 <= x < self._fb_width and 0 <= y < self._fb_height):
            raise IndexError("pixel out of range")

        panel_col = x // self.__panel_width
        panel_row = y // self.__panel_height
        panel_id = panel_row * self.__grid_width + panel_col

        lx = x % self.__panel_width
        ly = y % self.__panel_height
        if self.__origin.startswith("bottom"):
            ly = self.__panel_height - 1 - ly
        if self.__origin.endswith("right"):
            lx = self.__panel_width - 1 - lx
        if self.__zigzag and (ly % 2):
            lx = self.__panel_width - 1 - lx

        sm_idx = panel_id // self.__panels_per_sm
        local_id = panel_id % self.__panels_per_sm
        buf_idx = local_id * self.__pixels_per_panel + ly * self.__panel_width + lx
        
        return sm_idx, buf_idx

    @micropython.viper
    def __pack_grb_v(self, r:int, g:int, b:int) -> int:
        bt = ptr8(self.__btab)
        return (int(bt[g]) << 24) | (int(bt[r]) << 16) | (int(bt[b]) << 8)

    @micropython.native
    def __pack_grb(self, r: int, g: int, b: int) -> int:
        bt = self.__btab
        return (bt[g] << 24) | (bt[r] << 16) | (bt[b] << 8)

    @micropython.native
    def __normalize_color(self, color: int | tuple[int, int, int] | list[int]) -> tuple[int, int, int]:
        if isinstance(color, (tuple, list)):
            if len(color) != 3:
                raise ValueError("Color tuple/list must have 3 elements")
            
            r, g, b = color
            if not all(isinstance(c, int) and 0 <= c <= 255 for c in (r, g, b)):
                raise ValueError("RGB must be 0-255 ints")
            
            return int(r), int(g), int(b)
        elif isinstance(color, int):
            if not (0 <= color <= 0xFFFFFF):
                raise ValueError("hex 0x000000-0xFFFFFF")
            
            return (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF
        else:
            raise TypeError("Color must be tuple/list/int")

    @micropython.native
    def __pio_dreq(self, sm_id: int) -> int:
        if sm_id < 4:
            pio = 0
        elif sm_id < 8:
            pio = 1
        elif sm_id < 12:
            pio = 2
        else:
            raise ValueError("SM id out of range")
        
        return (pio << 3) | (sm_id & 0x03)

    def __unload_font(self):
        for attr in ("font", "font_width", "font_height", "_glyph_bpr", "_glyph_span", "_fallback_cp",
            "_sparse_font", "_codes", "_offsets", "char_first", "char_last"):
            if hasattr(self, attr):
                setattr(self, attr, None)

        self.__blank_cache = {}

        modname = getattr(self, "_font_modname", None)
        if isinstance(modname, str) and modname in sys.modules:
            try:
                del sys.modules[modname]
            except KeyError:
                pass

        gc.collect()

    def __ensure_buffers_for_font(self):
        rb_w = self._fb_width + self.font_width
        rb_h = self._fb_height
        rb_n = rb_w * rb_h

        vb_w = self._fb_width
        self._vb_max_ud_margin = 4
        self._vb_step_guard = 4
        vb_gh_cap = self.font_height + self._vb_max_ud_margin*2 + self._vb_step_guard
        vb_h = self._fb_height + vb_gh_cap
        vb_n = vb_w * vb_h

        cur_rb = getattr(self, "_rb", None)
        if (cur_rb is None) or (len(cur_rb) < rb_n):
            self._rb_w, self._rb_h = rb_w, rb_h
            self._rb_size = rb_n
            gc.collect()
            self._rb = array.array("I", (0 for _ in range(rb_n)))
        else:
            self._rb_w, self._rb_h = rb_w, rb_h
            self._rb_size = rb_n

        self._rb_head_px  = 0
        self._rb_write_px = self._fb_width

        cur_vb = getattr(self, "_vb", None)
        if (cur_vb is None) or (len(cur_vb) < vb_n):
            self._vb_w = vb_w
            self._vb_h = vb_h
            gc.collect()
            self._vb = array.array("I", (0 for _ in range(vb_n)))
        else:
            self._vb_w = vb_w
            self._vb_h = vb_h

        self._vb_head_row = 0
        self._vb_write_row = self._fb_height
        gc.collect()

    def __load_font(self, font_src: str | object) -> None:
        self.__unload_font()
        gc.collect()

        if isinstance(font_src, str):
            mod = __import__(font_src)
            self._font_name = font_src
            self._font_modname = font_src
        else:
            mod = font_src
            self._font_name = getattr(mod, "__name__", "<custom_font>")
            self._font_modname = getattr(mod, "__name__", None)

        gc.collect()
        self.font = mod.FONT
        self.font_width = mod.WIDTH
        self.font_height = mod.HEIGHT

        self._glyph_bpr = getattr(mod, "GLYPH_BYTES_PER_ROW", (self.font_width + 7) // 8)
        self._glyph_span = getattr(mod, "GLYPH_SPAN", self._glyph_bpr * self.font_height)
        self._fallback_cp = getattr(mod, "FALLBACK", ord("?"))
        self._sparse_font = getattr(mod, "SPARSE", False)

        if self._sparse_font:
            self._codes = mod.CODES
            self._offsets = mod.OFFSETS
            self.char_first = None
            self.char_last = None
        else:
            self.char_first = mod.FIRST
            self.char_last = mod.LAST

        self.__blank_cache = {}
        gc.collect()
        self.__ensure_buffers_for_font()

    @micropython.native
    def __glyph_offset_sparse(self, code: int) -> int:
        lo = 0
        hi = len(self._codes) - 1
        codes = self._codes

        while lo <= hi:
            mid = (lo + hi) >> 1
            c = codes[mid]
            if c == code:
                return self._offsets[mid]  

            if c < code:
                lo = mid + 1
            else:
                hi = mid - 1

        return -1

    @micropython.native
    def __glyph_offset(self, code: int) -> int:
        if self._sparse_font:
            off = self.__glyph_offset_sparse(code)
            if off >= 0:
                return off

            off = self.__glyph_offset_sparse(self._fallback_cp)
            return off
        else:
            if code < self.char_first or code > self.char_last:  
                code = self._fallback_cp
                if code < self.char_first or code > self.char_last:  
                    code = self.char_first  

            return (code - self.char_first) * self._glyph_span

    def __cached_block_width(self, ch: str, *, space_scale: float, left_margin: int, right_margin: int) -> int:
        key = (ch, space_scale, left_margin, right_margin)
        bw = self.__ink_width_cache.get(key)
   
        if bw is None:
            fw = self.font_width
            if ch == " ":
                return int(fw * space_scale)

            gi = self.__glyph_offset(ord(ch))
            blank_L, blank_R = self.__glyph_lr(gi)
            ink_w = fw - blank_L - blank_R
            bw = left_margin + ink_w + right_margin

            if len(self.__ink_width_cache) > 128:
                for k in list(self.__ink_width_cache)[:64]:
                    self.__ink_width_cache.pop(k, None)
            
            self.__ink_width_cache[key] = bw
        
        return bw

    @micropython.native
    def __fg_mode(self, fg, text: str, chars: list):
        if isinstance(fg, (tuple, list)) and fg:
            m = fg[0]
            if isinstance(m, int) and 1 <= m <= 4:
                return 'shader', None, 0, fg
            
        if isinstance(fg, (list, tuple)) and fg and isinstance(fg[0], (list, tuple)):
            fg_tbl = [self.__pack_grb(*c) for c in fg]
            return 'per_char', fg_tbl, 0, None
        
        if callable(fg):
            try:
                test = fg(0, text)
                _ = self.__pack_grb(*self.__normalize_color(test))
                fg_tbl = [self.__pack_grb(*self.__normalize_color(fg(i, text))) for i, _ in enumerate(chars)]
                return 'per_char', fg_tbl, 0, None
            except TypeError:
                return 'shader', None, 0, fg
        
        r, g, b = self.__normalize_color(fg)
        
        return 'flat', None, self.__pack_grb(r, g, b), None

    @micropython.native
    def __scroll_h_setup(self, bg, direction: str):
        rb_w = self._rb_w
        packed_bg = self.__pack_grb(*self.__normalize_color(bg)) if bg is not None else 0
        
        for col in range(rb_w): 
            self.__rb_fill_col(col, packed_bg)
        
        head = self._rb_head_px
        disp_w = self._fb_width
        write_x = (head + disp_w) % rb_w if direction == "left" else (head - 1) % rb_w
        
        return packed_bg, disp_w, rb_w, head, write_x

    @micropython.native
    def __scroll_h_clear_block(self, write_x:int, gw:int, rb_w:int, direction, packed_bg:int):
        dir_right = 0 if direction == "left" else 1
        self.__scroll_h_clear_block_v(write_x, gw, rb_w, dir_right, packed_bg)

    @micropython.viper
    def __scroll_h_blit_glyph_split_tb_v(self, gi:int, fw_in:int, fh_in:int,
        rb_w_in:int, write_x_in:int, gw_in:int, left_margin_in:int, dir_right_in:int, right_mirrored_in:int,
        thr_tb_in:int, ink_l_in:int, ink_r_in:int, pix_a_in:int, pix_b_in:int):
        rb = ptr32(self._rb)
        fw:int = int(fw_in)
        fh:int = int(fh_in)
        rb_w:int = int(rb_w_in)
        write_x:int = int(write_x_in)
        gw:int = int(gw_in)
        left_margin:int = int(left_margin_in)
        dir_right:int = int(dir_right_in)
        right_mirrored:int = int(right_mirrored_in)
        thr_tb:int = int(thr_tb_in)
        ink_l:int = int(ink_l_in)
        ink_r:int = int(ink_r_in)
        pix_a:int = int(pix_a_in)
        pix_b:int = int(pix_b_in)

        col0:int = 0
        if dir_right == 0:  # left
            col0 = write_x + left_margin
        else:
            if right_mirrored != 0:
                col0 = write_x - left_margin - (fw - 1)
            else:
                col0 = write_x - gw + left_margin

        while col0 < 0:
            col0 += rb_w

        while col0 >= rb_w:
            col0 -= rb_w

        mbase:int = fw - 1

        bit_lsb:int = 0
        if dir_right != 0:
            if right_mirrored != 0:
                bit_lsb = 1

        row:int = 0
        while row < fh:
            row_bits:int = int(self.__glyph_row_bits(gi, row))
            base:int = row * rb_w

            pix_row:int = pix_a
            if row >= thr_tb:
                pix_row = pix_b

            c:int = col0
            delta:int = ink_l
            while delta:
                c += 1
                if c == rb_w:
                    c = 0
                delta -= 1

            px:int = ink_l
            if bit_lsb == 0:
                shift:int = mbase - ink_l
                mask:int = 1 << shift
                while px <= ink_r:
                    while (px <= ink_r) and ((row_bits & mask) == 0):
                        px += 1
                        mask >>= 1
                        c += 1
                        if c == rb_w:
                            c = 0
                    if px > ink_r:
                        break

                    run:int = 0
                    while (px <= ink_r) and ((row_bits & mask) != 0):
                        run += 1
                        px += 1
                        mask >>= 1

                    rem:int = run
                    col:int = c
                    cap:int = rb_w - col
                    if cap > rem:
                        cap = rem

                    i:int = 0
                    while i < cap:
                        rb[base + col + i] = pix_row
                        i += 1

                    rem -= cap
                    if rem > 0:
                        j:int = 0
                        while j < rem:
                            rb[base + j] = pix_row
                            j += 1
                        col = rem

                    c += run
                    if c >= rb_w:
                        c -= rb_w
            else:
                shift:int = ink_l
                mask:int = 1 << shift
                while px <= ink_r:
                    while (px <= ink_r) and ((row_bits & mask) == 0):
                        px += 1
                        mask <<= 1
                        c += 1
                        if c == rb_w:
                            c = 0

                    if px > ink_r:
                        break

                    run:int = 0
                    while (px <= ink_r) and ((row_bits & mask) != 0):
                        run += 1
                        px += 1
                        mask <<= 1

                    rem:int = run
                    col:int = c
                    cap:int = rb_w - col
                    if cap > rem:
                        cap = rem

                    i:int = 0
                    while i < cap:
                        rb[base + col + i] = pix_row
                        i += 1

                    rem -= cap
                    if rem > 0:
                        j:int = 0
                        while j < rem:
                            rb[base + j] = pix_row
                            j += 1
                        col = rem

                    c += run
                    if c >= rb_w:
                        c -= rb_w

            row += 1

    @micropython.viper
    def __scroll_h_clear_block_v(self, write_x:int, gw:int, rb_w:int, dir_right:int, packed_bg:int):
        rb = ptr32(self._rb)
        fb_h:int = int(self._fb_height)

        if dir_right == 0:   # left
            start:int = int(write_x)
        else:                # right
            start = int(write_x) - (int(gw) - 1)

        while start < 0:
            start += rb_w
        while start >= rb_w:
            start -= rb_w

        y:int = 0
        while y < fb_h:
            row_off:int = y * rb_w

            first:int = rb_w - start
            if first > gw:
                first = gw

            i:int = 0
            while i < first:
                rb[row_off + start + i] = packed_bg
                i += 1

            remain:int = gw - first
            if remain > 0:
                j:int = 0
                while j < remain:
                    rb[row_off + j] = packed_bg
                    j += 1

            y += 1

    @micropython.viper
    def __kick_dma(self, wait_complete:int):
        dmas = self.__dmas
        sms = self.__sms
        sm_ids = self.__sm_ids
        bufs = self.__tx_bufs

        i:int = 0
        n:int = int(len(dmas))
        busy:int = 0
        while i < n:
            if int(dmas[i].active()):
                busy = 1
                break
            i += 1

        if busy == 0:
            i = 0
            m:int = int(len(sms))
            while i < m:
                dma = dmas[i]
                if int(dma.active()) == 0:
                    sm_id = sm_ids[i]
                    sm = sms[i]
                    src = bufs[i]
                    ctrl = dma.pack_ctrl(size=2, inc_write=False, treq_sel=self.__pio_dreq(int(sm_id)), bswap=False)
                    dma.config(read=src, write=sm, count=int(len(src)), ctrl=ctrl, trigger=True)
                i += 1

        if wait_complete:
            while True:
                any:int = 0
                i = 0
                while i < n:
                    if int(dmas[i].active()):
                        any = 1
                        break
                    i += 1
                if any == 0:
                    break

    @micropython.native
    def __scroll_h_advance(self, head:int, write_x:int, disp_w:int, rb_w:int, direction:str, shift:int, speed_ms:int, update_flag:bool, packed_bg:int):
        if shift <= 0:
            return head, write_x

        self._rb_head_px = head
        self._rb_head_px = head
        self.__rb_copy_to_fb()
        self.__flush_fb_to_txb()
        self.__kick_dma(1 if update_flag else 0)

        if speed_ms:
            utime.sleep_ms(speed_ms)

        if direction == "left":
            for s in range(shift):
                self.__rb_fill_col((head + s) % rb_w, packed_bg)

            head    = (head + shift)   % rb_w
            write_x = (write_x + shift) % rb_w
        else:
            for s in range(shift):
                self.__rb_fill_col((head + disp_w - 1 - s) % rb_w, packed_bg)

            head    = (head - shift)   % rb_w
            write_x = (write_x - shift) % rb_w

        return head, write_x

    @micropython.native
    def __scroll_h_blit_glyph_solid(self, gi, fw, fh, rb_w, write_x, gw, left_margin, direction, right_mirrored, pix_fg):
        rb = self._rb
        if direction == "left":
            col = (write_x + left_margin) % rb_w
            bit_mode = 0
        else:
            if right_mirrored:
                col = (write_x - left_margin - (fw - 1)) % rb_w
                bit_mode = 1
            else:
                col = (write_x - gw + left_margin) % rb_w
                bit_mode = 0

        mbase = fw - 1
        for row in range(fh):
            row_bits = self.__glyph_row_bits(gi, row)
            base = row * rb_w
            c = col
            for px in range(fw):
                on = (row_bits >> (px if bit_mode else (mbase - px))) & 1
                if on:
                    rb[base + c] = pix_fg
                c += 1
                if c == rb_w:
                    c = 0

    @micropython.native
    def __scroll_h_blit_glyph_shader(self, gi, fw, fh, rb_w, write_x, gw, left_margin, direction, right_mirrored, fg_shader, ch_idx, ch):
        rb = self._rb

        if direction == "left":
            col0 = (write_x + left_margin) % rb_w

            for row in range(fh):
                row_bits = self.__glyph_row_bits(gi, row)
                base = row * rb_w
                for px in range(fw):
                    if (row_bits >> (fw - 1 - px)) & 1:
                        r, g, b = self.__normalize_color(fg_shader(px, row, fw, fh, ch_idx, ch))
                        rb[base + (col0 + px) % rb_w] = self.__pack_grb_v(r, g, b)
        else:
            if right_mirrored:
                col0 = (write_x - left_margin - (fw - 1)) % rb_w
                for row in range(fh):
                    row_bits = self.__glyph_row_bits(gi, row)
                    base = row * rb_w
                    for px in range(fw):
                        if (row_bits >> px) & 1:
                            r, g, b = self.__normalize_color(fg_shader((fw - 1 - px), row, fw, fh, ch_idx, ch))
                            rb[base + (col0 + px) % rb_w] = self.__pack_grb_v(r, g, b)
            else:
                col0 = (write_x - gw + left_margin) % rb_w
                for row in range(fh):
                    row_bits = self.__glyph_row_bits(gi, row)
                    base = row * rb_w
                    for px in range(fw):
                        if (row_bits >> (fw - 1 - px)) & 1:
                            r, g, b = self.__normalize_color(fg_shader(px, row, fw, fh, ch_idx, ch))
                            rb[base + (col0 + px) % rb_w] = self.__pack_grb_v(r, g, b)

    @micropython.native
    def __scroll_h_blit_glyph_shader_spec(self, gi, fw, fh, rb_w, write_x, gw, left_margin, direction, right_mirrored, shspec):
        rb = self._rb
        mode = shspec[0]

        if mode == 1:  # split_lr
            pix_a, pix_b = shspec[1], shspec[2]
            rnum, rden = int(shspec[3]), int(shspec[4])
            thr_lr = (fw * rnum) // rden
        elif mode == 2:  # split_tb
            pix_a, pix_b = shspec[1], shspec[2]
            rnum, rden = int(shspec[3]), int(shspec[4])
            thr_tb = (fh * rnum) // rden
        elif mode == 3:  # checker
            pix_a, pix_b = shspec[1], shspec[2]
            cell_w = int(shspec[3])
            cell_h = int(shspec[4])
        else:            # snowflake
            pix_a, pix_b = shspec[1], shspec[2]
            arm = int(shspec[3])
            cx = fw // 2
            cy = fh // 2

        if direction == "left":
            col0 = (write_x + left_margin) % rb_w
            bit_mirror = 0
        else:
            if right_mirrored:
                col0 = (write_x - left_margin - (fw - 1)) % rb_w
                bit_mirror = 1
            else:
                col0 = (write_x - gw + left_margin) % rb_w
                bit_mirror = 0

        mbase = fw - 1

        for row in range(fh):
            row_bits = self.__glyph_row_bits(gi, row)
            base = row * rb_w

            if mode == 2:
                pix_row = pix_a if row < thr_tb else pix_b

                px = 0
                while px < fw:
                    on = ((row_bits >> (px if bit_mirror else (mbase - px))) & 1) != 0
                    if not on:
                        px += 1
                        continue

                    run_start = px
                    px += 1
                    while px < fw and ((row_bits >> (px if bit_mirror else (mbase - px))) & 1):
                        px += 1
                    run_len = px - run_start

                    c = col0 + run_start
                    while c >= rb_w:
                        c -= rb_w

                    first = rb_w - c
                    if first > run_len:
                        first = run_len

                    i = 0
                    while i < first:
                        rb[base + c + i] = pix_row
                        i += 1

                    rem = run_len - first
                    if rem > 0:
                        j = 0
                        while j < rem:
                            rb[base + j] = pix_row
                            j += 1

                continue

            if mode == 1:
                c = col0
                for px in range(fw):
                    on = (row_bits >> (px if bit_mirror else (mbase - px))) & 1
                    if on:
                        sxx = (fw - 1 - px) if (direction == "right" and right_mirrored) else px
                        rb[base + c] = pix_a if sxx < thr_lr else pix_b
                    c += 1
                    if c == rb_w:
                        c = 0
            elif mode == 3:
                c = col0
                for px in range(fw):
                    on = (row_bits >> (px if bit_mirror else (mbase - px))) & 1
                    if on:
                        sxx = (fw - 1 - px) if (direction == "right" and right_mirrored) else px
                        rb[base + c] = pix_a if (((sxx // cell_w) + (row // cell_h)) & 1) == 0 else pix_b
                    c += 1
                    if c == rb_w:
                        c = 0
            else:
                c = col0
                for px in range(fw):
                    on = (row_bits >> (px if bit_mirror else (mbase - px))) & 1
                    if on:
                        sxx = (fw - 1 - px) if (direction == "right" and right_mirrored) else px
                        dx = sxx - cx; dy = row - cy
                        on_cross = (abs(dx) <= arm) or (abs(dy) <= arm)
                        on_diag  = (abs(dx - dy) <= arm) or (abs(dx + dy) <= arm)
                        rb[base + c] = pix_a if (on_cross or on_diag) else pix_b
                    c += 1
                    if c == rb_w:
                        c = 0

    @micropython.native
    def __h_tail_cleanup(self, head:int, direction:str, step_px:int, packed_bg:int):
        n = step_px - 1

        if n <= 0:
            return

        rb_w   = self._rb_w
        disp_w = self._fb_width

        if direction == "left":
            start = (head + disp_w - 1) % rb_w
            for i in range(n):
                self.__rb_fill_col((start - i) % rb_w, packed_bg)
        else:
            start = head % rb_w
            for i in range(n):
                self.__rb_fill_col((start + i) % rb_w, packed_bg)

    @micropython.native
    def __scroll_text_h(self, text: str, direction: str, fg, bg, step_px: int, space_scale: float, right_margin: int, left_margin: int, speed_ms: int, update_flag: bool, right_mirrored: bool) -> None:
        fw, fh = self.font_width, self.font_height
        chars = list(text)
        mode, fg_tbl, packed_fg, _ = self.__fg_mode(fg, text, chars)

        if mode == 'shader':
            raise TypeError("__scroll_text_h(): fg is shader; use __scroll_text_h_shader()")

        packed_bg, disp_w, rb_w, head, write_x = self.__scroll_h_setup(bg, direction)
        for idx, ch in enumerate(chars):
            if ch == " ":
                gi = -1; gw = int(fw * space_scale)
            else:
                gi = self.__glyph_offset(ord(ch))
                gw = self.__cached_block_width(ch, space_scale=space_scale, left_margin=left_margin, right_margin=right_margin)

            self.__scroll_h_clear_block(write_x, gw, rb_w, direction, packed_bg)
            if gi >= 0:
                pix_fg = packed_fg if fg_tbl is None else fg_tbl[idx]
                self.__scroll_h_blit_glyph_solid(gi, fw, fh, rb_w, write_x, gw, left_margin, direction, right_mirrored, pix_fg)

            remain = gw
            while remain > 0:
                shift = step_px if remain >= step_px else remain
                head, write_x = self.__scroll_h_advance(head, write_x, disp_w, rb_w, direction, shift, speed_ms, update_flag, packed_bg)
                remain -= shift

        remain = disp_w
        while remain > 0:
            shift = step_px if remain >= step_px else remain
            head, write_x = self.__scroll_h_advance(head, write_x, disp_w, rb_w, direction, shift, speed_ms, update_flag, packed_bg)
            remain -= shift

        self.__h_tail_cleanup(head, direction, step_px, packed_bg)        
        self.__present_from_rb(head, 1 if update_flag else 0)

        self._rb_head_px  = head
        self._rb_write_px = write_x

    @micropython.native
    def __scroll_text_h_shader(self, text: str, direction: str, fg_shader, bg, step_px: int, space_scale: float, right_margin: int, left_margin: int, speed_ms: int, update_flag: bool, right_mirrored: bool) -> None:
        fw, fh = self.font_width, self.font_height
        chars = list(text)
        packed_bg, disp_w, rb_w, head, write_x = self.__scroll_h_setup(bg, direction)

        is_spec = (isinstance(fg_shader, (tuple, list))
                and len(fg_shader) >= 1
                and isinstance(fg_shader[0], int)
                and 1 <= fg_shader[0] <= 4)

        dir_right = 0 if direction == "left" else 1

        for idx, ch in enumerate(chars):
            if ch == " ":
                gi = -1; gw = int(fw * space_scale)
            else:
                gi = self.__glyph_offset(ord(ch))
                gw = self.__cached_block_width(ch, space_scale=space_scale, left_margin=left_margin, right_margin=right_margin)

            self.__scroll_h_clear_block(write_x, gw, rb_w, direction, packed_bg)

            if gi >= 0:
                if is_spec and int(fg_shader[0]) == int(self._SH_SPLIT_TB):
                    rnum = int(fg_shader[3]); rden = int(fg_shader[4])
                    thr_tb = (fh * rnum) // rden
                    blank_L, blank_R = self.__glyph_lr(gi)
                    ink_l = blank_L
                    ink_r = fw - 1 - blank_R
                    self.__scroll_h_blit_glyph_split_tb_v(
                        gi, fw, fh, rb_w, write_x, gw, left_margin,
                        dir_right, 1 if right_mirrored else 0,
                        thr_tb, ink_l, ink_r,
                        fg_shader[1], fg_shader[2]
                    )
                elif is_spec:
                    self.__scroll_h_blit_glyph_shader_spec(
                        gi, fw, fh, rb_w, write_x, gw, left_margin, direction, right_mirrored, fg_shader
                    )
                else:
                    self.__scroll_h_blit_glyph_shader(
                        gi, fw, fh, rb_w, write_x, gw, left_margin, direction, right_mirrored, fg_shader, idx, ch
                    )

            remain = gw
            while remain > 0:
                shift = step_px if remain >= step_px else remain
                head, write_x = self.__scroll_h_advance(head, write_x, disp_w, rb_w, direction, shift, speed_ms, update_flag, packed_bg)
                remain -= shift

        remain = disp_w
        while remain > 0:
            shift = step_px if remain >= step_px else remain
            head, write_x = self.__scroll_h_advance(head, write_x, disp_w, rb_w, direction, shift, speed_ms, update_flag, packed_bg)
            remain -= shift

        self.__h_tail_cleanup(head, direction, step_px, packed_bg)
        self._rb_head_px  = head
        self._rb_write_px = write_x
        self.__kick_dma(1 if update_flag else 0)

    @micropython.native
    def __line_fit(self, chars, start_idx:int, space_scale:float, left_margin:int, right_margin:int, max_w:int):
        fw = self.font_width
        segs = []
        x = 0
        i = start_idx
        n = len(chars)

        while i < n:
            ch = chars[i]
            if ch == " ":
                bw = int(fw * space_scale)
      
                if x and x + bw > max_w:
                    break
      
                segs.append((-1, i, ch, x, 0, -1, fw))
                x += bw
                i += 1
                continue

            gi = self.__glyph_offset(ord(ch))
            blank_L, blank_R = self.__glyph_lr(gi)
            ink_w = (fw - blank_L - blank_R)
            
            bw = left_margin + ink_w + right_margin
            if x and x + bw > max_w:
                break

            segs.append((gi, i, ch, x + left_margin, blank_L, fw - 1 - blank_R, fw))
            x += bw
            i += 1

        if (i == start_idx) and (start_idx < n):
            ch = chars[start_idx]
            if ch == " ":
                bw = int(fw * space_scale)
                segs.append((-1, start_idx, ch, 0, 0, -1, fw))
                x = bw
                i = start_idx + 1
            else:
                gi = self.__glyph_offset(ord(ch))
                blank_L, blank_R = self.__glyph_lr(gi)
                ink_w = (fw - blank_L - blank_R)
                segs.append((gi, start_idx, ch, left_margin, blank_L, fw - 1 - blank_R, fw))
                x = left_margin + ink_w + right_margin
                i = start_idx + 1

        return i, segs, x

    def __vb_blit_line_solid(self, write_y:int, direction:str, segs, fg_tbl, packed_fg:int, bg_packed:int, gh:int, y_off:int):
        vb = self._vb
        vb_w = self._vb_w
        vb_h = self._vb_h
        fh = self.font_height

        if direction == "up":
            base_y = write_y
            for rr in range(gh):
                self.__vb_fill_row((base_y + rr) % vb_h, bg_packed)
        else:
            base_y = (write_y - (gh - 1)) % vb_h
            for rr in range(gh):
                self.__vb_fill_row((base_y + rr) % vb_h, bg_packed)

        for (gi, ch_idx, ch, dst_x, ink_l, ink_r, fw_local) in segs:
            if gi < 0:
                continue

            pix_fg = packed_fg if fg_tbl is None else fg_tbl[ch_idx]
            mbase = fw_local - 1

            for rr in range(fh):
                row_bits = self.__glyph_row_bits(gi, rr)
                vb_row = (base_y + y_off + rr) % vb_h
                base = vb_row * vb_w

                for xx in range(ink_l, ink_r + 1):
                    if (row_bits >> (mbase - xx)) & 1:
                        x = dst_x + (xx - ink_l)
                        if 0 <= x < vb_w:
                            vb[base + x] = pix_fg

    @micropython.native
    def __vb_blit_line_shader(self, write_y:int, direction:str, segs, fg_shader, bg_packed:int, gh:int, y_off:int):
        vb = self._vb
        vb_w = self._vb_w
        vb_h = self._vb_h
        fh = self.font_height

        if direction == "up":
            base_y = write_y
            for rr in range(gh):
                self.__vb_fill_row((base_y + rr) % vb_h, bg_packed)
        else:
            base_y = (write_y - (gh - 1)) % vb_h
            for rr in range(gh):
                self.__vb_fill_row((base_y + rr) % vb_h, bg_packed)

        for (gi, ch_idx, ch, dst_x, ink_l, ink_r, fw_local) in segs:
            if gi < 0:
                continue
        
            mbase = fw_local - 1
            for rr in range(fh):
                row_bits = self.__glyph_row_bits(gi, rr)
                vb_row = (base_y + y_off + rr) % vb_h
                base = vb_row * vb_w
        
                for xx in range(ink_l, ink_r + 1):
                    if (row_bits >> (mbase - xx)) & 1:
                        x = dst_x + (xx - ink_l)
                        if 0 <= x < vb_w:
                            r, g, b = self.__normalize_color(fg_shader(xx, rr, fw_local, fh, ch_idx, ch))
                            vb[base + x] = self.__pack_grb_v(r, g, b)

    @micropython.native
    def __vb_blit_line_shader_spec(self, write_y:int, direction:str, segs, shspec, bg_packed:int, gh:int, y_off:int):
        vb = self._vb
        vb_w = self._vb_w
        vb_h = self._vb_h
        fh = self.font_height

        if direction == "up":
            base_y = write_y
            for rr in range(gh):
                self.__vb_fill_row((base_y + rr) % vb_h, bg_packed)
        else:
            base_y = (write_y - (gh - 1)) % vb_h
            for rr in range(gh):
                self.__vb_fill_row((base_y + rr) % vb_h, bg_packed)

        mode = shspec[0]
        if mode == 1:  # split_lr
            pix_a, pix_b = shspec[1], shspec[2]
            rnum, rden = int(shspec[3]), int(shspec[4])
        elif mode == 2:  # split_tb
            pix_a, pix_b = shspec[1], shspec[2]
            rnum, rden = int(shspec[3]), int(shspec[4])
        elif mode == 3:  # checker
            pix_a, pix_b = shspec[1], shspec[2]
            cell_w = int(shspec[3])
            cell_h = int(shspec[4])
        else:  # mode == 4 snowflake
            pix_a, pix_b = shspec[1], shspec[2]
            arm = int(shspec[3])

        for (gi, ch_idx, ch_char, dst_x, ink_l, ink_r, fw_local) in segs:
            if gi < 0:
                continue

            mbase = fw_local - 1
            if mode == 1:
                thr_lr = (fw_local * rnum) // rden
            elif mode == 2:
                thr_tb = (fh * rnum) // rden
            elif mode == 4:
                cx = fw_local // 2
                cy = fh // 2

            for rr in range(fh):
                row_bits = self.__glyph_row_bits(gi, rr)
                vb_row = (base_y + y_off + rr) % vb_h
                base = vb_row * vb_w

                if mode == 2:
                    pix_row = pix_a if rr < thr_tb else pix_b

                for xx in range(ink_l, ink_r + 1):
                    if (row_bits >> (mbase - xx)) & 1:
                        x = dst_x + (xx - ink_l)
                        if 0 <= x < vb_w:
                            if mode == 1:        # split_lr
                                pix_fg = pix_a if xx < thr_lr else pix_b
                            elif mode == 2:      # split_tb
                                pix_fg = pix_row
                            elif mode == 3:      # checker
                                pix_fg = pix_a if (((xx // cell_w) + (rr // cell_h)) & 1) == 0 else pix_b
                            else:                # snowflake
                                dx = xx - cx; dy = rr - cy
                                on_cross = (abs(dx) <= arm) or (abs(dy) <= arm)
                                on_diag = (abs(dx - dy) <= arm) or (abs(dx + dy) <= arm)
                                pix_fg = pix_a if (on_cross or on_diag) else pix_b
                            vb[base + x] = pix_fg

    def __vb_ensure(self, gh:int, packed_bg:int):
        need_h = self._fb_height + gh
        if self._vb_w == self._fb_width and self._vb_h >= need_h:
            return

        self._vb_w = self._fb_width
        self._vb_h = need_h
        self._vb = array.array('I', [packed_bg]) * (self._vb_w * self._vb_h)
        self._vb_head_row  = 0
        self._vb_write_row = self._fb_height

    @micropython.native
    def __scroll_v_setup(self, bg, direction:str, gh:int):
        packed_bg = self.__pack_grb(*self.__normalize_color(bg)) if bg is not None else 0
        self.__vb_ensure(gh, packed_bg)

        head = self._vb_head_row
        disp_h = self._fb_height
        vb_h = self._vb_h
        write_y = (head + disp_h) % vb_h if direction == "up" else (head - 1) % vb_h
        
        return packed_bg, disp_h, vb_h, head, write_y
    
    @micropython.native
    def __scroll_v_advance(self, head:int, write_y:int, disp_h:int, vb_h:int, direction:str, shift:int, speed_ms:int, update_flag:bool, packed_bg:int):
        if shift <= 0:
            return head, write_y

        self._vb_head_row = head        
        self.__present_from_vb(head, 1 if update_flag else 0)
        
        if speed_ms:
            utime.sleep_ms(speed_ms)

        if direction == "up":
            for s in range(shift):
                self.__vb_fill_row((head + s) % vb_h, packed_bg)

            head = (head + shift) % vb_h
            write_y = (write_y + shift) % vb_h
        else:
            for s in range(shift):
                self.__vb_fill_row((head + disp_h - 1 - s) % vb_h, packed_bg)
            
            head = (head - shift) % vb_h
            write_y = (write_y - shift) % vb_h

        return head, write_y

    @micropython.native
    def __v_tail_cleanup(self, head:int, direction:str, step_px:int, packed_bg:int) -> int:
        n = step_px - 1
        if n <= 0:
            return 0
        
        vb_h   = self._vb_h
        disp_h = self._fb_height
        
        if direction == "up":
            start = (head + disp_h - 1) % vb_h
            for i in range(n):
                self.__vb_fill_row((start - i) % vb_h, packed_bg)
        else:
            start = head % vb_h
            for i in range(n):
                self.__vb_fill_row((start + i) % vb_h, packed_bg)

        return n
    
    @micropython.native
    def __scroll_text_v(self, text: str, direction: str, x: int, y: int, fg, bg, step_px: int, space_scale: float, left_margin: int, right_margin: int, up_margin: int, down_margin: int, speed_ms: int, update_flag: bool) -> None:
        fw, fh = self.font_width, self.font_height
        chars = list(text)
        mode, fg_tbl, packed_fg, _ = self.__fg_mode(fg, text, chars)
        
        gh = up_margin + fh + down_margin
        y_off = up_margin
        packed_bg, disp_h, vb_h, head, write_row = self.__scroll_v_setup(bg, direction, gh)
        
        i = 0; n = len(chars)
        max_w = self._fb_width - (x if x > 0 else 0)

        while i < n:
            end_i, segs, line_w = self.__line_fit(chars, i, space_scale, left_margin, right_margin, max_w)
        
            if x:
                tmp = []
                for (gi, ch_idx, ch, dst_x, ink_l, ink_r, fw_local) in segs:
                    tmp.append((gi, ch_idx, ch, dst_x + x, ink_l, ink_r, fw_local))
                segs = tmp

            self.__vb_blit_line_solid(write_row, direction, segs, fg_tbl, packed_fg, packed_bg, gh, y_off)
            
            remain = gh
            while remain > 0:
                shift = step_px if remain >= step_px else remain
                head, write_row = self.__scroll_v_advance(head, write_row, disp_h, vb_h, direction, shift, speed_ms, update_flag, packed_bg)
                remain -= shift
        
            i = end_i

        tail = disp_h
        while tail > 0:
            shift = step_px if tail >= step_px else tail
            head, write_row = self.__scroll_v_advance(head, write_row, disp_h, vb_h, direction, shift, speed_ms, update_flag, packed_bg)
            tail -= shift

        n = self.__v_tail_cleanup(head, direction, step_px, packed_bg)
        if n > 0:
            self.__present_from_vb(head, 1 if update_flag else 0)

        self._vb_head_row  = head
        self._vb_write_row = write_row

    @micropython.native
    def __scroll_text_v_shader(self, text: str, direction: str, x: int, y: int, fg_shader, bg, step_px: int, space_scale: float, left_margin: int, right_margin: int, up_margin: int, down_margin: int, speed_ms: int, update_flag: bool) -> None:
        fw, fh = self.font_width, self.font_height
        chars = list(text)

        gh = up_margin + fh + down_margin
        y_off = up_margin
        packed_bg, disp_h, vb_h, head, write_row = self.__scroll_v_setup(bg, direction, gh)

        i = 0; n = len(chars)
        
        max_w = self._fb_width - (x if x > 0 else 0)
        while i < n:
            end_i, segs, line_w = self.__line_fit(chars, i, space_scale, left_margin, right_margin, max_w)

            if x:
                segs = [(gi, ch_idx, ch, dst_x + x, ink_l, ink_r, fw_local) for (gi, ch_idx, ch, dst_x, ink_l, ink_r, fw_local) in segs]

            if isinstance(fg_shader, (tuple, list)) and fg_shader and isinstance(fg_shader[0], int) and 1 <= fg_shader[0] <= 4:
                self.__vb_blit_line_shader_spec(write_row, direction, segs, fg_shader, packed_bg, gh, y_off)
            else:
                self.__vb_blit_line_shader(write_row, direction, segs, fg_shader, packed_bg, gh, y_off)
                        
            remain = gh
            while remain > 0:
                shift = step_px if remain >= step_px else remain
                head, write_row = self.__scroll_v_advance(head, write_row, disp_h, vb_h, direction, shift, speed_ms, update_flag, packed_bg)
                remain -= shift
        
            i = end_i

        tail = disp_h
        
        while tail > 0:
            shift = step_px if tail >= step_px else tail
            head, write_row = self.__scroll_v_advance(head, write_row, disp_h, vb_h, direction, shift, speed_ms, update_flag, packed_bg)
            tail -= shift

        n = self.__v_tail_cleanup(head, direction, step_px, packed_bg)
        if n > 0:
            self.__present_from_vb(head, 1 if update_flag else 0)

        self._vb_head_row  = head
        self._vb_write_row = write_row

