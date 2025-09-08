from . import (
    math,
    urandom,
    machine, 
    ext
)


class WS2812Matrix_Effect:
    def __init__(self, ws:ext.WS2812Matrix):
        """
        A class to apply various effects on a WS2812 LED matrix.
        It provides methods to create effects like sparkle, meteor rain, plasma, fireworks, campfire, and wave RGB.
        
        :param ws: WS2812Matrix object, the LED matrix to apply effects on
        """
        self.__ws           = ws
        self.__timer        = None
        self.__state        = {}
        self.__effect_id    = 0
        self.__busy         = False

    def __install(self, period_s:float, handler):
        """
        Install a periodic callback to handle the effect.
        :param period_s: float, period in seconds for the effect
        :param handler: callable, the function to call at each period
        """
        self.stop()
        self.__effect_id += 1
        eid = self.__effect_id
        period_ms = int(period_s * 1000)

        def __cb(t): 
            if eid != self.__effect_id or self.__busy:
                return
            self.__busy = True
            try:
                handler()
            finally:
                self.__busy = False

        tm = machine.Timer()
        tm.init(period=period_ms, mode=machine.Timer.PERIODIC, callback=__cb)
        self.__timer = tm

    def stop(self):
        """
        Stop the current effect and reset the effect ID.
        """
        self.__effect_id += 1
        if self.__timer:
            self.__timer.deinit()
            self.__timer = None

    def __wheel(self, pos:int):
        """
        Convert a position value to an RGB color using the color wheel algorithm.
        :param pos: int, position value (0-255)
        :return: tuple, RGB color
        """
        pos &= 255
        if pos < 85:
            return (255 - pos * 3, pos * 3, 0)
        if pos < 170:
            pos -= 85
            return (0, 255 - pos * 3, pos * 3)
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)

    def __heat_color(self, t:int):
        """
        Convert a temperature value to an RGB color.
        
        :param t: int, temperature value (0-255)
        :return: tuple, RGB color
        """
        if t <= 85:
            return (t * 3, 0, 0)
        if t <= 170:
            t -= 85
            return (255, t * 3, 0)
        t -= 170
        return (255, 255, t * 3)

    def sparkle(self, *, base=(0,0,0), sparkle_color=(255,255,255), density=0.1, decay=0.9, speed=0.03):
        """
        Create a sparkle effect on the LED matrix.
        
        :param base: tuple, RGB color for the base color of the matrix
        :param sparkle_color: tuple, RGB color for the sparkle
        :param density: float, density of the sparkles (0-1)
        :param decay: float, decay factor for the sparkle brightness (0-1)
        :param speed: float, speed of the effect in seconds
        """
        self.__ws.fill(base)
        self.__state['spark'] = {'decay': decay, 'dens': density, 'sc': sparkle_color}
        self.__install(speed, self.__sparkle_step)

    def __sparkle_step(self):
        """
        Step function for the sparkle effect.
        It decays the brightness of existing sparkles and adds new sparkles based on the density.
        """
        ws = self.__ws
        w, h = ws.display_w, ws.display_h
        N = w * h
        s = self.__state['spark']
        for i in range(N):
            x, y = i % w, i // w
            r, g, b = ws[x, y]
            ws[x, y] = (int(r * s['decay']), int(g * s['decay']), int(b * s['decay']))
        if urandom.getrandbits(16) < int(65535 * s['dens']):
            idx = urandom.getrandbits(16) % N
            x, y = idx % w, idx // w
            ws[x, y] = s['sc']
        ws.update()

    def meteor_rain(self, *, colors=((255,0,0),(0,0,255)), count=3, decay=0.8, speed=0.04):
        """
        Create a meteor rain effect on the LED matrix.
        
        :param colors: tuple, list of RGB colors for the meteors
        :param count: int, number of meteors to create
        :param decay: float, decay factor for the meteor brightness (0-1)
        :param speed: float, speed of the effect in seconds
        """
        w, h = self.__ws.display_w, self.__ws.display_h
        N = w * h
        mets = [{
            'pos': urandom.getrandbits(16) % N,
            'spd': 1 + urandom.getrandbits(2),
            'col': colors[urandom.getrandbits(8) % len(colors)]
        } for _ in range(count)]
        self.__state['meteor'] = {'ms': mets, 'decay': decay}
        self.__install(speed, self.__meteor_step)

    def __meteor_step(self):
        """
        Step function for the meteor rain effect.
        It decays the brightness of existing pixels and updates the position and color of meteors.
        """
        ws = self.__ws
        w, h = ws.display_w, ws.display_h
        N = w * h
        s = self.__state['meteor']
        for i in range(N):
            x, y = i % w, i // w
            r, g, b = ws[x, y]
            ws[x, y] = (int(r * s['decay']), int(g * s['decay']), int(b * s['decay']))
        for m in s['ms']:
            x, y = m['pos'] % w, m['pos'] // w
            ws[x, y] = m['col']
            m['pos'] = (m['pos'] + m['spd']) % N
        ws.update()

    def plasma(self, *, hue_shift=2, speed=0.05):
        """
        Create a plasma effect on the LED matrix.
        
        :param hue_shift: int, the shift in hue for the plasma effect
        :param speed: float, speed of the effect in seconds
        """
        self.__state['plasma'] = {'t': 0, 'shift': hue_shift}
        self.__install(speed, self.__plasma_step)

    def __plasma_step(self):
        """
        Step function for the plasma effect.
        It calculates the color for each pixel based on a sine wave function and updates the matrix.
        """
        ws = self.__ws
        w, h = ws.display_w, ws.display_h
        st = self.__state['plasma']
        t = st['t']
        for y in range(h):
            for x in range(w):
                hval = (math.sin(x * 0.5 + t) + math.sin(y * 0.5 + t)) * 180 + t
                ws[x, y] = self.__wheel(int(hval) & 255)
        st['t'] += st['shift']
        ws.update()

    def fireworks(self, *, sparks=24, fade=0.9, speed=0.03, colors=((255,128,0),(255,255,255),(0,255,255))):
        """
        Create a fireworks effect on the LED matrix.
        
        :param sparks: int, number of sparks in the fireworks
        :param fade: float, fade factor for the sparks (0-1)
        :param speed: float, speed of the effect in seconds
        :param colors: tuple, list of RGB colors for the sparks
        """
        self.__state['fire'] = {'parts': [], 'fade': fade, 'colors': colors, 'cool': 0, 'sparks': sparks}
        self.__fire_spawn()
        self.__install(speed, self.__fire_step)

    def __fire_spawn(self):
        """
        Spawn new sparks for the fireworks effect.
        It clears existing sparks and generates new ones at a random position on the matrix.
        """
        ws = self.__ws
        w, h = ws.display_w, ws.display_h
        N = w * h
        s = self.__state['fire']
        s['parts'].clear()
        center = urandom.getrandbits(16) % N
        for _ in range(s['sparks']):
            vel = (urandom.getrandbits(3) % 5) + 1
            dir_ = 1 if urandom.getrandbits(1) else -1
            s['parts'].append({
                'pos': center,
                'vel': vel * dir_,
                'col': s['colors'][urandom.getrandbits(8) % len(s['colors'])]
            })

    def __fire_step(self):
        """
        Step function for the fireworks effect.
        It fades existing sparks, updates their positions, and spawns new sparks if necessary.
        """
        ws = self.__ws
        w, h = ws.display_w, ws.display_h
        N = w * h
        s = self.__state['fire']
        # fade
        for i in range(N):
            x, y = i % w, i // w
            r, g, b = ws[x, y]
            ws[x, y] = (int(r * s['fade']), int(g * s['fade']), int(b * s['fade']))
        alive = False
        for p in s['parts']:
            p['pos'] = (p['pos'] + p['vel']) % N
            x, y = p['pos'] % w, p['pos'] // w
            ws[x, y] = p['col']
            alive = True
        s['cool'] += 1
        if not alive or s['cool'] > 25:
            self.__fire_spawn()
            s['cool'] = 0
        ws.update()

    def campfire(self, *, cooling=55, sparking=120, speed=0.03):
        """
        Create a campfire effect on the LED matrix.
        
        :param cooling: int, cooling factor for the heat (0-255)
        :param sparking: int, sparking factor for the heat (0-255)
        :param speed: float, speed of the effect in seconds
        """
        w, h = self.__ws.display_w, self.__ws.display_h
        N = w * h
        self.__state['camp'] = {'heat': [0]*N, 'cool': cooling, 'spark': sparking}
        self.__install(speed, self.__camp_step)

    def __camp_step(self):
        """
        Step function for the campfire effect.
        It cools down the heat, drifts it up, and adds sparks randomly.
        """
        ws = self.__ws
        w, h = ws.display_w, ws.display_h
        s = self.__state['camp']
        heat = s['heat']
        N = w * h
        # cool down
        for i in range(N):
            cool = urandom.getrandbits(8) % ((s['cool'] * 10 // N) + 2)
            heat[i] = max(0, heat[i] - cool)
        # drift up
        for i in range(N-1, 1, -1):
            heat[i] = (heat[i-1] + heat[i-2] + heat[i-2]) // 3
        # spark
        if urandom.getrandbits(8) < s['spark']:
            idx = urandom.getrandbits(8) % min(3, N)
            heat[idx] = min(255, heat[idx] + urandom.getrandbits(8)%95 + 160)
        # map to color
        for i in range(N):
            x, y = i % w, i // w
            ws[x, y] = self.__heat_color(heat[i])
        ws.update()

    def wave_rgb(self, *, speed=0.1):
        """
        Create a wave RGB effect on the LED matrix.
        
        :param speed: float, speed of the effect in seconds
        """
        self.__state['wave'] = {'step': 0}
        self.__install(speed, self.__wave_step)

    def __wave_step(self):
        """
        Step function for the wave RGB effect.
        It calculates the color for each pixel based on a sine wave function and updates the matrix.
        """
        ws = self.__ws
        w, h = ws.display_w, ws.display_h
        s = self.__state['wave']
        step = s['step']
        N = w * h
        for i in range(N):
            base = step + i * (360 / N)
            r = int((math.sin(math.radians(base)) + 1)/2 * 255)
            g = int((math.sin(math.radians(base+120)) + 1)/2 * 255)
            b = int((math.sin(math.radians(base+240)) + 1)/2 * 255)
            x, y = i % w, i // w
            ws[x, y] = (r, g, b)
        s['step'] = (step + 5) % 360
        ws.update()