from . import (
    utime,
    ticle
)


class BtAudioAmp:
    TYPE_A_HOLD = (50, 1300)
    TYPE_B_HOLD = (100, 900)

    MODE_BT = 0
    MODE_RADIO = 1
    MODE_AUX = 2

    def __init__(self, mode, scan, down, up, *, hold_type=TYPE_A_HOLD, init_state=MODE_BT, is_playing=False, volume=30, volume_low_offset=800):
        """
        Bluetooth Audio Amplifier Control
        
        :param mode: pin number for mode control
        :param scan: pin number for scan control
        :param down: pin number for down/previous control
        :param up: pin number for up/next control
        """
        self._baa = ticle.Dout([mode, scan, down, up]) 
        
        self._shot_hold_ms = hold_type[0]
        self._long_hold_ms = hold_type[1]
        self._current_mode = init_state
        self._is_playing = is_playing
        self._current_volume = volume
        self._volume_low_offset = volume_low_offset

    def __press(self, idx, hold_ms):
        """
        Press a button for a specified duration.
        :param pin: Pin to press
        :param hold_ms: Duration to press the button in milliseconds
        """
        self._baa[idx] = 1
        utime.sleep_ms(hold_ms)
        self._baa[idx] = 0
        utime.sleep_us(80)

    def __get_volume_delay(self, volume:int) -> int:
        """
        Get the delay for volume change based on the current volume.
        
        :param volume: int, current volume (0-30)
        :return: int, delay in milliseconds
        """
        if not 0 <= volume <= 30:
            raise ValueError("Volume must be between 0 and 30")

        # Cumulative sum of one cycle: [0,100,300,400,600]  (grouped by 5, increasing by self._volume_low_offset )
        partial = [0, 100, 300, 400, 600]

        # Quotient and remainder calculation
        q, r = divmod(volume - 1, 5)

        return self._long_hold_ms + self._volume_low_offset * q + partial[r]

    def set_playing(self, is_playing:bool):
        """
        Set the current playing state of the amplifier.
        
        :param is_playing: bool, True if playing, False otherwise
        """
        self._is_playing = is_playing

    @property
    def mode(self):
        """
        Get the current mode of the amplifier.
        :return: int, current mode (0: BT, 1: RADIO, 2: AUX)
        """
        return self._current_mode

    @mode.setter
    def mode(self, mode:int):
        """
        Set the current mode of the amplifier.

        :param mode: int, current mode (0: BT, 1: RADIO, 2: AUX)
        """
        self._current_mode = mode
        if self._current_mode == self.MODE_RADIO or self._current_mode == self.MODE_AUX:
            self.set_playing(True)
        elif self._current_mode == self.MODE_BT:
            self.set_playing(False)

    @property
    def is_playing(self):
        """
        Get the current playing state of the amplifier.
        :return: bool, True if playing, False otherwise
        """
        return self._is_playing
    
    @property
    def volume(self):
        """
        Get the current volume of the amplifier.
        :return: int, current volume (0-100)
        """
        return self._current_volume
    
    @volume.setter
    def volume(self, volume:int):
        """
        Set the current volume of the amplifier.

        :param volume: int, current volume (0-30)
        """
        current_volume = self.volume
        
        if not (0 <= volume <= 30):
            raise ValueError("Volume must be between 0 and 30")
        
        if current_volume == volume:
            return

        diff = abs(current_volume - volume)
        delay = self.__get_volume_delay(diff)
        if current_volume > volume:
            current_volume -= diff
            self.__press(2, delay)
        else:
            current_volume += diff
            self.__press(3, delay)
        
        self._current_volume = current_volume
 
    def change_mode(self):
        """
        Press the mode button for a specified duration.
        """
        self.__press(0, self._shot_hold_ms)
        self._current_mode = (self._current_mode + 1) % 3
        if self._current_mode in (self.MODE_RADIO, self.MODE_AUX):
            self.set_playing(True)
        elif self._current_mode == self.MODE_BT:
            self.set_playing(False)

    def down(self):
        """
        Press the down/previous button for a specified duration.
        """
        self._current_volume -= 1
        if self._current_volume < 0:
            self._current_volume = 0
        else:
            self.__press(2, self._long_hold_ms)

    def up(self):
        """
        Press the up/next button for a specified duration.
        """
        self._current_volume += 1
        if self._current_volume > 30:
            self._current_volume = 30
        else:
            self.__press(3, self._long_hold_ms)

    def prev(self):   
        """
        Press the down/previous button for a specified duration.
        """
        self.__press(2, self._shot_hold_ms)        

    def next(self):
        """
        Press the up/next button for a specified duration.
        """
        self.__press(3, self._shot_hold_ms)

    def pause_resume(self):
        """
        Press the mode button for a specified duration.
        :param duration: Duration to press the button in milliseconds
        """
        self.__press(1, self._shot_hold_ms)
    
    def radio_scan(self):
        """
        Press the scan button for a specified duration.
        """
        if self._current_mode != self.MODE_RADIO:
            raise ValueError("Scan is only available in RADIO mode")
        self.__press(1, self._long_hold_ms)
