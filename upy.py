import os 
import sys
import time
import re
import platform
import threading
import posixpath
import ast
import textwrap
import shutil
import stat
import subprocess
import json
import urllib.request
import pathlib
from typing import Callable, Optional, Tuple
import click

import serial
from serial.tools import list_ports
from genlib.ansiec import ANSIEC
 

from . import __version__  

#--------------------------------------------------------------

_stdout_lock = threading.Lock()
_buffer = b''
_expected_bytes = 0

_error_header       = b"Traceback (most recent call last):"
_error_header_buf   = b""
_skip_error_output  = False

def stdout_write_bytes(b):
    global _buffer, _expected_bytes
    global _error_header_buf, _skip_error_output

    if _skip_error_output:
        return

    _error_header_buf = (_error_header_buf + b)[-len(_error_header):]
    if _error_header_buf == _error_header:
        _skip_error_output = True
        return
    
    if b in (b'\x04', b''): # EOF or empty byte
        return

    with _stdout_lock:
        if _expected_bytes:
            _buffer += b
            _expected_bytes -= 1

            if _expected_bytes == 0: 
                sys.stdout.buffer.write(_buffer)
                sys.stdout.buffer.flush()
                _buffer = b''
                return 

        elif b[0] <= 0x7F:              # ASCII
            sys.stdout.buffer.write(b)
            sys.stdout.buffer.flush()
        else:                           # UTF-8 
            hdr = b[0]   
            if   hdr & 0xF8 == 0xF0: _expected_bytes = 3  
            elif hdr & 0xF0 == 0xE0: _expected_bytes = 2  
            elif hdr & 0xE0 == 0xC0: _expected_bytes = 1
            else:                       # Invalid UTF-8   
                sys.stdout.buffer.write(b.hex().encode())
                return 
            _buffer = b                  # save first byte

#--------------------------------------------------------------

IS_WINDOWS: bool = platform.system() == "Windows"
CR, LF = b"\r", b"\n"

_EXTMAP : dict[str, bytes] = {
    "H": b"\x1b[A",   # ↑
    "P": b"\x1b[B",   # ↓
    "M": b"\x1b[C",   # →
    "K": b"\x1b[D",   # ←
    "G": b"\x1b[H",   # Home
    "O": b"\x1b[F",   # End
    "R": b"\x1b[2~",  # Ins
    "S": b"\x1b[3~",  # Del
}

def _utf8_need_follow(b0: int) -> int:
    if b0 & 0b1000_0000 == 0:          # 0xxxxxxx → ASCII
        return 0
    if b0 & 0b1110_0000 == 0b1100_0000:    # 110xxxxx
        return 1
    if b0 & 0b1111_0000 == 0b1110_0000:    # 1110xxxx
        return 2
    if b0 & 0b1111_1000 == 0b1111_0000:    # 11110xxx
        return 3
    return 0                               

if IS_WINDOWS:
    import msvcrt
    from typing import Callable

    def getch() -> bytes:
        w = msvcrt.getwch()
        if w in ("\x00", "\xe0"):  # arrow keys ect.
            return _EXTMAP.get(msvcrt.getwch(), b"")
        return w.encode("utf-8")
else:                            
    import tty
    import termios
    import atexit
    import signal

    _FD = sys.stdin.fileno()
    _OLD = None
    _RAW_MODE_ACTIVE = False

    def _initialize_terminal():
        global _OLD
        if _OLD is None:
            _OLD = termios.tcgetattr(_FD)

    def _raw(on: bool):
        global _RAW_MODE_ACTIVE
        try:
            if on:
                _initialize_terminal()
                tty.setraw(_FD)
                _RAW_MODE_ACTIVE = True
            else:
                if _OLD is not None:
                    termios.tcsetattr(_FD, termios.TCSADRAIN, _OLD)
                _RAW_MODE_ACTIVE = False
        except Exception:
            pass

    def _restore_terminal():
        if _RAW_MODE_ACTIVE:
            _raw(False)

    def _signal_handler(signum, frame):
        _restore_terminal()
        sys.exit(0)

    atexit.register(_restore_terminal)
    
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    def getch() -> bytes:
        try:
            _raw(True)
            first = os.read(_FD, 1)
            need = _utf8_need_follow(first[0])
            return first + (os.read(_FD, need) if need else b"")
        except Exception:
            return b""
        finally:
            _raw(False)

if IS_WINDOWS:
    import msvcrt
    from typing import Callable

    _PUTB: Callable[[bytes], None] = msvcrt.putch 
    _PUTW: Callable[[str], None]   = msvcrt.putwch 
    
    def _write_bytes(data: bytes) -> None:
        sys.stdout.buffer.write(data)
        sys.stdout.flush()
        
    def putch(data: bytes) -> None:
        if data == CR: 
            _PUTB(LF);  return

        if len(data) > 1 and data.startswith(b"\x1b["):
            _write_bytes(data)   
        elif len(data) == 1 and data < b"\x80":  
            _PUTB(data)
        else:                                         
            _PUTW(data.decode("utf-8", "strict"))  
          
else:
    def putch(data: bytes) -> None:
        if data != CR:
            sys.stdout.buffer.write(data)
            sys.stdout.flush()

#--------------------------------------------------------------
class UpyBoardError(BaseException):
    """
    Custom exception for UpyBoard operations.
    """
    def __init__(self, message: str):
        super().__init__(message)
        self.message = message

    def __str__(self):
        return f"UpyBoardError: {self.message}"


class UpyBoard:
    """
    A class to interact with a MicroPython device over a serial connection.
    This class provides methods to execute commands, manage files, and interact with the device's REPL.
    """
    _REPL_BUFSIZE = 2048  # Min 256  
    
    _CTRL_A = b'\x01'
    _CTRL_B = b'\x02'
    _CTRL_C = b'\x03'
    _CTRL_D = b'\x04'
    _EOF_MARKER = b'\x04'
    _RAW_REPL_PROMPT = b'raw REPL; CTRL-B to exit\r\n>'
    _SOFT_REBOOT_MSG = b'soft reboot\r\n'
    _OK_RESPONSE = b'OK'
    _DEIVCE_CHUNK_SIZES = 4096
    
    def __init__(self, port:str, baudrate:int=115200):
        """
        Initialize the UpyBoard instance with the specified serial port and baud rate.
        :param port: The serial port to connect to.
        :param baudrate: The baud rate for the serial connection (default is 115200).
        :raises UpyBoardError: If the serial port cannot be opened or if the device is not found.
        """
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1.0, write_timeout=1.0)
            self.__stop_event = threading.Event()
            self.__follow_thread = None

        except serial.SerialException as e:
            raise UpyBoardError(f"failed to open {port} ({e})")
        except (OSError, IOError): 
            raise UpyBoardError(f"failed to open {port} (device not found)")
    
        self.__init_repl()

    def __init_repl(self):
        """
        Initialize the REPL (Read-Eval-Print Loop) for the UpyBoard.
        This function sets up the serial connection and prepares the board for REPL interaction.
        """
        self.serial_reader_running = None
        self.serial_out_put_enable = True
        self.serial_out_put_count = 0

    def __write(self, data:bytes) -> None:
        """
        Write data to the serial port.
        :param data: The data to write to the serial port.
        """
        self.serial.write(data)

    def __read(self, n:int=1) -> bytes:
        """
        Read a specified number of bytes from the serial port.
        :param n: Number of bytes to read from the serial port.
        :return: The bytes read from the serial port.
        """
        return self.serial.read(n)
    
    def __read_ex(self, min_num_bytes:int, ending:bytes, timeout:int=5, data_consumer:Optional[Callable[[bytes], None]]=None) -> bytes:
        """
        Read data from the serial port until the specified ending is found or timeout occurs.
        :param min_num_bytes: Minimum number of bytes to read before checking for the ending.
        :param ending: The byte sequence that indicates the end of the data.
        :param timeout: Maximum time to wait for the ending in seconds (0 = no timeout).
        :param data_consumer: Optional callback function to process the data as it is read.
        :return: The data read from the serial port as bytes.
        :raises UpyBoardError: If timeout occurs or serial communication fails.
        """
        data = b''
        start_time = time.time()
        last_activity = start_time
        check_interval = 0.01
        
        try:
            if min_num_bytes > 0:
                initial_data = self.__read(min_num_bytes)
                if initial_data:
                    data += initial_data
                    if data_consumer:
                        data_consumer(initial_data)
                    last_activity = time.time()
            
            while not data.endswith(ending):
                if self.__stop_event.is_set():
                    break
                
                current_time = time.time()
                
                if timeout > 0 and (current_time - start_time) >= timeout:
                    break
                
                if self.serial.in_waiting > 0:
                    new_data = self.__read(1)
                    if new_data:
                        data += new_data
                        if data_consumer:
                            data_consumer(new_data)
                        last_activity = current_time
                else:
                    time.sleep(check_interval)                    
                    if timeout > 0 and (current_time - last_activity) > max(timeout * 2, 10):
                        break
            
            return data
            
        except serial.SerialException as e:
            if not self.__stop_event.is_set():
                raise UpyBoardError(f"Serial communication error: {e}")
            return data
        except Exception as e:
            if isinstance(e, UpyBoardError):
                raise
            return data

    def __enter_repl(self, soft_reset:bool=True):
        """
        Enter the raw REPL mode of the device.
        This function sends the necessary commands to the device to enter the raw REPL mode.
        :param soft_reset: If True, perform a soft reset before entering the raw REPL.
        """
        # Clear any pending data
        self.serial.write(b'\r' + self._CTRL_C + self._CTRL_C)  # Double Ctrl-C
        time.sleep(0.1)
        self.serial.reset_input_buffer()
        
        # Enter raw REPL
        self.serial.write(b'\r' + self._CTRL_A)
        
        if soft_reset:
            data = self.__read_ex(1, self._RAW_REPL_PROMPT, timeout=3)
            if not data.endswith(self._RAW_REPL_PROMPT):
                raise UpyBoardError('could not enter raw repl')

            self.serial.write(self._CTRL_D)  # soft reset
            data = self.__read_ex(1, self._SOFT_REBOOT_MSG, timeout=3)
            if not data.endswith(self._SOFT_REBOOT_MSG):
                raise UpyBoardError('could not enter raw repl')
            
        # Wait for the raw REPL prompt
        data = self.__read_ex(1, self._RAW_REPL_PROMPT[:-1], timeout=3)
        if not data.endswith(self._RAW_REPL_PROMPT[:-1]):
            raise UpyBoardError('could not enter raw repl')
            
    def __leave_repl(self):
        """
        Leave the raw REPL mode of the device.
        """
        self.serial.write(b'\r' + self._CTRL_B)  # enter friendly REPL
        
    def __follow_task(self, echo:bool):
        """
        Follow the REPL output and echo input characters.
        This function reads characters from the input and writes them to the serial port.
        If echo is True, it also writes the characters to stdout.
        :param echo: If True, echo the input characters to stdout.
        """
        try:
            while not self.__stop_event.is_set():
                try:
                    ch = getch()    
                    if ch == self._CTRL_C:
                        self.__interrupt_requested = True
                        
                        for _ in range(3):
                            self.serial.write(self._CTRL_C)  # keyboard interrupt
                            time.sleep(0.01)
                        
                        time.sleep(0.1)
                        if _core != "EFR32MG":
                            self.serial.write(self._CTRL_D)  # soft reset
                            time.sleep(0.1)
                        
                        return
                    elif ch == self._CTRL_D:
                        return
                    
                    if echo:
                        putch(ch)
                    self.__write(CR if ch == LF else ch)  # convert LF('\n') to CR('\r') for REPL compatibility
                except:
                    pass
        finally:
            self.__stop_event.set()
                                        
    def __exec(self, command:str|bytes, stream_output:bool=False, echo:bool=False) -> bytes:
        """
        Execute a command on the device and return the output.
        :param command: The command to execute.
        :param stream_output: If True, stream the output to stdout.
        :param echo: If True, echo the command to stdout.
        :return: The output of the command as bytes.
        """
        if isinstance(command, str):
            command = command.encode('utf-8')
        
        data_err = b''
        data_consumer = stdout_write_bytes if stream_output else None
        follow_thread = None   

        data = self.__read_ex(1, b'>')
        if not data.endswith(b'>'):
            raise UpyBoardError('could not enter raw repl')

        command_len = len(command)
        
        current_buffer_size = 1024
        max_buffer_size = 8192
        bytes_sent = 0
        
        # Adaptive sending with performance monitoring
        start_time = time.time()
        
        while bytes_sent < command_len:
            chunk_start = bytes_sent
            chunk_end = min(bytes_sent + current_buffer_size, command_len)
            chunk = command[chunk_start:chunk_end]
            
            chunk_start_time = time.time()
            self.serial.write(chunk)
            bytes_sent += len(chunk)
            
            chunk_time = time.time() - chunk_start_time
            if chunk_time < 0.01 and current_buffer_size < max_buffer_size:
                current_buffer_size = min(current_buffer_size * 2, max_buffer_size)
            elif chunk_time > 0.05:
                current_buffer_size = max(current_buffer_size // 2, 512)
            
            if bytes_sent % 32768 == 0:  # Every 32KB
                time.sleep(0.005)  # 5ms pause
        
        self.serial.write(self._EOF_MARKER)
        
        transfer_time = time.time() - start_time
        timeout = max(5, int(transfer_time * 2))
        
        data = self.__read_ex(1, self._OK_RESPONSE, timeout=timeout)
        if not data.endswith(self._OK_RESPONSE):
            raise UpyBoardError('could not execute command (response: %r)' % data)
        
        if stream_output:            
            self.__stop_event.clear()
            self.__interrupt_requested = False 
            follow_thread = threading.Thread(target=self.__follow_task, args=(echo,), daemon=True)
            follow_thread.start()

        try:
            # Read first data
            data = self.__read_ex(1, self._EOF_MARKER, 0, data_consumer)

            if stream_output and self.__stop_event.is_set():
                if hasattr(self, '__interrupt_requested') and self.__interrupt_requested:
                    # Force interrupt cleanup
                    return data
                else:
                    # Normal termination (Ctrl+D)
                    return data
                
            if not data.endswith(self._EOF_MARKER):
                raise UpyBoardError('timeout waiting for first EOF reception')
            data = data[:-1]

            # read error data
            data_err = self.__read_ex(1, self._EOF_MARKER, 0, data_consumer)
            if not data_err.endswith(self._EOF_MARKER):
                raise UpyBoardError('timeout waiting for second EOF reception')
            data_err = data_err[:-1]
            
        finally:
            if stream_output and follow_thread and follow_thread.is_alive():
                self.__stop_event.set()
                try:
                    follow_thread.join(timeout=1.0)
                    if follow_thread.is_alive():
                        # Force termination logging if needed
                        pass
                except:
                    pass        
                    
        if data_err:
            raise UpyBoardError(data_err.decode('utf-8', errors='replace'))

        return data
                
    def __drain_eof(self, max_ms:int=200):
        """
        Drain the serial input buffer until EOF is received or timeout occurs.
        :param max_ms: Maximum time to wait for EOF in milliseconds.
        """
        deadline = time.time() + max_ms / 1000
        while time.time() < deadline:
            waiting = self.serial.in_waiting
            if waiting:
                _ = self.__read(waiting) 
            else:
                time.sleep(0.01)
                
    def __repl_serial_to_stdout(self):        
        """
        Read data from the serial port and write it to stdout.
        """
        def hexsend(string_data=''):
            import binascii
            hex_data = binascii.unhexlify(string_data)
            return hex_data

        try:
            data = b''
            try:
                while self.serial_reader_running:
                    count = self.serial.in_waiting
                    if count == 0: 
                        time.sleep(0.01)
                        continue

                    if count > 0:
                        data += self.serial.read(count)

                        if len(data) < 20:
                            try:
                                data.decode()
                            except UnicodeDecodeError:
                                continue

                        if data != b'':
                            if self.serial_out_put_enable and self.serial_out_put_count > 0:
                                if platform.system() == 'Windows':   
                                    sys.stdout.buffer.write(data.replace(b"\r", b""))
                                else:
                                    sys.stdout.buffer.write(data)
                                    
                                sys.stdout.buffer.flush()
                        else:
                            self.serial.write(hexsend(data))

                        data = b''
                        self.serial_out_put_count += 1
            except:
                print('')
                return
        except KeyboardInterrupt:
            if serial != None:
                serial.close()

    def __fs_ls_fallback(self, dir:str="/") -> list[Tuple[str, int, bool]]:
        """
        Fallback method for listing directory contents (original implementation).
        """
        if not dir.startswith("/"):
            dir = "/" + dir
            
        command = f"""
            import os
            def listdir(dir):
                if dir == '/':                
                    return sorted([dir + f for f in os.listdir(dir)])
                else:
                    return sorted([dir + '/' + f for f in os.listdir(dir)])
            print(listdir('{dir}'))
        """
        out = self.exec(command)
        file_list = ast.literal_eval(out.decode("utf-8"))
        
        # Convert to detailed format for compatibility
        result = []
        for f in file_list:
            f_name = f.split("/")[-1]
            is_dir = self.fs_is_dir(f)
            size = 0 if is_dir else self.fs_state(f)
            result.append([f_name, size, is_dir])
        
        return result

    def __reset(self):
        """
        Reset the device by executing a soft reset command. 
        """
        command = f"""
            import machine
            machine.soft_reset()  # Ctrl+D
        """
        self.exec(command)
        
    def exec(self, command:str=None):
        """
        Run a command or script on the device.
        :param command: The command to execute.
        """
        self.__enter_repl()
        try:
            command = textwrap.dedent(command)
            out = self.__exec(command)
        finally:
            self.__leave_repl()
    
        return out
    
    def run(self, local, stream_output:bool=False, echo:bool=False):
        """
        Run a command or script on the device.
        :param local: Path to the script file to execute.
        :param stream_output: If True, stream the output to stdout.
        :param echo: If True, echo the command to stdout.
        """
        self.__enter_repl()
        try:
            with open(local, "rb") as f:
                data = f.read()
            self.__exec(data, stream_output, echo)
            self.__drain_eof(max_ms=200)
        finally:
            self.__leave_repl()
    
    def close(self):
        """
        Close the serial connection.
        """
        self.serial.close()

    def reset(self):
        self.__write(b'\r' + self._CTRL_D)  
 
    def repl(self):
        """
        Enter the REPL mode, allowing interaction with the device.
        """
        self.serial_reader_running = True
        self.serial_out_put_enable = True
        self.serial_out_put_count = 1

        self.__reset()
        self.__read_ex(1, b'\x3E\x3E\x3E', timeout=1) # read prompt >>>

        repl_thread = threading.Thread(target=self.__repl_serial_to_stdout, daemon=True, name='REPL')
        repl_thread.start()

        self.serial.write(b'\r') # Update prompt
        
        while True:
            char = getch()

            if char == b'\x07': 
                self.serial_out_put_enable = False
                continue
            elif char == b'\x0F': 
                self.serial_out_put_enable = True
                self.serial_out_put_count = 0
                continue
            elif char == b'\x00' or not char: # Ignore null characters
                continue
            elif char == self._CTRL_C:  # Ctrl + C to exit repl mode
                break
            
            try:
                self.serial.write(b'\r' if char == b'\n' else char)
            except:
                break
            
        self.serial_reader_running = False
        self.__reset()
        print('')

    def fs_get(self, remote:str, local:str):
        """ 
        Download a file from the connected device to the local filesystem.
        :param remote: The path to the file on the device.
        :param local: The local path where the file should be saved.
        :raises UpyBoardError: If the file is empty or if the download fails.
        """
        local_file = None
        
        if local:
            if os.path.isdir(local):
                local = os.path.join(local, os.path.basename(remote))
            local_file = open(local, "wb")
        
        bytes_read = 0
        bar_length = 40 

        if local_file:
            print(f"{ANSIEC.FG.BRIGHT_BLUE}{remote.replace(_device_root_fs, '', 1)}{ANSIEC.OP.RESET}")

        try:
            file_size = self.fs_state(remote)

            self.__enter_repl()

            init_command = f"""
                import sys
                f = open('{remote}', 'rb')
                """
            self.__exec(textwrap.dedent(init_command))
                            
            while bytes_read < file_size:
                remaining = min(self._DEIVCE_CHUNK_SIZES, file_size - bytes_read)

                read_cmd = f"""
                    chunk = f.read({remaining})
                    if chunk:
                        sys.stdout.buffer.write(chunk)
                    """
                chunk_data = self.__exec(textwrap.dedent(read_cmd))
                
                if chunk_data:
                    if local_file:
                        local_file.write(chunk_data)
                    else:
                         sys.stdout.buffer.write(chunk_data)
                         sys.stdout.flush()
                
                    bytes_read += len(chunk_data)
                                        
                    if local_file:
                        pct = bytes_read / file_size
                        block = int(round(bar_length * pct))
                        bar = "#" * block + "-" * (bar_length - block)
                        percent = int(pct * 100)
                        print(f"{ANSIEC.OP.left()}[{bar}] {percent}% ({bytes_read}/{file_size})", end="", flush=True)
                else:
                    break

            self.__exec("f.close()")
        
        except Exception as e:
            raise UpyBoardError(f"Download failed: {e}")
        finally:
            self.__leave_repl()
            if local_file:
                print()
                local_file.close()

        if bytes_read != file_size:
            raise UpyBoardError(f"Download incomplete: got {bytes_read}/{file_size} bytes")
    
    def fs_state(self, path:str) -> int:
        """
        Return file size of given path.
        """
        if _core == "EFR32MG":
            command = f"""
                try:
                    with open('{path}', 'rb') as f:
                        f.seek(0, 2)
                        size = f.tell()
                    print(size)
                except Exception as e:
                    print(0)
            """
            out = self.exec(command)
            return int(out.decode('utf-8'))
        else:
            command = f"""
                import os
                try:
                    st = os.stat('{path}')
                    print(st[6])
                except:
                    print(0)
            """
        out = self.exec(command)
        return int(out.decode('utf-8'))

    def fs_is_dir(self, path:str) -> bool:
        """
        Check if the given path is a directory.
        :param path: The path to check.
        :return: True if the path is a directory, False otherwise.
        """
        command = f"""
            vstat = None
            try:
                from os import stat
            except ImportError:
                from os import listdir
                vstat = listdir
            def ls_dir(path):
                if vstat is None:
                    return stat(path)[0] & 0x4000 != 0
                else:
                    try:
                        vstat(path)
                        return True
                    except OSError as e:
                        return False
            print(ls_dir('{path}'))
        """
        out = self.exec(command)
        return ast.literal_eval(out.decode("utf-8"))

    def fs_ls_detailed(self, dir:str="/") -> list[Tuple[str, int, bool]]:
        """
        List the contents of a directory with detailed information (size, type) in a single operation.
        :param dir: The directory to list. Defaults to the root directory ("/").
        :return: A list of tuples containing (name, size, is_dir) for each item.
        """
        if not dir.startswith("/"):
            dir = "/" + dir

        command = f"""
            import os
            import json
            import sys
            def xbee3_zigbee_state(path):
                try:
                    with open(path, 'rb') as f:
                        f.seek(0, 2)
                        size = f.tell()
                    return size
                except Exception as e:
                    return 0

            def get_detailed_listing(path):
                try:
                    items = []
                    for item in os.listdir(path):
                        full_path = path + ('/' + item if path != '/' else item)
                        if sys.platform == 'xbee3-zigbee':
                            is_dir = False
                            size = xbee3_zigbee_state(full_path)
                            if size == 0:
                                is_dir = True
                            items.append([item, size, is_dir])
                            continue
                        try:
                            stat_info = os.stat(full_path)
                            is_dir = stat_info[0] & 0x4000 != 0
                            size = 0 if is_dir else stat_info[6]
                            items.append([item, size, is_dir])
                        except:
                            # If stat fails, try to determine if it's a directory
                            try:
                                os.listdir(full_path)
                                items.append([item, 0, True])  # It's a directory
                            except:
                                items.append([item, 0, False])  # It's a file
                    return sorted(items, key=lambda x: (not x[2], x[0].lower()))
                except Exception as e:
                    return []

            print(json.dumps(get_detailed_listing('{dir}')))
        """

        try:
            out = self.exec(command)
            result = json.loads(out.decode("utf-8").strip())
            return result
        except (json.JSONDecodeError, UpyBoardError):
            return self.__fs_ls_fallback(dir)

    def fs_mkdir(self, dir:str) -> bool:       
        """
        Create a directory on the connected device.
        :param dir: The directory to create.
        :return: True if the directory was created, False if it already exists.
        """
        command = f"""
            import os
            def mkdir(dir):
                parts = dir.split(os.sep)
                dirs = [os.sep.join(parts[:i+1]) for i in range(len(parts))]
                check = 0
                for d in dirs:
                    try:
                        os.mkdir(d)
                    except OSError as e:
                        check += 1
                        if "EEXIST" in str(e):
                            continue
                        else:
                            return False
                return check < len(parts)
            print(mkdir('{dir}'))
        """        
        out = self.exec(command)

        return ast.literal_eval(out.decode("utf-8"))

    def fs_putdir(self, local:str, remote:str):
        """
        Upload a directory and its contents to the connected device.
        :param local: The local directory to upload.
        :param remote: The remote directory path on the device.
        :param callback: Optional callback function to call with the remote file path and size.
        """        
        for parent, child_dirs, child_files in os.walk(local, followlinks=True):
            remote_parent = posixpath.normpath(posixpath.join(remote, os.path.relpath(parent, local)))
           
            try:
                self.fs_mkdir(remote_parent)
            except:
                pass

            for filename in child_files:
                local = os.path.join(parent, filename)
                remote = posixpath.join(remote_parent, filename)      
                self.fs_put(local, remote)

    def fs_put(self, local:str, remote:str):
        """
        Upload a file to the connected device.
        :param local: The local file path to upload.
        :param remote: The remote file path on the device.
        :raises UpyBoardError: If the upload fails or if the file already exists.
        """
        sent = 0
        bar_length = 40
        total = os.path.getsize(local)
         
        self.__enter_repl()
        try:
            self.__exec(f"f = open('{remote}', 'wb')")                
        except UpyBoardError as e:
            if "EEXIST" in str(e):
                self.__leave_repl()
                self.fs_rm(remote)
                self.fs_put(local, remote)
                return

        try:    
            f = open(local, "rb")
            print(f"{ANSIEC.FG.BRIGHT_BLUE}{remote.replace(_device_root_fs, '', 1)}{ANSIEC.OP.RESET}")

            while True:
                chunk = f.read(self._DEIVCE_CHUNK_SIZES)
                if not chunk:
                    break
                
                self.__exec(f"f.write({repr(chunk)})")
    
                sent += len(chunk)
                pct = sent / total
                block = int(round(bar_length * pct))
                bar = "#" * block + "-" * (bar_length - block)
                percent = int(pct * 100)
                print(f"{ANSIEC.OP.left()}[{bar}] {percent}% ({sent}/{total})", end="", flush=True)
            print()
                    
            self.__exec("f.close()")
        finally:
            self.__leave_repl()

    def fs_rm(self, filename:str):
        """
        Remove a file from the connected device.
        :param filename: The file to remove.
        """
        command = f"""
            import os
            os.remove('{filename}')
        """
        self.exec(command)

    def fs_rmdir(self, dir:str):
        """
        Remove a directory and all its contents recursively.
        :param dir: The directory to remove.
        """
        command = f"""
            import os
            def rmdir(dir):
                os.chdir(dir)
                for f in os.listdir():
                    try:
                        os.remove(f)
                    except OSError:
                        pass
                for f in os.listdir():
                    rmdir(f)
                os.chdir('..')
                os.rmdir(dir)
            rmdir('{dir}')
        """
        self.exec(command)

    def fs_format(self) -> bool:
        """
        Format the filesystem of the connected device based on its core type.
        :return: True if the filesystem was successfully formatted, False otherwise.
        """
        if _core == "ESP32":
            command = """ 
                import os
                os.fsformat('/flash')
            """
        elif _core in ("ESP32S3", "ESP32C6"):
            command = """
                import os
                from flashbdev import bdev
                os.umount('/')
                os.VfsLfs2.mkfs(bdev)
                os.mount(bdev, '/')
            """
        elif _core == "EFR32MG":
            command = """
                import os
                os.format()
            """
        elif _core == "RP2350":
            command = """
                import os
                import rp2
                bdev = rp2.Flash()
                os.VfsFat.mkfs(bdev)
            """
        else:
            return False

        try:
            self.exec(command)
        except UpyBoardError:
            return False
            
        return True
    
    def fs_df(self):
        """
        Get filesystem information including total, used, free space and usage percentage.
        :return: A tuple containing total space, used space, free space, and usage percentage.
        """
        command = f"""
            import os
            import json
            def get_fs_info(path='/'):
                stats = os.statvfs(path)
                block_size = stats[0]
                total_blocks = stats[2]
                free_blocks = stats[3]

                total = block_size * total_blocks
                free = block_size * free_blocks
                used = total - free
                usage_pct = round(used / total * 100, 2)
                
                return total, used, free, usage_pct
            print(get_fs_info())
        """
        out = self.exec(command)
        return ast.literal_eval(out.decode("utf-8"))

#--------------------------------------------------------------

def load_env_from_upyboard():
    """
    Load environment variables from the .upyboard file in the .vscode directory.
    This function searches for the .upyboard file in the current directory and its parent directories,
    and loads the key-value pairs into the environment variables.
    """
    current_path = os.getcwd()

    while True:
        upyboard_path = os.path.join(current_path, ".vscode", ".upyboard")
        if os.path.isfile(upyboard_path):
            with open(upyboard_path) as f:
                for line in f:
                    if '=' in line and not line.strip().startswith('#'):
                        key, val = line.strip().split('=', 1)
                        os.environ[key.strip()] = val.strip()
            break
        
        parent_path = os.path.dirname(current_path)
        if parent_path == current_path:
            break
        current_path = parent_path


class ColorfulGroup(click.Group):
    """
    Custom Click group that formats commands with colors and styles.
    """
    def format_commands(self, ctx, formatter):
        """
        Format the commands in the group with colors and styles.
        :param ctx: Click context object.
        :param formatter: Formatter object to format the output.
        """
        commands = self.list_commands(ctx)
        rows = []
        for cmd_name in commands:
            cmd = self.get_command(ctx, cmd_name)
            if cmd is None or cmd.hidden:
                continue
            help_text = cmd.get_short_help_str()
            cmd_display = click.style(cmd_name, fg='green', bold=True)
            rows.append((cmd_display, help_text))
        if rows:
            with formatter.section('Commands'):
                formatter.write_dl(rows)


SUPPORT_CORE_DEVICE_TYPES = {  #  core, platform, device
    'EFR32MG':{'zigbee':'xnode'}, 
    'ESP32':{'lopy4':'smartfarm1'}, 
    'ESP32S3':{},
    'ESP32C6':{},
    'RP2350':{
        'ticle':'ticle', 
        'xconvey':'xconvey', 
        'xhome':'xhome', 
        'autocon':'autocon'
        }, 
    }

def get_micropython_board_info(port:str, is_long:bool=False) -> Optional[Tuple[str, str, str, str]]:
    """
    Get the firmware version, build date, core name, and device name of the connected device.
    :param port: The port name of the connected device.
    :param is_long: If True, display detailed information about the connected device.
    :return: The (version, date, core, device) or string of the connected device.
    """
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            ser.write(b'\r\x03') 
            time.sleep(0.1)
            ser.reset_input_buffer()

            ser.write(b'\r\x02')
            time.sleep(0.1)

            response = ser.read_all().decode(errors='ignore').strip()
            if response:
                m = re.search(r"(?:MicroPython|Pycom MicroPython)\s+(.*)", response)
                if m:
                    response = m.group(1)

                if is_long:
                    return response

                rx = re.compile(
                    r"(?P<full_version>[^\s\[,]+),?"
                    r"(?:\s*\[[^\]]+\])?"
                    r"\s+on\s+(?P<date>\d{4}-\d{2}-\d{2});\s+"
                    r"(?P<manufacturer>.+?)\s+with\s+(?P<core>\S+)",
                    re.I,
                )
                m = rx.search(response)
                if not m:
                    return None

                full_version = m.group("full_version").lstrip("v").rstrip(",")
                date         = m.group("date")
                manufacturer = m.group("manufacturer").strip().lower().split()[-1]
                core         = m.group("core").strip().upper() 
                
                if len(manufacturer.split()) > 1:
                    manufacturer = manufacturer.split()[-1]
                    
                if manufacturer.startswith('pico2'):
                    manufacturer = manufacturer[:-1]
                
                num_match = re.match(r"(\d+\.\d+)", full_version)
                pico_match = re.match(r"pico2_w_(\d{4})_(\d{2})_(\d{2})", full_version, re.I)  # only pimoroni -> pico2_w_2024_01_01

                if num_match:
                    version = num_match.group(1)
                elif pico_match:
                    y, mth, d = pico_match.groups()

                    if int(y) >= 2025:
                        version = 1.25
                    else:
                        version = 1.24
                else:
                    version = "?"
                    
                device_list = SUPPORT_CORE_DEVICE_TYPES.get(core, None)
                if device_list:
                    device = device_list.get(manufacturer, core)
                else:
                    device = core

                return version, date, core, device
    except (OSError, serial.SerialException):
        pass
    
    return None


#--------------------------------------------------------------------------------------

import upyboard

def __tiny_command(cmd:str) -> None:
    """
    Execute a command on the connected device, wrapping it if necessary.
    :param cmd: The command to execute.
    """
    import ast

    try:
        tree = ast.parse(cmd, mode="exec")
        is_expr = (
            len(tree.body) == 1 and
            isinstance(tree.body[0], ast.Expr)
        )
    except SyntaxError:
        is_expr = False

    if is_expr:
        wrapped = (
            f"__r={cmd}\n"
            "if __r is not None:\n"
            "    print(repr(__r))\n"
        )
    else:
        wrapped = cmd if cmd.endswith("\n") else cmd + "\n"

    out = _upy.exec(wrapped)
    print(out.decode("utf-8", "replace"), end="", flush=True)

_upy = None
_version = 0.0
_core = ""
_device = ""
_device_root_fs = "/"
_core_path = ""
_device_path = ""
_sport = ""

@click.group(cls=ColorfulGroup, invoke_without_command=True)
@click.option(
    "--sport",
    "-s",
    envvar="SERIAL_PORT",
    default="",
    type=click.STRING,
    help="The serial port name for connected device.",
    metavar="SPORT",
)
@click.option(
    "--baud",
    '-b',
    envvar="SERIAL_BAUD",
    default=115200,
    type=click.INT,
    help="Baud rate set only when the connected device uses UART. (Default 115200)",
    metavar="BAUD",
)
@click.option(
    "--command",
    '-c',
    default="",
    type=click.STRING,
    help=" Command to execute on the connected device.",
)   
@click.version_option(__version__, "-v", "--version", message="upyboard %(version)s")
@click.pass_context
def cli(ctx, sport, baud, command):
    """
    UpyBoard CLI - A command line interface for managing MicroPython devices.
    
    This tool allows you to interact with MicroPython devices connected via serial port.
    It supports various commands to manage files, execute scripts, and interact with the device's filesystem.
    """
    global _upy, _version, _core, _device, _device_root_fs, _core_path, _device_path, _sport

    _sport = sport
    
      
    if ctx.invoked_subcommand in ("scan", "sport"):
            return

    descript =  get_micropython_board_info(_sport)
    if not descript:
        if _sport:
            print(f"There is no device connected to the {ANSIEC.FG.BRIGHT_RED}{_sport}{ANSIEC.OP.RESET}.")
        else:
            print(f"The serial port name is missing.")
        raise click.Abort()

    _version, _, _core, _device = descript
    _version = float(_version)
    
    if not _core in (SUPPORT_CORE_DEVICE_TYPES.keys()):
        print(f"The {ANSIEC.FG.BRIGHT_RED}{_device}{ANSIEC.OP.RESET} is not supported.")
        raise click.Abort()
    
    if _device in ('xnode', 'smartfarm1'):
        _device_root_fs = "/flash/"
           
    _core_path = os.path.join(os.path.dirname(upyboard.__file__), os.path.join("core", _core))
    if _core != _device:
        _device_path = os.path.join(os.path.dirname(upyboard.__file__), os.path.join("device", _device))  
        
    try:
        _upy = UpyBoard(_sport, baud)
    except UpyBoardError:
        print(f"Device is not connected to {ANSIEC.FG.BRIGHT_RED}{_sport}{ANSIEC.OP.RESET}")
        print(f"Please check the port with the scan command and try again.")
        raise click.Abort()

    if ctx.invoked_subcommand is None and command:
       __tiny_command(command)
       _upy.close()



@cli.command()
@click.argument("remote")
@click.argument("local", required=False)
def get(remote, local):
    """
    Download a file from the connected device to the local machine.
    :param remote: The remote file to download.
    :param local: The local file or directory to save the downloaded content. If not provided, print to stdout.
    """
    if not remote.startswith(_device_root_fs):
        remote = posixpath.join(_device_root_fs, remote)
    
    try:
        _upy.fs_get(remote, local)
    except UpyBoardError:
        remote = remote.replace(_device_root_fs, "", 1)
        print(f"The {ANSIEC.FG.BRIGHT_RED}{remote}{ANSIEC.OP.RESET} does not exist or is not a file.")

@cli.command()
@click.argument("remote")
def mkdir(remote):
    """
    Create a directory on the connected device.
    :param remote: The remote directory to create.
    """
    path_ = remote
    if not path_.startswith(_device_root_fs):
        path_ = _device_root_fs + remote
        
    if _upy.fs_mkdir(path_):
        print(f"{ANSIEC.FG.BRIGHT_BLUE}{remote}{ANSIEC.OP.RESET} is {ANSIEC.FG.BRIGHT_GREEN}created.{ANSIEC.OP.RESET}")
    else:
        print(f"{ANSIEC.FG.BRIGHT_BLUE}{remote}{ANSIEC.OP.RESET} is {ANSIEC.FG.BRIGHT_RED}already exists.{ANSIEC.OP.RESET}")

@cli.command()
@click.argument("remote")
def rm(remote):
    """
    Remove a file or directory from the connected device.
    :param remote: The remote file or directory to remove.
    """
    if not remote.startswith(_device_root_fs):
        remote = _device_root_fs + remote
        
    try:
        if _upy.fs_is_dir(remote):
            _upy.fs_rmdir(remote)
        else:
            _upy.fs_rm(remote)
    except UpyBoardError:
        remote = remote.replace(_device_root_fs, "", 1)
        print(f"The {ANSIEC.FG.BRIGHT_RED}{remote}{ANSIEC.OP.RESET} does not exist.")

def __get_icon_for_file(name, is_dir):
    """
    Return icon based on directory status or file extension.
    :param name: The name to get icon for.
    :param is_dir: True if the item is a directory.
    :return: Icon string for the file/directory.
    """
    if is_dir:
        return "📁"

    ext_icons = {
        ".py":   "🐍",
        ".mpy":  "📦",
        ".txt":  "📜",
        ".csv":  "📊",
        ".json": "🗄️",
    }
    _, ext = os.path.splitext(name.lower())
    return ext_icons.get(ext, "📄")

@cli.command()
@click.argument("path", default="/")
def ls(path):
    """
    List the files and directories in the specified path on the connected device,
    sorted and including file sizes and icons.
    :param path: The path to list. Defaults to the root directory ("/").
    """
    if not path.startswith(_device_root_fs):
        path = _device_root_fs + path

    try:
        items = _upy.fs_ls_detailed(path)
        
        if not items:
            print(f"{ANSIEC.FG.BRIGHT_RED}{path[1:]}{ANSIEC.OP.RESET} does not exist or is empty.")
            return
        
        display_items = []
        for name, size, is_dir in items:
            icon = __get_icon_for_file(name, is_dir)
            display_items.append((is_dir, name, size, icon))

        if display_items:
            size_width = max(len(str(item[2])) for item in display_items)

            for is_dir, f_name, size, icon in display_items:
                name_str = ANSIEC.FG.BRIGHT_BLUE + f_name + ANSIEC.OP.RESET if is_dir else f_name
                size_str = "" if is_dir else str(size)
                print(f"{size_str.rjust(size_width)}  {icon}  {name_str}")

    except UpyBoardError:
        print(f"{ANSIEC.FG.BRIGHT_RED}{path[1:]}{ANSIEC.OP.RESET} does not exist.")

@cli.command()
@click.argument("local", type=click.Path(exists=True))
@click.argument("remote", required=False)
def put(local, remote):
    """
    Upload a local file or directory to the connected device.
    :param local: The local file or directory to upload.
    :param remote: The remote path on the device. If not provided, the local file name will be used.
    """
    if remote is None:
        remote = os.path.basename(os.path.abspath(local))
    else:
        if not remote.startswith(_device_root_fs):
            remote = posixpath.join(_device_root_fs, remote)
        
        try:
            if _upy.fs_is_dir(remote):
                remote = remote + "/" + os.path.basename(os.path.abspath(local))
        except UpyBoardError:
            pass
        
    if os.path.isdir(local):
        _upy.fs_putdir(local, remote)
    else:
        _upy.fs_put(local, remote)

def __run_error_process(out, local_file):
    """
    Process the error output from the device and print it in a readable format.
    """
    print(f"{ANSIEC.OP.left(len(_error_header))}{ANSIEC.FG.BLACK}{'-'*40}Traceback{'-'*40}{ANSIEC.OP.RESET}")
    for l in out[1:-2]:
        if "<stdin>" in l:
            full_path = os.path.abspath(os.path.join(os.getcwd(), local_file))
            l = l.replace("<stdin>", full_path, 1)
        print(l.strip())
        
    try:
        err_line_raw = out[-2].strip()
        
        if "<stdin>" in err_line_raw:
            full_path = os.path.abspath(os.path.join(os.getcwd(), local_file))
            err_line = err_line_raw.replace("<stdin>", full_path, 1)
        else:
            match = re.search(r'File "([^"]+)"', err_line_raw)
            if match:
                device_src_path = os.path.join(_device_path, "src")
                full_path =  os.path.join(device_src_path, match.group(1))
                escaped_filename = re.sub(r"([\\\\])", r"\\\1", full_path)
                err_line = re.sub(r'File "([^"]+)"', rf'File "{escaped_filename}"', err_line_raw)
                
        print(f" {err_line}")
        
        err_content = out[-1].strip()

        match = re.search(r"line (\d+)", err_line)
        if match:
            line = int(match.group(1))
            try:
                with open(full_path, "r") as f:
                    lines = f.readlines()
                    print(f"  {lines[line - 1].rstrip()}")
            except:
                pass    

    except IndexError:
       err_content = out[-1].strip()
    

    print(f"{ANSIEC.FG.BRIGHT_MAGENTA}{err_content}{ANSIEC.OP.RESET}")
    
@cli.command()
@click.argument("local_file")
@click.option(
    "--not-waiting",
    "-n",
    is_flag=True,
    help="Don't wait for output.",
)
@click.option(
    "--input-echo",
    "-i",
    is_flag=True,
    help="Turn on echo for input",
)
def run(local_file, not_waiting, input_echo):
    """
    Run the local file on the connected device.
    :param local_file: The local file to run on the device.
    :param no_waiting: If True, do not wait for input/output stream.
    :param input_echo: If True, turn on echo for input.
    """
    try:
        _upy.run(local_file, not not_waiting, input_echo)
    except IOError:
        click.echo(f"File not found: {ANSIEC.FG.BRIGHT_RED + local_file + ANSIEC.OP.RESET}", err=True)
    except UpyBoardError as ex:
        __run_error_process(str(ex).strip().split('\n'), local_file)

@cli.command()
def repl():
    """
    Enter the REPL (Read-Eval-Print Loop) mode.
    """
    print(f"{ANSIEC.FG.MAGENTA}Entering REPL mode. Press Ctrl + C to exit{ANSIEC.OP.RESET}.")

    _upy.repl()


is_stop_formatting_process = None

def _formatting_process():
    while not is_stop_formatting_process:
        print(f"{ANSIEC.FG.BRIGHT_BLUE}.{ANSIEC.OP.RESET}", end="", flush=True)
        time.sleep(0.1)

@cli.command()
def format():
    """
    Format the file system of the connected device.
    """
    global is_stop_formatting_process

    print(f"Formatting the file system of {ANSIEC.FG.BRIGHT_YELLOW}{_device}{ANSIEC.OP.RESET}")
    
    is_stop_formatting_process = False
    th = threading.Thread(target=_formatting_process, daemon=True)
    th.start()
    ret = _upy.fs_format()
    is_stop_formatting_process = True
    th.join()
        
    if ret:
        print(f"{ANSIEC.OP.left()}{ANSIEC.OP.CLEAR_LINE}The file system has been {ANSIEC.FG.BRIGHT_BLUE}formatted{ANSIEC.OP.RESET}")
    else:
        print(f"{ANSIEC.OP.left()}The {ANSIEC.FG.BRIGHT_RED}{_device}{ANSIEC.OP.RESET} is not supported.")
    return ret

@cli.command()
def df():
    """
    Show the file system information of the connected device.
    """
    ret = _upy.fs_df()
    if ret:
        out_str = f"""Total: {ret[0]//1024:5} KByte ({ret[0]:5})
Used: {ret[1]//1024:6} KByte ({ret[1]:7})
Free: {ret[2]//1024:6} KByte ({ret[2]:6})
Usage: {round(ret[3],2):5} %""" 
        
        print(out_str)

@cli.command()
def shell():
    """
    Enter an interactive shell for device control.
    """
    import shlex

    COMMANDS = "clear, ls, cd, get, put, rm, mkdir, df, repl, pwd, help(?)"
    HELP = f"""Type 'exit' or press CTRL+C to exit.  
Available: {ANSIEC.FG.BRIGHT_BLUE}{COMMANDS}{ANSIEC.OP.RESET}"""
    
            
    current_path = '/' # _device_root_fs
    
    def print_prompt():
        print(f"\n📟 {_device}:{current_path} >", end=" ", flush=True)

    def run_cmd(cmdline):
        nonlocal current_path

        args = shlex.split(cmdline)
        if not args:
            return
        cmd = args[0]

        try:        
            if cmd == "ls":
                if len(args) > 1:
                    print("Usage: ls")
                    return

                click.Context(ls).invoke(ls, path=current_path)

            elif cmd == "cd":
                if len(args) != 2:
                    print("Usage: cd <dir>")
                    return
                
                new_path = posixpath.normpath(posixpath.join(current_path, args[1]))
                try:
                    _upy.fs_is_dir(new_path)
                    current_path = new_path
                except:
                    dirs = ANSIEC.FG.BRIGHT_RED + " ".join(args[1:]) + ANSIEC.OP.RESET
                    print(f"The {dirs} directory does not exist.")

            elif cmd == "get":
                if len(args) < 2 or len(args) > 3:
                    print("Usage: get <remote> [local]")
                    return
                remote = posixpath.join(current_path, args[1])
                local = args[2] if len(args) >= 3 else None
                click.Context(get).invoke(get, remote=remote, local=local)

            elif cmd == "put":
                if len(args) < 2 or len(args) > 3:
                    print("Usage: put <local> [remote]")
                    return
                
                local = args[1]
                remote = args[2] if len(args) >= 3 else None
                if remote is None:
                    remote = os.path.basename(local)
                remote = posixpath.join(current_path, remote)
                click.Context(put).invoke(put, local=local, remote=remote)

            elif cmd == "rm":
                if len(args) != 2:
                    print("Usage: rm <remote>")
                    return
                remote = posixpath.join(current_path, args[1])
                click.Context(rm).invoke(rm, remote=remote)

            elif cmd == "mkdir":
                if len(args) != 2:
                    print("Usage: mkdir <remote>")
                    return
                remote = posixpath.join(current_path, args[1])
                click.Context(mkdir).invoke(mkdir, remote=remote)

            elif cmd == "df":
                if len(args) > 1:
                    print("Usage: df")
                    return

                click.Context(df).invoke(df)

            elif cmd == "-c":
                if len(args) < 2:
                    print("Usage: -c <scripts>")
                    return
                
                scripts = cmdline[3:]
                __tiny_command(scripts)

            elif cmd == "repl":
                print("Press CTRL+C to exit.")
                if len(args) > 1:
                    print("Usage: repl")
                    return
                
                _upy.repl()

            elif cmd == "pwd":
                if len(args) > 1:
                    print("Usage: pwd")
                    return

                print(current_path)

            elif cmd == "clear":
                if len(args) > 1:
                    print("Usage: clear")
                    return
                print(ANSIEC.OP.CLEAR + ANSIEC.OP.RESET)

            elif cmd == "help" or cmd == "?":
                if len(args) > 1:
                    print("Usage: help or ?")
                    return
                
                print(HELP)
                
            else:
                raise Exception(f"Unknown command: {cmd}")
        except UpyBoardError:
            raise Exception(f"Unknown command: {cmdline}")
        
    print(f"Connected to {_device} on {ANSIEC.FG.BRIGHT_GREEN}{_core}{ANSIEC.OP.RESET}")
    print(HELP)
                
    try:
        while True:
            print_prompt()
            line = sys.stdin.buffer.readline().decode(errors='replace').rstrip()
            if line == 'exit':
                break
            try:
                run_cmd(line)
            except Exception as e:
                print(f"{e}")
                continue
            
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        print("Exiting shell.")
    
def __force_remove_readonly(func, path, exc_info):
    """
    Force remove a read-only file or directory.
    """
    try:
        os.chmod(path, stat.S_IWRITE)
        func(path)
    except Exception as e:
        print(f"Deletion failed: {path}, error: {e}")

@cli.command()
def reset():
    """
    Reset the connected device.
    """
    _upy.reset()
    
@cli.command()
@click.argument("device", required=False)
def env(device=None): 
    """
    Create a .vscode folder with the upyboard environment.
    """
    global _device, _device_path
        
    vscode_dir = ".vscode" 

    upyboard_env_file = os.path.join(vscode_dir, ".upyboard")
    task_file = os.path.join(vscode_dir, "tasks.json") 
    settings_file = os.path.join(vscode_dir, "settings.json") 
    launch_file = os.path.join(vscode_dir, "launch.json")
    
    task_file_contents = """{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Run micropython with upyboard",
            "type": "shell",
            "command": "upy",
            "args": [
                "${file}"
            ],
            "problemMatcher": [],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        }
    ]
}
"""

    settings_file_contents = """{
    "files.exclude": {
      "**/.vscode": true,
    },
    "python.languageServer": "Pylance",
    "python.analysis.diagnosticSeverityOverrides": {
        "reportMissingModuleSource": "none",
    },
    "python.analysis.extraPaths": [
        "./.vscode"
    ]
}
"""

    launch_file_contents = """{
    "version": "0.2.0",
    "configurations": [
      {
        "name": "Python: Current file debug",
        "type": "debugpy",
        "request": "launch",
        "program": "${file}",
        "console": "integratedTerminal"
      }
    ]
  }
"""
    if device:  
        leaf_values = [
            device_name
            for subdict in SUPPORT_CORE_DEVICE_TYPES.values()
            for device_name in subdict.values()
        ]
        if device in leaf_values:
            _device = device
            _device_path = os.path.join(os.path.dirname(upyboard.__file__), os.path.join("device", _device))
        else:
            print(f"The {ANSIEC.FG.BRIGHT_RED}{device}{ANSIEC.OP.RESET} is an unsupported device.")
            raise click.Abort()
            
    if os.path.exists(vscode_dir):    
        print(f"There is already a set environment. Do you want to set it again? (y or n): ", end='', flush=True)
        while True:
            char = getch().lower()
            if char == b'n':
                print("\nCanceling the operation")
                return
            elif char == b'y':
                break
            else:
                print(f"{ANSIEC.FG.BRIGHT_RED}Please enter 'y' or 'n'.{ANSIEC.OP.RESET}", end='', flush=True)

        shutil.rmtree(vscode_dir, onerror=__force_remove_readonly)

    core_typehints = os.path.join(_core_path, "typehints")
    shutil.copytree(core_typehints, vscode_dir) 
    
    if _device_path:
        vscode_device_dir = os.path.join(vscode_dir, _device)  
        device_typehints = os.path.join(_device_path, "typehints")  
        shutil.copytree(device_typehints, vscode_device_dir) 

    with open(upyboard_env_file, "w", encoding="utf-8") as f:
        f.write(f"SERIAL_PORT={_sport.upper()}\n")

    with open(task_file, "w", encoding="utf-8") as f:
        f.write(task_file_contents)  

    with open(settings_file, "w", encoding="utf-8") as f:
        f.write(settings_file_contents)

    with open(launch_file, "w", encoding="utf-8") as f:
        f.write(launch_file_contents)

    print(f"Serial port {ANSIEC.FG.BRIGHT_GREEN}{_sport}{ANSIEC.OP.RESET} is registered.") 

import mpy_cross

@cli.command()
@click.argument("local", type=click.Path(exists=True))
@click.argument("remote", required=False)
def upload(local, remote):
    """
    It compiles all source code from the local path into .mpy files and uploads them to the remote path.
    :param local: The local source code directory.
    :param remote: The remote path on the device. If not provided, root directory will be used.
    """
    def _mpy_output_path(base, filepath):
        relative_path = os.path.relpath(filepath, base)
        output_dir = os.path.join(os.path.dirname(base), "mpy", os.path.dirname(relative_path))
        os.makedirs(output_dir, exist_ok=True)
        
        filename = os.path.splitext(os.path.basename(filepath))[0] + ".mpy"
        return os.path.join(output_dir, filename)

    def _conv_py_to_mpy(local, base):
        args = ['_filepath_', '-o', '_outpath_', '-msmall-int-bits=31']
        if _core == "EFR32MG":
            if _version < 1.19:
                args.append('-mno-unicode')
        elif _core == "ESP32":
            args.append('-march=xtensa')
        elif _core == "ESP32S3":
            args.append('-march=xtensawin')
        elif _core == "RP2350":
            args.append('-march=armv7emsp')
        else:
            raise ValueError(f"The {ANSIEC.FG.BRIGHT_RED}{_core}{ANSIEC.OP.RESET} is not supported")
        
        if os.path.isfile(local):
            args[0] = local
            args[2] = os.path.splitext(local)[0] + ".mpy"    
            mpy_cross.run(*args)
        else:      
            for filename in os.listdir(local):
                filepath = os.path.join(local, filename)

                if os.path.isdir(filepath):
                    _conv_py_to_mpy(filepath, base)
                    continue

                if not filepath.endswith(".py"):
                    continue
    
                args[0] = filepath
                args[2] = _mpy_output_path(base, filepath)
                mpy_cross.run(*args)


    remote = _device_root_fs + (remote or "")
        
    _conv_py_to_mpy(local, base=local)
            
    if os.path.isfile(local):
        local = os.path.splitext(local)[0] + ".mpy"
        click.Context(put).invoke(put, local=local, remote=remote)
        os.remove(local)
    elif os.path.isdir(local):            
        shutil.rmtree(os.path.join(local, "__pycache__"), ignore_errors=True)

        local_mpy_dir = os.path.join(os.path.dirname(local), "mpy")
        if not os.path.exists(local_mpy_dir):
            return

        _upy.fs_mkdir(remote)

        for item in os.listdir(local_mpy_dir):
            local_item = os.path.join(local_mpy_dir, item)
            remote_item = os.path.join(remote, item).replace("\\", "/")
            click.Context(put).invoke(put, local=local_item, remote=remote_item)

        shutil.rmtree(local_mpy_dir, onerror=__force_remove_readonly)
    else:
        print("The local path is not a file or directory.")


def find_core_by_device(device: str) -> str | None:
    for core, platform_map in SUPPORT_CORE_DEVICE_TYPES.items():
        if device in platform_map.values():
            return core
    return None

@cli.command()
@click.argument("target", required=False, nargs=-1)
def init(target=None):
    """
    Initialize the file system of the device.
    """
    global _core, _core_path, _device, _device_path

    base_dir = os.path.dirname(upyboard.__file__)
    
    def _normalize_target(parts):
        if not parts:
            return None
        return "/".join(p.strip("/") for p in parts if p) or None

    def _parse_target(t):
        if not t:
            return (None, "both", None)

        parts = t.strip("/").split("/")
        if len(parts) == 1:
            if parts[0] in ("core", "device"):
                return (None, parts[0], None)
            return (parts[0], "both", None)  # upy init ticle

        first, second = parts[0], parts[1]

        if first in ("core", "device"):
            sub = "/".join(parts[1:]) or None
            return (None, first, sub)

        if second in ("core", "device"):
            sub = "/".join(parts[2:]) or None
            return (first, second, sub)

        sub = "/".join(parts[1:]) or None
        return (first, "auto", sub)


    def _resolve_board(board_name):
        global _core, _core_path, _device, _device_path
        if board_name is None:
            if _core and not _core_path:
                _core_path = os.path.join(base_dir, "core", _core)
            if _device and not _device_path:
                _device_path = os.path.join(base_dir, "device", _device)
            return

        core = find_core_by_device(board_name)
        if core is None:
            print(f"The {ANSIEC.FG.BRIGHT_RED}{board_name}{ANSIEC.OP.RESET} is an unsupported device.")
            raise click.Abort()

        _core = core
        _device = board_name
        _core_path = os.path.join(base_dir, "core", _core)
        _device_path = os.path.join(base_dir, "device", _device)

    def _ensure_dirs_for_scope(scope):
        _upy.fs_mkdir(_device_root_fs + "lib/")
        if scope in ("device", "both"):
            _upy.fs_mkdir(_device_root_fs + f"lib/{_device}/")

    def _upload_core(sub=None):
        src_base = os.path.join(_core_path, "src")
        src = os.path.join(src_base, sub) if sub else src_base

        if not os.path.exists(src):
            print(f"The {ANSIEC.FG.BRIGHT_RED}{sub or 'core'}{ANSIEC.OP.RESET} is not found in core/src.")
            raise click.Abort()

        print(upload_format_str.format("Core", _device))

        if os.path.isdir(src):
            rel = (sub.rstrip("/") + "/") if sub else ""
            remote_dir = "lib/" + rel
            _ensure_remote_dir(remote_dir)
            click.Context(upload).invoke(upload, local=src, remote=remote_dir)
        else:
            rel = os.path.relpath(src, src_base).replace("\\", "/")
            rel_dir = os.path.dirname(rel)
            base_name_mpy = os.path.splitext(os.path.basename(rel))[0] + ".mpy"
            remote_dir = "lib/" + (rel_dir + "/" if rel_dir else "")
            remote_file = remote_dir + base_name_mpy
            _ensure_remote_dir(remote_dir)
            click.Context(upload).invoke(upload, local=src, remote=remote_file)

    def _ensure_remote_dir(remote_dir):
        if not remote_dir:
            return
        parts = [p for p in remote_dir.replace("\\", "/").strip("/").split("/") if p]
        path = _device_root_fs
        for p in parts:
            path = path + p + "/"
            _upy.fs_mkdir(path)
            
    def _upload_device(sub=None):
        src_base = os.path.join(_device_path, "src")
        src = os.path.join(src_base, sub) if sub else src_base

        if not os.path.exists(src):
            print(f"The {ANSIEC.FG.BRIGHT_RED}{sub or 'device'}{ANSIEC.OP.RESET} is not found in device/src.")
            raise click.Abort()

        print(upload_format_str.format("Device", _device))

        if os.path.isdir(src):
            rel = (sub.rstrip("/") + "/") if sub else ""
            remote_dir = f"lib/{_device}/" + rel
            _ensure_remote_dir(remote_dir)
            click.Context(upload).invoke(upload, local=src, remote=remote_dir)
        else:
            rel = os.path.relpath(src, src_base).replace("\\", "/")
            rel_dir = os.path.dirname(rel)
            base_name_mpy = os.path.splitext(os.path.basename(rel))[0] + ".mpy"
            remote_dir = f"lib/{_device}/" + (rel_dir + "/" if rel_dir else "")
            remote_file = remote_dir + base_name_mpy
            _ensure_remote_dir(remote_dir)
            click.Context(upload).invoke(upload, local=src, remote=remote_file)

    raw_target = _normalize_target(target)
    board, scope, subpath = _parse_target(raw_target)

    if _version < 1.12:
        print("Unkown micropython version " + ANSIEC.FG.BRIGHT_RED + f"{_version}" + ANSIEC.OP.RESET)
        raise click.Abort()

    _resolve_board(board)

    if scope == "auto":
        dev_path = os.path.join(_device_path, "src", subpath)
        core_path = os.path.join(_core_path, "src", subpath)
        if os.path.exists(dev_path):
            scope = "device"
        elif os.path.exists(core_path):
            scope = "core"
        else:
            print(f"The {ANSIEC.FG.BRIGHT_RED}{subpath}{ANSIEC.OP.RESET} is not found in "
                f"device/src or core/src for board '{_device}'.")
            raise click.Abort()
        
    upload_format_str = (
        "Uploading the "
        + ANSIEC.FG.BRIGHT_YELLOW + "{0}" + ANSIEC.OP.RESET
        + " library on the "
        + ANSIEC.FG.BRIGHT_YELLOW + "{1}" + ANSIEC.OP.RESET
    )

    do_format = (scope == "both" and subpath is None)
    if do_format:
        if not click.Context(format).invoke(format):
            print("Unable to format the file system. Please check the "
                  + ANSIEC.FG.BRIGHT_RED + f"{_device}" + ANSIEC.OP.RESET)
            return
        _ensure_dirs_for_scope("both")
    else:
        _ensure_dirs_for_scope(scope)

    if scope == "both":
        _upload_core(None)
        if _device_path and os.path.exists(os.path.join(_device_path, "src")):
            _upload_device(None)
    elif scope == "core":
        _upload_core(subpath)
    elif scope == "device":
        _upload_device(subpath)
    else:
        print(f"The {ANSIEC.FG.BRIGHT_RED}{scope}{ANSIEC.OP.RESET} is not a valid scope.")
        raise click.Abort()

    print("The job is done!")



def __is_bluetooth_port(port_info):
    """
    Check if the given port_info is a Bluetooth port.
    :param port_info: The port information to check.
    :return: True if the port is a Bluetooth port, False otherwise.
    """
    bt_keywords = ['bluetooth', 'bth', 'devb', 'rfcomm', 'Blue', 'BT']
    description = port_info.description.lower()
    device = port_info.device.lower()
    return any(keyword in description or keyword in device for keyword in bt_keywords)

@cli.command()
@click.option(
    "--raw",
    "-r",
    is_flag=True,
    default=False,
    help="Enable raw REPL mode for scanning",
)
def scan(raw:bool):
    """
    Display the list of connected boards.
    :param raw: If True, display detailed information about the connected device. otherwise, display the version and device name.
    """

    color_tbl = (ANSIEC.FG.BRIGHT_YELLOW, ANSIEC.FG.BRIGHT_GREEN, ANSIEC.FG.BRIGHT_BLUE)
    color_pos = 0    
    
    for port in list_ports.comports():
        if __is_bluetooth_port(port):
            continue
        descript = get_micropython_board_info(port.device, raw)
            
        if descript:
            color_pos = (color_pos + 1) % len(color_tbl)
            if not raw:
                version, date, core, device = descript
                                                 
                print(color_tbl[color_pos] + f"{port.device:>6}" + ANSIEC.OP.RESET + f"\t{version:>4} {date:>11}" + color_tbl[color_pos] + f"  {device}" + ANSIEC.OP.RESET)
            else:
                print(color_tbl[color_pos] + f"{port.device:>6}" + ANSIEC.OP.RESET + f"\t{descript}")


def __is_valid_serial_port(port_name:str):
    """
    Check if the port_name is valid on the specified or current platform.
    :param port_name: The serial port string to validate.
    :return: True if valid, False otherwise.
    """
    platform = sys.platform

    if platform.startswith("win"):
        return re.fullmatch(r"COM[1-9][0-9]*", port_name, re.IGNORECASE) is not None
    elif platform.startswith("linux"):
        return re.fullmatch(r"/dev/tty(USB|S|ACM)[0-9]+", port_name) is not None
    elif platform == "darwin":
        return re.fullmatch(r"/dev/tty\..+", port_name) is not None
    else:
        return False

@cli.command()
@click.argument("port", required=False)
def sport(port:str=None):
    """
    Set or display the serial port for the connected device.
    If no port is specified, it will read the port from the .vscode/.upyboard file.
    :param port: The serial port to set. If None, read from the .vscode/.upyboard file.
    """
    if port is None:
        if os.path.exists(os.path.join(".vscode", ".upyboard")):
            with open(os.path.join(".vscode", ".upyboard"), "r") as f:
                content = f.read()
                match = re.search(r"SERIAL_PORT=(.*)", content)
                if match:
                    port = match.group(1).strip()
                    print(f"Current serial port: {ANSIEC.FG.BRIGHT_GREEN}{port}{ANSIEC.OP.RESET}")
                else:
                    print("No serial port found.")
        else:
            print("No serial port is configured.")
    else:
        if not __is_valid_serial_port(port):
            print(f"Invalid serial port: {ANSIEC.FG.BRIGHT_RED}{port}{ANSIEC.OP.RESET}")
            return
        if not get_micropython_board_info(port):
            print(f"Device is not connected to {ANSIEC.FG.BRIGHT_RED}{port}{ANSIEC.OP.RESET}")
            return
        
        if os.path.exists(os.path.join(".vscode", ".upyboard")):
            with open(os.path.join(".vscode", ".upyboard"), "w") as f:
                f.write(f"SERIAL_PORT={port.upper()}\n")
            print(f"Serial port set to: {ANSIEC.FG.BRIGHT_GREEN}{port.upper()}{ANSIEC.OP.RESET}")
        else:
            print("Requires configuration.")

#--------------------------------------------------------------

UPDATE_INTERVAL = 60 * 60 * 24  
UPDATE_TIMESTAMP_FILE = pathlib.Path.home() / ".upyboard_update_check"


def __should_check_for_updates() -> bool:
    """
    Check if the update check should be performed based on the last check time.
    :return: True if the update check should be performed, False otherwise.
    """
    
    if UPDATE_TIMESTAMP_FILE.exists():
        last_check = UPDATE_TIMESTAMP_FILE.stat().st_mtime
        if time.time() - last_check < UPDATE_INTERVAL:
            return False
    return True

def check_for_updates(current_version):
    """
    Check PyPI for a newer version of upyboard and prompt the user to upgrade.
    """
    if not __should_check_for_updates():
        return

    try:
        with urllib.request.urlopen("https://pypi.org/pypi/upyboard/json") as resp:
            data = json.loads(resp.read().decode("utf-8"))
        latest_version = data["info"]["version"]

        if tuple(map(int, latest_version.split('.'))) > tuple(map(int, current_version.split('.'))):
            print(f"A newer version ({latest_version}) is available. Update now? (y/n): ", end='', flush=True)
            if getch().decode().lower() == 'y':
                subprocess.run([
                    sys.executable, "-m", "pip", "install",
                    "--upgrade", "upyboard",
                    "--upgrade-strategy", "eager"
                ])
                sys.exit(0)
    except Exception as e:
        pass
    
    try:
        UPDATE_TIMESTAMP_FILE.touch()
    except Exception:
        pass


def main():
    check_for_updates(__version__)

    if len(sys.argv) == 1:
        print(f"Use {ANSIEC.FG.BRIGHT_BLUE}upy --help{ANSIEC.OP.RESET} to see available commands.")
        raise SystemExit()

    args = sys.argv[1:]

    try:
        known = set(cli.commands.keys())
    except Exception:
        known = {'init', 'upload', 'put', 'get', 'rm', 'run', 'format'}

    first_nonopt_idx = next((i for i, a in enumerate(sys.argv[1:], 1) if not a.startswith('-')), None)
    first_nonopt = sys.argv[first_nonopt_idx] if first_nonopt_idx is not None else None

    py_arg_idx = next((i for i, a in enumerate(sys.argv[1:], 1) if a.endswith('.py')), None)

    run_opts = {'-i', '--input-echo', '-n', '--not-waiting'}

    should_inject_run = (
        ('run' not in args) and
        (py_arg_idx is not None) and
        (first_nonopt is None or first_nonopt not in known)
    )

    if should_inject_run:
        opt_idx = next((i for i, a in enumerate(sys.argv[1:], 1) if a in run_opts), None)
        insert_at = opt_idx if opt_idx is not None else py_arg_idx
        sys.argv.insert(insert_at, 'run')

    load_env_from_upyboard()
    exit_code = cli()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()