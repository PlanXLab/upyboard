# UpyBoard User Guide 

> **TL;DR**  
> `upy` is a CLI that lets you **upload, run and manage MicroPython boards over serial**—from rapid development to one-command deployment. 🍀

---

## Table of Contents

1. [Installation & Requirements](#installation--requirements)  
2. [Quick-start](#quick-start)  
3. [Core Concepts & Board Detection](#core-concepts--board-detection)  
4. [Global Options & Command Overview](#global-options--command-overview)  
5. [File Operations](#file-operations) – `ls / get / put / mkdir / rm / df`  
6. [Running Code](#running-code) – `run / -c`  
7. [Using the REPL](#using-the-repl-repl) – `repl`  
8. [Initialising & Deploying Libraries](#initialising--deploying-libraries) – `init / format / upload`  
9. [Port Management & Board Scan](#port-management--board-scan) – `scan / sport`  
10. [VS Code Integration](#vs-code-integration) – `env`  
11. [Interactive Mini Shell](#interactive-mini-shell) – `shell`  
12. [Automatic Update Check](#automatic-update-check)  
13. [Serial-port Naming Rules per OS](#serial-port-naming-rules-per-os)  
14. [Troubleshooting](#troubleshooting)  
15. [Appendix](#appendix)  

---

## Installation & Requirements

### Install

```bash
pip install upyboard
```

upy will be added to your PATH.

### Prerequisites
Item  |	Why it’s needed
------|-----------------
Python 3.8 up	| Host interpreter
pyserial	| Serial communication
click	| Command-line framework
mpy_cross¹	| Compiles .py → .mpy

¹ Automatically invoked by the upload command.

## Quick-start
```sh
# 1 · Find attached boards
upy scan

# 2 · Set the port (example: Windows)
upy sport COM7
#   Linux: /dev/ttyACM0    macOS: /dev/tty.usbmodemXYZ

# 3 · Upload a file
upy put main.py /main.py

# 4 · Run a local script on the board (stream output)
upy run examples/blink.py

# 5 · Enter interactive REPL
upy repl
```
⚡ Shortcut: upy myfile.py is treated as upy run myfile.py.

## Core Concepts & Board Detection
- UpyBoard talks to the board via Raw REPL.

- On connection it parses the boot banner to read version / build date / MCU core.

- A lookup table maps the core + manufacturer to a device name (e.g. pico2w1 → xconvey).

- Most boards mount root as /. Some devices (xnode, smartfarm1) use /flash/; UpyBoard adapts automatically.

## Global Options & Command Overview
```text
upy [GLOBAL OPTIONS] <COMMAND> [args]

Global options
  -s, --sport PORT     Serial port (env SERIAL_PORT)
  -b, --baud  BAUD     Baud rate (default 115200)
  -c, --command "..."  One-liner executed on the board
  -v, --version        UpyBoard version
```

Category  |	Commands
----------|--------------
File I/O |	ls, get, put, mkdir, rm, df
Execution |	run, repl, -c
Deployment |	upload, format, init
Port/Detect |	scan, sport
Dev Env	| env
Interactive	| shell
Other |	reset

## File Operations
### ls [PATH]
Lists files and folders with icons & sizes.

```sh
upy ls          # root
upy ls /lib
```

### get <remote> [local]
Download a file. If local is omitted, binary data is sent to stdout.

```sh
upy get /main.py ./backup/
upy get /data.bin > local.bin
Progress bar and byte-count verification included.
```

### put <local> [remote]
Upload a file or directory.

```sh
upy put app.py /app.py
upy put assets /www/assets
Directories are walked recursively. Existing files are replaced safely.
```

### mkdir <remote>
Creates nested directories in one call.

```
upy mkdir /www/static/img
```

### rm <remote>
Removes file or directory (recursive).

```sh
upy rm /old.log
upy rm /www
```

### df
Shows total / used / free bytes & usage %.

```sh
upy df
```

## Running Code
### run <local.py>
Executes a local script on the board.

Options

Flag |	Meaning
-----|---------------
-n, --not-waiting |	Do not stream output (fire-and-forget)
-i, --input-echo |	Echo keyboard input (for interactive scripts)

```sh
upy run demo.py
upy run -n boot_prep.py
upy run -i serial_console.py
```

### -c "single line" (global)
Executes a one-liner. If it’s an expression, the result’s repr() is printed.

```sh
upy -c "import os; os.listdir('/')"
upy -c "1+2"         # prints 3
```

**During streaming**

- Ctrl+C – keyboard interrupt and soft-reset

- Ctrl+D – terminate streaming session

UpyBoard rewrites remote Tracebacks so that file paths and source lines map to your host files for easier debugging.

## Using the REPL (repl)
```sh
upy repl
```

- Ctrl+C – leave REPL

- Ctrl+D – soft-reset board

- Windows arrow / Home / End keys are converted to ANSI sequences.

- Toggle output: Ctrl+G (pause) / Ctrl+O (resume).

## Initialising & Deploying Libraries
### format
Formats the flash file-system (per core type).

```sh
upy format
```

### upload <local> [remote]
Compiles all .py → .mpy (via mpy-cross) and uploads.

```sh
upy upload src/ /lib/
```

Automatically sets -march flags:

- ESP32 → xtensa

- ESP32S3 → xtensawin

- RP2350 → armv7emsp

- EFR32MG (older FW) → -mno-unicode

### init [device]
Recommended one-command “factory reset + library deploy”.

```sh
upy init             # use detected device
upy init xconvey     # explicit
```

1. format

2. Creates /lib/

3. Uploads core libraries from upyboard/core/<CORE>/src

4. Uploads device-specific libs from upyboard/device/<DEVICE>/src

Requires MicroPython ≥ 1.12.

## Port Management & Board Scan
scan [-r]
Lists all attached MicroPython boards.

```sh
upy scan        # concise
upy scan -r     # raw banner
```

Bluetooth virtual ports are skipped.

### sport [PORT]
Reads or writes the serial port stored in .vscode/.upyboard.

```sh
upy sport              # show current
upy sport COM7         # set (Windows example)
```

## VS Code Integration
Generates a ready-to-use .vscode/ folder.

```sh
upy env             # for detected board
upy env ticle       # include device stubs
```

Creates

File |	Purpose
-----|---------------------
.vscode/.upyboard |	SERIAL_PORT=...
tasks.json |	upy ${file} build task
settings.json |	Adds type-hint paths, Pylance tuning
launch.json |	Basic Debugpy launcher
typehints/ |	Core + device stubs (copied)

Existing .vscode is backed up or replaced with confirmation.

## Interactive Mini Shell
```sh
upy shell
```

Supports: clear, ls, cd, get, put, rm, mkdir, df, repl, pwd, -c, help(?)

Example:
```text
📟 xconvey:/ > ls
📟 xconvey:/ > cd lib
📟 xconvey:/lib > put ../main.py
📟 xconvey:/lib > -c print(1+2)
📟 xconvey:/lib > repl
```
Exit with exit or Ctrl+C.

### Automatic Update Check
Once per 24 h UpyBoard queries PyPI; if a newer version exists you’ll be asked:

```text
A newer version (1.5.0) is available. Update now? (y/n):
```

Accepting immediately runs pip install --upgrade upyboard.

Timestamp stored in ~/.upyboard_update_check.

## Serial-port Naming Rules per OS
OS |	Example pattern
---|---------------------
Windows |	COM7, COM12, …
Linux |	/dev/ttyUSB0, /dev/ttyACM0
macOS |	/dev/tty.usbmodemXXXX

upy sport validates the pattern and checks that the board actually responds.

## Troubleshooting
Symptom / Message |	Possible Cause & Fix
------------------|-----------------------
“There is no device connected to PORT” |	Wrong port, cable, power—run upy scan, verify baud. Linux: add user to dialout group.
“could not enter raw repl”	| Board busy or REPL locked—reset board (upy reset) then retry.
Upload stalls or Download incomplete |	Cable/hub issue or flash full → check upy df or run upy format (erases data!).
Invalid serial port	| Port name doesn’t match OS pattern (see table above).
init says not supported	| Core missing format handler—check core/device via upy scan.

## Appendix
- Expression wrapping: -c wraps expressions as

```python
__r = <expr>
if __r is not None:
    print(repr(__r))
```

- UTF-8 safe print: Partial UTF-8 bytes are buffered until complete, preventing garbled output.

- Adaptive chunking: Script upload uses 1 KB→8 KB chunks, doubling on fast links, halving on lag.

- Traceback remap: Remote <stdin> filenames are swapped for your real file paths and the failing source line is printed.

- Environment cascading: .vscode/.upyboard is searched upward from CWD and loaded into os.environ.

## Cheatsheet Recipes
### A · Dev ➜ Test ➜ Deploy

```sh
upy sport /dev/ttyACM0
upy run examples/main.py          # live test
upy upload src/ /lib/             # compile & deploy
upy put boot.py /boot.py
upy reset
```
### B · Factory Reset + Library Install

```sh
upy scan
upy sport COM7
upy init xconvey
```
### C · Fetch Logs & Check Disk

```sh
upy get /logs/today.txt ./logs/
upy df
```