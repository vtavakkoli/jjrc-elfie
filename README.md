# JJRC Elfie — FPV Flight Deck

[![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB)](https://www.python.org/downloads/)
[![Tests](https://github.com/vtavakkoli/jjrc-elfie/actions/workflows/python-checks.yml/badge.svg?branch=master)](https://github.com/vtavakkoli/jjrc-elfie/actions/workflows/python-checks.yml)

Python 3.12 keyboard flight controller for JJRC-compatible drones with a Pygame interface.

> **New 5 November 2026:** `python main.py` now sends the **original Android app's 11-byte UDP packet** to port **8080** every **25 ms**. This packet layout was reconstructed from the supplied decompiled Android `ControlActivity.SendThread.run()`. It has **not yet been tested with a physical drone**. The previous **8-byte H37 mode**, which previously worked for video, motor start, ascent and emergency stop on a test drone, remains available via `--protocol h37`.

## Installation

Connect your computer to the drone's Wi-Fi before running in flight mode.

```powershell
git clone https://github.com/vtavakkoli/jjrc-elfie.git
cd jjrc-elfie
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py --demo
python main.py
```

Windows or Linux with Python 3.12; dependencies include Pygame and PyAV. Use `python main.py --no-video` to test flight control packets without needing a working video stream.

## Default: original Android control protocol

| Setting | Original Android mode (`python main.py`) | Legacy H37 (`python main.py --protocol h37`) |
| --- | --- | --- |
| UDP port | **8080** | 8895 |
| Datagram | **11 bytes: `FF 08 ... checksum`** | 8 bytes: `66 ... 99` |
| Checksum | 255 minus payload byte sum (modulo 256) | XOR |
| Cadence | **40 Hz** | 20 Hz |
| Modes | Original speed mode 0, 1, 2 | Software gain 10–80% |
| Startup | Android sends controls immediately | 5 s idle, 5 s altitude-hold |

Android byte layout: `FF 08 [throttle] [yaw] [pitch] [roll] [yaw trim + alt hold + calibrate] [pitch trim] [roll trim] [speed + headless + stop + takeoff + land] [checksum]`.

The exact format is implemented in [android_protocol.py](android_protocol.py). Original Android rocker coordinates run from 0 to 127; the desktop maps keyboard input to these coordinates. Neutral manual throttle is byte 0; neutral altitude-hold throttle is byte 126. Firmware-specific direction reversal is supported for `UDP_720` firmware `V2.3.9`, `V2.4.0`, and `V2.4.1`.

## Keyboard controls in original Android mode

| Key | Function |
| --- | --- |
| **W / S** | Forward / backward pitch |
| **A / D** | Roll left / right |
| **Up / Down** | Altitude movement in hold mode; change latched throttle in manual mode |
| **Left / Right** | Yaw / rotate |
| **T** | Takeoff pulse (turns on altitude hold automatically) |
| **L** | Landing pulse |
| **X** | Emergency motor-stop pulse (can cause a fall) |
| **H** | Toggle altitude hold when not armed |
| **N** | Headless mode toggle |
| **C** | Calibration pulse while disarmed; aircraft must be stationary |
| **+ / -** | Change Android speed mode 0 / 1 / 2 |
| **Space** | Neutral motion; in manual mode also sets throttle to zero |
| **Esc** | Quit and send a zero-throttle packet (not a guaranteed landing) |

**Important:** The original Android app defaults to manual throttle, which does not spring back to neutral; the desktop emulates that with a throttle latch. Up increases it gradually; releasing Up retains the throttle position. **Up can spin motors in manual mode even without pressing T.** The safest way to evaluate command encoding is with propellers removed. Pressing **T** enables altitude hold and sends the Android takeoff bit for one second, matching the original app's takeoff control bits (while saving the separate ALT button click).

## Test commands

```powershell
# New default: original Android control protocol, UDP 8080
python main.py

# Isolate flight controls from unverified video compatibility
python main.py --no-video

# If the drone Wi-Fi Default Gateway isn't 172.16.10.1:
python main.py --host 192.168.1.1 --no-video

# Start with altitude hold active instead of original manual mode
python main.py --altitude-hold

# If the original firmware reports this version and device type
python main.py --device-type UDP_720 --firmware-version V2.4.0

# Revert to the old working H37 controller
python main.py --protocol h37

# Run tests without hardware
python -m unittest discover -s tests -v
```

The original Android app selects the drone host from the Wi-Fi DHCP gateway. This desktop controller currently defaults to the previously used `172.16.10.1`; check your drone Wi-Fi **Default Gateway** and use `--host` if it differs.

**Video:** The desktop's existing H.264/TCP receiver on port 8888 was developed for the legacy H37 drone. The supplied Android app uses firmware-dependent video transport and it is **not established** that the desktop receiver can show its video. A missing video stream does not prevent UDP control packets. Use `--no-video` for the first flight-control protocol test.

**Connection limitations:** A successful UDP send confirms only local transmission, never drone reception. There is no command acknowledgement, telemetry confirmation, battery monitor, guaranteed radio failsafe or automatic landing. Tests require **propellers removed** until packet behavior is confirmed.

## Files

- [main.py](main.py): Pygame UI, UDP transport and selectable flight protocol
- [android_protocol.py](android_protocol.py): original Android 11-byte packet construction
- [tests/test_android_protocol.py](tests/test_android_protocol.py): neutral packet vectors, flags, checksum, trims and direction reversal
- [tests/test_main.py](tests/test_main.py): legacy H37 protocol and UDP loopback
- [tests/test_drone.py](tests/test_drone.py): older Python 2 implementation regressions
- [.github/workflows/python-checks.yml](.github/workflows/python-checks.yml): Python 3.12 CI for Windows and Linux

The legacy implementation is based on earlier protocol work by Max Golubev and Adria Junyent-Ferré. MIT license; preserve original attribution when redistributing.
