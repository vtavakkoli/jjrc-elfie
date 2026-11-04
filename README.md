# JJRC Elfie — FPV Flight Deck

[![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![Tests](https://github.com/vtavakkoli/jjrc-elfie/actions/workflows/python-checks.yml/badge.svg?branch=master)](https://github.com/vtavakkoli/jjrc-elfie/actions/workflows/python-checks.yml)
[![Platforms](https://img.shields.io/badge/platform-Windows%20%7C%20Linux-58657A)](#requirements)
[![MIT License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

**Keyboard-controlled FPV desktop application for the JJRC H37 Elfie / compatible Eachine drones.** One resizable window combines a live camera feed, keyboard flight inputs, UDP transmission status, and connectivity diagnostics.

> **Hardware status (4 November 2026):** Video streaming, motor spin-up at low speed, ascent with the Up arrow, and **X emergency motor stop** have been confirmed on a drone by a project user. Other directions and the landing request have not yet been independently confirmed. The code cannot guarantee safe flight.

## Highlights

- **One-command launch:** `python main.py` — no joystick or Python 2 needed
- **Live first-person video:** TCP H.264 decoding through [PyAV](https://pyav.org/) / FFmpeg
- **Keyboard piloting:** WASD, arrows, arm, landing request, and emergency stop
- **Windows-friendly network control:** explicitly bound UDP `sendto()` socket
- **Connection diagnostics:** active keys, movement axes, source IP, packet count and errors
- **Video auto-reconnect and demo mode:** inspect the interface without a drone
- **Automated tests:** Python 3.12 CI on Windows and Ubuntu, with protocol regression vectors and local UDP loopback checks

## Requirements

| Component | Requirement |
| --- | --- |
| Python | **3.12** (recommended and tested in CI) |
| Desktop OS | Windows or Linux with a graphical desktop |
| Drone | JJRC H37 Elfie or compatible variant using the original protocol |
| Network | Computer connected to the drone's Wi-Fi |
| Libraries | Pygame 2, PyAV, NumPy (installed from `requirements.txt`) |

No OpenCV or external video player is needed.

## Quick start

### Windows PowerShell

```powershell
git clone https://github.com/vtavakkoli/jjrc-elfie.git
cd jjrc-elfie
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py --demo
```

Connect your PC to the **drone Wi-Fi** and launch live flight mode:

```powershell
python main.py
```

If PowerShell blocks virtual environment activation, run `.venv\Scripts\python.exe main.py` and use `.venv\Scripts\python.exe -m pip install -r requirements.txt` instead.

### Linux

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py --demo
# Connect to drone Wi-Fi before starting the live controller
python main.py
```

**Important:** Start with the propellers **removed** when checking arm and keyboard commands for the first time.

## Keyboard controls

| Key | Control |
| :---: | --- |
| **W / S** | Pitch forward / backward |
| **A / D** | Roll / strafe left / right |
| **↑ / ↓** | Ascend / descend in altitude-hold mode |
| **← / →** | Yaw / rotate left / right |
| **T** | Request motor arm / spin-up after startup |
| **L** | Request landing (not confirmed by telemetry) |
| **X** | **Emergency motor shut-off** — immediate loss of lift |
| **Space** | Hold neutral movement axes |
| **+ / −** | Set movement gain (10–80%) |
| **Esc** | Close controller; sends neutral command, not a guaranteed landing |

Click the flight window to focus it before pressing keys. Losing window focus commands neutral movement axes. **The arm indicator reflects a sent request, not an acknowledged aircraft state.**

## Startup and network

```text
Computer                              JJRC Elfie
┌────────────────────────┐            ┌─────────────────────┐
│ main.py / Pygame UI    │            │ 172.16.10.1         │
│                        │── UDP ────>│ :8895 flight control│
│ keyboard → UDP packets │   20 Hz    │                     │
│                        │<─ TCP ─────│ :8888 H.264 video   │
│ video decoder (PyAV)   │            │                     │
└────────────────────────┘            └─────────────────────┘
```

The controller transmits the original startup sequence: **5 seconds of idle packets**, then **5 seconds of altitude-hold packets**. After 10 seconds, press **T** to send the arm request. **Up arrow** then requests upward throttle in altitude-hold mode.

Run with non-default parameters when needed:

```powershell
python main.py --host 172.16.10.1 --control-port 8895 --video-port 8888
```

### Troubleshoot Windows UDP errors

If the camera works but the UI shows **WinError 10022**, identify the IPv4 address of the wireless adapter connected to the drone:

```powershell
ipconfig
python main.py --local-ip YOUR_DRONE_WIFI_IPV4
```

The controller normally detects this interface from the working TCP video connection. The **UDP TX** count confirms transmission from the PC only; the drone does not send an acknowledgement for these control commands.

## Tests and development

The tests run **without hardware**, covering the original packet vectors, the H.264 handshake length, button command encoding, and local UDP socket transmission.

```powershell
python -m py_compile main.py
python -m unittest discover -s tests -v
python -m pip check
```

GitHub Actions runs these checks on **Windows and Ubuntu with Python 3.12**. Legacy Python 2 `nose2` / `mock` test dependencies are no longer needed.

## Safety and limitations

This is experimental reverse-engineered flight software, **not a certified flight controller**. Test with propellers removed before flight, use a clear environment, and keep the drone in direct sight. **X** requests immediate motor shut-off and can cause an uncontrolled fall. **L** sends a landing request, but its physical behavior remains unverified on the tested drone. Closing the app does not guarantee a landing. There is no telemetry confirmation, battery monitor, obstacle avoidance, autonomous recovery, or guaranteed radio-link failsafe.

## Repository layout

| Path | Purpose |
| --- | --- |
| [`main.py`](main.py) | Supported Python 3.12 application: video + flight UI |
| [`requirements.txt`](requirements.txt) | Runtime dependencies |
| [`tests/test_main.py`](tests/test_main.py) | Controller, packet and UDP loopback tests |
| [`tests/test_drone.py`](tests/test_drone.py) | Historical packet-vector regression tests, migrated to Python 3 |
| [`.github/workflows/python-checks.yml`](.github/workflows/python-checks.yml) | Python 3.12 checks on Windows and Linux |
| `drone.py`, `kbd_flight.py` | Historical Python 2 examples, **not** the supported entry point |

## Credits and license

The project builds on early work by Max Golubev and the drone protocol investigations by Adria Junyent-Ferré: [Controlling a JJRC H37 Elfie quad from a PC](https://hackaday.io/project/19680-controlling-a-jjrc-h37-elfie-quad-from-a-pc).

Distributed under the [MIT License](LICENSE). Preserve the original license and attribution when redistributing.
