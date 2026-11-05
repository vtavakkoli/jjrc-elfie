# JJRC Elfie — Original Android Flight Deck

[![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB)](https://www.python.org/downloads/)
[![Python CI](https://github.com/vtavakkoli/jjrc-elfie/actions/workflows/python-checks.yml/badge.svg?branch=master)](https://github.com/vtavakkoli/jjrc-elfie/actions/workflows/python-checks.yml)

A clean Windows/Linux desktop controller for the original JJRC Android drone. **Only the original Android 11-byte protocol is supported.** Takeoff, altitude response, steering, landing, emergency stop and speed selection have been confirmed by the drone owner. The experimental **F = camera-assisted visual hold** mode is new and has **not been physically tested or calibrated**.

## Quick start

Connect your computer to the drone's Wi-Fi first.

```powershell
git clone https://github.com/vtavakkoli/jjrc-elfie.git
cd jjrc-elfie
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
```

If you already installed the project, run `git pull origin master` and `python -m pip install -r requirements.txt` again; optical tracking requires OpenCV.

The program uses the original Android control packets (`FF 08 ...`, 11 bytes, additive-complement checksum), UDP port **8080**, approximately 40 Hz target sending, default host **172.16.10.1**. Set `--host` if your drone Wi-Fi Default Gateway differs. The existing TCP H.264 camera receiver uses port **8888**.

## Controls

| Key | Action |
| --- | --- |
| **T** | Request takeoff and enable the drone's own altitude hold |
| **Up / Down** | Climb / descend when altitude hold is active |
| **W / S** | Pitch forward / backward |
| **A / D** | Roll left / right |
| **Left / Right arrows** | Yaw / rotate |
| **F** | Toggle **experimental camera-assisted visual hold** (requires video and completed takeoff) |
| **L** | Request landing; disables F |
| **X** | Emergency stop motors; disables F and may cause an uncontrolled fall |
| **Space** | Neutral input, disables F |
| **H** | Toggle altitude hold (disarmed only) |
| **N** | Toggle headless mode |
| **C** | Calibration request (disarmed, stationary drone only) |
| **+ / −** | Cycle original Android speed modes 0 / 1 / 2 |
| **Esc** | Exit; sends one neutral packet, **not** a safe landing guarantee |

Window losing focus also cancels F and sends neutral movement input. Manual W/S/A/D/arrow keys immediately cancel F and return control to the pilot.

## F: experimental visual landmark stabilization

**This is not true 3-D position hold. Do not rely on it to avoid collisions, prevent drifting, or maintain a fixed height.** A forward-facing single camera can confuse drone translation with camera pitch/yaw and changes in landmark depth. A true hover controller normally needs additional sensors and calibrated flight dynamics. The drone's built-in altitude-hold mode handles altitude to the extent its firmware can; the camera code **does not command vertical throttle or yaw**.

When F is pressed, the application:

1. Requires takeoff to have completed, altitude hold to be enabled, and a fresh live video frame.
2. Acquires a landmark reference image using OpenCV ORB features.
3. Estimates relative image translation and apparent landmark scale using RANSAC.
4. Applies **small, bounded roll/pitch corrections** (default maximum 22% of joystick range).
5. Disables F and sends neutral axes when video stalls, tracking quality degrades, the image moves too far, UDP fails, window focus is lost, or the pilot moves a stick, lands or stops.

The overlay shows `ACQUIRING`, `TRACKING`, or an explicit disengagement reason. `TRACKING` means only that visual features match, **not** that the drone is stationary. It cannot estimate actual location, height, distance, velocity in meters/second, or guarantee stable hover. The sign and strength of corrections have **not** been confirmed on real hardware. Incorrect corrections can accelerate drift.

**Initial test sequence:** With propellers removed, first verify the camera is live and that F stays unavailable while the drone is disarmed. Verify F can be toggled off and that stale video disables it. Next, with the drone *powered off*, test visual motion with a prerecorded/simulated frame source. Do not attempt an untethered autonomous hover until correction directions have been independently validated in a controlled flight-test setup. If F produces unexpected movement, press **Space** to cancel it immediately; **X** stops motors but causes loss of lift.

Optional tuning (only after bench validation):

```powershell
# Restrict camera corrections to 10% of stick travel
python main.py --hold-max-command 0.10

# Run normal manual flight control without video; F is unavailable
python main.py --no-video

# Preview UI and packets without connecting to hardware
python main.py --demo

# Firmware-specific right-stick/yaw reversal if your device requires it
python main.py --device-type UDP_720 --firmware-version V2.4.0

# Original Android manual control input sensitivity
python main.py --stick-gain 0.75

# Run regression and synthetic image tests
python -m unittest discover -s tests -v
```

## Repository structure

- [main.py](main.py) — original Android flight deck, keyboard controls, video and status UI
- [android_protocol.py](android_protocol.py) — original Android packet encoder (takeoff/land/stop, speed, trim and altitude-hold flags)
- [vision_hold.py](vision_hold.py) — optional bounded camera feature tracker with loss-of-tracking cutoff
- [tests/test_android_protocol.py](tests/test_android_protocol.py) — protocol vectors and control-bit tests
- [tests/test_vision_hold.py](tests/test_vision_hold.py) — synthetic-image vision and UDP regressions
- [.github/workflows/python-checks.yml](.github/workflows/python-checks.yml) — Windows/Ubuntu Python 3.12 CI

**Limitations:** This is experimental software, not a certified flight controller. A successful UDP send is not proof that the drone has received or executed it. The app has no battery, inertial, altimeter, obstacle or GPS telemetry, no confirmed radio-loss failsafe, and no guaranteed safe landing on application exit. Keep the aircraft in sight, use an unobstructed test area, and maintain direct manual intervention.

Distributed under the MIT license.
