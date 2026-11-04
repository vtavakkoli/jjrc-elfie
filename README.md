# JJRC H37 Elfie / Eachine — Python 3 FPV Flight Deck

A modern, one-window keyboard controller for the original JJRC H37 Elfie-compatible Wi-Fi drone. The project retains its documented UDP flight protocol and adds an integrated H.264 live camera view.

## Quick start

1. Install **Python 3.10+** and connect your computer to the drone's Wi-Fi access point.
2. Create and activate a virtual environment.
3. Install the dependencies:

   ```bash
   python -m pip install -r requirements.txt
   ```

4. Run:

   ```bash
   python main.py
   ```

On Windows, use `py -3 -m venv .venv` and `.venv\\Scripts\\activate` before installing. On Linux/macOS, use `python3 -m venv .venv` and `source .venv/bin/activate`.

**Preview the interface without connecting a drone:** `python main.py --demo`.

## Controls

| Keys | Action |
| --- | --- |
| W / S | Pitch forward / backward |
| A / D | Roll left / right |
| Up / Down | Altitude-hold vertical input |
| Left / Right | Yaw left / right |
| T | Arm / spin-up after 10-second controller startup (not autonomous takeoff) |
| L | Request landing (not confirmed by telemetry) |
| X | Emergency motor shut-off (may cause immediate fall) |
| Space (hold) | Send neutral movement axes |
| + / - | Adjust maximum movement command strength (10–80%) |
| Escape | Quit the interface (neutralize axes; does **not** guarantee landing) |

Keep the application window focused to control the drone. When it loses focus, the program sends neutral commands.

## Video / network details

- Drone control: UDP `172.16.10.1:8895`, 20 commands/second. On Windows this uses an explicitly **bound, unconnected UDP socket** with `sendto()` and a 1-second retry after socket errors.
- The program first sends 5 seconds of neutral idle packets, then 5 seconds of altitude-hold packets, matching the original reverse-engineered initialization sequence. Only then can `T` send the arm request.
- The UI displays highlighted keys, last transmitted packet, packet count, local interface and errors. **UDP TX does not mean the drone acknowledged a command.**
- The controller attempts to bind to the same local IPv4 address that the working TCP video stream uses. If that is not correct, run `python main.py --local-ip 172.16.10.X` using your PC’s actual Wi-Fi IPv4 address.
- Camera: TCP `172.16.10.1:8888`, original 106-byte handshake protocol and raw H.264 stream.
- Decoding: **PyAV** (FFmpeg bindings) in a background thread.
- Display and keys: **Pygame 2**, one resizable desktop window.
- Override addresses using `--host`, `--control-port`, and `--video-port`.

This video protocol is documented for the **original JJRC H37 Elfie family**. Rebranded or later hardware revisions may use a different protocol. The video reader retries on disconnection, but compatibility needs verification on your exact drone.

## Windows troubleshooting: control error 10022

Windows error 10022 means an invalid socket argument or state. The earlier implementation used `connect()` + `send()` for UDP; this update instead explicitly binds a local UDP socket and uses `sendto()`, recreating it after errors. If control errors remain, run `ipconfig` in PowerShell and find the IPv4 address of your drone-connected Wi-Fi adapter (typically `172.16.10.x`). Run `python main.py --local-ip YOUR_WIFI_IPV4`. Avoid your Ethernet, VPN, loopback, or other interfaces. Confirm the drone IP/port in the UI. Video working establishes a TCP connection, not successful UDP control. If the indicator shows TX packets but the motors never respond with propellers removed, check drone variant and verify packets with Wireshark; there is no UDP acknowledgement.

## Flight safety

**This code has not been flight-tested on actual drone hardware.** First test with propellers removed, verify directions and the emergency-stop button, and only then consider flight in a safe open area. The emergency stop immediately cuts motor power. Use `L` to request a landing before quitting; landing is unconfirmed. Closing the app sends neutral commands, **not a verified safe landing sequence**. UDP does not acknowledge delivery, so an indicated command may not have been received by the drone. There is no battery telemetry, obstacle avoidance, autonomous landing, or guaranteed link-loss failsafe.

## Legacy files

`drone.py` and `kbd_flight.py` are historical Python 2 / joystick prototypes retained for reference. Use `main.py` for the new Python 3 UI.

## Credits

The original UDP protocol and raw-video handshake derive from the reverse-engineering work by Adria Junyent-Ferré: [Controlling a JJRC H37 Elfie quad from a PC](https://hackaday.io/project/19680-controlling-a-jjrc-h37-elfie-quad-from-a-pc).
