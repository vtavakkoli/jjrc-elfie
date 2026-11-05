"""11-byte flight command reconstructed from the supplied original Android APK.

Sources: ControlActivity.SendThread.run(), MyApplication.isContraryDirection(),
BaseApplication.writeUDPCmd(). Android app transmits to UDP 8080 every 25 ms.
This is independent of the H37 8-byte 0x66 protocol.
"""

ANDROID_CONTROL_PORT = 8080
ANDROID_SEND_PERIOD = 0.025
REVERSE_FIRMWARE = frozenset(("V2.3.9", "V2.4.0", "V2.4.1"))


def reverse_for_firmware(device_type: str, firmware_version: str) -> bool:
    """Mirror the Android app's firmware-specific yaw/roll reversal."""
    return device_type == "UDP_720" and firmware_version in REVERSE_FIRMWARE


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _stick(value: float) -> int:
    """Map centered normalized input to Android RockerView's 0..127 axis.

    RockerView's neutral is 63 (127 // 2). Actual endpoint rounding depends
    on the device's screen coordinates; these are desktop input equivalents.
    """
    return int(_clamp(round(63 + 64 * _clamp(value, -1, 1)), 0, 127))


def android_packet(
    roll: float = 0.0,
    pitch: float = 0.0,
    throttle: float = 0.0,
    yaw: float = 0.0,
    *,
    altitude_hold: bool = False,
    speed_mode: int = 0,
    headless: bool = False,
    calibrate: bool = False,
    emergency_stop: bool = False,
    takeoff: bool = False,
    land: bool = False,
    trim_yaw: int = 16,
    trim_pitch: int = 16,
    trim_roll: int = 16,
    reverse_direction: bool = False,
    flip: bool = False,
) -> bytes:
    """Build Android FF 08 ... checksum command using SendThread layout.

    Altitude hold throttle is centered [-1,+1] (neutral byte 126).
    Manual throttle is an absolute [0,+1] stick position.
    Other axes use centered normalized desktop input [-1,+1].
    """
    if not 0 <= speed_mode <= 2:
        raise ValueError("speed_mode must be 0, 1 or 2")
    if not all(0 <= x <= 32 for x in (trim_yaw, trim_pitch, trim_roll)):
        raise ValueError("trim values must be integers between 0 and 32")
    if takeoff and land:
        raise ValueError("takeoff and land flags cannot be combined")

    # Original: rockerLeft.getYV()*2, rockerLeft.getXV().
    left_y = _stick(throttle) if altitude_hold else round(127 * _clamp(throttle, 0, 1))
    b2 = (int(left_y) * 2) & 0xFF
    left_x = _stick(yaw)
    b3 = (127 - left_x) if reverse_direction else left_x

    # Original: 127-rockerRight.getYV(), rockerRight.getXV().
    right_y = _stick(pitch)
    b4 = 127 - right_y
    right_x = _stick(roll)
    b5 = (127 - right_x) if reverse_direction else right_x

    # Original: trimLeft | atmospheric<<7 | calibrate<<6.
    b6 = trim_yaw | (0x80 if altitude_hold else 0) | (0x40 if calibrate else 0)
    b7 = 32 - trim_pitch
    b8 = trim_roll

    # speed bits 0:1, flip bit 2, headless bit 4,
    # stop bit 5, takeoff bit 6, land bit 7.
    b9 = (speed_mode | (0x04 if flip else 0) | (0x10 if headless else 0)
          | (0x20 if emergency_stop else 0) | (0x40 if takeoff else 0)
          | (0x80 if land else 0))
    payload = [0xFF, 0x08, b2, b3, b4, b5, b6, b7, b8, b9]
    # Java: (byte)(255 - ((byte)(sum(b[1:10])))).
    checksum = (0xFF - sum(payload[1:])) & 0xFF
    return bytes([*payload, checksum])
