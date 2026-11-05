"""Compare original Android ControlActivity.SendThread packet fields."""
import unittest

from android_protocol import (
    ANDROID_CONTROL_PORT, ANDROID_SEND_PERIOD, android_packet,
    reverse_for_firmware,
)


class OriginalAndroidPacketTests(unittest.TestCase):
    def test_android_neutral_manual_vector(self):
        self.assertEqual(android_packet().hex(), "ff08003f403f1010100009")

    def test_android_altitude_hold_neutral_vector(self):
        self.assertEqual(
            android_packet(altitude_hold=True).hex(),
            "ff087e3f403f901010000b",
        )

    def test_flags_preserve_checksum(self):
        for flag, mask in (
            ("takeoff", 0x40), ("land", 0x80), ("emergency_stop", 0x20),
            ("headless", 0x10), ("flip", 0x04),
        ):
            with self.subTest(flag=flag):
                p = android_packet(altitude_hold=True, **{flag: True})
                self.assertEqual(len(p), 11)
                self.assertEqual(p[0:2], bytes.fromhex("ff08"))
                self.assertEqual(p[9], mask)
                self.assertEqual(sum(p[1:]) & 0xFF, 0xFF)

    def test_speed_modes_and_calibration(self):
        self.assertEqual(android_packet(speed_mode=2)[9], 2)
        self.assertEqual(android_packet(altitude_hold=True, calibrate=True)[6], 0xD0)
        for bad_speed in (-1, 3):
            with self.assertRaises(ValueError):
                android_packet(speed_mode=bad_speed)
        with self.assertRaises(ValueError):
            android_packet(takeoff=True, land=True)

    def test_rocker_directions_and_reverse_firmware(self):
        neutral = android_packet()
        self.assertLess(android_packet(pitch=1)[4], neutral[4])
        self.assertGreater(android_packet(roll=1)[5], neutral[5])
        normal = android_packet(yaw=0.4, roll=0.4)
        reversed_data = android_packet(yaw=0.4, roll=0.4, reverse_direction=True)
        self.assertEqual(normal[3] + reversed_data[3], 127)
        self.assertEqual(normal[5] + reversed_data[5], 127)
        for version in ("V2.3.9", "V2.4.0", "V2.4.1"):
            self.assertTrue(reverse_for_firmware("UDP_720", version))
            self.assertFalse(reverse_for_firmware("UDP_1080", version))

    def test_trim_and_throttle_limits(self):
        self.assertEqual(android_packet(trim_yaw=0, trim_pitch=32, trim_roll=32)[6:9],
                         bytes([0, 0, 32]))
        self.assertEqual(android_packet(throttle=1)[2], 254)
        self.assertEqual(android_packet(throttle=-1)[2], 0)
        self.assertEqual(android_packet(altitude_hold=True)[2], 126)
        for name in ("trim_yaw", "trim_pitch", "trim_roll"):
            with self.assertRaises(ValueError):
                android_packet(**{name: 33})

    def test_keyboard_deflection_matches_android_rocker_ranges(self):
        # Android original right/left rockers range 0..127 (neutral 63).
        # A held keyboard direction now uses 75% of the full rocker range.
        gain = 0.75
        neutral = android_packet(altitude_hold=True)
        left = android_packet(roll=-gain, altitude_hold=True)
        right = android_packet(roll=gain, altitude_hold=True)
        forward = android_packet(pitch=gain, altitude_hold=True)
        backward = android_packet(pitch=-gain, altitude_hold=True)
        yaw_left = android_packet(yaw=-gain, altitude_hold=True)
        yaw_right = android_packet(yaw=gain, altitude_hold=True)
        self.assertEqual(neutral[3:6], bytes([63, 64, 63]))
        self.assertLessEqual(left[5], 32)
        self.assertGreaterEqual(right[5], 96)
        self.assertLessEqual(forward[4], 32)
        self.assertGreaterEqual(backward[4], 96)
        self.assertLessEqual(yaw_left[3], 32)
        self.assertGreaterEqual(yaw_right[3], 96)
        for data in (left, right, forward, backward, yaw_left, yaw_right):
            self.assertEqual(len(data), 11)
            self.assertEqual(sum(data[1:]) & 0xff, 0xff)

    def test_speed_mode_only_changes_packet_flag(self):
        packets = [android_packet(roll=0.75, speed_mode=n) for n in range(3)]
        self.assertEqual([p[9] for p in packets], [0, 1, 2])
        for p in packets:
            self.assertEqual(p[0:9], packets[0][0:9])
            self.assertEqual(sum(p[1:]) & 0xff, 0xff)

    def test_android_network_values(self):
        self.assertEqual(ANDROID_CONTROL_PORT, 8080)
        self.assertEqual(ANDROID_SEND_PERIOD, 0.025)


if __name__ == "__main__":
    unittest.main()
