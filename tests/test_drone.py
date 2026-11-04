"""Historical JJRC H37 UDP packet regression vectors.

Originally these cases targeted the Python 2-only drone.py implementation
with nose2 and third-party mock. They now exercise the supported Python 3.12
packet encoder in main.py and require only the standard-library unittest.

The captured raw-throttle packets use the modern encoder's [-1, 1] normalized
axis representation. No live drone connection is made by these tests.
"""

import unittest

from main import HOLD_PACKET, IDLE_PACKET, packet


class OriginalJJRCProtocolTests(unittest.TestCase):
    def test_neutral_altitude_hold(self):
        self.assertEqual(packet(), HOLD_PACKET)
        self.assertEqual(packet().hex(), "6680808080000099")

    def test_original_idle_packet(self):
        self.assertEqual(IDLE_PACKET.hex(), "6680800180008199")

    def test_arm_command(self):
        self.assertEqual(packet(command=1).hex(), "6680808080010199")

    def test_emergency_motor_stop_command(self):
        self.assertEqual(packet(command=4).hex(), "6680808080040499")

    def test_original_capture_low_throttle_pitch(self):
        # Original captured packet: roll=128 pitch=127 throttle=1 yaw=128.
        # Legacy low-throttle 0.004 maps to normalized -0.992 in main.py.
        data = packet(roll=0.0, pitch=-0.004, throttle=-0.992, yaw=0.0)
        self.assertEqual(data.hex(), "66807f0180007e99")

    def test_original_capture_low_throttle_roll(self):
        # Original captured packet: roll=107 pitch=132 throttle=0 yaw=128.
        data = packet(roll=-0.16, pitch=0.035, throttle=-1.0, yaw=0.0)
        self.assertEqual(data.hex(), "666b840080006f99")

    def test_direction_and_limits(self):
        # Neutral is 128; controls must move in expected directions.
        self.assertGreater(packet(roll=0.3)[1], 128)
        self.assertLess(packet(roll=-0.3)[1], 128)
        self.assertGreater(packet(pitch=0.3)[2], 128)
        self.assertLess(packet(pitch=-0.3)[2], 128)
        self.assertGreater(packet(throttle=0.3)[3], 128)
        self.assertLess(packet(throttle=-0.3)[3], 128)
        self.assertGreater(packet(yaw=0.3)[4], 128)
        self.assertLess(packet(yaw=-0.3)[4], 128)
        self.assertEqual(packet(roll=999)[1], 255)
        self.assertEqual(packet(roll=-999)[1], 0)


if __name__ == "__main__":
    unittest.main()
