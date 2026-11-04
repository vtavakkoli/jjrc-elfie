"""Protocol regression checks. Run: python -m unittest discover -s tests -v"""
import unittest

from main import VIDEO_HELLO, packet


class FlightProtocolTests(unittest.TestCase):
    def test_video_handshake_length(self):
        self.assertEqual(len(VIDEO_HELLO), 106)

    def test_neutral_hold(self):
        self.assertEqual(packet().hex(), "6680808080000099")

    def test_arm(self):
        self.assertEqual(packet(command=1).hex(), "6680808080010199")

    def test_emergency(self):
        self.assertEqual(packet(command=4).hex(), "6680808080040499")

    def test_packet_layout_and_checksum(self):
        data = packet(roll=0.5, pitch=-0.5, throttle=0.2, yaw=-0.2)
        self.assertEqual(len(data), 8)
        self.assertEqual(data[0], 0x66)
        self.assertEqual(data[-1], 0x99)
        check = 0
        for byte in data[1:6]:
            check ^= byte
        self.assertEqual(data[6], check)

    def test_clamp(self):
        self.assertEqual(packet(roll=9)[1], 255)
        self.assertEqual(packet(roll=-9)[1], 0)


if __name__ == "__main__":
    unittest.main()
