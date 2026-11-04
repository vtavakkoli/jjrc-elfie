"""JJRC protocol and local UDP transport tests (no drone or propellers required)."""
import socket
import unittest

from main import HOLD_PACKET, IDLE_PACKET, VIDEO_HELLO, FlightLink, packet


class FlightProtocolTests(unittest.TestCase):
    def test_video_handshake_length(self):
        self.assertEqual(len(VIDEO_HELLO), 106)

    def test_original_idle_and_hold_packets(self):
        self.assertEqual(IDLE_PACKET.hex(), "6680800180008199")
        self.assertEqual(HOLD_PACKET.hex(), "6680808080000099")
        self.assertEqual(packet(), HOLD_PACKET)

    def test_arm_land_and_emergency(self):
        self.assertEqual(packet(command=1).hex(), "6680808080010199")
        self.assertEqual(packet(command=2).hex(), "6680808080020299")
        self.assertEqual(packet(command=4).hex(), "6680808080040499")

    def test_packet_layout_checksum_and_clamp(self):
        data = packet(roll=0.5, pitch=-0.5, throttle=0.2, yaw=-0.2)
        self.assertEqual(len(data), 8)
        self.assertEqual((data[0], data[-1]), (0x66, 0x99))
        checksum = 0
        for value in data[1:6]:
            checksum ^= value
        self.assertEqual(data[6], checksum)
        self.assertEqual(packet(roll=9)[1], 255)
        self.assertEqual(packet(roll=-9)[1], 0)

    def test_windows_style_explicitly_bound_udp_sendto(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(2)
            link = FlightLink("127.0.0.1", receiver.getsockname()[1],
                              local_ip="127.0.0.1")
            try:
                self.assertTrue(link.open())
                self.assertEqual(link.sock.getsockname()[0], "127.0.0.1")
                self.assertTrue(link.send(packet()))
                data, sender = receiver.recvfrom(1024)
                self.assertEqual(data, HOLD_PACKET)
                self.assertEqual(sender[0], "127.0.0.1")
                self.assertEqual(link.sent_count, 1)
                self.assertEqual(link.last_error, "")
            finally:
                link.close()

    def test_demo_mode_never_opens_socket(self):
        link = FlightLink("172.16.10.1", 8895, demo=True)
        self.assertTrue(link.open())
        self.assertTrue(link.send(packet()))
        self.assertIsNone(link.sock)
        self.assertEqual(link.sent_count, 1)
        self.assertEqual(link.last_sent_hex, HOLD_PACKET.hex())


if __name__ == "__main__":
    unittest.main()
