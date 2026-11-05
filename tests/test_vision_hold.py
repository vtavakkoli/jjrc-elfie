"""Offline tests for optical-flow landmark stabilization; never require a drone."""
import socket
import time
import unittest

import cv2
import numpy as np

from android_protocol import android_packet
from main import FlightLink, VIDEO_HELLO
from vision_hold import CameraHold, FeatureAnchor, TrackingFailure, MAX_ROLL_PITCH


def textured_frame():
    rng = np.random.default_rng(42)
    image = np.full((240, 320), 60, dtype=np.uint8)
    for _ in range(460):
        x = int(rng.integers(5, 315))
        y = int(rng.integers(5, 235))
        c = int(rng.integers(105, 255))
        cv2.circle(image, (x, y), int(rng.integers(1, 4)), c, -1)
    return image


class CameraEstimatorTests(unittest.TestCase):
    def setUp(self):
        self.image = textured_frame()
        self.anchor = FeatureAnchor()
        self.assertGreaterEqual(self.anchor.anchor(self.image), 45)

    def test_still_frame_produces_neutral_commands(self):
        p = self.anchor.estimate(self.image.copy())
        self.assertEqual(p.state, "TRACKING")
        self.assertAlmostEqual(p.roll, 0.0, delta=0.018)
        self.assertAlmostEqual(p.pitch, 0.0, delta=0.018)
        self.assertGreaterEqual(p.inliers, 14)

    def test_horizontal_scene_shift_produces_bounded_roll(self):
        shifted = cv2.warpAffine(self.image,
                                 np.float32([[1, 0, 14], [0, 1, 0]]),
                                 (320, 240), borderMode=cv2.BORDER_REFLECT)
        p = self.anchor.estimate(shifted)
        self.assertGreater(p.shift_x, 0.025)
        self.assertGreater(p.roll, 0.0)
        self.assertLessEqual(abs(p.roll), MAX_ROLL_PITCH)

    def test_apparent_expansion_produces_bounded_pitch(self):
        affine = cv2.getRotationMatrix2D((159.5, 119.5), 0, 1.045)
        enlarged = cv2.warpAffine(self.image, affine, (320, 240),
                                  borderMode=cv2.BORDER_REFLECT)
        p = self.anchor.estimate(enlarged)
        self.assertGreater(p.scale, 1.02)
        self.assertLess(p.pitch, 0)
        self.assertLessEqual(abs(p.pitch), MAX_ROLL_PITCH)

    def test_loss_of_features_fails_closed(self):
        with self.assertRaises(TrackingFailure):
            self.anchor.estimate(np.zeros_like(self.image))

    def test_grayscale_and_limits(self):
        rgb = np.repeat(self.image[:, :, None], 3, axis=2)
        gray = self.anchor.grayscale((rgb.tobytes(), 320, 240))
        self.assertEqual(gray.shape, (240, 320))
        with self.assertRaises(TrackingFailure):
            self.anchor.grayscale((b"bad", 320, 240))
        with self.assertRaises(ValueError):
            FeatureAnchor(max_command=0.99)


class CameraSwitchTests(unittest.TestCase):
    def test_f_state_toggle_and_stale_disengagement(self):
        hold = CameraHold()
        try:
            now = time.monotonic()
            hold.enable(now)
            self.assertTrue(hold.snapshot(now).enabled)
            self.assertEqual(hold.snapshot(now).state, "ACQUIRING")
            self.assertFalse(hold.snapshot(now + 2).enabled)
            self.assertIn("OFF", hold.snapshot(now + 2).state)
            hold.enable(time.monotonic())
            hold.disable("MANUAL OVERRIDE")
            self.assertFalse(hold.snapshot(time.monotonic()).enabled)
        finally:
            hold.close()


class AndroidTransportTests(unittest.TestCase):
    def test_original_video_handshake_retained(self):
        self.assertEqual(len(VIDEO_HELLO), 106)

    def test_android_udp_and_shutdown_use_original_format(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(2)
            flight = FlightLink("127.0.0.1", receiver.getsockname()[1],
                                local_ip="127.0.0.1")
            self.assertTrue(flight.open())
            self.assertTrue(flight.send(android_packet(altitude_hold=True, roll=0.75)))
            first, _ = receiver.recvfrom(200)
            self.assertEqual(first[0:2], b"\xff\x08")
            self.assertEqual(len(first), 11)
            flight.close()
            neutral, _ = receiver.recvfrom(200)
            self.assertEqual(neutral, android_packet(altitude_hold=True))


if __name__ == "__main__":
    unittest.main()
