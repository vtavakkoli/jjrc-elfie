"""Offline tests for optical-flow landmark stabilization; never require a drone."""
import socket
import math
import threading
import time
import unittest
from unittest import mock

import cv2
import numpy as np

from android_protocol import android_packet
from main import FlightLink, VIDEO_HELLO
from vision_hold import (
    CameraHold, FeatureAnchor, HoldReport, MotionLimitExceeded,
    TrackingFailure, MAX_ROLL_PITCH,
)


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

    def warped(self, angle=0, scale=1, dx=0, dy=0):
        affine = cv2.getRotationMatrix2D((159.5, 119.5), angle, scale)
        affine[:, 2] += (dx, dy)
        return cv2.warpAffine(self.image, affine, (320, 240))

    def test_left_right_rotation_does_not_create_translation_or_zoom(self):
        for angle in (-25, -15, -8, 8, 15, 25):
            with self.subTest(angle=angle):
                anchor = FeatureAnchor()
                anchor.anchor(self.image)
                p = anchor.estimate(self.warped(angle=angle))
                self.assertEqual(p.state, "TRACKING")
                self.assertAlmostEqual(p.shift_x, 0, delta=0.005)
                self.assertAlmostEqual(p.scale, 1, delta=0.01)
                self.assertAlmostEqual(p.roll, 0, delta=0.005)
                self.assertAlmostEqual(p.pitch, 0, delta=0.005)

    def test_rotation_and_return_keep_original_reference(self):
        original = self.anchor.anchor_points.copy()
        for angle in [0, 4, 8, 12, 16, 20, 16, 12, 8, 4, 0,
                      -4, -8, -12, -16, -20, -16, -12, -8, -4, 0]:
            p = self.anchor.estimate(self.warped(angle=angle))
            self.assertEqual(p.state, "TRACKING")
        self.assertAlmostEqual(p.shift_x, 0, delta=0.01)
        self.assertAlmostEqual(p.scale, 1, delta=0.015)
        np.testing.assert_array_equal(self.anchor.anchor_points, original)

    def test_rotated_translation_and_expansion_keep_correction_signs(self):
        p = self.anchor.estimate(self.warped(angle=15, scale=1.045, dx=12))
        self.assertAlmostEqual(p.shift_x, 12 / 320, delta=0.008)
        self.assertGreater(p.roll, 0)
        self.assertLess(p.pitch, 0)
        self.assertLessEqual(abs(p.roll), MAX_ROLL_PITCH)
        self.assertLessEqual(abs(p.pitch), MAX_ROLL_PITCH)

    def test_original_reference_can_be_found_after_flow_is_lost(self):
        self.anchor.estimate(self.warped(dx=10))
        original = self.anchor.anchor_desc.copy()
        self.anchor.pause()
        p = self.anchor.estimate(self.warped(angle=20, dx=20))
        self.assertAlmostEqual(p.shift_x, 20 / 320, delta=0.008)
        np.testing.assert_array_equal(self.anchor.anchor_desc, original)

    def test_mild_perspective_yaw_keeps_tracking(self):
        k = np.array([[240, 0, 159.5], [0, 240, 119.5], [0, 0, 1.]])
        for angle in (-5, 5):
            theta = math.radians(angle)
            rotation = np.array([[math.cos(theta), 0, math.sin(theta)],
                                 [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]])
            turned = cv2.warpPerspective(self.image, k @ rotation @ np.linalg.inv(k), (320, 240))
            anchor = FeatureAnchor()
            anchor.anchor(self.image)
            p = anchor.estimate(turned)
            self.assertEqual(p.state, "TRACKING")
            self.assertLessEqual(abs(p.roll), MAX_ROLL_PITCH)
            self.assertLessEqual(abs(p.pitch), MAX_ROLL_PITCH)

    def test_partial_occlusion_does_not_move_the_reference(self):
        occluded = self.image.copy()
        occluded[:120, :120] = 0
        p = self.anchor.estimate(occluded)
        self.assertAlmostEqual(p.shift_x, 0, delta=0.005)
        self.assertAlmostEqual(p.scale, 1, delta=0.01)

    def test_concentrated_landmarks_cannot_control_the_whole_view(self):
        image = np.zeros_like(self.image)
        image[100:140, 135:185] = np.random.default_rng(7).integers(0, 256, (40, 50), dtype=np.uint8)
        anchor = FeatureAnchor()
        anchor.anchor(image)
        with self.assertRaisesRegex(TrackingFailure, "CONCENTRATED"):
            anchor.estimate(image)

    def test_excessive_motion_still_disengages(self):
        for options in (dict(angle=45), dict(scale=1.35), dict(dx=75), dict(dy=60)):
            with self.subTest(options=options):
                anchor = FeatureAnchor()
                anchor.anchor(self.image)
                with self.assertRaises(MotionLimitExceeded):
                    anchor.estimate(self.warped(**options))

    def test_reference_motion_limit_cannot_be_overridden_by_drifted_flow(self):
        self.anchor.pause()  # force an absolute match before trying flow
        original = self.anchor.anchor_points.copy()
        with mock.patch.object(self.anchor, "_flow_matches", return_value=(original, original)):
            with self.assertRaises(MotionLimitExceeded):
                self.anchor.estimate(self.warped(angle=45))

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


class FakeEstimator:
    """Deterministic observations for worker timing/override tests."""

    def __init__(self, max_command):
        self.anchor_desc = None
        self.pauses = 0
        self.entered = threading.Event()
        self.release = threading.Event()

    @staticmethod
    def grayscale(frame):
        if isinstance(frame, Exception):
            raise frame
        return frame

    def anchor(self, gray):
        self.anchor_desc = True
        return 90

    def estimate(self, gray):
        if gray == "blocked":
            self.entered.set()
            if not self.release.wait(timeout=2):
                raise ValueError("test worker timed out")
            return HoldReport(True, "TRACKING", roll=0.20)
        return gray

    def pause(self):
        self.pauses += 1


class CameraRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.now = 1000.0
        self.estimators = []

        def factory(max_command):
            estimator = FakeEstimator(max_command)
            self.estimators.append(estimator)
            return estimator

        self.factory = mock.patch("vision_hold.FeatureAnchor", side_effect=factory)
        self.clock = mock.patch("vision_hold.time.monotonic", side_effect=lambda: self.now)
        self.factory.start()
        self.clock.start()
        self.hold = CameraHold()
        self.hold.enable(self.now)

    def tearDown(self):
        for estimator in self.estimators:
            estimator.release.set()
        self.hold.close()
        self.clock.stop()
        self.factory.stop()

    def wait_for_report(self, previous):
        deadline = time.perf_counter() + 2
        while time.perf_counter() < deadline:
            report = self.hold.snapshot(self.now)
            if report is not previous:
                return report
            time.sleep(0.001)
        self.fail("worker did not publish a report")

    def deliver(self, observation):
        previous = self.hold.snapshot(self.now)
        self.now += 0.11  # advance capture clock, not real time
        self.hold.submit(observation, self.now)
        return self.wait_for_report(previous)

    def start_tracking(self):
        self.deliver("anchor")
        report = self.deliver(HoldReport(True, "TRACKING", roll=0.1, pitch=-0.05))
        self.assertEqual(report.state, "TRACKING")

    def test_brief_loss_is_neutral_until_two_confirmed_observations(self):
        self.start_tracking()
        report = self.deliver(TrackingFailure("BLURRED"))
        self.assertTrue(report.enabled)
        self.assertEqual(report.state, "RECOVERING")
        self.assertEqual((report.roll, report.pitch), (0, 0))
        observation = HoldReport(True, "TRACKING", roll=0.1, pitch=-0.05)
        report = self.deliver(observation)
        self.assertEqual(report.state, "RECOVERING")
        self.assertEqual((report.roll, report.pitch), (0, 0))
        report = self.deliver(observation)
        self.assertEqual(report.state, "TRACKING")
        self.assertEqual(report.roll, 0.1)
        self.assertEqual(self.estimators[0].pauses, 1)

    def test_persistent_loss_expires_even_while_video_is_fresh(self):
        self.start_tracking()
        for _ in range(8):
            report = self.deliver(TrackingFailure("NO LANDMARKS"))
            if not report.enabled:
                break
        self.assertFalse(report.enabled)
        self.assertEqual(report.state, "CAMERA LOCK LOST: HOLD OFF")
        self.assertEqual((report.roll, report.pitch), (0, 0))

    def test_intermittent_unconfirmed_matches_do_not_extend_recovery(self):
        self.start_tracking()
        for i in range(8):
            observation = (TrackingFailure("BLURRED") if i % 2 == 0
                           else HoldReport(True, "TRACKING", roll=0.1))
            report = self.deliver(observation)
            self.assertEqual((report.roll, report.pitch), (0, 0))
            if not report.enabled:
                break
        self.assertFalse(report.enabled)

    def test_geometry_limit_disables_immediately(self):
        self.start_tracking()
        report = self.deliver(MotionLimitExceeded("IMAGE DRIFT TOO LARGE"))
        self.assertFalse(report.enabled)
        self.assertIn("IMAGE DRIFT TOO LARGE", report.state)

    def test_acquisition_retries_a_brief_texture_failure(self):
        report = self.deliver(TrackingFailure("NO TEXTURE"))
        self.assertTrue(report.enabled)
        self.assertEqual(report.state, "ACQUIRING")
        self.start_tracking()

    def test_old_generation_cannot_restore_commands_after_override(self):
        self.start_tracking()
        estimator = self.estimators[0]
        self.now += 0.11
        self.hold.submit("blocked", self.now)
        self.assertTrue(estimator.entered.wait(timeout=1))
        self.hold.disable("MANUAL OVERRIDE")
        self.hold.enable(self.now)
        self.assertEqual(self.hold.snapshot(self.now).roll, 0)
        previous = self.hold.snapshot(self.now)
        self.now += 0.11
        self.hold.submit("new anchor", self.now)
        estimator.release.set()
        report = self.wait_for_report(previous)
        self.assertEqual(report.state, "ACQUIRING")
        self.assertEqual(report.roll, 0)
        self.assertEqual(len(self.estimators), 2)

    def test_delayed_processing_cannot_publish_a_stale_correction(self):
        self.start_tracking()
        estimator = self.estimators[0]
        self.now += 0.11
        self.hold.submit("blocked", self.now)
        self.assertTrue(estimator.entered.wait(timeout=1))
        previous = self.hold.snapshot(self.now)
        self.now += 0.60
        estimator.release.set()
        report = self.wait_for_report(previous)
        self.assertFalse(report.enabled)
        self.assertEqual(report.roll, 0)


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
