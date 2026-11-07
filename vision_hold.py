"""Experimental monocular visual servoing for the JJRC Android-protocol drone.

NOT a GPS/3-D position hold. The forward-facing camera sees combined effects
of translation, attitude, yaw, depth and lighting. Treat this as an experimental
visual landmark stabilizer, never as a collision/altitude safety controller.

Image processing runs on a background thread to keep 40-Hz UDP control timely.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import queue
import threading
import time

import cv2
import numpy as np


FRAME_MAX_AGE = 0.55
RESULT_MAX_AGE = 0.85
ACQUIRE_TIMEOUT = 1.5
ANALYSIS_INTERVAL = 0.09  # ~11 Hz; network control continues at 40 Hz
RECOVERY_TIMEOUT = 0.65
RECOVERY_CONFIRMATIONS = 2
RELOCALIZE_INTERVAL = 5  # refresh tracks from the fixed reference, not a new anchor
MIN_ANCHOR_FEATURES = 45
MIN_MATCHES = 20
MIN_INLIERS = 14
MAX_ROLL_PITCH = 0.22  # deliberately limited to 22% of stick travel
MAX_VIEW_ROTATION = math.radians(30)


class TrackingFailure(Exception):
    """Vision confidence fell below the safe operating threshold."""


class MotionLimitExceeded(TrackingFailure):
    """A confident estimate exceeds the permitted view/motion envelope."""


@dataclass(frozen=True)
class HoldReport:
    enabled: bool = False
    state: str = "OFF"
    roll: float = 0.0
    pitch: float = 0.0
    shift_x: float = 0.0  # frame width fraction
    scale: float = 1.0
    inliers: int = 0
    note: str = "Press F after takeoff with live video"


class FeatureAnchor:
    """Track a fixed reference with optical flow and ORB relocalization.

    A camera's apparent horizontal frame translation is *not* guaranteed to
    equal lateral translation; yaw and camera attitude are confounders.
    We intentionally limit output and reject large/repetitive movements.
    """

    def __init__(self, max_command: float = MAX_ROLL_PITCH):
        if not (0.05 <= max_command <= 0.35):
            raise ValueError("max_command must be between 0.05 and 0.35")
        self.max_command = float(max_command)
        self.detector = cv2.ORB_create(nfeatures=900, fastThreshold=10, edgeThreshold=19)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.anchor_points = None
        self.anchor_desc = None
        self.frame_shape = None
        self.previous_gray = None
        self.track_reference = None
        self.track_points = None
        self.frames_since_relocalization = 0
        self.smoothed_roll = 0.0
        self.smoothed_pitch = 0.0

    @staticmethod
    def grayscale(frame: tuple[bytes, int, int]) -> np.ndarray:
        pixels, width, height = frame
        raw = np.frombuffer(pixels, dtype=np.uint8)
        if raw.size != width * height * 3:
            raise TrackingFailure("INVALID VIDEO FRAME")
        rgb = raw.reshape((height, width, 3))
        # Keep computational load modest and independent of FPV resolution.
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        w = 320
        h = max(120, min(270, round(gray.shape[0] * w / gray.shape[1])))
        return cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA)

    def anchor(self, gray: np.ndarray) -> int:
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)
        if descriptors is None or len(keypoints) < MIN_ANCHOR_FEATURES:
            raise TrackingFailure("NOT ENOUGH VISUAL FEATURES")
        self.anchor_points = np.float32([p.pt for p in keypoints])
        self.anchor_desc = descriptors
        self.frame_shape = gray.shape
        self.previous_gray = gray.copy()
        self.track_reference = self.anchor_points.copy()
        self.track_points = self.anchor_points.copy()
        self.frames_since_relocalization = 0
        self.smoothed_roll = 0.0
        self.smoothed_pitch = 0.0
        return len(keypoints)

    def estimate(self, gray: np.ndarray) -> HoldReport:
        if self.anchor_desc is None or gray.shape != self.frame_shape:
            raise TrackingFailure("NO MATCHING REFERENCE FRAME")
        # Flow follows small inter-frame motion even when ORB descriptors change
        # with rotation or compression. Every track retains its ORIGINAL anchor
        # coordinate, so the target does not walk along with the aircraft.
        candidates = []
        failures = []
        if (self.track_points is None
                or self.frames_since_relocalization >= RELOCALIZE_INTERVAL):
            candidates.append(self._reference_matches)
        candidates.append(self._flow_matches)
        if self._reference_matches not in candidates:
            candidates.append(self._reference_matches)
        for match in candidates:
            try:
                src, dst = match(gray)
                report, src, dst = self._motion(src, dst)
            except MotionLimitExceeded as exc:
                # A confident absolute reference match outranks possibly
                # drifted incremental flow, including its motion cutoff.
                if match == self._reference_matches:
                    raise
                failures.append(exc)
                continue
            except TrackingFailure as exc:
                failures.append(exc)
                continue
            self.previous_gray = gray.copy()
            self.track_reference = src
            self.track_points = dst
            if match == self._reference_matches:
                self.frames_since_relocalization = 0
            else:
                self.frames_since_relocalization += 1
            return report
        # Do not downgrade an observed large motion to a retryable texture loss.
        for failure in failures:
            if isinstance(failure, MotionLimitExceeded):
                raise failure
        raise failures[-1]

    def _reference_matches(self, gray: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        points, descriptors = self.detector.detectAndCompute(gray, None)
        if descriptors is None or len(points) < MIN_MATCHES:
            raise TrackingFailure("INSUFFICIENT VISIBLE TEXTURE")
        nearest = self.matcher.knnMatch(self.anchor_desc, descriptors, k=2)
        reverse = self.matcher.match(descriptors, self.anchor_desc)
        reverse_best = {m.queryIdx: m.trainIdx for m in reverse}
        matches = [m for pair in nearest if len(pair) == 2
                   for m, n in [pair] if m.distance < 0.75 * n.distance
                   and m.distance <= 64 and reverse_best.get(m.trainIdx) == m.queryIdx]
        if len(matches) < MIN_MATCHES:
            raise TrackingFailure("REFERENCE LANDMARKS LOST")
        src = np.float32([self.anchor_points[m.queryIdx] for m in matches])
        dst = np.float32([points[m.trainIdx].pt for m in matches])
        return src, dst

    def _flow_matches(self, gray: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.previous_gray is None or self.track_points is None:
            raise TrackingFailure("REFERENCE LANDMARKS LOST")
        previous = self.track_points.reshape(-1, 1, 2)
        lk = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        current, forward_status, error = cv2.calcOpticalFlowPyrLK(
            self.previous_gray, gray, previous, None, **lk)
        if current is None or forward_status is None or error is None:
            raise TrackingFailure("OPTICAL FLOW LOST")
        backward, backward_status, _ = cv2.calcOpticalFlowPyrLK(
            gray, self.previous_gray, current, None, **lk)
        if backward is None or backward_status is None:
            raise TrackingFailure("OPTICAL FLOW LOST")
        dst = current.reshape(-1, 2)
        fb_error = np.linalg.norm(backward.reshape(-1, 2) - self.track_points, axis=1)
        height, width = gray.shape
        valid = (forward_status.ravel().astype(bool)
                 & backward_status.ravel().astype(bool)
                 & np.isfinite(dst).all(axis=1) & (fb_error < 1.5)
                 & (error.ravel() < 30.0)
                 & (dst[:, 0] >= 3) & (dst[:, 0] < width - 3)
                 & (dst[:, 1] >= 3) & (dst[:, 1] < height - 3))
        if np.count_nonzero(valid) < MIN_MATCHES:
            raise TrackingFailure("TOO FEW CONSISTENT FLOW TRACKS")
        return self.track_reference[valid], dst[valid]

    def _motion(self, src: np.ndarray, dst: np.ndarray):
        affine, mask = cv2.estimateAffinePartial2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=2.5,
            maxIters=1200, confidence=0.99,
        )
        if affine is None or mask is None:
            raise TrackingFailure("MOTION ESTIMATE FAILED")
        inliers = int(np.count_nonzero(mask))
        if inliers < MIN_INLIERS or inliers / len(src) < 0.53:
            raise TrackingFailure("UNRELIABLE CAMERA TRACKING")

        src = src[mask.ravel().astype(bool)]
        dst = dst[mask.ravel().astype(bool)]
        height, width = self.frame_shape
        # A small moving object or one repeated patch cannot define whole-view
        # motion. Require support across both image axes and a nontrivial area.
        for support in (src, dst):
            span = np.ptp(support, axis=0)
            area = cv2.contourArea(cv2.convexHull(support))
            if span[0] < 0.20 * width or span[1] < 0.15 * height or area < 0.03 * width * height:
                raise TrackingFailure("LANDMARKS TOO CONCENTRATED")

        a, neg_b, tx = [float(v) for v in affine[0]]
        b, a2, ty = [float(v) for v in affine[1]]
        scale = math.hypot(a, b)
        rotation = math.atan2(b, a)
        if not np.isfinite(affine).all():
            raise TrackingFailure("INVALID MOTION ESTIMATE")
        if not (0.83 <= scale <= 1.21) or abs(rotation) > MAX_VIEW_ROTATION:
            raise MotionLimitExceeded("VIEW ROTATED OR DEPTH CHANGED TOO MUCH")
        cx, cy = (width - 1) / 2, (height - 1) / 2
        # Where the center of the reference scene has moved in the image.
        dx = (a * cx + neg_b * cy + tx - cx) / width
        dy = (b * cx + a2 * cy + ty - cy) / height
        if abs(dx) > 0.18 or abs(dy) > 0.20:
            raise MotionLimitExceeded("IMAGE DRIFT TOO LARGE")

        # Visual servo heuristic for a forward-facing fixed camera:
        # scene shifts left when the drone moves right -> roll left (dx<0).
        # moving toward a landmark makes its scale grow -> pitch backward.
        # Camera yaw/tilt can cause the same observations. These gains are
        # intentionally small and MUST be tested before flight.
        roll = 0.0 if abs(dx) < 0.012 else self._clamp(1.9 * dx)
        log_scale = math.log(scale)
        pitch = 0.0 if abs(log_scale) < 0.016 else self._clamp(-1.8 * log_scale)
        self.smoothed_roll = 0.60 * self.smoothed_roll + 0.40 * roll
        self.smoothed_pitch = 0.60 * self.smoothed_pitch + 0.40 * pitch
        report = HoldReport(
            enabled=True, state="TRACKING",
            roll=self.smoothed_roll, pitch=self.smoothed_pitch,
            shift_x=dx, scale=scale, inliers=inliers,
            note="Experimental landmark stabilization; not 3-D position lock",
        )
        return report, src, dst

    def pause(self) -> None:
        """Forget untrusted flow and old commands; preserve the fixed reference."""
        self.previous_gray = None
        self.track_reference = None
        self.track_points = None
        self.smoothed_roll = self.smoothed_pitch = 0.0

    def _clamp(self, value: float) -> float:
        return max(-self.max_command, min(self.max_command, value))


class CameraHold:
    """Latest-frame processing; neutral during gaps, disengage on lasting loss."""

    def __init__(self, max_command: float = MAX_ROLL_PITCH):
        self.max_command = max_command
        self.lock = threading.Lock()
        self.frames = queue.Queue(maxsize=1)
        self._enabled = False
        self._generation = 0
        self._enabled_at = 0.0
        self._last_result_at = 0.0
        self._recovery_started_at = 0.0
        self._recovery_matches = 0
        self._latest_frame_at = 0.0
        self._report = HoldReport()
        self._last_queued_at = 0.0
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, name="camera-hold", daemon=True)
        self._worker.start()

    def enable(self, now: float) -> None:
        with self.lock:
            self._generation += 1
            self._enabled = True
            self._enabled_at = now
            self._last_result_at = 0.0
            self._recovery_started_at = 0.0
            self._recovery_matches = 0
            self._last_queued_at = 0.0
            self._latest_frame_at = now
            self._report = HoldReport(True, "ACQUIRING", note="Finding reference landmarks")

    def disable(self, reason: str = "OFF") -> None:
        with self.lock:
            self._disable_locked(reason)

    def _disable_locked(self, reason: str) -> None:
        self._generation += 1
        self._enabled = False
        self._report = HoldReport(False, reason, note=reason)

    def submit(self, frame: tuple[bytes, int, int], stamp: float) -> None:
        with self.lock:
            if not self._enabled or stamp <= self._last_queued_at:
                return
            self._latest_frame_at = stamp
            self._last_queued_at = stamp
            generation = self._generation
        # The worker owns only the newest frame. Drop old frames under load.
        try:
            self.frames.put_nowait((generation, frame, stamp))
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frames.put_nowait((generation, frame, stamp))
            except queue.Full:
                pass

    def snapshot(self, now: float) -> HoldReport:
        with self.lock:
            self._expire_locked(now)
            return self._report

    def _expire_locked(self, now: float) -> None:
        if not self._enabled:
            return
        if now - self._latest_frame_at > FRAME_MAX_AGE:
            self._disable_locked("VIDEO STALE: HOLD OFF")
        elif not self._last_result_at and now - self._enabled_at > ACQUIRE_TIMEOUT:
            self._disable_locked("NO CAMERA LOCK: HOLD OFF")
        elif self._recovery_started_at and now - self._recovery_started_at > RECOVERY_TIMEOUT:
            self._disable_locked("CAMERA LOCK LOST: HOLD OFF")
        elif self._last_result_at and now - self._last_result_at > RESULT_MAX_AGE:
            self._disable_locked("TRACKING STALE: HOLD OFF")

    def close(self) -> None:
        self._stop.set()
        self._worker.join(timeout=1.0)

    def _run(self) -> None:
        estimator = None
        generation = -1
        last_processed_at = 0.0
        while not self._stop.is_set():
            try:
                queued_gen, frame, stamp = self.frames.get(timeout=0.1)
            except queue.Empty:
                continue
            with self.lock:
                if not self._enabled or queued_gen != self._generation:
                    continue
            if generation != queued_gen:
                estimator = FeatureAnchor(max_command=self.max_command)
                generation = queued_gen
                last_processed_at = 0.0
            if stamp - last_processed_at < ANALYSIS_INTERVAL:
                continue
            last_processed_at = stamp
            try:
                if time.monotonic() - stamp > FRAME_MAX_AGE:
                    raise TrackingFailure("VIDEO FRAME TOO OLD")
                gray = estimator.grayscale(frame)
                if estimator.anchor_desc is None:
                    n = estimator.anchor(gray)
                    report = HoldReport(True, "ACQUIRING", inliers=n,
                                        note="Reference acquired; checking motion")
                else:
                    report = estimator.estimate(gray)
                if time.monotonic() - stamp > FRAME_MAX_AGE:
                    raise TrackingFailure("VIDEO PROCESSING TOO SLOW")
            except (MotionLimitExceeded, cv2.error, ValueError, OverflowError) as exc:
                with self.lock:
                    if self._enabled and queued_gen == self._generation:
                        self._disable_locked("VISUAL HOLD OFF: " + str(exc)[:48])
                continue
            except TrackingFailure as exc:
                # A blur/compression glitch is not proof that the target moved.
                # Retry the ORIGINAL reference with zero output; never fly on
                # the last correction or silently re-anchor at a new position.
                estimator.pause()
                now = time.monotonic()
                with self.lock:
                    if not self._enabled or queued_gen != self._generation:
                        continue
                    if self._last_result_at:
                        if not self._recovery_started_at:
                            self._recovery_started_at = now
                        self._recovery_matches = 0
                        self._report = HoldReport(True, "RECOVERING",
                                                  note="Neutral axes: " + str(exc))
                    else:
                        self._report = HoldReport(True, "ACQUIRING", note=str(exc))
                    self._expire_locked(now)
                continue
            with self.lock:
                self._expire_locked(time.monotonic())
                if self._enabled and queued_gen == self._generation:
                    if report.state == "TRACKING":
                        self._last_result_at = stamp  # age the observation, not its processing time
                        if self._recovery_started_at:
                            self._recovery_matches += 1
                            if self._recovery_matches < RECOVERY_CONFIRMATIONS:
                                report = HoldReport(True, "RECOVERING", inliers=report.inliers,
                                                    note="Reference found; confirming camera lock")
                            else:
                                self._recovery_started_at = 0.0
                    self._report = report

