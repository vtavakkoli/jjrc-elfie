#!/usr/bin/env python3
"""JJRC Elfie flight deck: original working Android protocol and experimental F hold.

Run: python main.py
Only the 11-byte FF 08 Android protocol (UDP 8080) is supported.
The optional visual hold is not a true 3-D position or altitude lock.
"""
from __future__ import annotations

import argparse
import socket
import threading
import time

import av
import pygame

from android_protocol import (
    ANDROID_CONTROL_PORT, ANDROID_SEND_PERIOD, android_packet,
    reverse_for_firmware,
)
from vision_hold import CameraHold, FRAME_MAX_AGE

VIDEO_HELLO = bytes.fromhex(
    "495464000000580000009bf89049c926884d4f922b3b33ba7eceacef63f77157ab2f53e3f768ecd9e18547b8c22e21d01bfb6b3de3"
    "25a27b8fb3acef63f77157ab2f53e3f768ecd9e185eb20be383aab05a8c2a71f2c906d93f72a85e7356effe1b8f5af097f9147f87e"
)
BG = (12, 18, 30)
PANEL = (21, 31, 48)
PANEL_ALT = (29, 43, 61)
WHITE = (229, 238, 246)
MUTED = (144, 163, 185)
GREEN = (83, 219, 164)
CYAN = (77, 188, 247)
RED = (248, 104, 112)
AMBER = (252, 194, 95)
LINE = (48, 66, 87)


class FlightLink:
    """Unconnected UDP transport, explicitly bound for Windows/Wi-Fi stability.

    sendto() avoids a stale 'connected' UDP socket after Windows changes routes.
    A successful send only confirms local transmission, not drone reception.
    """

    def __init__(self, host, port, demo=False, local_ip=None, shutdown_packet=None):
        self.address = (host, port)
        self.shutdown_packet = shutdown_packet or android_packet(altitude_hold=True)
        self.demo = demo
        self.preferred_ip = local_ip
        self.local_ip = local_ip or "0.0.0.0"
        self.sock = None
        self.last_error = ""
        self.sent_count = 0
        self.error_count = 0
        self.last_sent_hex = "-"
        self.next_retry = 0.0

    def open(self, local_ip=None):
        if self.demo:
            return True
        target_ip = local_ip or self.local_ip
        if self.sock is not None and target_ip == self.local_ip:
            return True
        if self.sock is None and time.monotonic() < self.next_retry:
            return False
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind((target_ip, 0))  # Bind explicitly; port remains ephemeral.
            except OSError:
                sock.close()
                raise
            self.sock = sock
            self.local_ip = target_ip
            self.last_error = ""
            return True
        except OSError as exc:
            self.last_error = f"UDP bind {target_ip}: {exc}"
            self.next_retry = time.monotonic() + 1.0
            return False

    def send(self, data):
        if self.demo:
            self.last_sent_hex = data.hex()
            self.sent_count += 1
            return True
        if self.sock is None and time.monotonic() >= self.next_retry:
            self.open()
        if self.sock is None:
            return False
        try:
            num_sent = self.sock.sendto(data, self.address)
            if num_sent != len(data):
                raise OSError(f"short UDP write: {num_sent}/{len(data)}")
        except OSError as exc:
            self.last_error = f"UDP send: {exc}"
            self.error_count += 1
            self.sock.close()
            self.sock = None
            self.next_retry = time.monotonic() + 1.0
            return False
        self.last_error = ""
        self.last_sent_hex = data.hex()
        self.sent_count += 1
        return True

    def close(self):
        # Neutral command only: not an automatic land or a safety guarantee.
        if self.sock is not None:
            try:
                self.sock.sendto(self.shutdown_packet, self.address)
            except OSError:
                pass
            self.sock.close()
            self.sock = None


class VideoLink:
    """Background receiver: never stall keyboard input while reading TCP."""
    def __init__(self, host, port, demo=False):
        self.host, self.port, self.demo = host, port, demo
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.status = "DEMO" if demo else "CONNECTING"
        self.local_ip = None
        self.thread = None
        self.connection = None

    def start(self):
        if not self.demo:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.frame, self.frame_time, self.status, self.local_ip

    def _run(self):
        while not self.stop_event.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=3) as conn:
                    self.connection = conn
                    with self.lock:
                        self.local_ip = conn.getsockname()[0]
                    conn.settimeout(1.0)
                    conn.sendall(VIDEO_HELLO)
                    # Original Elfie stream begins with a 106-byte response header.
                    remaining = 106
                    while remaining and not self.stop_event.is_set():
                        chunk = conn.recv(remaining)
                        if not chunk:
                            raise ConnectionError("Video handshake closed")
                        remaining -= len(chunk)
                    decoder = av.CodecContext.create("h264", "r")
                    self.status = "STREAMING"
                    while not self.stop_event.is_set():
                        try:
                            raw = conn.recv(16384)
                        except socket.timeout:
                            continue
                        if not raw:
                            raise ConnectionError("Video stream ended")
                        for parsed in decoder.parse(raw):
                            try:
                                frames = decoder.decode(parsed)
                            except av.FFmpegError:
                                continue
                            for frame in frames:
                                # Convert off the UI thread, keeping only the newest frame.
                                rgb = frame.to_ndarray(format="rgb24")
                                with self.lock:
                                    self.frame = (rgb.tobytes(), frame.width, frame.height)
                                    self.frame_time = time.monotonic()
            except (OSError, av.FFmpegError, ConnectionError) as exc:
                with self.lock:
                    self.status = "RECONNECTING: " + str(exc)[:50]
            finally:
                self.connection = None
            self.stop_event.wait(1.5)

    def close(self):
        self.stop_event.set()
        if self.connection is not None:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.thread is not None:
            self.thread.join(timeout=2)



def draw_text(surface, font, value, xy, color=WHITE):
    surface.blit(font.render(str(value), True, color), xy)


def panel(surface, rect, fill=PANEL, radius=12):
    pygame.draw.rect(surface, fill, rect, border_radius=radius)


def pill(surface, font, xy, label, active=False, danger=False):
    ink = RED if danger else GREEN if active else MUTED
    text = font.render(label, True, ink)
    box = pygame.Rect(xy[0], xy[1], text.get_width() + 22, 29)
    pygame.draw.rect(surface, PANEL_ALT, box, border_radius=8)
    pygame.draw.rect(surface, ink if active or danger else LINE, box, 1, border_radius=8)
    surface.blit(text, (box.x + 11, box.y + 6))
    return box.right


def render_view(screen, frame, frame_time, now, viewport, font, small, hold_state):
    panel(screen, viewport, PANEL)
    fresh = frame is not None and now - frame_time < 1.0
    if fresh:
        pixels, fw, fh = frame
        try:
            img = pygame.image.frombuffer(pixels, (fw, fh), "RGB")
            scale = min(viewport.width / fw, viewport.height / fh)
            scaled = pygame.transform.scale(img, (max(1, int(fw * scale)), max(1, int(fh * scale))))
            screen.blit(scaled, scaled.get_rect(center=viewport.center))
        except (ValueError, pygame.error):
            fresh = False
    if not fresh:
        draw_text(screen, font, "WAITING FOR CAMERA VIDEO", (viewport.x + 26, viewport.centery - 20), MUTED)
        draw_text(screen, small, "Manual control remains available. F requires live video.",
                  (viewport.x + 26, viewport.centery + 21), MUTED)
    # Crosshair is an optical reference indicator, not a flight/altitude guarantee.
    c = viewport.center
    pygame.draw.circle(screen, CYAN if hold_state == "TRACKING" else MUTED, c, 22, 1)
    pygame.draw.line(screen, CYAN if hold_state == "TRACKING" else MUTED,
                     (c[0] - 31, c[1]), (c[0] - 9, c[1]), 2)
    pygame.draw.line(screen, CYAN if hold_state == "TRACKING" else MUTED,
                     (c[0] + 9, c[1]), (c[0] + 31, c[1]), 2)
    pygame.draw.line(screen, CYAN if hold_state == "TRACKING" else MUTED,
                     (c[0], c[1] - 31), (c[0], c[1] - 9), 2)
    pygame.draw.line(screen, CYAN if hold_state == "TRACKING" else MUTED,
                     (c[0], c[1] + 9), (c[0], c[1] + 31), 2)
    return fresh


def parse_args():
    p = argparse.ArgumentParser(description="JJRC Elfie - original Android flight deck")
    p.add_argument("--host", default="172.16.10.1", help="Drone Wi-Fi gateway address")
    p.add_argument("--control-port", type=int, default=ANDROID_CONTROL_PORT)
    p.add_argument("--video-port", type=int, default=8888)
    p.add_argument("--local-ip", default=None, help="Explicit local Wi-Fi IPv4 bind")
    p.add_argument("--no-video", action="store_true", help="Disable FPV receiver (F unavailable)")
    p.add_argument("--demo", action="store_true", help="No control packets sent to drone")
    p.add_argument("--altitude-hold", action="store_true", help="Enable altitude hold at startup")
    p.add_argument("--device-type", default="")
    p.add_argument("--firmware-version", default="")
    p.add_argument("--reverse-direction", action="store_true")
    p.add_argument("--trim-yaw", type=int, choices=range(33), default=16)
    p.add_argument("--trim-pitch", type=int, choices=range(33), default=16)
    p.add_argument("--trim-roll", type=int, choices=range(33), default=16)
    p.add_argument("--stick-gain", type=float, default=0.75,
                   help="Manual W/S/A/D/yaw deflection, 0.1 - 1.0")
    p.add_argument("--hold-max-command", type=float, default=0.22,
                   help="Camera F correction limit, 0.05 - 0.35 (experimental)")
    args = p.parse_args()
    if not 0.1 <= args.stick_gain <= 1.0:
        p.error("--stick-gain must be between 0.1 and 1.0")
    if not 0.05 <= args.hold_max_command <= 0.35:
        p.error("--hold-max-command must be between 0.05 and 0.35")
    return args


def main():
    args = parse_args()
    pygame.init()
    screen = pygame.display.set_mode((1180, 790), pygame.RESIZABLE)
    pygame.display.set_caption("ELFIE  |  Original Android Flight Deck")
    title_font = pygame.font.SysFont("segoeui", 30, bold=True)
    medium = pygame.font.SysFont("segoeui", 20, bold=True)
    body = pygame.font.SysFont("segoeui", 17)
    mono = pygame.font.SysFont("consolas", 15)
    small = pygame.font.SysFont("consolas", 13)
    clock = pygame.time.Clock()

    flight = FlightLink(args.host, args.control_port, demo=args.demo,
                        local_ip=args.local_ip, shutdown_packet=android_packet(altitude_hold=True))
    video = VideoLink(args.host, args.video_port, demo=(args.demo or args.no_video))
    hold = CameraHold(max_command=args.hold_max_command)
    reverse_direction = (args.reverse_direction or
                         reverse_for_firmware(args.device_type, args.firmware_version))
    altitude_hold = args.altitude_hold
    armed_requested = False
    headless = False
    speed_mode = 0
    manual_throttle = 0.0
    active_command = 0
    command_until = 0.0
    last_input_time = time.monotonic()
    last_send = 0.0
    focused = True
    running = True
    message = "F: experimental visual hold. Built-in altitude hold controls height."
    flight.open()
    video.start()

    try:
        while running:
            now = time.monotonic()
            frame, frame_time, video_status, video_local_ip = video.snapshot()
            video_fresh = (frame is not None and now - frame_time < FRAME_MAX_AGE
                           and video_status == "STREAMING")

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.WINDOWFOCUSLOST:
                    focused = False
                    hold.disable("FOCUS LOST - HOLD OFF")
                elif event.type == pygame.WINDOWFOCUSGAINED:
                    focused = True
                elif event.type == pygame.KEYDOWN and not event.repeat if hasattr(event, "repeat") else event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_x:
                        hold.disable("EMERGENCY STOP - HOLD OFF")
                        active_command, command_until = 4, now + 1.0
                        armed_requested = False
                        manual_throttle = 0.0
                        message = "EMERGENCY MOTOR STOP REQUESTED - LOSS OF LIFT"
                    elif event.key == pygame.K_l and now >= command_until:
                        hold.disable("LANDING - HOLD OFF")
                        active_command, command_until = 2, now + 1.0
                        armed_requested = False
                        manual_throttle = 0.0
                        message = "LANDING REQUESTED (NOT ACKNOWLEDGED)"
                    elif event.key == pygame.K_t and now >= command_until:
                        hold.disable("TAKEOFF - HOLD OFF")
                        altitude_hold = True
                        manual_throttle = 0.0
                        active_command, command_until = 1, now + 1.0
                        armed_requested = True
                        message = "TAKEOFF REQUESTED: ALTITUDE HOLD ENABLED"
                    elif event.key == pygame.K_f:
                        if hold.snapshot(now).enabled:
                            hold.disable("F: HOLD OFF - MANUAL CONTROL")
                            message = "Camera hold OFF"
                        elif not armed_requested or now < command_until:
                            message = "F requires completed takeoff (T) first"
                        elif not altitude_hold:
                            message = "F requires built-in altitude-hold mode"
                        elif not focused or not video_fresh or args.no_video or args.demo:
                            message = "F unavailable: reliable live camera stream required"
                        else:
                            hold.enable(now)
                            hold.submit(frame, frame_time)
                            message = "F acquiring visual landmarks; neutral controls until lock"
                    elif event.key == pygame.K_h and not armed_requested:
                        hold.disable("MODE CHANGED - HOLD OFF")
                        altitude_hold = not altitude_hold
                        manual_throttle = 0.0
                        message = "ALTITUDE HOLD ON" if altitude_hold else "MANUAL THROTTLE"
                    elif event.key == pygame.K_n:
                        hold.disable("HEADLESS TOGGLE - HOLD OFF")
                        headless = not headless
                        message = "HEADLESS ON" if headless else "HEADLESS OFF"
                    elif event.key == pygame.K_c and not armed_requested and now >= command_until:
                        hold.disable("CALIBRATION - HOLD OFF")
                        active_command, command_until = 8, now + 1.0
                        message = "CALIBRATION REQUESTED - KEEP DRONE STATIONARY"
                    elif (event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS,
                                        getattr(pygame, "K_PLUS", -1))
                          or getattr(event, "unicode", "") == "+"):
                        hold.disable("SPEED CHANGED - HOLD OFF")
                        speed_mode = (speed_mode + 1) % 3
                        message = "ANDROID SPEED MODE {} / 2".format(speed_mode)
                    elif (event.key in (pygame.K_MINUS, pygame.K_KP_MINUS)
                          or getattr(event, "unicode", "") == "-"):
                        hold.disable("SPEED CHANGED - HOLD OFF")
                        speed_mode = (speed_mode - 1) % 3
                        message = "ANDROID SPEED MODE {} / 2".format(speed_mode)
                    elif event.key == pygame.K_SPACE:
                        hold.disable("SPACE - HOLD OFF")
                        if not altitude_hold:
                            manual_throttle = 0.0

            if not args.local_ip and video_local_ip and flight.local_ip != video_local_ip:
                flight.open(video_local_ip)
            if not video_fresh and hold.snapshot(now).enabled:
                hold.disable("VIDEO LOST - HOLD OFF")
                message = "VIDEO LOST: CAMERA HOLD DISENGAGED"
            if flight.last_error and hold.snapshot(now).enabled:
                hold.disable("UDP ERROR - HOLD OFF")
                message = "UDP ERROR: CAMERA HOLD DISENGAGED"

            keys = pygame.key.get_pressed()
            move_keys = (pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d,
                         pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT)
            manual_override = keys[pygame.K_SPACE] or any(keys[k] for k in move_keys)
            if manual_override and hold.snapshot(now).enabled:
                hold.disable("MANUAL OVERRIDE - HOLD OFF")
                message = "CAMERA HOLD OFF - MANUAL PILOT OVERRIDE"
            dt = min(0.05, max(0.0, now - last_input_time))
            last_input_time = now
            if not altitude_hold:
                if not focused or keys[pygame.K_SPACE]:
                    manual_throttle = 0.0
                elif now >= command_until:
                    step = int(keys[pygame.K_UP]) - int(keys[pygame.K_DOWN])
                    manual_throttle = max(0.0, min(1.0, manual_throttle + step * dt * 0.35))

            if video_fresh:
                hold.submit(frame, frame_time)
            vision = hold.snapshot(now)
            controls_enabled = focused and now >= command_until and (armed_requested or not altitude_hold)
            if not controls_enabled or keys[pygame.K_SPACE]:
                roll = pitch = throttle = yaw = 0.0
            elif vision.enabled and vision.state == "TRACKING":
                # No camera-driven throttle or yaw: altitude hold is firmware-native.
                roll, pitch, throttle, yaw = vision.roll, vision.pitch, 0.0, 0.0
            elif vision.enabled:
                roll = pitch = throttle = yaw = 0.0
            else:
                gain = args.stick_gain
                roll = gain * (int(keys[pygame.K_d]) - int(keys[pygame.K_a]))
                pitch = gain * (int(keys[pygame.K_w]) - int(keys[pygame.K_s]))
                yaw = gain * (int(keys[pygame.K_RIGHT]) - int(keys[pygame.K_LEFT]))
                throttle = (manual_throttle if not altitude_hold else
                            0.30 * (int(keys[pygame.K_UP]) - int(keys[pygame.K_DOWN])))

            if now - last_send >= ANDROID_SEND_PERIOD:
                active = now < command_until
                data = android_packet(
                    roll, pitch, throttle, yaw,
                    altitude_hold=altitude_hold, speed_mode=speed_mode,
                    headless=headless, calibrate=active and active_command == 8,
                    emergency_stop=active and active_command == 4,
                    takeoff=active and active_command == 1,
                    land=active and active_command == 2,
                    trim_yaw=args.trim_yaw, trim_pitch=args.trim_pitch,
                    trim_roll=args.trim_roll, reverse_direction=reverse_direction,
                )
                flight.send(data)
                if not active:
                    active_command = 0
                last_send = now

            w, h = screen.get_size()
            screen.fill(BG)
            draw_text(screen, title_font, "ELFIE", (24, 14), WHITE)
            draw_text(screen, body, "/  FLIGHT DECK", (125, 24), CYAN)
            draw_text(screen, small, "ORIGINAL ANDROID   FF 08   UDP {}   40 Hz".format(args.control_port),
                      (24, 56), MUTED)

            toolbar_x = max(325, w - 390)
            toolbar_y = 18
            toolbar_x = pill(screen, small, (toolbar_x, toolbar_y),
                             "VIDEO LIVE" if video_fresh else "VIDEO WAITING", active=video_fresh) + 7
            pill(screen, small, (toolbar_x, toolbar_y),
                 "F HOLD" if vision.enabled else "F OFF",
                 active=vision.enabled, danger=vision.state.startswith("VISUAL HOLD OFF"))

            viewport = pygame.Rect(24, 91, max(240, w - 48), max(170, h - 329))
            render_view(screen, frame, frame_time, now, viewport, medium, body, vision.state)
            bar_y = viewport.y + 13
            pill(screen, small, (viewport.x + 12, bar_y),
                 "TRACKING {} POINTS".format(vision.inliers) if vision.state == "TRACKING"
                 else vision.state[:36],
                 active=vision.state == "TRACKING",
                 danger=vision.state.startswith(("VISUAL HOLD OFF", "VIDEO STALE", "NO CAMERA")))
            if vision.enabled:
                draw_text(screen, mono,
                          "VISION  X {:+.3f}  SCALE {:.3f}  R {:+.2f}  P {:+.2f}".format(
                              vision.shift_x, vision.scale, vision.roll, vision.pitch),
                          (viewport.x + 17, viewport.bottom - 31), GREEN)

            lower_y = viewport.bottom + 13
            info_h = max(185, h - lower_y - 18)
            left_w = max(270, int((w - 60) * 0.60))
            left = pygame.Rect(24, lower_y, left_w, info_h)
            right = pygame.Rect(left.right + 12, lower_y,
                                max(220, w - left.right - 36), info_h)
            panel(screen, left)
            panel(screen, right)
            draw_text(screen, medium, "FLIGHT CONTROLS", (left.x + 18, left.y + 12), WHITE)
            draw_text(screen, body,
                      "W/S  pitch    A/D  roll    arrows  yaw / altitude",
                      (left.x + 18, left.y + 46), MUTED)
            draw_text(screen, body,
                      "T  takeoff    L  land    X  emergency stop",
                      (left.x + 18, left.y + 74), WHITE)
            draw_text(screen, body,
                      "F  visual hold    H  altitude mode    +/-  speed",
                      (left.x + 18, left.y + 102), CYAN)
            draw_text(screen, small, "SPEED {}   STICK {}%   {}   {}".format(
                speed_mode, round(args.stick_gain * 100),
                "ALT-HOLD" if altitude_hold else "MANUAL THROTTLE",
                "ARM REQUESTED" if armed_requested else "DISARMED/UNKNOWN"),
                (left.x + 18, left.y + 134), GREEN if armed_requested else AMBER)
            draw_text(screen, small, "INPUT  R {:+.2f}  P {:+.2f}  T {:+.2f}  Y {:+.2f}".format(
                roll, pitch, throttle, yaw), (left.x + 18, left.y + 158), MUTED)

            draw_text(screen, medium, "VISUAL ASSIST", (right.x + 18, right.y + 12), WHITE)
            hold_color = GREEN if vision.state == "TRACKING" else AMBER if vision.enabled else MUTED
            draw_text(screen, body, vision.state[:37], (right.x + 18, right.y + 47), hold_color)
            draw_text(screen, small, "F toggles. Movement / SPACE cancels.", (right.x + 18, right.y + 80), MUTED)
            draw_text(screen, small, "No GPS, altitude or 3-D pose estimate.", (right.x + 18, right.y + 107), MUTED)
            draw_text(screen, small, "UDP TX {:,}   errors {}".format(flight.sent_count, flight.error_count),
                      (right.x + 18, right.y + 138), RED if flight.last_error else GREEN)
            draw_text(screen, small, "VIDEO {}".format(video_status[:30]),
                      (right.x + 18, right.y + 164), MUTED)

            # status line; never claim autonomous station keeping.
            footer_y = h - 20
            draw_text(screen, small, (message if focused else "WINDOW UNFOCUSED: NEUTRAL CONTROL / F OFF")[:145],
                      (24, footer_y), RED if not focused or active_command == 4 else AMBER)
            pygame.display.flip()
            clock.tick(60)
    finally:
        hold.disable("SHUTDOWN")
        hold.close()
        video.close()
        flight.close()
        pygame.quit()


if __name__ == "__main__":
    main()
