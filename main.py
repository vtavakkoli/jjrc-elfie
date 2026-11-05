#!/usr/bin/env python3
"""JJRC H37 / Eachine Elfie keyboard pilot with an integrated FPV window.

Connect your computer to the drone Wi-Fi, then run: python main.py
Commands use the original 8-byte UDP protocol; video is raw H.264 over TCP.
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


VIDEO_HELLO = bytes.fromhex("495464000000580000009bf89049c926884d4f922b3b33ba7eceacef63f77157ab2f53e3f768ecd9e18547b8c22e21d01bfb6b3de3"    
                            "25a27b8fb3acef63f77157ab2f53e3f768ecd9e185eb20be383aab05a8c2a71f2c906d93f72a85e7356effe1b8f5af097f9147f87e"
                           )
DARK = (13, 19, 29)
PANEL = (22, 32, 47)
WHITE = (230, 239, 247)
MUTED = (146, 165, 183)
GREEN = (72, 220, 155)
RED = (250, 105, 105)
BLUE = (80, 178, 248)
AMBER = (255, 195, 90)
IDLE_PACKET = bytes.fromhex("6680800180008199")
HOLD_PACKET = bytes.fromhex("6680808080000099")



def packet(roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0, command=0):
    """Original 0x66 + 4 axes + command + XOR + 0x99 flight packet."""
    def axis(value):
        return round((max(-1.0, min(1.0, value)) + 1) * 127.5)

    values = [axis(roll), axis(pitch), axis(throttle), axis(yaw), command]
    checksum = 0
    for value in values:
        checksum ^= value
    return bytes([0x66, *values, checksum, 0x99])


class FlightLink:
    """Unconnected UDP transport, explicitly bound for Windows/Wi-Fi stability.

    sendto() avoids a stale 'connected' UDP socket after Windows changes routes.
    A successful send only confirms local transmission, not drone reception.
    """

    def __init__(self, host, port, demo=False, local_ip=None, shutdown_packet=HOLD_PACKET):
        self.address = (host, port)
        self.shutdown_packet = shutdown_packet
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


def draw_text(surface, font, value, position, color=WHITE):
    surface.blit(font.render(value, True, color), position)


def draw_pill(surface, font, text, x, y, active=False):
    label = font.render(text, True, DARK if active else MUTED)
    rect = pygame.Rect(x, y, label.get_width() + 20, 29)
    pygame.draw.rect(surface, GREEN if active else PANEL, rect, border_radius=7)
    surface.blit(label, (x + 10, y + 6))
    return rect.right + 8


def main():
    parser = argparse.ArgumentParser(description="JJRC H37 Elfie keyboard FPV controller")
    parser.add_argument("--host", default="172.16.10.1")
    parser.add_argument("--protocol", choices=("android", "h37"), default="android",
                        help="Original Android 11-byte controls (default), or legacy H37 8-byte controls")
    parser.add_argument("--control-port", type=int, default=None,
                        help="UDP destination (Android: 8080; H37: 8895)")
    parser.add_argument("--video-port", type=int, default=8888)
    parser.add_argument("--no-video", action="store_true",
                        help="Disable video connection for flight-packet testing")
    parser.add_argument("--firmware-version", default="",
                        help="Android firmware version, e.g. V2.4.0 (enables documented direction correction)")
    parser.add_argument("--device-type", default="",
                        help="Android device type, e.g. UDP_720")
    parser.add_argument("--reverse-direction", action="store_true",
                        help="Explicitly reverse Android yaw and roll fields")
    parser.add_argument("--trim-yaw", type=int, default=16, choices=range(33))
    parser.add_argument("--trim-pitch", type=int, default=16, choices=range(33))
    parser.add_argument("--trim-roll", type=int, default=16, choices=range(33))
    parser.add_argument("--altitude-hold", action="store_true",
                        help="Start original Android protocol with altitude hold enabled")
    parser.add_argument("--stick-gain", type=float, default=0.75,
                        help="Keyboard stick deflection for Android controls (0.1 to 1.0, default 0.75)")
    parser.add_argument("--local-ip", help="Bind UDP to this Wi-Fi IPv4 address (default: auto from video connection)")
    parser.add_argument("--demo", action="store_true", help="Preview interface without sending commands")
    args = parser.parse_args()
    is_android = args.protocol == "android"
    if not 0.1 <= args.stick_gain <= 1.0:
        parser.error("--stick-gain must be between 0.1 and 1.0")
    if args.control_port is None:
        args.control_port = ANDROID_CONTROL_PORT if is_android else 8895
    reverse_direction = args.reverse_direction or reverse_for_firmware(
        args.device_type, args.firmware_version
    )
    send_period = ANDROID_SEND_PERIOD if is_android else 0.05

    pygame.init()
    screen = pygame.display.set_mode((1100, 760), pygame.RESIZABLE)
    pygame.display.set_caption("ELFIE FPV  |  Keyboard Flight Deck")
    title_font = pygame.font.SysFont("segoeui", 29, bold=True)
    font = pygame.font.SysFont("segoeui", 19)
    small = pygame.font.SysFont("consolas", 16)
    tiny = pygame.font.SysFont("consolas", 14)
    clock = pygame.time.Clock()

    flight = FlightLink(
        args.host, args.control_port, demo=args.demo, local_ip=args.local_ip,
        shutdown_packet=android_packet() if is_android else HOLD_PACKET,
    )
    video = VideoLink(args.host, args.video_port, demo=(args.demo or args.no_video))
    speed = 0.30  # Legacy H37 analog gain (independent of Android speed mode).
    stick_gain = args.stick_gain  # Android keyboard displacement into 0..127 sticks.
    # Android app never limits joystick deflection to 30% at speed mode 0.
    speed_mode = 0
    altitude_hold = args.altitude_hold if is_android else True
    headless = False
    manual_throttle = 0.0
    last_input_time = time.monotonic()
    last_send = 0.0
    start_time = time.monotonic()
    command_until = 0.0
    active_command = 0
    running = True
    armed_requested = False
    focused = True
    mode = "STARTING"
    notice = ("ORIGINAL ANDROID / UDP 8080. T TAKEOFF (enables ALT). "
              "PROPELLERS OFF for first test. No command ACK."
              if is_android else
              "H37 startup 10s. Remove propellers for first test. No command ACK.")
    flight.open()
    video.start()

    try:
        while running:
            now = time.monotonic()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.WINDOWFOCUSLOST:
                    focused = False
                elif event.type == pygame.WINDOWFOCUSGAINED:
                    focused = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_x:
                        # Android bit 5; H37 command 0x04. Cuts motors in flight.
                        active_command, command_until = 4, now + 1.0
                        armed_requested = False
                        manual_throttle = 0.0
                        mode = "EMERGENCY MOTOR STOP REQUESTED"
                    elif event.key == pygame.K_l and now >= command_until:
                        active_command, command_until = 2, now + 1.0
                        armed_requested = False
                        manual_throttle = 0.0
                        mode = "LAND REQUESTED (NOT ACKNOWLEDGED)"
                    elif event.key == pygame.K_t and now >= command_until:
                        if is_android:
                            # Original app exposes takeoff only in atmospheric mode.
                            altitude_hold = True
                            manual_throttle = 0.0
                        if is_android or now - start_time >= 10.0:
                            active_command, command_until = 1, now + 1.0
                            armed_requested = True
                            mode = "ANDROID TAKEOFF REQUESTED" if is_android else "H37 ARM REQUESTED"
                    elif event.key == pygame.K_h and is_android and not armed_requested:
                        altitude_hold = not altitude_hold
                        manual_throttle = 0.0
                        mode = "ALTITUDE HOLD ON" if altitude_hold else "MANUAL THROTTLE ON"
                    elif event.key == pygame.K_n and is_android:
                        headless = not headless
                    elif event.key == pygame.K_c and is_android and not armed_requested and now >= command_until:
                        # Original Android calibration: flag in packet byte 6.
                        active_command, command_until = 8, now + 1.0
                        mode = "CALIBRATION REQUESTED"
                    # Support physical PLUS keys on international keyboards,
                    # including German/Austrian layouts where '+' is not '='.
                    elif (event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS,
                                        getattr(pygame, "K_PLUS", -1))
                          or getattr(event, "unicode", "") == "+"):
                        if is_android:
                            speed_mode = (speed_mode + 1) % 3
                            mode = "ANDROID SPEED MODE {} / 2".format(speed_mode)
                        else:
                            speed = min(0.8, round(speed + 0.05, 2))
                    elif (event.key in (pygame.K_MINUS, pygame.K_KP_MINUS)
                          or getattr(event, "unicode", "") == "-"):
                        if is_android:
                            speed_mode = (speed_mode - 1) % 3
                            mode = "ANDROID SPEED MODE {} / 2".format(speed_mode)
                        else:
                            speed = max(0.1, round(speed - 0.05, 2))
                    elif event.key == pygame.K_SPACE and is_android and not altitude_hold:
                        manual_throttle = 0.0

            frame, frame_time, video_status, video_local_ip = video.snapshot()
            # TCP video already found the correct route to the drone. Use its IPv4
            # Wi-Fi interface for UDP too, rather than the Windows default route.
            if not args.local_ip and video_local_ip and flight.local_ip != video_local_ip:
                flight.open(video_local_ip)

            keys = pygame.key.get_pressed()
            dt = min(0.05, max(0.0, now - last_input_time))
            last_input_time = now
            if is_android and not altitude_hold:
                # Android's manual left throttle stick does not self-center.
                # Keyboard Up/Down therefore changes an absolute throttle latch.
                if not focused or keys[pygame.K_SPACE]:
                    manual_throttle = 0.0
                elif now >= command_until:
                    step = int(keys[pygame.K_UP]) - int(keys[pygame.K_DOWN])
                    manual_throttle = max(0.0, min(1.0, manual_throttle + step * dt * 0.35))
            controls_enabled = (focused and now >= command_until
                                and (armed_requested or (is_android and not altitude_hold)))
            if controls_enabled and not keys[pygame.K_SPACE]:
                # Android SendThread sends raw RockerView values (0..127).
                # Keyboard presses must have enough stick travel to be distinct
                # from center; firmware speed remains in packet byte 9 bits 0..1.
                direction_gain = stick_gain if is_android else speed
                roll = direction_gain * (int(keys[pygame.K_d]) - int(keys[pygame.K_a]))
                pitch = direction_gain * (int(keys[pygame.K_w]) - int(keys[pygame.K_s]))
                yaw = direction_gain * (int(keys[pygame.K_RIGHT]) - int(keys[pygame.K_LEFT]))
                throttle = (manual_throttle if is_android and not altitude_hold else
                            speed * (int(keys[pygame.K_UP]) - int(keys[pygame.K_DOWN])))
            else:
                roll = pitch = throttle = yaw = 0.0

            if now - last_send >= send_period:
                if is_android:
                    command_active = now < command_until
                    data = android_packet(
                        roll, pitch, throttle, yaw,
                        altitude_hold=altitude_hold, speed_mode=speed_mode,
                        headless=headless,
                        calibrate=command_active and active_command == 8,
                        emergency_stop=command_active and active_command == 4,
                        takeoff=command_active and active_command == 1,
                        land=command_active and active_command == 2,
                        trim_yaw=args.trim_yaw, trim_pitch=args.trim_pitch,
                        trim_roll=args.trim_roll,
                        reverse_direction=reverse_direction,
                    )
                    if not command_active:
                        active_command = 0
                        if mode == "STARTING":
                            mode = "ANDROID READY / T TAKEOFF / H ALT-HOLD"
                elif now - start_time < 5.0:
                    data = IDLE_PACKET
                    mode = "INITIALIZING IDLE  /  {}s".format(max(0, int(5 - (now - start_time))))
                elif now - start_time < 10.0:
                    data = HOLD_PACKET
                    mode = "INITIALIZING ALT-HOLD  /  {}s".format(max(0, int(10 - (now - start_time))))
                elif now < command_until:
                    data = packet(command=active_command)
                else:
                    active_command = 0
                    data = packet(roll, pitch, throttle, yaw)
                    if mode.startswith("INITIALIZING"):
                        mode = "READY: PRESS T TO ARM"
                flight.send(data)
                last_send = now

            width, height = screen.get_size()
            screen.fill(DARK)
            draw_text(screen, title_font, "ELFIE  /  FPV FLIGHT DECK", (24, 12))
            draw_text(screen, small, "SIMULATION" if args.demo else "{}  [{}]".format(args.host, args.protocol.upper()), (max(24, width - 205), 27), BLUE)
            video_bottom = max(240, height - 228)
            viewport = pygame.Rect(24, 66, max(200, width - 48), video_bottom - 66)
            pygame.draw.rect(screen, PANEL, viewport, border_radius=12)

            if frame is not None and now - frame_time < 3.0:
                pixels, fw, fh = frame
                image = pygame.image.frombuffer(pixels, (fw, fh), "RGB")
                scale = min(viewport.width / fw, viewport.height / fh)
                image = pygame.transform.scale(image, (int(fw * scale), int(fh * scale)))
                screen.blit(image, image.get_rect(center=viewport.center))
            else:
                draw_text(screen, title_font, "WAITING FOR LIVE VIDEO", (viewport.x + 32, viewport.centery - 25), MUTED)
                if args.demo:
                    draw_text(screen, font, "Demo mode: no commands sent to hardware",
                              (viewport.x + 32, viewport.centery + 20), BLUE)

            draw_text(screen, small, "VIDEO  " + video_status[:44],
                      (viewport.x + 16, viewport.y + 14),
                      GREEN if video_status == "STREAMING" else MUTED)
            draw_text(screen, small, mode, (viewport.x + 16, viewport.bottom - 28),
                      RED if active_command == 4 else (AMBER if armed_requested else MUTED))

            y = viewport.bottom + 13
            draw_text(screen, tiny, "LIVE KEY INPUT", (28, y + 8), MUTED)
            x = 167
            for name, key in [("W", pygame.K_w), ("A", pygame.K_a), ("S", pygame.K_s),
                              ("D", pygame.K_d), ("UP", pygame.K_UP), ("DN", pygame.K_DOWN),
                              ("LT", pygame.K_LEFT), ("RT", pygame.K_RIGHT)]:
                x = draw_pill(screen, tiny, name, x, y,
                              focused and keys[key] and not keys[pygame.K_SPACE])
            draw_text(screen, small,
                      "W/S FORWARD/BACK    A/D LEFT/RIGHT    UP/DOWN ASCEND/DESCEND    LEFT/RIGHT TURN",
                      (28, y + 43))
            draw_text(screen, small,
                      ("T TAKEOFF  L LAND  X STOP  H ALT-HOLD  N HEADLESS  C CALIBRATE  +/- MODE  ESC QUIT"
                       if is_android else
                       "T ARM    L LAND    X EMERGENCY STOP    SPACE NEUTRAL    +/- SPEED    ESC QUIT"),
                      (28, y + 75))
            draw_text(screen, small,
                      ("ANDROID SPEED {}  STICK {:2.0f}%  {}  THR {:+.2f}  R {:+.2f}  P {:+.2f}  Y {:+.2f}".format(
                          speed_mode, stick_gain * 100,
                          "ALT-HOLD" if altitude_hold else "MANUAL",
                          throttle, roll, pitch, yaw) if is_android else
                       "GAIN {:2.0f}%   ROLL {:+.2f}   PITCH {:+.2f}   ALT {:+.2f}   YAW {:+.2f}".format(
                          speed * 100, roll, pitch, throttle, yaw)),
                      (28, y + 108), BLUE)
            if flight.last_error:
                network = "CONTROL ERROR: " + flight.last_error
                network_color = RED
            elif flight.sent_count:
                network = "UDP TX {} (UNCONFIRMED)   {} -> {}:{}   FAILURES {}".format(
                    flight.sent_count, flight.local_ip, args.host, args.control_port, flight.error_count)
                network_color = GREEN
            else:
                network = "UDP: WAITING FOR TRANSMISSION"
                network_color = AMBER
            draw_text(screen, tiny, network[:130], (28, y + 140), network_color)
            hint = "FOCUS LOST - NEUTRAL INPUTS" if not focused else notice
            if is_android and not altitude_hold:
                hint = "MANUAL THROTTLE: UP/DOWN LATCH, SPACE ZERO. T switches to altitude hold."
            draw_text(screen, tiny, hint[:135], (28, y + 167),
                      AMBER if not focused else MUTED)
            draw_text(screen, tiny, "LAST UDP: " + flight.last_sent_hex.upper(),
                      (max(28, width - 270), y + 195), MUTED)
            pygame.display.flip()
            clock.tick(60)
    finally:
        video.close()
        flight.close()
        pygame.quit()


if __name__ == "__main__":
    main()
