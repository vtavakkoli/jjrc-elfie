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


VIDEO_HELLO = bytes.fromhex(
    "495464000000580000009bf89049c926884d4f922b3b33ba7eceacef63f77157"
    "ab2f53e3f768ecd9e18547b8c22e21d01bfb6b3de325a27b8fb3acef63f7"
    "7157ab2f53e3f768ecd9e185eb20be383aab05a8c2a71f2c906d93f72a85"
    "e7356effe1b8f5af097f9147f87e"
)
DARK = (13, 19, 29)
PANEL = (22, 32, 47)
WHITE = (230, 239, 247)
MUTED = (146, 165, 183)
GREEN = (72, 220, 155)
RED = (250, 105, 105)
BLUE = (80, 178, 248)


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
    def __init__(self, host, port, demo=False):
        self.demo = demo
        self.address = (host, port)
        self.sock = None
        self.last_error = ""

    def open(self):
        if not self.demo:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.connect(self.address)

    def send(self, data):
        if self.demo:
            return
        try:
            self.sock.send(data)
            self.last_error = ""
        except OSError as exc:
            self.last_error = str(exc)

    def close(self):
        if self.sock is not None:
            try:
                self.send(packet())
            finally:
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
        self.thread = None
        self.connection = None

    def start(self):
        if not self.demo:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.frame, self.frame_time, self.status

    def _run(self):
        while not self.stop_event.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=3) as conn:
                    self.connection = conn
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


def main():
    parser = argparse.ArgumentParser(description="JJRC Elfie keyboard FPV controller")
    parser.add_argument("--host", default="172.16.10.1")
    parser.add_argument("--control-port", type=int, default=8895)
    parser.add_argument("--video-port", type=int, default=8888)
    parser.add_argument("--demo", action="store_true", help="UI preview without a drone")
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((1100, 720), pygame.RESIZABLE)
    pygame.display.set_caption("ELFIE FPV  |  Keyboard Flight Deck")
    title_font = pygame.font.SysFont("segoeui", 29, bold=True)
    font = pygame.font.SysFont("segoeui", 19)
    small = pygame.font.SysFont("consolas", 16)
    clock = pygame.time.Clock()

    flight = FlightLink(args.host, args.control_port, demo=args.demo)
    video = VideoLink(args.host, args.video_port, demo=args.demo)
    speed = 0.35
    last_send = 0.0
    command_until = 0.0
    active_command = 0
    running = True
    armed = False
    focused = True
    warning = "Connect to the drone Wi-Fi before flight. Test without propellers first."
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
                    elif event.key == pygame.K_t and not event.repeat:
                        active_command, command_until = 1, now + 0.35
                        armed = True
                    elif event.key == pygame.K_x:
                        active_command, command_until = 4, now + 0.6
                        armed = False
                    elif event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS):
                        speed = min(0.8, round(speed + 0.05, 2))
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        speed = max(0.1, round(speed - 0.05, 2))

            keys = pygame.key.get_pressed()
            if focused and now >= command_until and not keys[pygame.K_SPACE]:
                roll = speed * (int(keys[pygame.K_d]) - int(keys[pygame.K_a]))
                pitch = speed * (int(keys[pygame.K_w]) - int(keys[pygame.K_s]))
                throttle = speed * (int(keys[pygame.K_UP]) - int(keys[pygame.K_DOWN]))
                yaw = speed * (int(keys[pygame.K_RIGHT]) - int(keys[pygame.K_LEFT]))
            else:
                roll = pitch = throttle = yaw = 0.0

            command = active_command if now < command_until else 0
            if now >= command_until:
                active_command = 0
            # 20 Hz keepalive, independent of camera reception and rendering.
            if now - last_send >= 0.05:
                flight.send(packet(roll, pitch, throttle, yaw, command))
                last_send = now

            width, height = screen.get_size()
            screen.fill(DARK)
            draw_text(screen, title_font, "ELFIE  /  FPV FLIGHT DECK", (25, 15))
            draw_text(screen, small, "SIMULATION" if args.demo else args.host, (width - 225, 24), BLUE)
            video_bottom = max(250, height - 195)
            viewport = pygame.Rect(24, 68, max(200, width - 48), video_bottom - 68)
            pygame.draw.rect(screen, PANEL, viewport, border_radius=12)

            frame, frame_time, video_status = video.snapshot()
            if frame is not None and now - frame_time < 3:
                pixels, fw, fh = frame
                image = pygame.image.frombuffer(pixels, (fw, fh), "RGB")
                ratio = min(viewport.width / fw, viewport.height / fh)
                image = pygame.transform.scale(image, (int(fw * ratio), int(fh * ratio)))
                target = image.get_rect(center=viewport.center)
                screen.blit(image, target)
            else:
                draw_text(screen, title_font, "WAITING FOR LIVE VIDEO", (viewport.x + 35, viewport.centery - 20), MUTED)
                if args.demo:
                    draw_text(screen, font, "Demo mode: drone commands disabled", (viewport.x + 35, viewport.centery + 22), BLUE)

            draw_text(screen, small, "VIDEO  " + video_status, (viewport.x + 16, viewport.y + 14),
                      GREEN if video_status == "STREAMING" else MUTED)
            state = "ARM COMMAND SENT" if armed else "MOTORS NOT ARMED BY UI"
            draw_text(screen, small, state, (viewport.x + 16, viewport.bottom - 30), GREEN if armed else MUTED)
            y = viewport.bottom + 14
            draw_text(screen, font, "W/S  FORWARD/BACK     A/D  STRAFE     UP/DOWN  ALTITUDE     LEFT/RIGHT  YAW", (28, y))
            draw_text(screen, font, "T  ARM     X  EMERGENCY STOP     SPACE  NEUTRAL     +/-  SPEED     ESC  QUIT", (28, y + 33))
            draw_text(screen, small, "SPEED {:3.0f}%   |   ROLL {:+.2f}   PITCH {:+.2f}   VERTICAL {:+.2f}   YAW {:+.2f}".format(
                speed * 100, roll, pitch, throttle, yaw), (28, y + 71), BLUE)
            status = ("FOCUS LOST: NEUTRAL COMMANDS" if not focused else
                      "CONTROL ERROR: " + flight.last_error if flight.last_error else warning)
            draw_text(screen, small, status[:110], (28, y + 102), RED if (not focused or flight.last_error) else MUTED)
            pygame.display.flip()
            clock.tick(60)
    finally:
        video.close()
        flight.close()
        pygame.quit()


if __name__ == "__main__":
    main()
