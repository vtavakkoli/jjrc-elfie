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

    def __init__(self, host, port, demo=False, local_ip=None):
        self.address = (host, port)
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
                self.sock.sendto(HOLD_PACKET, self.address)
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
    parser.add_argument("--control-port", type=int, default=8895)
    parser.add_argument("--video-port", type=int, default=8888)
    parser.add_argument("--local-ip", help="Bind UDP to this Wi-Fi IPv4 address (default: auto from video connection)")
    parser.add_argument("--demo", action="store_true", help="Preview interface without sending commands")
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((1100, 760), pygame.RESIZABLE)
    pygame.display.set_caption("ELFIE FPV  |  Keyboard Flight Deck")
    title_font = pygame.font.SysFont("segoeui", 29, bold=True)
    font = pygame.font.SysFont("segoeui", 19)
    small = pygame.font.SysFont("consolas", 16)
    tiny = pygame.font.SysFont("consolas", 14)
    clock = pygame.time.Clock()

    flight = FlightLink(args.host, args.control_port, demo=args.demo, local_ip=args.local_ip)
    video = VideoLink(args.host, args.video_port, demo=args.demo)
    speed = 0.30
    last_send = 0.0
    start_time = time.monotonic()
    command_until = 0.0
    active_command = 0
    running = True
    armed_requested = False
    focused = True
    mode = "STARTING"
    notice = "Remove propellers for the first control test. UDP TX does not confirm drone reception."
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
                        # Cut motors (may cause uncontrolled drop). Works even if not armed.
                        active_command, command_until = 4, now + 1.05
                        armed_requested = False
                        mode = "EMERGENCY STOP REQUESTED"
                    elif event.key == pygame.K_l and now >= command_until:
                        active_command, command_until = 2, now + 1.05
                        armed_requested = False
                        mode = "LAND REQUESTED"
                    elif event.key == pygame.K_t and now - start_time >= 2.0 and now >= command_until:
                        active_command, command_until = 1, now + 1.05
                        armed_requested = True
                        mode = "ARM REQUESTED"
                    elif event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS):
                        speed = min(0.8, round(speed + 0.05, 2))
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        speed = max(0.1, round(speed - 0.05, 2))

            frame, frame_time, video_status, video_local_ip = video.snapshot()
            # TCP video already found the correct route to the drone. Use its IPv4
            # Wi-Fi interface for UDP too, rather than the Windows default route.
            if not args.local_ip and video_local_ip and flight.local_ip != video_local_ip:
                flight.open(video_local_ip)

            keys = pygame.key.get_pressed()
            controls_enabled = focused and armed_requested and now >= command_until
            if controls_enabled and not keys[pygame.K_SPACE]:
                roll = speed * (int(keys[pygame.K_d]) - int(keys[pygame.K_a]))
                pitch = speed * (int(keys[pygame.K_w]) - int(keys[pygame.K_s]))
                throttle = speed * (int(keys[pygame.K_UP]) - int(keys[pygame.K_DOWN]))
                yaw = speed * (int(keys[pygame.K_RIGHT]) - int(keys[pygame.K_LEFT]))
            else:
                roll = pitch = throttle = yaw = 0.0

            if now - last_send >= 0.05:
                if now - start_time < 1.0:
                    data = IDLE_PACKET
                    mode = "INITIALIZING IDLE"
                elif now - start_time < 2.0:
                    data = HOLD_PACKET
                    mode = "INITIALIZING ALT-HOLD"
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
            draw_text(screen, small, "SIMULATION" if args.demo else args.host, (max(24, width - 205), 27), BLUE)
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
                      "T ARM    L LAND    X EMERGENCY STOP    SPACE NEUTRAL    +/- SPEED    ESC QUIT",
                      (28, y + 75))
            draw_text(screen, small,
                      "GAIN {:2.0f}%   ROLL {:+.2f}   PITCH {:+.2f}   ALT {:+.2f}   YAW {:+.2f}".format(
                          speed * 100, roll, pitch, throttle, yaw),
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
