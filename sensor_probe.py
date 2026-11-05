"""Non-flight JJRC sensor telemetry availability diagnostic.

The original Android app uses UDPService for request/reply traffic on an
ephemeral UDP port and UDPServiceByPort to listen on local port 8081.
BaseApplication.getDevType() requests 0x0F and getVersion() requests 0x28.
No verified IMU or barometer query/decoder has been found for this firmware.

This probe NEVER sends flight-control or motor commands. It listens to UDP
and may send ONLY the two original Android read-only identification requests.

A missing response means "not observed", NOT "sensor absent".
Raw binary data is NOT reinterpreted as guessed IMU/barometer values.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import select
import socket
import time
from typing import Any


IDENTITY_QUERIES = ((b"\x0f", "android_get_device_type"),
                    (b"\x28", "android_get_firmware_version"))
MAX_GROUPS = 64
MAX_EXAMPLES = 3
MAX_TEXT_MESSAGES = 15
MAX_SENSOR_SAMPLES = 8
MAX_HEX_BYTES = 96

# Explicit semantic labels are required. An unlabelled binary float, or an
# "altitude" value without provenance, is NOT evidence of barometer telemetry.
GYRO_KEYS = (("gyro_x", "gyrox", "gx"), ("gyro_y", "gyroy", "gy"),
             ("gyro_z", "gyroz", "gz"))
ACCEL_KEYS = (("accel_x", "accx", "ax"), ("accel_y", "accy", "ay"),
              ("accel_z", "accz", "az"))
PRESSURE_KEYS = ("pressure_hpa", "pressure_pa", "barometer_hpa",
                 "baro_hpa", "barometric_pressure_hpa", "barometric_pressure_pa")
HEIGHT_KEYS = ("height_m", "altitude_m", "height", "altitude")
NUMBER = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


def _numbers(data: str) -> dict[str, float]:
    """Accept only clearly labelled numeric key=value or key:value fields."""
    result: dict[str, float] = {}
    expr = re.compile(
        r'(?<![\w])["\']?([A-Za-z_][A-Za-z_0-9]*)["\']?\s*[:=]\s*'
        r'([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)'
    )
    for match in expr.finditer(data):
        try:
            number = float(match.group(2))
            if math.isfinite(number):
                result[match.group(1).lower()] = number
        except ValueError:
            continue
    return result


def _flatten_json(payload: Any, prefix: str = "") -> dict[str, float]:
    """Support labelled JSON sensor objects, never arbitrary numeric arrays."""
    flat: dict[str, float] = {}
    if not isinstance(payload, dict):
        return flat
    for key, value in payload.items():
        path = (prefix + "_" + str(key)).strip("_").lower()
        if isinstance(value, dict):
            flat.update(_flatten_json(value, path))
        elif isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            flat[path] = float(value)
    return flat


def _triplet(fields: dict[str, float], sensor: str,
             choices: tuple[tuple[str, ...], ...]) -> dict[str, float] | None:
    # "gyro:{x,y,z}" and "accel:{x,y,z}" flatten to gyro_x, accel_x etc.
    def find(groups: tuple[str, ...]) -> float | None:
        for key in groups:
            if key in fields:
                return fields[key]
        return None

    prefix = "gyro" if sensor == "gyro" else "accel"
    triples = []
    for dimension, alts in zip("xyz", choices):
        triples.append(find((*alts, f"imu_{prefix}_{dimension}",
                             f"imu_{alts[0]}", f"{prefix}meter_{dimension}")))
    if any(value is None for value in triples):
        return None
    return dict(zip("xyz", triples))


def interpret_sensor_text(payload: bytes) -> dict[str, Any]:
    """Only identify numeric telemetry with explicit gyro/accel/pressure labels.

    No attempt is made to decode the undocumented binary packets.
    """
    if not payload or len(payload) > 8192:
        return {}
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError:
        return {}
    if "\x00" in text or not text.isprintable() and not all(c.isprintable() or c.isspace() for c in text):
        return {}
    fields = _numbers(text)
    try:
        obj = json.loads(text)
    except (json.JSONDecodeError, ValueError):
        obj = None
    fields.update(_flatten_json(obj))
    result: dict[str, Any] = {}
    gyro = _triplet(fields, "gyro", GYRO_KEYS)
    accel = _triplet(fields, "accel", ACCEL_KEYS)
    if gyro is not None:
        result["gyro"] = gyro
    if accel is not None:
        result["accelerometer"] = accel
    if gyro is not None or accel is not None:
        result["imu"] = True
    for label in PRESSURE_KEYS:
        if label in fields:
            value = fields[label]
            unit = "hPa" if label.endswith("hpa") else "Pa"
            if (250 <= value <= 1200 if unit == "hPa" else 25000 <= value <= 120000):
                result["barometer"] = {"pressure": value, "unit": unit}
            break
    for label in HEIGHT_KEYS:
        if label in fields:
            result["altitude_unverified_source"] = {"value": fields[label],
                                                     "unit": "m" if label.endswith("_m") else "unknown"}
            break
    return result


def _printable(payload: bytes) -> str | None:
    if not payload or len(payload) > 160:
        return None
    try:
        decoded = payload.decode("utf-8")
    except UnicodeError:
        return None
    if all(ch.isprintable() or ch in "\r\n\t" for ch in decoded):
        return decoded.strip()[:140]
    return None


def _identity_label(payload: bytes) -> str | None:
    # Original Android getDevType() may return ASCII followed by NUL padding,
    # e.g. 56 47 41 00 00 means "VGA", not a telemetry data packet.
    readable = _printable(payload.rstrip(b"\x00"))
    if not readable:
        return None
    # Android's onUdpReceiveData() recognizes identity prefixes, e.g. VGA.
    id_prefixes = ("VGA", "VGA2", "MJVG", "VSVG", "UDP", "720P", "1080P",
                   "HI2K", "HI4K", "drone")
    if readable.startswith(id_prefixes):
        return readable
    return None


def _firmware_version(payload: bytes) -> str | None:
    readable = _printable(payload.rstrip(b"\x00"))
    if readable and re.fullmatch(r"V\d+\.\d+\.\d+(?:[A-Za-z0-9_-]+)?", readable):
        return readable
    return None


class ProbeReport:
    """Bounded records, strictly separating raw replies from decoded values."""

    def __init__(self, host: str, duration: float, listen_port: int):
        self.host = host
        self.duration = duration
        self.listen_port = listen_port
        self.started_at = datetime.now(timezone.utc).isoformat()
        self.total_packets = 0
        self.total_bytes = 0
        self.port_counts: Counter[str] = Counter()
        self.groups: dict[str, dict[str, Any]] = {}
        self.printable_messages: list[str] = []
        self.device_identifiers: list[str] = []
        self.firmware_versions: list[str] = []
        self.sensor_samples: dict[str, list[dict[str, Any]]] = {
            "gyro": [], "accelerometer": [], "barometer": [],
            "altitude_unverified_source": [],
        }
        self.queries_sent: list[str] = []
        self.warnings: list[str] = []

    def record(self, payload: bytes, source: tuple[str, int], channel: str) -> None:
        self.total_packets += 1
        self.total_bytes += len(payload)
        self.port_counts[channel] += 1
        # Fixed grouping by payload length and first two bytes keeps report
        # bounded even if the UDP port carries video stream packets.
        group_id = f"{channel}:len={len(payload)}:prefix={payload[:2].hex()}"
        group = self.groups.get(group_id)
        if group is None and len(self.groups) < MAX_GROUPS:
            group = {"channel": channel, "length": len(payload),
                     "prefix_hex": payload[:2].hex(), "count": 0, "examples": []}
            self.groups[group_id] = group
        if group is not None:
            group["count"] += 1
            if len(group["examples"]) < MAX_EXAMPLES:
                group["examples"].append({
                    "from": f"{source[0]}:{source[1]}",
                    "hex_prefix": payload[:MAX_HEX_BYTES].hex(),
                    "truncated": len(payload) > MAX_HEX_BYTES,
                })
        msg = _printable(payload)
        if msg is not None and msg not in self.printable_messages and len(self.printable_messages) < MAX_TEXT_MESSAGES:
            self.printable_messages.append(msg)
        identity = _identity_label(payload)
        if identity is not None and identity not in self.device_identifiers:
            if len(self.device_identifiers) < MAX_TEXT_MESSAGES:
                self.device_identifiers.append(identity)
        version = _firmware_version(payload)
        if version is not None and version not in self.firmware_versions:
            if len(self.firmware_versions) < MAX_TEXT_MESSAGES:
                self.firmware_versions.append(version)
        observations = interpret_sensor_text(payload)
        for key in self.sensor_samples:
            if key in observations and len(self.sensor_samples[key]) < MAX_SENSOR_SAMPLES:
                self.sensor_samples[key].append(observations[key])

    def as_dict(self) -> dict[str, Any]:
        gyro = bool(self.sensor_samples["gyro"])
        accel = bool(self.sensor_samples["accelerometer"])
        pressure = bool(self.sensor_samples["barometer"])
        # IMU usually comprises gyro+accelerometer; a partial signal is NOT
        # enough to declare complete 6-axis IMU exposure.
        def state(observed: bool) -> str:
            return "labelled_numeric_values_observed" if observed else "not_verified"
        return {
            "probe": "JJRC Wi-Fi non-flight sensor diagnostic",
            "host": self.host,
            "started_at_utc": self.started_at,
            "duration_seconds": self.duration,
            "passive_udp_listen_port": self.listen_port,
            "identity_queries_sent": self.queries_sent,
            "total_packets": self.total_packets,
            "total_bytes": self.total_bytes,
            "received_by_channel": dict(self.port_counts),
            "device_identifiers": self.device_identifiers,
            "firmware_versions": self.firmware_versions,
            "sensor_availability": {
                "gyroscope": {"status": state(gyro), "samples": self.sensor_samples["gyro"]},
                "accelerometer": {"status": state(accel), "samples": self.sensor_samples["accelerometer"]},
                "imu_6_axis": {"status": state(gyro and accel),
                               "note": "Requires explicit gyro and accel numeric values; cannot infer from a camera stream"},
                "barometer": {"status": state(pressure),
                              "samples": self.sensor_samples["barometer"]},
                "height_unknown_source": {"status": state(bool(self.sensor_samples["altitude_unverified_source"])),
                                          "samples": self.sensor_samples["altitude_unverified_source"],
                                          "note": "Altitude/height does not prove barometric pressure availability"},
            },
            "printable_messages": self.printable_messages,
            "packet_groups": sorted(self.groups.values(), key=lambda x: -x["count"]),
            "warnings": self.warnings,
            "limitations": [
                "No verified binary IMU/barometer telemetry decoder is available for this firmware.",
                "No decoded values or UDP response DOES NOT prove the drone lacks an internal sensor.",
                "No motor, takeoff, landing or flight-control packets were sent.",
                "The original Android GravityUtils.java reads the PHONE sensor, not drone IMU telemetry.",
            ],
        }


def run_probe(host: str = "172.16.10.1", duration: float = 20.0,
              listen_port: int = 8081, local_ip: str | None = None,
              report_path: str = "sensor_probe.json",
              passive_only: bool = False) -> dict[str, Any]:
    """Run on ground with motors OFF. Listen; optionally send Android ID queries."""
    if duration < 1 or duration > 300:
        raise ValueError("duration must be between 1 and 300 seconds")
    if not (1024 <= listen_port <= 65535):
        raise ValueError("listen port must be between 1024 and 65535")
    report = ProbeReport(host, duration, listen_port)
    sockets: list[socket.socket] = []
    reply_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        reply_sock.bind((local_ip or "0.0.0.0", 0))
        reply_sock.setblocking(False)
        sockets.append(reply_sock)
        passive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Do not steal the controller's port while flight app is running.
            passive.bind((local_ip or "0.0.0.0", listen_port))
            passive.setblocking(False)
            sockets.append(passive)
        except OSError as exc:
            report.warnings.append(f"Could not listen on UDP {listen_port}: {exc}")
            passive.close()
        print(f"JJRC sensor diagnostic | drone {host} | duration {duration:.0f}s")
        print("Motor/flight commands: NONE. Keep the drone stationary on the ground.")
        print(f"Listening for UDP replies at {reply_sock.getsockname()[1]}"
              + (f" and {listen_port}" if len(sockets) > 1 else ""))
        if not passive_only:
            # Only original app's getDevType() and getVersion() requests.
            for query, label in IDENTITY_QUERIES:
                try:
                    reply_sock.sendto(query, (host, 8080))
                    report.queries_sent.append(label)
                    print("Sent read-only identity request:", label)
                except OSError as exc:
                    report.warnings.append(f"{label}: {exc}")
        else:
            print("Passive mode: absolutely no outbound packets.")
        end = time.monotonic() + duration
        next_status = time.monotonic() + 5
        while time.monotonic() < end:
            timeout = min(0.35, max(0.0, end - time.monotonic()))
            readable, _, _ = select.select(sockets, [], [], timeout)
            for sock in readable:
                try:
                    payload, source = sock.recvfrom(65535)
                except (BlockingIOError, OSError):
                    continue
                # Avoid counting packets from unrelated equipment.
                if source[0] != host:
                    continue
                channel = "udp_8081" if sock is not reply_sock else "query_replies"
                report.record(payload, source, channel)
            if time.monotonic() >= next_status:
                print(f"  UDP packets captured: {report.total_packets}"
                      f" | identifiable IMU: {bool(report.sensor_samples['gyro'] and report.sensor_samples['accelerometer'])}"
                      f" | barometer: {bool(report.sensor_samples['barometer'])}")
                next_status += 5
    finally:
        for sock in sockets:
            sock.close()
    data = report.as_dict()
    if report_path:
        path = Path(report_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\nRESULT")
    print("Packets received:", data["total_packets"])
    print("Device identifiers:", ", ".join(data["device_identifiers"]) or "not observed")
    print("Firmware versions:", ", ".join(data["firmware_versions"]) or "not observed")
    for name, entry in data["sensor_availability"].items():
        print(f"  {name:22s}: {entry['status']}")
    if data["warnings"]:
        print("Warnings:", " | ".join(data["warnings"]))
    print("Important: 'not_verified' means no recognizable telemetry; the sensor may still exist internally.")
    if report_path:
        print("Report saved:", report_path)
    return data


def main() -> None:
    p = argparse.ArgumentParser(description="Read-only JJRC Wi-Fi IMU/barometer availability test")
    p.add_argument("--host", default="172.16.10.1")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--listen-port", type=int, default=8081)
    p.add_argument("--local-ip", default=None)
    p.add_argument("--report", default="sensor_probe.json")
    p.add_argument("--passive-only", action="store_true")
    args = p.parse_args()
    run_probe(args.host, args.duration, args.listen_port, args.local_ip,
              args.report, args.passive_only)


if __name__ == "__main__":
    main()
