"""Offline tests of the read-only sensor discovery and conservative reporting."""
import json
from pathlib import Path
import tempfile
import unittest

from sensor_probe import IDENTITY_QUERIES, ProbeReport, interpret_sensor_text, run_probe


class SensorDecodeTests(unittest.TestCase):
    def test_json_gyro_accel_and_barometer_values(self):
        payload = json.dumps({
            "gyro": {"x": 1.1, "y": -0.2, "z": 0.3},
            "accel": {"x": 0.0, "y": 9.81, "z": -0.1},
            "pressure_hpa": 1013.25,
        }).encode()
        result = interpret_sensor_text(payload)
        self.assertEqual(result["gyro"]["x"], 1.1)
        self.assertEqual(result["accelerometer"]["y"], 9.81)
        self.assertEqual(result["barometer"], {"pressure": 1013.25, "unit": "hPa"})

    def test_labelled_numeric_fields(self):
        sample = b"gx=0.3, gy=-0.4, gz=0.5, ax=0.0, ay=9.8, az=0.1, pressure_pa=100212"
        result = interpret_sensor_text(sample)
        self.assertEqual(result["gyro"], {"x": 0.3, "y": -0.4, "z": 0.5})
        self.assertEqual(result["accelerometer"]["y"], 9.8)
        self.assertEqual(result["barometer"], {"pressure": 100212.0, "unit": "Pa"})

    def test_unverified_height_is_not_barometer(self):
        result = interpret_sensor_text(b'{"height_m":12.5}')
        self.assertIn("altitude_unverified_source", result)
        self.assertNotIn("barometer", result)

    def test_binary_and_video_like_data_are_not_sensor_values(self):
        for raw in (b"\xff\x08\x10\x80\x04\x01\x10\xaa",
                    bytes(range(256)), b"VGA", b"hello",
                    b"pressure_hpa=999999", b"gyro_x=2.0"):
            with self.subTest(raw=raw[:16]):
                result = interpret_sensor_text(raw)
                self.assertNotIn("barometer", result)
                self.assertNotIn("gyro", result)

    def test_reports_do_not_infer_sensor_hardware(self):
        report = ProbeReport("127.0.0.1", 10, 8081)
        report.record(b"VGA", ("127.0.0.1", 8080), "query_replies")
        report.record(b"\xff\x08\x11\x80", ("127.0.0.1", 8080), "udp_8081")
        result = report.as_dict()
        self.assertEqual(result["total_packets"], 2)
        self.assertEqual(result["device_identifiers"], ["VGA"])
        self.assertEqual(result["sensor_availability"]["imu_6_axis"]["status"], "not_verified")
        self.assertEqual(result["sensor_availability"]["barometer"]["status"], "not_verified")
        self.assertEqual(len(result["packet_groups"]), 2)

    def test_partial_imu_is_not_six_axis(self):
        report = ProbeReport("127.0.0.1", 10, 8081)
        report.record(b"gyro_x=1 gyro_y=2 gyro_z=3", ("127.0.0.1", 8080), "query_replies")
        sensor = report.as_dict()["sensor_availability"]
        self.assertEqual(sensor["gyroscope"]["status"], "labelled_numeric_values_observed")
        self.assertEqual(sensor["imu_6_axis"]["status"], "not_verified")

    def test_identification_queries_cannot_arm(self):
        self.assertEqual([b for b, _ in IDENTITY_QUERIES], [bytes([15]), bytes([40])])
        for data, _ in IDENTITY_QUERIES:
            self.assertEqual(len(data), 1)
            self.assertFalse(data.startswith(b"\xff\x08"))


class ProbeNetworkTests(unittest.TestCase):
    def test_passive_loopback_no_sensor_data(self):
        with tempfile.TemporaryDirectory() as folder:
            output = str(Path(folder) / "result.json")
            report = run_probe(host="127.0.0.1", duration=1.0,
                               listen_port=43991, report_path=output,
                               passive_only=True)
            self.assertEqual(report["identity_queries_sent"], [])
            self.assertEqual(report["total_packets"], 0)
            self.assertEqual(report["sensor_availability"]["barometer"]["status"],
                             "not_verified")
            self.assertTrue(Path(output).is_file())


if __name__ == "__main__":
    unittest.main()
