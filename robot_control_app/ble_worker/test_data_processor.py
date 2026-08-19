# Copyright 2026 Filip Dymczyk and Konrad Grucel

import csv
import os
import struct
import tempfile
import unittest

from robot_control_app import ble_protocol
from robot_control_app.ble_worker.data_processor import (
    APP_VERSION,
    BATTERY_STATUS,
    COMMAND_RESULT,
    LOG_HEADER,
    LQR_STATE,
    PID_STATE,
    TELEMETRY_HEADER,
    TELEMETRY_SAMPLE,
    DataProcessor,
)


class DataProcessorTest(unittest.TestCase):
    @staticmethod
    def packet(message_type, payload=b"", sequence=0):
        return ble_protocol.unpack_packet(
            ble_protocol.pack_packet(
                message_type, payload, sequence=sequence
            )
        )

    def test_telemetry_records_every_sample_and_returns_latest(self):
        payload = TELEMETRY_HEADER.pack(2, 2, b"\0\0\0")
        payload += TELEMETRY_SAMPLE.pack(2000, *range(10))
        payload += TELEMETRY_SAMPLE.pack(3000, *range(10, 20))

        parsed = DataProcessor().process_packet(
            self.packet(ble_protocol.MessageType.TELEMETRY, payload, sequence=7)
        )

        self.assertEqual(len(parsed.telemetry_samples), 2)
        self.assertEqual(parsed.telemetry_samples[0]["timestamp_us"], 2000)
        self.assertEqual(parsed.telemetry["timestamp_us"], 3000)
        self.assertEqual(parsed.telemetry["pwm1"], 19.0)
        self.assertEqual(parsed.telemetry["frame_sequence"], 7)
        self.assertEqual(parsed.telemetry["dropped_samples"], 2)

    def test_rejects_bad_telemetry_payloads(self):
        processor = DataProcessor()
        cases = (
            TELEMETRY_HEADER.pack(0, 1, b"\0\0\0"),
            TELEMETRY_HEADER.pack(0, 0, b"\0\0\0"),
            TELEMETRY_HEADER.pack(0, 1, b"\0\1\0")
            + TELEMETRY_SAMPLE.pack(0, *([0.0] * 10)),
        )

        for payload in cases:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                processor.process_packet(
                    self.packet(ble_protocol.MessageType.TELEMETRY, payload)
                )

    def test_rejects_malformed_payload_for_every_notification_type(self):
        message_types = (
            ble_protocol.MessageType.LOG,
            ble_protocol.MessageType.BATTERY_STATUS,
            ble_protocol.MessageType.APP_VERSION,
            ble_protocol.MessageType.PID_STATE,
            ble_protocol.MessageType.LQR_STATE,
            ble_protocol.MessageType.TRAJECTORY_COMPLETE,
            ble_protocol.MessageType.COMMAND_RESULT,
            ble_protocol.MessageType.IDENTIFICATION_COMPLETE,
        )
        for message_type in message_types:
            with self.subTest(message_type=message_type), self.assertRaises(ValueError):
                DataProcessor().process_packet(self.packet(message_type, b"\x01"))

    def test_decodes_log_and_rejects_invalid_utf8(self):
        payload = LOG_HEADER.pack(3, 3, 5) + b"imu" + "ready".encode()
        parsed = DataProcessor().process_packet(
            self.packet(ble_protocol.MessageType.LOG, payload)
        )
        self.assertEqual((parsed.log.level, parsed.log.module, parsed.log.text), (3, "imu", "ready"))

        invalid = LOG_HEADER.pack(1, 1, 1) + b"\xffx"
        with self.assertRaisesRegex(ValueError, "invalid UTF-8"):
            DataProcessor().process_packet(
                self.packet(ble_protocol.MessageType.LOG, invalid)
            )

    def test_decodes_battery_pid_and_version(self):
        processor = DataProcessor()
        battery = processor.process_packet(
            self.packet(
                ble_protocol.MessageType.BATTERY_STATUS,
                BATTERY_STATUS.pack(7420, 87, 0),
            )
        ).battery
        self.assertEqual((battery.millivolts, battery.percent), (7420, 87))

        values = [value / 10 for value in range(15)]
        pid = processor.process_packet(
            self.packet(
                ble_protocol.MessageType.PID_STATE, PID_STATE.pack(*values)
            )
        ).pid_params
        self.assertAlmostEqual(pid["distance"]["kp"], 0.0)
        self.assertAlmostEqual(pid["balance"]["kd"], 0.8)
        self.assertAlmostEqual(pid["wheel_speed"]["kd"], 1.4)

        version = processor.process_packet(
            self.packet(
                ble_protocol.MessageType.APP_VERSION,
                APP_VERSION.pack(1, 2, 3, 0, 456),
            )
        ).app_version
        self.assertEqual(
            (version.major, version.minor, version.revision, version.build),
            (1, 2, 3, 456),
        )

        lqr = processor.process_packet(
            self.packet(
                ble_protocol.MessageType.LQR_STATE,
                LQR_STATE.pack(4.5, -1.25),
            )
        ).lqr_params
        self.assertEqual(lqr, {"kx": 4.5, "ky": -1.25})

    def test_decodes_completion_and_command_result(self):
        processor = DataProcessor()
        trajectory = processor.process_packet(
            self.packet(ble_protocol.MessageType.TRAJECTORY_COMPLETE)
        )
        self.assertTrue(trajectory.trajectory_complete)

        payload = COMMAND_RESULT.pack(
            123,
            ble_protocol.MessageType.SET_PID,
            ble_protocol.CommandStatus.INVALID_VALUE,
        )
        result = processor.process_packet(
            self.packet(ble_protocol.MessageType.COMMAND_RESULT, payload)
        ).command_result
        self.assertEqual(result.request_sequence, 123)
        self.assertEqual(result.request_type, ble_protocol.MessageType.SET_PID)
        self.assertEqual(result.status, ble_protocol.CommandStatus.INVALID_VALUE)

        identification = processor.process_packet(
            self.packet(ble_protocol.MessageType.IDENTIFICATION_COMPLETE)
        )
        self.assertTrue(identification.identification_complete)

    def test_tracks_telemetry_sequence_gaps_and_wraparound(self):
        processor = DataProcessor()
        payload = TELEMETRY_HEADER.pack(0, 1, b"\0\0\0")
        payload += TELEMETRY_SAMPLE.pack(1, *([0.0] * 10))

        processor.process_packet(
            self.packet(
                ble_protocol.MessageType.TELEMETRY,
                payload,
                sequence=0xFFFFFFFF,
            )
        )
        self.assertEqual(processor.expected_frame_sequence, 0)

        with self.assertLogs("DataProcessor", level="WARNING") as captured:
            processor.process_packet(
                self.packet(
                    ble_protocol.MessageType.TELEMETRY,
                    payload,
                    sequence=2,
                )
            )
        self.assertIn("expected 0, received 2", captured.output[0])

    def test_csv_off_closes_and_next_sample_creates_new_file(self):
        with tempfile.TemporaryDirectory() as directory:
            processor = DataProcessor(directory)
            payload = TELEMETRY_HEADER.pack(0, 2, b"\0\0\0")
            payload += TELEMETRY_SAMPLE.pack(1, *([1.0] * 10))
            payload += TELEMETRY_SAMPLE.pack(2, *([2.0] * 10))
            processor.set_recording(True)
            processor.process_packet(
                self.packet(
                    ble_protocol.MessageType.TELEMETRY, payload, sequence=0
                )
            )
            first_path = processor.csv_file.name
            processor.set_recording(False)
            self.assertIsNone(processor.csv_file)

            processor.process_packet(
                self.packet(
                    ble_protocol.MessageType.TELEMETRY, payload, sequence=1
                )
            )
            self.assertEqual(len(os.listdir(directory)), 1)

            processor.set_recording(True)
            processor.process_packet(
                self.packet(
                    ble_protocol.MessageType.TELEMETRY, payload, sequence=2
                )
            )
            second_path = processor.csv_file.name
            processor.set_recording(False)

            self.assertNotEqual(first_path, second_path)
            self.assertEqual(len(os.listdir(directory)), 2)
            with open(first_path, newline="") as recorded:
                rows = list(csv.reader(recorded))
            self.assertEqual(len(rows), 3)


if __name__ == "__main__":
    unittest.main()
