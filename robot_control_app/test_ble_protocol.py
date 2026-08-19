# Copyright 2026 Filip Dymczyk and Konrad Grucel

import struct
import unittest

from robot_control_app import ble_protocol


class BleProtocolTest(unittest.TestCase):
    def test_payload_writer_reader_roundtrip(self):
        payload = (
            ble_protocol.PayloadWriter()
            .put_u8(7)
            .put_u16(0x1234)
            .put_u32(0x89ABCDEF)
            .put_float(1.25)
            .put_bytes(b"ok")
            .to_bytes()
        )
        reader = ble_protocol.PayloadReader(payload)
        self.assertEqual(reader.get_u8(), 7)
        self.assertEqual(reader.get_u16(), 0x1234)
        self.assertEqual(reader.get_u32(), 0x89ABCDEF)
        self.assertAlmostEqual(reader.get_float(), 1.25)
        self.assertEqual(reader.get_bytes(2), b"ok")
        reader.finish()

    def test_payload_layer_enforces_bounds(self):
        writer = ble_protocol.PayloadWriter().put_bytes(
            bytes(ble_protocol.MAX_PAYLOAD_SIZE)
        )
        self.assertEqual(len(writer.to_bytes()), ble_protocol.MAX_PAYLOAD_SIZE)
        with self.assertRaisesRegex(ValueError, "Payload exceeds"):
            writer.put_u8(1)

        with self.assertRaisesRegex(ValueError, "truncated"):
            ble_protocol.PayloadReader(b"\x01").get_u32()
        with self.assertRaisesRegex(ValueError, "trailing"):
            ble_protocol.PayloadReader(b"\x01").finish()

    def test_packet_roundtrip(self):
        encoded = ble_protocol.pack_packet(
            ble_protocol.MessageType.SET_SETPOINT,
            b"\x01\x02",
            sequence=0x12345678,
            flags=0xA5,
        )

        packet = ble_protocol.unpack_packet(encoded)

        self.assertEqual(packet.message_type, ble_protocol.MessageType.SET_SETPOINT)
        self.assertEqual(packet.payload, b"\x01\x02")
        self.assertEqual(packet.sequence, 0x12345678)
        self.assertEqual(packet.flags, 0xA5)
        self.assertEqual(len(encoded), ble_protocol.HEADER.size + 2)

    def test_command_builders_use_binary_envelope(self):
        commands = (
            (
                ble_protocol.state_command(ble_protocol.StateAction.START),
                ble_protocol.MessageType.STATE_COMMAND,
                struct.pack("<B", ble_protocol.StateAction.START),
            ),
            (
                ble_protocol.dfu_command(ble_protocol.DfuAction.SKIP),
                ble_protocol.MessageType.DFU_COMMAND,
                struct.pack("<B", ble_protocol.DfuAction.SKIP),
            ),
            (
                ble_protocol.get_pid_state_command(),
                ble_protocol.MessageType.GET_PID_STATE,
                b"",
            ),
            (
                ble_protocol.set_pid_command(
                    ble_protocol.ControllerId.BALANCE, 1.0, 2.0, 3.0
                ),
                ble_protocol.MessageType.SET_PID,
                struct.pack("<Bfff", ble_protocol.ControllerId.BALANCE, 1.0, 2.0, 3.0),
            ),
            (
                ble_protocol.set_setpoint_command(
                    ble_protocol.ControllerId.ROTATE, 45.0
                ),
                ble_protocol.MessageType.SET_SETPOINT,
                struct.pack("<Bf", ble_protocol.ControllerId.ROTATE, 45.0),
            ),
            (
                ble_protocol.trajectory_command(90.0, 1.5),
                ble_protocol.MessageType.TRAJECTORY_COMMAND,
                struct.pack("<ff", 90.0, 1.5),
            ),
            (
                ble_protocol.set_lqr_command(1.25, -2.5),
                ble_protocol.MessageType.SET_LQR,
                struct.pack("<ff", 1.25, -2.5),
            ),
        )

        for encoded, expected_type, expected_payload in commands:
            with self.subTest(message_type=expected_type):
                packet = ble_protocol.unpack_packet(encoded)
                self.assertEqual(packet.message_type, expected_type)
                self.assertEqual(packet.payload, expected_payload)

    def test_rejects_malformed_packets(self):
        valid = ble_protocol.pack_packet(ble_protocol.MessageType.GET_PID_STATE)
        malformed = (
            b"",
            valid[:-1],
            b"BAD!" + valid[4:],
            valid + b"\0",
            valid[:4] + b"\xff" + valid[5:],
        )

        for encoded in malformed:
            with self.subTest(encoded=encoded), self.assertRaises(ValueError):
                ble_protocol.unpack_packet(encoded)

    def test_rejects_oversized_payload(self):
        with self.assertRaisesRegex(ValueError, "Payload exceeds"):
            ble_protocol.pack_packet(
                ble_protocol.MessageType.SET_PID,
                bytes(ble_protocol.MAX_PAYLOAD_SIZE + 1),
            )

    def test_five_sample_telemetry_packet_is_240_bytes(self):
        telemetry_payload_size = struct.calcsize("<IB3s") + 5 * struct.calcsize(
            "<I10f"
        )
        packet = ble_protocol.pack_packet(
            ble_protocol.MessageType.TELEMETRY,
            bytes(telemetry_payload_size),
        )
        self.assertEqual(telemetry_payload_size, 228)
        self.assertEqual(len(packet), 240)

    def test_identification_builder_validates_lengths(self):
        with self.assertRaisesRegex(ValueError, "requires 10"):
            ble_protocol.identification_command([0.0], [1.0])

        packet = ble_protocol.unpack_packet(
            ble_protocol.identification_command(
                [float(value) for value in range(10)],
                [float(value + 1) / 10 for value in range(10)],
            )
        )
        self.assertEqual(
            packet.message_type, ble_protocol.MessageType.IDENTIFICATION_CONFIG
        )
        self.assertEqual(len(packet.payload), struct.calcsize("<20f"))

        with self.assertRaisesRegex(ValueError, "positive"):
            ble_protocol.identification_command([0.0] * 10, [0.0] * 10)


if __name__ == "__main__":
    unittest.main()
