# Copyright 2026 Filip Dymczyk and Konrad Grucel

import csv
import logging
import os
import struct
import time
from dataclasses import dataclass, field

from robot_control_app import ble_protocol

TELEMETRY_HEADER = struct.Struct("<IB3s")
TELEMETRY_SAMPLE = struct.Struct("<I10f")
TELEMETRY_SAMPLES_PER_FRAME = 5
LOG_HEADER = struct.Struct("<BBH")
BATTERY_STATUS = struct.Struct("<HBB")
APP_VERSION = struct.Struct("<BBBBI")
PID_STATE = struct.Struct("<15f")
LQR_STATE = struct.Struct("<ff")
COMMAND_RESULT = struct.Struct("<IBB")
TELEMETRY_KEYS = (
    "timestamp_us",
    "bs",
    "ab",
    "rs",
    "ar",
    "ts0",
    "ts1",
    "s0",
    "s1",
    "pwm0",
    "pwm1",
)

logger = logging.getLogger("DataProcessor")


@dataclass(frozen=True)
class LogMessage:
    level: int
    module: str
    text: str


@dataclass(frozen=True)
class BatteryStatus:
    millivolts: int
    percent: int


@dataclass(frozen=True)
class AppVersion:
    major: int
    minor: int
    revision: int
    build: int


@dataclass(frozen=True)
class CommandResult:
    request_packet_number: int
    request_type: ble_protocol.MessageType
    status: ble_protocol.CommandStatus


@dataclass
class ParsedData:
    telemetry: dict[str, float | int] | None = None
    telemetry_samples: list[dict[str, float | int]] = field(default_factory=list)
    log: LogMessage | None = None
    battery: BatteryStatus | None = None
    app_version: AppVersion | None = None
    pid_params: dict | None = None
    lqr_params: dict | None = None
    trajectory_complete: bool = False
    command_result: CommandResult | None = None
    identification_complete: bool = False


class DataProcessor:
    def __init__(self, log_dir: str = "robot_data_logs"):
        self.log_dir = log_dir
        self.csv_file = None
        self.csv_writer = None
        self.last_csv_flush = 0.0
        self.expected_telemetry_packet_number = None
        self._recording_enabled = False

    def set_recording(self, enabled: bool):
        enabled = bool(enabled)
        if not enabled:
            self.close_recording()
        self._recording_enabled = enabled

    def process_packet(self, packet: ble_protocol.Packet) -> ParsedData:
        """Decode one validated RBT1 packet."""
        handlers = {
            ble_protocol.MessageType.TELEMETRY: self._parse_telemetry,
            ble_protocol.MessageType.LOG: self._parse_log,
            ble_protocol.MessageType.BATTERY_STATUS: self._parse_battery,
            ble_protocol.MessageType.APP_VERSION: self._parse_app_version,
            ble_protocol.MessageType.PID_STATE: self._parse_pid_state,
            ble_protocol.MessageType.LQR_STATE: self._parse_lqr_state,
            ble_protocol.MessageType.TRAJECTORY_COMPLETE: self._parse_trajectory_complete,
            ble_protocol.MessageType.COMMAND_RESULT: self._parse_command_result,
            ble_protocol.MessageType.IDENTIFICATION_COMPLETE: self._parse_identification_complete,
        }
        handler = handlers.get(packet.message_type)
        if handler is None:
            raise ValueError(
                f"Unexpected notification type: {packet.message_type.name}"
            )
        return handler(packet)

    def _parse_telemetry(self, packet: ble_protocol.Packet) -> ParsedData:
        reader = ble_protocol.PayloadReader(packet.payload)
        dropped_samples = reader.get_u32()
        sample_count = reader.get_u8()
        reserved = reader.get_bytes(3)
        if reserved != b"\0\0\0":
            raise ValueError("Telemetry reserved bytes are not zero")
        if not 1 <= sample_count <= TELEMETRY_SAMPLES_PER_FRAME:
            raise ValueError(f"Invalid telemetry sample count: {sample_count}")

        expected_size = sample_count * TELEMETRY_SAMPLE.size
        if reader.remaining != expected_size:
            raise ValueError(
                f"Invalid telemetry sample data size: got {reader.remaining}, expected {expected_size}"
            )

        if (
            self.expected_telemetry_packet_number is not None
            and packet.packet_number != self.expected_telemetry_packet_number
        ):
            logger.warning(
                "Telemetry frame gap: expected %u, received %u",
                self.expected_telemetry_packet_number,
                packet.packet_number,
            )
        self.expected_telemetry_packet_number = (packet.packet_number + 1) & 0xFFFFFFFF

        samples = []
        for _ in range(sample_count):
            values = [reader.get_u32()]
            values.extend(reader.get_float() for _ in range(10))
            sample = dict(zip(TELEMETRY_KEYS, values))
            sample["packet_number"] = packet.packet_number
            sample["dropped_samples"] = dropped_samples
            samples.append(sample)
        reader.finish()

        if self._recording_enabled:
            self.__record_telemetry_batch(samples)

        return ParsedData(telemetry=samples[-1], telemetry_samples=samples)

    def _parse_log(self, packet: ble_protocol.Packet) -> ParsedData:
        reader = ble_protocol.PayloadReader(packet.payload)
        level = reader.get_u8()
        module_len = reader.get_u8()
        text_len = reader.get_u16()
        if reader.remaining != module_len + text_len:
            raise ValueError(
                f"Invalid log text size: got {reader.remaining}, expected {module_len + text_len}"
            )
        try:
            module = reader.get_bytes(module_len).decode("utf-8")
            text = reader.get_bytes(text_len).decode("utf-8")
        except UnicodeDecodeError as error:
            raise ValueError("Log payload contains invalid UTF-8") from error
        reader.finish()
        return ParsedData(log=LogMessage(level, module, text))

    def _parse_battery(self, packet: ble_protocol.Packet) -> ParsedData:
        reader = ble_protocol.PayloadReader(packet.payload)
        millivolts = reader.get_u16()
        percent = reader.get_u8()
        reserved = reader.get_u8()
        reader.finish()
        if reserved != 0:
            raise ValueError("Battery reserved byte is not zero")
        if percent > 100:
            raise ValueError(f"Invalid battery percentage: {percent}")
        return ParsedData(battery=BatteryStatus(millivolts, percent))

    def _parse_app_version(self, packet: ble_protocol.Packet) -> ParsedData:
        reader = ble_protocol.PayloadReader(packet.payload)
        major = reader.get_u8()
        minor = reader.get_u8()
        revision = reader.get_u8()
        reserved = reader.get_u8()
        build = reader.get_u32()
        reader.finish()
        if reserved != 0:
            raise ValueError("App version reserved byte is not zero")
        return ParsedData(app_version=AppVersion(major, minor, revision, build))

    def _parse_pid_state(self, packet: ble_protocol.Packet) -> ParsedData:
        reader = ble_protocol.PayloadReader(packet.payload)
        values = [reader.get_float() for _ in range(15)]
        reader.finish()
        controllers = (
            "distance",
            "linear_speed",
            "balance",
            "rotate",
            "wheel_speed",
        )
        params = {
            controller: {
                "kp": values[index * 3],
                "ki": values[index * 3 + 1],
                "kd": values[index * 3 + 2],
            }
            for index, controller in enumerate(controllers)
        }
        return ParsedData(pid_params=params)

    def _parse_lqr_state(self, packet: ble_protocol.Packet) -> ParsedData:
        reader = ble_protocol.PayloadReader(packet.payload)
        kx = reader.get_float()
        ky = reader.get_float()
        reader.finish()
        return ParsedData(lqr_params={"kx": kx, "ky": ky})

    def _parse_trajectory_complete(self, packet: ble_protocol.Packet) -> ParsedData:
        self._require_empty(packet.payload, "trajectory complete")
        return ParsedData(trajectory_complete=True)

    def _parse_command_result(self, packet: ble_protocol.Packet) -> ParsedData:
        reader = ble_protocol.PayloadReader(packet.payload)
        request_packet_number = reader.get_u32()
        request_type = reader.get_u8()
        status = reader.get_u8()
        reader.finish()
        try:
            typed_request = ble_protocol.MessageType(request_type)
        except ValueError as error:
            raise ValueError(
                f"Invalid command result request type: 0x{request_type:02X}"
            ) from error
        try:
            typed_status = ble_protocol.CommandStatus(status)
        except ValueError as error:
            raise ValueError(f"Invalid command result status: {status}") from error
        return ParsedData(
            command_result=CommandResult(
                request_packet_number, typed_request, typed_status
            )
        )

    def _parse_identification_complete(self, packet: ble_protocol.Packet) -> ParsedData:
        self._require_empty(packet.payload, "identification complete")
        return ParsedData(identification_complete=True)

    @staticmethod
    def _require_empty(payload: bytes, name: str):
        if payload:
            raise ValueError(f"Invalid {name} payload size: expected 0")

    def close_recording(self):
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

    def reset_stream(self):
        self.expected_telemetry_packet_number = None

    def __record_telemetry_batch(self, samples: list[dict]):
        if not samples:
            return

        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        if not self.csv_file:
            filename = os.path.join(self.log_dir, f"data_{time.time_ns()}.csv")
            self.csv_file = open(filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            headers = ["host_timestamp"] + list(samples[0].keys())
            self.csv_writer.writerow(headers)
            self.last_csv_flush = time.monotonic()

        for sample in samples:
            row = [time.time()] + list(sample.values())
            self.csv_writer.writerow(row)

        now = time.monotonic()
        if now - self.last_csv_flush >= 1.0:
            self.csv_file.flush()
            self.last_csv_flush = now
