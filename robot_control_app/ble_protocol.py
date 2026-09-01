# Copyright 2026 Filip Dymczyk and Konrad Grucel

import struct
from dataclasses import dataclass
from enum import IntEnum

MAGIC = 0x31544252  # "RBT1"
HEADER = struct.Struct("<IBBHI")
MAX_PACKET_SIZE = 244
MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - HEADER.size


class MessageType(IntEnum):
    TELEMETRY = 0x01
    LOG = 0x02
    BATTERY_STATUS = 0x03
    APP_VERSION = 0x04
    PID_STATE = 0x05
    TRAJECTORY_COMPLETE = 0x06
    COMMAND_RESULT = 0x07
    IDENTIFICATION_COMPLETE = 0x08
    LQR_STATE = 0x09

    STATE_COMMAND = 0x20
    DFU_COMMAND = 0x21
    GET_PID_STATE = 0x22
    SET_PID = 0x23
    SET_SETPOINT = 0x24
    SET_LQR = 0x27
    DRIVE_COMMAND = 0x28
    SET_MODE = 0x29


class ControlMode(IntEnum):
    STANDARD = 0
    FREE_DRIVE = 1


class ControllerId(IntEnum):
    DISTANCE = 0
    LINEAR_SPEED = 1
    BALANCE = 2
    ROTATE = 3
    WHEEL_SPEED = 4


class StateAction(IntEnum):
    START = 0
    STOP = 1


class DfuAction(IntEnum):
    START = 0
    SKIP = 1


class CommandStatus(IntEnum):
    OK = 0
    INVALID_MESSAGE = 1
    INVALID_LENGTH = 2
    INVALID_VALUE = 3
    INVALID_STATE = 4
    UNSUPPORTED_MESSAGE = 5


@dataclass(frozen=True)
class Packet:
    message_type: MessageType
    reserved: int
    packet_number: int
    payload: bytes


class PayloadWriter:
    def __init__(self):
        self._data = bytearray()

    def _append(self, data: bytes):
        if len(self._data) + len(data) > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload exceeds {MAX_PAYLOAD_SIZE} bytes")
        self._data.extend(data)

    def put_u8(self, value: int):
        self._append(struct.pack("<B", value))
        return self

    def put_u16(self, value: int):
        self._append(struct.pack("<H", value))
        return self

    def put_u32(self, value: int):
        self._append(struct.pack("<I", value))
        return self

    def put_float(self, value: float):
        self._append(struct.pack("<f", value))
        return self

    def put_bytes(self, data: bytes):
        self._append(bytes(data))
        return self

    def to_bytes(self) -> bytes:
        return bytes(self._data)


class PayloadReader:
    def __init__(self, payload: bytes):
        if len(payload) > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload exceeds {MAX_PAYLOAD_SIZE} bytes")
        self._data = memoryview(payload)
        self._offset = 0

    def _read(self, size: int) -> memoryview:
        end = self._offset + size
        if end > len(self._data):
            raise ValueError("Payload is truncated")
        value = self._data[self._offset : end]
        self._offset = end
        return value

    def get_u8(self) -> int:
        return struct.unpack("<B", self._read(1))[0]

    def get_u16(self) -> int:
        return struct.unpack("<H", self._read(2))[0]

    def get_u32(self) -> int:
        return struct.unpack("<I", self._read(4))[0]

    def get_float(self) -> float:
        return struct.unpack("<f", self._read(4))[0]

    def get_bytes(self, size: int) -> bytes:
        return bytes(self._read(size))

    @property
    def remaining(self) -> int:
        return len(self._data) - self._offset

    def finish(self):
        if self.remaining:
            raise ValueError(f"Payload has {self.remaining} trailing bytes")


def pack_packet(
    message_type: MessageType,
    payload: bytes = b"",
    *,
    packet_number: int = 0,
    reserved: int = 0,
) -> bytes:
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"Payload exceeds {MAX_PAYLOAD_SIZE} bytes")
    return (
        HEADER.pack(
            MAGIC,
            int(message_type),
            reserved,
            len(payload),
            packet_number & 0xFFFFFFFF,
        )
        + payload
    )


def unpack_packet(data: bytes) -> Packet:
    if len(data) < HEADER.size:
        raise ValueError(f"Packet is too short: {len(data)} bytes")

    magic, message_type, reserved, payload_length, packet_number = HEADER.unpack_from(
        data
    )
    if magic != MAGIC:
        raise ValueError(f"Unexpected packet magic: 0x{magic:08X}")
    if payload_length > MAX_PAYLOAD_SIZE:
        raise ValueError(f"Payload is too large: {payload_length} bytes")
    if len(data) != HEADER.size + payload_length:
        raise ValueError(
            f"Invalid packet size: got {len(data)}, expected {HEADER.size + payload_length}"
        )

    try:
        typed_message = MessageType(message_type)
    except ValueError as error:
        raise ValueError(f"Unsupported message type: 0x{message_type:02X}") from error

    return Packet(
        message_type=typed_message,
        reserved=reserved,
        packet_number=packet_number,
        payload=data[HEADER.size :],
    )


def state_command(action: StateAction, packet_number: int = 0) -> bytes:
    return pack_packet(
        MessageType.STATE_COMMAND,
        PayloadWriter().put_u8(action).to_bytes(),
        packet_number=packet_number,
    )


def dfu_command(action: DfuAction, packet_number: int = 0) -> bytes:
    return pack_packet(
        MessageType.DFU_COMMAND,
        PayloadWriter().put_u8(action).to_bytes(),
        packet_number=packet_number,
    )


def get_pid_state_command(packet_number: int = 0) -> bytes:
    return pack_packet(MessageType.GET_PID_STATE, packet_number=packet_number)


def set_pid_command(
    controller: ControllerId,
    kp: float,
    ki: float,
    kd: float,
    packet_number: int = 0,
) -> bytes:
    return pack_packet(
        MessageType.SET_PID,
        PayloadWriter()
        .put_u8(controller)
        .put_float(kp)
        .put_float(ki)
        .put_float(kd)
        .to_bytes(),
        packet_number=packet_number,
    )


def set_setpoint_command(
    controller: ControllerId, value: float, packet_number: int = 0
) -> bytes:
    return pack_packet(
        MessageType.SET_SETPOINT,
        PayloadWriter().put_u8(controller).put_float(value).to_bytes(),
        packet_number=packet_number,
    )


def set_lqr_command(kx: float, ky: float, packet_number: int = 0) -> bytes:
    return pack_packet(
        MessageType.SET_LQR,
        PayloadWriter().put_float(kx).put_float(ky).to_bytes(),
        packet_number=packet_number,
    )


def drive_command(
    angular_speed: float, linear_speed: float, packet_number: int = 0
) -> bytes:
    return pack_packet(
        MessageType.DRIVE_COMMAND,
        PayloadWriter().put_float(angular_speed).put_float(linear_speed).to_bytes(),
        packet_number=packet_number,
    )


def set_mode_command(mode: ControlMode, packet_number: int = 0) -> bytes:
    return pack_packet(
        MessageType.SET_MODE,
        PayloadWriter().put_u8(mode).to_bytes(),
        packet_number=packet_number,
    )
