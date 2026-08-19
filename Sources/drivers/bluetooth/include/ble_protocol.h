// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace BLE_Protocol
{

// Identifies an RBT1 packet at the beginning of the header.
constexpr uint32_t magic = 0x31544252u;
// Header field offsets are derived from the sizes of all preceding fields.
constexpr size_t type_offset           = sizeof(magic);
constexpr size_t reserved_offset       = type_offset + sizeof(uint8_t);
constexpr size_t payload_length_offset = reserved_offset + sizeof(uint8_t);
constexpr size_t packet_number_offset  = payload_length_offset + sizeof(uint16_t);
// Total header size: magic, type, reserved byte, payload length, and packet number.
constexpr size_t header_size = packet_number_offset + sizeof(uint32_t);
// The requested ATT MTU and the part occupied by the ATT notification header.
constexpr size_t target_att_mtu              = 247u;
constexpr size_t att_notification_header_size = 3u;
// Largest complete RBT1 packet that fits in one unfragmented notification.
constexpr size_t max_packet_size = target_att_mtu - att_notification_header_size;
// Payload capacity after subtracting the RBT1 header.
constexpr size_t max_payload_size = max_packet_size - header_size;
// Every IEEE-754 float is encoded as one 32-bit little-endian value.
constexpr size_t encoded_float_size = sizeof(uint32_t);

static_assert(packet_number_offset + sizeof(uint32_t) == header_size);

class Payload_Writer
{
public:
    Payload_Writer(uint8_t* buffer, size_t capacity);

    bool
    put_u8(uint8_t value);
    bool
    put_u16(uint16_t value);
    bool
    put_u32(uint32_t value);
    bool
    put_float(float value);
    bool
    put_bytes(uint8_t const* data, size_t length);

    uint8_t const*
    data() const;
    uint16_t
    size() const;
    bool
    valid() const;

private:
    bool
    can_write_bytes(uint8_t const* data, size_t length) const;

    uint8_t* m_buffer;
    size_t m_capacity;
    size_t m_size;
    bool m_valid;
};

class Payload_Reader
{
public:
    Payload_Reader(uint8_t const* data, size_t length);

    bool
    get_u8(uint8_t& value);
    bool
    get_u16(uint16_t& value);
    bool
    get_u32(uint32_t& value);
    bool
    get_float(float& value);
    bool
    get_bytes(uint8_t* destination, size_t length);

    size_t
    remaining() const;
    bool
    done() const;
    bool
    valid() const;

private:
    bool
    can_read_bytes(uint8_t* destination, size_t length) const;

    uint8_t const* m_data;
    size_t m_length;
    size_t m_offset;
    bool m_valid;
};

// RBT1 frame header (all integers and IEEE-754 floats are little-endian):
//   magic          u32  Identifies the packet as RBT1; packets with another value are rejected.
//   type           u8   Selects the message and determines how its payload is decoded.
//   reserved       u8   Reserved for future protocol options; currently always zero.
//   payload_length u16  Number of bytes after the header, used to validate the complete packet.
//   packet_number  u32  Sender-assigned counter used to detect missing or out-of-order packets.
//   payload        ...  Message-specific data containing exactly payload_length bytes.
// The complete frame size is header_size + payload_length.
//
// Message payload formats:
//   telemetry: dropped u32, count u8, reserved[3], count * (timestamp u32 + 10 floats)
//   log: level u8, module_length u8, text_length u16, module UTF-8, text UTF-8
//   battery: millivolts u16, percent u8, reserved u8
//   app version: major u8, minor u8, revision u8, reserved u8, build u32
//   PID state: five controllers * (Kp float, Ki float, Kd float)
//   command result: request_packet_number u32, request_type u8, Command_Status u8
//   state/DFU command: action u8
//   set PID: Controller_Id u8, Kp float, Ki float, Kd float
//   set setpoint: Controller_Id u8, value float
//   trajectory: rotation_degrees float, distance_metres float
//   identification: 10 PWM floats followed by 10 duration floats
//   LQR: Kx float, Ky float
enum class Message_Type : uint8_t
{
    TELEMETRY               = 0x01u,
    LOG                     = 0x02u,
    BATTERY_STATUS          = 0x03u,
    APP_VERSION             = 0x04u,
    PID_STATE               = 0x05u,
    TRAJECTORY_COMPLETE     = 0x06u,
    COMMAND_RESULT          = 0x07u,
    IDENTIFICATION_COMPLETE = 0x08u,
    LQR_STATE               = 0x09u,

    STATE_COMMAND         = 0x20u,
    DFU_COMMAND           = 0x21u,
    GET_PID_STATE         = 0x22u,
    SET_PID               = 0x23u,
    SET_SETPOINT          = 0x24u,
    TRAJECTORY_COMMAND    = 0x25u,
    IDENTIFICATION_CONFIG = 0x26u,
    SET_LQR               = 0x27u,
};

enum class Controller_Id : uint8_t
{
    DISTANCE     = 0u,
    LINEAR_SPEED = 1u,
    BALANCE      = 2u,
    ROTATE       = 3u,
    WHEEL_SPEED  = 4u,
};

enum class State_Action : uint8_t
{
    START = 0u,
    STOP  = 1u,
};

enum class Dfu_Action : uint8_t
{
    START = 0u,
    SKIP  = 1u,
};

enum class Command_Status : uint8_t
{
    OK                  = 0u,
    INVALID_MESSAGE     = 1u,
    INVALID_LENGTH      = 2u,
    INVALID_VALUE       = 3u,
    INVALID_STATE       = 4u,
    UNSUPPORTED_MESSAGE = 5u,
};

// Non-owning, decoded view of one validated RBT1 frame. decode_packet() fills
// these fields from the raw header after validating its magic and length.
// payload points into the caller's raw input buffer and is not copied, so this
// view is valid only while that buffer remains valid. This is not a wire-format
// structure and must not be sent directly over BLE.
struct received_packet
{
    Message_Type type;
    uint8_t reserved;
    uint16_t payload_length;
    uint32_t packet_number;
    uint8_t const* payload;
};

enum class Decode_Result : uint8_t
{
    OK,
    TOO_SHORT,
    INVALID_MAGIC,
    INVALID_LENGTH,
};

Decode_Result
decode_packet(uint8_t const* data, size_t length, received_packet& decoded_packet);

size_t
encode_header(
    uint8_t* buffer, size_t capacity, Message_Type type, uint16_t payload_length, uint32_t packet_number,
    uint8_t reserved = 0u);

void
put_u16(uint8_t* destination, uint16_t value);

void
put_u32(uint8_t* destination, uint32_t value);

void
put_float(uint8_t* destination, float value);

uint16_t
get_u16(uint8_t const* source);

uint32_t
get_u32(uint8_t const* source);

float
get_float(uint8_t const* source);

}  // namespace BLE_Protocol
