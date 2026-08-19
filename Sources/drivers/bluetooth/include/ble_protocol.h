// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace BLE_Protocol
{

constexpr uint32_t magic            = 0x31544252u;  // "RBT1" on little-endian systems.
constexpr size_t header_size        = 12u;
constexpr size_t max_packet_size    = 244u;
constexpr size_t max_payload_size   = max_packet_size - header_size;
constexpr size_t encoded_float_size = 4u;

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
    uint8_t const* m_data;
    size_t m_length;
    size_t m_offset;
    bool m_valid;
};

// Wire format (all integers and IEEE-754 floats are little-endian):
//   envelope: magic u32, type u8, flags u8, payload_length u16, sequence u32
//   telemetry: dropped u32, count u8, reserved[3], count * (timestamp u32 + 10 floats)
//   log: level u8, module_length u8, text_length u16, module UTF-8, text UTF-8
//   battery: millivolts u16, percent u8, reserved u8
//   app version: major u8, minor u8, revision u8, reserved u8, build u32
//   PID state: five controllers * (Kp float, Ki float, Kd float)
//   command result: request_sequence u32, request_type u8, Command_Status u8
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

struct Packet_View
{
    Message_Type type;
    uint8_t flags;
    uint16_t payload_length;
    uint32_t sequence;
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
decode_packet(uint8_t const* data, size_t length, Packet_View& packet);

size_t
encode_header(
    uint8_t* buffer, size_t capacity, Message_Type type, uint16_t payload_length, uint32_t sequence,
    uint8_t flags = 0u);

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
