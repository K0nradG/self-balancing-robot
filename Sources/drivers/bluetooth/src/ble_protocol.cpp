// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_protocol.h"
#include <string.h>
#include <zephyr/sys/byteorder.h>

namespace BLE_Protocol
{
namespace
{

size_t
get_payload_buffer_size(size_t requested_size)
{
    if(requested_size > max_payload_size)
    {
        return max_payload_size;
    }

    return requested_size;
}

Decode_Result
check_received_frame(uint8_t const* data, size_t length, uint16_t& payload_length)
{
    if((data == nullptr) || (length < header_size))
    {
        return Decode_Result::TOO_SHORT;
    }

    if(get_u32(data) != magic)
    {
        return Decode_Result::INVALID_MAGIC;
    }

    payload_length = get_u16(data + payload_length_offset);
    if((payload_length > max_payload_size) || (length != (header_size + payload_length)))
    {
        return Decode_Result::INVALID_LENGTH;
    }

    return Decode_Result::OK;
}

void
read_received_packet_fields(uint8_t const* data, uint16_t payload_length, received_packet& decoded_packet)
{
    decoded_packet.type           = static_cast<Message_Type>(data[type_offset]);
    decoded_packet.reserved       = data[reserved_offset];
    decoded_packet.payload_length = payload_length;
    decoded_packet.packet_number  = get_u32(data + packet_number_offset);
    decoded_packet.payload        = data + header_size;
}

}  // namespace

Payload_Writer::Payload_Writer(uint8_t* buffer, size_t capacity)
    : m_buffer(buffer), m_capacity(get_payload_buffer_size(capacity)), m_size(0u), m_valid(buffer != nullptr)
{
}

bool
Payload_Writer::put_u8(uint8_t value)
{
    return put_bytes(&value, sizeof(value));
}

bool
Payload_Writer::put_u16(uint16_t value)
{
    uint8_t encoded[sizeof(value)];
    BLE_Protocol::put_u16(encoded, value);
    return put_bytes(encoded, sizeof(encoded));
}

bool
Payload_Writer::put_u32(uint32_t value)
{
    uint8_t encoded[sizeof(value)];
    BLE_Protocol::put_u32(encoded, value);
    return put_bytes(encoded, sizeof(encoded));
}

bool
Payload_Writer::put_float(float value)
{
    uint8_t encoded[encoded_float_size];
    BLE_Protocol::put_float(encoded, value);
    return put_bytes(encoded, sizeof(encoded));
}

bool
Payload_Writer::can_write_bytes(uint8_t const* data, size_t length) const
{
    bool const has_data  = (data != nullptr) || (length == 0u);
    bool const has_space = length <= (m_capacity - m_size);
    return m_valid && has_data && has_space;
}

bool
Payload_Writer::put_bytes(uint8_t const* data, size_t length)
{
    if(!can_write_bytes(data, length))
    {
        m_valid = false;
        return false;
    }

    if(length != 0u)
    {
        memcpy(m_buffer + m_size, data, length);
        m_size += length;
    }
    return true;
}

uint8_t const*
Payload_Writer::data() const
{
    return m_buffer;
}

uint16_t
Payload_Writer::size() const
{
    return static_cast<uint16_t>(m_size);
}

bool
Payload_Writer::valid() const
{
    return m_valid;
}

Payload_Reader::Payload_Reader(uint8_t const* data, size_t length)
    : m_data(data),
      m_length(length),
      m_offset(0u),
      m_valid(((data != nullptr) || (length == 0u)) && (length <= max_payload_size))
{
}

bool
Payload_Reader::get_u8(uint8_t& value)
{
    return get_bytes(&value, sizeof(value));
}

bool
Payload_Reader::get_u16(uint16_t& value)
{
    uint8_t encoded[sizeof(value)];
    if(!get_bytes(encoded, sizeof(encoded)))
    {
        return false;
    }
    value = BLE_Protocol::get_u16(encoded);
    return true;
}

bool
Payload_Reader::get_u32(uint32_t& value)
{
    uint8_t encoded[sizeof(value)];
    if(!get_bytes(encoded, sizeof(encoded)))
    {
        return false;
    }
    value = BLE_Protocol::get_u32(encoded);
    return true;
}

bool
Payload_Reader::get_float(float& value)
{
    uint8_t encoded[encoded_float_size];
    if(!get_bytes(encoded, sizeof(encoded)))
    {
        return false;
    }
    value = BLE_Protocol::get_float(encoded);
    return true;
}

bool
Payload_Reader::can_read_bytes(uint8_t* destination, size_t length) const
{
    bool const has_destination = (destination != nullptr) || (length == 0u);
    bool const has_bytes       = length <= remaining();
    return m_valid && has_destination && has_bytes;
}

bool
Payload_Reader::get_bytes(uint8_t* destination, size_t length)
{
    if(!can_read_bytes(destination, length))
    {
        m_valid = false;
        return false;
    }

    if(length != 0u)
    {
        memcpy(destination, m_data + m_offset, length);
        m_offset += length;
    }
    return true;
}

size_t
Payload_Reader::remaining() const
{
    if(!m_valid)
    {
        return 0u;
    }

    return m_length - m_offset;
}

bool
Payload_Reader::done() const
{
    return m_valid && (m_offset == m_length);
}

bool
Payload_Reader::valid() const
{
    return m_valid;
}

Decode_Result
decode_packet(uint8_t const* data, size_t length, received_packet& decoded_packet)
{
    uint16_t payload_length          = 0u;
    Decode_Result const check_result = check_received_frame(data, length, payload_length);
    if(check_result != Decode_Result::OK)
    {
        return check_result;
    }

    read_received_packet_fields(data, payload_length, decoded_packet);
    return Decode_Result::OK;
}

size_t
encode_header(
    uint8_t* buffer, size_t capacity, Message_Type type, uint16_t payload_length, uint32_t packet_number,
    uint8_t reserved)
{
    size_t const packet_length = header_size + payload_length;
    if((buffer == nullptr) || (payload_length > max_payload_size) || (capacity < packet_length))
    {
        return 0u;
    }

    put_u32(buffer, magic);
    buffer[type_offset]     = static_cast<uint8_t>(type);
    buffer[reserved_offset] = reserved;
    put_u16(buffer + payload_length_offset, payload_length);
    put_u32(buffer + packet_number_offset, packet_number);
    return packet_length;
}

void
put_u16(uint8_t* destination, uint16_t value)
{
    sys_put_le16(value, destination);
}

void
put_u32(uint8_t* destination, uint32_t value)
{
    sys_put_le32(value, destination);
}

void
put_float(uint8_t* destination, float value)
{
    static_assert(sizeof(float) == sizeof(uint32_t));
    uint32_t raw_value;
    memcpy(&raw_value, &value, sizeof(raw_value));
    put_u32(destination, raw_value);
}

uint16_t
get_u16(uint8_t const* source)
{
    return sys_get_le16(source);
}

uint32_t
get_u32(uint8_t const* source)
{
    return sys_get_le32(source);
}

float
get_float(uint8_t const* source)
{
    uint32_t const raw_value = get_u32(source);
    float value;
    memcpy(&value, &raw_value, sizeof(value));
    return value;
}

}  // namespace BLE_Protocol
