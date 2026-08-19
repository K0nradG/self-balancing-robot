// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_protocol.h"
#include <string.h>
#include <zephyr/sys/byteorder.h>

namespace BLE_Protocol
{

Payload_Writer::Payload_Writer(uint8_t* buffer, size_t capacity)
    : m_buffer(buffer),
      m_capacity(capacity < max_payload_size ? capacity : max_payload_size),
      m_size(0u),
      m_valid(buffer != nullptr)
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
Payload_Writer::put_bytes(uint8_t const* data, size_t length)
{
    if(!m_valid || ((data == nullptr) && (length != 0u)) || (length > (m_capacity - m_size)))
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
Payload_Reader::get_bytes(uint8_t* destination, size_t length)
{
    if(!m_valid || ((destination == nullptr) && (length != 0u)) || (length > remaining()))
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
    return m_valid ? (m_length - m_offset) : 0u;
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
decode_packet(uint8_t const* data, size_t length, Packet_View& packet)
{
    if((data == nullptr) || (length < header_size))
    {
        return Decode_Result::TOO_SHORT;
    }

    if(get_u32(data) != magic)
    {
        return Decode_Result::INVALID_MAGIC;
    }

    uint16_t const payload_length = get_u16(data + 6u);
    if((payload_length > max_payload_size) || (length != (header_size + payload_length)))
    {
        return Decode_Result::INVALID_LENGTH;
    }

    packet.type           = static_cast<Message_Type>(data[4]);
    packet.flags          = data[5];
    packet.payload_length = payload_length;
    packet.sequence       = get_u32(data + 8u);
    packet.payload        = data + header_size;
    return Decode_Result::OK;
}

size_t
encode_header(
    uint8_t* buffer, size_t capacity, Message_Type type, uint16_t payload_length, uint32_t sequence, uint8_t flags)
{
    size_t const packet_length = header_size + payload_length;
    if((buffer == nullptr) || (payload_length > max_payload_size) || (capacity < packet_length))
    {
        return 0u;
    }

    put_u32(buffer, magic);
    buffer[4] = static_cast<uint8_t>(type);
    buffer[5] = flags;
    put_u16(buffer + 6u, payload_length);
    put_u32(buffer + 8u, sequence);
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
