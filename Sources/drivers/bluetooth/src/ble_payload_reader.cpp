// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_payload_reader.h"
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include "ble_protocol_constants.h"

namespace BLE_Protocol
{

Payload_Reader::Payload_Reader(uint8_t const* data, size_t length)
    : m_data(data),
      m_length(length),
      m_offset(0u),
      m_valid(((data != nullptr) || (length == 0u)) && (length <= MAX_PAYLOAD_SIZE))
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
    value = sys_get_le16(encoded);
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
    value = sys_get_le32(encoded);
    return true;
}

bool
Payload_Reader::get_float(float& value)
{
    uint8_t encoded[ENCODED_FLOAT_SIZE];
    if(!get_bytes(encoded, sizeof(encoded)))
    {
        return false;
    }
    value = get_float_helper(encoded);
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

float
Payload_Reader::get_float_helper(uint8_t const* source)
{
    uint32_t const raw_value = sys_get_le32(source);
    float value;
    memcpy(&value, &raw_value, sizeof(value));
    return value;
}

}  // namespace BLE_Protocol