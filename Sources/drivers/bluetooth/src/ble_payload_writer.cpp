// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_payload_writer.h"
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include "ble_protocol_constants.h"

namespace BLE_Protocol
{

Payload_Writer::Payload_Writer(uint8_t* buffer, size_t capacity)
    : m_buffer(buffer),
      m_capacity((capacity < MAX_PAYLOAD_SIZE) ? capacity : MAX_PAYLOAD_SIZE),
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
    uint8_t destination[sizeof(value)];
    sys_put_le16(value, destination);
    return put_bytes(destination, sizeof(destination));
}

bool
Payload_Writer::put_u32(uint32_t value)
{
    uint8_t destination[sizeof(value)];
    sys_put_le32(value, destination);
    return put_bytes(destination, sizeof(destination));
}

bool
Payload_Writer::put_float(float value)
{
    uint8_t destination[ENCODED_FLOAT_SIZE];
    put_float_helper(value, destination);
    return put_bytes(destination, sizeof(destination));
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

void
Payload_Writer::put_float_helper(float value, uint8_t* destination)
{
    static_assert(sizeof(float) == sizeof(uint32_t));
    uint32_t raw_value {};
    memcpy(&raw_value, &value, sizeof(raw_value));
    sys_put_le32(value, destination);
}

}  // namespace BLE_Protocol