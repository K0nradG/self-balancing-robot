// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_protocol.h"
#include <string.h>
#include <zephyr/sys/byteorder.h>

namespace BLE_Protocol
{

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
