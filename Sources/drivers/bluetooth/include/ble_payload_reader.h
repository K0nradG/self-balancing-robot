// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace BLE_Protocol
{

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

    float
    get_float_helper(uint8_t const* source);

    uint8_t const* m_data;
    size_t m_length;
    size_t m_offset;
    bool m_valid;
};

}  // namespace BLE_Protocol