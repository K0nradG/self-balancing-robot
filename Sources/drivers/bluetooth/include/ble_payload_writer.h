// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace BLE_Protocol
{

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

    void
    put_float_helper(float value, uint8_t* destination);

    uint8_t* m_buffer;
    size_t m_capacity;
    size_t m_size;
    bool m_valid;
};

}  // namespace BLE_Protocol