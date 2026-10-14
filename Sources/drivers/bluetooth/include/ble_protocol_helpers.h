// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include "ble_protocol_types.h"

namespace BLE_Protocol
{

Decode_Result
decode_packet(uint8_t const* data, size_t length, Received_Packet& decoded_packet);

size_t
encode_header(
    uint8_t* buffer, size_t capacity, Message_Type type, uint16_t payload_length, uint32_t packet_number,
    uint8_t reserved = 0u);

bool
has_command_header(uint8_t const* data, uint16_t length);

}  // namespace BLE_Protocol
