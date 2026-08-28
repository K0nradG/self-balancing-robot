// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace BLE_Protocol
{

// Identifies an RBT1 packet at the beginning of the header.
constexpr uint32_t MAGIC = 0x31544252u;

// Header field offsets are derived from the sizes of all preceding fields.
constexpr size_t PACKET_TYPE_OFFSET           = sizeof(MAGIC);
constexpr size_t PACKER_RESERVED_OFFSET       = PACKET_TYPE_OFFSET + sizeof(uint8_t);
constexpr size_t PACKET_PAYLOAD_LENGTH_OFFSET = PACKER_RESERVED_OFFSET + sizeof(uint8_t);
constexpr size_t PACKET_NUMBER_OFFSET         = PACKET_PAYLOAD_LENGTH_OFFSET + sizeof(uint16_t);

// Total header size: magic, type, reserved byte, payload length, and packet number.
constexpr size_t HEADER_SIZE = PACKET_NUMBER_OFFSET + sizeof(uint32_t);

// The requested ATT MTU and the part occupied by the ATT notification header.
constexpr size_t TARGET_ATT_MTU               = 247u;
constexpr size_t ATT_NOTIFICATION_HEADER_SIZE = 3u;

// Largest complete RBT1 packet that fits in one non-fragmented notification.
constexpr size_t MAX_PACKET_SIZE = TARGET_ATT_MTU - ATT_NOTIFICATION_HEADER_SIZE;

// Payload capacity after subtracting the RBT1 header.
constexpr size_t MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - HEADER_SIZE;

// Every IEEE-754 float is encoded as one 32-bit little-endian value.
constexpr size_t ENCODED_FLOAT_SIZE = sizeof(uint32_t);

static_assert((PACKET_NUMBER_OFFSET + sizeof(uint32_t)) == HEADER_SIZE);

}  // namespace BLE_Protocol