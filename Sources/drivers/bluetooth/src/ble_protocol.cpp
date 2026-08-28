// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_protocol.h"
#include <zephyr/sys/byteorder.h>
#include "ble_protocol_constants.h"

namespace BLE_Protocol
{

// Values from STATE_COMMAND upwards represent packets sent from the app to the robot.
constexpr uint8_t minimum_command_type_value = static_cast<uint8_t>(Message_Type::STATE_COMMAND);

static Decode_Result
check_received_frame(uint8_t const* data, size_t length, uint16_t& payload_length)
{
    if((data == nullptr) || (length < HEADER_SIZE))
    {
        return Decode_Result::TOO_SHORT;
    }

    if(sys_get_le32(data) != MAGIC)
    {
        return Decode_Result::INVALID_MAGIC;
    }

    payload_length = sys_get_le16(data + PACKET_PAYLOAD_LENGTH_OFFSET);
    if((payload_length > MAX_PAYLOAD_SIZE) || (length != (HEADER_SIZE + payload_length)))
    {
        return Decode_Result::INVALID_LENGTH;
    }

    return Decode_Result::OK;
}

static void
read_received_packet_fields(uint8_t const* data, uint16_t payload_length, received_packet& decoded_packet)
{
    decoded_packet.type           = static_cast<Message_Type>(data[PACKET_TYPE_OFFSET]);
    decoded_packet.reserved       = data[PACKER_RESERVED_OFFSET];
    decoded_packet.payload_length = payload_length;
    decoded_packet.packet_number  = sys_get_le32(data + PACKET_NUMBER_OFFSET);
    decoded_packet.payload        = data + HEADER_SIZE;
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
    size_t const packet_length = HEADER_SIZE + payload_length;
    if((buffer == nullptr) || (payload_length > MAX_PAYLOAD_SIZE) || (capacity < packet_length))
    {
        return 0u;
    }

    sys_put_le32(MAGIC, buffer);
    buffer[PACKET_TYPE_OFFSET]     = static_cast<uint8_t>(type);
    buffer[PACKER_RESERVED_OFFSET] = reserved;
    sys_put_le16(payload_length, buffer + PACKET_PAYLOAD_LENGTH_OFFSET);
    sys_put_le32(packet_number, buffer + PACKET_NUMBER_OFFSET);
    return packet_length;
}

bool
has_command_header(uint8_t const* data, uint16_t length)
{
    return (data != nullptr) && (length >= HEADER_SIZE) && (sys_get_le32(data) == MAGIC) &&
           (data[PACKET_TYPE_OFFSET] >= minimum_command_type_value);
}

}  // namespace BLE_Protocol
