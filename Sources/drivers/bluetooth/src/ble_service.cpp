// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_service.h"
#include <bluetooth/services/nus.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include "ble_protocol_constants.h"
#include "ble_protocol_helpers.h"
#include "ble_transfer_handler.h"

bool nus_notification_enabled                   = false;
ble_packet_received_cb_t packet_received_cb     = nullptr;
ble_packet_received_cb_t dfu_packet_received_cb = nullptr;

static void
send_invalid_command_length_result(uint8_t const* command_data)
{
    constexpr size_t COMMAND_RESULT_PAYLOAD_SIZE = sizeof(uint32_t) + (2u * sizeof(uint8_t));
    uint8_t response_payload[COMMAND_RESULT_PAYLOAD_SIZE] {};
    BLE_Protocol::Payload_Writer response_payload_writer(response_payload, sizeof(response_payload));

    uint32_t const request_packet_number = sys_get_le32(command_data + BLE_Protocol::PACKET_NUMBER_OFFSET);
    response_payload_writer.put_u32(request_packet_number);
    response_payload_writer.put_u8(command_data[BLE_Protocol::PACKET_TYPE_OFFSET]);
    response_payload_writer.put_u8(static_cast<uint8_t>(BLE_Protocol::Command_Status::INVALID_LENGTH));

    ble_send_packet(BLE_Protocol::Message_Type::COMMAND_RESULT, response_payload_writer);
}

static void
pass_received_packet_to_callback(BLE_Protocol::Received_Packet const& received_packet)
{
    if(received_packet.type == BLE_Protocol::Message_Type::DFU_COMMAND)
    {
        if(dfu_packet_received_cb != nullptr)
        {
            dfu_packet_received_cb(received_packet);
        }
        return;
    }

    if(packet_received_cb != nullptr)
    {
        packet_received_cb(received_packet);
    }
}

static void
nus_data_received(bt_conn* conn, const uint8_t* data, uint16_t len)
{
    ARG_UNUSED(conn);

    // Holds the decoded fields and payload view of the frame received from NUS.
    BLE_Protocol::Received_Packet received_packet {};
    BLE_Protocol::Decode_Result const result = BLE_Protocol::decode_packet(data, len, received_packet);
    if(result != BLE_Protocol::Decode_Result::OK)
    {
        if(BLE_Protocol::has_command_header(data, len))
        {
            send_invalid_command_length_result(data);
        }
        return;
    }

    pass_received_packet_to_callback(received_packet);
}

static void
nus_notif_enabled(enum bt_nus_send_status status)
{
    if(status == BT_NUS_SEND_STATUS_ENABLED)
    {
        set_notif_status(true);
    }
    else if(status == BT_NUS_SEND_STATUS_DISABLED)
    {
        set_notif_status(false);
    }
}

static bt_nus_cb nus_callbacks = {
    .received     = nus_data_received,
    .send_enabled = nus_notif_enabled,
};

int
ble_service_init()
{
    return bt_nus_init(&nus_callbacks);
}

void
ble_packet_received_cb_register(ble_packet_received_cb_t callback)
{
    packet_received_cb = callback;
}

void
ble_dfu_packet_received_cb_register(ble_packet_received_cb_t callback)
{
    dfu_packet_received_cb = callback;
}

bool
get_notif_status()
{
    return nus_notification_enabled;
}

void
set_notif_status(bool enabled)
{
    nus_notification_enabled = enabled;
}
