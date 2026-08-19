// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include <bluetooth/services/nus.h>
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "ble_connection.h"
#include "ble_protocol.h"
#include "ble_service.h"

namespace
{

struct Tx_Packet
{
    uint16_t length;
    uint8_t data[BLE_Protocol::max_packet_size];
};

K_MSGQ_DEFINE(protocol_tx_queue, sizeof(Tx_Packet), 8, 4);
K_MSGQ_DEFINE(telemetry_tx_queue, sizeof(Tx_Packet), 8, 4);
K_MSGQ_DEFINE(log_tx_queue, sizeof(Tx_Packet), 8, 4);
K_SEM_DEFINE(tx_available, 0, 24);

bool nus_notification_enabled;
ble_packet_received_cb_t packet_received_cb;
ble_packet_received_cb_t dfu_packet_received_cb;
atomic_t tx_sequence;
atomic_t telemetry_sequence;

int
enqueue_packet(k_msgq* queue, BLE_Protocol::Message_Type type, uint8_t const* payload, uint16_t payload_length)
{
    if(!get_con_status() || !get_notif_status())
    {
        return -ENOTCONN;
    }

    if((payload == nullptr) && (payload_length != 0u))
    {
        return -EINVAL;
    }

    Tx_Packet packet {};
    atomic_t* const sequence_counter = (queue == &telemetry_tx_queue) ? &telemetry_sequence : &tx_sequence;
    uint32_t const sequence          = static_cast<uint32_t>(atomic_inc(sequence_counter));
    packet.length                    = static_cast<uint16_t>(
        BLE_Protocol::encode_header(packet.data, sizeof(packet.data), type, payload_length, sequence));
    if(packet.length == 0u)
    {
        return -EMSGSIZE;
    }
    if(packet.length > get_att_payload_size())
    {
        return -EMSGSIZE;
    }

    if(payload_length != 0u)
    {
        memcpy(packet.data + BLE_Protocol::header_size, payload, payload_length);
    }

    if(k_msgq_put(queue, &packet, K_NO_WAIT) != 0)
    {
        return -ENOMEM;
    }

    k_sem_give(&tx_available);
    return 0;
}

bool
get_next_packet(Tx_Packet& packet)
{
    if(k_msgq_get(&protocol_tx_queue, &packet, K_NO_WAIT) == 0)
    {
        return true;
    }
    if(k_msgq_get(&telemetry_tx_queue, &packet, K_NO_WAIT) == 0)
    {
        return true;
    }
    return k_msgq_get(&log_tx_queue, &packet, K_NO_WAIT) == 0;
}

void
tx_thread(void*, void*, void*)
{
    Tx_Packet packet {};

    while(true)
    {
        k_sem_take(&tx_available, K_FOREVER);
        if(!get_next_packet(packet))
        {
            continue;
        }

        int err;
        do
        {
            err = bt_nus_send(nullptr, packet.data, packet.length);
            if(err == -ENOMEM)
            {
                k_sleep(K_MSEC(1));
            }
        } while(err == -ENOMEM);
    }
}

K_THREAD_DEFINE(ble_tx_thread_id, 2048, tx_thread, nullptr, nullptr, nullptr, 5, 0, 0);

}  // namespace

bool
get_notif_status()
{
    return nus_notification_enabled;
}

void
set_notif_status(bool nus_notification_enabled)
{
    ::nus_notification_enabled = nus_notification_enabled;
}

static void
nus_data_received(bt_conn* conn, const uint8_t* data, uint16_t len)
{
    ARG_UNUSED(conn);

    BLE_Protocol::Packet_View packet {};
    BLE_Protocol::Decode_Result const result = BLE_Protocol::decode_packet(data, len, packet);
    if(result != BLE_Protocol::Decode_Result::OK)
    {
        if((len >= BLE_Protocol::header_size) && (BLE_Protocol::get_u32(data) == BLE_Protocol::magic) &&
           (data[4] >= 0x20u))
        {
            uint8_t response[6] {};
            BLE_Protocol::Payload_Writer writer(response, sizeof(response));
            writer.put_u32(BLE_Protocol::get_u32(data + 8u));
            writer.put_u8(data[4]);
            writer.put_u8(static_cast<uint8_t>(BLE_Protocol::Command_Status::INVALID_LENGTH));
            ble_send_packet(BLE_Protocol::Message_Type::COMMAND_RESULT, writer);
        }
        return;
    }

    if(packet.type == BLE_Protocol::Message_Type::DFU_COMMAND)
    {
        if(dfu_packet_received_cb)
        {
            dfu_packet_received_cb(packet);
        }
        return;
    }

    if(packet_received_cb)
    {
        packet_received_cb(packet);
    }
}

void
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

int
ble_send_packet(BLE_Protocol::Message_Type type, uint8_t const* payload, uint16_t payload_length)
{
    return enqueue_packet(&protocol_tx_queue, type, payload, payload_length);
}

int
ble_send_packet(BLE_Protocol::Message_Type type, BLE_Protocol::Payload_Writer const& payload)
{
    return payload.valid() ? ble_send_packet(type, payload.data(), payload.size()) : -EMSGSIZE;
}

int
ble_send_telemetry_packet(uint8_t const* payload, uint16_t payload_length)
{
    return enqueue_packet(&telemetry_tx_queue, BLE_Protocol::Message_Type::TELEMETRY, payload, payload_length);
}

int
ble_send_telemetry_packet(BLE_Protocol::Payload_Writer const& payload)
{
    return payload.valid() ? ble_send_telemetry_packet(payload.data(), payload.size()) : -EMSGSIZE;
}

int
ble_send_log(uint8_t level, char const* module, char const* message)
{
    if((module == nullptr) || (message == nullptr))
    {
        return -EINVAL;
    }

    size_t const header_length = 4u;
    size_t const max_module_length =
        MIN(static_cast<size_t>(UINT8_MAX), BLE_Protocol::max_payload_size - header_length);
    size_t const module_length  = MIN(strlen(module), max_module_length);
    size_t const message_length = MIN(strlen(message), BLE_Protocol::max_payload_size - header_length - module_length);

    uint8_t payload[BLE_Protocol::max_payload_size] {};
    BLE_Protocol::Payload_Writer writer(payload, sizeof(payload));
    writer.put_u8(level);
    writer.put_u8(static_cast<uint8_t>(module_length));
    writer.put_u16(static_cast<uint16_t>(message_length));
    writer.put_bytes(reinterpret_cast<uint8_t const*>(module), module_length);
    writer.put_bytes(reinterpret_cast<uint8_t const*>(message), message_length);

    return writer.valid() ?
               enqueue_packet(&log_tx_queue, BLE_Protocol::Message_Type::LOG, writer.data(), writer.size()) :
               -EMSGSIZE;
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