// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include <bluetooth/services/nus.h>
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include "ble_connection.h"
#include "ble_protocol_constants.h"
#include "ble_protocol_helpers.h"
#include "ble_service.h"

namespace
{

struct tx_packet
{
    uint16_t length;
    uint8_t data[BLE_Protocol::MAX_PACKET_SIZE];
};

constexpr size_t log_payload_header_size = (2u * sizeof(uint8_t)) + sizeof(uint16_t);

// Separate queues prevent telemetry and log bursts from delaying protocol responses.
constexpr size_t TX_QUEUE_CAPACITY = 8u;
K_MSGQ_DEFINE(protocol_tx_queue, sizeof(tx_packet), TX_QUEUE_CAPACITY, alignof(tx_packet));
K_MSGQ_DEFINE(telemetry_tx_queue, sizeof(tx_packet), TX_QUEUE_CAPACITY, alignof(tx_packet));
K_MSGQ_DEFINE(log_tx_queue, sizeof(tx_packet), TX_QUEUE_CAPACITY, alignof(tx_packet));

// One semaphore count represents one packet waiting across all three queues.
constexpr unsigned int INITIAL_TX_PENDING_PACKETS = 0u;
constexpr size_t TX_QUEUE_COUNT                   = 3u;
constexpr size_t MAX_PENDING_TX_PACKETS           = TX_QUEUE_COUNT * TX_QUEUE_CAPACITY;
K_SEM_DEFINE(tx_available, INITIAL_TX_PENDING_PACKETS, MAX_PENDING_TX_PACKETS);

constexpr size_t BLE_TX_THREAD_STACK_SIZE      = 2048u;
constexpr int BLE_TX_THREAD_PRIORITY           = 5;
constexpr uint32_t BLE_TX_THREAD_OPTIONS       = 0u;
constexpr int32_t BLE_TX_THREAD_START_DELAY_MS = 0;

bool nus_notification_enabled                   = false;
ble_packet_received_cb_t packet_received_cb     = nullptr;
ble_packet_received_cb_t dfu_packet_received_cb = nullptr;
// Telemetry has an independent packet-number stream for gap detection.
atomic_t tx_packet_number;
atomic_t telemetry_packet_number;

uint32_t
increment_packet_number(atomic_t& counter)
{
    return static_cast<uint32_t>(atomic_inc(&counter));
}

uint32_t
get_next_packet_number(k_msgq const* queue)
{
    if(queue == &telemetry_tx_queue)
    {
        return increment_packet_number(telemetry_packet_number);
    }

    return increment_packet_number(tx_packet_number);
}

int
check_packet_before_queue(uint8_t const* payload, uint16_t payload_length)
{
    if(!get_con_status() || !get_notif_status())
    {
        return -ENOTCONN;
    }

    if((payload == nullptr) && (payload_length != 0u))
    {
        return -EINVAL;
    }

    return 0;
}

int
build_tx_packet(
    tx_packet& packet_to_queue, k_msgq const* queue, BLE_Protocol::Message_Type type, uint8_t const* payload,
    uint16_t payload_length)
{
    uint32_t const packet_number = get_next_packet_number(queue);
    size_t const packet_length   = BLE_Protocol::encode_header(
        packet_to_queue.data, sizeof(packet_to_queue.data), type, payload_length, packet_number);

    if((packet_length == 0u) || (packet_length > get_att_payload_size()))
    {
        return -EMSGSIZE;
    }
    packet_to_queue.length = static_cast<uint16_t>(packet_length);

    if(payload_length != 0u)
    {
        memcpy(packet_to_queue.data + BLE_Protocol::HEADER_SIZE, payload, payload_length);
    }

    return 0;
}

int
put_packet_in_queue(k_msgq* queue, BLE_Protocol::Message_Type type, uint8_t const* payload, uint16_t payload_length)
{
    int const check_result = check_packet_before_queue(payload, payload_length);
    if(check_result != 0)
    {
        return check_result;
    }

    tx_packet packet_to_queue {};
    int const build_result = build_tx_packet(packet_to_queue, queue, type, payload, payload_length);
    if(build_result != 0)
    {
        return build_result;
    }

    if(k_msgq_put(queue, &packet_to_queue, K_NO_WAIT) != 0)
    {
        return -ENOMEM;
    }

    k_sem_give(&tx_available);
    return 0;
}

bool
get_next_packet(tx_packet& packet_to_send)
{
    // Always send protocol responses before telemetry, and telemetry before logs.
    if(k_msgq_get(&protocol_tx_queue, &packet_to_send, K_NO_WAIT) == 0)
    {
        return true;
    }
    if(k_msgq_get(&telemetry_tx_queue, &packet_to_send, K_NO_WAIT) == 0)
    {
        return true;
    }
    return k_msgq_get(&log_tx_queue, &packet_to_send, K_NO_WAIT) == 0;
}

void
send_packet_to_nus(tx_packet const& packet_to_send)
{
    int send_result;
    do
    {
        send_result = bt_nus_send(nullptr, packet_to_send.data, packet_to_send.length);
        if(send_result == -ENOMEM)
        {
            // Wait briefly until the Bluetooth stack releases a TX buffer.
            k_sleep(K_MSEC(1));
        }
    } while(send_result == -ENOMEM);
}

void
tx_thread(void*, void*, void*)
{
    tx_packet packet_to_send {};

    while(true)
    {
        k_sem_take(&tx_available, K_FOREVER);
        if(!get_next_packet(packet_to_send))
        {
            continue;
        }

        send_packet_to_nus(packet_to_send);
    }
}

K_THREAD_DEFINE(
    ble_tx_thread_id, BLE_TX_THREAD_STACK_SIZE, tx_thread, nullptr, nullptr, nullptr, BLE_TX_THREAD_PRIORITY,
    BLE_TX_THREAD_OPTIONS, BLE_TX_THREAD_START_DELAY_MS);

}  // namespace

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
    return put_packet_in_queue(&protocol_tx_queue, type, payload, payload_length);
}

int
ble_send_packet(BLE_Protocol::Message_Type type, BLE_Protocol::Payload_Writer const& payload)
{
    if(!payload.valid())
    {
        return -EMSGSIZE;
    }

    return ble_send_packet(type, payload.data(), payload.size());
}

int
ble_send_telemetry_packet(uint8_t const* payload, uint16_t payload_length)
{
    return put_packet_in_queue(&telemetry_tx_queue, BLE_Protocol::Message_Type::TELEMETRY, payload, payload_length);
}

int
ble_send_telemetry_packet(BLE_Protocol::Payload_Writer const& payload)
{
    if(!payload.valid())
    {
        return -EMSGSIZE;
    }

    return ble_send_telemetry_packet(payload.data(), payload.size());
}

static size_t
get_log_module_length(char const* module)
{
    size_t const available_length = BLE_Protocol::MAX_PAYLOAD_SIZE - log_payload_header_size;
    return MIN(strlen(module), MIN(static_cast<size_t>(UINT8_MAX), available_length));
}

static size_t
get_log_message_length(char const* message, size_t module_length)
{
    size_t const available_length = BLE_Protocol::MAX_PAYLOAD_SIZE - log_payload_header_size - module_length;
    return MIN(strlen(message), available_length);
}

static void
write_log_payload(
    BLE_Protocol::Payload_Writer& log_payload_writer, uint8_t level, char const* module, size_t module_length,
    char const* message, size_t message_length)
{
    log_payload_writer.put_u8(level);
    log_payload_writer.put_u8(static_cast<uint8_t>(module_length));
    log_payload_writer.put_u16(static_cast<uint16_t>(message_length));
    log_payload_writer.put_bytes(reinterpret_cast<uint8_t const*>(module), module_length);
    log_payload_writer.put_bytes(reinterpret_cast<uint8_t const*>(message), message_length);
}

int
ble_send_log(uint8_t level, char const* module, char const* message)
{
    if((module == nullptr) || (message == nullptr))
    {
        return -EINVAL;
    }

    size_t const module_length  = get_log_module_length(module);
    size_t const message_length = get_log_message_length(message, module_length);

    uint8_t log_payload[BLE_Protocol::MAX_PAYLOAD_SIZE] {};
    BLE_Protocol::Payload_Writer log_payload_writer(log_payload, sizeof(log_payload));
    write_log_payload(log_payload_writer, level, module, module_length, message, message_length);

    if(!log_payload_writer.valid())
    {
        return -EMSGSIZE;
    }

    return put_packet_in_queue(
        &log_tx_queue, BLE_Protocol::Message_Type::LOG, log_payload_writer.data(), log_payload_writer.size());
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