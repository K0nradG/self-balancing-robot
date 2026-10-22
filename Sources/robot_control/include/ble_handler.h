// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stddef.h>
#include "ble_protocol_types.h"

namespace Robot_Control
{

class BLE_Handler
{
public:
    static BLE_Handler&
    instance()
    {
        static BLE_Handler s_ble_handler {};
        return s_ble_handler;
    }

    bool
    enqueue_received_packet(BLE_Protocol::Received_Packet const& received_packet);

    void
    process_pending_commands(size_t maximum_commands);

private:
    void
    handle_received_packet(BLE_Protocol::Received_Packet const& received_packet);

    BLE_Handler()                   = default;
    BLE_Handler(BLE_Handler const&) = delete;
    BLE_Handler&
    operator=(BLE_Handler const&) = delete;

    void
    send_command_result(BLE_Protocol::Received_Packet const& received_packet, BLE_Protocol::Command_Status status);
};

}  // namespace Robot_Control