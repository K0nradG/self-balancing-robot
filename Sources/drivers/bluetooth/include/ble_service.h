// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "ble_protocol_types.h"

typedef void (*ble_packet_received_cb_t)(BLE_Protocol::Received_Packet const& received_packet);

int
ble_service_init();

void
ble_packet_received_cb_register(ble_packet_received_cb_t callback);

void
ble_dfu_packet_received_cb_register(ble_packet_received_cb_t callback);

bool
get_notif_status();

void
set_notif_status(bool nus_notification_enabled);