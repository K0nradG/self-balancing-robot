// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "ble_payload_writer.h"
#include "ble_protocol.h"

typedef void (*ble_packet_received_cb_t)(BLE_Protocol::received_packet const& received_packet);

int
ble_service_init();

void
ble_packet_received_cb_register(ble_packet_received_cb_t callback);

void
ble_dfu_packet_received_cb_register(ble_packet_received_cb_t callback);

int
ble_send_packet(BLE_Protocol::Message_Type type, uint8_t const* payload, uint16_t payload_length);

int
ble_send_packet(BLE_Protocol::Message_Type type, BLE_Protocol::Payload_Writer const& payload);

int
ble_send_telemetry_packet(uint8_t const* payload, uint16_t payload_length);

int
ble_send_telemetry_packet(BLE_Protocol::Payload_Writer const& payload);

int
ble_send_log(uint8_t level, char const* module, char const* message);

bool
get_notif_status();

void
set_notif_status(bool nus_notification_enabled);