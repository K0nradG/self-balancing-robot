// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include "ble_payload_writer.h"
#include "ble_protocol_types.h"

int
ble_send_packet(BLE_Protocol::Message_Type type, uint8_t const* payload, uint16_t payload_length);

int
ble_send_packet(BLE_Protocol::Message_Type type, BLE_Protocol::Payload_Writer const& payload_writer);

int
ble_send_telemetry_packet(uint8_t const* payload, uint16_t payload_length);

int
ble_send_telemetry_packet(BLE_Protocol::Payload_Writer const& payload_writer);

int
ble_send_log(uint8_t level, char const* module, char const* message);