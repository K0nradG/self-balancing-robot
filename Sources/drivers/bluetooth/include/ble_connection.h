// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stdbool.h>
#include <stdint.h>

bool
get_con_status();

void
set_con_status(bool connected_flag);

uint16_t
get_att_payload_size();

void
set_att_payload_size(uint16_t payload_size);