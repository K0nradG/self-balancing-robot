// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_connection.h"
#include <stdbool.h>
#include <zephyr/sys/atomic.h>

static bool g_connected_flag       = false;
static atomic_t g_att_payload_size = ATOMIC_INIT(20);

bool
get_con_status()
{
    return g_connected_flag;
}

void
set_con_status(bool connected_flag)
{
    g_connected_flag = connected_flag;
}

uint16_t
get_att_payload_size()
{
    return static_cast<uint16_t>(atomic_get(&g_att_payload_size));
}

void
set_att_payload_size(uint16_t payload_size)
{
    atomic_set(&g_att_payload_size, payload_size);
}
