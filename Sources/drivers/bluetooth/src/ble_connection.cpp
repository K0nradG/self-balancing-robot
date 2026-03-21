// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_connection.h"
#include <stdbool.h>

static bool g_connected_flag = false;

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
