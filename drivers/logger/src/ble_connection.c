#include "ble_connection.h"
#include <stdbool.h>

static bool connected_flag;

bool
get_con_status()
{
    return connected_flag;
}

void
set_con_status(bool value)
{
    connected_flag = value;
}
