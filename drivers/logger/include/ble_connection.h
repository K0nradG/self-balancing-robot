#ifdef CONFIG_LOG_OVER_BLE
#ifndef BLE_CONNECTION_H
#define BLE_CONNECTION_H

#include <stdbool.h>

bool
get_con_status();

void
set_con_status(bool value);

#endif /*BLE_CONNECTION_H*/
#endif /*CONFIG_LOG_OVER_BLE*/
