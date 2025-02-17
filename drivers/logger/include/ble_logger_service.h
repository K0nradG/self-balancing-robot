#ifdef CONFIG_LOG_OVER_BLE
#ifndef BLE_LOGGER_SVC_H
#define BLE_LOGGER_SVC_H

#include <stdbool.h>
#include <stdint.h>

void
ble_logger_send(char* data);

#endif /*BLE_LOGGER_SVC_H*/
#endif /*CONFIG_LOG_OVER_BLE*/
