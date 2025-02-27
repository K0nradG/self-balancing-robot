#ifdef CONFIG_LOG_OVER_BLE
#ifndef BLE_LOGGER_SVC_H
#define BLE_LOGGER_SVC_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*nus_data_recived_cb_t)(const uint8_t* data, uint16_t len);

void
new_nus_data_recived_cb_register(nus_data_recived_cb_t _nus_data_recived_cb);

void
ble_logger_send(char* data);

#endif /*BLE_LOGGER_SVC_H*/
#endif /*CONFIG_LOG_OVER_BLE*/
