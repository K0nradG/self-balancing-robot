#ifdef CONFIG_LOG_OVER_BLE
#ifndef BLE_LOGGER_SVC_H
#define BLE_LOGGER_SVC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*regulator_parameters_parser_cb_t)(const char* data);

void
new_regulator_parameters_parser_cb_register(regulator_parameters_parser_cb_t _regulator_parameters_parser_cb);

void
ble_logger_send(char* data);

bool
get_notif_status();

void
set_notif_status(bool value);

#ifdef __cplusplus
}
#endif

#endif /*BLE_LOGGER_SVC_H*/
#endif /*CONFIG_LOG_OVER_BLE*/
