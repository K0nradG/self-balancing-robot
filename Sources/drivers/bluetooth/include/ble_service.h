#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*regulator_parameters_parser_cb_t)(const char* data);

int
ble_service_init(void);

void
new_regulator_parameters_parser_cb_register(regulator_parameters_parser_cb_t _regulator_parameters_parser_cb);

void
ble_send(char* data);

bool
get_notif_status();

void
set_notif_status(bool nus_notification_enabled);

#ifdef __cplusplus
}
#endif

#endif /*BLE_SERVICE_H*/
