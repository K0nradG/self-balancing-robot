#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*regulator_parameters_parser_cb_t)(char const* data);
typedef void (*dfu_process_parser_cb_t)(char const* data);
typedef void (*state_machine_commands_parser_cb_t)(char const* data);

int
ble_service_init(void);

void
new_regulator_parameters_parser_cb_register(regulator_parameters_parser_cb_t _regulator_parameters_parser_cb);

void
dfu_process_parser_cb_register(dfu_process_parser_cb_t _dfu_process_parser_cb);

void
state_machine_commands_parser_cb_register(state_machine_commands_parser_cb_t _state_machine_commands_parser_cb);

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
