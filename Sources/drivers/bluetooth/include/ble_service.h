#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

typedef void (*regulator_parameters_parser_cb_t)(char const* data);
typedef void (*dfu_process_parser_cb_t)(char const* data);
typedef void (*state_machine_commands_parser_cb_t)(char const* data);

int
ble_service_init();

void
new_regulator_parameters_parser_cb_register(regulator_parameters_parser_cb_t _regulator_parameters_parser_cb);

void
dfu_process_parser_cb_register(dfu_process_parser_cb_t _dfu_process_parser_cb);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
void
identification_process_parser_cb_register(identification_process_cb_t _identification_process_parser_cb);
#endif

void
state_machine_commands_parser_cb_register(state_machine_commands_parser_cb_t _state_machine_commands_parser_cb);

void
ble_send(char const* data);

bool
get_notif_status();

void
set_notif_status(bool nus_notification_enabled);