// Entry point of the main robot control loop.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void
trigger_control_loop(void);

void
stop_control_loop(void);

#ifdef __cplusplus
}
#endif

#ifdef CONFIG_LOG_OVER_BLE
void
nus_data_parse_callback(char const* data);
#endif  // CONFIG_LOG_OVER_BLE

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"

typedef void (*send_identification_data_cb_t)(identification_data data);

void
new_send_identification_data_cb_register(send_identification_data_cb_t new_send_identification_data_cb);

#endif  // CONFIG_MODEL_IDENTIFICATION_DRV