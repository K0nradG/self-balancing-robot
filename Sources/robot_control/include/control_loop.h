// Entry point of the main robot control loop.

#pragma once

namespace Robot_Control
{

void
trigger_control_loop();

void
stop_control_loop();

int
control_loop_init();

#ifdef CONFIG_BLUETOOTH_DRV
void
nus_data_parse_callback(char const* data);
#endif  // CONFIG_BLUETOOTH_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"

typedef void (*send_identification_data_cb_t)(identification_data data);

void
new_send_identification_data_cb_register(send_identification_data_cb_t new_send_identification_data_cb);

#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

}  // namespace Robot_Control