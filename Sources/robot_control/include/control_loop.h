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

}  // namespace Robot_Control