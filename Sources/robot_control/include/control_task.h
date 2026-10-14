// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

namespace Robot_Control
{

int
control_task_init();

void
trigger_control_task();

void
stop_control_task();

#ifdef CONFIG_BLUETOOTH_DRV
void
nus_data_parse_callback(char const* data);
#endif  // CONFIG_BLUETOOTH_DRV

}  // namespace Robot_Control