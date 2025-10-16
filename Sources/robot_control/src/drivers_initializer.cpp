#include "drivers_initializer.h"
#include <zephyr/sys/reboot.h>

#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
#include "watchdog_controller.h"
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
#include "motor_controller.h"
#endif  // CONFIG_MOTOR_CONTROLLER_DRV

#ifdef CONFIG_IMU_DRV
#include "imu.h"
#endif  // CONFIG_IMU_DRV

#ifdef CONFIG_ENCODER_DRV
#include "encoder.h"
#endif  // CONFIG_ENCODER_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

void
Drivers_Initializer::init()
{
    int ret = 0;

#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
    ret = watchdog_controller_init();
    reboot_on_error(ret);
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
    ret = motor_controller_init();
    reboot_on_error(ret);
#endif  // CONFIG_MOTOR_CONTROLLER_DRV

#ifdef CONFIG_IMU_DRV
    ret = imu_init();
    reboot_on_error(ret);
#endif  // CONFIG_IMU_DRV

#ifdef CONFIG_ENCODER_DRV
    ret = encoders_init();
    reboot_on_error(ret);
#endif  // CONFIG_ENCODER_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
    ret = battery_level_init();
    reboot_on_error(ret);
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    ret = model_identification_init();
    reboot_on_error(ret);
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
}

void
Drivers_Initializer::reboot_on_error(int error)
{
    if(error != 0)
    {
        sys_reboot(SYS_REBOOT_COLD);
    }
}
