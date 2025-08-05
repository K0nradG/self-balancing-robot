#include "supervisor.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include "math.h"

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#endif

#if defined(CONFIG_LOGGER_DRV) && defined(CONFIG_LOG_OVER_BLE)
#include "ble_logger_service.h"
#include "ble_logger_setup.h"
#endif

#ifdef CONFIG_ENCODER_DRV
#include "encoder.h"
#endif

#ifdef CONFIG_IMU_DRV
#include "imu.h"
#endif

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#endif

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
#include "motor_controller.h"
#endif

#ifdef CONFIG_ROBOT_CONTROL
#include "control_loop.h"
#endif

static void __attribute__((unused))
system_reset_on_error(int err)
{
    if(err)
    {
        sys_reboot(SYS_REBOOT_COLD);
    }
}

int
drivers_init(void)

{
    int __attribute__((unused)) err;

#if defined(CONFIG_LOGGER_DRV) && defined(CONFIG_LOG_OVER_BLE)
    err = ble_logger_init();
    system_reset_on_error(err);

    err = ble_logger_service_init();
    system_reset_on_error(err);
#endif

#ifdef CONFIG_IMU_DRV
    err = imu_init();
    system_reset_on_error(err);
#endif

#ifdef ENCODER_DRV
    err = encoders_init();
    system_reset_on_error(err);
#endif

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
    err = motor_controller_init();
    system_reset_on_error(err);
#endif

#ifdef CONFIG_ROBOT_CONTROL
    err = control_loop_init();
#endif

#ifdef CONFIG_BATTERY_LEVEL_DRV
    err = battery_level_init();
    system_reset_on_error(err);
#endif

#ifdef CONFIG_INTERFACE_DRV
    err = interface_init();
    system_reset_on_error(err);
#endif

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    err = model_identification_init();
    system_reset_on_error(err);
#endif
    return 0;
}

SYS_INIT(drivers_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void
safety_supervisor(float current_angle)
{
    if(fabsf(current_angle) > (float)(CONFIG_SAFETY_ANGLE * DEG_TO_RAD))
    {
        set_enable_controller(false);
        set_start_motors(false);
        led_stop_periodic_blinking();
    }
}
