#include "drivers_initializer.h"
#include <zephyr/sys/reboot.h>

#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
#include "watchdog_controller.h"
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

#ifdef CONFIG_DFU_BLE
#include "dfu_ble.h"
#endif // CONFIG_DFU_BLE

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
#include "motor_controller.h"
#endif  // CONFIG_MOTOR_CONTROLLER_DRV

#ifdef CONFIG_IMU_DRV
#include "imu.h"
#endif  // CONFIG_IMU_DRV

#ifdef CONFIG_ENCODER_DRV
#include "encoder.h"
#endif  // CONFIG_ENCODER_DRV

#ifdef CONFIG_BLUETOOTH_DRV
#include "ble_service.h"
#include "ble_setup.h"
#endif  // CONFIG_BLUETOOTH_DRV

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_SHELL_DRV
#include "shell.h"
#endif // CONFIG_SHELL_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

void
Drivers_Initializer::init()
{
    int ret = 0;

#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
    ret = watchdog_controller_init();
    //reboot_on_error(ret);
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
    ret = motor_controller_init();
    //reboot_on_error(ret);
#endif  // CONFIG_MOTOR_CONTROLLER_DRV

#ifdef CONFIG_IMU_DRV
    ret = imu_init();
    //reboot_on_error(ret);
#endif  // CONFIG_IMU_DRV

#ifdef CONFIG_ENCODER_DRV
    ret = encoders_init();
    //reboot_on_error(ret);
#endif  // CONFIG_ENCODER_DRV

#ifdef CONFIG_BLUETOOTH_DRV
    ret = ble_init();
    //reboot_on_error(ret);

    ret = ble_service_init();
    //reboot_on_error(ret);

#ifdef CONFIG_DFU_BLE
    ret = dfu_smp_init();
    confirm_new_image();
    start_dfu_smp_adv();
#endif // CONFIG_DFU_BLE

#endif  // CONFIG_BLUETOOTH_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
    ret = battery_level_init();
    //reboot_on_error(ret);
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_INTERFACE_DRV
    ret = interface_init();
    //reboot_on_error(ret);
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_SHELL_DRV
    register_shell_commands();
#endif // CONFIG_SHELL_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    ret = model_identification_init();
    //reboot_on_error(ret);
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
