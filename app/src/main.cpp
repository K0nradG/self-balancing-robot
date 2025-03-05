#pragma GCC diagnostic ignored "-Wdouble-promotion"

#ifdef CONFIG_APP_LOG
#include "logger.h"
#endif // CONFIG_APP_LOG

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#define BLINKING_INTERVAL 500
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_LOG_OVER_BLE
#include "ble_logger_service.h"
#endif // CONFIG_IMU_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#define MEASUREMENT_INTERVAL 9000

static void
new_battery_level_callback(battery_level_data data)
{
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "bat lvl %u", data.battery_level_percent);
    platform_log("APP", LOG_LEVEL_INF, "bat lvl mv %u", data.battery_level_mv);
#endif  // CONFIG_APP_LOG
}
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_REGULATOR_DRV
#include "imu.h"
#include "regulator.h"

void new_pid_regulator_parameters(pid_regulator_parameters data) 
{
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Kp: %f, Ki: %f, Kd: %f, Setpoint: %f", data.K, data.I, data.D, data.setpoint);
#endif  // CONFIG_APP_LOG
}

#endif //CONFIG_REGULATOR_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif // CONFIG_MODEL_IDENTIFICATION_DRV

int
main(void)
{
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Application started.");
#endif  // CONFIG_APP_LOG

#ifdef CONFIG_INTERFACE_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "interface driver is enabled.");
#endif  // CONFIG_APP_LOG
    led_start_periodic_blinking(BLINKING_INTERVAL);
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Battery level driver is enabled.");
#endif  // CONFIG_APP_LOG
    new_battery_level_cb_register(new_battery_level_callback);
    battery_start_periodic_measurement(MEASUREMENT_INTERVAL);
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_LOG_OVER_BLE
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Receiving regulator parameters through NUS.");
#endif  // CONFIG_APP_LOG
#ifdef CONFIG_REGULATOR_DRV
    new_nus_data_received_cb_register(new_nus_parameters_received_for_regulator);
#endif // CONFIG_REGULATOR_DRV
#endif // CONFIG_LOG_OVER_BLE

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Model identification driver is enabled.");
#endif //CONFIG_APP_LOG
    new_imu_cb_register(new_imu_data_for_identification);
    model_identification_start();
#endif //CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_REGULATOR_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Regulator driver is enabled.");
#endif //CONFIG_APP_LOG
    new_imu_cb_register(new_imu_angle_for_regulator);
    regulator_start_automatic_control();
    new_pid_regulator_parameters_cb_register(new_pid_regulator_parameters);
#endif //CONFIG_REGULATOR_DRV
}
