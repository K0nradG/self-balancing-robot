#pragma GCC diagnostic ignored "-Wdouble-promotion"

#ifdef CONFIG_APP_LOG
#include "logger.h"
#endif  // CONFIG_APP_LOG

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#define BLINKING_INTERVAL 500
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_LOG_OVER_BLE
#include "ble_logger_service.h"
#endif  // CONFIG_IMU_DRV

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

#ifdef CONFIG_PID_ENABLED
#include "pid.h"
void
new_pid_regulator_parameters(pid_regulator_parameters data)
{
#ifdef CONFIG_APP_LOG
    // More than two ints can't be printed with print.
    platform_log("APP", LOG_LEVEL_INF, "Kp: %f, Ki: %f, Kd: %f, Setpoint: %f", data.K, data.I, data.D, data.setpoint);
#endif  // CONFIG_APP_LOG
}
#else
#include "lqr.h"
void
new_lqr_parameters(lqr_parameters data)
{
#ifdef CONFIG_APP_LOG
    // More than two ints can't be printed with print.
    platform_log("APP", LOG_LEVEL_INF, "K1: %f, K2: %f, Setpoint: %f", data.Kx, data.Ky, data.setpoint);
#endif  // CONFIG_APP_LOG
}
#endif // CONFIG_PID_ENABLED

#endif  // CONFIG_REGULATOR_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

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
    new_regulator_parameters_parser_cb_register(parse_regulator_data); // Parser callback definition depends on the regulator type.

#ifdef CONFIG_PID_ENABLED
    new_pid_parameters_cb_register(new_pid_regulator_parameters);
#else
    new_lqr_parameters_cb_register(new_lqr_parameters);
#endif // CONFIG_PID_ENABLED
#endif  // CONFIG_REGULATOR_DRV
#endif  // CONFIG_LOG_OVER_BLE

#ifdef CONFIG_REGULATOR_DRV
    new_imu_cb_register(new_imu_angle_for_regulator); // IMU callback is different when identification is ON or OFF.
    new_calculate_regulator_output_cb_register(calculate_regulator_output); // Regulator output calculation callback depends on the regulator type.
    new_get_setpoint_cb_register(get_setpoint); // Setpoint getter callback depends on the regulator type

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    new_pwm_cb_register(new_regulator_data_for_identification);

#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Model identification driver is enabled.");
#endif  // CONFIG_APP_LOG

#else
    regulator_start_automatic_control();
#endif // CONFIG_MODEL_IDENTIFICATION_DRV
#endif // CONFIG_REGULATOR_DRV
}
