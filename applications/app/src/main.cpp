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

#ifdef CONFIG_ENCODER_DRV
#include "encoder.h"
#endif  // CONFIG_ENCODER_DRV

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

#ifdef CONFIG_ENCODER_DRV
void
new_encoder_data_callback(encoders_data encoders_data)
{
    platform_log(
        "APP", LOG_LEVEL_INF, "c%d r%f a%f d%f rs%f ms%f", encoders_data.encoder_0.impulse_count,
        encoders_data.encoder_0.shaft_rotate_count, encoders_data.encoder_0.shaft_angle_rad, encoders_data.encoder_0.distance_m,
        encoders_data.encoder_0.angular_velocity_rad_s, encoders_data.encoder_0.linear_velocity_m_s);
}
#endif  // CONFIG_ENCODER_DRV

#ifdef CONFIG_ROBOT_CONTROL
#include "control_loop.h"

#endif  // CONFIG_ROBOT_CONTROL

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

int
main(void)
{
#ifdef CONFIG_ENCODER_DRV
    new_encoder_data_updated_cb_register(new_encoder_data_callback);
    encoder_start_periodic_data_update();
#endif
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

#ifdef CONFIG_ROBOT_CONTROL
    new_regulator_parameters_parser_cb_register(&nus_data_parse_callback);

#endif  // CONFIG_ROBOT_CONTROL
#endif  // CONFIG_LOG_OVER_BLE

#ifdef CONFIG_ROBOT_CONTROL
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    new_send_identification_data_cb_register(new_regulator_data_for_identification);
    
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Model identification driver is enabled.");
#endif  // CONFIG_APP_LOG

#else
    start_control_loop();
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
#endif  // CONFIG_ROBOT_CONTROL
}
