#include "model_identification.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <cstdint>
#include "logger.h"
#include "main_state_machine.h"
#include "robot_controller.h"

static identification_data g_identification_data {};

void
new_regulator_data_for_identification(identification_data data)
{
    g_identification_data = data;
}

static Logger<IS_ENABLED(CONFIG_MODEL_IDENTIFICATION_LOG)> logger("MODEL");

void
identification_logger_thread(void*, void*, void*)
{
    logger.platform_log(
        LOG_LEVEL::INF, "dt=%.3f angle=%.3f angle_dt=%.3f pwm=%.3f", (double)g_identification_data.dt,
        (double)g_identification_data.angle, (double)g_identification_data.angle_dt, (double)g_identification_data.pwm);

    k_sleep(K_MSEC(CONFIG_MODEL_IDENTIFICATION_SAMPLE_TIME));
}

K_THREAD_DEFINE(ident_logger_tid, 1024, identification_logger_thread, nullptr, nullptr, nullptr, 7, 0, 0);
