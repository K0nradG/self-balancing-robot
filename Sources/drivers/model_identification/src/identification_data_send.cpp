
#include <zephyr/kernel.h>
#include "logger.h"
#include "model_identification.h"

static Logger<IS_ENABLED(CONFIG_MODEL_IDENTIFICATION_LOG)> model_identification_logger("MODEL");

float g_buffer[BUFFER_SIZE] {};

static void
identification_data_sending_work_handler(k_work* work);

static K_WORK_DELAYABLE_DEFINE(identification_data_sending_work, identification_data_sending_work_handler);

static void
identification_data_sending_work_handler(k_work* work)
{
    model_identification_logger.platform_log(LOG_LEVEL::INF, "model data sending start\n");

    for(uint8_t i = 0u; i < BUFFER_COUNT; i++)
    {
        uint16_t len = buffer_get(i, g_buffer, BUFFER_SIZE);

        model_identification_logger.platform_log(LOG_LEVEL::INF, "buffor data: %d\n", i);

        if(len > 0)
        {
            for(uint16_t j = 0; j < len; j++)
            {
                char data_str[16];
                snprintf(data_str, sizeof(data_str), "%.3f\n", (double)g_buffer[j]);

                model_identification_logger.platform_log(LOG_LEVEL::INF, "%s\n", data_str);

                k_sleep(K_MSEC(1));
            }
            model_identification_logger.platform_log(
                LOG_LEVEL::INF, "----------------------------------------------------\n");
        }
    }
    model_identification_logger.platform_log(LOG_LEVEL::INF, "model data sending finished\n");

    notify_data_sent();
}

void
trigger_identification_data_sending()
{
    int const err = k_work_submit(&identification_data_sending_work.work);
    if(err != 0)
    {
        model_identification_logger.platform_log(
            LOG_LEVEL::ERR, "Identification data send trigger failed, err: %d", err);
    }
}
