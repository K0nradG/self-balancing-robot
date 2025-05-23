
#include <zephyr/kernel.h>
#include "ble_logger_service.h"
#include "logger.h"
#include "model_identification.h"

float g_buffer[BUFFER_SIZE] = {0};
static struct k_work_delayable identification_data_sending_work;

static void
identification_data_sending_work_handler(struct k_work* work)
{
    platform_log("MODEL", LOG_LEVEL_INF, "model data sending start\n");

    for(uint8_t i = 0u; i < BUFFER_COUNT; i++)
    {
        uint16_t len = buffer_get(i, g_buffer, BUFFER_SIZE);

        platform_log("MODEL", LOG_LEVEL_INF, "buffor data: %d\n", i);

        if(len > 0)
        {
            for(uint16_t j = 0; j < len; j++)
            {
                char data_str[16];
                snprintf(data_str, sizeof(data_str), "%.3f\n", (double)g_buffer[j]);

                platform_log("MODEL", LOG_LEVEL_INF, "%s\n", data_str);

                k_sleep(K_MSEC(1));
            }
            platform_log("MODEL", LOG_LEVEL_INF, "----------------------------------------------------\n");
        }
    }
    platform_log("MODEL", LOG_LEVEL_INF, "model data sending finished\n");

    notify_data_sent();
}

static K_WORK_DELAYABLE_DEFINE(identification_data_sending_work, identification_data_sending_work_handler);

void
trigger_identification_data_sending(void)
{
    k_work_submit(&identification_data_sending_work.work);
}
