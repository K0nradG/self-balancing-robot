#include "utils.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(utils, CONFIG_BAT_LVL_LOG_LEVEL);

void
reschedule_work(struct k_work_delayable* dwork, k_timeout_t delay, char* desc)
{
    int const ret = k_work_reschedule(dwork, delay);
    if(ret < 0)
    {
        LOG_ERR("Can't reschedule %s work: %d", desc, ret);
    }
}
