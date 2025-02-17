#include "utils.h"

#ifdef CONFIG_LOGGER_DRV
#include "logger.h"
#endif

void
reschedule_work(struct k_work_delayable* dwork, k_timeout_t delay, char* desc)
{
    int const ret = k_work_reschedule(dwork, delay);
    if(ret < 0)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_UTLIS_LOG
        platform_log("UTILS_WORKERS", LOG_LEVEL_ERR, "Can't reschedule %s work: %d", desc, ret);
#endif
#endif
    }
}
