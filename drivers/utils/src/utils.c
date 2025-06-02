#include "utils.h"

#ifdef CONFIG_UTILS_LOG
#include "logger.h"
#else
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif  // CONFIG_UTILS_LOG

void
reschedule_work(struct k_work_delayable* dwork, k_timeout_t delay, char* desc)
{
    int const ret = k_work_reschedule(dwork, delay);
#ifdef CONFIG_UTILS_LOG
    if(ret < 0)
    {
        platform_log("UTILS_WORKERS", LOG_LEVEL_ERR, "Can't reschedule %s work: %d", desc, ret);
    }
#endif  // CONFIG_UTILS_LOG
}