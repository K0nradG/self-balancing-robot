#include "utils.h"
#include "logger.h"

static Logging::Logger<IS_ENABLED(CONFIG_UTILS_LOG)> utils_logger("UTILS_WORKERS");

void
reschedule_work(k_work_delayable* dwork, k_timeout_t delay, char const* desc)
{
    int const ret = k_work_reschedule(dwork, delay);
    if(ret != 0)
    {
        utils_logger.platform_log(Logging::LOG_LEVEL::ERR, "Can't reschedule %s work, err: %d", desc, ret);
    }
}