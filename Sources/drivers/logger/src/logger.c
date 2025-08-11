#include "logger.h"
#include <stdarg.h>
#include <stdio.h>

#ifdef CONFIG_LOG_OVER_SERIAL
#include <zephyr/sys/printk.h>
#endif

#ifdef CONFIG_LOG_OVER_BLE
#include "ble_logger_service.h"
#endif

#define LOG_MSG_MAX_SIZE 244 /* safe size to not exceed MTU - MTU takes values between 20 - 517*/

void
platform_log(const char* module, int level, const char* fmt, ...)
{
#if defined(CONFIG_LOG_OVER_SERIAL) || defined(CONFIG_LOG_OVER_BLE)

    char log_msg[LOG_MSG_MAX_SIZE] = {0};
    va_list args                   = {0};

    va_start(args, fmt);
    vsnprintf(log_msg, sizeof(log_msg), fmt, args);
    va_end(args);

    char* level_str = NULL;
    switch(level)
    {
        case LOG_LEVEL_ERR:
            level_str = "ERR";
            break;
        case LOG_LEVEL_INF:
            level_str = "INF";
            break;
        case LOG_LEVEL_DBG:
            level_str = "DBG";
            break;
        default:
            level_str = "UNK";
    }
#endif

#ifdef CONFIG_LOG_OVER_SERIAL
    printk("[%s] %s: %s\n", level_str, module, log_msg);
#endif
#ifdef CONFIG_LOG_OVER_BLE
    ble_logger_send(log_msg);
#endif
}
