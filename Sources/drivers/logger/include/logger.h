// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once
#include <stdarg.h>
#include <stdint.h>

#ifdef CONFIG_LOG_OVER_SERIAL
#include <zephyr/sys/printk.h>
#endif

#ifdef CONFIG_BLUETOOTH_DRV
#include "ble_service.h"
#endif

// Used by the including modules
#include "zephyr/sys/util_macro.h"

enum LOG_LEVEL
{
    ERR = 0u,
    INF,
    DBG
};

template<bool module_logging_enabled>
class Logger
{
    static constexpr uint8_t LOG_MSG_MAX_SIZE = 244u;

public:
    Logger(const char* module) : m_module(module) {}

    void
    platform_log([[maybe_unused]] int level, [[maybe_unused]] char const* fmt, ...) const
    {
#ifdef CONFIG_LOGGER_DRV
        if constexpr(module_logging_enabled)
        {
            [[maybe_unused]] char log_msg[LOG_MSG_MAX_SIZE] {};
            [[maybe_unused]] const char* level_str = nullptr;
            [[maybe_unused]] va_list args {};

#if defined(CONFIG_LOG_OVER_SERIAL) || defined(CONFIG_BLUETOOTH_DRV)
            va_start(args, fmt);
            vsnprintf(log_msg, sizeof(log_msg), fmt, args);
            va_end(args);

            switch(level)
            {
                case LOG_LEVEL::ERR:
                    level_str = "ERR";
                    break;
                case LOG_LEVEL::INF:
                    level_str = "INF";
                    break;
                case LOG_LEVEL::DBG:
                    level_str = "DBG";
                    break;
                default:
                    level_str = "UNK";
                    break;
            }
#endif  // CONFIG_LOG_OVER_SERIAL || CONFIG_BLUETOOTH_DRV

#ifdef CONFIG_LOG_OVER_SERIAL
            printk("[%s] %s: %s\n", level_str, m_module, log_msg);
#endif  // CONFIG_LOG_OVER_SERIAL

#if defined(CONFIG_BLUETOOTH_DRV) && !defined(CONFIG_MODEL_IDENTIFICATION_DRV)
            ble_send(log_msg);
#endif  // CONFIG_BLUETOOTH_DRV

        }  // module_logging_enabled

#endif  // CONFIG_LOGGER_DRV
    }

private:
    [[maybe_unused]] char const* m_module;
};