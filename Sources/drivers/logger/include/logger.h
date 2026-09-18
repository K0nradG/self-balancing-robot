// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#ifdef CONFIG_BLUETOOTH_DRV
#include "ble_transfer_handler.h"
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
#if defined(CONFIG_LOGGER_DRV) && defined(CONFIG_BLUETOOTH_DRV)
        if constexpr(module_logging_enabled)
        {
            char log_msg[LOG_MSG_MAX_SIZE] {};
            va_list args {};

            va_start(args, fmt);
            vsnprintf(log_msg, sizeof(log_msg), fmt, args);
            va_end(args);

            ble_send_log(static_cast<uint8_t>(level), m_module, log_msg);
        }  // module_logging_enabled
#endif  // CONFIG_LOGGER_DRV && CONFIG_BLUETOOTH_DRV
    }

private:
    [[maybe_unused]] char const* m_module;
};