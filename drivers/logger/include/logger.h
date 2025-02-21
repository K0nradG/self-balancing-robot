#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdint.h>

#define LOG_LEVEL_ERR 0
#define LOG_LEVEL_INF 1
#define LOG_LEVEL_DBG 2

#ifdef __cplusplus
extern "C" {
#endif

void
platform_log(const char* module, int level, const char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /*LOGGER_H_*/
