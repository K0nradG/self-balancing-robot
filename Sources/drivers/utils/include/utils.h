#ifndef UTILS_H_
#define UTILS_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

void
reschedule_work(struct k_work_delayable* dwork, k_timeout_t delay, char* desc);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H_ */