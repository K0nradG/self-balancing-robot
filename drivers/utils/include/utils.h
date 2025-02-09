#ifndef UTILS_H_
#define UTILS_H_

#include <zephyr/kernel.h>

void
reschedule_work(struct k_work_delayable* dwork, k_timeout_t delay, char* desc);

#endif /* UTILS_H_ */