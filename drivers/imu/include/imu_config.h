#ifndef IMU_CONFIG_H_
#define IMU_CONFIG_H_

#include <zephyr/drivers/i2c.h>

bool
check_dlpf(const struct device* i2c_dev);

void
set_dlpf(const struct device* i2c_dev);

void
set_mesurement_interval(const struct device* i2c_dev);

#endif /* IMU_CONFIG_H_ */
