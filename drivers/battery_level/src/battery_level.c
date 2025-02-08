#include <zephyr/kernel.h>
#include "battery_level.h"

const struct device* adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
