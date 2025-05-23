#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void
led_start_periodic_blinking(uint16_t blinking_interval);

void
led_stop_periodic_blinking(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROL_H */
