#ifndef WATCHDOG_CONTROLLER_H
#define WATCHDOG_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

int
watchdog_controller_init(void);

void
feed_watchdog(void);

#ifdef __cplusplus
}
#endif

#endif  // WATCHDOG_CONTROLLER_H