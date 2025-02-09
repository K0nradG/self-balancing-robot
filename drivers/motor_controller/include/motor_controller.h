#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum DIRECTION
{
    POSITIVE = 0,
    NEGATIVE
} DIRECTION;

typedef struct MOTORS_DATA
{
    DIRECTION direction;
    int pwm_value;
    bool start;
} MOTORS_DATA;

static void
set_enable_controller(bool enable);

static void
set_start_motor(bool start);

static void
set_direction(DIRECTION direction);

static void
set_pwm_value(int pwm_value);

void
motor_controller_start(uint16_t interval_ms);

void
motor_controller_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H_ */
