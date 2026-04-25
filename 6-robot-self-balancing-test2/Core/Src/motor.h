#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

/* MOTOR_PWM_MAX is defined in main.h (999).
   Speed range passed to Motor_Set: -999 … +999               */

/**
 * Drive both motors independently.
 * left_speed / right_speed: -MOTOR_PWM_MAX … +MOTOR_PWM_MAX
 *   positive → forward, negative → reverse, 0 → short brake
 */
void Motor_Set(int32_t left_speed, int32_t right_speed);

/** Short-brake both motors immediately */
void Motor_Stop(void);

#endif /* MOTOR_H */
