#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

void Motor_Set(int32_t left_speed, int32_t right_speed);
void Motor_Stop(void);

#endif /* MOTOR_H */
