#ifndef __TIMER_H
#define __TIMER_H
#include "robot_hal.h"

extern TIM_HandleTypeDef htim4;

void Timer4_Init(u16 arr, u16 psc);

#endif
