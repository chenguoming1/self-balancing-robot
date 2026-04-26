#ifndef __ENCODER_H
#define __ENCODER_H
#include "robot_hal.h"

#define ENCODER_TIM_PERIOD (uint16_t)(65535)

void Encoder_Start(void);
int  Read_Encoder(u8 TIMX);

#endif
