#ifndef __SR04_H
#define __SR04_H
#include "robot_hal.h"

/* PB10 = TRIG, PB11 = ECHO */
#define TRIG_GPIO_Port  GPIOB
#define TRIG_Pin        GPIO_PIN_10
#define ECHO_GPIO_Port  GPIOB
#define ECHO_Pin        GPIO_PIN_11

void SR04_Configuration(void);
void SR04_StartMeasure(void);
void Timer4_Init(u16 arr, u16 psc);

#endif
