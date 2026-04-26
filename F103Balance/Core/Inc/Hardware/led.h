#ifndef __LED_H
#define __LED_H
#include "robot_hal.h"

/* LED on PC14, active low */
#define LED PCout(14)

void LED_Init(void);
void Led_Flash(u16 time);

#endif
