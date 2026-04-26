#ifndef __KEY_H
#define __KEY_H
#include "robot_hal.h"

/* KEY button on PC15, active low (pull-up) */
#define KEY PCin(15)

void KEY_Init(void);
u8   KEY_Press(int Ticks);

#endif
