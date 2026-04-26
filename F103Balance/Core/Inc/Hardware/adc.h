#ifndef __ADC_H
#define __ADC_H
#include "robot_hal.h"

#define Battery_Ch  ADC_CHANNEL_8   /* PB0 - battery voltage divider */

void  Adc_Init(void);
u16   Get_Adc(u32 channel);
float Get_battery_volt(void);

#endif
