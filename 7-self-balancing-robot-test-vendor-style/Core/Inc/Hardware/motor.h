#ifndef __MOTOR_H
#define __MOTOR_H
#include "robot_hal.h"

/* Match the original vendor motor assignment after rewiring:
 * PWMA -> TIM1_CH1 / PA8
 * PWMB -> TIM1_CH4 / PA11
 * AIN2=PB15 AIN1=PB14 BIN1=PB13 BIN2=PB12
 */
#define PWMA  TIM1->CCR1
#define AIN2  PBout(15)
#define AIN1  PBout(14)
#define BIN1  PBout(13)
#define BIN2  PBout(12)
#define PWMB  TIM1->CCR4

void Motor_Init(void);
void Set_Pwm(int moto1, int moto2);
int  myabs(int a);
void Xianfu_Pwm(void);
void Turn_Off(float angle, float voltage);

#endif
