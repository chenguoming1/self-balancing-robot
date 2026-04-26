#ifndef __MOTOR_H
#define __MOTOR_H
#include "robot_hal.h"

/* PWM via TIM1 CCR registers (direct register access, same in HAL) */
#define PWMA  TIM1->CCR1   /* PA8  - Motor A PWM */
#define PWMB  TIM1->CCR4   /* PA11 - Motor B PWM */

/* Motor direction pins via bit-banding */
#define AIN2  PBout(15)   /* PB15 */
#define AIN1  PBout(14)   /* PB14 */
#define BIN1  PBout(13)   /* PB13 */
#define BIN2  PBout(12)   /* PB12 */

void Motor_Init(void);
void Set_Pwm(int moto1, int moto2);
int  myabs(int a);
void Xianfu_Pwm(void);
void Turn_Off(float angle, float voltage);

#endif
