#include "motor.h"

/* Motor direction GPIO already configured by MX_GPIO_Init (PB12-15).
 * PWM channels started in main.c after MX_TIM1_Init via HAL_TIM_PWM_Start.
 */
void Motor_Init(void)
{
    AIN1 = 0; AIN2 = 0;
    BIN1 = 0; BIN2 = 0;
    PWMA = 0;
    PWMB = 0;
}

void Set_Pwm(int moto1, int moto2)
{
    if (moto1 < 0) { AIN2 = 1; AIN1 = 0; }
    else           { AIN2 = 0; AIN1 = 1; }
    PWMA = myabs(moto1);

    if (moto2 < 0) { BIN1 = 0; BIN2 = 1; }
    else           { BIN1 = 1; BIN2 = 0; }
    PWMB = myabs(moto2);
}

int myabs(int a)
{
    return (a < 0) ? -a : a;
}

void Xianfu_Pwm(void)
{
    if (Moto1 < -7000) Moto1 = -7000;
    if (Moto1 >  7000) Moto1 =  7000;
    if (Moto2 < -7000) Moto2 = -7000;
    if (Moto2 >  7000) Moto2 =  7000;
}

void Turn_Off(float angle, float voltage)
{
    if (angle < -40 || angle > 40 || voltage < 11.1f) {
        Moto1 = 0;
        Moto2 = 0;
    }
}
