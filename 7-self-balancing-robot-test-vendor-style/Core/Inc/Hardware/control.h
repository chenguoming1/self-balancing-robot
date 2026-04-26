#ifndef __CONTROL_H
#define __CONTROL_H
#include "robot_hal.h"

/* Line tracking sensor inputs */
#define C1 PBin(4)
#define C2 PBin(3)
#define C3 PAin(5)
#define C4 PAin(4)

#define PI 3.14159265f

int  balance_UP(float Angle, float Mechanical_balance, float Gyro);
int  velocity(int encoder_left, int encoder_right, int gyro_Z);
int  Turn_UP(int gyro_Z, int RC);
void Tracking(void);
void Get_RC(void);

/* MPU6050 EXTI callback (registered via HAL_GPIO_EXTI_Callback) */
void MPU6050_Control_IRQ(void);

#endif
