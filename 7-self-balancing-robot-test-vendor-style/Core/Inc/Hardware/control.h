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
void Control_ResetState(void);

extern float Mechanical_angle;
extern float Target_Speed;
extern float Turn_Speed;
extern float balance_UP_KP;
extern float balance_UP_KD;
extern float velocity_KP;
extern float velocity_KI;
extern float Turn_Kd;
extern float Turn_KP;
extern int Balance_Pwm;
extern int Velocity_Pwm;
extern int Turn_Pwm;
extern short gyroz;

/* MPU6050 EXTI callback (registered via HAL_GPIO_EXTI_Callback) */
void MPU6050_Control_IRQ(void);

#endif
