#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(void);

extern int16_t ax,ay,az;
extern int16_t gx,gy,gz;

#endif
