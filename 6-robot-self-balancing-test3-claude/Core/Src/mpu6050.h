#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define MPU6050_ADDR              (0x68 << 1)

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

#define ACCEL_SCALE_2G    16384.0f
#define GYRO_SCALE_250    131.0f

/* Complementary filter: 0.995 = 99.5% gyro trust, 0.5% accel correction */
#define COMP_FILTER_ALPHA  0.995f

typedef struct {
    float angle;        // fused pitch angle (degrees)
    float gyro_rate;    // gyro Y rate (°/s)
    float accel_angle;  // accelerometer pitch (degrees)
} MPU6050_Data_t;

bool MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt);

#endif /* MPU6050_H */
