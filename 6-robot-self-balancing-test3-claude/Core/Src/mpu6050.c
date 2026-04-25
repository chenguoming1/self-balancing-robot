#include "mpu6050.h"
#include <math.h>
#include <string.h>

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buf, 2, 10);
}

static HAL_StatusTypeDef read_regs(I2C_HandleTypeDef *hi2c,
                                    uint8_t reg, uint8_t *dst, uint16_t len)
{
    HAL_StatusTypeDef s;
    s = HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, &reg, 1, 10);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, dst, len, 20);
}

bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;
    if (read_regs(hi2c, MPU6050_REG_WHO_AM_I, &who, 1) != HAL_OK) return false;
    if (who != 0x68) return false;

    if (write_reg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) return false;
    if (write_reg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x07) != HAL_OK) return false;
    if (write_reg(hi2c, MPU6050_REG_CONFIG,      0x05) != HAL_OK) return false; // ~10 Hz DLPF
    if (write_reg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00) != HAL_OK) return false; // ±250°/s
    if (write_reg(hi2c, MPU6050_REG_ACCEL_CONFIG,0x00) != HAL_OK) return false; // ±2g

    return true;
}

void MPU6050_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt)
{
    uint8_t raw[14];
    if (read_regs(hi2c, MPU6050_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK) return;

    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);

    float ax_f = (float)ax / ACCEL_SCALE_2G;
    float az_f = (float)az / ACCEL_SCALE_2G;
    data->gyro_rate  = -(float)gy / GYRO_SCALE_250;
    data->accel_angle = atan2f(ax_f, az_f) * (180.0f / (float)M_PI);

    data->angle = COMP_FILTER_ALPHA * (data->angle + data->gyro_rate * dt)
                + (1.0f - COMP_FILTER_ALPHA) * data->accel_angle;
}
