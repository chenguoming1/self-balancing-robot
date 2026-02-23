#include "mpu6050.h"
#include <math.h>
#include <string.h>

/* ── Internal helpers ────────────────────────────────────────────────── */

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c,
                                   uint8_t reg, uint8_t val)
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

/* ── Public API ──────────────────────────────────────────────────────── */

bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;

    // Check WHO_AM_I
    if (read_regs(hi2c, MPU6050_REG_WHO_AM_I, &who, 1) != HAL_OK)
        return false;
    if (who != 0x68) return false;

    // Wake up (clear sleep bit), use gyro X as clock source
    if (write_reg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) return false;

    // Sample rate divider: SMPLRT_DIV = 7 → SR = 8000/(7+1) = 1000 Hz
    if (write_reg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x07) != HAL_OK) return false;

    // DLPF config = 3 → ~44 Hz bandwidth, reduces vibration noise
    if (write_reg(hi2c, MPU6050_REG_CONFIG, 0x03) != HAL_OK) return false;

    // Gyro ±250°/s
    if (write_reg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00) != HAL_OK) return false;

    // Accel ±2g
    if (write_reg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00) != HAL_OK) return false;

    return true;
}

void MPU6050_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt)
{
    uint8_t raw[14];

    // Read 14 bytes: ACCEL_X/Y/Z (6) + TEMP (2) + GYRO_X/Y/Z (6)
    if (read_regs(hi2c, MPU6050_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK)
        return;  // keep last angle on I2C error

    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t gy = (int16_t)((raw[8]  << 8) | raw[9]);

    // Convert
    float ax_f = (float)ax / ACCEL_SCALE_2G;   // g
    float az_f = (float)az / ACCEL_SCALE_2G;   // g
    data->gyro_rate = (float)gy / GYRO_SCALE_250;  // °/s

    // Accelerometer pitch angle (degrees)
    data->accel_angle = atan2f(ax_f, az_f) * (180.0f / (float)M_PI);

    // Complementary filter:  angle = α*(angle + gyro*dt) + (1-α)*accel_angle
    data->angle = COMP_FILTER_ALPHA * (data->angle + data->gyro_rate * dt)
                + (1.0f - COMP_FILTER_ALPHA) * data->accel_angle;
}
