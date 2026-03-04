#include "mpu6050_Caliberate.h"
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

/* Read one set of raw accel + gyro values */
static bool read_raw(I2C_HandleTypeDef *hi2c,
                     int16_t *ax, int16_t *az, int16_t *gy)
{
    uint8_t raw[14];
    if (read_regs(hi2c, MPU6050_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK)
        return false;
    *ax = (int16_t)((raw[0] << 8) | raw[1]);
    *az = (int16_t)((raw[4] << 8) | raw[5]);
    *gy = (int16_t)((raw[8] << 8) | raw[9]);
    return true;
}

/* ── Public API ──────────────────────────────────────────────────────── */

bool MPU6050_Caliberate_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;

    if (read_regs(hi2c, MPU6050_REG_WHO_AM_I, &who, 1) != HAL_OK)
        return false;
    if (who != 0x68) return false;

    // Wake up, use gyro X as clock source
    if (write_reg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) return false;

    // Sample rate: 1000 Hz
    if (write_reg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x07) != HAL_OK) return false;

    // DLPF = 3 → 44 Hz bandwidth
    if (write_reg(hi2c, MPU6050_REG_CONFIG, 0x03) != HAL_OK) return false;

    // Gyro ±250°/s
    if (write_reg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00) != HAL_OK) return false;

    // Accel ±2g
    if (write_reg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00) != HAL_OK) return false;

    return true;
}

void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    /* ── Reset offsets before sampling ──────────────────────────────── */
    data->gyro_offset  = 0.0f;
    data->angle_offset = 0.0f;
    data->angle        = 0.0f;

    float gyro_sum  = 0.0f;
    float angle_sum = 0.0f;
    int   count     = 0;

    /* ── Collect samples ─────────────────────────────────────────────
       Use HAL_Delay between samples so we get real independent readings
       at ~5 ms apart (matching the control loop rate).               */
    for (int i = 0; i < MPU6050_CALIB_SAMPLES; i++) {
        int16_t ax_raw, az_raw, gy_raw;
        if (read_raw(hi2c, &ax_raw, &az_raw, &gy_raw)) {
            float ax_f = (float)ax_raw / ACCEL_SCALE_2G;
            float az_f = (float)az_raw / ACCEL_SCALE_2G;
            float gy_f = (float)gy_raw / GYRO_SCALE_250;

            gyro_sum  += gy_f;
            angle_sum += atan2f(ax_f, az_f) * (180.0f / (float)M_PI);
            count++;
        }
        HAL_Delay(5);   // 5 ms between samples → 200 samples = ~1 second
    }

    if (count == 0) return;   // I2C failed entirely — leave offsets at zero

    /* ── Store averaged offsets ──────────────────────────────────────
       gyro_offset:  the gyro reading when the robot is stationary
                     (should be ~0 but never exactly 0 due to bias)
       angle_offset: the accel angle when the robot is truly upright
                     (corrects for MPU6050 not being mounted perfectly vertical) */
    data->gyro_offset  = gyro_sum  / (float)count;
    data->angle_offset = angle_sum / (float)count;

    /* Seed the complementary filter with the calibrated upright angle
       so it doesn't take several seconds to converge on startup       */
    data->angle = 0.0f;   // after calibration, upright = 0 by definition
}

void MPU6050_Caliberate_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt)
{
    uint8_t raw[14];
    if (read_regs(hi2c, MPU6050_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK)
        return;

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gy = (int16_t)((raw[8] << 8) | raw[9]);

    float ax_f = (float)ax / ACCEL_SCALE_2G;
    float az_f = (float)az / ACCEL_SCALE_2G;

    /* Apply gyro offset (subtract resting bias) */
    data->gyro_rate = (float)gy / GYRO_SCALE_250 - data->gyro_offset;

    /* Accelerometer angle, then subtract mounting offset */
    data->accel_angle = atan2f(ax_f, az_f) * (180.0f / (float)M_PI)
                      - data->angle_offset;

    /* Complementary filter — now works around true zero */
    data->angle = COMP_FILTER_ALPHA * (data->angle + data->gyro_rate * dt)
                + (1.0f - COMP_FILTER_ALPHA) * data->accel_angle;
}

