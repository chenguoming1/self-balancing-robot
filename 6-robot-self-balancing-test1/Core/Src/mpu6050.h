#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ── I2C Address ─────────────────────────────────────────────────────── */
#define MPU6050_ADDR        (0x68 << 1)   // AD0 = GND → 0x68

/* ── Register Map ────────────────────────────────────────────────────── */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

/* ── Sensitivity Scales ──────────────────────────────────────────────── */
#define ACCEL_SCALE_2G   16384.0f   // LSB/g   (±2g  range)
#define GYRO_SCALE_250   131.0f     // LSB/°/s (±250°/s range)
#define GYRO_SCALE_500   65.5f     // LSB/°/s (±500°/s range)

/* ── Complementary Filter Coefficient ───────────────────────────────── */
// Tune alpha: closer to 1.0 → trust gyro more (less noise, more drift)
//#define COMP_FILTER_ALPHA  0.98f
#define COMP_FILTER_ALPHA  0.995f

typedef struct {
    float angle;        // Fused pitch angle (degrees) — the balance angle
    float gyro_rate;    // Raw gyro Y rate (°/s)
    float accel_angle;  // Accelerometer-derived angle (degrees)
} MPU6050_Data_t;

/* ── API ─────────────────────────────────────────────────────────────── */
bool MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * Call this ONLY from the fixed-rate control loop timer ISR (or a function
 * called from it) so that dt is always exactly CONTROL_LOOP_DT.
 */
void MPU6050_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt);

#endif /* MPU6050_H */
