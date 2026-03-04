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

/* ── Complementary Filter ────────────────────────────────────────────── */
#define COMP_FILTER_ALPHA  0.98f

/* ── Calibration sample count ────────────────────────────────────────── *
 * Robot must be held perfectly still and upright during this period.     *
 * 200 samples × 5 ms = 1 second of averaging at startup.                */
#define MPU6050_CALIB_SAMPLES  200

typedef struct {
    float angle;        // Fused pitch angle (degrees) — already offset-corrected
    float gyro_rate;    // Raw gyro Y rate (°/s)       — already offset-corrected
    float accel_angle;  // Accelerometer-derived angle (degrees)

    /* Calibration offsets — set by MPU6050_Calibrate(), applied every update */
    float gyro_offset;   // gyro bias at rest (°/s)
    float angle_offset;  // accel angle bias when upright (degrees)
} MPU6050_Data_t;

/* ── API ─────────────────────────────────────────────────────────────── */

/** Initialise sensor registers. Returns false if sensor not found. */
bool MPU6050_Caliberate_Init(I2C_HandleTypeDef *hi2c);

/**
 * Calibrate gyro bias and angle offset.
 *
 * Hold the robot perfectly STILL and UPRIGHT before calling this.
 * Takes CALIB_SAMPLES readings (~1 second) and stores the average
 * as the zero reference. After this call, angle=0 means truly upright.
 *
 * Call this ONCE in main() after MPU6050_Init() and before starting
 * the control loop timer.
 */
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

/**
 * Call from the fixed-rate control loop ISR every dt seconds.
 * Applies calibration offsets automatically.
 */
void MPU6050_Caliberate_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt);

#endif /* MPU6050_H */

