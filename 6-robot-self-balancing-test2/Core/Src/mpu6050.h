#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ── I2C Address ─────────────────────────────────────────────────────── */
#define MPU6050_ADDR        (0x68 << 1)   // HAL 8-bit address
#define MPU_ADDR            0x68u         // vendor DMP code uses 7-bit address

/* ── Register Map ────────────────────────────────────────────────────── */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

/* Vendor DMP driver compatibility aliases. */
#define MPU_SMPLRT_DIV            MPU6050_REG_SMPLRT_DIV
#define MPU_CONFIG                MPU6050_REG_CONFIG
#define MPU_GYRO_CONFIG           MPU6050_REG_GYRO_CONFIG
#define MPU_ACCEL_CONFIG          MPU6050_REG_ACCEL_CONFIG
#define MPU_ACCEL_XOUT_H          MPU6050_REG_ACCEL_XOUT_H
#define MPU_GYRO_XOUT_H           MPU6050_REG_GYRO_XOUT_H

/* ── Sensitivity Scales ──────────────────────────────────────────────── */
#define ACCEL_SCALE_2G   16384.0f   // LSB/g   (±2g  range)
#define GYRO_SCALE_250   131.0f     // LSB/°/s (±250°/s range)
#define GYRO_SCALE_500   65.5f     // LSB/°/s (±500°/s range)

typedef struct {
    float angle;        // DMP pitch angle in degrees
    float gyro_rate;    // Raw gyro Y rate (°/s)
    float accel_angle;  // Raw accelerometer-derived pitch (degrees)
    float roll;
    float yaw;
    uint8_t dmp_ready;
} MPU6050_Data_t;

/* ── API ─────────────────────────────────────────────────────────────── */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data);
uint8_t MPU_Read_Byte(uint8_t reg);
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz);
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az);
bool MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * Call this ONLY from the fixed-rate control loop timer ISR (or a function
 * called from it) so that dt is always exactly CONTROL_LOOP_DT.
 */
void MPU6050_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt);

#endif /* MPU6050_H */
