#include "mpu6050.h"
#include "inv_mpu.h"
#include <math.h>

#define DMP_PITCH_SIGN     (-1.0f)
#define DMP_GYRO_Y_SIGN    (-1.0f)

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) {
        return 0u;
    }
    return 1u;
}

uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) {
        return 0u;
    }
    return 1u;
}

uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    return MPU_Write_Len(MPU_ADDR, reg, 1u, &data);
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t value = 0u;
    (void)MPU_Read_Len(MPU_ADDR, reg, 1u, &value);
    return value;
}

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CONFIG, (uint8_t)(fsr << 3));
}

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CONFIG, (uint8_t)(fsr << 3));
}

uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0u;
    if      (lpf >= 188u) data = 1u;
    else if (lpf >= 98u)  data = 2u;
    else if (lpf >= 42u)  data = 3u;
    else if (lpf >= 20u)  data = 4u;
    else if (lpf >= 10u)  data = 5u;
    else                  data = 6u;
    return MPU_Write_Byte(MPU_CONFIG, data);
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000u) rate = 1000u;
    if (rate < 4u)    rate = 4u;
    data = (uint8_t)(1000u / rate - 1u);
    MPU_Write_Byte(MPU_SMPLRT_DIV, data);
    return MPU_Set_LPF((uint16_t)(rate / 2u));
}

short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    MPU_Read_Len(MPU_ADDR, 0x41, 2u, buf);
    raw = (short)(((uint16_t)buf[0] << 8) | buf[1]);
    return (short)(36.53f + (float)raw / 340.0f) * 100;
}

uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6];
    uint8_t res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUT_H, 6u, buf);
    if (res == 0u) {
        *gx = (short)(((uint16_t)buf[0] << 8) | buf[1]);
        *gy = (short)(((uint16_t)buf[2] << 8) | buf[3]);
        *gz = (short)(((uint16_t)buf[4] << 8) | buf[5]);
    }
    return res;
}

uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6];
    uint8_t res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUT_H, 6u, buf);
    if (res == 0u) {
        *ax = (short)(((uint16_t)buf[0] << 8) | buf[1]);
        *ay = (short)(((uint16_t)buf[2] << 8) | buf[3]);
        *az = (short)(((uint16_t)buf[4] << 8) | buf[5]);
    }
    return res;
}

static float accel_pitch_from_raw(short ax, short az)
{
    float ax_f = (float)ax / ACCEL_SCALE_2G;
    float az_f = (float)az / ACCEL_SCALE_2G;
    return atan2f(ax_f, az_f) * (180.0f / (float)M_PI);
}

bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;

    if (mpu_dmp_init() != 0) {
        return false;
    }

    return true;
}

void MPU6050_Update(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, float dt)
{
    short gx = 0, gy = 0, gz = 0;
    short ax = 0, ay = 0, az = 0;
    float pitch = data->angle;
    float roll = data->roll;
    float yaw = data->yaw;

    (void)hi2c;
    (void)dt;

    if (MPU_Get_Gyroscope(&gx, &gy, &gz) == 0) {
        /* Keep the same sign convention the old complementary-filter balance
           loop was tuned for, even though the DMP frame comes out flipped. */
        data->gyro_rate = DMP_GYRO_Y_SIGN * ((float)gy / 16.4f);
    }

    if (MPU_Get_Accelerometer(&ax, &ay, &az) == 0) {
        (void)ay;
        data->accel_angle = accel_pitch_from_raw(ax, az);
    }

    if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
        data->angle = DMP_PITCH_SIGN * pitch;
        data->roll = roll;
        data->yaw = yaw;
        data->dmp_ready = 1u;
    }
}
