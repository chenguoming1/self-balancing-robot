#include "mpu6050.h"

#define MPU_ADDR 0x68<<1

static I2C_HandleTypeDef *mpu_i2c;

int16_t ax,ay,az;
int16_t gx,gy,gz;

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    mpu_i2c = hi2c;

    uint8_t data;

    HAL_Delay(100);

    data = 0x00;
    HAL_I2C_Mem_Write(mpu_i2c,MPU_ADDR,0x6B,1,&data,1,100);

    data = 0x00;
    HAL_I2C_Mem_Write(mpu_i2c,MPU_ADDR,0x1B,1,&data,1,100);

    data = 0x00;
    HAL_I2C_Mem_Write(mpu_i2c,MPU_ADDR,0x1C,1,&data,1,100);
}

void MPU6050_Read_All()
{
    uint8_t buffer[14];

    HAL_I2C_Mem_Read(mpu_i2c,
                     MPU_ADDR,
                     0x3B,
                     1,
                     buffer,
                     14,
                     100);

    ax = buffer[0]<<8 | buffer[1];
    ay = buffer[2]<<8 | buffer[3];
    az = buffer[4]<<8 | buffer[5];

    gx = buffer[8]<<8 | buffer[9];
    gy = buffer[10]<<8 | buffer[11];
    gz = buffer[12]<<8 | buffer[13];
}
