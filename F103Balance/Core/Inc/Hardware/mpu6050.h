#ifndef __MPU6050_H
#define __MPU6050_H
#include "robot_hal.h"

/* MPU6050 register addresses */
#define MPU_SELF_TESTX_REG      0x0D
#define MPU_SELF_TESTY_REG      0x0E
#define MPU_SELF_TESTZ_REG      0x0F
#define MPU_SELF_TESTA_REG      0x10
#define MPU_SAMPLE_RATE_REG     0x19
#define MPU_CFG_REG             0x1A
#define MPU_GYRO_CFG_REG        0x1B
#define MPU_ACCEL_CFG_REG       0x1C
#define MPU_MOTION_DET_REG      0x1F
#define MPU_FIFO_EN_REG         0x23
#define MPU_I2CMST_CTRL_REG     0x24
#define MPU_INTBP_CFG_REG       0x37
#define MPU_INT_EN_REG          0x38
#define MPU_INT_STA_REG         0x3A
#define MPU_ACCEL_XOUTH_REG     0x3B
#define MPU_GYRO_XOUTH_REG      0x43
#define MPU_TEMP_OUTH_REG       0x41
#define MPU_USER_CTRL_REG       0x6A
#define MPU_PWR_MGMT1_REG       0x6B
#define MPU_PWR_MGMT2_REG       0x6C
#define MPU_FIFO_CNTH_REG       0x72
#define MPU_FIFO_RW_REG         0x74
#define MPU_DEVICE_ID_REG       0x75

/* MPU6050 I2C address (AD0 = GND) */
#define MPU_ADDR                0x68

u8    MPU_Init(void);
u8    MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf);
u8    MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf);
u8    MPU_Write_Byte(u8 reg, u8 data);
u8    MPU_Read_Byte(u8 reg);

u8    MPU_Set_Gyro_Fsr(u8 fsr);
u8    MPU_Set_Accel_Fsr(u8 fsr);
u8    MPU_Set_LPF(u16 lpf);
u8    MPU_Set_Rate(u16 rate);

short MPU_Get_Temperature(void);
u8    MPU_Get_Gyroscope(short *gx, short *gy, short *gz);
u8    MPU_Get_Accelerometer(short *ax, short *ay, short *az);

/* DMP functions (implemented in inv_mpu_dmp_motion_driver.c) */
u8    mpu_dmp_init(void);
u8    mpu_dmp_get_data(float *pitch, float *roll, float *yaw);

#endif
