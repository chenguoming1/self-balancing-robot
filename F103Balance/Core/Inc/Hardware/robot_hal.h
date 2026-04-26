#ifndef __ROBOT_HAL_H
#define __ROBOT_HAL_H

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Type aliases for SPL compatibility */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;

/* GPIO bit-banding macros (same as SPL sys.h, work on STM32F1 with HAL too) */
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)        *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr  (GPIOA_BASE+12)
#define GPIOB_ODR_Addr  (GPIOB_BASE+12)
#define GPIOC_ODR_Addr  (GPIOC_BASE+12)
#define GPIOD_ODR_Addr  (GPIOD_BASE+12)

#define GPIOA_IDR_Addr  (GPIOA_BASE+8)
#define GPIOB_IDR_Addr  (GPIOB_BASE+8)
#define GPIOC_IDR_Addr  (GPIOC_BASE+8)
#define GPIOD_IDR_Addr  (GPIOD_BASE+8)

#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n)
#define PAin(n)  BIT_ADDR(GPIOA_IDR_Addr, n)
#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n)  BIT_ADDR(GPIOB_IDR_Addr, n)
#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)
#define PCin(n)  BIT_ADDR(GPIOC_IDR_Addr, n)

/* Motor type selection - defines PID constants */
#define GB37520

#ifdef GB37520
#define BLC_KP   240
#define BLC_KD   0.75f
#define SPD_KP   69
#define SPD_KI   0.345f
#define TURN_KP  -40
#define TURN_KD  -0.6f
#endif

/* Microsecond delay using DWT cycle counter */
static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

static inline void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/* HAL peripheral handles (defined in main.c) */
extern ADC_HandleTypeDef  hadc1;
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* Global robot state variables (defined in main.c) */
extern float  Voltage;
extern float  pitch, roll, yaw;
extern short  aacx, aacy, aacz;
extern short  gyrox, gyroy, gyroz;
extern float  SR04_Distance;
extern int    Encoder_Left, Encoder_Right;
extern int    Moto1, Moto2;
extern u8     CTRL_MODE, Mode_Change;
extern int    Uart_Receive;
extern u8     TkSensor;
extern int    PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY;
extern u8     PS2_Plugin;
extern float  Target_Speed, Turn_Speed;
extern u8     Fore, Back, Left, Right;

void Mode_Init(void);
void Tracking_Init(void);

#endif /* __ROBOT_HAL_H */
