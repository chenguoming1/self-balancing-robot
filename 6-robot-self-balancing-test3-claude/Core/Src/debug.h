#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define DEBUG_INTERVAL_MS  100u

void Debug_Init(UART_HandleTypeDef *huart);

/* Fields streamed every DEBUG_INTERVAL_MS:
   angle        – fused pitch (degrees)
   gyro_rate    – raw gyro Y (°/s)
   target_pitch – velocity PID output / inner setpoint (degrees)
   pid_stb_out  – stabilisation PWM output
   enc_l_delta  – left encoder delta (counts/loop)
   enc_r_delta  – right encoder delta (counts/loop)
   vel_out      – velocity PID output before LPF (degrees)
   stb_kp       – current pid_stb Kp
   vel_kp       – current pid_vel Kp
   vel_ki       – current pid_vel Ki                             */
void Debug_Update(float angle,
                  float gyro_rate,
                  float target_pitch,
                  float pid_stb_out,
                  int32_t enc_l_delta,
                  int32_t enc_r_delta,
                  float vel_out,
                  float stb_kp,
                  float vel_kp,
                  float vel_ki);

void Debug_Print(const char *msg);

#endif /* DEBUG_H */
