#include "debug.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_huart    = NULL;
static uint32_t            s_last_ms  = 0;

static void uart_send(const char *buf, uint16_t len)
{
    if (!s_huart) return;
    HAL_UART_Transmit(s_huart, (uint8_t *)buf, len, 200);
}

void Debug_Init(UART_HandleTypeDef *huart)
{
    s_huart   = huart;
    s_last_ms = 0;
    const char *banner = "\r\n--- Self-Balance Robot (Cascade PID) ---\r\n";
    uart_send(banner, (uint16_t)strlen(banner));
    const char *hdr = "[ms]\tAngle\tGyroR\tTgtPitch\tStbOut\tEncL\tEncR\tVelOut\tStbKp\tVelKp\tVelKi\r\n";
    uart_send(hdr, (uint16_t)strlen(hdr));
}

void Debug_Update(float angle,
                  float gyro_rate,
                  float target_pitch,
                  float pid_stb_out,
                  int32_t enc_l_delta,
                  int32_t enc_r_delta,
                  float vel_out,
                  float stb_kp,
                  float vel_kp,
                  float vel_ki)
{
    if (!s_huart) return;
    uint32_t now = HAL_GetTick();
    if (now - s_last_ms < DEBUG_INTERVAL_MS) return;
    s_last_ms = now;

    char buf[128];
    int n = snprintf(buf, sizeof(buf),
                     "%lu\t%.2f\t%.2f\t%.2f\t%.1f\t%ld\t%ld\t%.2f\t%.1f\t%.3f\t%.3f\r\n",
                     now, angle, gyro_rate, target_pitch, pid_stb_out,
                     enc_l_delta, enc_r_delta, vel_out,
                     stb_kp, vel_kp, vel_ki);
    uart_send(buf, (uint16_t)n);
}

void Debug_Print(const char *msg)
{
    uart_send(msg, (uint16_t)strlen(msg));
}
