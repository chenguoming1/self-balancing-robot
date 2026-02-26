#include "debug.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_huart = NULL;
static uint32_t            s_last_tx_ms = 0;

/* ── Internal transmit helper ────────────────────────────────────────── */
static void uart_send(const char *buf, uint16_t len)
{
    if (s_huart == NULL) return;
    /* Blocking transmit with short timeout.
       At 9600 baud a 128-char packet takes ~133 ms max —
       keep your debug strings short or increase timeout.       */
    HAL_UART_Transmit(s_huart, (uint8_t *)buf, len, 200);
}

/* ── Public API ──────────────────────────────────────────────────────── */

void Debug_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_last_tx_ms = 0;

    /* Send a header line so your serial monitor columns are labelled */
    const char *header = "\r\n--- Self-Balance Robot Debug ---\r\n";
    uart_send(header, (uint16_t)strlen(header));

    /* Column headers matching the data fields enabled above */
    char hdr[192];
    int n = 0;
    n += snprintf(hdr + n, sizeof(hdr) - n, "[ms]");
#if DEBUG_ANGLE
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tAngle");
#endif
#if DEBUG_GYRO_RATE
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tGyroRate");
#endif
#if DEBUG_ACCEL_ANGLE
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tAccelAng");
#endif
#if DEBUG_PID_OUTPUT
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tPID_Out");
#endif
#if DEBUG_ENCODER_LEFT
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tEncL");
#endif
#if DEBUG_ENCODER_RIGHT
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tEncR");
#endif
#if DEBUG_SETPOINT
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tSetpt");
#endif
#if DEBUG_KP
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tKp");
#endif
#if DEBUG_KI
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tKi");
#endif
#if DEBUG_KD
    n += snprintf(hdr + n, sizeof(hdr) - n, "\tKd");
#endif
    n += snprintf(hdr + n, sizeof(hdr) - n, "\r\n");
    uart_send(hdr, (uint16_t)n);
}

void Debug_Update(float angle,
                  float gyro_rate,
                  float accel_angle,
                  float pid_output,
                  int32_t enc_left_delta,
                  int32_t enc_right_delta,
                  float setpoint,
                  float kp, float ki, float kd)
{
    if (s_huart == NULL) return;

    uint32_t now = HAL_GetTick();
    if (now - s_last_tx_ms < DEBUG_INTERVAL_MS) return;
    s_last_tx_ms = now;

    char buf[192];
    int n = 0;

    /* Timestamp in ms */
    n += snprintf(buf + n, sizeof(buf) - n, "%lu", now);

#if DEBUG_ANGLE
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.2f", angle);
#endif
#if DEBUG_GYRO_RATE
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.2f", gyro_rate);
#endif
#if DEBUG_ACCEL_ANGLE
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.2f", accel_angle);
#endif
#if DEBUG_PID_OUTPUT
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.1f", pid_output);
#endif
#if DEBUG_ENCODER_LEFT
    n += snprintf(buf + n, sizeof(buf) - n, "\t%ld", enc_left_delta);
#endif
#if DEBUG_ENCODER_RIGHT
    n += snprintf(buf + n, sizeof(buf) - n, "\t%ld", enc_right_delta);
#endif
#if DEBUG_SETPOINT
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.2f", setpoint);
#endif
#if DEBUG_KP
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.3f", kp);
#endif
#if DEBUG_KI
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.3f", ki);
#endif
#if DEBUG_KD
    n += snprintf(buf + n, sizeof(buf) - n, "\t%.3f", kd);
#endif

    n += snprintf(buf + n, sizeof(buf) - n, "\r\n");
    uart_send(buf, (uint16_t)n);
}

void Debug_Print(const char *msg)
{
    if (s_huart == NULL) return;
    uart_send(msg, (uint16_t)strlen(msg));
}
