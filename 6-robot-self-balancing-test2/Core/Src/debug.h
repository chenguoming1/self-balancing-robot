#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Debug interval ──────────────────────────────────────────────────── */
#define DEBUG_INTERVAL_MS   100u    // send debug packet every 100 ms
                                    // change to 50 for faster updates
                                    // change to 500 for slower scrolling

/* ── Select what to stream ───────────────────────────────────────────── *
 * Set to 1 to enable, 0 to disable each field.                          *
 * Disable fields you don't need to keep the output readable.            *
 * ─────────────────────────────────────────────────────────────────────── */
#define DEBUG_ANGLE          1   // fused pitch angle (degrees)
#define DEBUG_GYRO_RATE      1   // raw gyro Y rate (°/s)
#define DEBUG_ACCEL_ANGLE    1   // accelerometer-only angle (degrees)
#define DEBUG_PID_OUTPUT     1   // PID output sent to motors
#define DEBUG_ENCODER_LEFT   1   // left  encoder delta per loop tick
#define DEBUG_ENCODER_RIGHT  1   // right encoder delta per loop tick
#define DEBUG_SETPOINT       1   // current balance angle setpoint
#define DEBUG_KP             0   // current Kp (disable to reduce clutter)
#define DEBUG_KI             0   // current Ki
#define DEBUG_KD             0   // current Kd

/* ── API ─────────────────────────────────────────────────────────────── */

/**
 * Call once in main() after UART is initialised.
 */
void Debug_Init(UART_HandleTypeDef *huart);

/**
 * Call from the MAIN LOOP (not the ISR).
 * Checks if DEBUG_INTERVAL_MS has elapsed and sends one packet.
 * Pass all live values — unused fields are compiled out by the #defines above.
 */
void Debug_Update(float angle,
                  float gyro_rate,
                  float accel_angle,
                  float pid_output,
                  int32_t enc_left_delta,
                  int32_t enc_right_delta,
                  float setpoint,
                  float kp, float ki, float kd);

/**
 * Send a plain string message over Bluetooth immediately.
 * Useful for one-shot events: Debug_Print("MPU6050 OK\r\n");
 */
void Debug_Print(const char *msg);

#endif /* DEBUG_H */
