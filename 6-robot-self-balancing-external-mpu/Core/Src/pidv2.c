#include "pidv2.h"
#include <math.h>

/*
 * pidv2.c
 * Alternative PID implementation with configurable derivative filter alpha
 * and conditional integral anti-windup. Functions are named PID2_* so
 * they can coexist with the original PID_* API.
 */

void PID2_Init(PID2_t *pid,
               float Kp, float Ki, float Kd,
               float integral_limit, float output_limit,
               float d_filter_alpha)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->prev_measured  = 0.0f;
    pid->d_filter       = 0.0f;
    pid->d_filter_alpha = d_filter_alpha;
    pid->first_run      = 1;
}

float PID2_Compute(PID2_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    /* Proportional */
    float p_term = pid->Kp * error;

    /* Integral with anti-windup clamp */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_term = pid->Ki * pid->integral;

    /* Derivative with optional low-pass filter on error derivative */
    if (pid->first_run) {
        pid->first_run = 0;
    }
    float d_raw = (error - pid->prev_error) / dt;
    float alpha = pid->d_filter_alpha;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    float d_filtered = alpha * pid->d_filter + (1.0f - alpha) * d_raw;
    float d_term = pid->Kd * d_filtered;
    pid->d_filter = d_filtered;
    pid->prev_error = error;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    if (output > pid->output_limit) output = pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void PID2_Reset(PID2_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filter = 0.0f;
    pid->first_run = 1;
}
