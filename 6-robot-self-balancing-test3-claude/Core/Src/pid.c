#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float integral_limit, float output_limit)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
}

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    float p_term = pid->Kp * error;

    pid->integral += error * dt;
    if      (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_term = pid->Ki * pid->integral;

    /* Derivative on error to detect angle change direction */
    float d_term = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = p_term + i_term + d_term;
    if      (output >  pid->output_limit) output =  pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
