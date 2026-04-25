#include "pid.h"
#include <math.h>  // fabsf

void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float integral_limit, float output_limit,
              float output_ramp)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->output_ramp    = output_ramp;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->prev_output    = 0.0f;
    pid->initialized    = 0u;
}

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    /* Proportional */
    float p_term = pid->Kp * error;

    /* Integral with anti-windup clamp */
    pid->integral += error * dt;
    if      (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_term = pid->Ki * pid->integral;

    /* Derivative (on measurement to avoid derivative kick on setpoint change) */
    float d_term = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    if      (output >  pid->output_limit) output =  pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    /* Slew limit the output to reduce backlash chatter and step shocks. */
    if (!pid->initialized) {
        pid->prev_output = output;
        pid->initialized = 1u;
    } else if ((pid->output_ramp > 0.0f) && (dt > 0.0f)) {
        float max_step = pid->output_ramp * dt;
        float delta = output - pid->prev_output;
        if (delta > max_step) {
            output = pid->prev_output + max_step;
        } else if (delta < -max_step) {
            output = pid->prev_output - max_step;
        }
    }

    pid->prev_output = output;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->initialized = 0u;
}
