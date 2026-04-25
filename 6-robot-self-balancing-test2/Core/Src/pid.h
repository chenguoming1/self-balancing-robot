#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    /* Tuning */
    float Kp;
    float Ki;
    float Kd;
    float output_ramp;

    /* State */
    float integral;
    float prev_error;
    float prev_output;
    uint8_t initialized;

    /* Anti-windup clamp on integral term */
    float integral_limit;

    /* Output clamp */
    float output_limit;
} PID_t;

/**
 * Initialize PID with gains and limits.
 * Call once before the control loop starts.
 */
void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float integral_limit, float output_limit,
              float output_ramp);

/**
 * Compute PID output for one step.
 * dt  – time since last call in seconds (must be constant for best results)
 */
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);

/** Reset integrator and derivative state (e.g. after robot is picked up) */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
