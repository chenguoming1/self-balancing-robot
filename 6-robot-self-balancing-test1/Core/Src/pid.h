#ifndef PID_H
#define PID_H

typedef struct {
    /* Tuning */
    float Kp;
    float Ki;
    float Kd;

    /* State */
    float integral;
    float prev_error;

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
              float integral_limit, float output_limit);

/**
 * Compute PID output for one step.
 * dt  – time since last call in seconds (must be constant for best results)
 */
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);

/** Reset integrator and derivative state (e.g. after robot is picked up) */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
