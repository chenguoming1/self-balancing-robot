#ifndef PIDV2_H
#define PIDV2_H

#include <stdint.h>

typedef struct {
    /* Tuning */
    float Kp;
    float Ki;
    float Kd;

    /* State */
    float integral;
    float prev_error;

    /* Derivative (measurement) state */
    float prev_measured;
    float d_filter;        /* filtered derivative value */
    float d_filter_alpha;  /* 0..1 filter alpha, smaller = smoother */

    uint8_t first_run;     /* set on init, cleared on first compute */

    /* Anti-windup clamp on integral term */
    float integral_limit;

    /* Output clamp */
    float output_limit;
} PID2_t;

void PID2_Init(PID2_t *pid,
               float Kp, float Ki, float Kd,
               float integral_limit, float output_limit,
               float d_filter_alpha);

float PID2_Compute(PID2_t *pid, float setpoint, float measured, float dt);

void PID2_Reset(PID2_t *pid);

#endif /* PIDV2_H */
