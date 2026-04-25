#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
} PID_t;

void  PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
               float integral_limit, float output_limit);
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);
void  PID_Reset(PID_t *pid);

#endif /* PID_H */
