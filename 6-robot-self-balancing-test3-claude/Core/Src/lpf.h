#ifndef LPF_H
#define LPF_H

/* First-order IIR low-pass filter.
   Matches the LowPassFilter used in Arduino-FOC-balancer.
   Transfer function: y[n] = y[n-1] + (x[n] - y[n-1]) * dt / (Tf + dt)
   Tf – time constant in seconds (larger = more smoothing / more lag).  */
typedef struct {
    float Tf;   // time constant (s)
    float y;    // current output
} LPF_t;

/* Update filter with new input, returns filtered output. */
float LPF_Update(LPF_t *lpf, float input, float dt);

/* Reset output to zero (call after fall detection or disable). */
void  LPF_Reset(LPF_t *lpf);

#endif /* LPF_H */
