#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"

typedef struct {
    int32_t  count;        // Absolute pulse count (signed, accumulates)
    int32_t  delta;        // Change since last read (read by control loop)
    int16_t  prev_cnt;     // Last raw TIM counter value (for overflow handling)
} Encoder_t;

extern Encoder_t enc_left;   // TIM2  (PA0=CH1, PA1=CH2)
extern Encoder_t enc_right;  // TIM3  (PA6=CH1, PA7=CH2)

/**
 * Start both encoder timers.
 * Call once in main() after MX_TIMx_Init().
 */
void Encoder_Start(void);

/**
 * Update delta for both encoders.
 * Call from the FIXED-RATE control loop ISR (TIM3) BEFORE reading delta.
 * Handles 16-bit counter overflow correctly.
 */
void Encoder_Update(void);

#endif /* ENCODER_H */
