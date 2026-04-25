#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"

typedef struct {
    int32_t  count;
    int32_t  delta;
    int16_t  prev_cnt;
} Encoder_t;

extern Encoder_t enc_left;   // TIM2  (PA0=CH1, PA1=CH2)
extern Encoder_t enc_right;  // TIM3  (PA6=CH1, PA7=CH2)

void Encoder_Start(void);
void Encoder_Update(void);

#endif /* ENCODER_H */
