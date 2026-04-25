#include "encoder.h"
#include "main.h"

Encoder_t enc_left  = {0};   // TIM2  PA0=CH1  PA1=CH2
Encoder_t enc_right = {0};   // TIM3  PA6=CH1  PA7=CH2

void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // Left
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Right

    enc_left.prev_cnt  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc_right.prev_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

/* ── Called from TIM4 ISR (200 Hz control loop) ──────────────────────── */
void Encoder_Update(void)
{
    /* Left – TIM2 */
    int16_t cur_l      = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc_left.delta     = (int32_t)(cur_l - enc_left.prev_cnt);
    enc_left.count    += enc_left.delta;
    enc_left.prev_cnt  = cur_l;

    /* Right – TIM3 */
    int16_t cur_r      = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    enc_right.delta    = (int32_t)(cur_r - enc_right.prev_cnt);
    enc_right.count   += enc_right.delta;
    enc_right.prev_cnt = cur_r;
}
