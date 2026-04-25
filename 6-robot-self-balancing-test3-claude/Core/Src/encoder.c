#include "encoder.h"
#include "main.h"

Encoder_t enc_left  = {0};
Encoder_t enc_right = {0};

void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    enc_left.prev_cnt  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc_right.prev_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void Encoder_Update(void)
{
    int16_t cur_l      = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc_left.delta     = (int32_t)(cur_l - enc_left.prev_cnt);
    enc_left.count    += enc_left.delta;
    enc_left.prev_cnt  = cur_l;

    int16_t cur_r      = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    enc_right.delta    = (int32_t)(cur_r - enc_right.prev_cnt);
    enc_right.count   += enc_right.delta;
    enc_right.prev_cnt = cur_r;
}
