#include "encoder.h"

/* TIM2 and TIM3 encoder mode already configured by MX_TIM2_Init / MX_TIM3_Init.
 * Call Encoder_Start() after HAL init to begin counting.
 */
void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

/* Read and reset encoder counter for the given timer (2 or 3) */
int Read_Encoder(u8 TIMX)
{
    int val;
    switch (TIMX) {
        case 2:
            val = (short)TIM2->CNT;
            TIM2->CNT = 0;
            break;
        case 3:
            val = (short)TIM3->CNT;
            TIM3->CNT = 0;
            break;
        default:
            val = 0;
    }
    return val;
}
