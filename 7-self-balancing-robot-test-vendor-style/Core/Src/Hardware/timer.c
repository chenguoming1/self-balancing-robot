#include "timer.h"
#include "SR04.h"

TIM_HandleTypeDef htim4;

/* Timer 4 used as free-running counter for SR04 ultrasonic distance measurement.
 * arr: auto-reload value, psc: prescaler
 * For 1 us resolution at 72 MHz: psc = 7199, arr = 5000 (500 ms max range)
 */
void Timer4_Init(u16 arr, u16 psc)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = psc;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period             = arr;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);
    /* Timer is started/stopped by SR04 driver on demand */
}
