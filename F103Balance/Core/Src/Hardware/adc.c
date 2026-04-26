#include "adc.h"

/* ADC1 channel 8 already configured by MX_ADC1_Init in main.c.
 * Call HAL_ADCEx_Calibration_Start after init for accuracy.
 */
void Adc_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start(&hadc1);
}

/* Read ADC value for the specified channel */
u16 Get_Adc(u32 channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    return (u16)HAL_ADC_GetValue(&hadc1);
}

/* Battery voltage: divider ratio 11:1, 3.3 V reference, 12-bit ADC */
float Get_battery_volt(void)
{
    float volt = Get_Adc(Battery_Ch) * 3.3f * 11.0f / 4096.0f;
    return volt;
}
