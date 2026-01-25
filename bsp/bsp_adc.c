
#include "adc.h"
#include "bsp_adc.h"
#include "main.h"

#define ADC_CH_CNT 4

static uint16_t adc_value[ADC_CH_CNT];

void BSP_ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, ADC_CH_CNT);
    
}
