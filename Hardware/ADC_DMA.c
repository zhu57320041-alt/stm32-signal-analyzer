#include "ADC_DMA.h"

/* DMA must use normal SRAM; the F407 CCM RAM is not reachable by DMA2. */
static uint16_t adc_sample_buffer[ADC_NUMBER];
static volatile uint8_t adc_frame_ready;

static void ADC_ConfigureTimer(uint32_t sample_rate_hz)
{
  uint32_t timer_ticks;

  /* TIM8 is clocked at 144 MHz with the current clock tree. */
  if ((sample_rate_hz != ADC_SAMPLE_RATE_HZ) ||
      ((144000000UL % sample_rate_hz) != 0U))
  {
    sample_rate_hz = ADC_SAMPLE_RATE_HZ;
  }

  timer_ticks = 144000000UL / sample_rate_hz;
  HAL_TIM_Base_Stop(&adc_htimx);
  __HAL_TIM_SET_PRESCALER(&adc_htimx, 0U);
  __HAL_TIM_SET_AUTORELOAD(&adc_htimx, timer_ticks - 1U);
  __HAL_TIM_SET_COUNTER(&adc_htimx, 0U);
  adc_htimx.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&adc_htimx, TIM_FLAG_UPDATE);
}

void ADC_DMA_Start(uint32_t Fs)
{
  ADC_ConfigureTimer(Fs);
  adc_frame_ready = 0U;

  (void)HAL_ADC_Stop_DMA(&hadcx);
  if (HAL_ADC_Start_DMA(&hadcx, (uint32_t *)adc_sample_buffer,
                        ADC_NUMBER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&adc_htimx) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Restart(uint32_t Fs)
{
  ADC_DMA_Start(Fs);
}

void ADC_Change_Fs(void)
{
  /* The analysis path has one fixed, calibrated 2 MSa/s acquisition mode. */
  ADC_ConfigureTimer(ADC_SAMPLE_RATE_HZ);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_TIM_Base_Stop(&adc_htimx);
    adc_frame_ready = 1U;
  }
}

uint16_t *ADC_ReadData_u16(void)
{
  if (adc_frame_ready != 0U)
  {
    adc_frame_ready = 0U;
    return adc_sample_buffer;
  }
  return NULL;
}

const uint16_t *ADC_GetBuffer(void)
{
  return adc_sample_buffer;
}
