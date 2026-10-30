#ifndef ADC_DMA_H
#define ADC_DMA_H
#include "main.h"
#include "adc.h"
#include "tim.h"
#define adc_htimx htim8
#define hadcx hadc1
#define ADC_NUMBER 4096U
#define ADC_SAMPLE_RATE_HZ 2000000UL

void ADC_DMA_Start(uint32_t Fs);
void ADC_Restart(uint32_t Fs);
void ADC_Change_Fs(void);
uint16_t *ADC_ReadData_u16(void);
const uint16_t *ADC_GetBuffer(void);

#endif
