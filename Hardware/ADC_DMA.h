#ifndef ADC_DMA_H
#define ADC_DMA_H
#include "main.h"
#include "stdio.h"
#include "Config.h"
#include "adc.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "FFT.h"
#define adc_htimx htim8
#define hadcx hadc1
#define ADC_NUMBER 1024 // 定义ADC一次采样的点数
#define ADC_BUFFER_MultiMode_SIZE ADC_NUMBER / 2

void ADC_DMA_Start(uint32_t Fs);

uint32_t *ADC_ReadData_u32(void);
void ADC_MultiModeBufferConvert(uint32_t *adc_value_buffer_u32, uint32_t *adc_value_u32, uint16_t size);

void ADC_Restart(uint32_t Fs);
void ADC_Change_Fs(void);

void Filter_FastMA(uint32_t *in, uint32_t *out);
void Filter_IIR(uint32_t *in, uint32_t *out, float alpha);
void Scale_X_Axis(uint32_t *in, uint32_t *out, float scale_percent);

void ADC_DataHandle(uint32_t *adc_value, uint16_t begin, uint16_t len);
uint8_t Auto_Measure(void);

#endif