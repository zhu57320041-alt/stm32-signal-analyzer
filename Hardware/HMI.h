#ifndef HMI_H
#define HMI_H
#include "main.h"
#include "stdio.h"
#include "Config.h"
#include "ADC_DMA.h"
#include "arm_math.h"
void HMI_SeedFrame_Wave(uint32_t *data, uint16_t len, uint16_t totallen, int16_t offset);
uint8_t HMI_ReceiveHandle(uint8_t *RxData, uint8_t Size);
void HMI_SeedFrame_MeasureData(void);

void HMI_SeedFrame_FFT(float *data, uint16_t len);

#endif // HMI_H