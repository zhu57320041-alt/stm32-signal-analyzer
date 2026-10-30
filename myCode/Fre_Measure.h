#ifndef Fre_Measure_h
#define Fre_Measure_h
#include "main.h"
#define Fre_htimx htim5
#define TIM_FREQ 2000000 // 定时器频率2MHz
void Fre_Measure_Init(void);
uint8_t Fre_Measure_Get_Fre(float *frequency, float *duty);


#endif