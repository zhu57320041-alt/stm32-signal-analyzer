#ifndef FFT_H
#define FFT_H
#include "arm_math.h"
#include "arm_const_structs.h"
#include "config.h"
#include "ADC_DMA.h"

#define FFT_LENGTH 1024 // FFT点数，必须是2的幂
// #define SAMPLE_FREQ 10000 // 你的采样频率，例如10kHz

extern arm_rfft_fast_instance_f32 fftInstance; // FFT实例
extern float32_t fftinput[FFT_LENGTH];
extern float32_t fftoutput[FFT_LENGTH * 2];
extern float32_t output[FFT_LENGTH / 2 + 1];

void FFT_Init(void);
void FFT_Process(float32_t *fftinput, float32_t *output);

uint32_t FFT_App(float *Freq);
void FFT_ResetSmoothing(void);

// 谐波结构：基波+二次+三次
typedef struct
{
  float freq; // 频率(Hz)
  float amp;  // 幅值(V)
} FFT_Harmonic_t;

/**
 * @brief 检测基波+二次+三次谐波
 * @param fs     采样率(Hz)
 * @param h      [0]=基波, [1]=二次谐波, [2]=三次谐波；未检测到则 freq/amp=0
 * @retval 实际检测到的峰数（至少1，含基波）
 * @note 必须在 FFT_App() 之后调用（使用其填充的全局 output[] 频谱）
 */
uint32_t FFT_GetHarmonics(float fs, FFT_Harmonic_t *h);

#endif // FFT_H