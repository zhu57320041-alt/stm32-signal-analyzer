#include "FFT.h"
#include "Config.h"
#include <math.h>

void Wave_Interpolation(
    float *input,
    float *output
);
	
arm_rfft_fast_instance_f32 fftInstance; // FFT实例
float32_t fftinput[FFT_LENGTH];         // FFT输入数组
float32_t fftoutput[FFT_LENGTH * 2];    // FFT输出数组
float32_t output[FFT_LENGTH / 2 + 1];   // 输出数组

#define DISPLAY_LENGTH 2048
float32_t display_wave[DISPLAY_LENGTH];


extern uint32_t *adc_value_buffer_u32;
extern uint32_t adc_value_u32[];
extern OSC_MeasureData_t OSC_MeasureData;

#define FREQ_ALPHA 0.2f                 // 频率平滑系数（与MATLAB一致）
static float smoothed_freq = 0.0f;      // 平滑后的频率

void FFT_Init(void)
{
  // 初始化实数FFT
  arm_rfft_fast_init_f32(&fftInstance, FFT_LENGTH);
  smoothed_freq = 0.0f;
}

void FFT_Process(float32_t *fftinput, float32_t *output)
{
  // 执行实数FFT
  arm_rfft_fast_f32(&fftInstance, fftinput, fftoutput, 0);
  arm_cmplx_mag_f32(fftoutput, output, FFT_LENGTH / 2 + 1);
}

/**
 * @brief  FFT 处理（含抛物线插值与频率平滑）
 * @param  Freq 输出频率指针
 * @retval 0 成功, 1 失败
 */
uint32_t FFT_App(float *Freq)
{
  float max_val = 0;
  uint32_t max_index = 0;
  float fs = (float)OSC_Config.OSC_Fs;

  const float adc_to_volt = 3.3f / 4095.0f; // Vref / ADC分辨率，同MATLAB

  // --- 准备输入数据：ADC原始值 → 电压，与MATLAB voltage = raw/adc_resolution*vref 一致 ---
  if (OSC_Config.OSC_Fs == 7200000)
  {
    for (uint16_t i = 0; i < FFT_LENGTH; i++)
    {
      fftinput[i] = (float)adc_value_u32[i] * adc_to_volt;
    }
  }
  else
  {
    for (uint16_t i = 0; i < FFT_LENGTH; i++)
    {
      fftinput[i] = (float)adc_value_buffer_u32[i] * adc_to_volt;
    }
  }

  // --- FFT 与幅值计算 ---
  FFT_Process(fftinput, output);   //不要乱改

  Wave_Interpolation(
    fftinput,
    display_wave
	);
  // --- 归一化：arm_cmplx_mag_f32 输出 sqrt(real^2+imag^2)，需除以 L 同MATLAB abs(Y/L) ---
  uint16_t half_len = FFT_LENGTH / 2 + 1;
  for (uint16_t i = 0; i < half_len; i++)
  {
    output[i] /= (float)FFT_LENGTH;
  }
  output[0] = 0;  // 去直流分量（同MATLAB P1(1)=0）

  // --- 单边谱修正：同MATLAB P1(2:end-1) = 2 * P1(2:end-1) ---
  for (uint16_t i = 1; i < half_len - 1; i++)
  {
    output[i] *= 2.0f;
  }

  // --- 找主峰（跳过直流和低频噪声，从bin 5开始，同MATLAB P1(5:end)） ---
  uint32_t search_start = 5;
  uint32_t search_len = half_len - search_start;
  arm_max_f32(&output[search_start], search_len, &max_val, &max_index);
  max_index += search_start;

  // --- 抛物线插值（三点拟合，同MATLAB FFT1.m 第125~136行） ---
  float peak_bin = (float)max_index;
  if (max_index > 1 && max_index < FFT_LENGTH / 2)
  {
    float a = output[max_index - 1];
    float b = output[max_index];
    float c = output[max_index + 1];
    float denom = a - 2.0f * b + c;
    if (fabsf(denom) > 1e-12f)
    {
      float delta = (a - c) / (2.0f * denom);
      peak_bin = (float)max_index + delta;
    }
  }

  float peak_freq = peak_bin * (fs / (float)FFT_LENGTH);

  // --- 一阶低通平滑频率（同MATLAB 第141~145行） ---
  if (smoothed_freq == 0.0f)
  {
    smoothed_freq = peak_freq;
  }
  else
  {
    smoothed_freq = FREQ_ALPHA * peak_freq + (1.0f - FREQ_ALPHA) * smoothed_freq;
  }

  *Freq = smoothed_freq;

  // --- 输出峰值幅值 ---
  OSC_MeasureData.Freq_FFT_Amp = max_val;

  return 0;
}

/**
 * @brief  重置频率平滑状态（切换采样率时调用）
 */
void FFT_ResetSmoothing(void)
{
  smoothed_freq = 0.0f;
}

/* 抛物线插值（三点拟合）：返回修正后的峰 bin 位置 */
static float PeakInterp(float *data, uint32_t idx, uint32_t n)
{
  if (idx < 1 || idx > n - 2)
    return (float)idx;

  float a = data[idx - 1];
  float b = data[idx];
  float c = data[idx + 1];
  float denom = a - 2.0f * b + c;

  if (fabsf(denom) < 1e-12f)
    return (float)idx;

  float delta = (a - c) / (2.0f * denom);
  if (delta > 1.0f)  delta = 1.0f;
  if (delta < -1.0f) delta = -1.0f;
  return (float)idx + delta;
}

/* 在 [lo, hi] bin 区间内找最大峰，返回插值 bin 位置；*amp_out 输出该峰幅值 */
static float FindPeakInRange(uint32_t lo, uint32_t hi, float *amp_out, uint32_t n)
{
  if (hi < lo)
  {
    *amp_out = 0.0f;
    return -1.0f;
  }

  uint32_t len = hi - lo + 1;
  float max_val;
  uint32_t max_idx;
  arm_max_f32(&output[lo], len, &max_val, &max_idx);
  max_idx += lo;

  *amp_out = max_val;
  return PeakInterp(output, max_idx, n);
}

/**
 * @brief 检测基波+二次+三次谐波
 * @param fs     采样率(Hz)
 * @param h      [0]=基波, [1]=二次谐波, [2]=三次谐波；未检测到则 freq/amp=0
 * @retval 实际检测到的峰数（至少1，含基波）
 * @note 必须在 FFT_App() 之后调用（使用其填充的全局 output[] 频谱）
 */
uint32_t FFT_GetHarmonics(float fs, FFT_Harmonic_t *h)
{
  uint16_t half_len = FFT_LENGTH / 2 + 1;

  if (h == NULL)
    return 0;

  // 清空输出，保证"未检测到"的槽位为 0
  for (int i = 0; i < 3; i++)
  {
    h[i].freq = 0.0f;
    h[i].amp  = 0.0f;
  }

  /* 1) 基波：整个有效谱区间的最大峰（跳过直流与低频噪声） */
  float f0_amp;
  float f0_bin = FindPeakInRange(5, half_len - 1, &f0_amp, half_len);
  if (f0_bin < 1.0f)
    return 0;

  h[0].freq = f0_bin * (fs / (float)FFT_LENGTH);
  h[0].amp  = f0_amp;
  uint32_t count = 1;

  /* 2) 二次、三次谐波：在 k*基波bin 附近窗口内找局部最大峰 */
  for (uint32_t k = 2; k <= 3; k++)
  {
    float target = f0_bin * (float)k;
    if (target >= (float)(half_len - 1))
      break;   // 谐波已超出频谱范围

    // 搜索窗口：基波bin的±10%，至少±3个bin，最多±20个bin
    uint32_t win = (uint32_t)(f0_bin * 0.10f + 0.5f);
    if (win < 3)  win = 3;
    if (win > 20) win = 20;

    uint32_t lo = ((uint32_t)target > win) ? ((uint32_t)target - win) : 1;
    uint32_t hi = (uint32_t)target + win;
    if (hi > half_len - 1)
      hi = half_len - 1;

    float amp;
    float bin = FindPeakInRange(lo, hi, &amp, half_len);
    if (bin < 1.0f)
      continue;

    // 幅度相对基波过低（<2%）→ 视为无此谐波（噪声/旁瓣）
    if (amp < f0_amp * 0.02f)
      continue;

    h[k - 1].freq = bin * (fs / (float)FFT_LENGTH);
    h[k - 1].amp  = amp;
    count++;
  }

  return count;
}
//--插值函数--
static float Cubic(
    float y0,
    float y1,
    float y2,
    float y3,
    float t
)
{

    float a0 =
    -0.5f*y0
    +1.5f*y1
    -1.5f*y2
    +0.5f*y3;


    float a1 =
    y0
    -2.5f*y1
    +2.0f*y2
    -0.5f*y3;


    float a2 =
    -0.5f*y0
    +0.5f*y2;


    float a3=y1;


    return ((a0*t+a1)*t+a2)*t+a3;
}



void Wave_Interpolation(
    float *input,
    float *output
)
{

    float scale =
    (float)(FFT_LENGTH-1)
    /(float)(DISPLAY_LENGTH-1);



    for(int i=0;i<DISPLAY_LENGTH;i++)
    {

        float pos=i*scale;

        int index=(int)pos;

        float t=pos-index;


        int i0=index-1;
        int i1=index;
        int i2=index+1;
        int i3=index+2;


        if(i0<0)
            i0=0;

        if(i2>=FFT_LENGTH)
            i2=FFT_LENGTH-1;

        if(i3>=FFT_LENGTH)
            i3=FFT_LENGTH-1;



        output[i]=Cubic(
            input[i0],
            input[i1],
            input[i2],
            input[i3],
            t
        );

    }
}

