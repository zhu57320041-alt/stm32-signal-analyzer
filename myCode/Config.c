#include "Config.h"
OSC_Config_t OSC_Config;
OSC_MeasureData_t OSC_MeasureData;
DAC_Param_t DAC_Param;
uint32_t OSC_Fs_Arr[] = {100, 1000, 10000, 100000, 1000000, 7200000};
uint32_t OSC_Fs_Arr_config[][2] = {{144, 10000}, {144, 1000}, {144, 100}, {144, 10}, {72, 2}, {30, 2}};
/**
 * @brief 配置和数据初始化
 * @param None
 * @retval None
 */
void ConfigAndData_Init(void)
{
  OSC_Config.OSC_Enable = 1;
  OSC_Config.OSC_DAC_Enable = 1;
  OSC_Config.OSC_FFT_Enable = 0;
  OSC_Config.OSC_Auto_Enable = 0;
  OSC_Config.OSC_Auto_Success = 1;
  OSC_Config.OSC_Print_Enable = 1; // 上电默认开启电脑串口打印（波形+FFT帧）
  OSC_Config.OSC_Fs_Index = 5; // 默认 7.2MHz 三重交替采样，三次谐波范围 0~3.6MHz
  OSC_Config.OSC_Fs = OSC_Fs_Arr[OSC_Config.OSC_Fs_Index];
  OSC_Config.Scale_X_Axis = 100;
  OSC_MeasureData.VPP = 0.0;
  OSC_MeasureData.VMax = 0.0;
  OSC_MeasureData.VMin = 0.0;
  OSC_MeasureData.Freq_TIM = 0;
  OSC_MeasureData.Freq_FFT = 0;
  OSC_MeasureData.Freq_FFT_Amp = 0;
  OSC_MeasureData.Freq_FFT_H2 = 0;
  OSC_MeasureData.Freq_FFT_H2_Amp = 0;
  OSC_MeasureData.Freq_FFT_H3 = 0;
  OSC_MeasureData.Freq_FFT_H3_Amp = 0;
  OSC_MeasureData.Duty = 0;

  DAC_Param.HaveChange_Flag = 1;
  DAC_Param.type = 0;
  DAC_Param.Freq = 1000;
  DAC_Param.VPP = 2.0f;
  DAC_Param.offset = 0.5f;
  DAC_Param.Duty = 0.5f;
}
/**
 * @brief 配置频率更新
 */
void Config_Update(void)
{
  OSC_Config.OSC_Fs = OSC_Fs_Arr[OSC_Config.OSC_Fs_Index];
}