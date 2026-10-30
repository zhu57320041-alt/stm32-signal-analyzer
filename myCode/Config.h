#ifndef CONFIG_H
#define CONFIG_H
#include "main.h"
#include "arm_math.h"
#define FS_NUM 6 // 采样频率选择数量
typedef struct
{
  uint8_t OSC_Enable;       // 是否进行采样
  uint8_t OSC_FFT_Enable;   // 是否进行FFT
  uint8_t OSC_DAC_Enable;   // 是否进行DAC
  uint8_t OSC_Auto_Enable;  // 是否自动
  uint8_t OSC_Auto_Success; // 自动是否成功
  uint32_t OSC_Fs;          // 采样频率
  uint8_t OSC_Fs_Index;     // 采样频率索引;
  uint16_t Scale_X_Axis;    // 横轴缩放比例
} OSC_Config_t;

typedef struct
{
  uint8_t HaveChange_Flag; // 是否有变化
  uint8_t type;            // 波形类型
  float VPP;               // 峰峰值
  uint32_t Freq;           // 频率
  float Duty;              // 占空比
  float offset;            // 偏移量
} DAC_Param_t;

typedef struct
{
  float32_t VPP;  // 峰峰值
  float32_t VMax; // 最大值
  float32_t VMin; // 最小值
  float Freq_TIM; // 频率
  float Freq_FFT; // 频率
  float Duty;     // 占空比
} OSC_MeasureData_t;

extern OSC_Config_t OSC_Config;
extern OSC_MeasureData_t OSC_MeasureData;
extern DAC_Param_t DAC_Param;
extern uint32_t OSC_Fs_Arr[]; // 采样频率数组
extern uint32_t OSC_Fs_Arr_config[][2];
void ConfigAndData_Init(void);
void Config_Update(void);

#endif // CONFIG_H
