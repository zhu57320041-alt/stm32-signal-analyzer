#include "ADC_DMA.h"
static uint32_t adc_value_buffer_u32[ADC_NUMBER]; // 定义ADC值数组
float32_t adc_V[ADC_NUMBER];                      // 定义ADC转换为电压值的数组
uint8_t ADCFinish_Flag = 0;                       // 定义ADC完成标志
extern TIM_HandleTypeDef adc_htimx;               // 定义定时器句柄
extern ADC_HandleTypeDef hadcx;                   // 定义ADC句柄
extern ADC_HandleTypeDef hadc2;                   // 定义ADC2句柄
extern ADC_HandleTypeDef hadc3;                   // 定义ADC3句柄
// void ADC_DataHandle(void);      // 声明数据处理函数

/**
 * @brief ADC初始化函数
 * @retval None
 */

void ADC_DMA_Start(uint32_t Fs)
{
  if (Fs == 7200000)
  {
    // 三重交替采样：先使能并启动从 ADC(2/3)，再启动主 ADC1 的多模式 DMA
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start(&hadc2);
    HAL_TIM_Base_Start(&adc_htimx);
    HAL_ADCEx_MultiModeStart_DMA(&hadcx, (uint32_t *)adc_value_buffer_u32, ADC_BUFFER_MultiMode_SIZE);
    ADC->CCR |= ADC_CCR_DDS;
  }
  else
  {
    HAL_TIM_Base_Start(&adc_htimx);
    HAL_ADC_Start_DMA(&hadcx, (uint32_t *)adc_value_buffer_u32, ADC_NUMBER);
  }
}


/**
 * @brief ADC完成回调函数
 * @param hadc ADC句柄
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    ADCFinish_Flag = 1;
  }
}

/**
 * @brief ADC数据读取函数
 * @retval ADC值数组指针
 */
uint32_t *ADC_ReadData_u32(void)
{
  if (ADCFinish_Flag)
  {
    ADCFinish_Flag = 0; // 清标志
    return adc_value_buffer_u32;
  }
  return NULL;
}

/**
 * @brief ADC重启函数
 * @param Fs 采样频率
 * @retval None
 */
void ADC_Restart(uint32_t Fs)
{
  static uint8_t Flag_7200000 = 0; // 定义7.2M标志
  if (Fs == 7200000)
  {
    Flag_7200000 = 1;
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_Stop(&hadc2);
    HAL_ADC_Stop(&hadc3);
    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start(&hadc2);
    // HAL_TIM_Base_Start(&htim2);
    HAL_ADCEx_MultiModeStart_DMA(&hadcx, (uint32_t *)adc_value_buffer_u32, ADC_BUFFER_MultiMode_SIZE);
  }
  else
  {
    // if (Flag_7200000 == 1)
    // {
    //   Flag_7200000 = 0;
    //   HAL_ADC_Stop(&hadc1);
    //   HAL_ADC_Stop(&hadc2);
    //   HAL_ADC_Stop(&hadc3);
    //   HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value_buffer_u32, ADC_NUMBER);
    // }
    // else
    // {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value_buffer_u32, ADC_NUMBER);
    // }
  }
}

/**
 * @brief MultiMode ADC值数组转换函数
 * @retval None
 * @note 此函数将ADC值数组中的高16位和低16位分别存储到adc_value_u32数组的中
 */
void ADC_MultiModeBufferConvert(uint32_t *adc_value_buffer_u32, uint32_t *adc_value_u32, uint16_t size)
{
  for (int i = 0; i < size; i++)
  {
    adc_value_u32[2 * i] = adc_value_buffer_u32[i] & 0x0000FFFF;
    adc_value_u32[2 * i + 1] = (adc_value_buffer_u32[i] & 0xFFFF0000 >> 16);
  }
}

// void ADC_Change_Fs(void)
// {
//   static uint8_t Flag_7200000 = 0; // 定义7.2M标志
//   // 停止定时器
//   HAL_TIM_Base_Stop(&htimx);
//   // 设置新的预分频值
//   __HAL_TIM_SET_PRESCALER(&htimx, OSC_Fs_Arr_config[OSC_Config.OSC_Fs_Index][0] - 1);
//   // 设置新的自动重装载值
//   __HAL_TIM_SET_AUTORELOAD(&htimx, OSC_Fs_Arr_config[OSC_Config.OSC_Fs_Index][1] - 1);
//   // 产生更新事件
//   htimx.Instance->EGR = TIM_EGR_UG;

//   if (OSC_Config.OSC_Fs_Index == 5)
//   {
//     Flag_7200000 = 1;
//     HAL_ADC_Stop(&hadc1);
//     HAL_ADC_Stop(&hadc2);
//     HAL_ADC_Stop(&hadc3);
//     // 第二步：重新配置ADC通用控制寄存器（CCR）为三重交替模式
//     ADC_MultiModeTypeDef multimode = {0};
//     multimode.Mode = ADC_TRIPLEMODE_INTERL;                    // 设置为三重交替模式
//     multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;             // DMA访问模式2
//     multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES; // 设置采样间隔[1,3](@ref)

//     // 使用HAL库函数配置多重模式（此配置主要作用于ADC1，因为ADC1通常作为主ADC）
//     if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//     {
//       Error_Handler();
//     }
//     // 第三步：重新配置ADC1的DMA请求
//     ADC_DMA_Start(7200000);
//   }
//   else if (Flag_7200000){
//     Flag_7200000 = 0;
//   }
//   // 重新启动定时器
//   HAL_TIM_Base_Start(&htimx);
// }

/**
 * @brief ADC频率切换函数
 * @retval None
 */
void ADC_Change_Fs(void)
{
  static uint8_t Flag_7200000 = 0; // 定义7.2M标志
  // 停止定时器
  HAL_TIM_Base_Stop(&adc_htimx);
  // 设置新的预分频值
  __HAL_TIM_SET_PRESCALER(&adc_htimx, OSC_Fs_Arr_config[OSC_Config.OSC_Fs_Index][0] - 1);
  // 设置新的自动重装载值
  __HAL_TIM_SET_AUTORELOAD(&adc_htimx, OSC_Fs_Arr_config[OSC_Config.OSC_Fs_Index][1] - 1);
  // 产生更新事件
  adc_htimx.Instance->EGR = TIM_EGR_UG;

  if (OSC_Config.OSC_Fs_Index == 5)
  {
    Flag_7200000 = 1;
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    /* 三重交替必须用连续转换 + 软件触发（参考 ST 社区交替采样例程） */
    ADC_ChannelConfTypeDef ch = {0};
    /* ADC1 主：连续转换、软件触发 */
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    /* ADC2/3 从：连续转换、软件触发 */
    hadc2.Init.ContinuousConvMode = ENABLE;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
    hadc3.Init.ContinuousConvMode = ENABLE;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) Error_Handler();
    /* 重新配置通道 */
    ch.Channel = ADC_CHANNEL_1; ch.Rank = 1; ch.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &ch);
    HAL_ADC_ConfigChannel(&hadc2, &ch);
    HAL_ADC_ConfigChannel(&hadc3, &ch);
    /* 第二步：重新配置ADC通用控制寄存器（CCR）为三重交替模式 */
    ADC_MultiModeTypeDef multimode = {0};
    multimode.Mode = ADC_TRIPLEMODE_INTERL;                    // 设置为三重交替模式
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;             // DMA访问模式2
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES; // 设置采样间隔[1,3](@ref)

    // 使用HAL库函数配置多重模式（此配置主要作用于ADC1，因为ADC1通常作为主ADC）
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
      Error_Handler();
    }
    // 第三步：重新配置ADC1的DMA请求
    ADC_DMA_Start(7200000);
  }
  else if (Flag_7200000)
  {
    Flag_7200000 = 0;
    /* 停止 multi-mode DMA（如果在 multi 模式下） */
    HAL_ADCEx_MultiModeStop_DMA(&hadc1); // 如果不是 multi-mode 会安全返回或报错，请检查返回值

    /* 单独停止每个 ADC 的 DMA（如果曾分别启动过）*/
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    /* 若 HAL 停止失败，强制 Abort DMA */
    if (hadc1.DMA_Handle)
      HAL_DMA_Abort(hadc1.DMA_Handle);
    if (hadc2.DMA_Handle)
      HAL_DMA_Abort(hadc2.DMA_Handle);
    if (hadc3.DMA_Handle)
      HAL_DMA_Abort(hadc3.DMA_Handle);
    HAL_Delay(2);
    /* 重新配置 ADC 为独立模式 */
    HAL_ADC_DeInit(&hadc1);
    HAL_ADC_DeInit(&hadc2);
    HAL_ADC_DeInit(&hadc3);
    HAL_Delay(2);
    // printf("ADC->CCR=0x%08X\n", ADC->CCR);
    MX_ADC1_Init(); // 独立模式配置
    MX_ADC2_Init();
    MX_ADC3_Init();
    /* Ensure ADC common is set to independent mode (override any multimode set elsewhere) */
    ADC_MultiModeTypeDef multimodeCfg;
    multimodeCfg.Mode = ADC_MODE_INDEPENDENT;
    multimodeCfg.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
    multimodeCfg.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES; /* any valid value */
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimodeCfg) != HAL_OK)
    {
      /* handle error if needed */
      Error_Handler();
    }
    // printf("ADC->CCR=0x%08X\n", ADC->CCR);
    HAL_Delay(2);
    HAL_TIM_Base_Start(&adc_htimx);
    HAL_ADC_Start_DMA(&hadcx, (uint32_t *)adc_value_buffer_u32, ADC_NUMBER);
  }
  // 重新启动定时器
  HAL_TIM_Base_Start(&adc_htimx);
}
/**
 * @brief 快速移动平均滤波函数
 * @param in 输入数组
 * @param out 输出数组
 * @retval None
 */
void Filter_FastMA(uint32_t *in, uint32_t *out)
{
  uint32_t sum = 0;
  // 初始求和
  for (int i = 0; i < ADC_NUMBER; i++)
    sum += in[i];

  out[0] = sum / ADC_NUMBER;

  for (int i = 1; i < ADC_NUMBER; i++)
  {
    sum = sum - in[i - 1] + in[(i + ADC_NUMBER - 1) % ADC_NUMBER];
    out[i] = sum / ADC_NUMBER;
  }
}

/**
 * @brief 滤波函数
 * @param in 输入数组
 * @param out 输出数组
 * @param alpha 滤波系数
 * @retval None
 */
void Filter_IIR(uint32_t *in, uint32_t *out, float alpha)
{
  out[0] = in[0];
  for (int i = 1; i < ADC_NUMBER; i++)
  {
    out[i] = (uint32_t)(alpha * in[i] + (1.0f - alpha) * out[i - 1]);
  }
}
/**
 * @brief 缩放X轴函数
 * @param in 输入数组
 * @param out 输出数组
 * @param scale_percent 缩放比例(%)
 * @retval None
 */
void Scale_X_Axis(uint32_t *in, uint32_t *out, float scale_percent)
{
  float step = 100.0f / scale_percent; // 例：200% → step=0.5

  for (int i = 0; i < ADC_NUMBER; i++)
  {
    float idx = i * step;

    int i0 = (int)idx;
    int i1 = i0 + 1;

    if (i1 >= ADC_NUMBER)
      i1 = ADC_NUMBER - 1;

    float t = idx - i0; // fractional

    // 线性插值
    out[i] = (uint32_t)(in[i0] * (1.0f - t) + in[i1] * t);
  }
}
/**
 * @brief ADC数据提取函数
 * @param adc_value ADC值数组
 * @param len 数组长度
 * @retval None
 */
void ADC_DataHandle(uint32_t *adc_value, uint16_t begin, uint16_t len)
{
  uint32_t Index;
  // 将ADC值转换为电压值
  for (int i = 0; i < len; i++)
  {
    adc_V[i] = adc_value[i + begin] / 4095.0f * 3.3f; // 转换为电压值
  }
  arm_max_f32(adc_V, len, &OSC_MeasureData.VMax, &Index);
  arm_min_f32(adc_V, len, &OSC_MeasureData.VMin, &Index);
  OSC_MeasureData.VPP = OSC_MeasureData.VMax - OSC_MeasureData.VMin;
}

uint8_t Auto_Measure(void)
{
  uint8_t HaveNewData = 1;
  static uint8_t state = 0;

  if (state == 0) // 第一次FFT
  {
    OSC_Config.OSC_Auto_Success = 0;
    state = 1;
    HaveNewData = 0;
    FFT_App(&OSC_MeasureData.Freq_FFT);
    if (OSC_MeasureData.Freq_FFT >= 49500 && OSC_MeasureData.Freq_FFT <= 500000)
    {
      OSC_Config.OSC_Auto_Enable = 0;
      state = 99;
    }
    else
    {
      OSC_Config.OSC_Fs_Index = 3;
      Config_Update();
      ADC_Change_Fs();
    }
  }
  if (state == 1) // 第二次FFT
  {
    if (HaveNewData)
    {
      state = 2;
      HaveNewData = 0;
      FFT_App(&OSC_MeasureData.Freq_FFT);
      if (OSC_MeasureData.Freq_FFT >= 4950 && OSC_MeasureData.Freq_FFT <= 50000)
      {
        OSC_Config.OSC_Auto_Enable = 0;
        state = 99;
      }
      else
      {
        OSC_Config.OSC_Fs_Index = 2;
        Config_Update();
        ADC_Change_Fs();
      }
    }
  }
  if (state == 2) // 第三次FFT
  {
    if (HaveNewData)
    {
      state = 3;
      HaveNewData = 0;
      FFT_App(&OSC_MeasureData.Freq_FFT);
      if (OSC_MeasureData.Freq_FFT >= 480 && OSC_MeasureData.Freq_FFT <= 5000)
      {
        OSC_Config.OSC_Auto_Enable = 0;
        state = 99;
      }
      else
      {
        OSC_Config.OSC_Fs_Index = 1;
        Config_Update();
        ADC_Change_Fs();
      }
    }
  }
  if (state == 3) // 第四次FFT
  {
    if (HaveNewData)
    {
      state = 4;
      HaveNewData = 0;
      FFT_App(&OSC_MeasureData.Freq_FFT);
      OSC_Config.OSC_Auto_Enable = 0;
      state = 99;
    }
  }
  if (state == 99) // 结束
  {
    state = 0;
    OSC_Config.OSC_Auto_Success = 1;
    if (OSC_MeasureData.Freq_FFT > 15000)
    {
      OSC_Config.OSC_Fs_Index = 5;
      Config_Update();
      ADC_Change_Fs();
    }
    else if (OSC_MeasureData.Freq_FFT > 2000 && OSC_MeasureData.Freq_FFT <= 15000)
    {
      OSC_Config.OSC_Fs_Index = 4;
      Config_Update();
      ADC_Change_Fs();
    }
    else if (OSC_MeasureData.Freq_FFT > 200 && OSC_MeasureData.Freq_FFT <= 2000)
    {
      OSC_Config.OSC_Fs_Index = 3;
      Config_Update();
      ADC_Change_Fs();
    }
    else if (OSC_MeasureData.Freq_FFT > 20 && OSC_MeasureData.Freq_FFT <= 200)
    {
      OSC_Config.OSC_Fs_Index = 2;
      Config_Update();
      ADC_Change_Fs();
    }
    else if (OSC_MeasureData.Freq_FFT > 0 && OSC_MeasureData.Freq_FFT <= 20)
    {
      OSC_Config.OSC_Fs_Index = 1;
      Config_Update();
      ADC_Change_Fs();
    }
  }
  printf("n4.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.Freq_FFT+0.5f));
  return 0;
}

