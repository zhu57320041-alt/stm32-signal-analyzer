#include "HMI.h"

/**
 * @brief  发送一帧数据
 * @param  data 数据指针
 * @param  len 发送数据长度
 * @param  totallen 总数据长度
 * @param  offset 数据偏移量
 * @retval None
 */
void HMI_SeedFrame_Wave(uint32_t *data, uint16_t len, uint16_t totallen, int16_t offset)
{
  if (offset < 0 || offset > totallen - len) // 检查偏移量是否有效
    return;
  printf("addt s0.id,0,%d\xff\xff\xff", len);
  // 等待适量时间
  HAL_Delay(20);
  for (int i = 0; i < len; i++)
  {
    printf("%c", (data[i + offset]) / 16);
  }
  // 确保透传结束，以免影响下一条指令
  printf("\x01\xff\xff\xff");
  HAL_Delay(1);
}
/**
 * @brief  发送测量数据
 * @param  None
 * @retval None
 */
void HMI_SeedFrame_MeasureData(void)
{
  printf("n0.val=%d\xFF\xFF\xFF", (int)(OSC_Config.OSC_Fs));
  printf("x2.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.VPP * 1000));
  printf("x3.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.VMax * 1000));
  printf("x4.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.VMin * 1000));
  printf("n2.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.Freq_TIM + 0.5f));
  printf("n3.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.Duty + 0.5f));
  // printf("Freq=%f\n", (OSC_MeasureData.Freq));
  // printf("Duty=%f\n", (OSC_MeasureData.Duty));
}

void HMI_SeedFrame_FFT(float *data, uint16_t len)
{
  printf("n4.val=%d\xFF\xFF\xFF", (int)(OSC_Config.OSC_Fs / 2));
  printf("n5.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.Freq_FFT + 0.5f));
  printf("n6.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.Freq_FFT_H2 + 0.5f)); // 二次谐波
  printf("n7.val=%d\xFF\xFF\xFF", (int)(OSC_MeasureData.Freq_FFT_H3 + 0.5f)); // 三次谐波
  float32_t max = 0;
  uint32_t index = 0;

  data[0] = 0;                          // 去除直流分量
  arm_max_f32(data, len, &max, &index); // 找到最大值和索引
  // printf("Max: %f at index: %d\n", max, index);
  int P = max / 255;
  // for (int i = 0; i < len; i++)
  // {
  //   printf("%d\n", ((int)data[i]) / P);
  // }

  printf("addt s0.id,0,%d\xff\xff\xff", len);
  // 等待适量时间
  HAL_Delay(20);
  for (int i = len - 1; i >= 0; i--)
  {
    printf("%c", ((int)data[i]) / P);
  }
  // 确保透传结束，以免影响下一条指令
  printf("\x01\xff\xff\xff");
}

/**
 * @brief 处理接收到的指令
 * @param RxData 接收到的数据
 * @param Size 数据长度
 * @return 0 成功并修改输出参数结构体, 1 设置范围超出, 2 指令错误
 */
uint8_t HMI_ReceiveHandle(uint8_t *RxData, uint8_t Size)
{
  if (RxData[6] != 0x00) // 简单指令校验
  {
    return 2;
  }
  else if (RxData[0] == 0xAA) // 开启采样
  {
    OSC_Config.OSC_Enable = 1;
    return 0;
  }
  else if (RxData[0] == 0xBB) // 关闭采样
  {
    OSC_Config.OSC_Enable = 0;
    return 0;
  }
  else if (RxData[0] == 0xBC) // 自动
  {
    OSC_Config.OSC_Auto_Enable = 1;
    if (OSC_Config.OSC_Auto_Success == 1)
    {
      OSC_Config.OSC_Fs_Index = 4;
      Config_Update();
      ADC_Change_Fs();
    }
  }
  else if (RxData[0] == 0xCC)
  {
    OSC_Config.OSC_FFT_Enable = 1;
    return 0;
  }
  else if (RxData[0] == 0xDD)
  {
    OSC_Config.OSC_FFT_Enable = 0;
    return 0;
  }
  else if (RxData[0] == 0xEE) // 开关DAC
  {
    OSC_Config.OSC_DAC_Enable = !OSC_Config.OSC_DAC_Enable;
    DAC_Param.HaveChange_Flag = 1;
    return 0;
  }
  else if (RxData[0] == 0x12) // 串口打印波形数据开关 (0x00=开, 0x01=关)
  {
    OSC_Config.OSC_Print_Enable = (RxData[1] == 0x00) ? 1 : 0;
    return 0;
  }
  else if (RxData[0] == 0x01) // 设置采样频率
  {
    if (RxData[1] == 0x00)
    {
      if (OSC_Config.OSC_Fs_Index <= 0)
      {
        printf("n0.val=%d\xFF\xFF\xFF", OSC_Config.OSC_Fs);
        return 1;
      }
      OSC_Config.OSC_Fs_Index--;
    }
    else if (RxData[1] == 0x01)
    {
      if (OSC_Config.OSC_Fs_Index >= FS_NUM - 1)
      {
        printf("n0.val=%d\xFF\xFF\xFF", OSC_Config.OSC_Fs);
        return 1;
      }
      OSC_Config.OSC_Fs_Index++;
    }
    Config_Update();
    ADC_Change_Fs();
    // printf("ADC->CCR=0x%08X\n", ADC->CCR);
    printf("n0.val=%d\xFF\xFF\xFF", OSC_Config.OSC_Fs);
    return 0;
  }
  else if (RxData[0] == 0x02) // 设置横轴缩放比例
  {
    if (RxData[1] == 0x00)
    {
      if (OSC_Config.Scale_X_Axis <= 100)
      {
        printf("n1.val=%d\xFF\xFF\xFF", OSC_Config.Scale_X_Axis);
        return 1;
      }
      OSC_Config.Scale_X_Axis -= 50;
    }
    else if (RxData[1] == 0x01)
    {
      if (OSC_Config.Scale_X_Axis >= 1000)
      {
        printf("n1.val=%d\xFF\xFF\xFF", OSC_Config.Scale_X_Axis);
        return 1;
      }
      OSC_Config.Scale_X_Axis += 50;
    }
    printf("n1.val=%d\xFF\xFF\xFF", OSC_Config.Scale_X_Axis);
    return 0;
  }
  else if (RxData[0] == 0x10) // 设置波形参数
  {
    if (RxData[1] == 0x00)
    {
      DAC_Param.type = 0;
    }
    else if (RxData[1] == 0x01)
    {
      DAC_Param.type = 1;
    }
    else if (RxData[1] == 0x02)
    {
      DAC_Param.type = 2;
    }
    else if (RxData[1] == 0x03)
    {
      DAC_Param.type = 3;
    }
    else if (RxData[1] == 0x04)
    {
      DAC_Param.type = 4;
    }
    DAC_Param.HaveChange_Flag = 1;
    return 0;
  }
  else if (RxData[0] == 0x11) // 设置波形参数
  {
    if (RxData[1] == 0x00)
    {
      uint32_t temp2, temp3, temp4;
      temp2 = RxData[3];
      temp3 = RxData[4];
      temp4 = RxData[5];
      DAC_Param.Freq = temp4 << 24 | temp3 << 16 | temp2 << 8 | RxData[2];
    }
    else if (RxData[1] == 0x01)
    {
      uint16_t temp2;
      temp2 = RxData[3];
      DAC_Param.VPP = (float)(temp2 << 8 | RxData[2]) / 100.0f;
    }
    else if (RxData[1] == 0x02)
    {
      uint16_t temp2;
      temp2 = RxData[3];
      DAC_Param.Duty = (float)(temp2 << 8 | RxData[2]) / 100.0f;
    }
    else if (RxData[1] == 0x03)
    {
      uint16_t temp2;
      temp2 = RxData[3];
      DAC_Param.offset = (float)(temp2 << 8 | RxData[2]) / 100.0f;
    }
    DAC_Param.HaveChange_Flag = 1;
    return 0;
  }
  return 2;
}
