#include "OSC_Trigger.h"
/**
 * @brief 带迟滞的上升沿触发检测
 * @param data ADC数据数组
 * @param size 数据长度
 * @param threshold 触发电平
 * @param hysteresis 迟滞宽度（例如20）
 * @return 触发点的索引，未找到返回-1
 */
int16_t OSC_Rising_Edge_Trigger(uint32_t *data, int size, int16_t threshold, int16_t hysteresis)
{
  int16_t low_thresh = threshold - hysteresis;  // 低阈值
  int16_t high_thresh = threshold + hysteresis; // 高阈值

  for (int i = 2; i < size; i++)
  {
    // 条件：前一点低于低阈值，且当前点高于高阈值
    if ((data[i - 2] < low_thresh) && (data[i - 1] < low_thresh) && (data[i] >= high_thresh) && (data[i + 1] >= high_thresh))
    {
      return i - 2;
    }
  }
  return -1;
}

// 下降沿触发可类似实现
int16_t OSC_Falling_Edge_Trigger(uint32_t *data, int size, int16_t threshold, int16_t hysteresis)
{
  int16_t low_thresh = threshold - hysteresis;
  int16_t high_thresh = threshold + hysteresis;

  for (int i = 1; i < size; i++)
  {
    // 条件：前一点高于高阈值，且当前点低于低阈值
    if ((data[i - 1] > high_thresh) && (data[i] <= low_thresh))
    {
      return i;
    }
  }
  return -1;
}
/**
 * @brief 简单的触发检测
 * @param data ADC数据数组
 * @param size 数据长度
 * @param threshold 触发电平
 * @return 触发点的索引，未找到返回-1
 */
int16_t OSC_Trigger_Potential(uint32_t *data, int size, int16_t threshold)
{
  for (int i = size / 3; i < size; i++)
  {
    // 条件：前一点低于阈值，且当前点高于阈值
    if ((data[i] >= threshold))
    {
      return i;
    }
  }
  return -1;
}