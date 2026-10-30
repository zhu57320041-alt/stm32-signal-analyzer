#include "fre_measure.h"

extern TIM_HandleTypeDef Fre_htimx;

// static uint32_t rise_last = 0; // 上一次上升沿
// static uint32_t fall_last = 0; // 上一次下降沿
static uint32_t period_cnt = 0;
static uint32_t high_cnt = 0;

// static uint8_t period_ok = 0;
// static uint8_t high_ok = 0;

void Fre_Measure_Init(void)
{
  HAL_TIM_IC_Start_IT(&Fre_htimx, TIM_CHANNEL_1); // Rising
  HAL_TIM_IC_Start_IT(&Fre_htimx, TIM_CHANNEL_2); // Falling
}

// /**
//  * @brief  获取频率和占空比
//  * @return 1 = 已计算出有效数据
//  */
// uint8_t Fre_Measure_Get_Fre(float *frequency, float *duty)
// {
//   if (period_ok && high_ok)
//   {
//     *frequency = TIM_FREQ / period_cnt; // 定时器2MHz
//     *duty = ((float)high_cnt / (float)period_cnt) * 100.0f;

//     period_ok = 0;
//     high_ok = 0;
//     return 1;
//   }
//   return 0;
// }

// /**
//  * @brief  输入捕获中断回调
//  */
// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
// {
//   if (htim->Instance != Fre_htimx.Instance)
//     return;

//   //-----------------------------
//   // 上升沿：周期
//   //-----------------------------
//   if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//   {
//     uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

//     period_cnt = (now >= rise_last) ? (now - rise_last) : (0xFFFFFFFF - rise_last + now + 1);

//     rise_last = now;
//     period_ok = 1; // 周期有效
//   }

//   //-----------------------------
//   // 下降沿：高电平时间
//   //-----------------------------
//   if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//   {
//     uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

//     high_cnt = (now >= rise_last) ? (now - rise_last) : (0xFFFFFFFF - rise_last + now + 1);

//     fall_last = now;
//     high_ok = 1; // 高电平有效
//   }
// }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
      period_cnt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
      high_cnt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
  }
}

/**
 * @brief  获取频率和占空比
 * @return 1 = 已计算出有效数据
 */
uint8_t Fre_Measure_Get_Fre(float *frequency, float *duty)
{
  if (period_cnt > 0)
  {
    *frequency = TIM_FREQ / (period_cnt + 1);
    *duty = (float)(high_cnt + 1) / (period_cnt + 1) * 100;
    return 1;
  }
  return 0;
}


