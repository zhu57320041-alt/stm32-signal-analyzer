#include "DAC_DMA.h"
static uint16_t out_table[DAC_MAX_POINTS];

DAC_Config_t DAC_Config;

extern TIM_HandleTypeDef dac_htimx;
extern DAC_HandleTypeDef dac_hdacx;

void GenSineTable(float Vpp, float offset, uint16_t Point)
{
  float off = 4095 * (offset / 3.3f); // 转换为 DAC 输出值
  for (int i = 0; i < Point; i++)
  {
    float t = 2.0f * PI * i / Point;
    float v = 2047.5f * (arm_sin_f32(t) + 1.0f);

    v = (int16_t)(v * (Vpp / 3.3f) + off + 0.5f);
    if (v < 0)
      v = 0;
    if (v > 4095)
      v = 4095;
    out_table[i] = v;
  }
}

void GenSquareTable(float Vpp, float duty, float offset, uint16_t Point)
{
  float off = 4095 * (offset / 3.3f); // 转换为 DAC 输出值
  float temp;
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 1.0f)
    duty = 1.0f;

  int high_len = (int)(Point * duty);

  for (int i = 0; i < Point; i++)
  {
    if (i < high_len)
    {
      if ((temp = (uint16_t)(4095 * (Vpp / 3.3f) + off + 0.5f)) < 4095)
        out_table[i] = temp; // 高电平
      else
        out_table[i] = 4095;
    }
    else if ((temp = off) > 0)
      out_table[i] = (uint16_t)(temp + 0.5f); // 低电平
    else
      out_table[i] = 0;
  }
}

void GenTriangleTable(float Vpp, float offset, uint16_t Point)
{
  float off = 4095 * (offset / 3.3f); // 转换为 DAC 输出值
  for (int i = 0; i < Point; i++)
  {
    float pos = (float)i / Point;
    float v;
    if (pos < 0.5f)
      v = 4095.0f * (pos * 2.0f); // 上升
    else
      v = 4095.0f * (2.0f - pos * 2.0f); // 下降

    v = (uint16_t)(v * (Vpp / 3.3f) + off + 0.5f);
    if (v < 0)
      v = 0;
    if (v > 4095)
      v = 4095;
    out_table[i] = v;
  }
}

// void GenSawTable(float Vpp, float offset, uint16_t Point)
// {
//   float off = 4095 * (offset / 3.3f); // 转换为 DAC 输出值
//   for (int i = 0; i < Point; i++)
//   {
//     float pos = (float)i / Point;
//     float v = 4095.0f * pos;
//     v = (uint16_t)(v * (Vpp / 3.3f) + off + 0.5f);
//     if (v < 0)
//       v = 0;
//     if (v > 4095)
//       v = 4095;
//     out_table[i] = v;
//   }
// }

// // 生成可调占空比的锯齿波表
// // Vpp   : 峰峰值，单位 V（建议范围 0..3.3）
// // offset: 直流偏移，单位 V（0..3.3）
// // Point : 表长度（每周期点数）
// // duty  : 上升阶段占比（0.0 .. 1.0）
// // 说明：输出被映射到 0..4095（12-bit DAC）
// void GenSawTable(float Vpp, float offset, uint16_t Point, float duty)
// {
//   if (Point == 0)
//     return;

//   // 限制并规范化输入
//   if (Vpp < 0.0f)
//     Vpp = 0.0f;
//   if (Vpp > 3.3f)
//     Vpp = 3.3f;
//   if (offset < 0.0f)
//     offset = 0.0f;
//   if (offset > 3.3f)
//     offset = 3.3f;
//   if (duty <= 0.0f)
//     duty = 0.0f;
//   if (duty >= 1.0f)
//     duty = 1.0f;

//   // 将 offset 转为 DAC 单位
//   const float dac_full = 4095.0f;
//   float off = dac_full * (offset / 3.3f);

//   // 上升段采样点数（至少 1）
//   uint32_t riseSamples = (uint32_t)(Point * duty + 0.5f);
//   if (riseSamples == 0)
//     riseSamples = 1;
//   if (riseSamples > Point)
//     riseSamples = Point;

//   // 实际上我们按上升段线性从 0 -> Vpp 映射，
//   // 上升段后点直接置为基线（回落到 0 + offset）
//   for (uint32_t i = 0; i < Point; i++)
//   {
//     float v_dac; // 0..4095 范围的电压量化值（不含 offset）

//     if (i < riseSamples)
//     {
//       // 当 riseSamples == 1 时，直接取顶点（避免除 0）
//       if (riseSamples == 1)
//       {
//         v_dac = dac_full * (Vpp / 3.3f);
//       }
//       else
//       {
//         // 线性上升：t 从 0 到 1
//         float t = (float)i / (float)(riseSamples - 1);
//         v_dac = dac_full * (t * (Vpp / 3.3f));
//       }
//     }
//     else
//     {
//       // 上升结束后瞬间回落到基线
//       v_dac = 0.0f;
//     }

//     int32_t v = (int32_t)(v_dac + off + 0.5f);
//     if (v < 0)
//       v = 0;
//     if (v > 4095)
//       v = 4095;
//     out_table[i] = (uint16_t)v;
//   }
// }

// #include <stdint.h>
// #include <math.h>

// extern uint16_t out_table[]; // 用户需在别处定义，长度至少为 Point

// 生成：线性上升 -> 线性下降 不对称锯齿（占空比 duty 控制上升占比）
// Vpp   : 峰峰值（从基线 offset 到峰值的电压），单位 V（建议 0..3.3）
// offset: 基线电压（最低点），单位 V（建议 0..3.3）
// Point : 每周期采样点数
// duty  : 上升占比（0.0 .. 1.0）
// 输出：out_table[0..Point-1] 填 0..4095 的 DAC 值
void GenSawTable(float Vpp, float offset, uint16_t Point, float duty)
{
  if (Point == 0)
    return;

  // clamp inputs
  if (Vpp < 0.0f)
    Vpp = 0.0f;
  if (Vpp > 3.3f)
    Vpp = 3.3f;
  if (offset < 0.0f)
    offset = 0.0f;
  if (offset > 3.3f)
    offset = 3.3f;
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 1.0f)
    duty = 1.0f;

  const float dac_full = 4095.0f;
  // 把 offset 映射到 DAC 单位（这是最低点）
  float off_dac = dac_full * (offset / 3.3f);

  // 计算上升 / 下降的采样点数（上升至少 1 点）
  uint32_t riseSamples = (uint32_t)(Point * duty + 0.5f);
  if (riseSamples == 0)
    riseSamples = 1;
  if (riseSamples > Point)
    riseSamples = Point;

  uint32_t fallSamples = Point - riseSamples; // 可能为 0

  // 当 fallSamples == 0 时，整个周期都为上升（单向上升）
  // 当 riseSamples == 0（被禁止，因为我们至少设为1）会导致只下降（但这里不发生）

  for (uint32_t i = 0; i < Point; i++)
  {
    float v_volt; // 电压值，相对于基线 offset 的瞬时幅度（范围 0..Vpp）

    if (i < riseSamples)
    {
      // 上升段：线性从 0 -> Vpp
      if (riseSamples == 1)
      {
        v_volt = Vpp; // 单点上升到顶
      }
      else
      {
        float t = (float)i / (float)(riseSamples - 1); // 0..1
        v_volt = t * Vpp;
      }
    }
    else
    {
      // 下降段：线性从 Vpp -> 0
      if (fallSamples == 0)
      {
        // 不会进来，因为 i < riseSamples 覆盖了所有点
        v_volt = 0.0f;
      }
      else if (fallSamples == 1)
      {
        // 只有一个下降点，直接回到基线
        v_volt = 0.0f;
      }
      else
      {
        // i 从 riseSamples .. Point-1
        uint32_t idx = i - riseSamples;                  // 0 .. fallSamples-1
        float t = (float)idx / (float)(fallSamples - 1); // 0..1
        // t=0 -> Vpp, t=1 -> 0
        v_volt = (1.0f - t) * Vpp;
      }
    }

    // 把 v_volt + offset 转为 DAC 单位
    float v_total_volt = offset + v_volt; // 0 .. offset+Vpp (确保不超 3.3V)
    // 如果超出 3.3V，则裁剪（以防 Vpp+offset 超出）
    if (v_total_volt < 0.0f)
      v_total_volt = 0.0f;
    if (v_total_volt > 3.3f)
      v_total_volt = 3.3f;

    int32_t v_dac = (int32_t)(dac_full * (v_total_volt / 3.3f) + 0.5f);
    if (v_dac < 0)
      v_dac = 0;
    if (v_dac > 4095)
      v_dac = 4095;
    out_table[i] = (uint16_t)v_dac;
  }
}

void GenSawDownTable(float Vpp, float offset, uint16_t Point)
{
  float off = 4095 * (0.5f / 3.3f); // 转换为 DAC 输出值
  for (int i = 0; i < Point; i++)
  {
    float pos = (float)i / Point;
    float v = 4095.0f * (1.0f - pos);
    out_table[i] = (uint16_t)(v * (Vpp / 3.3f) + off + 0.5f);
  }
}

int8_t SetSineFreq(float freq_hz, uint16_t Point)
{
  if (freq_hz < FREQ_MIN)
    freq_hz = FREQ_MIN;
  if (freq_hz > FREQ_MAX)
    freq_hz = FREQ_MAX;

  // 计算 ARR： ARR = BASE_FS_HZ / freq - 1
  float desired = (float)(TIMER_CLOCK_HZ / Point) / freq_hz - 1.0f;
  if (desired < 0.0f)
    desired = 0.0f;
  uint32_t arr = (uint32_t)(desired + 0.5f); // 四舍五入

  // 安全检测（TIM2 为 32-bit）
  if (arr > 0xFFFFFFFFUL)
    return -1;

  HAL_TIM_Base_Stop(&dac_htimx);
  __HAL_TIM_SET_AUTORELOAD(&dac_htimx, arr);
  __HAL_TIM_SET_COUNTER(&dac_htimx, 0);
  // 产生更新事件
  dac_htimx.Instance->EGR = TIM_EGR_UG;
  // 定时器开始
  HAL_TIM_Base_Start(&dac_htimx);

  return 0;
}

/**
 * @brief 生成波形
 * @param type 波形类型
 * @param Freq 频率
 * @param Vpp 峰峰值
 * @param offset 偏移电压
 * @param duty 占空比（仅用于 SQUARE）
 */
void GenWaveform(uint8_t type, uint32_t Freq, float Vpp, float offset, float duty)
{
  DAC_Config.Point = (uint16_t)(8000000 / Freq + 0.5f);
  if (DAC_Config.Point > DAC_MAX_POINTS)
    DAC_Config.Point = DAC_MAX_POINTS;
  if (DAC_Config.Point < 1)
    DAC_Config.Point = 1;

  if (type == SINE)
  {
    GenSineTable(Vpp, offset, DAC_Config.Point);
  }
  else if (type == SQUARE)
  {
    GenSquareTable(Vpp, duty, offset, DAC_Config.Point);
  }
  else if (type == TRIANGLE)
  {
    GenTriangleTable(Vpp, offset, DAC_Config.Point);
  }
  else if (type == SAW)
  {
    GenSawTable(Vpp, offset, DAC_Config.Point, duty);
  }
}

void DAC_DMA_Init(void)
{
  GenWaveform(SINE, 10000, 1, 0.5, 0.5);
  // // memcpy(out_table, sine_table, DAC_POINTS * sizeof(uint16_t));
  // for (int i = 0; i < DAC_POINTS; i++)
  // {
  //   printf("%d\n", out_table[i]);
  // }
}

void DAC_DMA_Start(void)
{
  // 启动 TIM6
  HAL_TIM_Base_Start(&dac_htimx);
  // DAC CH1 DMA 输出
  HAL_DAC_Start_DMA(&dac_hdacx,
                    DAC_CHANNEL_x,
                    (uint32_t *)out_table,
                    DAC_Config.Point,
                    DAC_ALIGN_12B_R);
}
void DAC_DMA_Stop(void)
{
  HAL_DAC_Stop_DMA(&dac_hdacx, DAC_CHANNEL_x);
}

/**
 * @brief 改变波形
 * @param type 波形类型
 * @param Freq 频率
 * @param Vpp 峰峰值
 * @param offset 偏移电压
 * @param duty 占空比（仅用于 SQUARE）
 * @return 无
 * @note 需再调用 DAC_DMA_Start 函数启动
 */
void DAC_DMA_ChangeOut(uint8_t type, uint32_t Freq, float Vpp, float offset, float duty)
{
  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
  // 修改数组
  GenWaveform(type, Freq, Vpp, offset, duty);
  SetSineFreq(Freq, DAC_Config.Point);
}