#ifndef DAC_DMA_H
#define DAC_DMA_H
#include "main.h"
#include "arm_math.h"
#include "stdio.h"
#define FREQ_MIN 1.0f            // Hz
#define FREQ_MAX 500000.0f        // Hz
#define TIMER_CLOCK_HZ 72000000UL // 定时器时钟频率，单位 Hz
#define DAC_MAX_POINTS 1440
// #define BASE_FS_HZ (TIMER_CLOCK_HZ / DAC_POINTS) // Hz
#define dac_htimx htim2
#define DAC_CHANNEL_x DAC_CHANNEL_1
#define dac_hdacx hdac

enum DAC_WAVEFORM_TYPE
{
  SINE,
  SQUARE,
  TRIANGLE,
  SAW,
  SAW_DOWN
};

typedef struct
{
  uint16_t Point;
}DAC_Config_t;



void DAC_DMA_Init(void);
void DAC_DMA_Start(void);
void DAC_DMA_Stop(void);
void DAC_DMA_ChangeOut(uint8_t type, uint32_t Freq, float Vpp, float offset, float duty);


#endif // DAC_DMA_H