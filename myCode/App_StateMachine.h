#ifndef APP_STATE_MACHINE_H
#define APP_STATE_MACHINE_H

#include "main.h"

typedef enum
{
  APP_MODE_Q1_WAVEFORM = 0,
  APP_MODE_Q1_MEASURE,
  APP_MODE_Q1_SPECTRUM,
  APP_MODE_Q1_COMPONENTS,
  APP_MODE_Q2_WAVEFORM,
  APP_MODE_Q2_MEASURE,
  APP_MODE_Q2_SPECTRUM,
  APP_MODE_Q2_COMPONENTS,
  APP_MODE_Q3_WAVEFORM,
  APP_MODE_Q3_MEASURE,
  APP_MODE_Q3_SPECTRUM,
  APP_MODE_Q3_COMPONENTS,
  APP_MODE_UNIT_RAW,
  APP_MODE_COUNT
} AppMode_t;

typedef enum
{
  APP_STATE_CAPTURE_START = 0,
  APP_STATE_CAPTURE_WAIT,
  APP_STATE_PROCESS,
  APP_STATE_OUTPUT,
  APP_STATE_RAW_WAIT,
  APP_STATE_FAULT
} AppState_t;

void App_Init(void);
void App_Run(void);
void App_SetMode(AppMode_t mode);
AppMode_t App_GetMode(void);
void App_SetDisplayCycles(uint8_t cycles);
uint8_t App_GetDisplayCycles(void);
void App_HmiUartIrqHandler(void);

#endif
