#include "FFT.h"
    
#include "ADC_DMA.h"

void FFT_Init(void)
{
  SignalAnalysis_Init();
}

uint32_t FFT_App(float *Freq)
{
  SignalAnalysisResult_t result;

  if (Freq == NULL)
  {
    return 1U;
  }

  if (SignalAnalysis_Process(ADC_GetBuffer(), &result) == 0U)
  {
    return 1U;
  }

  *Freq = result.fundamental_hz * 100;
  return 0U;
}
