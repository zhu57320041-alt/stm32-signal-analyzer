#ifndef FFT_H
#define FFT_H

#include "Signal_Analysis.h"

#define FFT_LENGTH SIGNAL_ANALYSIS_FFT_SIZE

/* Legacy compatibility entry points. New application code uses Signal_Analysis. */
void FFT_Init(void);
uint32_t FFT_App(float *Freq);

#endif
