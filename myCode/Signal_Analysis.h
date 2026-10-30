#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

#include <stdint.h>

#define SIGNAL_ANALYSIS_FFT_SIZE          4096U
#define SIGNAL_ANALYSIS_SAMPLE_RATE_HZ    2000000UL
#define SIGNAL_ANALYSIS_MAX_FREQUENCY_HZ   1000000UL
#define SIGNAL_ANALYSIS_MAX_COMPONENTS    3U
#define SIGNAL_ANALYSIS_WAVEFORM_POINTS   480U

typedef struct
{
  float frequency_hz;
  float amplitude_peak_v;
  float amplitude_rms_v;
  float phase_rad;
  uint8_t harmonic_order;
} SignalAnalysisComponent_t;

typedef struct
{
  float adc_dc_voltage_v;
  float sampled_peak_to_peak_v;
  float peak_to_peak_v;
  float true_rms_v;
  float fundamental_hz;
  uint8_t component_count;
  SignalAnalysisComponent_t components[SIGNAL_ANALYSIS_MAX_COMPONENTS];
  float waveform_v[SIGNAL_ANALYSIS_WAVEFORM_POINTS];
} SignalAnalysisResult_t;

void SignalAnalysis_Init(void);
uint8_t SignalAnalysis_Process(const uint16_t *samples,
                               SignalAnalysisResult_t *result);
float SignalAnalysis_AdcCodeToVoltage(uint16_t code);
float SignalAnalysis_GetSpectrumPeakV(uint16_t bin);

#endif
