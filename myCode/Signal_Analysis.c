#include "Signal_Analysis.h"

#include <float.h>
#include <math.h>

#include "main.h"
#include "arm_math.h"

#define SIGNAL_ANALYSIS_ADC_VREF_V          3.162f
#define SIGNAL_ANALYSIS_FRONTEND_GAIN        1.067f
#define SIGNAL_ANALYSIS_ADC_MAX_CODE         4095.0f
#define SIGNAL_ANALYSIS_MIN_FREQUENCY_HZ     10000.0f
#define SIGNAL_ANALYSIS_PEAK_FLOOR_V         0.002f
#define SIGNAL_ANALYSIS_RELATIVE_THRESHOLD   0.005f
#define SIGNAL_ANALYSIS_NOISE_CLIP_RATIO     0.10f
#define SIGNAL_ANALYSIS_NOISE_MULTIPLIER     6.0f
#define SIGNAL_ANALYSIS_MAX_CANDIDATES       16U
#define SIGNAL_ANALYSIS_MAX_HARMONIC_ORDER   20U
#define SIGNAL_ANALYSIS_TWO_PI               6.28318530717958647692f
#define SIGNAL_ANALYSIS_INV_SQRT_TWO         0.70710678118654752440f

typedef struct
{
  float frequency_hz;
  float amplitude_v;
} SignalPeak_t;

/* These buffers are CPU-only.  The ADC DMA buffer remains in normal SRAM. */
#pragma arm section zidata = "CCMRAM"
static float fft_input[SIGNAL_ANALYSIS_FFT_SIZE];
static float fft_output[SIGNAL_ANALYSIS_FFT_SIZE];
#pragma arm section zidata

static arm_rfft_fast_instance_f32 fft_instance;
static uint8_t signal_analysis_initialized;

float SignalAnalysis_AdcCodeToVoltage(uint16_t code)
{
  return ((float)code * SIGNAL_ANALYSIS_ADC_VREF_V) /
         (SIGNAL_ANALYSIS_ADC_MAX_CODE * SIGNAL_ANALYSIS_FRONTEND_GAIN);
}

static float SignalAnalysis_Window(uint32_t index)
{
  return 0.5f - 0.5f * arm_cos_f32(
      SIGNAL_ANALYSIS_TWO_PI * (float)index /
      (float)SIGNAL_ANALYSIS_FFT_SIZE);
}

static float SignalAnalysis_Magnitude(uint32_t bin)
{
  float real;
  float imaginary;

  if (bin == 0U)
  {
    return fabsf(fft_output[0]);
  }
  if (bin == (SIGNAL_ANALYSIS_FFT_SIZE / 2U))
  {
    return fabsf(fft_output[1]);
  }

  real = fft_output[2U * bin];
  imaginary = fft_output[(2U * bin) + 1U];
  return sqrtf((real * real) + (imaginary * imaginary));
}

static float SignalAnalysis_InterpolatedFrequency(uint32_t bin)
{
  float left = SignalAnalysis_Magnitude(bin - 1U);
  float center = SignalAnalysis_Magnitude(bin);
  float right = SignalAnalysis_Magnitude(bin + 1U);
  float delta = 0.0f;
  float denominator;

  if ((left > 0.0f) && (center > 0.0f) && (right > 0.0f))
  {
    left = logf(left);
    center = logf(center);
    right = logf(right);
    denominator = left - (2.0f * center) + right;
    if (fabsf(denominator) > 1.0e-12f)
    {
      delta = 0.5f * (left - right) / denominator;
      if (delta > 0.5f)
      {
        delta = 0.5f;
      }
      else if (delta < -0.5f)
      {
        delta = -0.5f;
      }
    }
  }

  return ((float)bin + delta) *
         ((float)SIGNAL_ANALYSIS_SAMPLE_RATE_HZ /
          (float)SIGNAL_ANALYSIS_FFT_SIZE);
}

static void SignalAnalysis_InsertPeak(SignalPeak_t *peaks,
                                      uint32_t *count,
                                      float frequency_hz,
                                      float amplitude_v)
{
  uint32_t position;
  uint32_t limit = *count;

  if ((limit == SIGNAL_ANALYSIS_MAX_CANDIDATES) &&
      (amplitude_v <= peaks[limit - 1U].amplitude_v))
  {
    return;
  }

  if (limit < SIGNAL_ANALYSIS_MAX_CANDIDATES)
  {
    (*count)++;
  }
  else
  {
    limit--;
  }

  position = limit;
  while ((position > 0U) && (amplitude_v > peaks[position - 1U].amplitude_v))
  {
    peaks[position] = peaks[position - 1U];
    position--;
  }
  peaks[position].frequency_hz = frequency_hz;
  peaks[position].amplitude_v = amplitude_v;
}

static uint32_t SignalAnalysis_FindPeaks(SignalPeak_t *peaks)
{
  const float bin_width = (float)SIGNAL_ANALYSIS_SAMPLE_RATE_HZ /
                          (float)SIGNAL_ANALYSIS_FFT_SIZE;
  const uint32_t minimum_bin = (uint32_t)(SIGNAL_ANALYSIS_MIN_FREQUENCY_HZ /
                                          bin_width);
  const uint32_t maximum_bin =
      (uint32_t)(((float)SIGNAL_ANALYSIS_MAX_FREQUENCY_HZ / bin_width) +
                 1.0f);
  float maximum = 0.0f;
  float noise_sum = 0.0f;
  float noise_floor = 0.0f;
  float noise_clip;
  float threshold;
  uint32_t noise_count = 0U;
  uint32_t count = 0U;
  uint32_t bin;

  for (bin = minimum_bin; bin <= maximum_bin; ++bin)
  {
    float magnitude = SignalAnalysis_Magnitude(bin);
    if (magnitude > maximum)
    {
      maximum = magnitude;
    }
  }

  noise_clip = maximum * SIGNAL_ANALYSIS_NOISE_CLIP_RATIO;
  for (bin = minimum_bin; bin <= maximum_bin; ++bin)
  {
    float magnitude = SignalAnalysis_Magnitude(bin);
    if (magnitude <= noise_clip)
    {
      noise_sum += magnitude;
      noise_count++;
    }
  }
  if (noise_count > 0U)
  {
    noise_floor = noise_sum / (float)noise_count;
  }

  threshold = noise_floor * SIGNAL_ANALYSIS_NOISE_MULTIPLIER;
  if (threshold < (maximum * SIGNAL_ANALYSIS_RELATIVE_THRESHOLD))
  {
    threshold = maximum * SIGNAL_ANALYSIS_RELATIVE_THRESHOLD;
  }
  if (threshold < (SIGNAL_ANALYSIS_PEAK_FLOOR_V *
                   ((float)SIGNAL_ANALYSIS_FFT_SIZE * 0.25f)))
  {
    threshold = SIGNAL_ANALYSIS_PEAK_FLOOR_V *
                ((float)SIGNAL_ANALYSIS_FFT_SIZE * 0.25f);
  }

  for (bin = minimum_bin; bin <= maximum_bin; ++bin)
  {
    float magnitude = SignalAnalysis_Magnitude(bin);
    if ((magnitude >= threshold) &&
        (magnitude > SignalAnalysis_Magnitude(bin - 1U)) &&
        (magnitude >= SignalAnalysis_Magnitude(bin + 1U)))
    {
      SignalAnalysis_InsertPeak(
          peaks, &count, SignalAnalysis_InterpolatedFrequency(bin),
          magnitude / ((float)SIGNAL_ANALYSIS_FFT_SIZE * 0.25f));
    }
  }
  return count;
}

static float SignalAnalysis_SelectFundamental(const SignalPeak_t *peaks,
                                              uint32_t peak_count)
{
  const float bin_width = (float)SIGNAL_ANALYSIS_SAMPLE_RATE_HZ /
                          (float)SIGNAL_ANALYSIS_FFT_SIZE;
  const float tolerance = 1.5f * bin_width;
  float best_frequency = 0.0f;
  float best_score = -1.0f;
  float weighted_frequency_sum = 0.0f;
  float weighted_order_square_sum = 0.0f;
  uint32_t i;

  for (i = 0U; i < peak_count; ++i)
  {
    uint8_t divisor;
    for (divisor = 1U; divisor <= SIGNAL_ANALYSIS_MAX_HARMONIC_ORDER; ++divisor)
    {
      float fundamental = peaks[i].frequency_hz / (float)divisor;
      float score = 0.0f;
      uint8_t matched_harmonics = 0U;
      uint32_t j;

      if ((fundamental < (SIGNAL_ANALYSIS_MIN_FREQUENCY_HZ - tolerance)) ||
          (fundamental >
           ((float)SIGNAL_ANALYSIS_MAX_FREQUENCY_HZ + tolerance)))
      {
        continue;
      }

      for (j = 0U; j < peak_count; ++j)
      {
        float ratio = peaks[j].frequency_hz / fundamental;
        uint8_t order = (uint8_t)(ratio + 0.5f);
        if ((order >= 1U) &&
            (order <= SIGNAL_ANALYSIS_MAX_HARMONIC_ORDER) &&
            (fabsf(peaks[j].frequency_hz -
                   ((float)order * fundamental)) <= tolerance))
        {
          score += peaks[j].amplitude_v;
          matched_harmonics++;
          if (order == 1U)
          {
            score += 0.25f * peaks[j].amplitude_v;
          }
        }
      }

      if (matched_harmonics >= 2U)
      {
        score *= 1.5f;
      }
      if (score > best_score)
      {
        best_score = score;
        best_frequency = fundamental;
      }
    }
  }

  for (i = 0U; i < peak_count; ++i)
  {
    float ratio;
    uint8_t order;

    if (best_frequency <= 0.0f)
    {
      break;
    }
    ratio = peaks[i].frequency_hz / best_frequency;
    order = (uint8_t)(ratio + 0.5f);
    if ((order >= 1U) &&
        (order <= SIGNAL_ANALYSIS_MAX_HARMONIC_ORDER) &&
        (fabsf(peaks[i].frequency_hz -
               ((float)order * best_frequency)) <= tolerance))
    {
      weighted_frequency_sum += peaks[i].amplitude_v * (float)order *
                                peaks[i].frequency_hz;
      weighted_order_square_sum += peaks[i].amplitude_v * (float)order *
                                   (float)order;
    }
  }
  if (weighted_order_square_sum > 0.0f)
  {
    best_frequency = weighted_frequency_sum / weighted_order_square_sum;
  }
  return best_frequency;
}

static void SignalAnalysis_MeasureComponent(const uint16_t *samples,
                                            float dc_voltage_v,
                                            float frequency_hz,
                                            uint8_t harmonic_order,
                                            SignalAnalysisComponent_t *component)
{
  float angular_step = SIGNAL_ANALYSIS_TWO_PI * frequency_hz /
                       (float)SIGNAL_ANALYSIS_SAMPLE_RATE_HZ;
  float step_cosine = arm_cos_f32(angular_step);
  float step_sine = arm_sin_f32(angular_step);
  float oscillator_cosine = 1.0f;
  float oscillator_sine = 0.0f;
  float real_sum = 0.0f;
  float imaginary_sum = 0.0f;
  float window_sum = 0.0f;
  uint32_t index;

  for (index = 0U; index < SIGNAL_ANALYSIS_FFT_SIZE; ++index)
  {
    float window = SignalAnalysis_Window(index);
    float sample = SignalAnalysis_AdcCodeToVoltage(samples[index]) - dc_voltage_v;
    float next_cosine;

    sample *= window;
    real_sum += sample * oscillator_cosine;
    imaginary_sum -= sample * oscillator_sine;
    window_sum += window;

    next_cosine = (oscillator_cosine * step_cosine) -
                  (oscillator_sine * step_sine);
    oscillator_sine = (oscillator_sine * step_cosine) +
                      (oscillator_cosine * step_sine);
    oscillator_cosine = next_cosine;

    if ((index & 0xFFU) == 0xFFU)
    {
      float norm = sqrtf((oscillator_cosine * oscillator_cosine) +
                         (oscillator_sine * oscillator_sine));
      if (norm > 0.0f)
      {
        oscillator_cosine /= norm;
        oscillator_sine /= norm;
      }
    }
  }

  component->frequency_hz = frequency_hz;
  component->amplitude_peak_v =
      2.0f * sqrtf((real_sum * real_sum) +
                   (imaginary_sum * imaginary_sum)) /
      window_sum;
  component->amplitude_rms_v = component->amplitude_peak_v *
                               SIGNAL_ANALYSIS_INV_SQRT_TWO;
  component->phase_rad = atan2f(imaginary_sum, real_sum);
  component->harmonic_order = harmonic_order;
}

static uint32_t SignalAnalysis_SelectHarmonicOrders(
    const SignalPeak_t *peaks,
    uint32_t peak_count,
    float fundamental_hz,
    uint8_t *selected_orders,
    float *selected_frequencies_hz)
{
  const float tolerance = 1.5f *
      ((float)SIGNAL_ANALYSIS_SAMPLE_RATE_HZ /
       (float)SIGNAL_ANALYSIS_FFT_SIZE);
  float selected_amplitudes[SIGNAL_ANALYSIS_MAX_COMPONENTS - 1U] = {0.0f};
  uint8_t order;

  for (order = 2U; order <= SIGNAL_ANALYSIS_MAX_HARMONIC_ORDER; ++order)
  {
    float expected_frequency = fundamental_hz * (float)order;
    float best_amplitude = 0.0f;
    float best_frequency = 0.0f;
    uint32_t peak_index;

    if (expected_frequency > (float)SIGNAL_ANALYSIS_MAX_FREQUENCY_HZ)
    {
      break;
    }
    for (peak_index = 0U; peak_index < peak_count; ++peak_index)
    {
      if ((fabsf(peaks[peak_index].frequency_hz - expected_frequency) <=
           tolerance) &&
          (peaks[peak_index].amplitude_v > best_amplitude))
      {
        best_amplitude = peaks[peak_index].amplitude_v;
        best_frequency = peaks[peak_index].frequency_hz;
      }
    }

    if (best_amplitude > selected_amplitudes[0])
    {
      selected_amplitudes[1] = selected_amplitudes[0];
      selected_orders[1] = selected_orders[0];
      selected_frequencies_hz[1] = selected_frequencies_hz[0];
      selected_amplitudes[0] = best_amplitude;
      selected_orders[0] = order;
      selected_frequencies_hz[0] = best_frequency;
    }
    else if (best_amplitude > selected_amplitudes[1])
    {
      selected_amplitudes[1] = best_amplitude;
      selected_orders[1] = order;
      selected_frequencies_hz[1] = best_frequency;
    }
  }

  if (selected_amplitudes[1] > 0.0f)
  {
    if (selected_orders[0] > selected_orders[1])
    {
      uint8_t temporary = selected_orders[0];
      float temporary_frequency = selected_frequencies_hz[0];
      selected_orders[0] = selected_orders[1];
      selected_orders[1] = temporary;
      selected_frequencies_hz[0] = selected_frequencies_hz[1];
      selected_frequencies_hz[1] = temporary_frequency;
    }
    return 2U;
  }
  return (selected_amplitudes[0] > 0.0f) ? 1U : 0U;
}

static void SignalAnalysis_Reconstruct(SignalAnalysisResult_t *result)
{
  float maximum = -FLT_MAX;
  float minimum = FLT_MAX;
  uint32_t point;

  for (point = 0U; point < SIGNAL_ANALYSIS_WAVEFORM_POINTS; ++point)
  {
    float time = (float)point /
                 (result->fundamental_hz *
                  (float)SIGNAL_ANALYSIS_WAVEFORM_POINTS);
    float value = 0.0f;
    uint32_t component;

    for (component = 0U; component < result->component_count; ++component)
    {
      const SignalAnalysisComponent_t *current = &result->components[component];
      value += current->amplitude_peak_v * arm_cos_f32(
          (SIGNAL_ANALYSIS_TWO_PI * current->frequency_hz * time) +
          current->phase_rad);
    }
    result->waveform_v[point] = value;
    if (value > maximum)
    {
      maximum = value;
    }
    if (value < minimum)
    {
      minimum = value;
    }
  }
  result->peak_to_peak_v = maximum - minimum;
}

void SignalAnalysis_Init(void)
{
  if (arm_rfft_fast_init_f32(&fft_instance, SIGNAL_ANALYSIS_FFT_SIZE) !=
      ARM_MATH_SUCCESS)
  {
    Error_Handler();
  }
  signal_analysis_initialized = 1U;
}

uint8_t SignalAnalysis_Process(const uint16_t *samples,
                               SignalAnalysisResult_t *result)
{
  SignalPeak_t peaks[SIGNAL_ANALYSIS_MAX_CANDIDATES];
  uint8_t harmonic_orders[SIGNAL_ANALYSIS_MAX_COMPONENTS - 1U] = {0U};
  float harmonic_frequencies_hz[SIGNAL_ANALYSIS_MAX_COMPONENTS - 1U] = {0.0f};
  float dc_sum = 0.0f;
  float square_sum = 0.0f;
  float sampled_minimum = FLT_MAX;
  float sampled_maximum = -FLT_MAX;
  uint32_t peak_count;
  uint32_t harmonic_count;
  uint32_t index;

  if ((samples == NULL) || (result == NULL))
  {
    return 0U;
  }
  if (signal_analysis_initialized == 0U)
  {
    SignalAnalysis_Init();
  }

  for (index = 0U; index < SIGNAL_ANALYSIS_FFT_SIZE; ++index)
  {
    dc_sum += SignalAnalysis_AdcCodeToVoltage(samples[index]);
  }
  result->adc_dc_voltage_v = dc_sum / (float)SIGNAL_ANALYSIS_FFT_SIZE;

  for (index = 0U; index < SIGNAL_ANALYSIS_FFT_SIZE; ++index)
  {
    float sample = SignalAnalysis_AdcCodeToVoltage(samples[index]) -
                   result->adc_dc_voltage_v;
    fft_input[index] = sample * SignalAnalysis_Window(index);
    square_sum += sample * sample;
    if (sample < sampled_minimum)
    {
      sampled_minimum = sample;
    }
    if (sample > sampled_maximum)
    {
      sampled_maximum = sample;
    }
  }
  result->sampled_peak_to_peak_v = sampled_maximum - sampled_minimum;
  arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0U);

  peak_count = SignalAnalysis_FindPeaks(peaks);
  result->component_count = 0U;
  result->fundamental_hz = SignalAnalysis_SelectFundamental(peaks, peak_count);
  result->true_rms_v = sqrtf(square_sum /
                             (float)SIGNAL_ANALYSIS_FFT_SIZE);
  result->peak_to_peak_v = result->sampled_peak_to_peak_v;

  if (result->fundamental_hz <= 0.0f)
  {
    return 1U;
  }

  SignalAnalysis_MeasureComponent(samples, result->adc_dc_voltage_v,
                                  result->fundamental_hz, 1U,
                                  &result->components[0]);
  result->component_count = 1U;

  harmonic_count = SignalAnalysis_SelectHarmonicOrders(
      peaks, peak_count, result->fundamental_hz, harmonic_orders,
      harmonic_frequencies_hz);
  for (index = 0U; index < harmonic_count; ++index)
  {
    uint8_t order = harmonic_orders[index];
    SignalAnalysis_MeasureComponent(
        samples, result->adc_dc_voltage_v, harmonic_frequencies_hz[index],
        order, &result->components[result->component_count]);
    result->component_count++;
  }
  SignalAnalysis_Reconstruct(result);
  return 1U;
}

float SignalAnalysis_GetSpectrumPeakV(uint16_t bin)
{
  if (bin > (SIGNAL_ANALYSIS_FFT_SIZE / 2U))
  {
    return 0.0f;
  }
  return SignalAnalysis_Magnitude(bin) /
         ((float)SIGNAL_ANALYSIS_FFT_SIZE * 0.25f);
}
