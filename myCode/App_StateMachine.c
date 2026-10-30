#include "App_StateMachine.h"

#include <math.h>

#include "ADC_DMA.h"
#include "Config.h"
#include "Signal_Analysis.h"
#include "usart.h"

#define APP_BUTTON_DEBOUNCE_MS       30UL
#define APP_UART_BUFFER_SIZE          8192U
#define APP_HMI_WAVEFORM_WIDTH         603U   /* 2026G: s0 波形控件宽度 */
#define APP_HMI_WAVEFORM_HEIGHT        181U   /* 2026G: s0 波形控件高度 */
#define APP_HMI_VERTICAL_MARGIN          8U
#define APP_HMI_SPECTRUM_WIDTH         603U   /* 2026G: s1 频谱控件宽度 */
#define APP_HMI_SPECTRUM_HEIGHT        181U   /* 2026G: s1 频谱控件高度 */
#define APP_HMI_SPECTRUM_MARGIN          6U
#define APP_HMI_SPECTRUM_X_MARGIN        4U
#define APP_HMI_SPECTRUM_MIN_HZ       10000.0f
#define APP_HMI_READY_FALLBACK_MS      25UL
#define APP_HMI_RESPONSE_TIMEOUT_MS    150UL
#define APP_HMI_BUTTON_ONE_CYCLE       0x34U
#define APP_HMI_BUTTON_THREE_CYCLES    0x35U
#define APP_HMI_READY                  0xFEU
#define APP_HMI_FINISHED               0xFDU
#define APP_HMI_RX_BUFFER_SIZE           32U
#define APP_HMI_RX_BUFFER_MASK           (APP_HMI_RX_BUFFER_SIZE - 1U)
#define APP_HMI_TEXT_BUFFER_SIZE         128U
#define APP_HMI_COMPONENT_BUFFER_SIZE    192U
#define APP_START_MODE                APP_MODE_Q1_WAVEFORM
#define APP_MEASURE_FRAME_COUNT       8U
#define APP_COMPONENT_FRAME_COUNT     8U
#define APP_MEASURE_MIN_FREQUENCY_HZ  9500.0f
#define APP_MEASURE_MAX_FREQUENCY_HZ  1000500.0f
#define APP_HMI_UPP_COMPENSATION_HZ   100000.0f
#define APP_HMI_UPP_COMPENSATION_MV   2U
#define APP_TWO_PI                    6.28318530717958647692f
#define APP_RISING_ZERO_PHASE_RAD     (-1.57079632679489661923f)
#define APP_TRIGGER_HYSTERESIS_RATIO  0.05f
#define APP_TRIGGER_MIN_HYSTERESIS_V  0.002f
#define APP_WAVEFORM_AVERAGE_CYCLES     16U
#define APP_VOFA_WAVEFORM_ENABLE         0U
#define APP_UART_HEARTBEAT_MS         500UL

typedef enum
{
  APP_HMI_IDLE = 0,
  APP_HMI_WAIT_READY,
  APP_HMI_WAIT_FINISHED
} AppHmiState_t;

typedef enum
{
  APP_HMI_TRANSFER_NONE = 0,
  APP_HMI_TRANSFER_WAVEFORM,
  APP_HMI_TRANSFER_SPECTRUM
} AppHmiTransfer_t;

typedef struct
{
  AppMode_t mode;
  AppState_t state;
  uint8_t display_cycles;
  uint8_t capture_started;
  uint8_t button_raw;
  uint8_t button_stable;
  uint32_t button_change_tick;
  uint32_t uart_heartbeat_tick;
  uint8_t measurement_frame_count;
  float measurement_upp_sum_v;
  float measurement_rms_sum_v;
  float measurement_f1_sum_hz;
  uint8_t component_frame_count;
  uint8_t component_count;
  uint8_t component_orders[SIGNAL_ANALYSIS_MAX_COMPONENTS];
  float component_frequency_sum_hz[SIGNAL_ANALYSIS_MAX_COMPONENTS];
  float component_amplitude_sum_v[SIGNAL_ANALYSIS_MAX_COMPONENTS];
  AppHmiState_t hmi_state;
  uint8_t hmi_marker_count;
  uint8_t hmi_waveform_request;
  uint8_t hmi_waveform_pending;
  uint8_t hmi_spectrum_request;
  uint8_t hmi_spectrum_pending;
  uint8_t hmi_measurement_request;
  uint8_t hmi_measurement_pending;
  uint8_t hmi_component_request;
  uint8_t hmi_component_pending;
  uint8_t vofa_waveform_pending;
  AppHmiTransfer_t hmi_transfer;
  uint32_t hmi_deadline_tick;
} AppContext_t;

static AppContext_t app;
static SignalAnalysisResult_t analysis_result;
static uint16_t *adc_frame;
static char uart_buffer[APP_UART_BUFFER_SIZE];
static float hmi_waveform_voltage[APP_HMI_WAVEFORM_WIDTH];
static float hmi_waveform_smoothed[APP_HMI_WAVEFORM_WIDTH];
static uint8_t hmi_waveform_data[APP_HMI_WAVEFORM_WIDTH];
static uint8_t hmi_spectrum_data[APP_HMI_SPECTRUM_WIDTH];
static volatile uint8_t hmi_rx_buffer[APP_HMI_RX_BUFFER_SIZE];
static volatile uint8_t hmi_rx_head;
static volatile uint8_t hmi_rx_tail;
static uint8_t hmi_text_buffer[APP_HMI_TEXT_BUFFER_SIZE];
static uint16_t hmi_text_length;
static uint8_t hmi_component_buffer[APP_HMI_COMPONENT_BUFFER_SIZE];
static uint16_t hmi_component_length;

static void App_UpdateButton(void);
static void App_HmiRun(void);
static void App_HmiHandleByte(uint8_t value);
static void App_HmiHandleMarker(uint8_t value);
static void App_HmiStartTransfer(AppHmiTransfer_t transfer);
static void App_HmiSendPayload(void);
static void App_CollectTriggeredMeasurement(void);
static void App_HmiQueueMeasurement(float peak_to_peak_v,
                                    float true_rms_v,
                                    float fundamental_hz);
static void App_CollectTriggeredComponents(void);
static void App_HmiQueueComponents(void);
static uint8_t App_IsWaveformMode(AppMode_t mode);
static uint8_t App_IsSpectrumMode(AppMode_t mode);
static uint8_t App_IsComponentMode(AppMode_t mode);
static void App_SendWaveform(void);
static void App_SendVofaWaveform(void);
static void App_BuildTriggeredSpectrum(void);
static void App_SendCaptureHeartbeat(void);
static uint8_t App_FindWaveformStart(float *start_sample,
                                     float *period_samples);
static float App_InterpolateRawVoltage(float sample_index);
static void App_SmoothWaveform(void);
static void App_ResetMeasurementAverage(void);
static void App_ResetComponentAverage(void);
static void App_SendMeasurements(void);
static void App_SendSpectrum(void);
static void App_SendComponents(void);
static uint16_t App_AppendText(char *buffer, uint16_t offset, const char *text);
static uint16_t App_AppendUnsigned(char *buffer, uint16_t offset, uint32_t value);
static uint16_t App_AppendFixed3(char *buffer, uint16_t offset, float value);

void App_Init(void)
{
  ConfigAndData_Init();
  OSC_Config.OSC_Enable = 1U;
  OSC_Config.OSC_DAC_Enable = 0U;
  OSC_Config.OSC_FFT_Enable = 1U;
  OSC_Config.OSC_Auto_Enable = 0U;
  /* This migration uses a fixed analysis sampler, not the legacy rate table. */
  OSC_Config.OSC_Fs = ADC_SAMPLE_RATE_HZ;

  SignalAnalysis_Init();
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  app.mode = APP_START_MODE;
  app.state = APP_STATE_CAPTURE_START;
  app.display_cycles = 1U;
  /* Queue an initial frame so the screen is populated after power-up even
     before the first b3/b6 event arrives.  Button presses still requeue the
     same set of views through App_SetDisplayCycles(). */
  app.hmi_waveform_request = 1U;
  app.hmi_spectrum_request = 1U;
  app.hmi_measurement_request = 1U;
  app.hmi_component_request = 1U;
  app.button_change_tick = HAL_GetTick();
  app.uart_heartbeat_tick = HAL_GetTick();
  App_ResetMeasurementAverage();
  App_ResetComponentAverage();
}

void App_Run(void)
{
  App_UpdateButton();
  App_HmiRun();

  switch (app.state)
  {
    case APP_STATE_CAPTURE_START:
      if (app.capture_started == 0U)
      {
        ADC_Change_Fs();
        app.capture_started = 1U;
      }
      ADC_DMA_Start(ADC_SAMPLE_RATE_HZ);
      app.state = APP_STATE_CAPTURE_WAIT;
      break;

    case APP_STATE_CAPTURE_WAIT:
      adc_frame = ADC_ReadData_u16();
      if (adc_frame != NULL)
      {
        app.state = APP_STATE_PROCESS;
      }
      else if ((HAL_GetTick() - app.uart_heartbeat_tick) >=
               APP_UART_HEARTBEAT_MS)
      {
        App_SendCaptureHeartbeat();
      }
      break;

    case APP_STATE_PROCESS:
      if (SignalAnalysis_Process(adc_frame, &analysis_result) != 0U)
      {
        app.state = APP_STATE_OUTPUT;
      }
      else
      {
        app.state = APP_STATE_CAPTURE_START;
      }
      break;

    case APP_STATE_OUTPUT:
      App_CollectTriggeredMeasurement();
      App_CollectTriggeredComponents();
      App_BuildTriggeredSpectrum();
      if (App_IsWaveformMode(app.mode) != 0U)
      {
        App_SendWaveform();
      }
      else if (App_IsSpectrumMode(app.mode) != 0U)
      {
        App_SendSpectrum();
      }
      else if (App_IsComponentMode(app.mode) != 0U)
      {
        App_SendComponents();
      }
      else
      {
        App_SendMeasurements();
      }
      app.state = APP_STATE_CAPTURE_START;
      break;

    case APP_STATE_RAW_WAIT:
    case APP_STATE_FAULT:
    default:
      app.state = APP_STATE_CAPTURE_START;
      break;
  }
}

void App_SetMode(AppMode_t mode)
{
  if (mode < APP_MODE_COUNT)
  {
    if (app.mode != mode)
    {
      App_ResetMeasurementAverage();
      App_ResetComponentAverage();
    }
    app.mode = mode;
  }
}

AppMode_t App_GetMode(void)
{
  return app.mode;
}

void App_SetDisplayCycles(uint8_t cycles)
{
  if ((cycles == 1U) || (cycles == 3U))
  {
    app.display_cycles = cycles;
    app.hmi_waveform_request = 1U;
    app.hmi_waveform_pending = 0U;
    app.hmi_spectrum_request = 1U;
    app.hmi_spectrum_pending = 0U;
    app.hmi_measurement_request = 1U;
    app.hmi_measurement_pending = 0U;
    app.hmi_component_request = 1U;
    app.hmi_component_pending = 0U;
    App_ResetMeasurementAverage();
    App_ResetComponentAverage();
  }
}

uint8_t App_GetDisplayCycles(void)
{
  return app.display_cycles;
}

static void App_UpdateButton(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t pressed = (HAL_GPIO_ReadPin(MODE_BUTTON_GPIO_Port, MODE_BUTTON_Pin) ==
                     GPIO_PIN_RESET) ? 1U : 0U;

  if (pressed != app.button_raw)
  {
    app.button_raw = pressed;
    app.button_change_tick = now;
  }

  if (((now - app.button_change_tick) >= APP_BUTTON_DEBOUNCE_MS) &&
      (app.button_stable != app.button_raw))
  {
    app.button_stable = app.button_raw;
    if (app.button_stable != 0U)
    {
      App_SetDisplayCycles((app.display_cycles == 3U) ? 1U : 3U);
    }
  }
}

static void App_HmiRun(void)
{
  uint8_t value;
  uint8_t receive_count = 0U;
  uint32_t now = HAL_GetTick();

  while ((receive_count < 16U) && (hmi_rx_tail != hmi_rx_head))
  {
    value = hmi_rx_buffer[hmi_rx_tail];
    hmi_rx_tail = (uint8_t)((hmi_rx_tail + 1U) & APP_HMI_RX_BUFFER_MASK);
    App_HmiHandleByte(value);
    receive_count++;
  }

  if ((app.hmi_state != APP_HMI_IDLE) &&
      ((int32_t)(now - app.hmi_deadline_tick) >= 0))
  {
    if (app.hmi_state == APP_HMI_WAIT_READY)
    {
      /* Some screen firmware does not return FE; send after the normal
         command settling time instead of dropping the pending frame. */
      App_HmiSendPayload();
    }
    else
    {
      app.hmi_state = APP_HMI_IDLE;
      app.hmi_marker_count = 0U;
      app.hmi_transfer = APP_HMI_TRANSFER_NONE;
    }
  }

  if ((app.hmi_state == APP_HMI_IDLE) &&
      (app.hmi_waveform_pending != 0U))
  {
    App_HmiStartTransfer(APP_HMI_TRANSFER_WAVEFORM);
  }
  else if ((app.hmi_state == APP_HMI_IDLE) &&
           (app.hmi_spectrum_pending != 0U))
  {
    App_HmiStartTransfer(APP_HMI_TRANSFER_SPECTRUM);
  }
  else if ((app.hmi_state == APP_HMI_IDLE) &&
           (app.hmi_measurement_pending != 0U))
  {
    app.hmi_measurement_pending = 0U;
    (void)HAL_UART_Transmit(&huart3, hmi_text_buffer,
                            hmi_text_length, 30U);
  }
  else if ((app.hmi_state == APP_HMI_IDLE) &&
           (app.hmi_component_pending != 0U))
  {
    app.hmi_component_pending = 0U;
    (void)HAL_UART_Transmit(&huart3, hmi_component_buffer,
                            hmi_component_length, 40U);
  }
  else if ((app.hmi_state == APP_HMI_IDLE) &&
           (APP_VOFA_WAVEFORM_ENABLE != 0U) &&
           (app.vofa_waveform_pending != 0U) &&
           (app.hmi_waveform_pending == 0U) &&
           (app.hmi_spectrum_pending == 0U) &&
           (app.hmi_measurement_pending == 0U) &&
           (app.hmi_component_pending == 0U))
  {
    app.vofa_waveform_pending = 0U;
    App_SendVofaWaveform();
  }
}

void App_HmiUartIrqHandler(void)
{
  uint32_t status = huart3.Instance->SR;

  if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
                 USART_SR_FE | USART_SR_PE)) != 0U)
  {
    uint8_t value = (uint8_t)huart3.Instance->DR;

    if ((status & USART_SR_RXNE) != 0U)
    {
      uint8_t next_head =
          (uint8_t)((hmi_rx_head + 1U) & APP_HMI_RX_BUFFER_MASK);
      if (next_head != hmi_rx_tail)
      {
        hmi_rx_buffer[hmi_rx_head] = value;
        hmi_rx_head = next_head;
      }
    }
  }
}

static void App_HmiHandleByte(uint8_t value)
{
  if (value == APP_HMI_BUTTON_ONE_CYCLE)
  {
    static const uint8_t b3_debug_message[] = "print35\r\n";

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)b3_debug_message,
                            sizeof(b3_debug_message) - 1U, 20U);
    App_SetMode(APP_MODE_Q1_WAVEFORM);
    App_SetDisplayCycles(1U);
    return;
  }
  if (value == APP_HMI_BUTTON_THREE_CYCLES)
  {
    App_SetMode(APP_MODE_Q1_WAVEFORM);
    App_SetDisplayCycles(3U);
    return;
  }

  App_HmiHandleMarker(value);
}

static void App_HmiHandleMarker(uint8_t value)
{
  uint8_t expected;

  if (app.hmi_state == APP_HMI_WAIT_READY)
  {
    expected = APP_HMI_READY;
  }
  else if (app.hmi_state == APP_HMI_WAIT_FINISHED)
  {
    expected = APP_HMI_FINISHED;
  }
  else
  {
    app.hmi_marker_count = 0U;
    return;
  }

  if (app.hmi_marker_count == 0U)
  {
    if (value == expected)
    {
      app.hmi_marker_count = 1U;
    }
    return;
  }

  if (value == 0xFFU)
  {
    app.hmi_marker_count++;
    if (app.hmi_marker_count >= 4U)
    {
      app.hmi_marker_count = 0U;
      if (app.hmi_state == APP_HMI_WAIT_READY)
      {
        App_HmiSendPayload();
      }
      else
      {
        app.hmi_state = APP_HMI_IDLE;
        app.hmi_transfer = APP_HMI_TRANSFER_NONE;
      }
    }
  }
  else
  {
    app.hmi_marker_count = (value == expected) ? 1U : 0U;
  }
}

static void App_HmiStartTransfer(AppHmiTransfer_t transfer)
{
  static const uint8_t waveform_clear_command[] = {
      'c', 'l', 'e', ' ', 's', '0', '.', 'i', 'd', ',', '0',
      0xFFU, 0xFFU, 0xFFU};
  static const uint8_t waveform_addt_command[] = {
      'a', 'd', 'd', 't', ' ', 's', '0', '.', 'i', 'd', ',', '0', ',',
      '6', '0', '3', 0xFFU, 0xFFU, 0xFFU};
  static const uint8_t spectrum_clear_command[] = {
      'c', 'l', 'e', ' ', 's', '1', '.', 'i', 'd', ',', '0',
      0xFFU, 0xFFU, 0xFFU};
  static const uint8_t spectrum_addt_command[] = {
      'a', 'd', 'd', 't', ' ', 's', '1', '.', 'i', 'd', ',', '0', ',',
      '6', '0', '3', 0xFFU, 0xFFU, 0xFFU};

  app.hmi_marker_count = 0U;
  app.hmi_transfer = transfer;
  if (transfer == APP_HMI_TRANSFER_WAVEFORM)
  {
    app.hmi_waveform_pending = 0U;
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)waveform_clear_command,
                            sizeof(waveform_clear_command), 20U);
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)waveform_addt_command,
                            sizeof(waveform_addt_command), 20U);
  }
  else
  {
    app.hmi_spectrum_pending = 0U;
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)spectrum_clear_command,
                            sizeof(spectrum_clear_command), 20U);
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)spectrum_addt_command,
                            sizeof(spectrum_addt_command), 20U);
  }
  app.hmi_state = APP_HMI_WAIT_READY;
  app.hmi_deadline_tick = HAL_GetTick() + APP_HMI_READY_FALLBACK_MS;
}

static void App_HmiSendPayload(void)
{
  static const uint8_t hmi_transparent_finish[] = {
      0x01U, 0xFFU, 0xFFU, 0xFFU};

  if (app.hmi_transfer == APP_HMI_TRANSFER_WAVEFORM)
  {
    app.hmi_waveform_pending = 0U;
    (void)HAL_UART_Transmit(&huart3, hmi_waveform_data,
                            APP_HMI_WAVEFORM_WIDTH, 30U);
  }
  else if (app.hmi_transfer == APP_HMI_TRANSFER_SPECTRUM)
  {
    app.hmi_spectrum_pending = 0U;
    (void)HAL_UART_Transmit(&huart3, hmi_spectrum_data,
                            APP_HMI_SPECTRUM_WIDTH, 30U);
  }
  else
  {
    app.hmi_state = APP_HMI_IDLE;
    app.hmi_transfer = APP_HMI_TRANSFER_NONE;
    return;
  }

  /*
   * TJC transparent-data mode needs an explicit 01 FF FF FF terminator
   * after the raw samples.  Without it, the screen keeps consuming the
   * following commands as waveform bytes and neither s0 nor s1 updates.
   */
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)hmi_transparent_finish,
                          sizeof(hmi_transparent_finish), 20U);
  HAL_Delay(1U);

  /* The FD response is optional and is not used to gate the next transfer. */
  app.hmi_state = APP_HMI_IDLE;
  app.hmi_marker_count = 0U;
  app.hmi_transfer = APP_HMI_TRANSFER_NONE;
  app.hmi_deadline_tick = 0U;
}

static void App_CollectTriggeredMeasurement(void)
{
  float frame_count;

  if (app.hmi_measurement_request == 0U)
  {
    return;
  }

  if ((analysis_result.component_count == 0U) ||
      (analysis_result.fundamental_hz < APP_MEASURE_MIN_FREQUENCY_HZ) ||
      (analysis_result.fundamental_hz > APP_MEASURE_MAX_FREQUENCY_HZ))
  {
    App_ResetMeasurementAverage();
    return;
  }

  app.measurement_upp_sum_v += analysis_result.peak_to_peak_v;
  app.measurement_rms_sum_v += analysis_result.true_rms_v;
  app.measurement_f1_sum_hz += analysis_result.fundamental_hz;
  app.measurement_frame_count++;

  if (app.measurement_frame_count < APP_MEASURE_FRAME_COUNT)
  {
    return;
  }

  frame_count = (float)app.measurement_frame_count;
  App_HmiQueueMeasurement(app.measurement_upp_sum_v / frame_count,
                          app.measurement_rms_sum_v / frame_count,
                          app.measurement_f1_sum_hz / frame_count);
  app.hmi_measurement_request = 0U;
  App_ResetMeasurementAverage();
}

static void App_HmiQueueMeasurement(float peak_to_peak_v,
                                    float true_rms_v,
                                    float fundamental_hz)
{
  uint16_t length = 0U;
  uint32_t peak_to_peak_mv =
      (uint32_t)(peak_to_peak_v * 1000.0f + 0.5f);
  uint32_t true_rms_mv =
      (uint32_t)(true_rms_v * 1000.0f + 0.5f);
  uint32_t fundamental_hz_integer =
      (uint32_t)(fundamental_hz + 0.5f);

  /* The high-frequency display calibration applies only to the HMI Upp value. */
  if (fundamental_hz > APP_HMI_UPP_COMPENSATION_HZ)
  {
    peak_to_peak_mv += APP_HMI_UPP_COMPENSATION_MV;
  }

  /* 2026G 布局: x1=Ua峰峰值(mVpp) x2=Ua有效值(mVpp) x3=Ua基频值(kHz) */
  length = App_AppendText((char *)hmi_text_buffer, length, "x1.val=");
  length = App_AppendUnsigned((char *)hmi_text_buffer, length,
                              peak_to_peak_mv);
  hmi_text_buffer[length++] = 0xFFU;
  hmi_text_buffer[length++] = 0xFFU;
  hmi_text_buffer[length++] = 0xFFU;

  length = App_AppendText((char *)hmi_text_buffer, length, "x2.val=");
  length = App_AppendUnsigned((char *)hmi_text_buffer, length,
                              true_rms_mv);
  hmi_text_buffer[length++] = 0xFFU;
  hmi_text_buffer[length++] = 0xFFU;
  hmi_text_buffer[length++] = 0xFFU;

  length = App_AppendText((char *)hmi_text_buffer, length, "x3.val=");
  length = App_AppendUnsigned((char *)hmi_text_buffer, length,
                              fundamental_hz_integer / 1000UL);
  hmi_text_buffer[length++] = 0xFFU;
  hmi_text_buffer[length++] = 0xFFU;
  hmi_text_buffer[length++] = 0xFFU;

  hmi_text_length = length;
  app.hmi_measurement_pending = 1U;
}

static void App_CollectTriggeredComponents(void)
{
  uint32_t index;
  uint8_t frame_matches = 1U;

  if (app.hmi_component_request == 0U)
  {
    return;
  }
  if ((analysis_result.component_count == 0U) ||
      (analysis_result.fundamental_hz < APP_MEASURE_MIN_FREQUENCY_HZ) ||
      (analysis_result.fundamental_hz > APP_MEASURE_MAX_FREQUENCY_HZ))
  {
    App_ResetComponentAverage();
    return;
  }

  if (app.component_frame_count != 0U)
  {
    if (app.component_count != analysis_result.component_count)
    {
      frame_matches = 0U;
    }
    else
    {
      for (index = 0U; index < analysis_result.component_count; ++index)
      {
        if (app.component_orders[index] !=
            analysis_result.components[index].harmonic_order)
        {
          frame_matches = 0U;
          break;
        }
      }
    }
  }

  if (frame_matches == 0U)
  {
    App_ResetComponentAverage();
  }
  if (app.component_frame_count == 0U)
  {
    app.component_count = analysis_result.component_count;
    for (index = 0U; index < app.component_count; ++index)
    {
      app.component_orders[index] =
          analysis_result.components[index].harmonic_order;
    }
  }

  for (index = 0U; index < app.component_count; ++index)
  {
    app.component_frequency_sum_hz[index] +=
        analysis_result.components[index].frequency_hz;
    app.component_amplitude_sum_v[index] +=
        analysis_result.components[index].amplitude_peak_v;
  }
  app.component_frame_count++;

  if (app.component_frame_count < APP_COMPONENT_FRAME_COUNT)
  {
    return;
  }

  App_HmiQueueComponents();
  app.hmi_component_request = 0U;
  App_ResetComponentAverage();
}

static void App_HmiQueueComponents(void)
{
  float frame_count = (float)app.component_frame_count;
  float fundamental_amplitude_v = 0.0f;
  float harmonic1_amplitude_v = 0.0f;
  float harmonic1_frequency_hz = 0.0f;
  float harmonic2_amplitude_v = 0.0f;
  float harmonic2_frequency_hz = 0.0f;
  uint32_t fundamental_mv;
  uint32_t harmonic1_mv;
  uint32_t harmonic1_hz;
  uint32_t harmonic2_mv;
  uint32_t harmonic2_hz;
  uint16_t length = 0U;

  if (app.component_count > 0U)
  {
    fundamental_amplitude_v =
        app.component_amplitude_sum_v[0] / frame_count;
  }
  if (app.component_count > 1U)
  {
    harmonic1_amplitude_v =
        app.component_amplitude_sum_v[1] / frame_count;
    harmonic1_frequency_hz =
        app.component_frequency_sum_hz[1] / frame_count;
  }
  if (app.component_count > 2U)
  {
    harmonic2_amplitude_v =
        app.component_amplitude_sum_v[2] / frame_count;
    harmonic2_frequency_hz =
        app.component_frequency_sum_hz[2] / frame_count;
  }

  fundamental_mv =
      (uint32_t)(fundamental_amplitude_v * 1000.0f + 0.5f);
  harmonic1_mv =
      (uint32_t)(harmonic1_amplitude_v * 1000.0f + 0.5f);
  harmonic1_hz =
      (uint32_t)(harmonic1_frequency_hz + 0.5f);
  harmonic2_mv =
      (uint32_t)(harmonic2_amplitude_v * 1000.0f + 0.5f);
  harmonic2_hz =
      (uint32_t)(harmonic2_frequency_hz + 0.5f);

#define APP_APPEND_TEXT_INTEGER(NAME, VALUE)                                \
  do                                                                        \
  {                                                                         \
    length = App_AppendText((char *)hmi_component_buffer, length,            \
                            NAME);                                          \
    length = App_AppendUnsigned((char *)hmi_component_buffer, length,        \
                                (VALUE));                                   \
    hmi_component_buffer[length++] = 0xFFU;                                 \
    hmi_component_buffer[length++] = 0xFFU;                                 \
    hmi_component_buffer[length++] = 0xFFU;                                 \
  } while (0)

  /* 2026G 布局: x4=基波幅值 x0=谐波1幅值 x5=谐波1频率(kHz)
     x6=谐波2幅值 x7=谐波2频率(kHz) */
  APP_APPEND_TEXT_INTEGER("x4.val=", fundamental_mv);
  APP_APPEND_TEXT_INTEGER("x0.val=", harmonic1_mv);
  APP_APPEND_TEXT_INTEGER("x5.val=", harmonic1_hz / 1000UL);
  APP_APPEND_TEXT_INTEGER("x6.val=", harmonic2_mv);
  APP_APPEND_TEXT_INTEGER("x7.val=", harmonic2_hz / 1000UL);

#undef APP_APPEND_TEXT_INTEGER

  hmi_component_length = length;
  app.hmi_component_pending = 1U;
}

static uint8_t App_IsWaveformMode(AppMode_t mode)
{
  return ((mode == APP_MODE_Q1_WAVEFORM) || (mode == APP_MODE_Q2_WAVEFORM) ||
          (mode == APP_MODE_Q3_WAVEFORM)) ? 1U : 0U;
}

static uint8_t App_IsSpectrumMode(AppMode_t mode)
{
  return ((mode == APP_MODE_Q1_SPECTRUM) || (mode == APP_MODE_Q2_SPECTRUM) ||
          (mode == APP_MODE_Q3_SPECTRUM)) ? 1U : 0U;
}

static uint8_t App_IsComponentMode(AppMode_t mode)
{
  return ((mode == APP_MODE_Q1_COMPONENTS) ||
          (mode == APP_MODE_Q2_COMPONENTS) ||
          (mode == APP_MODE_Q3_COMPONENTS)) ? 1U : 0U;
}

static void App_SendWaveform(void)
{
  float minimum = 1.0e30f;
  float maximum = -1.0e30f;
  float maximum_absolute;
  float vertical_center = ((float)APP_HMI_WAVEFORM_HEIGHT - 1.0f) * 0.5f;
  float vertical_span = vertical_center - (float)APP_HMI_VERTICAL_MARGIN;
  float start_sample;
  float period_samples;
  uint32_t average_cycle_count;
  uint16_t point;

  if (app.hmi_waveform_request == 0U)
  {
    return;
  }

  if (App_FindWaveformStart(&start_sample, &period_samples) == 0U)
  {
    return;
  }

  average_cycle_count = (uint32_t)(
      ((float)(ADC_NUMBER - 1U) - start_sample) / period_samples);
  if (average_cycle_count > APP_WAVEFORM_AVERAGE_CYCLES)
  {
    average_cycle_count = APP_WAVEFORM_AVERAGE_CYCLES;
  }
  if (average_cycle_count == 0U)
  {
    return;
  }

  for (point = 0U; point < APP_HMI_WAVEFORM_WIDTH; ++point)
  {
    float cycle_position =
        ((float)app.display_cycles * (float)point) /
        (float)(APP_HMI_WAVEFORM_WIDTH - 1U);
    float phase_position = cycle_position - floorf(cycle_position);
    float voltage = 0.0f;
    float cycle_minimum = 1.0e30f;
    float cycle_maximum = -1.0e30f;
    uint32_t cycle;

    for (cycle = 0U; cycle < average_cycle_count; ++cycle)
    {
      float sample_index = start_sample +
                           (((float)cycle + phase_position) * period_samples);
      float sample_voltage = App_InterpolateRawVoltage(sample_index);

      voltage += sample_voltage;
      if (sample_voltage < cycle_minimum)
      {
        cycle_minimum = sample_voltage;
      }
      if (sample_voltage > cycle_maximum)
      {
        cycle_maximum = sample_voltage;
      }
    }
    if (average_cycle_count >= 5U)
    {
      voltage = (voltage - cycle_minimum - cycle_maximum) /
                (float)(average_cycle_count - 2U);
    }
    else
    {
      voltage /= (float)average_cycle_count;
    }

    hmi_waveform_voltage[point] = voltage;
  }

  App_SmoothWaveform();
  for (point = 0U; point < APP_HMI_WAVEFORM_WIDTH; ++point)
  {
    if (hmi_waveform_smoothed[point] < minimum)
    {
      minimum = hmi_waveform_smoothed[point];
    }
    if (hmi_waveform_smoothed[point] > maximum)
    {
      maximum = hmi_waveform_smoothed[point];
    }
  }

  maximum_absolute = fmaxf(fabsf(minimum), fabsf(maximum));
  if (maximum_absolute < 0.001f)
  {
    return;
  }

  for (point = 0U; point < APP_HMI_WAVEFORM_WIDTH; ++point)
  {
    float scaled = vertical_center +
                   ((hmi_waveform_smoothed[point] / maximum_absolute) *
                    vertical_span);
    int32_t pixel = (int32_t)(scaled + 0.5f);

    if (pixel < 0)
    {
      pixel = 0;
    }
    else if (pixel >= (int32_t)APP_HMI_WAVEFORM_HEIGHT)
    {
      pixel = (int32_t)APP_HMI_WAVEFORM_HEIGHT - 1;
    }
    hmi_waveform_data[point] = (uint8_t)pixel;
  }
#if (APP_VOFA_WAVEFORM_ENABLE != 0U)
  app.vofa_waveform_pending = 1U;
#else
  app.vofa_waveform_pending = 0U;
#endif
  app.hmi_waveform_request = 0U;
  app.hmi_waveform_pending = 1U;
}

/* Send one captured waveform as one FireWater channel in mV for VOFA+. */
static void App_SendVofaWaveform(void)
{
  uint16_t length = 0U;
  uint16_t point;

  for (point = 0U; point < APP_HMI_WAVEFORM_WIDTH; ++point)
  {
    length = App_AppendFixed3(
        uart_buffer, length, hmi_waveform_smoothed[point] * 1000.0f);
    uart_buffer[length++] = '\r';
    uart_buffer[length++] = '\n';
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, length, 200U);
}

static void App_BuildTriggeredSpectrum(void)
{
  float maximum_amplitude = 0.0f;
  float vertical_span = (float)(APP_HMI_SPECTRUM_HEIGHT - 1U -
                                APP_HMI_SPECTRUM_MARGIN);
  uint32_t component;
  uint32_t point;

  if (app.hmi_spectrum_request == 0U)
  {
    return;
  }
  if ((analysis_result.component_count == 0U) ||
      (analysis_result.fundamental_hz < APP_MEASURE_MIN_FREQUENCY_HZ) ||
      (analysis_result.fundamental_hz > APP_MEASURE_MAX_FREQUENCY_HZ))
  {
    return;
  }

  for (component = 0U;
       component < analysis_result.component_count;
       ++component)
  {
    float amplitude = analysis_result.components[component].amplitude_peak_v;
    if (amplitude > maximum_amplitude)
    {
      maximum_amplitude = amplitude;
    }
  }
  if (maximum_amplitude <= 0.0f)
  {
    return;
  }

  for (point = 0U; point < APP_HMI_SPECTRUM_WIDTH; ++point)
  {
    hmi_spectrum_data[point] = 0U;
  }

  for (component = 0U;
       component < analysis_result.component_count;
       ++component)
  {
    const SignalAnalysisComponent_t *current =
        &analysis_result.components[component];
    uint32_t position;
    uint32_t height;

    if ((current->frequency_hz < APP_HMI_SPECTRUM_MIN_HZ) ||
        (current->frequency_hz > (float)SIGNAL_ANALYSIS_MAX_FREQUENCY_HZ))
    {
      continue;
    }

    position = (uint32_t)(
        (float)APP_HMI_SPECTRUM_X_MARGIN +
        ((current->frequency_hz - APP_HMI_SPECTRUM_MIN_HZ) *
         (float)(APP_HMI_SPECTRUM_WIDTH - 1U -
                 (2U * APP_HMI_SPECTRUM_X_MARGIN)) /
         ((float)SIGNAL_ANALYSIS_MAX_FREQUENCY_HZ -
          APP_HMI_SPECTRUM_MIN_HZ)) + 0.5f);
    height = (uint32_t)(
        ((current->amplitude_peak_v / maximum_amplitude) * vertical_span) +
        0.5f);
    if (height == 0U)
    {
      height = 1U;
    }
    if (height >= APP_HMI_SPECTRUM_HEIGHT)
    {
      height = APP_HMI_SPECTRUM_HEIGHT - 1U;
    }
    hmi_spectrum_data[position] = (uint8_t)height;
  }

  app.hmi_spectrum_request = 0U;
  app.hmi_spectrum_pending = 1U;
}

static void App_SendCaptureHeartbeat(void)
{
  static const uint8_t measurement_heartbeat[] =
      "Upp_mV=0.000,Urms_mV=0.000,f1_Hz=0.000\r\n";
  static const uint8_t component_heartbeat[] = "Q4_WAITING\r\n";

  /* Q1 waveform mode reserves USART1 for VOFA waveform samples only. */
  if (App_IsWaveformMode(app.mode) != 0U)
  {
    return;
  }

  if (App_IsComponentMode(app.mode) != 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)component_heartbeat,
                           sizeof(component_heartbeat) - 1U, 20U);
  }
  else
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)measurement_heartbeat,
                           sizeof(measurement_heartbeat) - 1U, 20U);
  }

  app.uart_heartbeat_tick = HAL_GetTick();
}

/*
 * Lock the raw display frame to the rising zero crossing predicted from the
 * measured fundamental phase.  The raw crossing preserves square-wave edges
 * and waveform detail that a limited harmonic reconstruction would lose.
 */
static uint8_t App_FindWaveformStart(float *start_sample,
                                     float *period_samples)
{
  float minimum = 0.0f;
  float maximum = 0.0f;
  float hysteresis;
  float low_threshold;
  float high_threshold;
  float expected_phase_sample;
  float best_error = 1.0e30f;
  float selected_start = -1.0f;
  uint8_t armed = 0U;
  uint32_t index;

  if ((adc_frame == NULL) || (start_sample == NULL) ||
      (period_samples == NULL) ||
      (analysis_result.component_count == 0U) ||
      (analysis_result.fundamental_hz < 10000.0f) ||
      (analysis_result.fundamental_hz > 1000000.0f))
  {
    return 0U;
  }

  *period_samples = (float)ADC_SAMPLE_RATE_HZ /
                    analysis_result.fundamental_hz;
  if ((*period_samples < 3.0f) ||
      (((float)APP_WAVEFORM_AVERAGE_CYCLES * (*period_samples)) >=
       (float)(ADC_NUMBER - 1U)))
  {
    return 0U;
  }

  minimum = SignalAnalysis_AdcCodeToVoltage(adc_frame[0]);
  maximum = minimum;
  for (index = 1U; index < ADC_NUMBER; ++index)
  {
    float value = SignalAnalysis_AdcCodeToVoltage(adc_frame[index]);
    if (value < minimum)
    {
      minimum = value;
    }
    if (value > maximum)
    {
      maximum = value;
    }
  }

  hysteresis = (maximum - minimum) * APP_TRIGGER_HYSTERESIS_RATIO;
  if (hysteresis < APP_TRIGGER_MIN_HYSTERESIS_V)
  {
    hysteresis = APP_TRIGGER_MIN_HYSTERESIS_V;
  }
  low_threshold = analysis_result.adc_dc_voltage_v - hysteresis;
  high_threshold = analysis_result.adc_dc_voltage_v + hysteresis;

  expected_phase_sample =
      (APP_RISING_ZERO_PHASE_RAD - analysis_result.components[0].phase_rad) /
      APP_TWO_PI;
  expected_phase_sample -= floorf(expected_phase_sample);
  expected_phase_sample *= *period_samples;

  for (index = 1U; index < ADC_NUMBER; ++index)
  {
    float previous = SignalAnalysis_AdcCodeToVoltage(adc_frame[index - 1U]);
    float current = SignalAnalysis_AdcCodeToVoltage(adc_frame[index]);

    if (current <= low_threshold)
    {
      armed = 1U;
    }
    else if ((armed != 0U) && (current >= high_threshold))
    {
      float crossing;
      float crossing_phase;
      float phase_error;
      float denominator = current - previous;

      if (fabsf(denominator) > 1.0e-9f)
      {
        crossing = (float)(index - 1U) +
                   ((analysis_result.adc_dc_voltage_v - previous) / denominator);
      }
      else
      {
        crossing = (float)index;
      }
      armed = 0U;

      if ((crossing +
           ((float)APP_WAVEFORM_AVERAGE_CYCLES * (*period_samples))) >=
          (float)(ADC_NUMBER - 1U))
      {
        continue;
      }

      crossing_phase = fmodf(crossing, *period_samples);
      phase_error = crossing_phase - expected_phase_sample;
      if (phase_error > (0.5f * (*period_samples)))
      {
        phase_error -= *period_samples;
      }
      else if (phase_error < (-0.5f * (*period_samples)))
      {
        phase_error += *period_samples;
      }

      if (fabsf(phase_error) < best_error)
      {
        best_error = fabsf(phase_error);
        selected_start = crossing;
      }
    }
  }

  if (selected_start < 0.0f)
  {
    return 0U;
  }
  *start_sample = selected_start;
  return 1U;
}

static float App_InterpolateRawVoltage(float sample_index)
{
  uint32_t index = (uint32_t)sample_index;
  float fraction = sample_index - (float)index;
  float previous;
  float first;
  float second;
  float next;
  float fraction_squared;
  float fraction_cubed;
  float interpolated;
  float segment_minimum;
  float segment_maximum;

  if (index >= (ADC_NUMBER - 1U))
  {
    return SignalAnalysis_AdcCodeToVoltage(adc_frame[ADC_NUMBER - 1U]) -
           analysis_result.adc_dc_voltage_v;
  }

  first = SignalAnalysis_AdcCodeToVoltage(adc_frame[index]);
  second = SignalAnalysis_AdcCodeToVoltage(adc_frame[index + 1U]);
  if ((index == 0U) || (index >= (ADC_NUMBER - 2U)))
  {
    return first + ((second - first) * fraction) -
           analysis_result.adc_dc_voltage_v;
  }

  previous = SignalAnalysis_AdcCodeToVoltage(adc_frame[index - 1U]);
  next = SignalAnalysis_AdcCodeToVoltage(adc_frame[index + 2U]);
  fraction_squared = fraction * fraction;
  fraction_cubed = fraction_squared * fraction;
  interpolated = 0.5f *
                 ((2.0f * first) +
                  ((-previous + second) * fraction) +
                  (((2.0f * previous) - (5.0f * first) +
                    (4.0f * second) - next) * fraction_squared) +
                  ((-previous + (3.0f * first) - (3.0f * second) + next) *
                   fraction_cubed));

  segment_minimum = fminf(first, second);
  segment_maximum = fmaxf(first, second);
  if (interpolated < segment_minimum)
  {
    interpolated = segment_minimum;
  }
  else if (interpolated > segment_maximum)
  {
    interpolated = segment_maximum;
  }

  return interpolated - analysis_result.adc_dc_voltage_v;
}

static void App_SmoothWaveform(void)
{
  static const uint8_t one_cycle_weights[9] = {
      1U, 8U, 28U, 56U, 70U, 56U, 28U, 8U, 1U};
  static const uint8_t three_cycle_weights[3] = {1U, 2U, 1U};
  const uint8_t *weights;
  uint8_t weight_count;
  uint16_t point;

  if (app.display_cycles == 1U)
  {
    weights = one_cycle_weights;
    weight_count = 9U;
  }
  else
  {
    weights = three_cycle_weights;
    weight_count = 3U;
  }

  for (point = 0U; point < (APP_HMI_WAVEFORM_WIDTH - 1U); ++point)
  {
    int32_t source = (int32_t)point - (int32_t)(weight_count / 2U);
    float weighted_sum = 0.0f;
    uint16_t weight_sum = 0U;
    uint8_t weight;

    for (weight = 0U; weight < weight_count; ++weight, ++source)
    {
      int32_t wrapped_source = source;

      if (wrapped_source < 0)
      {
        wrapped_source += (int32_t)(APP_HMI_WAVEFORM_WIDTH - 1U);
      }
      else if (wrapped_source >= (int32_t)(APP_HMI_WAVEFORM_WIDTH - 1U))
      {
        wrapped_source -= (int32_t)(APP_HMI_WAVEFORM_WIDTH - 1U);
      }
      weighted_sum += hmi_waveform_voltage[wrapped_source] *
                      (float)weights[weight];
      weight_sum += weights[weight];
    }
    hmi_waveform_smoothed[point] = weighted_sum / (float)weight_sum;
  }
  hmi_waveform_smoothed[APP_HMI_WAVEFORM_WIDTH - 1U] =
      hmi_waveform_smoothed[0];
}

static void App_SendMeasurements(void)
{
  float frame_count;
  uint16_t length = 0U;

  /* Do not mix Upp/Urms/f1 text into a Q1 VOFA waveform stream. */
  if (App_IsWaveformMode(app.mode) != 0U)
  {
    App_ResetMeasurementAverage();
    return;
  }

  if ((analysis_result.component_count == 0U) ||
      (analysis_result.fundamental_hz < APP_MEASURE_MIN_FREQUENCY_HZ) ||
      (analysis_result.fundamental_hz > APP_MEASURE_MAX_FREQUENCY_HZ))
  {
    App_ResetMeasurementAverage();
    if ((HAL_GetTick() - app.uart_heartbeat_tick) >= APP_UART_HEARTBEAT_MS)
    {
      App_SendCaptureHeartbeat();
    }
    return;
  }

  app.measurement_upp_sum_v += analysis_result.peak_to_peak_v;
  app.measurement_rms_sum_v += analysis_result.true_rms_v;
  app.measurement_f1_sum_hz += analysis_result.fundamental_hz;
  app.measurement_frame_count++;

  if (app.measurement_frame_count < APP_MEASURE_FRAME_COUNT)
  {
    return;
  }

  frame_count = (float)app.measurement_frame_count;
  length = App_AppendText(uart_buffer, length, "Upp_mV=");
  length = App_AppendFixed3(uart_buffer, length,
                            (app.measurement_upp_sum_v * 1000.0f) /
                            frame_count);
  length = App_AppendText(uart_buffer, length, ",Urms_mV=");
  length = App_AppendFixed3(uart_buffer, length,
                            (app.measurement_rms_sum_v * 1000.0f) /
                            frame_count);
  length = App_AppendText(uart_buffer, length, ",f1_Hz=");
  length = App_AppendFixed3(uart_buffer, length,
                            app.measurement_f1_sum_hz / frame_count);
  uart_buffer[length++] = '\r';
  uart_buffer[length++] = '\n';
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, length, 100U);
  app.uart_heartbeat_tick = HAL_GetTick();
  App_ResetMeasurementAverage();
}

static void App_ResetMeasurementAverage(void)
{
  app.measurement_frame_count = 0U;
  app.measurement_upp_sum_v = 0.0f;
  app.measurement_rms_sum_v = 0.0f;
  app.measurement_f1_sum_hz = 0.0f;
}

static void App_SendSpectrum(void)
{
  uint16_t length = 0U;
  uint16_t bin;
  const uint16_t maximum_bin = (uint16_t)(
      (((uint32_t)SIGNAL_ANALYSIS_MAX_FREQUENCY_HZ *
        SIGNAL_ANALYSIS_FFT_SIZE) + ADC_SAMPLE_RATE_HZ - 1UL) /
      ADC_SAMPLE_RATE_HZ);

  /* VOFA XY data: channel 0 is frequency in kHz and channel 1 is Vpeak in
     mV.  One point per bin gives a frequency interval of 488.28125 Hz. */
  for (bin = 0U; bin <= maximum_bin; ++bin)
  {
    float frequency_khz = ((float)bin * (float)ADC_SAMPLE_RATE_HZ) /
                          ((float)SIGNAL_ANALYSIS_FFT_SIZE * 1000.0f);
    length = App_AppendFixed3(uart_buffer, length, frequency_khz);
    uart_buffer[length++] = ',';
    length = App_AppendFixed3(uart_buffer, length,
                              SignalAnalysis_GetSpectrumPeakV(bin) * 1000.0f);
    uart_buffer[length++] = '\r';
    uart_buffer[length++] = '\n';
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, length, 300U);
}

static void App_SendComponents(void)
{
  uint16_t length = 0U;
  uint32_t index;
  uint8_t frame_matches = 1U;

  if ((analysis_result.component_count == 0U) ||
      (analysis_result.fundamental_hz < APP_MEASURE_MIN_FREQUENCY_HZ) ||
      (analysis_result.fundamental_hz > APP_MEASURE_MAX_FREQUENCY_HZ))
  {
    App_ResetComponentAverage();
    if ((HAL_GetTick() - app.uart_heartbeat_tick) >= APP_UART_HEARTBEAT_MS)
    {
      App_SendCaptureHeartbeat();
    }
    return;
  }

  if (app.component_frame_count != 0U)
  {
    if (app.component_count != analysis_result.component_count)
    {
      frame_matches = 0U;
    }
    else
    {
      for (index = 0U; index < analysis_result.component_count; ++index)
      {
        if (app.component_orders[index] !=
            analysis_result.components[index].harmonic_order)
        {
          frame_matches = 0U;
          break;
        }
      }
    }
  }

  if (frame_matches == 0U)
  {
    App_ResetComponentAverage();
  }

  if (app.component_frame_count == 0U)
  {
    app.component_count = analysis_result.component_count;
    for (index = 0U; index < app.component_count; ++index)
    {
      app.component_orders[index] =
          analysis_result.components[index].harmonic_order;
    }
  }

  for (index = 0U; index < app.component_count; ++index)
  {
    app.component_frequency_sum_hz[index] +=
        analysis_result.components[index].frequency_hz;
    app.component_amplitude_sum_v[index] +=
        analysis_result.components[index].amplitude_peak_v;
  }
  app.component_frame_count++;

  if (app.component_frame_count < APP_COMPONENT_FRAME_COUNT)
  {
    return;
  }

  for (index = 0U; index < app.component_count; ++index)
  {
    length = App_AppendText(uart_buffer, length, "U");
    length = App_AppendUnsigned(uart_buffer, length,
                                app.component_orders[index]);
    length = App_AppendText(uart_buffer, length, "_f_Hz=");
    length = App_AppendFixed3(uart_buffer, length,
        app.component_frequency_sum_hz[index] /
        (float)app.component_frame_count);
    length = App_AppendText(uart_buffer, length, ",U");
    length = App_AppendUnsigned(uart_buffer, length,
                                app.component_orders[index]);
    length = App_AppendText(uart_buffer, length, "_mVpk=");
    length = App_AppendFixed3(uart_buffer, length,
        (app.component_amplitude_sum_v[index] * 1000.0f) /
        (float)app.component_frame_count);
    uart_buffer[length++] = '\r';
    uart_buffer[length++] = '\n';
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, length, 100U);
  app.uart_heartbeat_tick = HAL_GetTick();
  App_ResetComponentAverage();
}

static void App_ResetComponentAverage(void)
{
  uint32_t index;

  app.component_frame_count = 0U;
  app.component_count = 0U;
  for (index = 0U; index < SIGNAL_ANALYSIS_MAX_COMPONENTS; ++index)
  {
    app.component_orders[index] = 0U;
    app.component_frequency_sum_hz[index] = 0.0f;
    app.component_amplitude_sum_v[index] = 0.0f;
  }
}

static uint16_t App_AppendText(char *buffer, uint16_t offset, const char *text)
{
  while (*text != '\0')
  {
    buffer[offset++] = *text++;
  }
  return offset;
}

static uint16_t App_AppendUnsigned(char *buffer, uint16_t offset, uint32_t value)
{
  char digits[10];
  uint8_t count = 0U;

  do
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U);

  while (count > 0U)
  {
    buffer[offset++] = digits[--count];
  }
  return offset;
}

static uint16_t App_AppendFixed3(char *buffer, uint16_t offset, float value)
{
  int32_t value_x1000 = (int32_t)(value * 1000.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
  uint32_t fraction;

  if (value_x1000 < 0)
  {
    buffer[offset++] = '-';
    value_x1000 = -value_x1000;
  }
  offset = App_AppendUnsigned(buffer, offset, (uint32_t)value_x1000 / 1000U);
  buffer[offset++] = '.';
  fraction = (uint32_t)value_x1000 % 1000U;
  buffer[offset++] = (char)('0' + (fraction / 100U));
  buffer[offset++] = (char)('0' + ((fraction / 10U) % 10U));
  buffer[offset++] = (char)('0' + (fraction % 10U));
  return offset;
}
