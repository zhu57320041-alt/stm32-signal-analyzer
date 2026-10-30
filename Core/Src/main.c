/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "ADC_DMA.h"
#include "message_UART.h"
#include "HMI.h"
#include "OSC_Trigger.h"
#include "Config.h"
#include "Fre_Measure.h"
#include "FFT.h"
#include "DAC_DMA.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t *adc_value_buffer_u32;
uint32_t adc_value_u32[ADC_NUMBER];
static uint32_t adc_value_u32_HasFilter[ADC_NUMBER];
static uint8_t print_frame[8 + ADC_NUMBER * 2];  // 电脑串口帧缓冲区
static uint32_t adc_value_u32_HasScale[ADC_NUMBER];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM5_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  
  uint8_t RxBuffer[16];             // 定义接收缓冲区
  uint16_t count_1 = 0;             // 用于计数
  message_init();                   // 初始化串口
  Fre_Measure_Init();               // 初始化频率测量
  ConfigAndData_Init();             // 初始化配置和数据
  ADC_DMA_Start(OSC_Config.OSC_Fs); // 启动ADC
  ADC_Change_Fs();                  // 改变ADC采样频率
  FFT_Init();                       // 初始化FFT
  DAC_DMA_Init();                   // 初始化DAC


  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (message_Read(RxBuffer, 16, 1)) // 读取串口数据
    {
      if (HMI_ReceiveHandle(RxBuffer, 16) == 0) // 处理串口数据
      {
      }
    }
    if (OSC_Config.OSC_Enable) // 判断示波器是否使能
    {
      if (count_1 >= 100) // 判断定时
      {
        count_1 = 0;
        // 无屏：不发送 HMI_SeedFrame_MeasureData，避免屏指令污染电脑数据流
        if ((adc_value_buffer_u32 = ADC_ReadData_u32()) != NULL) // 判断ADC是否有数据
        {
          if (OSC_Config.OSC_Fs == 7200000)
          {
            ADC_MultiModeBufferConvert(adc_value_buffer_u32, adc_value_u32, ADC_BUFFER_MultiMode_SIZE);    // 交替采样数据的转换
            Filter_IIR(adc_value_u32, adc_value_u32_HasFilter, 0.8);                                       // 滤波
            ADC_DataHandle(adc_value_u32_HasFilter, 20, 730);                                              // 数据处理，获得Vpp等
            Fre_Measure_Get_Fre(&OSC_MeasureData.Freq_TIM, &OSC_MeasureData.Duty);                         // 测量频率
            Scale_X_Axis(adc_value_u32_HasFilter, adc_value_u32_HasScale, (float)OSC_Config.Scale_X_Axis); // 缩放
          }
          else
          {
            Filter_IIR(adc_value_buffer_u32, adc_value_u32_HasFilter, 0.4);                             // 滤波
            ADC_DataHandle(adc_value_u32_HasFilter, 20, 730);                                           // 数据处理，获得Vpp等
            Fre_Measure_Get_Fre(&OSC_MeasureData.Freq_TIM, &OSC_MeasureData.Duty);                      // 测量频率
            Scale_X_Axis(adc_value_buffer_u32, adc_value_u32_HasScale, (float)OSC_Config.Scale_X_Axis); // 缩放
          }
          if (OSC_Config.OSC_Print_Enable) // 发送二进制帧到电脑
          {
            // ---- 波形帧 (0xAA 0xBB) ----
            print_frame[0] = 0xAA; print_frame[1] = 0xBB; // 帧同步头
            uint16_t len = ADC_NUMBER;
            print_frame[2] = len & 0xFF;             // 点数低字节
            print_frame[3] = (len >> 8) & 0xFF;      // 点数高字节
            uint32_t fs_hz = OSC_Config.OSC_Fs;
            print_frame[4] = fs_hz & 0xFF;            // 采样率[31:24]
            print_frame[5] = (fs_hz >> 8) & 0xFF;
            print_frame[6] = (fs_hz >> 16) & 0xFF;
            print_frame[7] = (fs_hz >> 24) & 0xFF;    // 采样率[7:0]
            uint32_t *pSrc = (OSC_Config.OSC_Fs == 7200000) ? adc_value_u32 : adc_value_buffer_u32;
            for (int i = 0; i < ADC_NUMBER; i++)
            {
              uint16_t val = (uint16_t)(pSrc[i] & 0xFFF);
              print_frame[8 + i * 2] = val & 0xFF;
              print_frame[8 + i * 2 + 1] = (val >> 8) & 0xFF;
            }
            HAL_UART_Transmit(&huart1, print_frame, sizeof(print_frame), 500);

            // ---- FFT频谱帧 (0xCC 0xDD) - 自动运行FFT并发送结果 ----
            FFT_App(&OSC_MeasureData.Freq_FFT);
            {
              static uint8_t fft_frame[16 + (FFT_LENGTH / 2 + 1) * 4 + 16]; // 2+4+4+4+2 + 513*4 + 16(二三次谐波) = 2084B
              uint32_t max_freq = OSC_Config.OSC_Fs / 2;
              uint16_t fft_len = FFT_LENGTH / 2 + 1;  // 513
              uint8_t *p;
              // 同步头
              fft_frame[0] = 0xCC; fft_frame[1] = 0xDD;
              // max_freq (uint32 LE)
              p = (uint8_t *)&max_freq;
              fft_frame[2] = p[0]; fft_frame[3] = p[1];
              fft_frame[4] = p[2]; fft_frame[5] = p[3];
              // peak_freq (float32)
              p = (uint8_t *)&OSC_MeasureData.Freq_FFT;
              fft_frame[6] = p[0]; fft_frame[7] = p[1];
              fft_frame[8] = p[2]; fft_frame[9] = p[3];
              // peak_amp (float32)
              p = (uint8_t *)&OSC_MeasureData.Freq_FFT_Amp;
              fft_frame[10] = p[0]; fft_frame[11] = p[1];
              fft_frame[12] = p[2]; fft_frame[13] = p[3];
              // fft_len (uint16 LE)
              p = (uint8_t *)&fft_len;
              fft_frame[14] = p[0]; fft_frame[15] = p[1];
              // FFT幅度数据 (float32数组)
              for (uint16_t i = 0; i < fft_len; i++)
              {
                p = (uint8_t *)&output[i];
                uint32_t idx = 16 + i * 4;
                fft_frame[idx]     = p[0];
                fft_frame[idx + 1] = p[1];
                fft_frame[idx + 2] = p[2];
                fft_frame[idx + 3] = p[3];
              }
              // ---- 帧尾追加：二次+三次谐波 频率+幅值 (float32 LE, 16字节) ----
              // 布局: 16 + fft_len*4 + [h2_freq(4) | h2_amp(4) | h3_freq(4) | h3_amp(4)]
              FFT_Harmonic_t harms[3];
              FFT_GetHarmonics((float)OSC_Config.OSC_Fs, harms);
              OSC_MeasureData.Freq_FFT_H2     = harms[1].freq;
              OSC_MeasureData.Freq_FFT_H2_Amp = harms[1].amp;
              OSC_MeasureData.Freq_FFT_H3     = harms[2].freq;
              OSC_MeasureData.Freq_FFT_H3_Amp = harms[2].amp;
              uint32_t tail = 16 + fft_len * 4;
              p = (uint8_t *)&OSC_MeasureData.Freq_FFT_H2;       // h2_freq
              fft_frame[tail] = p[0]; fft_frame[tail+1] = p[1]; fft_frame[tail+2] = p[2]; fft_frame[tail+3] = p[3];
              p = (uint8_t *)&OSC_MeasureData.Freq_FFT_H2_Amp;   // h2_amp
              fft_frame[tail+4] = p[0]; fft_frame[tail+5] = p[1]; fft_frame[tail+6] = p[2]; fft_frame[tail+7] = p[3];
              p = (uint8_t *)&OSC_MeasureData.Freq_FFT_H3;       // h3_freq
              fft_frame[tail+8] = p[0]; fft_frame[tail+9] = p[1]; fft_frame[tail+10] = p[2]; fft_frame[tail+11] = p[3];
              p = (uint8_t *)&OSC_MeasureData.Freq_FFT_H3_Amp;   // h3_amp
              fft_frame[tail+12] = p[0]; fft_frame[tail+13] = p[1]; fft_frame[tail+14] = p[2]; fft_frame[tail+15] = p[3];
              HAL_UART_Transmit(&huart1, fft_frame, sizeof(fft_frame), 500);
            }
          }
          if (!OSC_Config.OSC_Print_Enable) // 开启电脑打印时，跳过HMI屏幕的波形发送
          {
            if (OSC_Config.OSC_Auto_Enable == 1) // 判断是否开启自动测量，
            {
              Auto_Measure(); // 自动测量
            }
            else if (OSC_Config.OSC_FFT_Enable == 1) // 判断是否开启FFT
            {
              FFT_App(&OSC_MeasureData.Freq_FFT); // 进行FFT（基波）
              // 检测基波+二次+三次谐波
              FFT_Harmonic_t harms[3];
              FFT_GetHarmonics((float)OSC_Config.OSC_Fs, harms);
              OSC_MeasureData.Freq_FFT_H2     = harms[1].freq;
              OSC_MeasureData.Freq_FFT_H2_Amp = harms[1].amp;
              OSC_MeasureData.Freq_FFT_H3     = harms[2].freq;
              OSC_MeasureData.Freq_FFT_H3_Amp = harms[2].amp;
              HMI_SeedFrame_FFT(output, 512);     // 发送FFT数据
            }
            else
            {
              HMI_SeedFrame_Wave(adc_value_u32_HasScale, 800, ADC_NUMBER, 0);
            }
          }
          ADC_Restart(OSC_Config.OSC_Fs); // 重新启动ADC
        }
      }
    }
    if (DAC_Param.HaveChange_Flag == 1) // 判断DAC有无修改参数
    {
      DAC_Param.HaveChange_Flag = 0;
      if (OSC_Config.OSC_DAC_Enable == 1) // 判断DAC是否使能
      {
        DAC_DMA_ChangeOut(DAC_Param.type, DAC_Param.Freq, DAC_Param.VPP, DAC_Param.offset, DAC_Param.Duty); // 修改DAC参数
        DAC_DMA_Start();                                                                                    // 启动DAC
      }
      else
      {
        DAC_DMA_Stop(); // 停止DAC
      }
    }
    HAL_Delay(1);
    count_1++; // 计数值加1

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
