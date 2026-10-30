#include "message_uart.h"
#include "string.h"
uint8_t Data_Ready_Flag_1 = 0;             // 定义数据就绪标志
static uint8_t RxBuffer_1[MAX_BUFFER_SIZE];       // 定义接收缓冲区
extern DMA_HandleTypeDef hdma_usartx_rx_1; // 声明DMA句柄
extern UART_HandleTypeDef huartx_1;        // 声明UART句柄
#if USART_NUM >= 2
uint8_t Data_Ready_Flag_2 = 0;             // 定义数据就绪标志
uint8_t RxBuffer_2[MAX_BUFFER_SIZE];       // 定义接收缓冲区
extern DMA_HandleTypeDef hdma_usartx_rx_2; // 声明DMA句柄
extern UART_HandleTypeDef huartx_2;        // 声明UART句柄
#endif
// 初始化函数
void message_init(void)
{
  memset(RxBuffer_1, '\0', sizeof(RxBuffer_1));                            // 清空接收缓冲区
  HAL_UARTEx_ReceiveToIdle_DMA(&huartx_1, RxBuffer_1, sizeof(RxBuffer_1)); // 启动DMA接收
  __HAL_DMA_DISABLE_IT(&hdma_usartx_rx_1, DMA_IT_HT);                      // 禁用半传输中断
#if USART_NUM >= 2
  memset(RxBuffer_2, '\0', sizeof(RxBuffer_2));                            // 清空接收缓冲区
  HAL_UARTEx_ReceiveToIdle_DMA(&huartx_2, RxBuffer_2, sizeof(RxBuffer_2)); // 启动DMA接收
  __HAL_DMA_DISABLE_IT(&hdma_usartx_rx_2, DMA_IT_HT);                      // 禁用半传输中断
#endif
}
// 回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USARTx_1)
  {
    __HAL_UNLOCK(huart);                                                     // 解锁UART句柄
    Data_Ready_Flag_1 = 1;                                                   // 设置数据就绪标志
    HAL_UARTEx_ReceiveToIdle_DMA(&huartx_1, RxBuffer_1, sizeof(RxBuffer_1)); // 重新启动DMA接收
    __HAL_DMA_DISABLE_IT(&hdma_usartx_rx_1, DMA_IT_HT);                      // 禁用半传输中断
  }
#if USART_NUM >= 2
  else if (huart->Instance == USARTx_2)
  {
    __HAL_UNLOCK(huart);                                                     // 解锁UART句柄
    Data_Ready_Flag_2 = 1;                                                   // 设置数据就绪标志
    HAL_UARTEx_ReceiveToIdle_DMA(&huartx_2, RxBuffer_2, sizeof(RxBuffer_2)); // 重新启动DMA接收
    __HAL_DMA_DISABLE_IT(&hdma_usartx_rx_2, DMA_IT_HT);                      // 禁用半传输中断
  }
#endif
}
/**
 * @brief 读取接收到的数据
 * @param data 存储接收到数据
 * @param size 要读取的数据大小
 * @param UART_ID UART ID
 * @return 1 如果有新数据可读，0 否则
 */
uint8_t message_Read(uint8_t *data, uint16_t size, uint8_t UART_ID)
{
  if (UART_ID == 1)
  {
    if (Data_Ready_Flag_1)
    {
      Data_Ready_Flag_1 = 0;                        // 清除数据就绪标志
      memcpy(data, RxBuffer_1, size);               // 复制接收到的数据
      memset(RxBuffer_1, '\0', sizeof(RxBuffer_1)); // 清空接收缓冲区
      return 1;
    }
    else
      return 0; // 没有新数据可读
  }
#if USART_NUM >= 2
  else if (UART_ID == 2)
  {
    if (Data_Ready_Flag_2)
    {
      Data_Ready_Flag_2 = 0;                        // 清除数据就绪标志
      memcpy(data, RxBuffer_2, size);               // 复制接收到的数据
      memset(RxBuffer_2, '\0', sizeof(RxBuffer_2)); // 清空接收缓冲区
      return 1;
    }
  }
#endif
  return 0;
}

uint8_t *message_Read_RxBuffer(uint8_t UART_ID)
{
  if (UART_ID == 2)
  {
    if (Data_Ready_Flag_1)
    {
      return RxBuffer_1;
    }
    else
      return NULL; // 返回接收到的数据
  }
#if USART_NUM >= 2
  else if (UART_ID == 2)
  {
    if (Data_Ready_Flag_2)
    {
      return RxBuffer_2;
    }
    else
      return NULL; // 返回接收到的数据
  }
#endif
  return 0;
}

/**
 * @brief 发送数据
 * @param RxData 要发送的数据
 * @param UART_ID UART ID
 * @return 无
 * @note 该函数使用IT发送数据, 使其在发送时可继续执行其他任务，但不能连续发送多个数据，
 */
void message_Transmit_IT(uint8_t *RxData, uint8_t UART_ID)
{
  if (UART_ID == 1)
  {
    HAL_UART_Transmit_IT(&huartx_1, RxData, strlen((char *)RxData)); // 发送数据
  }
#if USART_NUM >= 2
  else if (UART_ID == 2)
  {
    HAL_UART_Transmit_IT(&huartx_2, RxData, strlen((char *)RxData)); // 发送数据
  }
#endif
}

/**
 * @brief 发送数据
 * @param RxData 要发送的数据
 * @param UART_ID UART ID
 * @return 无
 */
void message_Transmit(uint8_t *RxData, uint8_t UART_ID)
{
  if (UART_ID == 1)
  {
    HAL_UART_Transmit(&huartx_1, RxData, strlen((char *)RxData), HAL_MAX_DELAY); // 发送数据
  }
#if USART_NUM >= 2
  else if (UART_ID == 2)
  {
    HAL_UART_Transmit(&huartx_2, RxData, strlen((char *)RxData), HAL_MAX_DELAY); // 发送数据
  }
#endif
}
void message_Transmit_Byte(uint8_t RxData, uint8_t UART_ID)
{
  if (UART_ID == 1)
  {
    HAL_UART_Transmit(&huartx_1, &RxData, 1, HAL_MAX_DELAY); // 发送数据
  }
#if USART_NUM >= 2
  else if (UART_ID == 2)
  {
    HAL_UART_Transmit(&huartx_2, &RxData, 1, HAL_MAX_DELAY); // 发送数据
  }
#endif
}

// 错误回调函数

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  // __HAL_UNLOCK(huart);
  // __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE|UART_FLAG_PE|UART_FLAG_ORE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huartx_1, RxBuffer_1, sizeof(RxBuffer_1));
  __HAL_DMA_DISABLE_IT(&hdma_usartx_rx_1, DMA_IT_HT);
#if USART_NUM >= 2
  HAL_UARTEx_ReceiveToIdle_DMA(&huartx_2, RxBuffer_2, sizeof(RxBuffer_2));
  __HAL_DMA_DISABLE_IT(&hdma_usartx_rx_2, DMA_IT_HT);
#endif
}

void message_CleanRxBuffer(uint8_t UART_ID)
{
  if (UART_ID == 1)
  {
    memset(RxBuffer_1, '\0', sizeof(RxBuffer_1)); // 清空接收缓冲区
  }
#if USART_NUM >= 2
  else if (UART_ID == 2)
  {
    memset(RxBuffer_2, '\0', sizeof(RxBuffer_2)); // 清空接收缓冲区
  }
#endif
}
