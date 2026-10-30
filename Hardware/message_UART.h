#ifndef MESSAGE_UART_H
#define MESSAGE_UART_H
#include "main.h"
#include "usart.h"
#define USART_NUM 1                     // 定义使用的UART数量
// 定义UART和DMA句柄
#define USARTx_1 USART1                 // 定义UART
#define huartx_1 huart1                 // 定义UART句柄
#define hdma_usartx_rx_1 hdma_usart1_rx // 定义DMA句柄

#if USART_NUM >= 2

#define USARTx_2 USART3                 // 定义UART
#define huartx_2 huart3                 // 定义UART句柄
#define hdma_usartx_rx_2 hdma_usart3_rx // 定义DMA句柄

#endif

#define MAX_BUFFER_SIZE 128 // 定义缓冲区大小

void message_init(void);          // 初始化函数
void message_CleanRxBuffer(uint8_t UART_ID); // 清空接收缓冲区

uint8_t *message_Read_RxBuffer(uint8_t UART_ID);               // 读取接收缓冲区
uint8_t message_Read(uint8_t *data, uint16_t size,uint8_t UART_ID); // 读取接收数据

void message_Transmit(uint8_t *RxData,uint8_t UART_ID); // 发送数据函数
void message_Transmit_IT(uint8_t *RxData,uint8_t UART_ID);
void message_Transmit_Byte(uint8_t RxData,uint8_t UART_ID);

#endif
