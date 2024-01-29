/**
  ******************************************************************************
  * @file    printf.c
  * @author  Alson Tang
  * @version V1.0.0
  * @date    17-January-2024
  * @brief   This file contains all the functions prototypes for printf API
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  * Change History
  *  <Date>     | <Version> | <Author>       | <Description>
  *  2024/01/24 | 1.0.0     | Alson.Tang     | Create file
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"

#include "printf.h"

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static USART_InitTypeDef s_st_debug_uart = { 0 };

/**
  * @brief  Configures UART.
  * @param  USARTx: Specifies the UART to be configured. 
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
static void debug_uart_cfg(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(DEBUG_UART_RCC_PORT | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(DEBUG_UART_RCC_CLK, ENABLE);

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = DEBUG_UART_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_UART_PORT, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = DEBUG_UART_RX;
    GPIO_Init(DEBUG_UART_PORT, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(DEBUG_UART, USART_InitStruct);

    /* Enable USART */
    USART_Cmd(DEBUG_UART, ENABLE);

    /* Read from the USART_SR register followed by a write to the USART_DR register to clear TC flag */
    USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TC);
}

void debug_uart_init(void)
{
    /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    s_st_debug_uart.USART_BaudRate = 115200;
    s_st_debug_uart.USART_WordLength = USART_WordLength_8b;
    s_st_debug_uart.USART_StopBits = USART_StopBits_1;
    s_st_debug_uart.USART_Parity = USART_Parity_No;
    s_st_debug_uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    s_st_debug_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    debug_uart_cfg(DEBUG_UART, &s_st_debug_uart);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(DEBUG_UART, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
