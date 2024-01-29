/**
  ******************************************************************************
  * @file    printf.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRINTF_H__
#define __PRINTF_H__

#ifdef __cplusplus
 extern "C" {
#endif

#define DEBUG_UART1

#ifdef DEBUG_UART1
#define DEBUG_UART                  USART1
#define DEBUG_UART_TX               GPIO_Pin_9
#define DEBUG_UART_RX               GPIO_Pin_10
#define DEBUG_UART_PORT             GPIOA
#define DEBUG_UART_RCC_PORT         RCC_APB2Periph_GPIOA
#define DEBUG_UART_RCC_CLK          RCC_APB2Periph_USART1
#endif

void debug_uart_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PRINTF_H__ */
