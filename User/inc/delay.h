/**
  ******************************************************************************
  * @file    delay.h
  * @author  Alson Tang
  * @version V1.0.0
  * @date    17-January-2024
  * @brief   This file contains all the functions prototypes for delay operations
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

extern uint32_t g_u32_time;

void delay_ms(uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H__ */
