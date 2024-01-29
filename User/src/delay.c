/**
  ******************************************************************************
  * @file    delay.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "delay.h"

uint32_t g_u32_time = 0;

void delay_ms(uint16_t ms)
{
    g_u32_time = ms;

    while(g_u32_time != 0);
}
