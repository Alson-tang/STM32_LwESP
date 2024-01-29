/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.6.0
  * @date    20-September-2021
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2011 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "lwesp_opts.h"

#include "FreeRTOS.h"
#include "task.h"

#include "printf.h"
#include "delay.h"

#include "lwesp/lwesp.h"
#include "station_manager.h"
#include "netconn_client.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
lwespr_t lwesp_callback_func(lwesp_evt_t* evt) {
    switch (lwesp_evt_get_type(evt)) {
        case LWESP_EVT_AT_VERSION_NOT_SUPPORTED: {
            lwesp_sw_version_t v_min, v_curr;

            lwesp_get_min_at_fw_version(&v_min);
            lwesp_get_current_at_fw_version(&v_curr);

            printf("Current ESP8266 AT version is not supported by library!\r\n");
            printf("Minimum required AT version is: %d.%d.%d\r\n", (int)v_min.major, (int)v_min.minor, (int)v_min.patch);
            printf("Current AT version is: %d.%d.%d\r\n", (int)v_curr.major, (int)v_curr.minor, (int)v_curr.patch);
            break;
        }
        case LWESP_EVT_INIT_FINISH: {
            printf("Library initialized!\r\n");
            break;
        }
        case LWESP_EVT_RESET_DETECTED: {
            printf("Device reset detected!\r\n");
            break;
        }
        case LWESP_EVT_WIFI_IP_ACQUIRED: {        /* We have IP address and we are fully ready to work */
            if (lwesp_sta_is_joined()) {          /* Check if joined on any network */
                lwesp_sys_thread_create(NULL, "netconn_client", (lwesp_sys_thread_fn)netconn_client_thread, NULL, 512, LWESP_SYS_THREAD_PRIO);
            }
            break;
        }
        case LWESP_EVT_WIFI_CONNECTED: {
            printf("Successfully joined AP\r\n");
            break;
        }
        default:
            break;
    }
    return lwespOK;
}

void join_ap_test(void *pvParameters)
{
    /* Initialize ESP with default callback function */
    printf("Initializing LwESP\r\n");
    if (lwesp_init(lwesp_callback_func, 1) != lwespOK) {
        printf("Cannot initialize LwESP!\r\n");
    } else {
        printf("LwESP initialized!\r\n");
    }

    /*
     * Continuously try to connect to WIFI network
     * but only in case device is not already connected
     */
    while (1) {
        if (!lwesp_sta_is_joined()) {
            /*
             * Connect to access point.
             *
             * Try unlimited time until access point accepts up.
             * Check for station_manager.c to define preferred access points ESP should connect to
             */
            connect_to_preferred_access_point(1);
        }
        vTaskDelay(1000);
    }

    vTaskDelete(NULL);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured, 
        this is done through SystemInit() function which is called from startup
        file (startup_stm32f10x_xx.s) before to branch to application main.
        To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f10x.c file
        */

    /* Add your application code here
        */
    BaseType_t xReturn = pdPASS;

    NVIC_Configuration();

    debug_uart_init();
    printf("Debug UART output success\r\n");
	
    xReturn = xTaskCreate(join_ap_test, "join_ap_test", 1024 * 2, NULL, 1, NULL);
    if(pdPASS == xReturn)
        vTaskStartScheduler();
    else
        return -1;

    /* Infinite loop */
    while (1);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
