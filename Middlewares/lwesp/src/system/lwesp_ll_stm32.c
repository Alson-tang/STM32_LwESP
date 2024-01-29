/**
 * \file            lwesp_ll_stm32.c
 * \brief           Generic STM32 driver, included in various STM32 driver variants
 */

/*
 * Copyright (c) 2020 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of LwESP - Lightweight ESP-AT parser library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         v1.1.2-dev
 */

/*
 * How it works
 *
 * On first call to \ref lwesp_ll_init, new thread is created and processed in usart_ll_thread function.
 * USART is configured in RX DMA mode and any incoming bytes are processed inside thread function.
 * DMA and USART implement interrupt handlers to notify main thread about new data ready to send to upper layer.
 *
 * More about UART + RX DMA: https://github.com/MaJerle/stm32-usart-dma-rx-tx
 *
 * \ref LWESP_CFG_INPUT_USE_PROCESS must be enabled in `lwesp_config.h` to use this driver.
 */
#include "stm32f10x.h"

#include "lwesp/lwesp.h"
#include "lwesp/lwesp_mem.h"
#include "lwesp/lwesp_input.h"
#include "system/lwesp_ll.h"

#if !__DOXYGEN__

/* USART */
#define LWESP_USART                           USART2
#define LWESP_USART_CLK                       RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)
#define LWESP_USART_IRQ                       USART2_IRQn
#define LWESP_USART_IRQHANDLER                USART2_IRQHandler
#define LWESP_USART_RDR_NAME                  DR

/* USART TX PIN */
#define LWESP_USART_TX_PORT_CLK               RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE)
#define LWESP_USART_TX_PORT                   GPIOA
#define LWESP_USART_TX_PIN                    GPIO_Pin_2

/* USART RX PIN */
#define LWESP_USART_RX_PORT_CLK               RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE)
#define LWESP_USART_RX_PORT                   GPIOA
#define LWESP_USART_RX_PIN                    GPIO_Pin_3

/* DMA settings */
#define LWESP_USART_DMA                       DMA1
#define LWESP_USART_DMA_CLK                   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE)
#define LWESP_USART_DMA_RX_CH                 DMA1_Channel6
#define LWESP_USART_DMA_RX_IRQ                DMA1_Channel6_IRQn
#define LWESP_USART_DMA_RX_IRQHANDLER         DMA1_Channel6_IRQHandler

/* RESET PIN */
#define LWESP_RESET_PORT_CLK                  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)
#define LWESP_RESET_PORT                      GPIOA
#define LWESP_RESET_PIN                       GPIO_Pin_4

/* DMA flags management */
#define LWESP_USART_DMA_RX_CLEAR_TC           DMA_ClearFlag(DMA1_IT_TC6)
#define LWESP_USART_DMA_RX_CLEAR_HT           DMA_ClearFlag(DMA1_IT_HT6)

#if !LWESP_CFG_INPUT_USE_PROCESS
#error "LWESP_CFG_INPUT_USE_PROCESS must be enabled in `lwesp_config.h` to use this driver."
#endif /* LWESP_CFG_INPUT_USE_PROCESS */

#if !defined(LWESP_USART_DMA_RX_BUFF_SIZE)
#define LWESP_USART_DMA_RX_BUFF_SIZE      0x1000
#endif /* !defined(LWESP_USART_DMA_RX_BUFF_SIZE) */

#if !defined(LWESP_MEM_SIZE)
#define LWESP_MEM_SIZE                    0x1000
#endif /* !defined(LWESP_MEM_SIZE) */

#if !defined(LWESP_USART_RDR_NAME)
#define LWESP_USART_RDR_NAME              RDR
#endif /* !defined(LWESP_USART_RDR_NAME) */

/* USART memory */
static uint8_t      usart_mem[LWESP_USART_DMA_RX_BUFF_SIZE];
static uint8_t      is_running, initialized;
static size_t       old_pos;

/* USART thread */
static void usart_ll_thread(void* arg);
static TaskHandle_t usart_ll_thread_id;

/* Message queue */
static QueueHandle_t usart_ll_mbox_id;

/**
 * \brief           USART data processing
 */
static void
usart_ll_thread(void* arg) {
    size_t pos;

    LWESP_UNUSED(arg);

    while (1) {
        void* d;
        /* Wait for the event message from DMA or USART */
        xQueueReceive(usart_ll_mbox_id, &d, portMAX_DELAY);

        /* Read data */
#if defined(LWESP_USART_DMA_RX_STREAM)
        pos = sizeof(usart_mem) - LL_DMA_GetDataLength(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
#else
        pos = sizeof(usart_mem) - DMA_GetCurrDataCounter(LWESP_USART_DMA_RX_CH);
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */
        if (pos != old_pos && is_running) {
            if (pos > old_pos) {
                lwesp_input_process(&usart_mem[old_pos], pos - old_pos);
            } else {
                lwesp_input_process(&usart_mem[old_pos], sizeof(usart_mem) - old_pos);
                if (pos > 0) {
                    lwesp_input_process(&usart_mem[0], pos);
                }
            }
            old_pos = pos;
            if (old_pos == sizeof(usart_mem)) {
                old_pos = 0;
            }
        }
    }
}

/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
static void
configure_uart(uint32_t baudrate) {
    static USART_InitTypeDef USART_InitStruct = { 0 };
    static DMA_InitTypeDef DMA_InitStruct = { 0 };
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    if (!initialized) {
        /* Enable peripheral clocks */
        LWESP_USART_CLK;
        LWESP_USART_DMA_CLK;
        LWESP_USART_TX_PORT_CLK;
        LWESP_USART_RX_PORT_CLK;

#if defined(LWESP_RESET_PIN)
        LWESP_RESET_PORT_CLK;
#endif /* defined(LWESP_RESET_PIN) */

#if defined(LWESP_GPIO0_PIN)
        LWESP_GPIO0_PORT_CLK;
#endif /* defined(LWESP_GPIO0_PIN) */

#if defined(LWESP_GPIO2_PIN)
        LWESP_GPIO2_PORT_CLK;
#endif /* defined(LWESP_GPIO2_PIN) */

#if defined(LWESP_CH_PD_PIN)
        LWESP_CH_PD_PORT_CLK;
#endif /* defined(LWESP_CH_PD_PIN) */

#if defined(LWESP_RESET_PIN)
        /* Configure RESET pin */
        memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitTypeDef));
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Pin = LWESP_RESET_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(LWESP_RESET_PORT, &GPIO_InitStructure);
#endif /* defined(LWESP_RESET_PIN) */

#if defined(LWESP_GPIO0_PIN)
        /* Configure GPIO0 pin */
        gpio_init.Pin = LWESP_GPIO0_PIN;
        LL_GPIO_Init(LWESP_GPIO0_PORT, &gpio_init);
        LL_GPIO_SetOutputPin(LWESP_GPIO0_PORT, LWESP_GPIO0_PIN);
#endif /* defined(LWESP_GPIO0_PIN) */

#if defined(LWESP_GPIO2_PIN)
        /* Configure GPIO2 pin */
        gpio_init.Pin = LWESP_GPIO2_PIN;
        LL_GPIO_Init(LWESP_GPIO2_PORT, &gpio_init);
        LL_GPIO_SetOutputPin(LWESP_GPIO2_PORT, LWESP_GPIO2_PIN);
#endif /* defined(LWESP_GPIO2_PIN) */

#if defined(LWESP_CH_PD_PIN)
        /* Configure CH_PD pin */
        gpio_init.Pin = LWESP_CH_PD_PIN;
        LL_GPIO_Init(LWESP_CH_PD_PORT, &gpio_init);
        LL_GPIO_SetOutputPin(LWESP_CH_PD_PORT, LWESP_CH_PD_PIN);
#endif /* defined(LWESP_CH_PD_PIN) */

        /* Configure USART pins */
        /* TX PIN */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Pin = LWESP_USART_TX_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(LWESP_USART_TX_PORT, &GPIO_InitStructure);

        /* RX PIN */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Pin = LWESP_USART_RX_PIN;
        GPIO_Init(LWESP_USART_TX_PORT, &GPIO_InitStructure);

        /* Configure UART */
        USART_DeInit(LWESP_USART);
        USART_InitStruct.USART_BaudRate = baudrate;
        USART_InitStruct.USART_WordLength = USART_WordLength_8b;
        USART_InitStruct.USART_StopBits = USART_StopBits_1;
        USART_InitStruct.USART_Parity = USART_Parity_No;
        USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(LWESP_USART, &USART_InitStruct);

        /* Enable USART interrupts and DMA request */
        USART_ITConfig(LWESP_USART, USART_IT_IDLE, ENABLE);
        USART_ITConfig(LWESP_USART, USART_IT_PE, ENABLE);
        USART_ITConfig(LWESP_USART, USART_IT_ERR, ENABLE);
        USART_DMACmd(LWESP_USART, USART_DMAReq_Rx, ENABLE);

        /* Enable USART interrupts */
        memset(&NVIC_InitStructure, 0, sizeof(NVIC_InitTypeDef));
        NVIC_InitStructure.NVIC_IRQChannel = LWESP_USART_IRQ;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        /* Configure DMA */
        is_running = 0;
#if defined(LWESP_USART_DMA_RX_STREAM)
        LL_DMA_DeInit(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
        dma_init.Channel = LWESP_USART_DMA_RX_CH;
#else
        // DMA_DeInit(LWESP_USART_DMA_RX_CH);
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */

        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(LWESP_USART->LWESP_USART_RDR_NAME);
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)usart_mem;
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStruct.DMA_BufferSize = sizeof(usart_mem);
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
        DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;

#if defined(LWESP_USART_DMA_RX_STREAM)
        LL_DMA_Init(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM, &dma_init);
#else
        DMA_Init(LWESP_USART_DMA_RX_CH, &DMA_InitStruct);
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */

        /* Enable DMA interrupts */
#if defined(LWESP_USART_DMA_RX_STREAM)
        LL_DMA_EnableIT_HT(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
        LL_DMA_EnableIT_TC(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
        LL_DMA_EnableIT_TE(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
        LL_DMA_EnableIT_FE(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
        LL_DMA_EnableIT_DME(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
#else
        DMA_ITConfig(LWESP_USART_DMA_RX_CH, DMA_IT_HT, ENABLE);
        DMA_ITConfig(LWESP_USART_DMA_RX_CH, DMA_IT_TC, ENABLE);
        DMA_ITConfig(LWESP_USART_DMA_RX_CH, DMA_IT_TE, ENABLE);

#endif /* defined(LWESP_USART_DMA_RX_STREAM) */

        /* Enable DMA interrupts */
        memset(&NVIC_InitStructure, 0, sizeof(NVIC_InitTypeDef));
        NVIC_InitStructure.NVIC_IRQChannel = LWESP_USART_DMA_RX_IRQ;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        old_pos = 0;
        is_running = 1;

        /* Start DMA and USART */
#if defined(LWESP_USART_DMA_RX_STREAM)
        LL_DMA_EnableStream(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM);
#else
        DMA_Cmd(LWESP_USART_DMA_RX_CH, ENABLE);
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */
        USART_Cmd(LWESP_USART, ENABLE);

        /* Read from the USART_SR register followed by a write to the USART_DR register to clear TC flag */
        USART_GetFlagStatus(LWESP_USART, USART_FLAG_TC);
    } else {
        vTaskDelay(100);
        USART_Cmd(LWESP_USART, DISABLE);
        USART_InitStruct.USART_BaudRate = baudrate;
        USART_Init(LWESP_USART, &USART_InitStruct);
        USART_Cmd(LWESP_USART, ENABLE);

        /* Read from the USART_SR register followed by a write to the USART_DR register to clear TC flag */
        USART_GetFlagStatus(LWESP_USART, USART_FLAG_TC);
    }

    /* Create mbox and start thread */
    if (usart_ll_mbox_id == NULL) {
        usart_ll_mbox_id = xQueueCreate(10, sizeof(void*));
    }
    if (usart_ll_thread_id == NULL) {
        xTaskCreate(usart_ll_thread, "usart_ll_thread", 1024, NULL, 10, &usart_ll_thread_id);
    }
}

#if defined(LWESP_RESET_PIN)
/**
 * \brief           Hardware reset callback
 */
static uint8_t
reset_device(uint8_t state) {
    if (state) {
        /* Activate reset line */
        GPIO_ResetBits(LWESP_RESET_PORT, LWESP_RESET_PIN);
    } else {
        GPIO_SetBits(LWESP_RESET_PORT, LWESP_RESET_PIN);
    }
    return 1;
}
#endif /* defined(LWESP_RESET_PIN) */

/**
 * \brief           Send data to ESP device
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t
send_data(const void* data, size_t len) {
    const uint8_t* d = data;

    for (size_t i = 0; i < len; ++i, ++d) {
        USART_SendData(LWESP_USART, (uint8_t)(*d));
        while (USART_GetFlagStatus(LWESP_USART, USART_FLAG_TC) == RESET) {}
    }
    return len;
}

/**
 * \brief           Callback function called from initialization process
 */
lwespr_t
lwesp_ll_init(lwesp_ll_t* ll) {
#if !LWESP_CFG_MEM_CUSTOM
    static uint8_t memory[LWESP_MEM_SIZE];
    lwesp_mem_region_t mem_regions[] = {
        { memory, sizeof(memory) }
    };

    if (!initialized) {
        lwesp_mem_assignmemory(mem_regions, LWESP_ARRAYSIZE(mem_regions));  /* Assign memory for allocations */
    }
#endif /* !LWESP_CFG_MEM_CUSTOM */

    if (!initialized) {
        ll->send_fn = send_data;                /* Set callback function to send data */
#if defined(LWESP_RESET_PIN)
        ll->reset_fn = reset_device;            /* Set callback for hardware reset */
#endif /* defined(LWESP_RESET_PIN) */
    }

    configure_uart(ll->uart.baudrate);          /* Initialize UART for communication */
    initialized = 1;
    return lwespOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 */
lwespr_t
lwesp_ll_deinit(lwesp_ll_t* ll) {
    if (usart_ll_mbox_id != NULL) {
        QueueHandle_t tmp = usart_ll_mbox_id;
        usart_ll_mbox_id = NULL;
        vQueueDelete(tmp);
    }
    if (usart_ll_thread_id != NULL) {
        TaskHandle_t tmp = usart_ll_thread_id;
        usart_ll_thread_id = NULL;
        vTaskDelete(tmp);
    }
    initialized = 0;
    LWESP_UNUSED(ll);
    return lwespOK;
}

/**
 * \brief           UART global interrupt handler
 */
void
LWESP_USART_IRQHANDLER(void) {
    USART_ClearFlag(LWESP_USART, USART_IT_PE);
    USART_ClearFlag(LWESP_USART, USART_IT_FE);
    USART_ClearFlag(LWESP_USART, USART_IT_ORE_ER);
    USART_ClearFlag(LWESP_USART, USART_IT_NE);

    if (USART_GetITStatus(LWESP_USART, USART_IT_IDLE) != RESET) {
        /* Clear IDLE bit */
        USART_ReceiveData(LWESP_USART);
        USART_ClearITPendingBit(LWESP_USART, USART_IT_IDLE);
    }

    if (usart_ll_mbox_id != NULL) {
        void* d = (void*)1;
        xQueueSendToBackFromISR(usart_ll_mbox_id, &d, NULL);
    }
}

/**
 * \brief           UART DMA stream/channel handler
 */
void
LWESP_USART_DMA_RX_IRQHANDLER(void) {
    LWESP_USART_DMA_RX_CLEAR_TC;
    LWESP_USART_DMA_RX_CLEAR_HT;

    if (usart_ll_mbox_id != NULL) {
        void* d = (void*)1;
        xQueueSendToBackFromISR(usart_ll_mbox_id, &d, NULL);
    }
}

#endif /* !__DOXYGEN__ */
