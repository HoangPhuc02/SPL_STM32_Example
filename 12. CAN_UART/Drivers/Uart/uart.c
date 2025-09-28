/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: uart.c
 * Author: hoangphuc540202@gmail.com
 * Date: 22/9/2025
 *
 * Description: Comprehensive UART module with transmission and reception handling
 * Support: DMA, Interrupt, Polling modes
 * =============================================================================
 */

#include "uart.h"
#include "systick.h"
#include <stdarg.h>
#include <stdio.h>

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/

/* UART Handle */
UART_HandleTypeDef huart1;

/* Buffers */
uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

/* Status flags */
volatile uint8_t uart_tx_complete = 1;
volatile uint8_t uart_rx_complete = 1;
volatile uint8_t uart_dma_busy = 0;

/* Printf buffer */
static char printf_buffer[UART_PRINTF_BUFFER_SIZE];

/*==================================================================================================
*                                    PRIVATE FUNCTIONS
==================================================================================================*/

/**
 * @brief Get system tick count (simple counter)
 * @return Current tick count
 */
static inline uint32_t UART_GetTick(void)
{
    return get_system_ms();
}

/**
 * @brief Simple delay function
 * @param ms: Delay in milliseconds
 */
static void UART_Delay(uint32_t ms)
{
    delay_ms(ms);
}

/*==================================================================================================
*                                 INITIALIZATION FUNCTIONS
==================================================================================================*/

/**
 * @brief Configure UART hardware (GPIO, Clock, USART)
 */
void UART_Config(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* Enable peripheral clocks */
    RCC_APB2PeriphClockCmd(UART_GPIO_CLK, ENABLE);
    
    #if defined(USART1)
    if(huart->Instance == USART1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    }
    #endif
    
    #if defined(USART2)
    if(huart->Instance == USART2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    }
    #endif
    
    #if defined(USART3)
    if(huart->Instance == USART3)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    }
    #endif

    /* Configure UART pins: TX (Alternate Function Push-Pull) */
    GPIO_InitStructure.GPIO_Pin = UART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(UART_PORT, &GPIO_InitStructure);

    /* Configure UART pins: RX (Input Floating) */
    GPIO_InitStructure.GPIO_Pin = UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_PORT, &GPIO_InitStructure);

    /* USART configuration */
    USART_InitStructure.USART_BaudRate = huart->BaudRate;
    USART_InitStructure.USART_WordLength = huart->WordLength;
    USART_InitStructure.USART_StopBits = huart->StopBits;
    USART_InitStructure.USART_Parity = huart->Parity;
    USART_InitStructure.USART_HardwareFlowControl = huart->HwFlowCtl;
    USART_InitStructure.USART_Mode = huart->Mode;
    USART_Init(huart->Instance, &USART_InitStructure);
    
    /* Enable USART */
    USART_Cmd(huart->Instance, ENABLE);
    
    /* Initialize state */
    huart->State = UART_STATUS_READY;
    huart->ErrorCode = UART_ERROR_NONE;
}

/**
 * @brief Configure UART interrupt
 */
void UART_ITR_Config(UART_HandleTypeDef *huart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure USART interrupt */
    if(huart->Instance == USART1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    }
    else if(huart->Instance == USART2)
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    }
    else if(huart->Instance == USART3)
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    }
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART_ITR_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Configure UART DMA
 */
void UART_DMA_Config(UART_HandleTypeDef *huart)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* Enable DMA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    /* Configure DMA interrupt for UART1 */
    if(huart->Instance == USART1)
    {
        /* TX DMA interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = UART1_DMA_TX_IRQ;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART_ITR_DMA_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        /* RX DMA interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = UART1_DMA_RX_IRQ;
        NVIC_Init(&NVIC_InitStructure);
        
        /* Store DMA channels */
        huart->hdmatx = UART1_DMA_TX_PERIPH;
        huart->hdmarx = UART1_DMA_RX_PERIPH;
    }
    
    /* Enable USART DMA requests */
    USART_DMACmd(huart->Instance, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(huart->Instance, USART_DMAReq_Rx, ENABLE);
}

/**
 * @brief Initialize UART with specific mode
 */
void UART_Init(UART_HandleTypeDef *huart, uint8_t mode)
{
    /* Set default parameters if not configured */
    if(huart->BaudRate == 0) huart->BaudRate = UART_BAUDRATE;
    if(huart->WordLength == 0) huart->WordLength = USART_WordLength_8b;
    if(huart->StopBits == 0) huart->StopBits = USART_StopBits_1;
    if(huart->Parity == 0) huart->Parity = USART_Parity_No;
    if(huart->Mode == 0) huart->Mode = USART_Mode_Rx | USART_Mode_Tx;
    if(huart->HwFlowCtl == 0) huart->HwFlowCtl = USART_HardwareFlowControl_None;
    
    /* Configure hardware */
    UART_Config(huart);
    
    /* Configure based on mode */
    switch(mode)
    {
        case UART_MODE_INTERRUPT:
            UART_ITR_Config(huart);
            break;
            
        case UART_MODE_DMA:
            UART_DMA_Config(huart);
            break;
            
        case UART_MODE_POLLING:
        default:
            /* No additional configuration needed for polling */
            break;
    }
}

/*==================================================================================================
*                                  TRANSMISSION FUNCTIONS
==================================================================================================*/

/**
 * @brief Transmit data in polling mode
 */
uint8_t UART_Transmit_Polling(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = UART_GetTick();
    uint16_t i;
    
    if(huart->State != UART_STATUS_READY)
    {
        return 1; /* Busy */
    }
    
    huart->State = UART_STATUS_BUSY_TX;
    
    for(i = 0; i < Size; i++)
    {
        /* Wait for TXE flag */
        while(USART_GetFlagStatus(huart->Instance, USART_FLAG_TXE) == RESET)
        {
            if((UART_GetTick() - tickstart) > Timeout)
            {
                huart->State = UART_STATUS_READY;
                return 1; /* Timeout */
            }
        }
        
        /* Send data */
        USART_SendData(huart->Instance, pData[i]);
    }
    
    /* Wait for TC flag */
    while(USART_GetFlagStatus(huart->Instance, USART_FLAG_TC) == RESET)
    {
        if((UART_GetTick() - tickstart) > Timeout)
        {
            huart->State = UART_STATUS_READY;
            return 1; /* Timeout */
        }
    }
    
    huart->State = UART_STATUS_READY;
    return 0; /* Success */
}

/**
 * @brief Transmit data in interrupt mode
 */
uint8_t UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if(huart->State != UART_STATUS_READY)
    {
        return 1; /* Busy */
    }
    
    huart->State = UART_STATUS_BUSY_TX;
    huart->pTxBuffPtr = pData;
    huart->TxXferSize = Size;
    huart->TxXferCount = 0;
    
    uart_tx_complete = 0;
    
    /* Enable TXE interrupt */
    USART_ITConfig(huart->Instance, USART_IT_TXE, ENABLE);
    
    return 0; /* Success */
}

/**
 * @brief Transmit data in DMA mode
 */
uint8_t UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    if(huart->State != UART_STATUS_READY)
    {
        return 1; /* Busy */
    }
    
    huart->State = UART_STATUS_BUSY_TX;
    uart_dma_busy = 1;
    uart_tx_complete = 0;
    
    /* Configure DMA for transmission */
    DMA_DeInit(huart->hdmatx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&huart->Instance->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = Size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(huart->hdmatx, &DMA_InitStructure);
    
    /* Enable DMA transfer complete interrupt */
    DMA_ITConfig(huart->hdmatx, DMA_IT_TC, ENABLE);
    
    /* Enable DMA channel */
    DMA_Cmd(huart->hdmatx, ENABLE);
    
    return 0; /* Success */
}

/*==================================================================================================
*                                   RECEPTION FUNCTIONS
==================================================================================================*/

/**
 * @brief Receive data in polling mode
 */
uint8_t UART_Receive_Polling(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = UART_GetTick();
    uint16_t i;
    
    if(huart->State != UART_STATUS_READY)
    {
        return 1; /* Busy */
    }
    
    huart->State = UART_STATUS_BUSY_RX;
    
    for(i = 0; i < Size; i++)
    {
        /* Wait for RXNE flag */
        while(USART_GetFlagStatus(huart->Instance, USART_FLAG_RXNE) == RESET)
        {
            if((UART_GetTick() - tickstart) > Timeout)
            {
                huart->State = UART_STATUS_READY;
                return 1; /* Timeout */
            }
        }
        
        /* Receive data */
        pData[i] = USART_ReceiveData(huart->Instance);
    }
    
    huart->State = UART_STATUS_READY;
    return 0; /* Success */
}

/**
 * @brief Receive data in interrupt mode
 */
uint8_t UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if(huart->State != UART_STATUS_READY)
    {
        return 1; /* Busy */
    }
    
    huart->State = UART_STATUS_BUSY_RX;
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->RxXferCount = 0;
    
    uart_rx_complete = 0;
    
    /* Enable RXNE interrupt */
    USART_ITConfig(huart->Instance, USART_IT_RXNE, ENABLE);
    
    return 0; /* Success */
}

/**
 * @brief Receive data in DMA mode
 */
uint8_t UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    if(huart->State != UART_STATUS_READY)
    {
        return 1; /* Busy */
    }
    
    huart->State = UART_STATUS_BUSY_RX;
    uart_rx_complete = 0;
    
    /* Configure DMA for reception */
    DMA_DeInit(huart->hdmarx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&huart->Instance->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = Size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(huart->hdmarx, &DMA_InitStructure);
    
    /* Enable DMA transfer complete interrupt */
    DMA_ITConfig(huart->hdmarx, DMA_IT_TC, ENABLE);
    
    /* Enable DMA channel */
    DMA_Cmd(huart->hdmarx, ENABLE);
    
    return 0; /* Success */
}

/*==================================================================================================
*                                    UTILITY FUNCTIONS
==================================================================================================*/

/**
 * @brief Send a single character (polling)
 */
void UART_SendChar(UART_HandleTypeDef *huart, char ch)
{
    while(USART_GetFlagStatus(huart->Instance, USART_FLAG_TXE) == RESET);
    USART_SendData(huart->Instance, (uint16_t)ch);
    while(USART_GetFlagStatus(huart->Instance, USART_FLAG_TC) == RESET);
}

/**
 * @brief Send a string (polling)
 */
void UART_SendString(UART_HandleTypeDef *huart, const char* str)
{
    while(*str)
    {
        UART_SendChar(huart, *str++);
    }
}

/**
 * @brief Send formatted string (polling)
 */
void UART_Printf(UART_HandleTypeDef *huart, const char* format, ...)
{
    va_list args;
    int len;
    
    va_start(args, format);
    len = vsnprintf(printf_buffer, sizeof(printf_buffer), format, args);
    va_end(args);
    
    if(len > 0)
    {
        UART_Transmit_Polling(huart, (uint8_t*)printf_buffer, len, 1000);
    }
}

/**
 * @brief Receive a single character (polling)
 */
char UART_ReceiveChar(UART_HandleTypeDef *huart)
{
    while(USART_GetFlagStatus(huart->Instance, USART_FLAG_RXNE) == RESET);
    return (char)USART_ReceiveData(huart->Instance);
}

/**
 * @brief Get UART state
 */
uint32_t UART_GetState(UART_HandleTypeDef *huart)
{
    return huart->State;
}

/**
 * @brief Get UART error
 */
uint32_t UART_GetError(UART_HandleTypeDef *huart)
{
    return huart->ErrorCode;
}

/*==================================================================================================
*                                   CALLBACK FUNCTIONS
==================================================================================================*/

/**
 * @brief Tx Transfer completed callback (weak implementation)
 */
__weak void UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* User can implement this function */
    uart_tx_complete = 1;
    huart->State = UART_STATUS_READY;
}

/**
 * @brief Rx Transfer completed callback (weak implementation)
 */
__weak void UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* User can implement this function */
    uart_rx_complete = 1;
    huart->State = UART_STATUS_READY;
}

/**
 * @brief UART error callback (weak implementation)
 */
__weak void UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* User can implement this function */
    huart->State = UART_STATUS_ERROR;
}

/*==================================================================================================
*                                  INTERRUPT HANDLERS
==================================================================================================*/

/**
 * @brief Handle UART interrupt request
 */
void UART_IRQHandler(UART_HandleTypeDef *huart)
{
    uint32_t isrflags   = READ_REG(huart->Instance->SR);
    uint32_t cr1its     = READ_REG(huart->Instance->CR1);
    uint32_t errorflags = 0x00U;
    
    /* Check for errors */
    errorflags = (isrflags & (uint32_t)(USART_FLAG_PE | USART_FLAG_FE | USART_FLAG_ORE | USART_FLAG_NE));
    if(errorflags != RESET)
    {
        /* Handle errors */
        if((isrflags & USART_FLAG_PE) != RESET)
        {
            huart->ErrorCode |= UART_ERROR_PARITY;
        }
        if((isrflags & USART_FLAG_FE) != RESET)
        {
            huart->ErrorCode |= UART_ERROR_FRAME;
        }
        if((isrflags & USART_FLAG_ORE) != RESET)
        {
            huart->ErrorCode |= UART_ERROR_OVERRUN;
        }
        if((isrflags & USART_FLAG_NE) != RESET)
        {
            huart->ErrorCode |= UART_ERROR_NOISE;
        }
        
        /* Call error callback */
        UART_ErrorCallback(huart);
        return;
    }
    
    /* Handle RXNE interrupt */
    if(((isrflags & USART_FLAG_RXNE) != RESET) && ((cr1its & USART_IT_RXNE) != RESET))
    {
        if(huart->State == UART_STATUS_BUSY_RX)
        {
            huart->pRxBuffPtr[huart->RxXferCount++] = (uint8_t)USART_ReceiveData(huart->Instance);
            
            if(huart->RxXferCount >= huart->RxXferSize)
            {
                /* Disable RXNE interrupt */
                USART_ITConfig(huart->Instance, USART_IT_RXNE, DISABLE);
                
                /* Call completion callback */
                UART_RxCpltCallback(huart);
            }
        }
    }
    
    /* Handle TXE interrupt */
    if(((isrflags & USART_FLAG_TXE) != RESET) && ((cr1its & USART_IT_TXE) != RESET))
    {
        if(huart->State == UART_STATUS_BUSY_TX)
        {
            if(huart->TxXferCount < huart->TxXferSize)
            {
                USART_SendData(huart->Instance, (uint16_t)huart->pTxBuffPtr[huart->TxXferCount++]);
            }
            else
            {
                /* Disable TXE interrupt */
                USART_ITConfig(huart->Instance, USART_IT_TXE, DISABLE);
                
                /* Enable TC interrupt for completion */
                USART_ITConfig(huart->Instance, USART_IT_TC, ENABLE);
            }
        }
    }
    
    /* Handle TC interrupt */
    if(((isrflags & USART_FLAG_TC) != RESET) && ((cr1its & USART_IT_TC) != RESET))
    {
        /* Disable TC interrupt */
        USART_ITConfig(huart->Instance, USART_IT_TC, DISABLE);
        
        /* Call completion callback */
        UART_TxCpltCallback(huart);
    }
}

/**
 * @brief Handle UART DMA Tx interrupt request
 */
void UART_DMA_TxIRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        if(DMA_GetITStatus(UART1_DMA_TX_FLAG_TC) != RESET)
        {
            DMA_ClearITPendingBit(UART1_DMA_TX_FLAG_TC);
            DMA_Cmd(huart->hdmatx, DISABLE);
            uart_dma_busy = 0;
            UART_TxCpltCallback(huart);
        }
    }
}

/**
 * @brief Handle UART DMA Rx interrupt request
 */
void UART_DMA_RxIRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        if(DMA_GetITStatus(UART1_DMA_RX_FLAG_TC) != RESET)
        {
            DMA_ClearITPendingBit(UART1_DMA_RX_FLAG_TC);
            DMA_Cmd(huart->hdmarx, DISABLE);
            UART_RxCpltCallback(huart);
        }
    }
}

/*==================================================================================================
*                                  INTERRUPT SERVICE ROUTINES
==================================================================================================*/

/**
 * @brief USART1 IRQ Handler
 */
void USART1_IRQHandler(void)
{
    UART_IRQHandler(&huart1);
}

/**
 * @brief DMA1 Channel 4 IRQ Handler (USART1 TX)
 */
void DMA1_Channel4_IRQHandler(void)
{
    UART_DMA_TxIRQHandler(&huart1);
}

/**
 * @brief DMA1 Channel 5 IRQ Handler (USART1 RX)
 */
void DMA1_Channel5_IRQHandler(void)
{
    UART_DMA_RxIRQHandler(&huart1);
}
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    // Enable DMA transfer complete interrupt
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    // Start the transmission
    uart_dma_busy = 1;
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

void UART_SendData(uint8_t* data, uint16_t length)
{
    if (length > UART_TX_BUFFER_SIZE || uart_dma_busy)
        return; // Invalid length or DMA busy

    // Copy data to transmission buffer
    for (uint16_t i = 0; i < length; i++)
    {
        uart_tx_buffer[i] = data[i];
    }

}

void UART_ReceiveData(uint8_t* data)
{

}