/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: uart.h
 * Author: hoangphuc540202@gmail.com
 * Date: 22/9/2025
 * 
 * Description: Comprehensive UART module with transmission and reception handling
 * Support: DMA, Interrupt, Polling modes
 * =============================================================================
 */

#ifndef __UART_H
#define __UART_H

#include "uart_cfg.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include <stdint.h>
#include <string.h>

/*==================================================================================================
*                                        DEFINES AND MACROS
==================================================================================================*/

/* UART Transmission Modes */
#define UART_MODE_POLLING       0x00U
#define UART_MODE_INTERRUPT     0x01U
#define UART_MODE_DMA           0x02U

/* UART Status */
#define UART_STATUS_READY       0x00U
#define UART_STATUS_BUSY_TX     0x01U
#define UART_STATUS_BUSY_RX     0x02U
#define UART_STATUS_ERROR       0x03U

/* Error Codes */
#define UART_ERROR_NONE         0x00U
#define UART_ERROR_PARITY       0x01U
#define UART_ERROR_NOISE        0x02U
#define UART_ERROR_FRAME        0x04U
#define UART_ERROR_OVERRUN      0x08U
#define UART_ERROR_DMA          0x10U

/*==================================================================================================
*                                 STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

/**
 * @brief UART Handle Structure
 */
typedef struct
{
    USART_TypeDef*         Instance;          /* UART registers base address        */
    uint32_t               BaudRate;          /* Baud rate                           */
    uint32_t               WordLength;        /* Word length                         */
    uint32_t               StopBits;          /* Stop bits                           */
    uint32_t               Parity;            /* Parity                              */
    uint32_t               Mode;              /* UART communication mode            */
    uint32_t               HwFlowCtl;         /* Hardware flow control              */
    uint32_t               OverSampling;      /* Oversampling                        */
    
    uint8_t*               pTxBuffPtr;        /* Pointer to UART Tx transfer Buffer */
    uint16_t               TxXferSize;        /* UART Tx Transfer size               */
    uint16_t               TxXferCount;       /* UART Tx Transfer Counter            */
    
    uint8_t*               pRxBuffPtr;        /* Pointer to UART Rx transfer Buffer */
    uint16_t               RxXferSize;        /* UART Rx Transfer size               */
    uint16_t               RxXferCount;       /* UART Rx Transfer Counter            */
    
    DMA_Channel_TypeDef*   hdmatx;            /* UART Tx DMA Handle                  */
    DMA_Channel_TypeDef*   hdmarx;            /* UART Rx DMA Handle                  */
    
    volatile uint32_t      State;             /* UART communication state           */
    volatile uint32_t      ErrorCode;         /* UART Error code                     */
    
} UART_HandleTypeDef;

/**
 * @brief UART Callback Function Type
 */
typedef void (*UART_CallbackTypeDef)(UART_HandleTypeDef *huart);

/*==================================================================================================
*                                 GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

extern UART_HandleTypeDef huart1;
extern uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
extern uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern volatile uint8_t uart_tx_complete;
extern volatile uint8_t uart_rx_complete;
extern volatile uint8_t uart_dma_busy;

/*==================================================================================================
*                                     FUNCTION PROTOTYPES
==================================================================================================*/

/* ============================= Initialization Functions ============================= */

/**
 * @brief Configure UART hardware (GPIO, Clock, USART)
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_Config(UART_HandleTypeDef *huart);

/**
 * @brief Configure UART interrupt
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_ITR_Config(UART_HandleTypeDef *huart);

/**
 * @brief Configure UART DMA
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_DMA_Config(UART_HandleTypeDef *huart);

/**
 * @brief Initialize UART with specific mode
 * @param huart: Pointer to UART handle
 * @param mode: Transmission mode (POLLING/INTERRUPT/DMA)
 * @return None
 */
void UART_Init(UART_HandleTypeDef *huart, uint8_t mode);

/* ============================= Transmission Functions ============================= */

/**
 * @brief Transmit data in polling mode
 * @param huart: Pointer to UART handle
 * @param pData: Pointer to data buffer
 * @param Size: Amount of data to be sent
 * @param Timeout: Timeout duration
 * @return Status (0: OK, 1: Error)
 */
uint8_t UART_Transmit_Polling(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief Transmit data in interrupt mode
 * @param huart: Pointer to UART handle
 * @param pData: Pointer to data buffer
 * @param Size: Amount of data to be sent
 * @return Status (0: OK, 1: Error)
 */
uint8_t UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief Transmit data in DMA mode
 * @param huart: Pointer to UART handle
 * @param pData: Pointer to data buffer
 * @param Size: Amount of data to be sent
 * @return Status (0: OK, 1: Error)
 */
uint8_t UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/* ============================= Reception Functions ============================= */

/**
 * @brief Receive data in polling mode
 * @param huart: Pointer to UART handle
 * @param pData: Pointer to data buffer
 * @param Size: Amount of data to be received
 * @param Timeout: Timeout duration
 * @return Status (0: OK, 1: Error)
 */
uint8_t UART_Receive_Polling(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief Receive data in interrupt mode
 * @param huart: Pointer to UART handle
 * @param pData: Pointer to data buffer
 * @param Size: Amount of data to be received
 * @return Status (0: OK, 1: Error)
 */
uint8_t UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief Receive data in DMA mode
 * @param huart: Pointer to UART handle
 * @param pData: Pointer to data buffer
 * @param Size: Amount of data to be received
 * @return Status (0: OK, 1: Error)
 */
uint8_t UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/* ============================= Utility Functions ============================= */

/**
 * @brief Send a single character (polling)
 * @param huart: Pointer to UART handle
 * @param ch: Character to send
 * @return None
 */
void UART_SendChar(UART_HandleTypeDef *huart, char ch);

/**
 * @brief Send a string (polling)
 * @param huart: Pointer to UART handle
 * @param str: String to send
 * @return None
 */
void UART_SendString(UART_HandleTypeDef *huart, const char* str);

/**
 * @brief Send formatted string (polling)
 * @param huart: Pointer to UART handle
 * @param format: Format string
 * @param ...: Variable arguments
 * @return None
 */
void UART_Printf(UART_HandleTypeDef *huart, const char* format, ...);

/**
 * @brief Receive a single character (polling)
 * @param huart: Pointer to UART handle
 * @return Received character
 */
char UART_ReceiveChar(UART_HandleTypeDef *huart);

/**
 * @brief Get UART state
 * @param huart: Pointer to UART handle
 * @return UART state
 */
uint32_t UART_GetState(UART_HandleTypeDef *huart);

/**
 * @brief Get UART error
 * @param huart: Pointer to UART handle
 * @return Error code
 */
uint32_t UART_GetError(UART_HandleTypeDef *huart);

/* ============================= Callback Functions ============================= */

/**
 * @brief Tx Transfer completed callback
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief Rx Transfer completed callback
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief UART error callback
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_ErrorCallback(UART_HandleTypeDef *huart);

/* ============================= Interrupt Handlers ============================= */

/**
 * @brief Handle UART interrupt request
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_IRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief Handle UART DMA Tx interrupt request
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_DMA_TxIRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief Handle UART DMA Rx interrupt request
 * @param huart: Pointer to UART handle
 * @return None
 */
void UART_DMA_RxIRQHandler(UART_HandleTypeDef *huart);

#endif /* __UART_H */