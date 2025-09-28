/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: can_cfg.h
 * Author: hoangphuc540202@gmail.com
 * Date: 22/9/2025
 * 
 * Description: Comprehensive CAN module with Data Frame, Remote Frame, 
 *              and Error Frame handling
 * =============================================================================
 */


#ifndef __CAN_CFG_H
#define __CAN_CFG_H


/* ======================= Configuration Parameters ======================= */

// GPIO Pin Definitions

#define CAN_PORT          PIOA
#define CAN_RX_PIN        GPIO_Pin_11             // PA11 - CAN RX
#define CAN_TX_PIN        GPIO_Pin_12             // PA12 - CAN TX
#define CAN_PERIPH        CAN1


#define UART_CLK          RCC_APB2Periph_USART1
#define UART_GPIO_CLK     RCC_APB2Periph_GPIOA



// CAN Configuration
#define CAN_MODE_DEFAULT            CAN_Mode_Normal         // Default CAN mode
#define CAN_PRESCALER_DEFAULT       9                       // Default bitrate prescaler for 400kbps


#define CAN_BAUDRATE                500000                  // CAN baud rate 
#define CAN_MSG_ID                  0x123                   // Standard CAN message ID
#define CAN_MSG_DLC                 8                       // Data Length Code

// Interrupt Priorities
#define CAN_IRQ_PRIORITY            0                       // CAN interrupt priority


// Buffer Sizes
#define CAN_MAX_TX_BUFFER_SIZE      10                      // Max size of TX buffer
#define CAN_MAX_RX_BUFFER_SIZE      8                       // Max size of RX buffer



#endif /* __CAN_CFG_H */