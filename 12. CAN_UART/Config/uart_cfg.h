/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: uart_cfg.h
 * Author: hoangphuc540202@gmail.com
 * Date: 22/9/2025
 *
 * Description: Comprehensive UART module with various configurations
 * Support: Multiple UART instances, DMA, Interrupt, Polling modes
 * =============================================================================
 */

#ifndef __UART_CONFIG_H
#define __UART_CONFIG_H

#include "stm32f10x.h"

/*==================================================================================================
*                                   UART HARDWARE CONFIGURATION
==================================================================================================*/

/* === UART1 Configuration === */
#define UART1_ENABLE              1
#define UART1_PORT               GPIOA
#define UART1_TX_PIN             GPIO_Pin_9
#define UART1_RX_PIN             GPIO_Pin_10
#define UART1_PERIPH             USART1
#define UART1_CLK                RCC_APB2Periph_USART1
#define UART1_GPIO_CLK           RCC_APB2Periph_GPIOA
#define UART1_BAUDRATE           115200
#define UART1_IRQ                USART1_IRQn

/* === UART2 Configuration === */
#define UART2_ENABLE              0    /* Set to 1 to enable UART2 */
#define UART2_PORT               GPIOA
#define UART2_TX_PIN             GPIO_Pin_2
#define UART2_RX_PIN             GPIO_Pin_3
#define UART2_PERIPH             USART2
#define UART2_CLK                RCC_APB1Periph_USART2
#define UART2_GPIO_CLK           RCC_APB2Periph_GPIOA
#define UART2_BAUDRATE           9600
#define UART2_IRQ                USART2_IRQn

/* === UART3 Configuration === */
#define UART3_ENABLE              0    /* Set to 1 to enable UART3 */
#define UART3_PORT               GPIOB
#define UART3_TX_PIN             GPIO_Pin_10
#define UART3_RX_PIN             GPIO_Pin_11
#define UART3_PERIPH             USART3
#define UART3_CLK                RCC_APB1Periph_USART3
#define UART3_GPIO_CLK           RCC_APB2Periph_GPIOB
#define UART3_BAUDRATE           9600
#define UART3_IRQ                USART3_IRQn

/* === Default UART (Backward compatibility) === */
#define UART_PORT                UART1_PORT
#define UART_TX_PIN              UART1_TX_PIN
#define UART_RX_PIN              UART1_RX_PIN
#define UART_PERIPH              UART1_PERIPH
#define UART_CLK                 UART1_CLK
#define UART_GPIO_CLK            UART1_GPIO_CLK
#define UART_BAUDRATE            UART1_BAUDRATE
#define UART_IRQ                 UART1_IRQ

/*==================================================================================================
*                                      DMA CONFIGURATION
==================================================================================================*/

/* === UART1 DMA Configuration === */
#define UART1_DMA_TX_PERIPH      DMA1_Channel4
#define UART1_DMA_RX_PERIPH      DMA1_Channel5
#define UART1_DMA_TX_IRQ         DMA1_Channel4_IRQn
#define UART1_DMA_RX_IRQ         DMA1_Channel5_IRQn
#define UART1_DMA_TX_FLAG_TC     DMA1_FLAG_TC4
#define UART1_DMA_RX_FLAG_TC     DMA1_FLAG_TC5

/* === UART2 DMA Configuration === */
#define UART2_DMA_TX_PERIPH      DMA1_Channel7
#define UART2_DMA_RX_PERIPH      DMA1_Channel6
#define UART2_DMA_TX_IRQ         DMA1_Channel7_IRQn
#define UART2_DMA_RX_IRQ         DMA1_Channel6_IRQn
#define UART2_DMA_TX_FLAG_TC     DMA1_FLAG_TC7
#define UART2_DMA_RX_FLAG_TC     DMA1_FLAG_TC6

/* === UART3 DMA Configuration === */
#define UART3_DMA_TX_PERIPH      DMA1_Channel2
#define UART3_DMA_RX_PERIPH      DMA1_Channel3
#define UART3_DMA_TX_IRQ         DMA1_Channel2_IRQn
#define UART3_DMA_RX_IRQ         DMA1_Channel3_IRQn
#define UART3_DMA_TX_FLAG_TC     DMA1_FLAG_TC2
#define UART3_DMA_RX_FLAG_TC     DMA1_FLAG_TC3

/* === Default DMA (Backward compatibility) === */
#define UART_DMA_PERIPH          UART1_DMA_TX_PERIPH
#define UART_DMA_IRQ             UART1_DMA_TX_IRQ

/*==================================================================================================
*                                   INTERRUPT CONFIGURATION
==================================================================================================*/

#define UART_ITR_PRIORITY        1
#define UART_ITR_DMA_PRIORITY    1

/* Enable/Disable specific interrupts */
#define UART_ENABLE_TXE_IT       1    /* Transmit Data Register Empty interrupt */
#define UART_ENABLE_TC_IT        1    /* Transmission Complete interrupt */
#define UART_ENABLE_RXNE_IT      1    /* Read Data Register Not Empty interrupt */
#define UART_ENABLE_IDLE_IT      0    /* IDLE Line Detection interrupt */
#define UART_ENABLE_PE_IT        1    /* Parity Error interrupt */
#define UART_ENABLE_ERR_IT       1    /* Error interrupt (Frame, Noise, Overrun) */

/*==================================================================================================
*                                     BUFFER CONFIGURATION
==================================================================================================*/

#define UART_TX_BUFFER_SIZE      256
#define UART_RX_BUFFER_SIZE      256

/* Circular buffer enable */
#define UART_USE_CIRCULAR_BUFFER 1

/* Printf buffer size */
#define UART_PRINTF_BUFFER_SIZE  128

/*==================================================================================================
*                                      MODE CONFIGURATION
==================================================================================================*/

/* Default transmission mode */
#define UART_DEFAULT_MODE        UART_MODE_POLLING    /* UART_MODE_POLLING / UART_MODE_INTERRUPT / UART_MODE_DMA */

/* Enable DMA mode */
#define UART_DMA_ENABLE          1

/* Enable Interrupt mode */
#define UART_IT_ENABLE           1

/* Enable polling timeout */
#define UART_POLLING_TIMEOUT     1000  /* milliseconds */

/*==================================================================================================
*                                    FEATURE CONFIGURATION
==================================================================================================*/

/* Enable hardware flow control */
#define UART_HW_FLOW_CONTROL     0

/* Enable parity checking */
#define UART_PARITY_ENABLE       0

/* Enable 9-bit data mode */
#define UART_9BIT_MODE           0

/* Enable multi-processor communication */
#define UART_MULTIPROCESSOR      0

/* Enable wake-up from stop mode */
#define UART_WAKEUP_ENABLE       0

/* Enable auto baud rate detection */
#define UART_AUTO_BAUDRATE       0

/* Enable transmitter/receiver */
#define UART_TX_ENABLE           1
#define UART_RX_ENABLE           1

/*==================================================================================================
*                                      DEBUG CONFIGURATION
==================================================================================================*/

/* Enable debug mode */
#define UART_DEBUG_ENABLE        1

/* Debug UART instance (for printf redirection) */
#define UART_DEBUG_INSTANCE      UART1_PERIPH

/* Enable error checking */
#define UART_ERROR_CHECK         1

/* Enable statistics */
#define UART_STATISTICS          0

/*==================================================================================================
*                                    COMPATIBILITY DEFINES
==================================================================================================*/

/* Backward compatibility */
#define UARTCLK                  UART_CLK
#define UART_DEBUG_PRIORITY      UART_ITR_PRIORITY

#endif /* __UART_CONFIG_H */