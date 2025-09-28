/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: config.h
 * Author: hoangphuc540202@gmail.com
 * Date: 22/9/2025
 * 
 * Description: Configuration file containing all peripheral setup functions
 *              separated from main.c for better code organization
 * =============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Include standard integer types */
#include <stdint.h>
#include "can_cfg.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "system_stm32f10x.h"
#include "misc.h"

/* ======================= Configuration Parameters ======================= */


// UART Configuration  
#define UART_BAUDRATE               115200                  // UART baud rate

// GPIO Pin Definitions
#define CAN_RX_PIN                  GPIO_Pin_11             // PA11 - CAN RX
#define CAN_TX_PIN                  GPIO_Pin_12             // PA12 - CAN TX
#define UART_TX_PIN                 GPIO_Pin_9              // PA9 - UART TX
#define UART_RX_PIN                 GPIO_Pin_10             // PA10 - UART RX
#define BUTTON_PIN                  GPIO_Pin_0              // PA0 - Button
#define LED_PIN                     GPIO_Pin_13             // PC13 - LED

// Timing and Delays
#define MAIN_LOOP_DELAY             100000                  // Main loop delay
#define DELAY_TEST_GPIO             10000000                // GPIO test delay

// Buffer Sizes
#define UART_TX_BUFFER_SIZE         50                      // UART TX buffer size
#define CAN_RX_BUFFER_SIZE          8                       // CAN RX buffer size
#define BUTTON_PIN                  GPIO_Pin_0              // PA0 - Button input
#define LED_PIN                     GPIO_Pin_13             // PC13 - LED output

// Interrupt Priorities
#define DMA_IRQ_PRIORITY            1                       // DMA interrupt priority

/* ======================= External Variables ======================= */

extern CanTxMsg TxMessage;
extern CanRxMsg RxMessage;

extern uint8_t uart_tx_buffer[50];

extern volatile uint8_t uart_dma_busy;
extern uint32_t transmission_counter;
extern uint8_t gpio_last_read;

/* ======================= Function Prototypes ======================= */

/**
 * @brief Configure all system peripherals
 */
void System_Config(void);

/**
 * @brief Configure CAN peripheral
 */
void CAN_Config(void);

/**
 * @brief Configure UART peripheral
 */
void UART_Config(void);

/**
 * @brief Configure DMA for UART transmission
 */
void DMA_Config(void);

/**
 * @brief Configure GPIO pins
 */
void GPIO_Config(void);

/**
 * @brief Test GPIO A12 pin functionality
 */
void Test_GPIO_A12(void);

/**
 * @brief Transmit CAN message
 */
void CAN_TransmitMessage(void);

/**
 * @brief Format CAN data for UART transmission
 * @param rx_data: Pointer to received CAN data
 * @param counter: Message counter
 * @return Length of formatted string
 */
uint8_t Format_CAN_Data_For_UART(uint8_t* rx_data, uint32_t counter);

/**
 * @brief Start UART DMA transmission
 * @param data_length: Number of bytes to transmit
 */
void Start_UART_DMA_Transmission(uint16_t data_length);

/**
 * @brief Simple delay function
 * @param delay: Delay count
 */
void Delay(uint32_t delay);

/**
 * @brief Debug buffer integrity check
 */
void Debug_Check_Buffer_Integrity(void);




/**
 * @brief Complete system configuration
 */
void System_Config(void);

/* Initialization Check Functions */
uint8_t Is_CAN_Initialized(void);
uint8_t Is_UART_Initialized(void);
uint8_t Is_DMA_Initialized(void);
uint8_t Is_GPIO_Initialized(void);
uint8_t Is_System_Initialized(void);
uint8_t Get_Init_Status(void);

/* CAN Dynamic Configuration Functions */
uint8_t CAN_Change_Filter(uint16_t filter_id, uint16_t filter_mask, uint32_t filter_scale);
uint8_t CAN_Change_Mode(uint32_t new_mode);
uint8_t CAN_Change_BitTiming(uint16_t prescaler, uint32_t bs1, uint32_t bs2, uint32_t sjw);
uint8_t CAN_Set_Default_Clock(void);
uint8_t CAN_Get_Current_Config(uint32_t* mode, uint32_t* prescaler);

#endif /* CONFIG_H */