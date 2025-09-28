/*
 * =============================================================================
 * Project: STM32F103 CAN Configuration Implementation (SPL)
 * File: config.c
 * Author: hoangphuc540202@gmail.com  
 * Github: https://github.com/hoangphuc540202
 * Date: August 2025
 * 
 * Description: Implementation of all peripheral configuration functions
 * Includes initialization check functions and dynamic configuration functions
 * =============================================================================
 */

#include "config.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "system_stm32f10x.h"
#include "misc.h"

/* ======================= Global Variables ======================= */
CanTxMsg TxMessage;
CanRxMsg RxMessage;
uint8_t can_rx_buffer[CAN_RX_BUFFER_SIZE];         // Buffer to store CAN received data
uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];       // Buffer for UART transmission
volatile uint8_t can_data_ready = 0;               // Flag indicating new CAN data available
volatile uint8_t uart_dma_busy = 0;                // Flag indicating UART DMA is busy
uint32_t transmission_counter = 0;
uint8_t gpio_last_read = 0;

/* Initialization status flags */
static uint8_t can_initialized = 0;
static uint8_t uart_initialized = 0;
static uint8_t dma_initialized = 0;
static uint8_t gpio_initialized = 0;
static uint8_t system_initialized = 0;

/* Current CAN configuration tracking */
static uint32_t current_can_mode = CAN_MODE_DEFAULT;
static uint32_t current_can_prescaler = CAN_PRESCALER_DEFAULT;

/* ======================= Initialization Check Functions ======================= */

/**
 * @brief Check if CAN peripheral is initialized
 * @return 1 if initialized, 0 if not
 */
uint8_t Is_CAN_Initialized(void)
{
    return can_initialized;
}

/**
 * @brief Check if UART peripheral is initialized
 * @return 1 if initialized, 0 if not
 */
uint8_t Is_UART_Initialized(void)
{
    return uart_initialized;
}

/**
 * @brief Check if DMA is initialized
 * @return 1 if initialized, 0 if not
 */
uint8_t Is_DMA_Initialized(void)
{
    return dma_initialized;
}

/**
 * @brief Check if GPIO is initialized
 * @return 1 if initialized, 0 if not
 */
uint8_t Is_GPIO_Initialized(void)
{
    return gpio_initialized;
}

/**
 * @brief Check if system is fully initialized
 * @return 1 if all peripherals are initialized, 0 if not
 */
uint8_t Is_System_Initialized(void)
{
    return system_initialized;
}

/**
 * @brief Get comprehensive initialization status
 * @return Bit field indicating which peripherals are initialized
 *         Bit 0: CAN, Bit 1: UART, Bit 2: DMA, Bit 3: GPIO, Bit 4: System
 */
uint8_t Get_Init_Status(void)
{
    uint8_t status = 0;
    status |= (can_initialized << 0);
    status |= (uart_initialized << 1);
    status |= (dma_initialized << 2);
    status |= (gpio_initialized << 3);
    status |= (system_initialized << 4);
    return status;
}

/* ======================= CAN Dynamic Configuration Functions ======================= */

/**
 * @brief Change CAN filter configuration dynamically
 * @param filter_id: Filter ID value (0x0000 to 0xFFFF)
 * @param filter_mask: Filter mask value (0x0000 = accept all, 0xFFFF = exact match)
 * @param filter_scale: CAN_FilterScale_16bit or CAN_FilterScale_32bit
 * @return 1 if successful, 0 if failed
 */
uint8_t CAN_Change_Filter(uint16_t filter_id, uint16_t filter_mask, uint32_t filter_scale)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    
    if (!can_initialized) {
        return 0; // CAN not initialized
    }
    
    /* Disable filter before reconfiguration */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    /* Configure new filter settings */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = filter_scale;
    
    if (filter_scale == CAN_FilterScale_32bit) {
        /* 32-bit filter configuration */
        CAN_FilterInitStructure.CAN_FilterIdHigh = (filter_id << 5);  // Standard ID in upper 11 bits
        CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (filter_mask << 5);
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    } else {
        /* 16-bit filter configuration */
        CAN_FilterInitStructure.CAN_FilterIdHigh = filter_id;
        CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = filter_mask;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    }
    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    return 1; // Success
}

/**
 * @brief Change CAN operating mode dynamically
 * @param new_mode: CAN_Mode_Normal, CAN_Mode_LoopBack, CAN_Mode_Silent, CAN_Mode_Silent_LoopBack
 * @return 1 if successful, 0 if failed
 */
uint8_t CAN_Change_Mode(uint32_t new_mode)
{
    CAN_InitTypeDef CAN_InitStructure;
    
    if (!can_initialized) {
        return 0; // CAN not initialized
    }
    
    /* Get current configuration */
    CAN_StructInit(&CAN_InitStructure);
    
    /* Update mode while keeping other settings */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = new_mode;  // Set new mode
    
    /* Keep current bit timing */
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
    CAN_InitStructure.CAN_Prescaler = current_can_prescaler;
    
    /* Reinitialize CAN with new mode */
    CAN_Init(CAN1, &CAN_InitStructure);
    
    /* Update current mode tracking */
    current_can_mode = new_mode;
    
    return 1; // Success
}

/**
 * @brief Change CAN bit timing (baudrate) dynamically
 * @param prescaler: CAN prescaler value (1-1024)
 * @param bs1: Bit segment 1 (CAN_BS1_1tq to CAN_BS1_16tq)
 * @param bs2: Bit segment 2 (CAN_BS2_1tq to CAN_BS2_8tq)
 * @param sjw: Synchronization jump width (CAN_SJW_1tq to CAN_SJW_4tq)
 * @return 1 if successful, 0 if failed
 */
uint8_t CAN_Change_BitTiming(uint16_t prescaler, uint32_t bs1, uint32_t bs2, uint32_t sjw)
{
    CAN_InitTypeDef CAN_InitStructure;
    
    if (!can_initialized) {
        return 0; // CAN not initialized
    }
    
    /* Validate prescaler range */
    if (prescaler < 1 || prescaler > 1024) {
        return 0; // Invalid prescaler
    }
    
    /* Get current configuration */
    CAN_StructInit(&CAN_InitStructure);
    
    /* Update bit timing while keeping other settings */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = current_can_mode;  // Keep current mode
    
    /* Set new bit timing */
    CAN_InitStructure.CAN_SJW = sjw;
    CAN_InitStructure.CAN_BS1 = bs1;
    CAN_InitStructure.CAN_BS2 = bs2;
    CAN_InitStructure.CAN_Prescaler = prescaler;
    
    /* Reinitialize CAN with new bit timing */
    CAN_Init(CAN1, &CAN_InitStructure);
    
    /* Update current prescaler tracking */
    current_can_prescaler = prescaler;
    
    return 1; // Success
}

/**
 * @brief Set CAN to default clock configuration (400kbps)
 * @return 1 if successful, 0 if failed
 */
uint8_t CAN_Set_Default_Clock(void)
{
    /* Default configuration: 400kbps @ 36MHz APB1 clock */
    /* Formula: Baudrate = APB1_Clock / (Prescaler * (1 + BS1 + BS2)) */
    /* 400kbps = 36MHz / (9 * (1 + 8 + 1)) = 36MHz / 90 = 400kbps */
    return CAN_Change_BitTiming(CAN_PRESCALER_DEFAULT, CAN_BS1_8tq, CAN_BS2_1tq, CAN_SJW_1tq);
}

/**
 * @brief Get current CAN configuration
 * @param mode: Pointer to store current mode
 * @param prescaler: Pointer to store current prescaler
 * @return 1 if successful, 0 if failed
 */
uint8_t CAN_Get_Current_Config(uint32_t* mode, uint32_t* prescaler)
{
    if (!can_initialized || mode == 0 || prescaler == 0) {
        return 0;
    }
    
    *mode = current_can_mode;
    *prescaler = current_can_prescaler;
    
    return 1;
}

/* ======================= Original Configuration Functions ======================= */

/**
 * @brief Configure CAN1 peripheral
 */
void CAN_Config(void)
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable peripheral clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure CAN pins: PA11 (RX) and PA12 (TX) */
    GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Reset and initialize CAN peripheral */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN Cell Configuration */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_MODE_DEFAULT;

    /* Default bit timing configuration */
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
    CAN_InitStructure.CAN_Prescaler = CAN_PRESCALER_DEFAULT;
    CAN_Init(CAN1, &CAN_InitStructure);

    /* CAN Filter Configuration - Accept All */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  // Accept all
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;   // Accept all
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* Configure CAN interrupts */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    /* Configure NVIC for CAN interrupts */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /* Set initialization flag and track current config */
    can_initialized = 1;
    current_can_mode = CAN_MODE_DEFAULT;
    current_can_prescaler = CAN_PRESCALER_DEFAULT;
}

/**
 * @brief Configure USART1 for data transmission
 */
void UART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable peripheral clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Configure USART1 pins: PA9 (TX) and PA10 (RX) */
    GPIO_InitStructure.GPIO_Pin = UART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 configuration */
    USART_InitStructure.USART_BaudRate = UART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    
    /* Configure DMA transfer complete interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Set initialization flag */
    uart_initialized = 1;
}

/* ======================= DMA Configuration ======================= */
/**
 * @brief Configure DMA for UART TX data transmission
 * @note  DMA1_Channel4 is used for USART1_TX (Memory to Peripheral)
 *        This enables automatic data transfer from memory buffer to UART
 */
void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Configure DMA1 Channel4 for USART1_TX */
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_tx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    /* Enable DMA transfer complete interrupt */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    /* Set initialization flag */
    dma_initialized = 1;
}

/**
 * @brief Configure GPIO pins
 */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure button pin PA0 */
    GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure LED pin PC13 */
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, LED_PIN); // LED off initially

    /* Set initialization flag */
    gpio_initialized = 1;
}


/**
 * @brief Complete system configuration
 */
void System_Config(void)
{
    /* Initialize system clock */
    SystemInit();
    
    /* Configure all peripherals */
    GPIO_Config();
    DMA_Config();
    UART_Config();
    CAN_Config();
    
    /* Enable global interrupts */
    __enable_irq();
    
    /* Set system initialization flag */
    system_initialized = 1;
}

/**
 * @brief Transmit CAN message with incrementing data
 */
void CAN_TransmitMessage(void)
{
    uint8_t TransmitMailbox;

    /* Prepare CAN message */
    TxMessage.StdId = CAN_MSG_ID;
    TxMessage.ExtId = 0x00;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = CAN_MSG_DLC;
    
    /* Fill data with incrementing pattern */
    TxMessage.Data[0] = (transmission_counter >> 24) & 0xFF;
    TxMessage.Data[1] = (transmission_counter >> 16) & 0xFF;
    TxMessage.Data[2] = (transmission_counter >> 8) & 0xFF;
    TxMessage.Data[3] = transmission_counter & 0xFF;
    TxMessage.Data[4] = 0xAA;
    TxMessage.Data[5] = 0x55;
    TxMessage.Data[6] = 0xCC;
    TxMessage.Data[7] = 0x33;

    /* Transmit the message */
    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
    transmission_counter++;
}

/**
 * @brief Format CAN data for UART transmission
 */
uint8_t Format_CAN_Data_For_UART(uint8_t* rx_data, uint32_t counter)
{
    uint8_t length = 0;
    const uint8_t BUFFER_MAX = sizeof(uart_tx_buffer) - 1;
    
    /* Add message header */
    if (length + 4 >= BUFFER_MAX) return length;
    uart_tx_buffer[length++] = 'M';
    uart_tx_buffer[length++] = 'S';
    uart_tx_buffer[length++] = 'G';
    uart_tx_buffer[length++] = ':';
    
    /* Add counter (4 digits hex) */
    if (length + 4 >= BUFFER_MAX) return length;
    uart_tx_buffer[length++] = ((counter >> 12) & 0x0F) < 10 ? 
                               '0' + ((counter >> 12) & 0x0F) : 
                               'A' + ((counter >> 12) & 0x0F) - 10;
    uart_tx_buffer[length++] = ((counter >> 8) & 0x0F) < 10 ? 
                               '0' + ((counter >> 8) & 0x0F) : 
                               'A' + ((counter >> 8) & 0x0F) - 10;
    uart_tx_buffer[length++] = ((counter >> 4) & 0x0F) < 10 ? 
                               '0' + ((counter >> 4) & 0x0F) : 
                               'A' + ((counter >> 4) & 0x0F) - 10;
    uart_tx_buffer[length++] = (counter & 0x0F) < 10 ? 
                               '0' + (counter & 0x0F) : 
                               'A' + (counter & 0x0F) - 10;
    
    /* Add data header */
    if (length + 6 >= BUFFER_MAX) return length;
    uart_tx_buffer[length++] = ' ';
    uart_tx_buffer[length++] = 'D';
    uart_tx_buffer[length++] = 'A';
    uart_tx_buffer[length++] = 'T';
    uart_tx_buffer[length++] = 'A';
    uart_tx_buffer[length++] = ':';
    
    /* Add data bytes in hex format */
    for (uint8_t i = 0; i < 8; i++) {
        if (length + 2 >= BUFFER_MAX) return length;
        uart_tx_buffer[length++] = (rx_data[i] >> 4) < 10 ? 
                                   '0' + (rx_data[i] >> 4) : 
                                   'A' + (rx_data[i] >> 4) - 10;
        uart_tx_buffer[length++] = (rx_data[i] & 0x0F) < 10 ? 
                                   '0' + (rx_data[i] & 0x0F) : 
                                   'A' + (rx_data[i] & 0x0F) - 10;
        if (i < 7) {
            if (length + 1 >= BUFFER_MAX) return length;
            uart_tx_buffer[length++] = ' ';
        }
    }
    
    /* Add line ending */
    if (length + 2 >= BUFFER_MAX) return length;
    uart_tx_buffer[length++] = '\r';
    uart_tx_buffer[length++] = '\n';
    
    return length;
}

/**
 * @brief Start DMA transmission of UART data
 */
void Start_UART_DMA_Transmission(uint16_t data_length)
{
    /* Wait for previous transmission to complete */
    while (uart_dma_busy);
    
    /* Set DMA busy flag */
    uart_dma_busy = 1;
    
    /* Disable DMA channel */
    DMA_Cmd(DMA1_Channel4, DISABLE);
    
    /* Set new buffer size */
    DMA1_Channel4->CNDTR = data_length;
    
    /* Enable DMA channel to start transmission */
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/**
 * @brief Simple delay function
 */
void Delay(uint32_t delay)
{
    while (delay--);
}

/**
 * @brief Debug function to check buffer integrity
 */
void Debug_Check_Buffer_Integrity(void)
{
    if (uart_dma_busy > 1) {
        GPIO_ResetBits(GPIOC, LED_PIN); // LED on for error indication
        uart_dma_busy = 0;
    }
}

/* ======================= Interrupt Handlers ======================= */

/**
 * @brief CAN RX interrupt handler
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET) {
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        /* Copy received data to buffer */
        for (uint8_t i = 0; i < 8; i++) {
            can_rx_buffer[i] = RxMessage.Data[i];
        }
        
        can_data_ready = 1;
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/**
 * @brief CAN error interrupt handler
 */
void CAN1_SCE_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_ERR) == SET) {
        GPIOC->ODR ^= LED_PIN; // Toggle LED on error
        CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
    }

    if (CAN_GetITStatus(CAN1, CAN_IT_BOF) == SET) {
        GPIOC->ODR ^= LED_PIN;
        CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
    }

    if (CAN_GetITStatus(CAN1, CAN_IT_EPV) == SET) {
        GPIOC->ODR ^= LED_PIN;
        CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
    }

    if (CAN_GetITStatus(CAN1, CAN_IT_EWG) == SET) {
        GPIOC->ODR ^= LED_PIN;
        CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
    }
}

/**
 * @brief DMA1 Channel4 interrupt handler
 */
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) == SET) {
        uart_dma_busy = 0;
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}

/* ======================= End of File ======================= */