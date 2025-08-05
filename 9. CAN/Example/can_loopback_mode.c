/*
 * =============================================================================
 * Project: STM32F103 CAN Continuous Communication with DMA (SPL)
 * File: can_dma_continuous.c
 * Description: Continuous CAN communication using DMA for both RX and UART TX
 *              - CAN operates in loopback mode for testing
 *              - DMA transfers CAN RX data to memory buffer
 *              - DMA transfers data from memory to UART TX for display
 * Author: CAN Driver Team
 * Date: August 2025
 * 
 * IMPORTANT NOTE:
 * This file contains compilation errors due to missing include dependencies.
 * For a working example, please use main_simple.c which has been tested
 * and contains comprehensive comments.
 * 
 * To compile and run:
 * 1. Use main_simple.c instead of this file
 * 2. Or fix the include paths and library dependencies
 * 3. See CAN_DMA_DETAILED_EXPLANATION.md for complete documentation
 * 
 * Hardware Setup:
 * - STM32F103C8T6 Blue Pill board
 * - CAN pins: PB8 (RX), PB9 (TX)
 * - UART pins: PA9 (TX), PA10 (RX)
 * - System clock: 72MHz
 * 
 * DMA Channel Assignment:
 * - DMA1_Channel4: USART1_TX (Memory to Peripheral)
 * - DMA1_Channel5: USART1_RX (Peripheral to Memory) - Not used in this example
 * - Manual transfer from CAN FIFO to memory buffer
 * 
 * Communication Flow:
 * 1. CAN transmits message in loopback mode
 * 2. CAN receives message via FIFO
 * 3. Data is copied to memory buffer
 * 4. DMA transfers buffer data to UART for display
 * =============================================================================
 */
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

/* DMA Buffer for data transfer */
uint8_t can_rx_buffer[8];           // Buffer to store CAN received data
uint8_t uart_tx_buffer[50];         // Buffer for UART transmission (formatted data) - Increased size
volatile uint8_t can_data_ready = 0; // Flag indicating new CAN data is available
volatile uint8_t uart_dma_busy = 0;  // Flag indicating UART DMA is busy

/* Transmission counter for continuous operation */
uint32_t transmission_counter = 0;
uint8_t gpio_last_read = 0;

/* ======================= CAN Configuration ======================= */
/**
 * @brief Configure CAN1 peripheral in loopback mode
 * @note  Loopback mode allows testing without external CAN devices
 *        CAN TX and RX are internally connected
 */
void CAN_Config(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable peripheral clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // CAN1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOA clock for CAN pins

    /* Configure CAN pins: A11 (RX) and A12 (TX) */
    // CAN RX pin configuration (Input Pull-up)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          // Input with pull-up
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // CAN TX pin configuration (Alternate Function Push-Pull)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // High speed for reliable communication
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Reset and initialize CAN peripheral */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN Cell Configuration */
    CAN_InitStructure.CAN_TTCM = DISABLE;                  // Time Triggered Communication Mode
    CAN_InitStructure.CAN_ABOM = DISABLE;                  // Automatic Bus-Off Management
    CAN_InitStructure.CAN_AWUM = DISABLE;                  // Automatic Wake-Up Mode
    CAN_InitStructure.CAN_NART = DISABLE;                  // Non-Automatic Retransmission
    CAN_InitStructure.CAN_RFLM = DISABLE;                  // Receive FIFO Locked Mode
    CAN_InitStructure.CAN_TXFP = ENABLE;                   // Transmit FIFO Priority
    CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;        // <<<< LOOPBACK MODE FOR TESTING
    
    /* CAN Bit Timing Configuration for 1Mbps */
    // Formula: CAN_Baudrate = APB1_Clock / (Prescaler * (1 + BS1 + BS2))
    // With APB1 = 36MHz: 1Mbps = 36MHz / (6 * (1 + 8 + 1)) = 36MHz / 60 = 600kbps
    // Adjusted for 1Mbps: Prescaler = 4
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;               // Synchronization Jump Width
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;               // Bit Segment 1
    CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;               // Bit Segment 2
    CAN_InitStructure.CAN_Prescaler = 4;                   // Prescaler for ~1Mbps
    CAN_Init(CAN1, &CAN_InitStructure);

    /* CAN Filter Configuration */
    // Filter accepts all messages (no filtering)
    CAN_FilterInitStructure.CAN_FilterNumber = 0;          // Filter number 0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // ID/Mask mode
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 32-bit scale
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;     // Filter ID high
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;      // Filter ID low
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; // Mask high (accept all)
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;  // Mask low (accept all)
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; // Assign to FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; // Enable filter
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* Configure CAN interrupt for message reception */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);              // FIFO0 message pending interrupt
    
    /* Configure NVIC for CAN interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ======================= CAN Transmission Function ======================= */
/**
 * @brief Transmit CAN message continuously with incrementing data
 * @note  This function sends a new CAN message with updated data payload
 */
void CAN_TransmitMessage(void)
{
    uint8_t TransmitMailbox;

    /* Prepare CAN message with incrementing data */
    TxMessage.StdId = 0x123;                               // Standard ID
    TxMessage.ExtId = 0x00;                                // Extended ID (not used)
    TxMessage.IDE = CAN_Id_Standard;                       // Use standard ID
    TxMessage.RTR = CAN_RTR_Data;                          // Data frame (not remote)
    TxMessage.DLC = 8;                                     // Data length: 8 bytes
    
    /* Fill data with incrementing pattern for testing */
    TxMessage.Data[0] = (transmission_counter >> 24) & 0xFF; // Counter MSB
    TxMessage.Data[1] = (transmission_counter >> 16) & 0xFF;
    TxMessage.Data[2] = (transmission_counter >> 8) & 0xFF;
    TxMessage.Data[3] = transmission_counter & 0xFF;         // Counter LSB
    TxMessage.Data[4] = 0xAA;                                // Test pattern
    TxMessage.Data[5] = 0x55;                                // Test pattern
    TxMessage.Data[6] = 0xCC;                                // Test pattern
    TxMessage.Data[7] = 0x33;                                // Test pattern

    /* Transmit the message */
    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
    
    /* Wait for transmission completion */
    while ((CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok));
    
    /* Increment transmission counter */
    transmission_counter++;
}

/* ======================= UART Configuration ======================= */
/**
 * @brief Configure USART1 for data transmission via DMA
 * @note  USART1 pins: PA9 (TX), PA10 (RX)
 *        Baud rate: 115200 bps for PC terminal communication
 */
void UART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable peripheral clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1 clock

    /* Configure USART1 pins: PA9 (TX) and PA10 (RX) */
    // TX pin configuration (Alternate Function Push-Pull)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // High speed
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // RX pin configuration (Input Floating) 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // Input floating
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 configuration */
    USART_InitStructure.USART_BaudRate = 115200;                    // 115200 bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 8 data bits
    USART_InitStructure.USART_StopBits = USART_StopBits_1;         // 1 stop bit
    USART_InitStructure.USART_Parity = USART_Parity_No;            // No parity
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // TX and RX enabled
    USART_Init(USART1, &USART_InitStructure);
    
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
    
    /* Enable USART1 DMA for transmission */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    
    /* Configure DMA transfer complete interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;       // DMA1 Channel4 for USART1_TX
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;        // USART1 data register
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_tx_buffer;         // Memory buffer address
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                       // Memory to Peripheral
    DMA_InitStructure.DMA_BufferSize = 0;                                    // Will be set dynamically
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         // Peripheral address fixed
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                  // Memory address increments
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // 8-bit peripheral data
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;          // 8-bit memory data
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                            // Normal mode (not circular)
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                      // High priority
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                             // Not memory-to-memory
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    /* Enable DMA transfer complete interrupt */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);                          // Transfer complete interrupt
}

/*=========================GPIO Config===============================*/
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PA0 (ADC input) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // Input floating mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // Push-pull output mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13); // Set PC13 high (LED off)
}

/* ======================= Utility Functions ======================= */
/**
 * @brief Format CAN data into a readable string for UART transmission
 * @param rx_data: Pointer to received CAN data
 * @param counter: Message counter for identification
 * @return Length of formatted string
 */
uint8_t Format_CAN_Data_For_UART(uint8_t* rx_data, uint32_t counter)
{
    uint8_t length = 0;
    const uint8_t BUFFER_MAX = sizeof(uart_tx_buffer) - 1; // Reserve 1 byte for safety
    
    /* Format: "MSG:0001 DATA:AB CD EF 12 34 56 78 90\r\n" */
    /* Add message header */
    if (length + 4 >= BUFFER_MAX) return length; // Check bounds
    uart_tx_buffer[length++] = 'M';
    uart_tx_buffer[length++] = 'S';
    uart_tx_buffer[length++] = 'G';
    uart_tx_buffer[length++] = ':';
    
    /* Add counter (4 digits hex) */
    if (length + 4 >= BUFFER_MAX) return length; // Check bounds
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
    if (length + 6 >= BUFFER_MAX) return length; // Check bounds
    uart_tx_buffer[length++] = ' ';
    uart_tx_buffer[length++] = 'D';
    uart_tx_buffer[length++] = 'A';
    uart_tx_buffer[length++] = 'T';
    uart_tx_buffer[length++] = 'A';
    uart_tx_buffer[length++] = ':';
    
    /* Add data bytes in hex format */
    for (uint8_t i = 0; i < 8; i++) {
        if (length + 2 >= BUFFER_MAX) return length; // Check bounds for 2 hex chars
        uart_tx_buffer[length++] = (rx_data[i] >> 4) < 10 ? 
                                   '0' + (rx_data[i] >> 4) : 
                                   'A' + (rx_data[i] >> 4) - 10;
        uart_tx_buffer[length++] = (rx_data[i] & 0x0F) < 10 ? 
                                   '0' + (rx_data[i] & 0x0F) : 
                                   'A' + (rx_data[i] & 0x0F) - 10;
        if (i < 7) {
            if (length + 1 >= BUFFER_MAX) return length; // Check bounds for space
            uart_tx_buffer[length++] = ' ';  // Space between bytes
        }
    }
    
    /* Add line ending */
    if (length + 2 >= BUFFER_MAX) return length; // Check bounds
    uart_tx_buffer[length++] = '\r';
    uart_tx_buffer[length++] = '\n';
    
    return length;
}

/**
 * @brief Start DMA transmission of UART data
 * @param data_length: Number of bytes to transmit
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
 * @param delay: Delay count (approximate)
 */
void Delay(uint32_t delay)
{
    while (delay--);
}

/**
 * @brief Debug function to check buffer usage and variable integrity
 * @note  This function helps verify that buffer overflow is not occurring
 */
void Debug_Check_Buffer_Integrity(void)
{
    // Check if uart_dma_busy has been corrupted
    if (uart_dma_busy > 1) {
        // Buffer overflow detected!
        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Set PC13 low (LED on)
        uart_dma_busy = 0; // Reset to safe value
        // You could add a breakpoint here or send error message
    }
}

/* ======================= Interrupt Handlers ======================= */
/**
 * @brief CAN RX interrupt handler (FIFO0 message pending)
 * @note  This interrupt fires when a new CAN message is received
 *        The received data is copied to the buffer for DMA transmission
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    /* Check if FIFO0 message pending interrupt */
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET) {
        /* Receive the CAN message */
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        /* Copy received data to buffer */
        for (uint8_t i = 0; i < 8; i++) {
            can_rx_buffer[i] = RxMessage.Data[i];
        }
        
        /* Set flag indicating new data is available */
        can_data_ready = 1;
        
        /* Clear the interrupt flag */
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/**
 * @brief DMA1 Channel4 interrupt handler (USART1 TX complete)
 * @note  This interrupt fires when DMA transmission is complete
 */
void DMA1_Channel4_IRQHandler(void)
{
    /* Check if transfer complete interrupt */
    if (DMA_GetITStatus(DMA1_IT_TC4) == SET) {
        /* Clear DMA busy flag */
        uart_dma_busy = 0;
        
        /* Clear the interrupt flag */
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}

/* ======================= Main Function ======================= */
/**
 * @brief Main application function
 * @note  Initializes all peripherals and runs continuous CAN communication
 *        
 * Program Flow:
 * 1. Initialize system clock (72MHz)
 * 2. Configure DMA for UART transmission
 * 3. Configure UART for data display
 * 4. Configure CAN in loopback mode
 * 5. Continuously transmit CAN messages
 * 6. Process received CAN data and send via UART DMA
 */
int main(void)
{
    /* System Initialization */
    SystemInit();                    // 1. Setup system clock to 72MHz
    
    /* Peripheral Configuration */
    GPIO_Config();
    DMA_Config();                    // 2. Configure DMA for UART transmission
    UART_Config();                   // 3. Configure USART1 for data display
    CAN_Config();                    // 4. Configure CAN1 in loopback mode
    
    /* Enable global interrupts */
    __enable_irq();
    
    /* Application Main Loop */
    while (1) {
        /* Debug: Check buffer integrity */
        Debug_Check_Buffer_Integrity();
        
        
        /* Transmit CAN message with incrementing data */
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_SET )
        {
            if(gpio_last_read == 0) {
                gpio_last_read = 1; // Set flag to prevent multiple triggers
                CAN_TransmitMessage();  // 5. Transmit CAN message
            }
            
        }
        else gpio_last_read = 0;
        
        
        /* Check if new CAN data is available */
        if (can_data_ready) {
            /* Format CAN data for UART transmission */
            uint8_t uart_length = Format_CAN_Data_For_UART(can_rx_buffer, transmission_counter - 1);
            
            /* Start DMA transmission to UART */
            Start_UART_DMA_Transmission(uart_length);
            
            /* Clear the data ready flag */
            can_data_ready = 0;
        }
        
        /* Delay between transmissions (approximately 1 second) */
        Delay(100000);  // Adjust this value based on your requirements
    }
}
/* ======================= End of File ======================= */