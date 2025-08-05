/*
 * =============================================================================
 * Project: STM32F103 CAN Continuous Communication with DMA (SPL) - Simple Version
 * File: can_dma_simple.c
 * Description: Simplified version with basic CAN communication and UART output
 * Author: CAN Driver Team
 * Date: August 2025
 * 
 * This is a simplified version that focuses on the core functionality
 * without complex DMA operations for easier understanding and debugging.
 * =============================================================================
 */

#include "stm32f10x.h"

/* ======================= Global Variables ======================= */
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

/* Buffer for data transfer */
uint8_t can_rx_buffer[8];
volatile uint8_t can_data_ready = 0;
uint32_t transmission_counter = 0;

/* ======================= Function Prototypes ======================= */
void SystemInit_Custom(void);
void CAN_Config(void);
void UART_Config(void);
void CAN_TransmitMessage(void);
void UART_SendString(char* str);
void UART_SendByte(uint8_t byte);
void Delay(uint32_t delay);
void Format_And_Send_CAN_Data(void);

/* ======================= System Configuration ======================= */
/**
 * @brief Custom system initialization
 * @note  Sets up the system clock to 72MHz
 */
void SystemInit_Custom(void)
{
    /* Reset the RCC clock configuration to the default reset state */
    RCC_DeInit();
    
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);
    
    /* Wait till HSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
    
    /* Configure PLL */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // 8MHz * 9 = 72MHz
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08);
    
    /* Configure HCLK, PCLK1, PCLK2 */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   // HCLK = SYSCLK = 72MHz
    RCC_PCLK1Config(RCC_HCLK_Div2);    // PCLK1 = HCLK/2 = 36MHz (for CAN)
    RCC_PCLK2Config(RCC_HCLK_Div1);    // PCLK2 = HCLK = 72MHz
}

/* ======================= CAN Configuration ======================= */
/**
 * @brief Configure CAN1 peripheral in loopback mode
 * @note  Comprehensive CAN setup with detailed comments
 */
void CAN_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable peripheral clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

    /* Configure CAN pins: PB8 (RX) and PB9 (TX) */
    // CAN RX pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // CAN TX pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* CAN peripheral configuration */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN Cell Configuration */
    CAN_InitStructure.CAN_TTCM = DISABLE;                  
    CAN_InitStructure.CAN_ABOM = DISABLE;                  
    CAN_InitStructure.CAN_AWUM = DISABLE;                  
    CAN_InitStructure.CAN_NART = DISABLE;                  
    CAN_InitStructure.CAN_RFLM = DISABLE;                  
    CAN_InitStructure.CAN_TXFP = ENABLE;                   
    CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;        // Loopback mode for testing
    
    /* CAN Bit Timing for 1Mbps (APB1 = 36MHz) */
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;               
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;               
    CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;               
    CAN_InitStructure.CAN_Prescaler = 4;                   
    CAN_Init(CAN1, &CAN_InitStructure);

    /* CAN Filter Configuration - Accept all messages */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;          
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; 
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; 
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;     
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;      
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; 
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;  
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; 
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; 
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* Configure CAN interrupt */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);              
    
    /* Configure NVIC for CAN interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ======================= UART Configuration ======================= */
/**
 * @brief Configure USART1 for simple data transmission
 * @note  Basic UART setup without DMA for simplified debugging
 */
void UART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* Enable peripheral clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

    /* Configure UART pins: PA9 (TX) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure PA10 (RX) - optional for this example */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 configuration */
    USART_InitStructure.USART_BaudRate = 115200;                    
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     
    USART_InitStructure.USART_StopBits = USART_StopBits_1;         
    USART_InitStructure.USART_Parity = USART_Parity_No;            
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStructure.USART_Mode = USART_Mode_Tx; // Only TX for this example
    USART_Init(USART1, &USART_InitStructure);
    
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
}

/* ======================= CAN Transmission Function ======================= */
/**
 * @brief Transmit CAN message with incrementing data
 * @note  Creates and sends a CAN message with test data
 */
void CAN_TransmitMessage(void)
{
    uint8_t TransmitMailbox;

    /* Prepare CAN message */
    TxMessage.StdId = 0x123;                               
    TxMessage.ExtId = 0x00;                                
    TxMessage.IDE = CAN_Id_Standard;                       
    TxMessage.RTR = CAN_RTR_Data;                          
    TxMessage.DLC = 8;                                     
    
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
    
    /* Wait for transmission completion */
    while ((CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok));
    
    /* Increment transmission counter */
    transmission_counter++;
}

/* ======================= UART Utility Functions ======================= */
/**
 * @brief Send a single byte via UART
 * @param byte: Byte to send
 */
void UART_SendByte(uint8_t byte)
{
    /* Wait until transmit data register is empty */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    
    /* Send byte */
    USART_SendData(USART1, byte);
}

/**
 * @brief Send a string via UART
 * @param str: Null-terminated string to send
 */
void UART_SendString(char* str)
{
    while (*str) {
        UART_SendByte(*str++);
    }
}

/**
 * @brief Convert nibble (4 bits) to hex character
 * @param nibble: Value 0-15
 * @return Hex character '0'-'9', 'A'-'F'
 */
char Nibble_To_Hex(uint8_t nibble)
{
    if (nibble < 10) {
        return '0' + nibble;
    } else {
        return 'A' + nibble - 10;
    }
}

/**
 * @brief Format and send CAN data via UART
 * @note  Sends formatted string: "MSG:XXXX DATA:AA BB CC DD EE FF 11 22\r\n"
 */
void Format_And_Send_CAN_Data(void)
{
    uint8_t i;
    
    /* Send header */
    UART_SendString("MSG:");
    
    /* Send counter in hex (4 digits) */
    UART_SendByte(Nibble_To_Hex((transmission_counter >> 12) & 0x0F));
    UART_SendByte(Nibble_To_Hex((transmission_counter >> 8) & 0x0F));
    UART_SendByte(Nibble_To_Hex((transmission_counter >> 4) & 0x0F));
    UART_SendByte(Nibble_To_Hex(transmission_counter & 0x0F));
    
    /* Send data header */
    UART_SendString(" DATA:");
    
    /* Send data bytes in hex */
    for (i = 0; i < 8; i++) {
        UART_SendByte(Nibble_To_Hex((can_rx_buffer[i] >> 4) & 0x0F));
        UART_SendByte(Nibble_To_Hex(can_rx_buffer[i] & 0x0F));
        if (i < 7) {
            UART_SendByte(' ');  // Space between bytes
        }
    }
    
    /* Send line ending */
    UART_SendString("\r\n");
}

/* ======================= Utility Functions ======================= */
/**
 * @brief Simple delay function
 * @param delay: Delay count
 */
void Delay(uint32_t delay)
{
    volatile uint32_t i;
    for (i = 0; i < delay; i++);
}

/* ======================= Interrupt Handler ======================= */
/**
 * @brief CAN RX interrupt handler
 * @note  Called when CAN message is received
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    /* Check if FIFO0 message pending */
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET) {
        /* Receive CAN message */
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        /* Copy data to buffer */
        for (uint8_t i = 0; i < 8; i++) {
            can_rx_buffer[i] = RxMessage.Data[i];
        }
        
        /* Set data ready flag */
        can_data_ready = 1;
        
        /* Clear interrupt flag */
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/* ======================= Main Function ======================= */
/**
 * @brief Main application function
 * @note  Demonstrates continuous CAN communication with UART output
 * 
 * Program Flow:
 * 1. Initialize system (72MHz clock)
 * 2. Configure UART for data display
 * 3. Configure CAN in loopback mode
 * 4. Main loop:
 *    - Send CAN message
 *    - Wait for reception (interrupt)
 *    - Format and send data via UART
 *    - Delay before next transmission
 */
int main(void)
{
    /* System initialization */
    SystemInit_Custom();                // Custom system clock setup
    
    /* Peripheral configuration */
    UART_Config();                      // Configure UART for output
    CAN_Config();                       // Configure CAN in loopback mode
    
    /* Send startup message */
    UART_SendString("STM32F103 CAN Loopback Demo Started\r\n");
    UART_SendString("Format: MSG:XXXX DATA:AA BB CC DD EE FF 11 22\r\n");
    UART_SendString("========================================\r\n");
    
    /* Main application loop */
    while (1) {
        /* Transmit CAN message */
        CAN_TransmitMessage();
        
        /* Check if new CAN data received */
        if (can_data_ready) {
            /* Format and send data via UART */
            Format_And_Send_CAN_Data();
            
            /* Clear flag */
            can_data_ready = 0;
        }
        
        /* Delay approximately 1 second */
        Delay(3600000);  // Adjust based on actual timing requirements
    }
}
