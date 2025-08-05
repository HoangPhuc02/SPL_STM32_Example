/*
 * =============================================================================
 * Project: STM32F103 CAN Receiver (SPL)
 * File: can_normal_mode_rx.c
 * Description: CAN Receiver using Normal Mode with interrupt handling
 * Author: CAN Driver Team
 * Date: August 2025
 * 
 * Hardware Setup:
 * - STM32F103C8T6 Blue Pill board
 * - CAN pins: PA11 (RX), PA12 (TX)
 * - LED: PC13 (indicates message reception)
 * - UART: PA9 (TX) for data display (optional)
 * - CAN Transceiver: TJA1050 or similar
 * 
 * Features:
 * - Interrupt-based CAN message reception
 * - LED indication for received messages
 * - Message filtering and validation
 * - 250kbps CAN baudrate
 * - Standard 11-bit CAN ID reception
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "stm32f10x_usart.h"
#include "system_stm32f10x.h"
#include "misc.h"

/* ======================= Global Variables ======================= */
CanRxMsg RxMessage;
volatile uint8_t message_received = 0;
volatile uint32_t message_count = 0;
volatile uint8_t last_data[8];

/* ======================= Function Prototypes ======================= */
void SystemClock_Config(void);
void GPIO_Config(void);
void CAN_Config(void);
void NVIC_Config(void);
void UART_Config(void);
void UART_SendByte(uint8_t byte);
void UART_SendString(char* str);
void UART_SendHex(uint8_t value);
void Display_CAN_Message(void);
void Delay(uint32_t count);

/* ======================= System Clock Configuration ======================= */
/**
 * @brief Configure system clock to 72MHz using HSE and PLL
 * @note  APB1 = 36MHz (for CAN), APB2 = 72MHz (for GPIO, UART)
 */
void SystemClock_Config(void)
{
    ErrorStatus HSEStartUpStatus;

    /* Reset RCC configuration */
    RCC_DeInit();
    
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus == SUCCESS)
    {
        /* Configure Flash latency and prefetch buffer */
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        
        /* Configure AHB, APB1, APB2 clocks */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);    // AHB = 72MHz
        RCC_PCLK1Config(RCC_HCLK_Div2);     // APB1 = 36MHz (for CAN)
        RCC_PCLK2Config(RCC_HCLK_Div1);     // APB2 = 72MHz (for GPIO, UART)
        
        /* Configure PLL: 8MHz * 9 = 72MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        
        /* Wait for PLL ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        
        /* Select PLL as system clock */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource() != 0x08);
    }
    else
    {
        /* HSE failed - halt system */
        while(1);
    }
}

/* ======================= GPIO Configuration ======================= */
/**
 * @brief Configure GPIO pins for CAN, LED, and UART
 */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure CAN pins: PA11 (RX), PA12 (TX) */
    // CAN RX (PA11) - Input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // CAN TX (PA12) - Alternate function push-pull (needed even for RX-only)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure UART pins: PA9 (TX) for debug output */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure LED pin: PC13 - Output push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /* Turn off LED initially (active low) */
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/* ======================= CAN Configuration ======================= */
/**
 * @brief Configure CAN peripheral for normal mode reception
 * @note  Baudrate: 250kbps, accepts ID 0x123
 */
void CAN_Config(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    /* Enable CAN1 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* Reset CAN peripheral */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN Cell Configuration */
    CAN_InitStructure.CAN_TTCM = DISABLE;         // Time Triggered Communication Mode
    CAN_InitStructure.CAN_ABOM = DISABLE;         // Automatic Bus-Off Management
    CAN_InitStructure.CAN_AWUM = DISABLE;         // Automatic Wake-Up Mode
    CAN_InitStructure.CAN_NART = DISABLE;         // No Automatic Retransmission
    CAN_InitStructure.CAN_RFLM = DISABLE;         // Receive FIFO Locked Mode
    CAN_InitStructure.CAN_TXFP = DISABLE;         // Transmit FIFO Priority
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // Normal mode for real CAN communication
    
    /* CAN Bit Timing for 250kbps */
    // Formula: Baudrate = APB1_Clock / (Prescaler × (1 + BS1 + BS2))
    // 250kbps = 36MHz / (8 × (1 + 8 + 7)) = 36MHz / 128 = 281.25kbps ≈ 250kbps
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      // Synchronization Jump Width
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;      // Bit Segment 1
    CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;      // Bit Segment 2
    CAN_InitStructure.CAN_Prescaler = 8;          // Prescaler for 250kbps

    CAN_Init(CAN1, &CAN_InitStructure);

    /* CAN Filter Configuration - Accept only ID 0x123 */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    
    /* Filter for ID 0x123 */
    CAN_FilterInitStructure.CAN_FilterIdHigh = (0x123 << 5);      // ID in upper 11 bits
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (0x7FF << 5);  // Exact match for 11-bit ID
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* Enable FIFO0 message pending interrupt */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/* ======================= NVIC Configuration ======================= */
/**
 * @brief Configure NVIC for CAN interrupt
 */
void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Set priority group */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Configure CAN RX interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ======================= UART Configuration ======================= */
/**
 * @brief Configure UART for debug output
 */
void UART_Config(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* Enable USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* USART1 configuration */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
}

/* ======================= UART Utility Functions ======================= */
/**
 * @brief Send a byte via UART
 */
void UART_SendByte(uint8_t byte)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

/**
 * @brief Send a string via UART
 */
void UART_SendString(char* str)
{
    while(*str)
    {
        UART_SendByte(*str++);
    }
}

/**
 * @brief Send a hex value via UART
 */
void UART_SendHex(uint8_t value)
{
    uint8_t high = (value >> 4) & 0x0F;
    uint8_t low = value & 0x0F;
    
    UART_SendByte(high < 10 ? '0' + high : 'A' + high - 10);
    UART_SendByte(low < 10 ? '0' + low : 'A' + low - 10);
}

/* ======================= CAN Message Display ======================= */
/**
 * @brief Display received CAN message via UART
 */
void Display_CAN_Message(void)
{
    UART_SendString("RX MSG: ID=0x");
    UART_SendHex((RxMessage.StdId >> 8) & 0xFF);
    UART_SendHex(RxMessage.StdId & 0xFF);
    UART_SendString(" DLC=");
    UART_SendByte('0' + RxMessage.DLC);
    UART_SendString(" DATA=");
    
    for(uint8_t i = 0; i < RxMessage.DLC; i++)
    {
        UART_SendHex(RxMessage.Data[i]);
        if(i < RxMessage.DLC - 1) UART_SendByte(' ');
    }
    
    UART_SendString(" COUNT=");
    UART_SendHex((message_count >> 8) & 0xFF);
    UART_SendHex(message_count & 0xFF);
    UART_SendString("\\r\\n");
}

/* ======================= CAN Interrupt Handler ======================= */
/**
 * @brief CAN RX interrupt handler
 * @note  Called when CAN message is received in FIFO0
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    /* Check if FIFO0 message pending */
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET)
    {
        /* Receive the message */
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        /* Copy data to last_data buffer */
        for(uint8_t i = 0; i < 8; i++)
        {
            last_data[i] = (i < RxMessage.DLC) ? RxMessage.Data[i] : 0;
        }
        
        /* Set received flag */
        message_received = 1;
        message_count++;
        
        /* Clear interrupt flag */
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/* ======================= Utility Functions ======================= */
/**
 * @brief Simple delay function
 */
void Delay(uint32_t count)
{
    volatile uint32_t i;
    for (i = 0; i < count; i++);
}

/* ======================= Main Function ======================= */
/**
 * @brief Main application function - CAN Receiver
 * @note  Receives CAN messages and displays them via UART and LED
 * 
 * Program Flow:
 * 1. Initialize system clock (72MHz)
 * 2. Configure GPIO (CAN, LED, UART)
 * 3. Configure CAN peripheral with filter
 * 4. Configure UART for debug output
 * 5. Configure interrupts
 * 6. Main loop: Process received messages
 */
int main(void)
{
    /* System Initialization */
    SystemClock_Config();    // 1. Setup system clock to 72MHz
    GPIO_Config();           // 2. Configure GPIO pins
    CAN_Config();            // 3. Configure CAN peripheral
    UART_Config();           // 4. Configure UART for debug
    NVIC_Config();           // 5. Configure interrupts
    
    /* Send startup message */
    UART_SendString("STM32F103 CAN Receiver Started\\r\\n");
    UART_SendString("Waiting for CAN messages (ID=0x123)...\\r\\n");
    
    /* Startup indication - blink LED 3 times */
    for (uint8_t i = 0; i < 3; i++) {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // LED on
        Delay(200000);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);    // LED off
        Delay(200000);
    }
    
    /* Main application loop */
    while (1)
    {
        /* Check if message was received */
        if (message_received)
        {
            /* Turn on LED to indicate reception */
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            
            /* Display message via UART */
            Display_CAN_Message();
            
            /* Reset flag */
            message_received = 0;
            
            /* Brief LED indication */
            Delay(100000);
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
        }
        
        /* Small delay */
        Delay(1000);
    }
}
