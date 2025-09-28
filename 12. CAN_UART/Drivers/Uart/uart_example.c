/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: uart_example.c
 * Author: hoangphuc540202@gmail.com
 * Date: 22/9/2025
 *
 * Description: Example usage of UART driver with DMA, Interrupt, and Polling modes
 * =============================================================================
 */

#include "stm32f10x.h"
#include "uart.h"
#include <string.h>

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/

/* Test data */
uint8_t tx_data[] = "Hello STM32F103 UART Driver!\r\n";
uint8_t rx_data[100];
char menu_text[] = 
    "\r\n=== STM32F103 UART Driver Test Menu ===\r\n"
    "1. Test Polling Mode\r\n"
    "2. Test Interrupt Mode\r\n"
    "3. Test DMA Mode\r\n"
    "4. Echo Test (Type and press Enter)\r\n"
    "5. Printf Test\r\n"
    "Select option (1-5): ";

/*==================================================================================================
*                                    PRIVATE FUNCTIONS
==================================================================================================*/

/**
 * @brief Simple delay function
 */
void Delay_ms(uint32_t ms)
{
    volatile uint32_t delay = ms * 8000; // Approximate for 72MHz
    while(delay--);
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    /* Configure the system clock to 72 MHz */
    SystemInit();
}

/**
 * @brief Initialize UART1 with default settings
 */
void UART1_Init_Default(void)
{
    /* Configure UART1 handle */
    huart1.Instance = USART1;
    huart1.BaudRate = 115200;
    huart1.WordLength = USART_WordLength_8b;
    huart1.StopBits = USART_StopBits_1;
    huart1.Parity = USART_Parity_No;
    huart1.Mode = USART_Mode_Rx | USART_Mode_Tx;
    huart1.HwFlowCtl = USART_HardwareFlowControl_None;
    
    /* Initialize UART in polling mode by default */
    UART_Init(&huart1, UART_MODE_POLLING);
}

/**
 * @brief Test Polling Mode
 */
void Test_Polling_Mode(void)
{
    uint8_t status;
    
    UART_SendString(&huart1, "\r\n=== Testing Polling Mode ===\r\n");
    
    /* Send test message */
    status = UART_Transmit_Polling(&huart1, tx_data, strlen((char*)tx_data), 1000);
    if(status == 0)
    {
        UART_SendString(&huart1, "✓ Polling transmission successful!\r\n");
    }
    else
    {
        UART_SendString(&huart1, "✗ Polling transmission failed!\r\n");
    }
    
    /* Test formatted output */
    UART_Printf(&huart1, "System tick: %d, UART State: %d\r\n", 
                (int)UART_GetTick(), (int)UART_GetState(&huart1));
    
    UART_SendString(&huart1, "Press any key to continue...\r\n");
    UART_ReceiveChar(&huart1); /* Wait for user input */
}

/**
 * @brief Test Interrupt Mode
 */
void Test_Interrupt_Mode(void)
{
    uint8_t status;
    uint32_t timeout;
    
    UART_SendString(&huart1, "\r\n=== Testing Interrupt Mode ===\r\n");
    
    /* Reconfigure for interrupt mode */
    UART_Init(&huart1, UART_MODE_INTERRUPT);
    
    /* Send test message */
    status = UART_Transmit_IT(&huart1, tx_data, strlen((char*)tx_data));
    if(status == 0)
    {
        UART_SendString(&huart1, "✓ Interrupt transmission started!\r\n");
        
        /* Wait for completion */
        timeout = 1000;
        while(!uart_tx_complete && timeout--)
        {
            Delay_ms(1);
        }
        
        if(uart_tx_complete)
        {
            UART_SendString(&huart1, "✓ Interrupt transmission completed!\r\n");
        }
        else
        {
            UART_SendString(&huart1, "✗ Interrupt transmission timeout!\r\n");
        }
    }
    else
    {
        UART_SendString(&huart1, "✗ Failed to start interrupt transmission!\r\n");
    }
    
    /* Test interrupt reception */
    UART_SendString(&huart1, "Type 5 characters: ");
    status = UART_Receive_IT(&huart1, rx_data, 5);
    if(status == 0)
    {
        /* Wait for completion */
        timeout = 5000;
        while(!uart_rx_complete && timeout--)
        {
            Delay_ms(1);
        }
        
        if(uart_rx_complete)
        {
            rx_data[5] = '\0'; /* Null terminate */
            UART_Printf(&huart1, "\r\nReceived: %s\r\n", rx_data);
        }
        else
        {
            UART_SendString(&huart1, "\r\n✗ Reception timeout!\r\n");
        }
    }
    
    UART_SendString(&huart1, "Press any key to continue...\r\n");
    UART_ReceiveChar(&huart1);
}

/**
 * @brief Test DMA Mode
 */
void Test_DMA_Mode(void)
{
    uint8_t status;
    uint32_t timeout;
    
    UART_SendString(&huart1, "\r\n=== Testing DMA Mode ===\r\n");
    
    /* Reconfigure for DMA mode */
    UART_Init(&huart1, UART_MODE_DMA);
    
    /* Prepare test data in buffer */
    strcpy((char*)uart_tx_buffer, "DMA transmission test message!\r\n");
    
    /* Send test message via DMA */
    status = UART_Transmit_DMA(&huart1, uart_tx_buffer, strlen((char*)uart_tx_buffer));
    if(status == 0)
    {
        UART_SendString(&huart1, "✓ DMA transmission started!\r\n");
        
        /* Wait for completion */
        timeout = 1000;
        while(uart_dma_busy && timeout--)
        {
            Delay_ms(1);
        }
        
        if(!uart_dma_busy)
        {
            UART_SendString(&huart1, "✓ DMA transmission completed!\r\n");
        }
        else
        {
            UART_SendString(&huart1, "✗ DMA transmission timeout!\r\n");
        }
    }
    else
    {
        UART_SendString(&huart1, "✗ Failed to start DMA transmission!\r\n");
    }
    
    /* Test DMA reception */
    UART_SendString(&huart1, "Type 10 characters for DMA RX: ");
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    status = UART_Receive_DMA(&huart1, uart_rx_buffer, 10);
    if(status == 0)
    {
        /* Wait for completion */
        timeout = 5000;
        while(!uart_rx_complete && timeout--)
        {
            Delay_ms(1);
        }
        
        if(uart_rx_complete)
        {
            uart_rx_buffer[10] = '\0'; /* Null terminate */
            UART_Printf(&huart1, "\r\nDMA Received: %s\r\n", uart_rx_buffer);
        }
        else
        {
            UART_SendString(&huart1, "\r\n✗ DMA Reception timeout!\r\n");
        }
    }
    
    UART_SendString(&huart1, "Press any key to continue...\r\n");
    UART_ReceiveChar(&huart1);
}

/**
 * @brief Echo Test
 */
void Test_Echo(void)
{
    char ch;
    
    UART_SendString(&huart1, "\r\n=== Echo Test ===\r\n");
    UART_SendString(&huart1, "Type characters (ESC to exit):\r\n");
    
    while(1)
    {
        ch = UART_ReceiveChar(&huart1);
        
        if(ch == 27) /* ESC key */
        {
            UART_SendString(&huart1, "\r\nEcho test ended.\r\n");
            break;
        }
        
        /* Echo back the character */
        UART_SendChar(&huart1, ch);
        
        /* Handle special characters */
        if(ch == '\r')
        {
            UART_SendChar(&huart1, '\n');
        }
    }
    
    UART_SendString(&huart1, "Press any key to continue...\r\n");
    UART_ReceiveChar(&huart1);
}

/**
 * @brief Printf Test
 */
void Test_Printf(void)
{
    int counter = 0;
    float voltage = 3.3f;
    
    UART_SendString(&huart1, "\r\n=== Printf Test ===\r\n");
    
    for(counter = 0; counter < 5; counter++)
    {
        voltage += 0.1f;
        UART_Printf(&huart1, "Counter: %d, Voltage: %.2f V, Hex: 0x%04X\r\n", 
                    counter, voltage, counter * 0x100);
        Delay_ms(500);
    }
    
    UART_Printf(&huart1, "Current UART State: %s\r\n", 
                (UART_GetState(&huart1) == UART_STATUS_READY) ? "READY" : "BUSY");
    
    UART_SendString(&huart1, "Press any key to continue...\r\n");
    UART_ReceiveChar(&huart1);
}

/*==================================================================================================
*                                     CALLBACK FUNCTIONS
==================================================================================================*/

/**
 * @brief Custom Tx Complete Callback
 */
void UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Custom processing can be added here */
    uart_tx_complete = 1;
    huart->State = UART_STATUS_READY;
    
    /* Optional: Toggle LED or set flag */
}

/**
 * @brief Custom Rx Complete Callback
 */
void UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Custom processing can be added here */
    uart_rx_complete = 1;
    huart->State = UART_STATUS_READY;
    
    /* Optional: Process received data */
}

/**
 * @brief Custom Error Callback
 */
void UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Handle errors */
    huart->State = UART_STATUS_ERROR;
    
    /* Optional: Log error or reset UART */
    if(huart->ErrorCode & UART_ERROR_OVERRUN)
    {
        /* Handle overrun error */
        USART_ClearFlag(huart->Instance, USART_FLAG_ORE);
    }
    
    if(huart->ErrorCode & UART_ERROR_FRAME)
    {
        /* Handle frame error */
        USART_ClearFlag(huart->Instance, USART_FLAG_FE);
    }
    
    if(huart->ErrorCode & UART_ERROR_PARITY)
    {
        /* Handle parity error */
        USART_ClearFlag(huart->Instance, USART_FLAG_PE);
    }
    
    /* Reset error code and state */
    huart->ErrorCode = UART_ERROR_NONE;
    huart->State = UART_STATUS_READY;
}

/*==================================================================================================
*                                       MAIN FUNCTION
==================================================================================================*/

int main(void)
{
    char option;
    
    /* Initialize system */
    SystemClock_Config();
    
    /* Initialize UART1 */
    UART1_Init_Default();
    
    /* Welcome message */
    UART_SendString(&huart1, "\r\n");
    UART_SendString(&huart1, "========================================\r\n");
    UART_SendString(&huart1, "  STM32F103 UART Driver Demo\r\n");
    UART_SendString(&huart1, "  Author: hoangphuc540202@gmail.com\r\n");
    UART_SendString(&huart1, "  Date: 22/9/2025\r\n");
    UART_SendString(&huart1, "========================================\r\n");
    
    /* Main loop */
    while(1)
    {
        /* Display menu */
        UART_SendString(&huart1, menu_text);
        
        /* Get user input */
        option = UART_ReceiveChar(&huart1);
        UART_Printf(&huart1, "%c\r\n", option); /* Echo selection */
        
        /* Process selection */
        switch(option)
        {
            case '1':
                Test_Polling_Mode();
                break;
                
            case '2':
                Test_Interrupt_Mode();
                break;
                
            case '3':
                Test_DMA_Mode();
                break;
                
            case '4':
                Test_Echo();
                break;
                
            case '5':
                Test_Printf();
                break;
                
            default:
                UART_SendString(&huart1, "Invalid option! Please select 1-5.\r\n");
                Delay_ms(1000);
                break;
        }
        
        Delay_ms(500);
    }
}

/*==================================================================================================
*                                SYSTEM CONFIGURATION
==================================================================================================*/

/**
 * @brief System Tick Handler (if using SysTick)
 */
void SysTick_Handler(void)
{
    /* Increment system tick counter */
}

/**
 * @brief Hard Fault Handler
 */
void HardFault_Handler(void)
{
    /* Handle hard fault */
    while(1)
    {
        /* Stay here or reset system */
    }
}

/**
 * @brief Bus Fault Handler
 */
void BusFault_Handler(void)
{
    /* Handle bus fault */
    while(1);
}

/**
 * @brief Usage Fault Handler
 */
void UsageFault_Handler(void)
{
    /* Handle usage fault */
    while(1);
}