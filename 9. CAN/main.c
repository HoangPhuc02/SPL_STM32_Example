/*
 * =============================================================================
 * Project: STM32F103 CAN Testing(SPL) - Modular Design with Remote & Error Frames
 * File: main.c
 * Author: hoangphuc540202@gmail.com
 * Github: https://github.com/hoangphuc540202
 * Date: August 2025
 * 
 * Description: Complete CAN implementation using modular design
 *              Supports Data Frames, Remote Frames, and Error Frames
 * 
 * Hardware Setup:
 * - STM32F103C8T6 Blue Pill board
 * - CAN pins: PA11 (RX), PA12 (TX)
 * - UART pins: PA9 (TX), PA10 (RX)
 * - Buttons: PA0 (Data), PA1 (Remote), PA2 (Error)
 * - LED: PC13
 * 
 * Button Functions:
 * - PA0: Send different types of data frames (Temperature, Humidity, Status)
 * - PA1: Send remote frame requests
 * - PA2: Simulate error conditions
 * 
 * Features:
 * - Modular CAN implementation
 * - Data Frame transmission/reception
 * - Remote Frame handling
 * - Error Frame simulation and handling
 * - Real-time statistics via UART
 * - Comprehensive error monitoring
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
#include "SPL/config.h"
#include "can_module.h"
#include <stdio.h>
#include <string.h>

/* ======================= Global Variables ======================= */
volatile uint8_t uart_dma_busy = 0;
uint8_t uart_tx_buffer[300];        // Increased buffer size for detailed messages

/* Button debounce variables */
uint8_t button_data_state = 0;
uint8_t button_remote_state = 0;
uint8_t button_error_state = 0;

/* System state variables */
uint32_t system_tick_counter = 0;
uint8_t current_data_type = 0;       // Cycle through data types
uint8_t display_stats_flag = 0;

/* ======================= Button Handling Functions ======================= */

/**
 * @brief Reads button state with debouncing
 * @param GPIOx: GPIO port (e.g., GPIOA, GPIOB, GPIOC)
 * @param GPIO_Pin: GPIO pin number (e.g., GPIO_Pin_0, GPIO_Pin_1)
 * @param button_state: Pointer to button state variable for debouncing
 * @return 1 if button press detected, 0 otherwise
 * 
 * This function implements simple software debouncing by tracking the previous
 * button state and only returning true on the falling edge (button press).
 * Assumes buttons are active LOW (pressed = 0V, released = 3.3V).
 */
uint8_t ReadButton(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* button_state)
{
    uint8_t button_pressed = 0;
    
    if (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_RESET) {
        if (*button_state == 0) {
            *button_state = 1;
            button_pressed = 1;
        }
    } else {
        *button_state = 0;
    }
    
    return button_pressed;
}

/* ======================= UART DMA Functions ======================= */

/**
 * @brief Initiates UART transmission using DMA
 * @param data_length: Number of bytes to transmit
 * 
 * This function configures and starts DMA transmission for UART.
 * It waits for any ongoing transmission to complete before starting
 * a new one, then reloads the DMA counter and enables the channel.
 */
void Start_UART_DMA_Transmission(uint16_t data_length)
{
    while (uart_dma_busy);
    uart_dma_busy = 1;
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = data_length;
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/**
 * @brief Sends debug message via UART using DMA
 * @param message: Null-terminated string to send
 * 
 * This function copies the message to the UART buffer, adds CRLF line
 * endings for proper terminal display, and initiates DMA transmission.
 * Includes buffer overflow protection.
 */
void SendDebugMessage(const char* message)
{
    uint16_t length = strlen(message);
    if (length > sizeof(uart_tx_buffer) - 3) {
        length = sizeof(uart_tx_buffer) - 3;
    }
    
    strcpy((char*)uart_tx_buffer, message);
    strcat((char*)uart_tx_buffer, "\r\n");
    
    Start_UART_DMA_Transmission(strlen((char*)uart_tx_buffer));
}

/* ======================= CAN Callback Functions ======================= */

/**
 * @brief Callback function for received CAN data frames
 * @param msg: Pointer to received CAN message structure
 * 
 * This callback is triggered when a CAN data frame is received.
 * It formats the message for debugging output via UART and toggles
 * the LED as a visual indication of data frame reception.
 */
void CAN_DataFrameReceived_Callback(CAN_Message_t* msg)
{
    // Format and send data frame info via UART
    uint16_t length = CAN_FormatMessageForDebug(msg, uart_tx_buffer, sizeof(uart_tx_buffer));
    Start_UART_DMA_Transmission(length);
    
    // Toggle LED on data frame receive
    GPIOC->ODR ^= GPIO_Pin_13;
}

/**
 * @brief Callback function for received CAN remote frames
 * @param id: CAN message identifier of the remote frame
 * @param dlc: Data Length Code (requested data length)
 * 
 * This callback is triggered when a CAN remote frame is received.
 * Remote frames are requests for data - no actual data is included.
 * The function sends debug info via UART and blinks LED rapidly.
 */
void CAN_RemoteFrameReceived_Callback(CAN_MessageID_t id, uint8_t dlc)
{
    // Create a message for remote frame
    sprintf((char*)uart_tx_buffer, "REMOTE REQ: ID=0x%03X DLC=%d\r\n", (unsigned int)id, dlc);
    Start_UART_DMA_Transmission(strlen((char*)uart_tx_buffer));
    
    // Fast LED blink for remote frame
    for (int i = 0; i < 4; i++) {
        GPIOC->ODR ^= GPIO_Pin_13;
        Delay(50000);
    }
}

/**
 * @brief Callback function for CAN error detection
 * @param error_type: Type of CAN error detected (enumerated type)
 * 
 * This callback is triggered when CAN errors are detected.
 * It translates error codes to human-readable strings and sends
 * error information via UART. Keeps LED on to indicate error state.
 */
void CAN_ErrorDetected_Callback(CAN_ErrorType_t error_type)
{
    const char* error_names[] = {
        "NONE", "STUFF", "FORM", "ACK", "BIT_REC", "BIT_DOM", "CRC", "BUS_OFF", "PASSIVE"
    };
    
    sprintf((char*)uart_tx_buffer, "CAN ERROR: %s\r\n", error_names[error_type]);
    Start_UART_DMA_Transmission(strlen((char*)uart_tx_buffer));
    
    // Keep LED on for errors
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

/* ======================= Utility Functions ======================= */

/**
 * @brief Simple software delay function
 * @param delay: Number of loop iterations for delay
 * 
 * This is a blocking delay function that burns CPU cycles.
 * The actual time depends on system clock frequency and compiler
 * optimization. Used for simple timing where precision isn't critical.
 */
void Delay(uint32_t delay)
{
    while (delay--);
}

/**
 * @brief Displays comprehensive system status via UART
 * 
 * This function retrieves and displays current CAN system statistics
 * including transmission/reception counts, error counts, sensor data,
 * and system state. Useful for debugging and system monitoring.
 */
void DisplaySystemInfo(void)
{
    CAN_Statistics_t* stats = CAN_GetStatistics();
    CAN_SensorData_t* sensor = CAN_GetSensorData();
    
    sprintf((char*)uart_tx_buffer, 
            "=== CAN System Status ===\r\n"
            "TX: %lu, RX: %lu, Remote TX: %lu, Remote RX: %lu\r\n"
            "Errors: %lu, Bus-Off: %lu\r\n"
            "Temperature: %lu.%02lu°C, Humidity: %lu.%02lu%%\r\n"
            "Counter: %lu, Last Error: %d\r\n"
            "========================\r\n",
            stats->tx_count, stats->rx_count, 
            stats->remote_tx_count, stats->remote_rx_count,
            stats->error_count, stats->bus_off_count,
            sensor->temperature / 100, sensor->temperature % 100,
            sensor->humidity / 100, sensor->humidity % 100,
            sensor->transmission_counter, sensor->last_error);
    
    Start_UART_DMA_Transmission(strlen((char*)uart_tx_buffer));
}

/**
 * @brief Processes data frame button press (PA0)
 * 
 * This function cycles through different types of CAN data frames:
 * - Temperature data frame
 * - Humidity data frame  
 * - Status data frame
 * Each button press sends the next type in sequence and updates sensor data.
 */
void ProcessDataFrameButton(void)
{
    static uint8_t data_type_cycle = 0;
    
    switch(data_type_cycle) {
        case 0:
            CAN_SendDataFrame(CAN_ID_TEMPERATURE, CAN_DATA_TEMPERATURE, NULL, 8);
            SendDebugMessage("Sent: Temperature Data Frame");
            break;
        case 1:
            CAN_SendDataFrame(CAN_ID_HUMIDITY, CAN_DATA_HUMIDITY, NULL, 8);
            SendDebugMessage("Sent: Humidity Data Frame");
            break;
        case 2:
            CAN_SendDataFrame(CAN_ID_STATUS, CAN_DATA_STATUS, NULL, 8);
            SendDebugMessage("Sent: Status Data Frame");
            break;
    }
    
    data_type_cycle = (data_type_cycle + 1) % 3;
    CAN_UpdateSensorData(); // Update sensor values
}

/**
 * @brief Processes remote frame button press (PA1)
 * 
 * This function cycles through different types of CAN remote frame requests:
 * - Temperature data request
 * - Humidity data request
 * - Status data request
 * Remote frames are used to request specific data from other CAN nodes.
 */
void ProcessRemoteFrameButton(void)
{
    static uint8_t remote_type_cycle = 0;
    
    switch(remote_type_cycle) {
        case 0:
            CAN_SendRemoteFrame(CAN_ID_TEMPERATURE, 8);
            SendDebugMessage("Sent: Remote Request for Temperature");
            break;
        case 1:
            CAN_SendRemoteFrame(CAN_ID_HUMIDITY, 8);
            SendDebugMessage("Sent: Remote Request for Humidity");
            break;
        case 2:
            CAN_SendRemoteFrame(CAN_ID_STATUS, 8);
            SendDebugMessage("Sent: Remote Request for Status");
            break;
    }
    
    remote_type_cycle = (remote_type_cycle + 1) % 3;
}

/**
 * @brief Processes error frame button press (PA2)
 * 
 * This function cycles through different types of CAN error simulations:
 * - Stuff error (bit stuffing violation)
 * - Form error (frame format violation)
 * - Acknowledgment error (no ACK received)
 * - CRC error (checksum mismatch)
 * - Error passive state
 * Used for testing error handling and recovery mechanisms.
 */
void ProcessErrorFrameButton(void)
{
    static uint8_t error_type_cycle = 0;
    CAN_ErrorType_t error_types[] = {
        CAN_ERROR_STUFF, CAN_ERROR_FORM, CAN_ERROR_ACK, 
        CAN_ERROR_CRC, CAN_ERROR_PASSIVE
    };
    
    CAN_SendErrorFrame(error_types[error_type_cycle]);
    
    sprintf((char*)uart_tx_buffer, "Sent: Error Frame Type %d\r\n", error_types[error_type_cycle]);
    Start_UART_DMA_Transmission(strlen((char*)uart_tx_buffer));
    
    error_type_cycle = (error_type_cycle + 1) % 5;
}

/* ======================= Enhanced GPIO Configuration ======================= */

/**
 * @brief Configures GPIO pins for buttons and LED
 * 
 * This function initializes:
 * - PA0, PA1, PA2 as input pins with internal pull-up resistors (buttons)
 * - PC13 as push-pull output pin (onboard LED)
 * Button pins are configured as active-LOW with pull-up resistors.
 */
void GPIO_Config_Extended(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure button pins PA0, PA1, PA2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // Input with pull-up
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure LED pin PC13 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // Push-pull output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13); // LED off initially
}

/* ======================= Interrupt Handlers ======================= */

/**
 * @brief CAN RX FIFO 0 interrupt handler
 * 
 * This interrupt is triggered when a CAN message is received in FIFO 0.
 * It reads the message from the FIFO and processes it using the CAN module.
 * The interrupt flag is cleared to prevent continuous triggering.
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET) {
        CanRxMsg RxMessage;
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        // Process message using CAN module
        CAN_ProcessReceivedMessage(&RxMessage);
        
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/**
 * @brief CAN status change and error interrupt handler
 * 
 * This interrupt handles CAN error conditions and status changes:
 * - Bus errors (stuff, form, ACK, bit, CRC errors)
 * - Bus-off condition (node disconnected from bus)
 * - Error passive state (high error count)
 * - Error warning (moderate error count)
 */
void CAN1_SCE_IRQHandler(void)
{
    // Check for various CAN errors using module function
    CAN_CheckAndHandleErrors();
    
    // Clear all error interrupt flags
    if (CAN_GetITStatus(CAN1, CAN_IT_ERR) == SET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
    }
    if (CAN_GetITStatus(CAN1, CAN_IT_BOF) == SET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
    }
    if (CAN_GetITStatus(CAN1, CAN_IT_EPV) == SET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
    }
    if (CAN_GetITStatus(CAN1, CAN_IT_EWG) == SET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
    }
}

/**
 * @brief DMA Channel 4 transfer complete interrupt handler
 * 
 * This interrupt is triggered when UART DMA transmission is complete.
 * It clears the busy flag to allow new transmissions and clears the
 * interrupt flag. Channel 4 is used for USART1 TX DMA.
 */
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) == SET) {
        uart_dma_busy = 0;
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}

/* ======================= Main Function ======================= */
/**
 * @brief Main application function with enhanced CAN functionality
 * 
 * Program Flow:
 * 1. Initialize system and peripherals using config API
 * 2. Initialize CAN module with extended features
 * 3. Display startup message
 * 4. Main loop:
 *    - Handle button presses for different frame types
 *    - Periodic system status display
 *    - Error monitoring and handling
 *    - Sensor data updates
 * 
 * The main loop continuously monitors button states, handles CAN operations,
 * and performs periodic maintenance tasks like status display and error checking.
 * This creates a complete CAN testing and demonstration system.
 */
int main(void)
{
    /* System Initialization */
    SystemInit();                    // Setup system clock to 72MHz
    
    /* Initialize peripherals using existing config system */
    GPIO_Config_Extended();          // Extended GPIO configuration for multiple buttons
    DMA_Config();                    // Configure DMA for UART transmission
    UART_Config();                   // Configure USART1 for debug output
    
    /* Initialize CAN module with enhanced features */
    CAN_Module_Init();               // Initialize CAN module (uses CAN_Config internally)
    
    /* Enable global interrupts */
    __enable_irq();
    
    /* Startup indication */
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // LED on
    Delay(1000000);                      // 1 second delay
    GPIO_SetBits(GPIOC, GPIO_Pin_13);    // LED off
    
    /* Send startup message */
    SendDebugMessage("=== STM32 CAN Module Test Started ===");
    SendDebugMessage("PA0: Data Frame, PA1: Remote Frame, PA2: Error Frame");
    
    /* Display initial system info */
    DisplaySystemInfo();
    
    /* Application Main Loop */
    while (1) {
        /* Button PA0: Send Data Frame */
        if (ReadButton(GPIOA, GPIO_Pin_0, &button_data_state)) {
            ProcessDataFrameButton();
        }
        
        /* Button PA1: Send Remote Frame */
        if (ReadButton(GPIOA, GPIO_Pin_1, &button_remote_state)) {
            ProcessRemoteFrameButton();
        }
        
        /* Button PA2: Send Error Frame */
        if (ReadButton(GPIOA, GPIO_Pin_2, &button_error_state)) {
            ProcessErrorFrameButton();
        }
        
        /* Periodic system status display (every ~10 seconds) */
        system_tick_counter++;
        if (system_tick_counter >= 100000) {  // Adjust based on loop frequency
            DisplaySystemInfo();
            system_tick_counter = 0;
        }
        
        /* Update sensor data periodically */
        if ((system_tick_counter % 5000) == 0) {
            CAN_UpdateSensorData();
        }
        
        /* Check for CAN errors */
        if ((system_tick_counter % 1000) == 0) {
            CAN_CheckAndHandleErrors();
        }
        
        /* Main loop delay */
        Delay(1000);  // Adjust this value based on your requirements
    }
}

/* ======================= End of File ======================= */
