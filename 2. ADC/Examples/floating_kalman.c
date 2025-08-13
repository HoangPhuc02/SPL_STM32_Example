#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"
#include <stdio.h>
#include <math.h>
#include "misc.h"
/**
 * =============================================================================
 * ADC with Kalman Filter for Voltage Measurement
 * =============================================================================
 * Features:
 * - Kalman Filter for noise reduction
 * - Selectable voltage range: 3.3V or 5V
 * - DMA-based ADC sampling
 * - UART output for monitoring
 * - Real-time voltage estimation
 * =============================================================================
 */

// Configuration
#define ADC_BUFFER_SIZE     50      // Buffer size for DMA
#define VREF_3V3           3300     // 3.3V in millivolts
#define VREF_5V            5000     // 5.0V in millivolts
#define ADC_RESOLUTION     4095     // 12-bit ADC (2^12 - 1)

// Voltage range selection
typedef enum {
    VOLTAGE_RANGE_3V3 = 0,
    VOLTAGE_RANGE_5V = 1
} voltage_range_t;

// Kalman Filter Structure
typedef struct {
    float x;            // State (estimated voltage)
    float P;            // Error covariance
    float Q;            // Process noise covariance
    float R;            // Measurement noise covariance
    float K;            // Kalman gain
    float x_pred;       // Predicted state
    float P_pred;       // Predicted error covariance
    uint8_t initialized; // Initialization flag
} kalman_filter_t;

// Global Variables
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t conversion_complete = 0;
volatile uint32_t sample_count = 0;

static kalman_filter_t voltage_filter;
static voltage_range_t current_range = VOLTAGE_RANGE_3V3;
static volatile float filtered_voltage = 0.0f;
static volatile float raw_voltage = 0.0f;

// UART buffer for output
char uart_buffer[200];

/* =============================================================================
 * KALMAN FILTER IMPLEMENTATION
 * =============================================================================
 */

/**
 * @brief Initialize Kalman Filter
 * @param kf: Pointer to Kalman Filter structure
 * @param initial_value: Initial voltage estimate
 * @param process_noise: Process noise (Q)
 * @param measurement_noise: Measurement noise (R)
 */
void Kalman_Init(kalman_filter_t* kf, float initial_value, float process_noise, float measurement_noise)
{
    kf->x = initial_value;          // Initial state
    kf->P = 1.0f;                   // Initial error covariance
    kf->Q = process_noise;          // Process noise (smaller = smoother)
    kf->R = measurement_noise;      // Measurement noise
    kf->initialized = 1;
}

/**
 * @brief Update Kalman Filter with new measurement
 * @param kf: Pointer to Kalman Filter structure  
 * @param measurement: New voltage measurement
 * @return Filtered voltage estimate
 */
float Kalman_Update(kalman_filter_t* kf, float measurement)
{
    if (!kf->initialized) {
        // First measurement - initialize with this value
        Kalman_Init(kf, measurement, 0.01f, 0.1f);
        return measurement;
    }
    
    // Prediction Step
    kf->x_pred = kf->x;                 // State prediction (assuming constant voltage)
    kf->P_pred = kf->P + kf->Q;         // Error covariance prediction
    
    // Update Step
    kf->K = kf->P_pred / (kf->P_pred + kf->R);  // Kalman gain
    kf->x = kf->x_pred + kf->K * (measurement - kf->x_pred);  // State update
    kf->P = (1.0f - kf->K) * kf->P_pred;        // Error covariance update
    
    return kf->x;
}

/**
 * @brief Reset Kalman Filter
 */
void Kalman_Reset(kalman_filter_t* kf)
{
    kf->initialized = 0;
    kf->x = 0.0f;
    kf->P = 1.0f;
}

/* =============================================================================
 * VOLTAGE MEASUREMENT FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Convert ADC value to voltage in millivolts
 * @param adc_value: Raw ADC reading (0-4095)
 * @return Voltage in millivolts
 */
float ADC_To_Voltage(uint16_t adc_value)
{
    uint16_t vref = (current_range == VOLTAGE_RANGE_3V3) ? VREF_3V3 : VREF_5V;
    return ((float)adc_value * vref) / ADC_RESOLUTION;
}

/**
 * @brief Set voltage measurement range
 * @param range: VOLTAGE_RANGE_3V3 or VOLTAGE_RANGE_5V
 */
void Set_Voltage_Range(voltage_range_t range)
{
    current_range = range;
    Kalman_Reset(&voltage_filter);  // Reset filter when changing range
}

/**
 * @brief Get current voltage range setting
 * @return Current voltage range
 */
voltage_range_t Get_Voltage_Range(void)
{
    return current_range;
}

/* =============================================================================
 * HARDWARE CONFIGURATION
 * =============================================================================
 */

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Enable GPIO clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
    
    // Configure ADC input pin (PA0) as analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure LED pin (PC13) for status indication
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // Configure button pin (PA1) for range switching
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure UART pins (PA9=TX, PA10=RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void UART_Config(void)
{
    // Enable USART1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
}

void DMA_Config(void)
{
    // Enable DMA1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADC_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    // Enable DMA transfer complete interrupt
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    
    // Configure NVIC for DMA interrupt
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC_Config(void)
{
    // Enable ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // Configure ADC channel (PA0 = ADC_Channel_0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
    
    // Enable ADC1 DMA
    ADC_DMACmd(ADC1, ENABLE);
    
    // Enable and calibrate ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void System_Clock_Config(void)
{
    // Configure system clock to 72MHz
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    if(RCC_WaitForHSEStartUp() == SUCCESS)
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);    // AHB clock = SYSCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);     // APB2 clock = HCLK
        RCC_PCLK1Config(RCC_HCLK_Div2);     // APB1 clock = HCLK/2
        
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // 8MHz * 9 = 72MHz
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource() != 0x08);
    }
}

/* =============================================================================
 * UTILITY FUNCTIONS
 * =============================================================================
 */

void UART_SendString(char* str)
{
    while(*str)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *str++);
    }
}

void Delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms * 8000; i++); // Approximate delay
}

uint8_t Button_Pressed(void)
{
    static uint8_t button_state = 1;  // Released state (pull-up)
    static uint32_t debounce_timer = 0;
    
    uint8_t current_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
    
    if(current_state == 0 && button_state == 1) // Button pressed
    {
        if(debounce_timer == 0)
        {
            debounce_timer = 1;
            button_state = 0;
            return 1;
        }
    }
    else if(current_state == 1)
    {
        button_state = 1;
        debounce_timer = 0;
    }
    
    return 0;
}

/* =============================================================================
 * INTERRUPT HANDLERS
 * =============================================================================
 */

void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1))
    {
        conversion_complete = 1;
        sample_count++;
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

/* =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */

int main(void)
{
    // System initialization
    System_Clock_Config();
    GPIO_Config();
    UART_Config();
    DMA_Config();
    ADC_Config();
    
    // Initialize Kalman filter
    Kalman_Init(&voltage_filter, 1650.0f, 0.01f, 50.0f); // Initial: 1.65V, Q=0.01, R=50mV
    
    // Send startup message
    sprintf(uart_buffer, "\r\n=== STM32 Voltage Meter with Kalman Filter ===\r\n");
    UART_SendString(uart_buffer);
    sprintf(uart_buffer, "Range: %s | Press button to switch\r\n", 
            (current_range == VOLTAGE_RANGE_3V3) ? "3.3V" : "5.0V");
    UART_SendString(uart_buffer);
    sprintf(uart_buffer, "Format: Raw(mV) | Filtered(mV) | Noise\r\n\r\n");
    UART_SendString(uart_buffer);
    
    // Start ADC conversions
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    uint32_t last_display_time = 0;
    uint32_t display_counter = 0;
    
    while(1)
    {
        // Check for button press to switch voltage range
        if(Button_Pressed())
        {
            if(current_range == VOLTAGE_RANGE_3V3)
            {
                Set_Voltage_Range(VOLTAGE_RANGE_5V);
                UART_SendString(">>> Switched to 5.0V range <<<\r\n");
            }
            else
            {
                Set_Voltage_Range(VOLTAGE_RANGE_3V3);
                UART_SendString(">>> Switched to 3.3V range <<<\r\n");
            }
            Delay_ms(200); // Debounce delay
        }
        
        // Process ADC data when buffer is filled
        if(conversion_complete)
        {
            conversion_complete = 0;
            
            // Calculate average of all samples in buffer (for better accuracy)
            uint32_t sum = 0;
            for(int i = 0; i < ADC_BUFFER_SIZE; i++)
            {
                sum += adc_buffer[i];
            }
            uint16_t avg_adc = sum / ADC_BUFFER_SIZE;
            
            // Convert to voltage
            raw_voltage = ADC_To_Voltage(avg_adc);
            
            // Apply Kalman filter
            filtered_voltage = Kalman_Update(&voltage_filter, raw_voltage);
            
            // Calculate noise (difference between raw and filtered)
            float noise = fabsf(raw_voltage - filtered_voltage);
            
            // Toggle LED every few measurements
            if(display_counter % 10 == 0)
            {
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, 
                             (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));
            }
        }
        
        // Display results every 100ms (approximately)
        if(display_counter++ >= 5) // Adjust frequency as needed
        {
            display_counter = 0;
            
            sprintf(uart_buffer, "Raw: %4.0f mV | Filtered: %4.0f mV | Noise: %3.0f mV | Range: %s\r\n",
                    raw_voltage, filtered_voltage, fabsf(raw_voltage - filtered_voltage),
                    (current_range == VOLTAGE_RANGE_3V3) ? "3.3V" : "5.0V");
            UART_SendString(uart_buffer);
        }
        
        Delay_ms(20); // Main loop delay
    }
}