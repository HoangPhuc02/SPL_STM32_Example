/*
 * =============================================================================
 * ADC with Integer-Based Kalman Filter for Voltage Measurement
 * =============================================================================
 * Features:
 * - Integer-only Kalman Filter (no floating point)
 * - Selectable voltage range: 3.3V or 5V
 * - DMA-based ADC sampling
 * - UART output for monitoring
 * - Fixed-point arithmetic for STM32F103
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"

// Configuration
#define ADC_BUFFER_SIZE     50      // Buffer size for DMA
#define VREF_3V3           3300     // 3.3V in millivolts
#define VREF_5V            5000     // 5.0V in millivolts
#define ADC_RESOLUTION     4095     // 12-bit ADC (2^12 - 1)

// Fixed-point arithmetic scaling (Q16.16 format)
#define FIXED_SCALE        65536    // 2^16 for fixed-point
#define FIXED_MUL(a, b)    (((int64_t)(a) * (b)) >> 16)
#define FIXED_DIV(a, b)    (((int64_t)(a) << 16) / (b))
#define INT_TO_FIXED(x)    ((int32_t)(x) << 16)
#define FIXED_TO_INT(x)    ((int32_t)(x) >> 16)

// Voltage range selection
typedef enum {
    VOLTAGE_RANGE_3V3 = 0,
    VOLTAGE_RANGE_5V = 1
} voltage_range_t;

// Integer Kalman Filter Structure (all values in fixed-point Q16.16)
typedef struct {
    int32_t x;          // State (estimated voltage in mV, fixed-point)
    int32_t P;          // Error covariance (fixed-point)
    int32_t Q;          // Process noise covariance (fixed-point)
    int32_t R;          // Measurement noise covariance (fixed-point)
    int32_t K;          // Kalman gain (fixed-point)
    int32_t x_pred;     // Predicted state
    int32_t P_pred;     // Predicted error covariance
    uint8_t initialized;// Initialization flag
} kalman_filter_int_t;

// Global Variables
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t conversion_complete = 0;
volatile uint32_t sample_count = 0;

static kalman_filter_int_t voltage_filter;
static voltage_range_t current_range = VOLTAGE_RANGE_3V3;
static volatile int32_t filtered_voltage_mv = 0;    // in millivolts
static volatile int32_t raw_voltage_mv = 0;         // in millivolts

// UART buffer for output
char uart_buffer[200];

/* =============================================================================
 * INTEGER KALMAN FILTER IMPLEMENTATION
 * =============================================================================
 */

/**
 * @brief Initialize Integer Kalman Filter
 * @param kf: Pointer to Kalman Filter structure
 * @param initial_value_mv: Initial voltage estimate in mV
 * @param process_noise: Process noise (Q) in fixed-point
 * @param measurement_noise: Measurement noise (R) in fixed-point
 */
void Kalman_Init_Int(kalman_filter_int_t* kf, int32_t initial_value_mv, int32_t process_noise, int32_t measurement_noise)
{
    kf->x = INT_TO_FIXED(initial_value_mv);     // Initial state in fixed-point
    kf->P = INT_TO_FIXED(100);                  // Initial error covariance
    kf->Q = process_noise;                      // Process noise (small = smooth)
    kf->R = measurement_noise;                  // Measurement noise
    kf->initialized = 1;
}

/**
 * @brief Update Integer Kalman Filter with new measurement
 * @param kf: Pointer to Kalman Filter structure  
 * @param measurement_mv: New voltage measurement in mV
 * @return Filtered voltage estimate in mV
 */
int32_t Kalman_Update_Int(kalman_filter_int_t* kf, int32_t measurement_mv)
{
    int32_t measurement_fixed = INT_TO_FIXED(measurement_mv);
    
    if (!kf->initialized) {
        // First measurement - initialize with this value
        Kalman_Init_Int(kf, measurement_mv, INT_TO_FIXED(1), INT_TO_FIXED(50));
        return measurement_mv;
    }
    
    // Prediction Step
    kf->x_pred = kf->x;                         // State prediction (constant voltage)
    kf->P_pred = kf->P + kf->Q;                 // Error covariance prediction
    
    // Update Step
    int32_t denominator = kf->P_pred + kf->R;
    kf->K = FIXED_DIV(kf->P_pred, denominator); // Kalman gain
    
    int32_t innovation = measurement_fixed - kf->x_pred;
    kf->x = kf->x_pred + FIXED_MUL(kf->K, innovation);  // State update
    
    int32_t one_minus_k = INT_TO_FIXED(1) - kf->K;
    kf->P = FIXED_MUL(one_minus_k, kf->P_pred);         // Error covariance update
    
    return FIXED_TO_INT(kf->x);
}

/**
 * @brief Reset Integer Kalman Filter
 */
void Kalman_Reset_Int(kalman_filter_int_t* kf)
{
    kf->initialized = 0;
    kf->x = 0;
    kf->P = INT_TO_FIXED(100);
}

/* =============================================================================
 * VOLTAGE MEASUREMENT FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Convert ADC value to voltage in millivolts (integer math)
 * @param adc_value: Raw ADC reading (0-4095)
 * @return Voltage in millivolts
 */
int32_t ADC_To_Voltage_Int(uint16_t adc_value)
{
    uint32_t vref = (current_range == VOLTAGE_RANGE_3V3) ? VREF_3V3 : VREF_5V;
    return (((uint32_t)adc_value * vref) / ADC_RESOLUTION);
}

/**
 * @brief Set voltage measurement range
 * @param range: VOLTAGE_RANGE_3V3 or VOLTAGE_RANGE_5V
 */
void Set_Voltage_Range(voltage_range_t range)
{
    current_range = range;
    Kalman_Reset_Int(&voltage_filter);  // Reset filter when changing range
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
 * SIMPLE MATH UTILITIES
 * =============================================================================
 */

/**
 * @brief Integer absolute value
 */
int32_t abs_int32(int32_t x)
{
    return (x < 0) ? -x : x;
}

/**
 * @brief Simple integer sprintf replacement for voltage display
 */
void int_to_string(int32_t value, char* buffer)
{
    if (value < 0) {
        *buffer++ = '-';
        value = -value;
    }
    
    // Extract digits
    uint32_t temp = value;
    uint32_t divisor = 1;
    while (temp >= 10) {
        temp /= 10;
        divisor *= 10;
    }
    
    // Convert to string
    while (divisor > 0) {
        *buffer++ = '0' + (value / divisor);
        value %= divisor;
        divisor /= 10;
    }
    *buffer = '\0';
}

/* =============================================================================
 * HARDWARE CONFIGURATION (Same as before)
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
    SystemInit(); // Use SystemInit from system_stm32f10x.c
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

void UART_SendChar(char c)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, c);
}

void Delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms * 8000; i++); // Approximate delay for 72MHz
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
 * SIMPLE STRING FUNCTIONS (No sprintf needed)
 * =============================================================================
 */

void UART_SendNumber(int32_t number)
{
    char buffer[12];
    int_to_string(number, buffer);
    UART_SendString(buffer);
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
    
    // Initialize Integer Kalman filter
    Kalman_Init_Int(&voltage_filter, 1650, INT_TO_FIXED(1), INT_TO_FIXED(50)); // 1.65V, Q=1, R=50mV
    
    // Send startup message
    UART_SendString("\r\n=== STM32 Voltage Meter with Integer Kalman Filter ===\r\n");
    UART_SendString("Range: ");
    UART_SendString((current_range == VOLTAGE_RANGE_3V3) ? "3.3V" : "5.0V");
    UART_SendString(" | Press button to switch\r\n");
    UART_SendString("Format: Raw(mV) | Filtered(mV) | Noise\r\n\r\n");
    
    // Start ADC conversions
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
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
            
            // Calculate average of all samples in buffer
            uint32_t sum = 0;
            for(int i = 0; i < ADC_BUFFER_SIZE; i++)
            {
                sum += adc_buffer[i];
            }
            uint16_t avg_adc = sum / ADC_BUFFER_SIZE;
            
            // Convert to voltage (integer)
            raw_voltage_mv = ADC_To_Voltage_Int(avg_adc);
            
            // Apply Integer Kalman filter
            filtered_voltage_mv = Kalman_Update_Int(&voltage_filter, raw_voltage_mv);
            
            // Calculate noise (difference between raw and filtered)
            int32_t noise = abs_int32(raw_voltage_mv - filtered_voltage_mv);
            
            // Toggle LED every few measurements
            if(display_counter % 10 == 0)
            {
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, 
                             (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));
            }
        }
        
        // Display results every few loops
        if(display_counter++ >= 5)
        {
            display_counter = 0;
            
            UART_SendString("Raw: ");
            UART_SendNumber(raw_voltage_mv);
            UART_SendString(" mV | Filtered: ");
            UART_SendNumber(filtered_voltage_mv);
            UART_SendString(" mV | Noise: ");
            UART_SendNumber(abs_int32(raw_voltage_mv - filtered_voltage_mv));
            UART_SendString(" mV | Range: ");
            UART_SendString((current_range == VOLTAGE_RANGE_3V3) ? "3.3V" : "5.0V");
            UART_SendString("\r\n");
        }
        
        Delay_ms(20); // Main loop delay
    }
}
