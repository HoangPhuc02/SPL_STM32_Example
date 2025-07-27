/**********************************************************
* File Name   : Group_Oneshot_Scan_SW.c
* Module      : ADC
* Description : This file is part of the ADC example for STM32F103C8T6
* Details     : This file demonstrates the use of ADC in one-shot scan mode for a group of channels with software trigger.
* Version     : 1.0.0
* Date        : 26/06/2025
* Author      : hoangphuc540202@gmail.com
* Github      : https://github.com/HoangPhuc02
 **********************************************************/
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "misc.h" // For NVIC configuration


// ADC channel
typedef enum 
{
    ADC_CHANNEL_0 = 0, /*!< ADC Channel 0 */
    ADC_CHANNEL_1,     /*!< ADC Channel 1 */
    ADC_CHANNEL_2,     /*!< ADC Channel 2 */
    ADC_CHANNEL_3,     /*!< ADC Channel 3 */
    ADC_CHANNEL_4,     /*!< ADC Channel 4 */
    ADC_CHANNEL_5,     /*!< ADC Channel 5 */
    ADC_CHANNEL_6,     /*!< ADC Channel 6 */
    ADC_CHANNEL_7,     /*!< ADC Channel 7 */
    ADC_CHANNEL_8,     /*!< ADC Channel 8 */
    ADC_CHANNEL_9,     /*!< ADC Channel 9 */
    ADC_CHANNEL_10,    /*!< ADC Channel 10 */
    ADC_CHANNEL_11,    /*!< ADC Channel 11 */
    ADC_CHANNEL_12,    /*!< ADC Channel 12 */
    ADC_CHANNEL_13,    /*!< ADC Channel 13 */
    ADC_CHANNEL_14,    /*!< ADC Channel 14 */
    ADC_CHANNEL_15     /*!< ADC Channel 15 */
}Adc_ChannelId;
#define MAX_CHANNEL_GROUP 1
Adc_ChannelId adcChannel[MAX_CHANNEL_GROUP] = {ADC_CHANNEL_0}; // Array of ADC channels to read
uint16_t buffer_result[MAX_CHANNEL_GROUP] = {0}; // Buffer for ADC conversion results

volatile uint32_t last_trigger_time = 0;
volatile uint32_t adc_triggered = 0;

#define ADC_NOTIFY_ENABLE  1 // Enable notification for ADC conversion completion
#if ADC_NOTIFY_ENABLE
volatile uint8_t conversion_complete = 0; // Flag to indicate conversion completion
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        last_trigger_time = TIM_GetCounter(TIM2); // Lưu thời điểm trigger
        adc_triggered = 1;
        buffer_result[0] = ADC_GetConversionValue(ADC1); // Read ADC value for Channel 0 (PA0)
        // Set the conversion complete flag
        conversion_complete = 1;
        
        // Clear the interrupt flag
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}
void NVIC_Config(void)
{
    NVIC_InitTypeDef nvic;
    
    // Configure ADC1_2 interrupt
    nvic.NVIC_IRQChannel = ADC1_2_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 0; // Highest priority
    nvic.NVIC_IRQChannelSubPriority = 0; 
    NVIC_Init(&nvic);
}
#endif



// extern const Port_ConfigType PortCfg_Port; 
void delay(uint16_t time)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < time);
}

/* Function declarations */
void Timer_Init(void);
void TimerAdc_Init(void);
void Adc_PinConfig(void);
void Adc_Config(void);


#define SAMPLING_FREQUENCY 1000  // Hz

int main()
{
    volatile uint16_t voltageInMv[1] = {0};  // Array for two voltage readings in millivolts

    Timer_Init();      // Initialize Timer for delay
    TimerAdc_Init();   // Initialize Timer for ADC
    Adc_PinConfig();   // Configure ADC pins (PA0 and PA1)
    
    Adc_Config();      // Configure ADC settings
    #if ADC_NOTIFY_ENABLE
    NVIC_Config();     // Configure NVIC for ADC interrupts
    #endif
    /* Start ADC conversion in continuous mode */
    // ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    while (1)
    {
        // Wait for DMA transfer to complete (interrupt-based approach)
        #if ADC_NOTIFY_ENABLE
        if (conversion_complete)
        {
            // Reset the flag for next conversion
            conversion_complete = 0;
            
            // Process the ADC values that were updated by DMA
            voltageInMv[0] = (buffer_result[0] * 3300) / 4095;  // Channel 0 (PA0) in millivolts
    
            
            // Small delay to make values visible in Live Watch
            // This is not needed for functionality but helps with debugging
            delay(100);
        }
        #else
        buffer_result[0] = ADC_GetConversionValue(ADC1); // Read ADC value for Channel 0 (PA0)
        // Polling method for ADC conversion completion
        voltageInMv[0] = (buffer_result[0] * 3300) / 4095;  // Channel 0 (PA0) in millivolts
        #endif
    }
}

void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 0xFFFF;
    tim.TIM_Prescaler = 8000 - 1; // Prescaler for 1ms tick
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2, ENABLE);
}

void TimerAdc_Init(void)
{
    //Clock for ADC pin
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    uint16_t prescaler;
    uint16_t period;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    
    // Enable timer clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Calculate prescaler & period for desired frequency
    // System clock / (prescaler+1) / (period+1) = sampling frequency
    prescaler = (SystemCoreClock / 1000000) - 1; // 1MHz timer clock
    period = (1000000 / SAMPLING_FREQUENCY) - 1;
    
    // Configure timer base
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // Configure output compare to generate a pulse
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = period / 2; // 50% duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    
    // Enable TIM3 output trigger for ADC
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

    // Enable timer
    TIM_Cmd(TIM3, ENABLE);
}
void Adc_PinConfig(void)
{
    GPIO_InitTypeDef gpio;
    
    // Configure PA0 (ADC Channel 0) and PA1 (ADC Channel 1)
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  // Both pins for ADC
    gpio.GPIO_Mode = GPIO_Mode_AIN;           // Analog input mode
    gpio.GPIO_Speed = GPIO_Speed_50MHz;       // Not used for analog mode
    GPIO_Init(GPIOA, &gpio);
}
void Adc_Config(void)
{
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;    // Independent mode
    adc.ADC_NbrOfChannel = 1;               // Number of channels to be converted
    adc.ADC_ScanConvMode = ENABLE;         // Single channel conversion
    adc.ADC_ContinuousConvMode = DISABLE;   // One-shot mode for DMA
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO; // No external trigger
    adc.ADC_DataAlign = ADC_DataAlign_Right; // Right alignment

    ADC_Init(ADC1, &adc);
    
    // Configure two different channels (PA0 and PA1)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5); // ADC Channel 0 (PA0)
    #if ADC_NOTIFY_ENABLE
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // Enable End of Conversion interrupt
    #endif
    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    
    // Calibrate ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

