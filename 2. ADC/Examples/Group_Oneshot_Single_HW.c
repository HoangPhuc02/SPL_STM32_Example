/**********************************************************
* File Name   : Group_Oneshot_Single_HW.c
* Module      : ADC
* Description : this file is part of the ADC example for STM32F103C8T6
* Details     : This file demonstrates the use of ADC in one-shot single mode for a group of channels with hardware trigger.
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
// extern const Port_ConfigType PortCfg_Port; 
void delay(uint16_t time)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < time)
        ;
}

/* Function declarations */
void DMA_Config(void);
void Timer_Init(void);
void TimerAdc_Init(void);
void Adc_PinConfig(void);
void Adc_Config(void);
void NVIC_Config(void);

uint16_t adcValue = 0;  // Legacy variable kept for compatibility
volatile uint16_t buffer[8] = {0}; // Buffer for DMA ADC values, marked volatile for debugging & watch
volatile uint16_t* buffer_reoder[8] = {0}; // Buffer for reordered ADC values, marked volatile for debugging & watch
/*
 * DMA and ADC interrupt handlers for interrupt-based operation
 */

volatile uint8_t dma_conversion_complete = 0; // Flag to indicate conversion is complete
volatile uint8_t i = 0;
volatile uint8_t flag = 0;
void switch_channel_test(void);
void buffer_reorder_init(void)
{
    // Reorder the buffer for display or further processing
    // This function can be customized based on the application needs
    buffer_reoder[0] = &buffer[0];
    buffer_reoder[1] = &buffer[2];
    buffer_reoder[2] = &buffer[4];
    buffer_reoder[3] = &buffer[6];
    buffer_reoder[4] = &buffer[1];
    buffer_reoder[5] = &buffer[3];
    buffer_reoder[6] = &buffer[5];
    buffer_reoder[7] = &buffer[7];
}
void ADC1_2_IRQHandler(void)
{
    // Check if the ADC conversion is complete
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        // Read the converted value - not needed when using DMA
        // adcValue = ADC_GetConversionValue(ADC1);
        flag =! flag;
        buffer[i++] = ADC_GetConversionValue(ADC1);
        if (i == 8)
        {
            i = 0;
        }
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
        switch_channel_test(); // Switch channel for next conversion
        // Clear the ADC interrupt flag
        
        // ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
        // NVIC_DisableIRQ(ADC1_2_IRQn);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    // Check if the DMA transfer is complete
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        // Set flag to indicate conversion is complete
        //  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5); // ADC Channel 1 (PA1)
        flag = !flag;
        // ADC_Cmd(ADC1, DISABLE);
        // Clear the DMA transfer complete flag
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}
int main()
{
     volatile uint16_t voltageInMv[2] = {0, 0};  // Array for two voltage readings in millivolts
    buffer_reorder_init(); // Initialize the buffer for reordered ADC values
    Timer_Init();      // Initialize Timer for delay
    TimerAdc_Init();   // Initialize Timer for ADC
    Adc_PinConfig();   // Configure ADC pins (PA0 and PA1)
    

    DMA_Config();      // Configure DMA for ADC
    Adc_Config();      // Configure ADC settings
    // NVIC_Config();
    
    // Enable DMA Channel 1 (CRITICAL - this must be done before starting ADC)
     DMA_Cmd(DMA1_Channel1, ENABLE);
    
    /* Start ADC conversion in continuous mode */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    while (1)
    {

    }
}
void DMA_Config(void)
{
    // Configure DMA for ADC
    DMA_InitTypeDef dma;
    
    // Reset DMA1 Channel1 config
    DMA_DeInit(DMA1_Channel1);
    
    // Basic settings
    dma.DMA_Mode = DMA_Mode_Circular;           // Circular mode for continuous updates
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;        // Peripheral to memory
    dma.DMA_M2M = DMA_M2M_Disable;              // Not memory-to-memory
    dma.DMA_Priority = DMA_Priority_High;       // High priority
    dma.DMA_BufferSize = 8;                     // Number of ADC channels to read
    
    // Peripheral settings
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;  // ADC data register address
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // 16-bit data
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Don't increment peripheral address
    
    // Memory settings
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;          // Increment memory address
    dma.DMA_MemoryBaseAddr = (uint32_t)buffer;         // Buffer address
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  // 16-bit data
    
    // Initialize DMA
    DMA_Init(DMA1_Channel1, &dma);
    
    // Enable DMA Transfer Complete interrupt
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    
    // Configure NVIC for DMA interrupts
    NVIC_Config();
}
/* 
 * NVIC configuration for DMA interrupt-based operation
 */
void NVIC_Config(void)
{
    NVIC_InitTypeDef nvic;
    
    // Configure DMA1 Channel1 interrupt (for ADC)
    nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1; // Highest priority
    nvic.NVIC_IRQChannelSubPriority = 0; 
    NVIC_Init(&nvic);
    
    // Optionally, you can also enable the ADC interrupt if needed
    
    // nvic.NVIC_IRQChannel = ADC1_2_IRQn;
    // nvic.NVIC_IRQChannelCmd = ENABLE;
    // nvic.NVIC_IRQChannelPreemptionPriority = 0;
    // nvic.NVIC_IRQChannelSubPriority = 0;
    // NVIC_Init(&nvic);
    
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
void switch_channel_test()
{
    if(flag == 0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5); // ADC Channel 0 (PA0)
    else
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_28Cycles5); // ADC Channel 0 (PA0)
}
void Adc_Config(void)
{
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;    // Independent mode
    adc.ADC_NbrOfChannel = 2;               // Number of channels to be converted
    // adc.ADC_ScanConvMode = DISABLE;          // Multi channel conversion
    adc.ADC_ScanConvMode = ENABLE;          // Multi channel conversion
    adc.ADC_ContinuousConvMode = ENABLE;    // CHANGED: Continuous mode for DMA
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // No external trigger
    adc.ADC_DataAlign = ADC_DataAlign_Right; // Right alignment

    ADC_Init(ADC1, &adc);
    
    // Configure two different channels (PA0 and PA1)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5); // ADC Channel 0 (PA0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5); // ADC Channel 1 (PA1)
    
    // Enable DMA for ADC1 - must be done before enabling ADC
    ADC_DMACmd(ADC1, ENABLE);
    // ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    
    // Calibrate ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

