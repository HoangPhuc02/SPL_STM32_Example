/**********************************************************
* File Name   : adc_8samples_optimized
* Description : ADC with 8 samples buffering and notification after each conversion
* Details     : Optimized solution using DMA Half Transfer + Transfer Complete
* Version     : 1.0.0
* Date        : 22/07/2025
 **********************************************************/

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "misc.h" // For NVIC configuration
#include <stdint.h>

volatile uint16_t adcBuffer[8] = {0}; // Buffer for 8 ADC samples
volatile uint32_t conversionCount = 0;
volatile uint32_t currentSample = 0;
volatile uint8_t bufferReady = 0;

void delay(uint16_t time)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < time);
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

void Clock_Init(void)
{
    // Clock for ADC pin
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void Adc_PinConfig(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0; // ADC Channel 0
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz; // Not used for analog mode
    GPIO_Init(GPIOA, &gpio);
}

void NVIC_Config(void)
{
    NVIC_InitTypeDef nvic;
    
    // DMA interrupt configuration
    nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void Adc_Config(void)
{
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    
    // Enable DMA for ADC (không cần EOC interrupt)
    ADC_DMACmd(ADC1, ENABLE);
    
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
    /* Calibrate ADC */
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void DMA_Config(void)
{
    DMA_InitTypeDef dma;
    
    // Reset DMA1 Channel1 config
    DMA_DeInit(DMA1_Channel1);
    
    // Basic settings
    dma.DMA_Mode = DMA_Mode_Circular;           // Circular mode
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;        // Peripheral to memory
    dma.DMA_M2M = DMA_M2M_Disable;              // Not memory-to-memory
    dma.DMA_Priority = DMA_Priority_High;       // High priority
    dma.DMA_BufferSize = 8;                     // 8 samples buffer
    
    // Peripheral settings
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    
    // Memory settings
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_MemoryBaseAddr = (uint32_t)adcBuffer;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    
    // Initialize DMA
    DMA_Init(DMA1_Channel1, &dma);
    
    // Enable both Half Transfer and Transfer Complete interrupts
    DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);  // Half Transfer (sau 4 samples)
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);  // Transfer Complete (sau 8 samples)
    
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

// DMA Interrupt Handler
void DMA1_Channel1_IRQHandler(void)
{
    // Half Transfer Complete - sau 4 samples đầu
    if (DMA_GetITStatus(DMA1_IT_HT1))
    {
        // Xử lý thông báo cho 4 samples đầu
        for(int i = 0; i < 4; i++)
        {
            conversionCount++;
            // Ghi thông báo cho mỗi sample
            // Log: "Sample %d: %d mV", conversionCount, (adcBuffer[i] * 3300) / 4095
        }
        DMA_ClearITPendingBit(DMA1_IT_HT1);
    }
    
    // Transfer Complete - sau 8 samples đầy đủ
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        // Xử lý thông báo cho 4 samples cuối
        for(int i = 4; i < 8; i++)
        {
            conversionCount++;
            // Ghi thông báo cho mỗi sample
            // Log: "Sample %d: %d mV", conversionCount, (adcBuffer[i] * 3300) / 4095
        }
        
        bufferReady = 1; // Báo hiệu buffer đã đầy 8 samples
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

int main(void)
{
    Timer_Init();
    Clock_Init();
    Adc_PinConfig();
    NVIC_Config();
    DMA_Config();
    Adc_Config();
    
    /* Start ADC conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    while (1)
    {
        if(bufferReady)
        {
            bufferReady = 0;
            // Xử lý dữ liệu từ buffer 8 samples
            // Ví dụ: tính trung bình, gửi qua UART, etc.
            
            uint32_t average = 0;
            for(int i = 0; i < 8; i++)
            {
                average += adcBuffer[i];
            }
            average /= 8;
            
            volatile uint16_t avgVoltage = (average * 3300) / 4095;
            // Process averaged data...
        }
        
        delay(10);
    }
}
