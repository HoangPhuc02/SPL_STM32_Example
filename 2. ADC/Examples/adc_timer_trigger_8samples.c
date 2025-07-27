/**********************************************************
* File Name   : adc_timer_trigger_8samples
* Description : ADC with Timer trigger for 8 samples and periodic notifications
* Details     : Using Timer to trigger ADC and provide periodic notifications
* Version     : 1.0.0
* Date        : 22/07/2025
 **********************************************************/

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include <stdint.h>

volatile uint16_t adcBuffer[8] = {0};
volatile uint32_t conversionCount = 0;
volatile uint8_t sampleIndex = 0;
volatile uint8_t bufferReady = 0;

void delay(uint16_t time)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < time);
}

void Timer_Init(void)
{
    // Timer 2 for delay
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 0xFFFF;
    tim.TIM_Prescaler = 8000 - 1; // 1ms tick
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2, ENABLE);
}

void Timer3_Init(void)
{
    // Timer 3 for ADC trigger và notification
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000 - 1;      // 1ms period (1kHz sampling rate)
    tim.TIM_Prescaler = 72 - 1;     // 72MHz/72 = 1MHz
    TIM_TimeBaseInit(TIM3, &tim);
    
    // Enable Timer 3 interrupt
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void Clock_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void Adc_PinConfig(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

void NVIC_Config(void)
{
    NVIC_InitTypeDef nvic;
    
    // Timer 3 interrupt
    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    // DMA interrupt
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
    adc.ADC_ContinuousConvMode = DISABLE;  // Single conversion mode
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    
    ADC_Cmd(ADC1, ENABLE);
    
    // Calibration
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void DMA_Config(void)
{
    DMA_InitTypeDef dma;
    
    DMA_DeInit(DMA1_Channel1);
    
    dma.DMA_Mode = DMA_Mode_Normal;             // Normal mode (không circular)
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_M2M = DMA_M2M_Disable;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_BufferSize = 8;                     // 8 samples
    
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_MemoryBaseAddr = (uint32_t)adcBuffer;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    
    DMA_Init(DMA1_Channel1, &dma);
    
    // Enable Transfer Complete interrupt only
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    // Enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);
}

// Timer 3 interrupt - triggers every 1ms
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        if(sampleIndex < 8)
        {
            // Trigger ADC conversion
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            
            // Wait for conversion complete (very fast, ~1-2us)
            while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
            
            // Read value manually và ghi thông báo
            uint16_t value = ADC_GetConversionValue(ADC1);
            conversionCount++;
            
            // Ghi thông báo cho mỗi sample
            // Log: "Sample %d: %d mV", conversionCount, (value * 3300) / 4095
            
            sampleIndex++;
            
            // Nếu đã đủ 8 samples, reset DMA và trigger transfer
            if(sampleIndex >= 8)
            {
                sampleIndex = 0;
                
                // Reset và restart DMA để transfer 8 samples
                DMA_Cmd(DMA1_Channel1, DISABLE);
                DMA_SetCurrDataCounter(DMA1_Channel1, 8);
                DMA_Cmd(DMA1_Channel1, ENABLE);
                
                // Trigger DMA bằng cách đọc lại buffer
                for(int i = 0; i < 8; i++)
                {
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
                    // DMA sẽ tự động copy vào buffer
                }
            }
        }
        
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

// DMA Complete interrupt - sau khi transfer đủ 8 samples
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        bufferReady = 1; // Buffer đã đầy 8 samples
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

int main(void)
{
    Timer_Init();
    Timer3_Init();
    Clock_Init();
    Adc_PinConfig();
    NVIC_Config();
    Adc_Config();
    DMA_Config();
    
    while (1)
    {
        if(bufferReady)
        {
            bufferReady = 0;
            
            // Xử lý 8 samples trong buffer
            uint32_t sum = 0;
            for(int i = 0; i < 8; i++)
            {
                sum += adcBuffer[i];
            }
            uint16_t average = sum / 8;
            volatile uint16_t avgVoltage = (average * 3300) / 4095;
            
            // Process data...
        }
        
        delay(10);
    }
}
