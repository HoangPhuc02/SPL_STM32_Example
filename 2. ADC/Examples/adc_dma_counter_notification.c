/**********************************************************
* File Name   : adc_dma_counter_notification
* Description : ADC with DMA using software counter for per-sample notifications
* Details     : DMA in circular mode with software counter tracking each sample
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
volatile uint8_t lastProcessedIndex = 0;
volatile uint8_t bufferFullCount = 0;

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
    tim.TIM_Prescaler = 8000 - 1;
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2, ENABLE);
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
    
    ADC_DMACmd(ADC1, ENABLE);
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
    
    dma.DMA_Mode = DMA_Mode_Circular;           // Circular mode
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_M2M = DMA_M2M_Disable;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_BufferSize = 1;                     // Buffer size = 1 (quan trọng!)
    
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_MemoryBaseAddr = (uint32_t)adcBuffer;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    
    DMA_Init(DMA1_Channel1, &dma);
    
    // Enable Transfer Complete interrupt (sẽ trigger sau mỗi sample)
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

// DMA interrupt - được gọi sau mỗi sample
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        conversionCount++;
        
        // Ghi thông báo cho sample hiện tại
        uint8_t currentIndex = (conversionCount - 1) % 8;
        uint16_t currentValue = adcBuffer[currentIndex];
        
        // Log notification cho mỗi sample
        // printf("Sample %d: %d mV\n", conversionCount, (currentValue * 3300) / 4095);
        
        // Kiểm tra nếu đã đủ 8 samples
        if(conversionCount % 8 == 0)
        {
            bufferFullCount++;
            // Buffer đã đầy 8 samples - có thể xử lý tổng hợp
        }
        
        // Reset memory address để tiếp tục circular
        if(currentIndex == 7)
        {
            DMA_Cmd(DMA1_Channel1, DISABLE);
            DMA_SetCurrDataCounter(DMA1_Channel1, 1);
            dma.DMA_MemoryBaseAddr = (uint32_t)adcBuffer;  // Reset về đầu buffer
            DMA_Init(DMA1_Channel1, &dma);
            DMA_Cmd(DMA1_Channel1, ENABLE);
        }
        
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

int main(void)
{
    Timer_Init();
    Clock_Init();
    Adc_PinConfig();
    NVIC_Config();
    Adc_Config();
    DMA_Config();
    
    // Start ADC conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    while (1)
    {
        // Xử lý khi buffer đầy 8 samples
        static uint32_t lastBufferCount = 0;
        if(bufferFullCount > lastBufferCount)
        {
            lastBufferCount = bufferFullCount;
            
            // Tính trung bình 8 samples
            uint32_t sum = 0;
            for(int i = 0; i < 8; i++)
            {
                sum += adcBuffer[i];
            }
            uint16_t average = sum / 8;
            volatile uint16_t avgVoltage = (average * 3300) / 4095;
            
            // Process averaged data...
        }
        
        delay(10);
    }
}
