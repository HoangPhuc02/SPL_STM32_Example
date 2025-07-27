/**********************************************************
* File Name   : adc_interrupt
* Description : ADC with interrupt example
* Details     : Example of using ADC with interrupt-based reading
* Version     : 1.0.0
* Date        : 30/06/2025
 **********************************************************/

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "misc.h" // For NVIC configuration
#include <stdint.h>

volatile uint16_t adcValue = 0;
volatile uint32_t it_count = 0;
volatile uint32_t dma_count = 0;
volatile uint16_t buffer[8] = {0}; // Buffer for DMA ADC values, marked volatile for debugging & watch
void delay(uint16_t time)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < time)
        ;
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
    nvic.NVIC_IRQChannel = ADC1_2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
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
    
    /* Enable ADC interrupt */
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    ADC_DMACmd(ADC1, ENABLE); // Enable DMA for ADC
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
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
}

/* ADC Interrupt Handler */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        // Read the converted value
        adcValue = ADC_GetConversionValue(ADC1);
        it_count++;
        if(it_count > 16)
        {
            NVIC_DisableIRQ(ADC1_2_IRQn);
        }
        // Clear the ADC interrupt flag
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    // Check if the DMA transfer is complete
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        // Set flag to indicate conversion is complete
        //  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5); // ADC Channel 1 (PA1)
        dma_count++;
        if(dma_count > 4)
        {
            NVIC_DisableIRQ(DMA1_Channel1_IRQn);
        }
        // ADC_Cmd(ADC1, DISABLE);
        // Clear the DMA transfer complete flag
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

int main(void)
{
    volatile uint16_t voltageInMv = 0;
    
    Timer_Init();       // Initialize Timer for delay
    TimerAdc_Init();    // Initialize Timer for ADC
    Adc_PinConfig();    // Configure ADC pin
    NVIC_Config();      // Configure NVIC for ADC interrupts
    Adc_Config();       // Configure ADC
    DMA_Config();      // Configure DMA
    /* Start ADC conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    while (1)
    {
        // Convert ADC value to millivolts (assuming 3.3V reference)
        voltageInMv = (adcValue * 3300) / 4095;
        
        // Add a short delay
        delay(100);
    }
}
