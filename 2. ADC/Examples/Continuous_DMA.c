#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

/**
 * Example: ADC Continuous Sampling with DMA Transfer
 * 
 * This example demonstrates:
 * 1. ADC in continuous conversion mode
 * 2. Software trigger to start conversions
 * 3. DMA to transfer ADC values to memory without CPU intervention
 * 4. DMA Transfer Complete interrupt
 */

#define ADC_BUFFER_SIZE  100

// ADC results buffer
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t conversion_complete = 0;

void GPIO_Config(void)
{
    // Configure analog input pin (PA0)
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void DMA_Config(void)
{
    // Enable DMA1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    // DMA configuration
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
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    
    // Enable DMA channel
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC_Config(void)
{
    // Enable ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    // ADC configuration
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;           // Single channel, no scan needed
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;      // Continuous mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // Software trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;                 // 1 channel
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // Configure channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    
    // Enable ADC1 DMA
    ADC_DMACmd(ADC1, ENABLE);
    
    // Calibrate ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

// DMA interrupt handler
void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1))
    {
        conversion_complete = 1;
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}

int main(void)
{
    // Configure system clock
    SystemInit();
    
    // Initialize GPIO, DMA, and ADC
    GPIO_Config();
    DMA_Config();
    ADC_Config();
    
    // Start ADC conversions
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    while(1)
    {
        if(conversion_complete)
        {
            conversion_complete = 0;
            
            // The entire buffer is now filled with ADC values
            // Process the data as needed
            
            // For debugging, you can examine the buffer:
            volatile uint32_t sum = 0;
            for(int i = 0; i < ADC_BUFFER_SIZE; i++)
            {
                sum += adc_buffer[i];
            }
            volatile uint16_t average = sum / ADC_BUFFER_SIZE;
            
            // DMA is in circular mode, so it will automatically continue to fill the buffer
        }
    }
}
