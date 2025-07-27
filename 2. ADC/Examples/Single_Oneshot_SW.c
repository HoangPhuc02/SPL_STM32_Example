#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"

/**
 * Example: Single Channel ADC with Software Trigger in Oneshot Mode
 * 
 * This example demonstrates:
 * 1. Single channel ADC conversion
 * 2. Software trigger to start conversions
 * 3. Oneshot mode (need to trigger each conversion)
 * 4. Polling method to check when conversion is complete
 */

void GPIO_Config(void)
{
    // Configure analog input pin (PA0)
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure an LED for visual feedback (PC13 on most Blue Pill boards)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void ADC_Config(void)
{
    // Enable ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    // ADC configuration
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;           // Single channel, no scan needed
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     // Oneshot mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // Software trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;                 // 1 channel
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // Configure channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    
    // Calibrate ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

// Simple delay function
void delay_ms(uint32_t ms)
{
    uint32_t i;
    for(i = 0; i < ms * 12000; i++) // Approximate for 72MHz system clock
    {
        __NOP();
    }
}

uint16_t Read_ADC(void)
{
    // Start ADC conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    // Wait for conversion to complete
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    
    // Read and return ADC value
    return ADC_GetConversionValue(ADC1);
}

int main(void)
{
    // Configure system clock
    SystemInit();
    
    // Initialize GPIO and ADC
    GPIO_Config();
    ADC_Config();
    
    volatile uint16_t adc_value;
    
    while(1)
    {
        // Read ADC value
        adc_value = Read_ADC();
        
        // Simple LED control based on ADC value (threshold at half of max value)
        if(adc_value > 2048)
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_13);   // Turn LED off (Blue Pill LED is active low)
        }
        else
        {
            GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Turn LED on
        }
        
        // Wait a bit before next conversion
        delay_ms(100);
    }
}
