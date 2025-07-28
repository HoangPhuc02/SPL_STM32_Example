#include "stm32f10x_flash.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "stm32f10x_tim.h"

#include <stdint.h>

/**
 * Example: Single Channel ADC with Software Trigger in Oneshot Mode
 * 
 * This example demonstrates:
 * 1. Single channel ADC conversion
 * 2. Software trigger to start conversions
 * 3. Oneshot mode (need to trigger each conversion)
 * 4. Polling method to check when conversion is complete
 */
volatile uint8_t adc_counter = 0;
volatile uint16_t adc_value[2] = {0};
uint32_t a = 0;
volatile uint32_t it_counter = 1;
volatile uint32_t irq_time_start = 0;
volatile uint32_t irq_time_end = 0;
volatile uint32_t irq_time_diff = 0;
void ADC1_2_IRQHandler(void)
{
    it_counter++;
    irq_time_start = TIM_GetCounter(TIM2); // Bắt đầu đo thời gian
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) 
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13); // Turn on LED for visual feedback
        adc_value[adc_counter] = ADC_GetConversionValue(ADC1);
        adc_counter = (++adc_counter == 2) ? 0 : adc_counter; // Toggle between 0 and 1

        ADC_RegularChannelConfig(ADC1, adc_counter, 1, ADC_SampleTime_55Cycles5);
        // Clear the ADC interrupt flag
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Turn off LED after reading
    }
    irq_time_end = TIM_GetCounter(TIM2); // Kết thúc đo thời gian
    if (irq_time_end >= irq_time_start)
        irq_time_diff = irq_time_end - irq_time_start;
    else
        irq_time_diff = (0xFFFF - irq_time_start) + irq_time_end + 1;
}
void TIM2_TimeBase_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 72MHz/72 = 1MHz, 1 tick = 1us
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
}
void GPIO_Config(void);
void ADC_Config(void);
void SystemClock_Config(void);
void NVIC_Config(void);



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
    // Configure system clock to 72MHz
    SystemClock_Config();

    // Initialize GPIO, NVIC, TIM2, and ADC
    GPIO_Config();
    NVIC_Config();
    TIM2_TimeBase_Config(); // Khởi tạo timer đo thời gian
    ADC_Config();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // Start the first conversion
    GPIO_SetBits(GPIOC, GPIO_Pin_14); // Turn on another LED for visual feedback

    while(1)
    {
        a++;
        // Có thể debug giá trị irq_time_diff để biết thời gian thực thi của ngắt ADC (đơn vị: us)
    }
}

void GPIO_Config(void)
{
    // Configure analog input pin (PA0)
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // PA0 and PA1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure an LED for visual feedback (PC13 on most Blue Pill boards)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void ADC_Config(void)
{
    // Enable ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Set ADC clock = PCLK2/6 = 72MHz/6 = 12MHz (max for STM32F1 ADC)
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    // ADC configuration
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;           // Single channel, no scan needed
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;     // Oneshot mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // Software trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;                 // 1 channel
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // Configure channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // Enable EOC interrupt
    // Calibrate ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void SystemClock_Config(void)
{
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource() != 0x08);
    }
    else
    {
        while(1);
    }
}

void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // Enable ADC1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}