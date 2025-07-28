#include "stm32f10x_flash.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "misc.h"

/**
 * Ví dụ: Đọc nhiều kênh ADC sử dụng ngắt (interrupt)
 *
 * - Sử dụng ngắt ADC để đọc luân phiên nhiều kênh (ví dụ: PA0, PA1)
 * - Khi có ngắt ADC (EOC), đọc giá trị và chuyển sang kênh tiếp theo
 * - Không cần polling, CPU rảnh trong khi chờ ADC chuyển đổi
 *
 * Hạn chế:
 * - Không đảm bảo thời gian lấy mẫu đều nhau giữa các kênh (do thời gian xử lý ngắt)
 * - Không phù hợp cho ứng dụng cần lấy mẫu đồng thời nhiều kênh
 * - Nếu số kênh nhiều hoặc tần số lấy mẫu cao, CPU có thể bị quá tải bởi ngắt ADC
 */
volatile uint8_t adc_counter = 0;
volatile uint16_t adc_value[2] = {0};
uint32_t a = 0;
volatile uint32_t it_counter = 1;
void ADC1_2_IRQHandler(void)
{
    it_counter++;
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) 
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_14); // Turn on LED for visual feedback
        adc_value[adc_counter] = ADC_GetConversionValue(ADC1);
        adc_counter = (++adc_counter == 2) ? 0 : adc_counter; // Toggle between 0 and 1

        ADC_RegularChannelConfig(ADC1, adc_counter, 1, ADC_SampleTime_55Cycles5);
        // Clear the ADC interrupt flag
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
        GPIO_ResetBits(GPIOC, GPIO_Pin_14); // Turn off LED after reading
    }
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
    
    // Initialize GPIO and ADC
    GPIO_Config();
    NVIC_Config();
    ADC_Config();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // Start the first conversion
    GPIO_SetBits(GPIOC, GPIO_Pin_13); // Turn on another LED for visual feedback
    
    while(1)
    {
        a++;
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