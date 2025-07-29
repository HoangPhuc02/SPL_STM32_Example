#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "misc.h"
/**
 * Example: STM32F103 ADC with Timer3 Hardware Trigger
 *
 * Description:
 * - Reads analog value from PA0 using ADC1
 * - ADC conversion is automatically triggered by Timer3 (TIM1_TRGO)
 * - Uses oneshot mode - each trigger initiates one conversion
 * - ADC value is read using interrupt method
 * - LED on PC13 toggles on each ADC conversion completion
 * - LED on PC14 toggles on each Timer3 update for debugging
 * 
 * Features:
 * - Timer3 configured for 50Hz trigger frequency (20ms period)
 * - ADC sampling time set to 1.5 cycles for fastest conversion
 * - System clock configured to 72MHz
 * - ADC clock set to 12MHz (max allowed for STM32F1)
 *
 * Applications:
 * - Precise periodic analog measurements
 * - Sensor sampling with accurate timing
 * - Data acquisition systems
 * - Signal monitoring and analysis
 */

void TIM1_CC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        GPIOC->ODR ^= GPIO_Pin_14;  // Toggle LED PC14
    }
}

void ADC1_2_IRQHandler(void)
{

    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) 
    {
        // Clear the ADC interrupt flag
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
        GPIOC->ODR ^= GPIO_Pin_13;  // Toggle LED PC13
    }

}
void SystemClock_Config(void);
void ADC_Config(void);
void TIM1_Config(void);
void GPIO_Config(void);
void NVIC_Config(void);

int main(void)
{
    // Configure system clock to 72MHz
    SystemClock_Config();
    
    // Initialize GPIO, Timer1 và ADC
    GPIO_Config();
    TIM1_Config();
    NVIC_Config();
    ADC_Config();
    ADC_ExternalTrigConvCmd(ADC1, ENABLE); // Enable external trigger for ADC conversion

    
    while(1)
    {

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

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_Init(&NVIC_InitStructure);
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

void ADC_Config(void)
{
    // Enable ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Set ADC clock = PCLK2/6 = 72MHz/6 = 12MHz (max for STM32F1 ADC)
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 12 MHz for ADC1, 6 MHz for ADC2 and ADC3 (max for STM32F1 ADC)
    
    // ADC configuration
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;           // Single channel, no scan needed
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     // Oneshot mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // Hardware trigger on TIM1 TRGO
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;                 // 1 channel
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // Configure channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5);
    
    // Enable external trigger for ADC conversion
    // ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // Enable EOC interrupt
    
    // Calibrate ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}


/**
 * @brief Configure Timer3 to generate trigger signals for ADC via TIM1_TRGO
 * 
 * This function sets up Timer3 with the following specifications:
 * - Clock source: APB1 (36MHz)
 * - Timer frequency: 50Hz (20ms period)
 * - Configuration:
 *   - Period = 7199
 *   - Prescaler = 199
 *   - Resulting frequency = 72MHz / (200 * 7200) = 50Hz
 * 
 * Timer3 is configured to:
 * 1. Generate update events as trigger output (TRGO)
 * 2. Use up-counting mode
 * 3. Enable ARR preload
 * 
 * The timer can be debugged using:
 * - PC14 LED toggle in timer interrupt (commented out)
 * - Logic analyzer on timer outputs
 * 
 * @note Timer frequency calculation:
 * System clock = 72MHz
 * Timer clock = 72MHz (APB1 x2 due to prescaler settings)
 * Counter clock = 72MHz / 200 = 360kHz
 * Output frequency = 360kHz / 7200 = 50Hz
 */
void TIM1_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Bật clock cho TIM1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Cấu hình timer: ví dụ 1ms (1kHz) trigger
    TIM_TimeBaseStructure.TIM_Period = 7199; // (72MHz/7200) - 1 = 9999, ở đây chọn 7199 cho 10kHz, 7199 cho 1kHz
    TIM_TimeBaseStructure.TIM_Prescaler = 199; // 72MHz/200 = 360kHz, 360kHz/7200 = 50Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // Cấu hình Output Compare cho TIM1 Channel 1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 3600; // 50% duty
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // // Cho phép TIM1 tạo trigger cho ADC (TRGO)
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1Ref); // hoặc TIM_TRGOSource_OC1 nếu muốn trigger theo compare
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE); // Enable update interrupt for TIM1
    // Bật TIM1
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE); // Bắt buộc với TIM1 (advanced timer)
}

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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14; // Use PC13 and PC14 for Debugging
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //
    /* Configure PA8 (TIM1_CH1) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed for clean edges
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}