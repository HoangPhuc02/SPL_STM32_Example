#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"

/**
 * Example: STM32F103 ADC Hardware Triggered by Timer (TIM1)
 *
 * Mô tả:
 * - Đọc giá trị analog từ chân PA0 sử dụng ADC1.
 * - Việc chuyển đổi ADC được kích hoạt tự động bởi tín hiệu trigger từ Timer1 (TIM1_CC1).
 * - Sử dụng chế độ Oneshot (không liên tục), mỗi lần trigger sẽ thực hiện một lần chuyển đổi.
 * - Giá trị ADC được đọc bằng phương pháp polling (kiểm tra cờ kết thúc chuyển đổi).
 * - Điều khiển LED trên chân PC13 dựa vào giá trị ADC đọc được (ví dụ: ngưỡng 2048).
 *
 * Ứng dụng:
 * - Đo tín hiệu analog định kỳ với thời gian chính xác nhờ timer.
 * - Có thể mở rộng cho các ứng dụng đo cảm biến, lấy mẫu tín hiệu, v.v.
 */

 
// Cấu hình TIM1 để tạo trigger cho ADC qua TIM1_CC1 (PWM mode hoặc Output Compare)
void TIM1_Trigger_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Bật clock cho TIM1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Cấu hình timer: ví dụ 1ms (1kHz) trigger
    TIM_TimeBaseStructure.TIM_Period = 7199; // (72MHz/7200) - 1 = 9999, ở đây chọn 7199 cho 10kHz, 7199 cho 1kHz
    TIM_TimeBaseStructure.TIM_Prescaler = 9; // 72MHz/10 = 7.2MHz, 7.2MHz/7200 = 1kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // Cấu hình Output Compare cho TIM1 Channel 1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period / 2; // 50% duty
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // Cho phép TIM1 tạo trigger cho ADC (TRGO)
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update); // hoặc TIM_TRGOSource_OC1 nếu muốn trigger theo compare

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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
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
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     // Oneshot mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // Hardware trigger on TIM1 CC1
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
    
    // Initialize GPIO, Timer1 và ADC
    GPIO_Config();
    TIM1_Trigger_Config();
    ADC_Config();

    // Đọc lại giá trị clock ADC thực tế (tham khảo)
    // uint32_t adc_clk = Get_ADC_Clock();
    // Bạn có thể in giá trị này qua UART hoặc debug để kiểm tra
    
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
