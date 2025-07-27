/*
 * =============================================================================
 * Project: STM32F103 PWM Examples
 * File: Basic_PWM_LED_Control.c
 * Description: Basic PWM example to control LED brightness using TIM3 Channel 1
 * Author: PWM Driver Team
 * Date: July 2025
 * =============================================================================
 * 
 * OVERVIEW:
 * This example demonstrates basic PWM generation to control LED brightness.
 * We use TIM3 Channel 1 connected to PA6 pin to generate PWM signal.
 * 
 * HARDWARE SETUP:
 * - Connect LED + resistor between PA6 and GND
 * - LED will fade in and out continuously
 * 
 * PWM CONFIGURATION:
 * - Timer: TIM3
 * - Channel: Channel 1 (PA6)
 * - Frequency: 1kHz (good for LED control)
 * - Duty cycle: Variable (0% to 100%)
 * 
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
#include "misc.h"
/* =============================================================================
 * CONFIGURATION DEFINES
 * =============================================================================
 */
#define PWM_FREQUENCY       1           // 1Hz PWM frequency
#define TIMER_CLOCK         72000000    // 72MHz APB1 timer clock
#define PRESCALER           7200        // Prescaler to get 10KHz timer clock
// #define PERIOD              10000       // Period for 1Hz PWM (10KHz/10000 = 1Hz)

// #define PRESCALER           36000        // Prescaler to get 10KHz timer clock
#define PERIOD              20000       // Period for 1Hz PWM (10KHz/10000 = 1Hz)

#define FADE_STEP           5           // Step size for fade effect
#define FADE_DELAY          10000   // Delay between fade steps

/* =============================================================================
 * FUNCTION PROTOTYPES
 * =============================================================================
 */
void PWM_GPIO_Config(void);
void PWM_Timer_Config(void);
void PWM_SetDutyCycle(uint16_t dutyCycle);
void SystemClock_Config(void);
void Delay(uint32_t count);
void NVIC_Config(void);
volatile uint8_t i = 0;
volatile uint8_t j = 0;
void TIM3_IRQHandler(void)
{
    // Check if the timer update interrupt is pending
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        i = ~i;
        if(i)
        GPIO_SetBits(GPIOC, GPIO_Pin_13); // Toggle PC13 (for example)
        else 
        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Toggle PC13 (for example)
        // Clear the interrupt flag
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        // Add your timer handling code here
        // For example, toggle an LED or update a variable
    }
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        j = ~j;
        if(j)
        GPIO_SetBits(GPIOA, GPIO_Pin_7); // Toggle PA7 (for example)
        else 
        GPIO_ResetBits(GPIOA, GPIO_Pin_7); // Toggle PA7 (for example)
        // Clear the interrupt flag
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        
        // Add your timer handling code here
        // For example, toggle an LED or update a variable
    }
}
/* =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */
int main(void)
{
    uint16_t dutyCycle = 0;
    uint8_t fadeDirection = 1; // 1 = fade in, 0 = fade out
    SystemClock_Config();
    NVIC_Config(); // Configure NVIC for TIM3 interrupts
    // Initialize system
    PWM_GPIO_Config();
    PWM_Timer_Config();
    
    // Start with 0% duty cycle (LED off)
    PWM_SetDutyCycle(PERIOD/2);
    
    /* =================================================================
     * MAIN LOOP - LED FADE EFFECT
     * =================================================================
     * This loop creates a breathing LED effect by gradually increasing
     * and decreasing the PWM duty cycle from 0% to 100% and back.
     */
    while(1)
    {
        // // Set current duty cycle
        // PWM_SetDutyCycle(dutyCycle);
        
        // // Wait before next step
        // Delay(FADE_DELAY);
        
        // // Update duty cycle based on fade direction
        // if(fadeDirection == 1) // Fading in (getting brighter)
        // {
        //     dutyCycle += FADE_STEP;
        //     if(dutyCycle >= PERIOD) // Reached maximum brightness
        //     {
        //         dutyCycle = PERIOD;
        //         fadeDirection = 0; // Start fading out
        //     }
        // }
        // else // Fading out (getting dimmer)
        // {
        //     if(dutyCycle >= FADE_STEP)
        //     {
        //         dutyCycle -= FADE_STEP;
        //     }
        //     else
        //     {
        //         dutyCycle = 0;
        //         fadeDirection = 1; // Start fading in
        //     }
        // }
    }
}
void SystemClock_Config(void)
{
    ErrorStatus HSEStartUpStatus;
    
    /* Reset RCC configuration */
    RCC_DeInit();
    
    /* Enable HSE (High Speed External) oscillator */
    RCC_HSEConfig(RCC_HSE_ON);
    
    /* Wait for HSE to be ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    
    if(HSEStartUpStatus == SUCCESS)
    {   
        FLASH_SetLatency(FLASH_Latency_2); // why need this code
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* Configure AHB clock (HCLK) */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);    // AHB = SYSCLK = 72MHz
        
        /* Configure APB1 clock (PCLK1) */
        RCC_PCLK1Config(RCC_HCLK_Div2);     // APB1 = 36MHz (Timer clock = 72MHz)
        
        /* Configure APB2 clock (PCLK2) */
        RCC_PCLK2Config(RCC_HCLK_Div1);     // APB2 = 72MHz
        
        /* Configure PLL: HSE × 9 = 8MHz × 9 = 72MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        
        /* Enable PLL */
        RCC_PLLCmd(ENABLE);
        
        /* Wait for PLL to be ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        
        /* Wait until PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08);
    }
    else
    {
        /* HSE failed to start - handle error */
        /* You can add error handling here */
        while(1); // Infinite loop - system halt
    }
}
/* =============================================================================
 * PWM GPIO CONFIGURATION
 * =============================================================================
 * Configures PA6 as alternate function push-pull for TIM3 Channel 1 PWM output.
 * 
 * GPIO Configuration Details:
 * - Pin: PA6 (TIM3_CH1)
 * - Mode: Alternate Function Push-Pull
 * - Speed: 50MHz for clean PWM edges
 */

void PWM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
    
    
    /* Configure PA6 (TIM3_CH1) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed for clean edges
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure PA6 (TIM3_CH1) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed for clean edges
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure PA6 (TIM3_CH1) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed for clean edges
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* =============================================================================
 * PWM TIMER CONFIGURATION
 * =============================================================================
 * Configures TIM3 for PWM generation with the following parameters:
 * - Timer Clock: 72MHz (APB1)
 * - Prescaler: 72 (gives 1MHz timer frequency)
 * - Period: 1000 (gives 1kHz PWM frequency)
 * - PWM Mode: PWM Mode 1 (active high)
 */
void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // Enable TIM3 interrupt in NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Highest priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // No sub-priority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable the interrupt
    NVIC_Init(&NVIC_InitStructure);
}
void PWM_Timer_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // Enable TIM3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    /* =================================================================
     * TIMER BASE CONFIGURATION
     * =================================================================
     * Timer frequency calculation:
     * Timer_Freq = APB1_Clock / (Prescaler + 1)
     * PWM_Freq = Timer_Freq / (Period + 1)
     * 
     * Example: 72MHz / 72 = 1MHz timer frequency
     *          1MHz / 1000 = 1kHz PWM frequency
     */
    TIM_TimeBaseStructure.TIM_Period = PERIOD - 1;        // ARR register (1000-1 = 999)
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER - 1;  // PSC register (72-1 = 71)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // No clock division
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // Count up mode
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    /* =================================================================
     * PWM OUTPUT COMPARE CONFIGURATION
     * =================================================================
     * PWM Mode 1: Output is active when counter < compare value
     * PWM Mode 2: Output is active when counter > compare value
     * 
     * We use PWM Mode 1 for normal operation (active high)
     */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;           // PWM Mode 2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // Enable output
    TIM_OCInitStructure.TIM_Pulse = 0;                          // Initial duty cycle (0%)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   // Active high
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // Initialize Channel 1
    
    // Enable auto-reload preload for smooth operation
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    // Start the timer
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE); // Enable update interrupt if needed

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // Enable update interrupt if needed
    NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt in NVIC if using interrupts
}

/* =============================================================================
 * SET PWM DUTY CYCLE
 * =============================================================================
 * Sets the PWM duty cycle by updating the compare register.
 * 
 * Parameters:
 *   dutyCycle - Compare value (0 to PERIOD)
 *              0 = 0% duty cycle (always low)
 *              PERIOD/2 = 50% duty cycle 
 *              PERIOD = 100% duty cycle (always high)
 * 
 * The duty cycle percentage is calculated as: (dutyCycle / PERIOD) * 100%
 */
void PWM_SetDutyCycle(uint16_t dutyCycle)
{
    // Ensure duty cycle doesn't exceed period
    if(dutyCycle > PERIOD)
        dutyCycle = PERIOD;
    
    // Update compare register for Channel 1
    TIM_SetCompare1(TIM3, dutyCycle);
}

/* =============================================================================
 * SIMPLE DELAY FUNCTION
 * =============================================================================
 * Simple software delay function.
 * Note: In real applications, use SysTick timer or other precise timing methods.
 */
void Delay(uint32_t count)
{
    volatile uint32_t i;
    for(i = 0; i < count; i++)
    {
        // Empty loop for delay
    }
}

/* =============================================================================
 * PWM THEORY AND CALCULATIONS
 * =============================================================================
 * 
 * PWM (Pulse Width Modulation) generates a square wave with variable duty cycle.
 * 
 * KEY CONCEPTS:
 * 
 * 1. FREQUENCY:
 *    - Determines how fast the PWM switches on/off
 *    - Higher frequency = smoother output (less ripple)
 *    - Lower frequency = more efficient (less switching losses)
 * 
 * 2. DUTY CYCLE:
 *    - Percentage of time signal is HIGH in one period
 *    - 0% = always LOW, 50% = half time HIGH, 100% = always HIGH
 *    - Controls average output voltage
 * 
 * 3. RESOLUTION:
 *    - Number of discrete duty cycle steps available
 *    - Higher period value = higher resolution
 *    - Resolution = Period value (e.g., 1000 = 1000 steps)
 * 
 * TIMER CALCULATIONS:
 * 
 * System Clock = 72MHz
 * APB1 Clock = 72MHz (if APB1 prescaler = 1)
 * Timer Clock = APB1 Clock / (Prescaler + 1)
 * PWM Frequency = Timer Clock / (Period + 1)
 * 
 * Example:
 * Prescaler = 71 (register value)
 * Period = 999 (register value)
 * Timer Clock = 72MHz / 72 = 1MHz
 * PWM Frequency = 1MHz / 1000 = 1kHz
 * 
 * DUTY CYCLE CALCULATION:
 * Duty Cycle % = (Compare Value / Period) * 100%
 * Compare Value = (Desired % / 100) * Period
 * 
 * Examples:
 * 25% duty cycle: Compare = (25/100) * 1000 = 250
 * 50% duty cycle: Compare = (50/100) * 1000 = 500
 * 75% duty cycle: Compare = (75/100) * 1000 = 750
 * 
 * =============================================================================
 */
