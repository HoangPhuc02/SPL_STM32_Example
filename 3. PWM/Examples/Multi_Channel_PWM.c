/*
 * =============================================================================
 * Project: STM32F103 PWM Examples  
 * File: Multi_Channel_PWM.c
 * Description: Multi-channel PWM example using TIM3 (4 channels)
 * Author: PWM Driver Team
 * Date: July 2025
 * =============================================================================
 * 
 * OVERVIEW:
 * This example demonstrates multi-channel PWM generation using all 4 channels
 * of TIM3. Each channel generates different duty cycles to show various PWM
 * patterns and effects.
 * 
 * HARDWARE SETUP:
 * - Channel 1 (PA6): Connect LED + resistor (Breathing effect)
 * - Channel 2 (PA7): Connect LED + resistor (Blinking effect)  
 * - Channel 3 (PB0): Connect LED + resistor (Static 25% brightness)
 * - Channel 4 (PB1): Connect LED + resistor (Static 75% brightness)
 * 
 * PWM CHANNELS:
 * - TIM3_CH1 -> PA6 (Variable breathing LED)
 * - TIM3_CH2 -> PA7 (Square wave blinking)
 * - TIM3_CH3 -> PB0 (Constant 25% duty cycle)
 * - TIM3_CH4 -> PB1 (Constant 75% duty cycle)
 * 
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

/* =============================================================================
 * CONFIGURATION DEFINES
 * =============================================================================
 */
#define PWM_FREQUENCY       2000    // 2kHz PWM frequency
#define TIMER_CLOCK         72000000 // 72MHz APB1 timer clock
#define PRESCALER           36      // Prescaler to get 2MHz timer clock
#define PERIOD              1000    // Period for 2kHz PWM (2MHz/1000 = 2kHz)
// 1-> 1000 => time 5ms
#define BREATHING_STEP      20       // Step size for breathing effect
#define BLINKING_PERIOD     200     // Blink period (in main loop cycles)
#define DELAY_COUNT         5000    // Delay between updates

/* Duty cycle presets (percentage converted to compare values) */
#define DUTY_25_PERCENT     (PERIOD * 25 / 100)    // 25% = 250
#define DUTY_50_PERCENT     (PERIOD * 50 / 100)    // 50% = 500  
#define DUTY_75_PERCENT     (PERIOD * 75 / 100)    // 75% = 750

/* =============================================================================
 * FUNCTION PROTOTYPES
 * =============================================================================
 */
void PWM_GPIO_Config(void);
void PWM_Timer_Config(void);
void PWM_SetChannelDutyCycle(uint8_t channel, uint16_t dutyCycle);
void Delay(uint32_t count);

volatile uint8_t i = 0;
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
}
/* =============================================================================
 * GLOBAL VARIABLES
 * =============================================================================
 */
uint16_t breathing_duty = 0;        // Current duty cycle for breathing LED
uint8_t breathing_direction = 1;    // Direction: 1=increase, 0=decrease
uint16_t blink_counter = 0;         // Counter for blinking LED
uint8_t blink_state = 0;            // Blink state: 0=off, 1=on

/* =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */
int main(void)
{
    // Initialize PWM system
    PWM_GPIO_Config();
    PWM_Timer_Config();
    
    // Set initial duty cycles
    PWM_SetChannelDutyCycle(1, 0);                  // CH1: Start with 0% (breathing)
    PWM_SetChannelDutyCycle(2, 0);                  // CH2: Start with 0% (blinking)
    PWM_SetChannelDutyCycle(3, DUTY_25_PERCENT);    // CH3: Constant 25%
    PWM_SetChannelDutyCycle(4, DUTY_75_PERCENT);    // CH4: Constant 75%
    
    /* =================================================================
     * MAIN LOOP - MULTI-CHANNEL PWM PATTERNS
     * =================================================================
     * Channel 1: Breathing effect (smooth fade in/out)
     * Channel 2: Blinking effect (on/off pattern)
     * Channel 3: Constant 25% brightness
     * Channel 4: Constant 75% brightness
     */
    while(1)
    {
        /* =========================================================
         * CHANNEL 1 - BREATHING EFFECT
         * =========================================================
         * Smoothly increases and decreases duty cycle to create
         * a breathing/pulsing effect
         */
        PWM_SetChannelDutyCycle(1, breathing_duty);
        
        // Update breathing duty cycle
        if(breathing_direction == 1) // Increasing brightness
        {
            breathing_duty += BREATHING_STEP;
            if(breathing_duty >= PERIOD)
            {
                breathing_duty = PERIOD;
                breathing_direction = 0; // Start decreasing
            }
        }
        else // Decreasing brightness
        {
            if(breathing_duty >= BREATHING_STEP)
            {
                breathing_duty -= BREATHING_STEP;
            }
            else
            {
                breathing_duty = 0;
                breathing_direction = 1; // Start increasing
            }
        }
        
        /* =========================================================
         * CHANNEL 2 - BLINKING EFFECT
         * =========================================================
         * Creates a simple on/off blinking pattern
         */
        blink_counter++;
        if(blink_counter >= BLINKING_PERIOD)
        {
            blink_counter = 0;
            blink_state = !blink_state; // Toggle state
            
            if(blink_state)
            {
                PWM_SetChannelDutyCycle(2, DUTY_50_PERCENT); // 50% when on
            }
            else
            {
                PWM_SetChannelDutyCycle(2, 0); // 0% when off
            }
        }
        
        /* Note: Channels 3 and 4 maintain constant duty cycles set in initialization */
        
        // Small delay between updates
        Delay(DELAY_COUNT);
    }
}

/* =============================================================================
 * PWM GPIO CONFIGURATION
 * =============================================================================
 * Configures all 4 GPIO pins for TIM3 PWM output:
 * - PA6 (TIM3_CH1) - Channel 1
 * - PA7 (TIM3_CH2) - Channel 2  
 * - PB0 (TIM3_CH3) - Channel 3
 * - PB1 (TIM3_CH4) - Channel 4
 */
void PWM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Enable GPIO clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PA6 and PA7 (TIM3_CH1 and TIM3_CH2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure PB0 and PB1 (TIM3_CH3 and TIM3_CH4) */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure PA6 and PA7 (TIM3_CH1 and TIM3_CH2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* =============================================================================
 * PWM TIMER CONFIGURATION
 * =============================================================================
 * Configures TIM3 with all 4 channels for PWM generation.
 * All channels share the same frequency but can have different duty cycles.
 */
void PWM_Timer_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // Enable TIM3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    /* =================================================================
     * TIMER BASE CONFIGURATION (COMMON FOR ALL CHANNELS)
     * =================================================================
     * All PWM channels on the same timer share:
     * - Same frequency (determined by prescaler and period)
     * - Same time base
     * - Independent duty cycles (compare values)
     */
    TIM_TimeBaseStructure.TIM_Period = PERIOD - 1;        // ARR = 999 (1000 steps)
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER - 1;  // PSC = 35 (72MHz/36 = 2MHz) = 0.5us
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    // timer will count u
    /* =================================================================
     * PWM CHANNEL CONFIGURATION (COMMON SETTINGS)
     * =================================================================
     * All channels use the same PWM mode and polarity for consistency
     */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           // PWM Mode 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   // Active high
    TIM_OCInitStructure.TIM_Pulse = 0;                          // Initial duty cycle
    
    /* Configure Channel 1 (PA6) */
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* Configure Channel 2 (PA7) */
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* Configure Channel 3 (PB0) */
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* Configure Channel 4 (PB1) */
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // Enable auto-reload preload
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    // Start the timer (all channels start simultaneously)
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // Enable update interrupt if needed
    NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt in NVIC if using interrupts
    
}

/* =============================================================================
 * SET CHANNEL DUTY CYCLE
 * =============================================================================
 * Sets the duty cycle for a specific PWM channel.
 * 
 * Parameters:
 *   channel - PWM channel number (1-4)
 *   dutyCycle - Compare value (0 to PERIOD)
 * 
 * Each channel has its own compare register that determines duty cycle:
 * - Channel 1: CCR1 register
 * - Channel 2: CCR2 register  
 * - Channel 3: CCR3 register
 * - Channel 4: CCR4 register
 */
void PWM_SetChannelDutyCycle(uint8_t channel, uint16_t dutyCycle)
{
    // Ensure duty cycle doesn't exceed period
    if(dutyCycle > PERIOD)
        dutyCycle = PERIOD;
    
    // Set compare value for specified channel
    switch(channel)
    {
        case 1:
            TIM_SetCompare1(TIM3, dutyCycle);
            break;
            
        case 2:
            TIM_SetCompare2(TIM3, dutyCycle);
            break;
            
        case 3:
            TIM_SetCompare3(TIM3, dutyCycle);
            break;
            
        case 4:
            TIM_SetCompare4(TIM3, dutyCycle);
            break;
            
        default:
            // Invalid channel - do nothing
            break;
    }
}

/* =============================================================================
 * SIMPLE DELAY FUNCTION
 * =============================================================================
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
 * MULTI-CHANNEL PWM THEORY
 * =============================================================================
 * 
 * MULTI-CHANNEL PWM CONCEPTS:
 * 
 * 1. SHARED TIME BASE:
 *    - All channels on same timer share frequency and period
 *    - Individual duty cycles controlled by separate compare registers
 *    - Synchronous operation - all channels aligned to same time base
 * 
 * 2. INDEPENDENT DUTY CYCLES:
 *    - Each channel can have different duty cycle (0% to 100%)
 *    - Compare registers (CCR1, CCR2, CCR3, CCR4) set independently
 *    - Real-time duty cycle changes without affecting other channels
 * 
 * 3. APPLICATIONS:
 *    - RGB LED control (3 channels for Red, Green, Blue)
 *    - Motor control (multiple motors with different speeds)
 *    - Audio applications (multiple tone generation)
 *    - Display dimming (multiple zones with different brightness)
 * 
 * TIMER REGISTER DETAILS:
 * 
 * - ARR (Auto Reload Register): Sets period for all channels
 * - CCR1-CCR4: Compare registers for each channel's duty cycle
 * - CNT (Counter): Current timer count value
 * 
 * PWM Generation Logic:
 * - When CNT < CCRx: Output is HIGH (for PWM Mode 1)
 * - When CNT >= CCRx: Output is LOW (for PWM Mode 1)
 * - When CNT reaches ARR: Counter resets to 0
 * 
 * SYNCHRONIZATION:
 * All channels switch simultaneously at timer overflow, providing:
 * - Precise phase alignment
 * - Reduced electromagnetic interference
 * - Predictable timing relationships
 * 
 * PERFORMANCE NOTES:
 * - Higher PWM frequency = smoother output, higher CPU overhead
 * - Lower PWM frequency = more efficient, potential audible noise
 * - Typical frequencies: 1kHz-20kHz for LEDs, 20kHz+ for motors
 * 
 * =============================================================================
 */
