/*
 * =============================================================================
 * Project: STM32F103 Simple Button Test
 * File: button_test_simple.c
 * Description: Simple button test với responsive debouncing
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "system_stm32f10x.h"

/* ======================= Global Variables ======================= */
volatile uint32_t msTicks = 0;

/* ======================= Function Prototypes ======================= */
void SysTick_Config_1ms(void);
uint32_t GetTick(void);
void GPIO_Config(void);
uint8_t Button_Simple_Press(void);

/* ======================= SysTick Configuration ======================= */
void SysTick_Config_1ms(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1);
    }
}

void SysTick_Handler(void)
{
    msTicks++;
}

uint32_t GetTick(void)
{
    return msTicks;
}

/* ======================= GPIO Configuration ======================= */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    // Button PA0 with internal pull-down
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // LED PC13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // LED off initially
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/* ======================= Simple Button Function ======================= */
/**
 * @brief Very simple button detection - responsive và fast
 * @return 1 if button just pressed, 0 otherwise
 */
uint8_t Button_Simple_Press(void)
{
    static uint8_t button_was_pressed = 0;
    static uint32_t last_change_time = 0;
    
    uint8_t button_current = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    uint32_t current_time = GetTick();
    
    // Debounce: ignore changes trong 30ms
    if ((current_time - last_change_time) < 30) {
        return 0;
    }
    
    // Rising edge detection: 0 → 1
    if (button_current && !button_was_pressed) {
        button_was_pressed = 1;
        last_change_time = current_time;
        return 1; // Button just pressed!
    }
    
    // Falling edge detection: 1 → 0
    if (!button_current && button_was_pressed) {
        button_was_pressed = 0;
        last_change_time = current_time;
    }
    
    return 0;
}

/* ======================= Main Function ======================= */
int main(void)
{
    uint32_t led_toggle_count = 0;
    
    /* Initialize system */
    SystemInit();
    SysTick_Config_1ms();
    GPIO_Config();
    
    /* Startup blink - 3 times */
    for (uint8_t i = 0; i < 3; i++) {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // LED on
        for(volatile uint32_t j = 0; j < 200000; j++); // Simple delay
        GPIO_SetBits(GPIOC, GPIO_Pin_13);    // LED off
        for(volatile uint32_t j = 0; j < 200000; j++);
    }
    
    /* Main loop */
    while (1)
    {
        /* Check button press */
        if (Button_Simple_Press()) {
            /* Toggle LED immediately */
            if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)) {
                GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // Turn on
            } else {
                GPIO_SetBits(GPIOC, GPIO_Pin_13);    // Turn off
            }
            
            led_toggle_count++;
            
            /* Optional: brief indication */
            for(volatile uint32_t j = 0; j < 50000; j++); // Very short delay
        }
        
        /* Very small delay */
        for(volatile uint32_t j = 0; j < 1000; j++);
    }
}
