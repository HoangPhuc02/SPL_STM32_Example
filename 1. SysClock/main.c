/**
 * @file    main.c
 * @brief   Comprehensive RCC Clock Management Example for STM32F103C8T6
 * @details This example demonstrates all aspects of RCC clock management including:
 *          - HSE + PLL configuration for 72MHz operation
 *          - Peripheral clock management and power optimization
 *          - Clock monitoring and failure recovery
 *          - Real-time clock information display
 *          - Practical power management scenarios
 * 
 * @author  STM32 Development Team
 * @date    2025-01-08
 * @version 1.0.0
 * 
 * @note    Hardware Requirements:
 *          - STM32F103C8T6 with 8MHz HSE crystal
 *          - UART connection for debug output (PA9/PA10)
 *          - LED on PC13 for status indication
 *          - Button on PA0 for mode switching
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "rcc_clock_management.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define LED_PIN                     GPIO_Pin_13
#define LED_PORT                    GPIOC
#define BUTTON_PIN                  GPIO_Pin_0
#define BUTTON_PORT                 GPIOA

#define UART_TX_PIN                 GPIO_Pin_9
#define UART_RX_PIN                 GPIO_Pin_10
#define UART_PORT                   GPIOA

#define MAIN_LOOP_DELAY_MS          (1000)      /* 1 second main loop */
#define POWER_MODE_SWITCH_DELAY     (5000)      /* 5 seconds between power mode switches */
#define CLOCK_MONITOR_INTERVAL      (10000)     /* 10 seconds clock monitoring */

/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
static volatile uint32_t s_SystemTicks = 0;
static volatile bool s_ButtonPressed = false;
static PowerMode_t s_CurrentPowerMode = POWER_MODE_FULL_PERFORMANCE;
static uint32_t s_PowerModeCounter = 0;
static uint32_t s_ClockMonitorCounter = 0;

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
static void Hardware_Init(void);
static void GPIO_Init(void);
static void UART_Init(void);
static void SysTick_Init(void);
static void LED_Toggle(void);
static void LED_SetState(bool state);
static bool Button_IsPressed(void);
static void PowerMode_Cycle(void);
static void ClockMonitoring_Task(void);
static void DisplaySystemStatus(void);
static void DemonstratePowerModes(void);
static void DemonstrateClockSources(void);
static void HandleClockFailure(void);

/* Printf redirection to UART */
int fputc(int ch, FILE *f);

/*==================================================================================================
*                                       MAIN FUNCTION
==================================================================================================*/

/**
 * @brief   Main function demonstrating comprehensive RCC usage
 */
int main(void)
{
    ClockConfigStatus_t clockStatus;
    
    /* Configure system clock to 72MHz using HSE + PLL */
    clockStatus = RCC_SystemClockConfig();
    
    if (clockStatus != CLOCK_CONFIG_SUCCESS)
    {
        /* Clock configuration failed - handle error */
        HandleClockFailure();
    }
    
    /* Initialize hardware */
    Hardware_Init();
    
    /* Display startup message */
    printf("\r\n");
    printf("=====================================\r\n");
    printf("STM32F103C8T6 RCC Management Example\r\n");
    printf("=====================================\r\n");
    printf("Clock Configuration Status: %s\r\n", 
           (clockStatus == CLOCK_CONFIG_SUCCESS) ? "SUCCESS" : "FAILED");
    
    /* Display initial clock information */
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
    RCC_DisplayClockInfo(&g_SystemClockInfo);
    
    /* Display initial peripheral status */
    RCC_GetPeripheralClockStatus(&g_PeripheralClockStatus);
    RCC_DisplayPeripheralStatus(&g_PeripheralClockStatus);
    
    /* Enable clock monitoring */
    g_ClockMonitoringEnabled = true;
    
    printf("\r\nStarting main application loop...\r\n");
    printf("Press button (PA0) to cycle through power modes\r\n");
    
    /* Main application loop */
    while (1)
    {
        /* Toggle LED to show system is running */
        LED_Toggle();
        
        /* Check for button press to cycle power modes */
        if (Button_IsPressed())
        {
            PowerMode_Cycle();
            s_ButtonPressed = false;
        }
        
        /* Demonstrate power modes automatically every 5 seconds */
        if (s_SystemTicks % POWER_MODE_SWITCH_DELAY == 0)
        {
            DemonstratePowerModes();
        }
        
        /* Monitor clocks every 10 seconds */
        if (s_SystemTicks % CLOCK_MONITOR_INTERVAL == 0)
        {
            ClockMonitoring_Task();
        }
        
        /* Display system status every 5 seconds */
        if (s_SystemTicks % POWER_MODE_SWITCH_DELAY == 0)
        {
            DisplaySystemStatus();
        }
        
        /* Demonstrate different clock sources every 30 seconds */
        if (s_SystemTicks % (6 * POWER_MODE_SWITCH_DELAY) == 0)
        {
            DemonstrateClockSources();
        }
        
        /* Delay for main loop timing */
        for (volatile uint32_t i = 0; i < 1000000; i++);
        s_SystemTicks += MAIN_LOOP_DELAY_MS;
    }
    
    return 0;
}

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/

/**
 * @brief   Initialize all hardware components
 */
static void Hardware_Init(void)
{
    /* Initialize SysTick for timing */
    SysTick_Init();
    
    /* Configure peripheral clocks for full performance mode */
    RCC_PeripheralClockConfig(POWER_MODE_FULL_PERFORMANCE);
    
    /* Initialize GPIO */
    GPIO_Init();
    
    /* Initialize UART for debug output */
    UART_Init();
    
    /* Configure USB clock (if needed) */
    RCC_ConfigureUSBClock();
}

/**
 * @brief   Initialize GPIO pins
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Configure LED pin (PC13) as output */
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    
    /* Configure button pin (PA0) as input with pull-up */
    GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
    
    /* Configure UART pins (PA9 = TX, PA10 = RX) */
    /* TX pin as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = UART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(UART_PORT, &GPIO_InitStructure);
    
    /* RX pin as input floating */
    GPIO_InitStructure.GPIO_Pin = UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_PORT, &GPIO_InitStructure);
    
    /* Turn off LED initially */
    LED_SetState(false);
}

/**
 * @brief   Initialize UART for debug output
 */
static void UART_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    
    /* Configure UART */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief   Initialize SysTick timer
 */
static void SysTick_Init(void)
{
    /* Configure SysTick for 1ms interrupts */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1);
    }
}

/**
 * @brief   Toggle LED state
 */
static void LED_Toggle(void)
{
    GPIO_WriteBit(LED_PORT, LED_PIN, 
                  (BitAction)(1 - GPIO_ReadOutputDataBit(LED_PORT, LED_PIN)));
}

/**
 * @brief   Set LED state
 */
static void LED_SetState(bool state)
{
    GPIO_WriteBit(LED_PORT, LED_PIN, state ? Bit_SET : Bit_RESET);
}

/**
 * @brief   Check if button is pressed
 */
static bool Button_IsPressed(void)
{
    static bool lastState = true;  /* Button is active low with pull-up */
    bool currentState = (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN) == Bit_RESET);
    
    /* Detect falling edge (button press) */
    if (!lastState && currentState)
    {
        lastState = currentState;
        return true;
    }
    
    lastState = currentState;
    return false;
}

/**
 * @brief   Cycle through power modes
 */
static void PowerMode_Cycle(void)
{
    s_CurrentPowerMode = (PowerMode_t)((s_CurrentPowerMode + 1) % 4);
    
    printf("\r\nSwitching to power mode: ");
    switch (s_CurrentPowerMode)
    {
        case POWER_MODE_FULL_PERFORMANCE:
            printf("FULL PERFORMANCE\r\n");
            break;
        case POWER_MODE_NORMAL:
            printf("NORMAL\r\n");
            break;
        case POWER_MODE_LOW_POWER:
            printf("LOW POWER\r\n");
            break;
        case POWER_MODE_SLEEP_READY:
            printf("SLEEP READY\r\n");
            break;
    }
    
    /* Apply new power mode */
    RCC_PeripheralClockConfig(s_CurrentPowerMode);
    
    /* Update peripheral status */
    RCC_GetPeripheralClockStatus(&g_PeripheralClockStatus);
    RCC_DisplayPeripheralStatus(&g_PeripheralClockStatus);
}

/**
 * @brief   Clock monitoring task
 */
static void ClockMonitoring_Task(void)
{
    bool clocksStable;

    printf("\r\n--- Clock Monitoring ---\r\n");

    /* Monitor system clocks */
    clocksStable = RCC_MonitorSystemClocks();

    if (clocksStable)
    {
        printf("All clocks are stable and operating correctly.\r\n");
    }
    else
    {
        printf("WARNING: Clock instability detected!\r\n");
        printf("Error count: %lu\r\n", RCC_GetClockErrorCount());

        if (RCC_IsHSEFailureDetected())
        {
            printf("HSE failure detected - attempting recovery...\r\n");
            ClockConfigStatus_t status = RCC_HandleClockFailure();
            printf("Recovery status: %s\r\n",
                   (status == CLOCK_CONFIG_SUCCESS) ? "SUCCESS" : "FAILED");
        }
    }

    /* Update and display current clock information */
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
    RCC_DisplayClockInfo(&g_SystemClockInfo);
}

/**
 * @brief   Display current system status
 */
static void DisplaySystemStatus(void)
{
    printf("\r\n--- System Status ---\r\n");
    printf("System uptime: %lu seconds\r\n", s_SystemTicks / 1000);
    printf("Current power mode: ");

    switch (s_CurrentPowerMode)
    {
        case POWER_MODE_FULL_PERFORMANCE:
            printf("FULL PERFORMANCE\r\n");
            break;
        case POWER_MODE_NORMAL:
            printf("NORMAL\r\n");
            break;
        case POWER_MODE_LOW_POWER:
            printf("LOW POWER\r\n");
            break;
        case POWER_MODE_SLEEP_READY:
            printf("SLEEP READY\r\n");
            break;
    }

    printf("Clock error count: %lu\r\n", RCC_GetClockErrorCount());
    printf("HSE failure detected: %s\r\n", RCC_IsHSEFailureDetected() ? "YES" : "NO");

    /* Display current peripheral power consumption estimate */
    uint32_t activePeripherals = 0;
    if (g_PeripheralClockStatus.GPIO_A_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.GPIO_B_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.GPIO_C_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.USART1_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.SPI1_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.I2C1_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.TIM2_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.TIM3_Enabled) activePeripherals++;
    if (g_PeripheralClockStatus.ADC1_Enabled) activePeripherals++;

    printf("Active peripherals: %lu/9\r\n", activePeripherals);
    printf("Estimated power level: %s\r\n",
           (activePeripherals > 6) ? "HIGH" :
           (activePeripherals > 3) ? "MEDIUM" : "LOW");
}

/**
 * @brief   Demonstrate different power modes
 */
static void DemonstratePowerModes(void)
{
    static PowerMode_t demoMode = POWER_MODE_FULL_PERFORMANCE;

    printf("\r\n=== Power Mode Demonstration ===\r\n");
    printf("Demonstrating power mode: ");

    switch (demoMode)
    {
        case POWER_MODE_FULL_PERFORMANCE:
            printf("FULL PERFORMANCE\r\n");
            printf("All peripherals enabled for maximum functionality\r\n");
            break;

        case POWER_MODE_NORMAL:
            printf("NORMAL\r\n");
            printf("Essential peripherals only for balanced operation\r\n");
            break;

        case POWER_MODE_LOW_POWER:
            printf("LOW POWER\r\n");
            printf("Minimal peripherals for power conservation\r\n");
            break;

        case POWER_MODE_SLEEP_READY:
            printf("SLEEP READY\r\n");
            printf("Preparing for sleep mode - minimal power consumption\r\n");
            break;
    }

    /* Apply demonstration power mode */
    RCC_PeripheralClockConfig(demoMode);

    /* Wait a moment to show the effect */
    for (volatile uint32_t i = 0; i < 2000000; i++);

    /* Display peripheral status for this mode */
    RCC_GetPeripheralClockStatus(&g_PeripheralClockStatus);
    RCC_DisplayPeripheralStatus(&g_PeripheralClockStatus);

    /* Restore original power mode */
    RCC_PeripheralClockConfig(s_CurrentPowerMode);

    /* Cycle to next demonstration mode */
    demoMode = (PowerMode_t)((demoMode + 1) % 4);
}

/**
 * @brief   Demonstrate different clock sources and configurations
 */
static void DemonstrateClockSources(void)
{
    printf("\r\n=== Clock Source Demonstration ===\r\n");

    /* Save current clock configuration */
    SystemClockInfo_t originalClocks;
    RCC_GetSystemClockInfo(&originalClocks);

    printf("Current configuration (HSE + PLL):\r\n");
    RCC_DisplayClockInfo(&originalClocks);

    /* Demonstrate ADC clock prescaler changes */
    printf("\r\nDemonstrating ADC clock prescaler changes:\r\n");

    printf("Setting ADC prescaler to /2...\r\n");
    RCC_ConfigureADCClock(RCC_PCLK2_Div2);
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
    printf("ADC Clock: %lu Hz\r\n", g_SystemClockInfo.ADCCLK_Hz);

    printf("Setting ADC prescaler to /4...\r\n");
    RCC_ConfigureADCClock(RCC_PCLK2_Div4);
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
    printf("ADC Clock: %lu Hz\r\n", g_SystemClockInfo.ADCCLK_Hz);

    printf("Setting ADC prescaler to /6...\r\n");
    RCC_ConfigureADCClock(RCC_PCLK2_Div6);
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
    printf("ADC Clock: %lu Hz\r\n", g_SystemClockInfo.ADCCLK_Hz);

    printf("Restoring original ADC prescaler (/6)...\r\n");
    RCC_ConfigureADCClock(RCC_PCLK2_Div6);

    printf("Clock source demonstration complete.\r\n");
}

/**
 * @brief   Handle clock configuration failure
 */
static void HandleClockFailure(void)
{
    printf("CRITICAL: Clock configuration failed!\r\n");
    printf("Attempting to reset to default clock configuration...\r\n");

    /* Reset to default HSI clock */
    RCC_ResetToDefaultClock();

    /* Initialize hardware with HSI clock */
    Hardware_Init();

    printf("System running on HSI clock (8 MHz)\r\n");
    printf("Attempting to reconfigure HSE + PLL...\r\n");

    /* Try to reconfigure the clock system */
    ClockConfigStatus_t status = RCC_SystemClockConfig();

    if (status == CLOCK_CONFIG_SUCCESS)
    {
        printf("Clock reconfiguration successful!\r\n");
        /* Re-initialize hardware with new clock */
        Hardware_Init();
    }
    else
    {
        printf("Clock reconfiguration failed - continuing with HSI\r\n");
    }
}

/**
 * @brief   SysTick interrupt handler
 */
void SysTick_Handler(void)
{
    /* Increment system tick counter */
    /* Note: This is a simple implementation for demonstration */
    /* In a real application, you might use a proper RTOS or timing system */
}

/**
 * @brief   NMI interrupt handler (for Clock Security System)
 */
void NMI_Handler(void)
{
    /* Handle Clock Security System interrupt */
    RCC_CSS_IRQHandler();
}

/**
 * @brief   Printf redirection to UART
 */
int fputc(int ch, FILE *f)
{
    /* Wait until transmit data register is empty */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

    /* Send character */
    USART_SendData(USART1, (uint8_t)ch);

    return ch;
}
