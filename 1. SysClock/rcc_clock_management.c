/**
 * @file    rcc_clock_management.c
 * @brief   Comprehensive RCC Clock Management Implementation for STM32F103C8T6
 * @details This file implements comprehensive clock management functions using
 *          STM32 Standard Peripheral Library. It demonstrates HSE+PLL configuration,
 *          peripheral clock control, clock monitoring, and power management.
 * 
 * @author  STM32 Development Team
 * @date    2025-01-08
 * @version 1.0.0
 * 
 * @note    Target: STM32F103C8T6 with 8MHz HSE crystal
 *          Achieves 72MHz system clock using PLL
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "rcc_clock_management.h"

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
SystemClockInfo_t g_SystemClockInfo;
PeripheralClockStatus_t g_PeripheralClockStatus;
volatile bool g_ClockMonitoringEnabled = false;

/*==================================================================================================
*                                       LOCAL VARIABLES
==================================================================================================*/
static volatile uint32_t s_ClockErrorCount = 0;
static volatile bool s_HSEFailureDetected = false;

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
static void RCC_ConfigureFlashLatency(void);
static bool RCC_WaitForHSEReady(void);
static bool RCC_WaitForPLLReady(void);
static bool RCC_SwitchToSystemClock(uint32_t clockSource);

/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

/**
 * @brief   Initialize system clock to 72MHz using HSE + PLL
 */
ClockConfigStatus_t RCC_SystemClockConfig(void)
{
    ErrorStatus HSEStatus = ERROR;
    uint32_t timeout = 0;
    
    /* Reset RCC configuration to default state */
    RCC_DeInit();
    
    /* Enable HSE (High Speed External) oscillator */
    RCC_HSEConfig(RCC_HSE_ON);
    
    /* Wait for HSE to be ready */
    if (!RCC_WaitForHSEReady())
    {
        return CLOCK_CONFIG_HSE_FAIL;
    }
    
    /* Configure Flash latency for 72MHz operation */
    RCC_ConfigureFlashLatency();
    
    /* Configure AHB clock (HCLK) = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    
    /* Configure APB2 clock (PCLK2) = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    /* Configure APB1 clock (PCLK1) = HCLK/2 (max 36MHz) */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    /* Configure PLL: HSE * 9 = 8MHz * 9 = 72MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, PLL_MULTIPLIER);
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    
    /* Wait for PLL to be ready */
    if (!RCC_WaitForPLLReady())
    {
        return CLOCK_CONFIG_PLL_FAIL;
    }
    
    /* Switch system clock to PLL */
    if (!RCC_SwitchToSystemClock(RCC_SYSCLKSource_PLLCLK))
    {
        return CLOCK_CONFIG_SWITCH_FAIL;
    }
    
    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();
    
    /* Configure ADC clock (ADCCLK = PCLK2/6 = 72MHz/6 = 12MHz) */
    RCC_ConfigureADCClock(RCC_PCLK2_Div6);
    
    /* Enable Clock Security System */
    RCC_EnableClockSecurity();
    
    /* Update clock information */
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
    
    return CLOCK_CONFIG_SUCCESS;
}

/**
 * @brief   Configure peripheral clocks for specific power mode
 */
void RCC_PeripheralClockConfig(PowerMode_t mode)
{
    /* First disable all peripheral clocks */
    RCC_DisablePeripheralClocks();
    
    switch (mode)
    {
        case POWER_MODE_FULL_PERFORMANCE:
            /* Enable all peripheral clocks for maximum performance */
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                                  RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO |
                                  RCC_APB2Periph_USART1 | RCC_APB2Periph_SPI1 |
                                  RCC_APB2Periph_ADC1, ENABLE);
            
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2 |
                                  RCC_APB1Periph_TIM3, ENABLE);
            break;
            
        case POWER_MODE_NORMAL:
            /* Enable essential peripherals only */
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                                  RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
            
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
            break;
            
        case POWER_MODE_LOW_POWER:
            /* Enable minimal peripherals */
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
            break;
            
        case POWER_MODE_SLEEP_READY:
            /* Disable all non-essential peripherals */
            /* Keep only GPIOA for wakeup pin */
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            break;
            
        default:
            /* Default to normal mode */
            RCC_PeripheralClockConfig(POWER_MODE_NORMAL);
            break;
    }
    
    /* Update peripheral clock status */
    RCC_GetPeripheralClockStatus(&g_PeripheralClockStatus);
}

/**
 * @brief   Enable specific peripheral clocks
 */
void RCC_EnablePeripheralClocks(void)
{
    /* Enable APB2 peripheral clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    /* Enable APB1 peripheral clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

/**
 * @brief   Disable specific peripheral clocks
 */
void RCC_DisablePeripheralClocks(void)
{
    /* Disable APB2 peripheral clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
    
    /* Disable APB1 peripheral clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
}

/**
 * @brief   Get current system clock information
 */
void RCC_GetSystemClockInfo(SystemClockInfo_t* clockInfo)
{
    if (clockInfo == NULL) return;
    
    /* Get RCC clock frequencies */
    RCC_GetClocksFreq(&clockInfo->RCC_Clocks);
    
    /* Populate clock information structure */
    clockInfo->SystemClock_Hz = clockInfo->RCC_Clocks.SYSCLK_Frequency;
    clockInfo->HCLK_Hz = clockInfo->RCC_Clocks.HCLK_Frequency;
    clockInfo->PCLK1_Hz = clockInfo->RCC_Clocks.PCLK1_Frequency;
    clockInfo->PCLK2_Hz = clockInfo->RCC_Clocks.PCLK2_Frequency;
    clockInfo->ADCCLK_Hz = clockInfo->RCC_Clocks.ADCCLK_Frequency;
    
    /* Calculate timer clocks */
    RCC_CalculateTimerClocks(clockInfo);
}

/**
 * @brief   Get peripheral clock status
 */
void RCC_GetPeripheralClockStatus(PeripheralClockStatus_t* status)
{
    if (status == NULL) return;
    
    /* Check APB2 peripheral clocks */
    status->GPIO_A_Enabled = (RCC->APB2ENR & RCC_APB2Periph_GPIOA) ? true : false;
    status->GPIO_B_Enabled = (RCC->APB2ENR & RCC_APB2Periph_GPIOB) ? true : false;
    status->GPIO_C_Enabled = (RCC->APB2ENR & RCC_APB2Periph_GPIOC) ? true : false;
    status->USART1_Enabled = (RCC->APB2ENR & RCC_APB2Periph_USART1) ? true : false;
    status->SPI1_Enabled = (RCC->APB2ENR & RCC_APB2Periph_SPI1) ? true : false;
    status->ADC1_Enabled = (RCC->APB2ENR & RCC_APB2Periph_ADC1) ? true : false;
    status->AFIO_Enabled = (RCC->APB2ENR & RCC_APB2Periph_AFIO) ? true : false;
    
    /* Check APB1 peripheral clocks */
    status->I2C1_Enabled = (RCC->APB1ENR & RCC_APB1Periph_I2C1) ? true : false;
    status->TIM2_Enabled = (RCC->APB1ENR & RCC_APB1Periph_TIM2) ? true : false;
    status->TIM3_Enabled = (RCC->APB1ENR & RCC_APB1Periph_TIM3) ? true : false;
}

/**
 * @brief   Configure ADC clock prescaler
 */
void RCC_ConfigureADCClock(uint32_t prescaler)
{
    /* Configure ADC clock prescaler */
    RCC_ADCCLKConfig(prescaler);
    
    /* Update clock information */
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
}

/**
 * @brief   Monitor system clocks
 */
bool RCC_MonitorSystemClocks(void)
{
    SystemClockInfo_t currentClockInfo;
    bool clocksStable = true;
    
    /* Get current clock information */
    RCC_GetSystemClockInfo(&currentClockInfo);
    
    /* Check if system clock is at expected frequency (within 1% tolerance) */
    if ((currentClockInfo.SystemClock_Hz < (SYSTEM_CLOCK_TARGET_HZ * 99 / 100)) ||
        (currentClockInfo.SystemClock_Hz > (SYSTEM_CLOCK_TARGET_HZ * 101 / 100)))
    {
        clocksStable = false;
        s_ClockErrorCount++;
    }
    
    /* Check HSE status */
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
    {
        s_HSEFailureDetected = true;
        clocksStable = false;
        s_ClockErrorCount++;
    }
    
    /* Check PLL status */
    if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
        clocksStable = false;
        s_ClockErrorCount++;
    }
    
    return clocksStable;
}

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/

/**
 * @brief   Configure Flash latency for high-speed operation
 */
static void RCC_ConfigureFlashLatency(void)
{
    /* Enable Prefetch Buffer and set Flash Latency for 72MHz */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_LATENCY_CYCLES);
}

/**
 * @brief   Wait for HSE to be ready with timeout
 */
static bool RCC_WaitForHSEReady(void)
{
    uint32_t timeout = HSE_STARTUP_TIMEOUT;
    
    while ((RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) && (timeout > 0))
    {
        timeout--;
    }
    
    return (timeout > 0);
}

/**
 * @brief   Wait for PLL to be ready with timeout
 */
static bool RCC_WaitForPLLReady(void)
{
    uint32_t timeout = PLL_STARTUP_TIMEOUT;
    
    while ((RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) && (timeout > 0))
    {
        timeout--;
    }
    
    return (timeout > 0);
}

/**
 * @brief   Switch system clock source with timeout
 */
static bool RCC_SwitchToSystemClock(uint32_t clockSource)
{
    uint32_t timeout = CLOCK_SWITCH_TIMEOUT;
    
    /* Select system clock source */
    RCC_SYSCLKConfig(clockSource);
    
    /* Wait until the clock source is used as system clock */
    while ((RCC_GetSYSCLKSource() != (clockSource << 2)) && (timeout > 0))
    {
        timeout--;
    }
    
    return (timeout > 0);
}

/**
 * @brief   Display clock information via UART
 */
void RCC_DisplayClockInfo(const SystemClockInfo_t* clockInfo)
{
    if (clockInfo == NULL) return;

    /* Note: This function assumes UART is initialized and printf is redirected to UART */
    printf("\r\n=== STM32F103C8T6 Clock Information ===\r\n");
    printf("System Clock (SYSCLK): %lu Hz (%.2f MHz)\r\n",
           clockInfo->SystemClock_Hz, (float)clockInfo->SystemClock_Hz / 1000000.0f);
    printf("AHB Clock (HCLK):      %lu Hz (%.2f MHz)\r\n",
           clockInfo->HCLK_Hz, (float)clockInfo->HCLK_Hz / 1000000.0f);
    printf("APB1 Clock (PCLK1):    %lu Hz (%.2f MHz)\r\n",
           clockInfo->PCLK1_Hz, (float)clockInfo->PCLK1_Hz / 1000000.0f);
    printf("APB2 Clock (PCLK2):    %lu Hz (%.2f MHz)\r\n",
           clockInfo->PCLK2_Hz, (float)clockInfo->PCLK2_Hz / 1000000.0f);
    printf("ADC Clock (ADCCLK):    %lu Hz (%.2f MHz)\r\n",
           clockInfo->ADCCLK_Hz, (float)clockInfo->ADCCLK_Hz / 1000000.0f);
    printf("Timer Clock 1:         %lu Hz (%.2f MHz)\r\n",
           clockInfo->TIMCLK1_Hz, (float)clockInfo->TIMCLK1_Hz / 1000000.0f);
    printf("Timer Clock 2:         %lu Hz (%.2f MHz)\r\n",
           clockInfo->TIMCLK2_Hz, (float)clockInfo->TIMCLK2_Hz / 1000000.0f);

    /* Display clock source information */
    uint8_t clockSource = RCC_GetSYSCLKSource();
    printf("Clock Source: ");
    switch (clockSource)
    {
        case 0x00: printf("HSI (8 MHz)\r\n"); break;
        case 0x04: printf("HSE (%lu MHz)\r\n", HSE_FREQUENCY_HZ / 1000000); break;
        case 0x08: printf("PLL\r\n"); break;
        default: printf("Unknown\r\n"); break;
    }

    /* Display HSE and PLL status */
    printf("HSE Status: %s\r\n", (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == SET) ? "Ready" : "Not Ready");
    printf("PLL Status: %s\r\n", (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == SET) ? "Ready" : "Not Ready");
    printf("CSS Status: %s\r\n", (RCC_GetFlagStatus(RCC_FLAG_CSSD) == SET) ? "Detected" : "Normal");
    printf("========================================\r\n");
}

/**
 * @brief   Handle clock failure
 */
ClockConfigStatus_t RCC_HandleClockFailure(void)
{
    /* Check if HSE failure occurred */
    if (s_HSEFailureDetected || (RCC_GetFlagStatus(RCC_FLAG_CSSD) == SET))
    {
        /* Clear CSS flag if set */
        if (RCC_GetFlagStatus(RCC_FLAG_CSSD) == SET)
        {
            RCC_ClearFlag(RCC_FLAG_CSSD);
        }

        /* Switch to HSI as emergency clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

        /* Wait for clock switch */
        uint32_t timeout = CLOCK_SWITCH_TIMEOUT;
        while ((RCC_GetSYSCLKSource() != 0x00) && (timeout > 0))
        {
            timeout--;
        }

        if (timeout == 0)
        {
            return CLOCK_CONFIG_TIMEOUT;
        }

        /* Update system core clock */
        SystemCoreClockUpdate();

        /* Reset HSE failure flag */
        s_HSEFailureDetected = false;

        /* Attempt to reconfigure HSE + PLL */
        return RCC_SystemClockConfig();
    }

    return CLOCK_CONFIG_SUCCESS;
}

/**
 * @brief   Reset to default clock configuration
 */
void RCC_ResetToDefaultClock(void)
{
    /* Reset RCC to default state */
    RCC_DeInit();

    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();

    /* Reset error counters */
    s_ClockErrorCount = 0;
    s_HSEFailureDetected = false;

    /* Update clock information */
    RCC_GetSystemClockInfo(&g_SystemClockInfo);
}

/**
 * @brief   Enable clock security system
 */
void RCC_EnableClockSecurity(void)
{
    /* Enable Clock Security System to detect HSE failures */
    RCC_ClockSecuritySystemCmd(ENABLE);
}

/**
 * @brief   Calculate timer clock frequencies
 */
void RCC_CalculateTimerClocks(SystemClockInfo_t* clockInfo)
{
    if (clockInfo == NULL) return;

    /* Timer clocks depend on APB prescaler settings */

    /* Timer clocks on APB1 (TIM2, TIM3, TIM4) */
    if (clockInfo->PCLK1_Hz == clockInfo->HCLK_Hz)
    {
        /* APB1 prescaler = 1, timer clock = PCLK1 */
        clockInfo->TIMCLK1_Hz = clockInfo->PCLK1_Hz;
    }
    else
    {
        /* APB1 prescaler > 1, timer clock = 2 * PCLK1 */
        clockInfo->TIMCLK1_Hz = 2 * clockInfo->PCLK1_Hz;
    }

    /* Timer clocks on APB2 (TIM1) */
    if (clockInfo->PCLK2_Hz == clockInfo->HCLK_Hz)
    {
        /* APB2 prescaler = 1, timer clock = PCLK2 */
        clockInfo->TIMCLK2_Hz = clockInfo->PCLK2_Hz;
    }
    else
    {
        /* APB2 prescaler > 1, timer clock = 2 * PCLK2 */
        clockInfo->TIMCLK2_Hz = 2 * clockInfo->PCLK2_Hz;
    }
}

/**
 * @brief   Clock Security System interrupt handler
 * @note    This function should be called from NMI_Handler
 */
void RCC_CSS_IRQHandler(void)
{
    /* Check if CSS flag is set */
    if (RCC_GetFlagStatus(RCC_FLAG_CSSD) == SET)
    {
        /* HSE failure detected */
        s_HSEFailureDetected = true;
        s_ClockErrorCount++;

        /* Clear CSS flag */
        RCC_ClearFlag(RCC_FLAG_CSSD);

        /* Handle clock failure in main loop */
        /* Note: Actual recovery should be done in main context, not in interrupt */
    }
}

/**
 * @brief   Get clock error statistics
 */
uint32_t RCC_GetClockErrorCount(void)
{
    return s_ClockErrorCount;
}

/**
 * @brief   Reset clock error statistics
 */
void RCC_ResetClockErrorCount(void)
{
    s_ClockErrorCount = 0;
    s_HSEFailureDetected = false;
}

/**
 * @brief   Check if HSE failure was detected
 */
bool RCC_IsHSEFailureDetected(void)
{
    return s_HSEFailureDetected;
}

/**
 * @brief   Configure USB clock (48MHz required)
 * @note    Only available on STM32F103 devices with USB
 */
void RCC_ConfigureUSBClock(void)
{
    /* USB clock = PLL clock / 1.5 = 72MHz / 1.5 = 48MHz */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
}

/**
 * @brief   Get peripheral clock frequency
 */
uint32_t RCC_GetPeripheralClock(uint32_t peripheral)
{
    SystemClockInfo_t clockInfo;
    RCC_GetSystemClockInfo(&clockInfo);

    /* Determine which bus the peripheral is on */
    if ((peripheral & 0x10000000) != 0) /* APB2 peripheral */
    {
        return clockInfo.PCLK2_Hz;
    }
    else if ((peripheral & 0x20000000) != 0) /* APB1 peripheral */
    {
        return clockInfo.PCLK1_Hz;
    }
    else /* AHB peripheral */
    {
        return clockInfo.HCLK_Hz;
    }
}

/**
 * @brief   Display peripheral clock status
 */
void RCC_DisplayPeripheralStatus(const PeripheralClockStatus_t* status)
{
    if (status == NULL) return;

    printf("\r\n=== Peripheral Clock Status ===\r\n");
    printf("GPIOA:  %s\r\n", status->GPIO_A_Enabled ? "Enabled" : "Disabled");
    printf("GPIOB:  %s\r\n", status->GPIO_B_Enabled ? "Enabled" : "Disabled");
    printf("GPIOC:  %s\r\n", status->GPIO_C_Enabled ? "Enabled" : "Disabled");
    printf("USART1: %s\r\n", status->USART1_Enabled ? "Enabled" : "Disabled");
    printf("SPI1:   %s\r\n", status->SPI1_Enabled ? "Enabled" : "Disabled");
    printf("I2C1:   %s\r\n", status->I2C1_Enabled ? "Enabled" : "Disabled");
    printf("TIM2:   %s\r\n", status->TIM2_Enabled ? "Enabled" : "Disabled");
    printf("TIM3:   %s\r\n", status->TIM3_Enabled ? "Enabled" : "Disabled");
    printf("ADC1:   %s\r\n", status->ADC1_Enabled ? "Enabled" : "Disabled");
    printf("AFIO:   %s\r\n", status->AFIO_Enabled ? "Enabled" : "Disabled");
    printf("===============================\r\n");
}
