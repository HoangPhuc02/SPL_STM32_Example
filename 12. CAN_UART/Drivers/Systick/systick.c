/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: systick.c
 * Author: hoangphuc540202@gmail.com
 * Date: 25/9/2025
 *
 * Description: SysTick timer driver implementation for precise timing
 * Features:
 *   - 1ms tick resolution with overflow handling
 *   - System uptime tracking with 32-bit wraparound protection
 *   - Blocking and non-blocking delay functions
 *   - Timeout support for peripheral operations
 *   - Optional power-saving delays
 * =============================================================================
 */

#include "systick.h"

/*==================================================================================================
*                                        LOCAL VARIABLES
==================================================================================================*/

/* System tick counter - incremented every 1ms by SysTick interrupt */
volatile uint32_t system_ticks = 0;

/* SysTick initialization flag */
static bool systick_initialized = false;

/* SysTick configuration parameters */
static uint32_t systick_reload_value = 0;

#if (SYSTICK_ENABLE_STATISTICS == 1)
/* SysTick usage statistics */
SysTick_StatsTypeDef systick_stats = {0, 0, 0, 0};
#endif

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

#if (SYSTICK_ENABLE_STATISTICS == 1)
static void SysTick_UpdateStatistics(uint32_t delay_ms);
#endif

/*==================================================================================================
*                                       PUBLIC FUNCTIONS
==================================================================================================*/

/**
 * @brief Initialize SysTick timer for 1ms interrupts
 * @details Configures SysTick to generate interrupts every 1ms based on system clock
 *          Uses SysTick_Config() for automatic configuration with processor clock
 * @param None
 * @return None
 */
void SysTick_Init(void)
{
    /* Configure SysTick for 1ms interrupts using processor clock (HCLK) */
    /* SysTick_Config() automatically:
     * - Sets LOAD register = (SystemCoreClock / 1000) - 1
     * - Clears VAL register = 0
     * - Enables SysTick counter
     * - Enables SysTick interrupt
     * - Uses processor clock (HCLK) as clock source
     */
    systick_reload_value = SystemCoreClock / SYSTICK_FREQUENCY_HZ - 1;
    
    if (SysTick_Config(SystemCoreClock / SYSTICK_FREQUENCY_HZ))
    {
        /* Configuration failed - system clock too low or other error */
        while (1)
        {
            /* Error: Cannot configure SysTick - check system clock */
            __NOP();
        }
    }
    
    /* Reset system tick counter */
    system_ticks = 0;
    
    /* Mark as initialized */
    systick_initialized = true;
    
#if (SYSTICK_ENABLE_STATISTICS == 1)
    /* Reset statistics */
    SysTick_ResetStatistics();
#endif
}

/**
 * @brief SysTick interrupt handler
 * @details Called every 1ms by SysTick timer interrupt
 *          Increments system tick counter with overflow handling
 * @param None
 * @return None
 * @note This function is called from interrupt context
 *       Function name MUST be "SysTick_Handler" - required by CMSIS
 */
void SysTick_Handler(void)
{
    /* Increment system tick counter */
    system_ticks++;
    
#if (SYSTICK_ENABLE_OVERFLOW == 1)
    /* Handle overflow at maximum 32-bit value */
    if (system_ticks == 0xFFFFFFFF)
    {
        system_ticks = 0;   /* Wrap around to 0 */
    }
#endif
}

/**
 * @brief Blocking delay function using SysTick
 * @details Blocks execution for specified milliseconds
 * @param ms: Delay time in milliseconds (0 to SYSTICK_MAX_DELAY_MS)
 * @return None
 */
void SysTick_DelayMs(uint32_t ms)
{
    uint32_t start_tick;
    
    /* Validate input parameter */
    if (ms == 0 || ms > SYSTICK_MAX_DELAY_MS)
    {
        return;     /* Invalid delay time */
    }
    
    /* Check if SysTick is initialized */
    if (!systick_initialized)
    {
        /* Fallback to simple loop delay (less accurate) */
        volatile uint32_t count = ms * (SystemCoreClock / 8000);
        while(count--) { __NOP(); }
        return;
    }
    
    /* Record start time */
    start_tick = system_ticks;
    
    /* Wait for specified time to elapse */
    while ((system_ticks - start_tick) < ms)
    {
#if (SYSTICK_USE_WFI_DELAY == 1)
        __WFI();    /* Wait for interrupt (power saving) */
#else
        __NOP();    /* No operation (keep CPU active) */
#endif
    }
    
#if (SYSTICK_ENABLE_STATISTICS == 1)
    /* Update usage statistics */
    SysTick_UpdateStatistics(ms);
#endif
}

/**
 * @brief Get current system uptime in milliseconds
 * @details Returns the number of milliseconds since SysTick initialization
 * @param None
 * @return Current system time in milliseconds
 */
uint32_t SysTick_GetTick(void)
{
    return system_ticks;
}

/**
 * @brief Get elapsed time since a reference tick
 * @details Calculates time difference handling counter overflow correctly
 * @param start_tick: Reference tick value from SysTick_GetTick()
 * @return Elapsed time in milliseconds
 */
uint32_t SysTick_GetElapsedMs(uint32_t start_tick)
{
    uint32_t current_tick = system_ticks;
    
    /* Handle counter wraparound correctly */
    if (current_tick >= start_tick)
    {
        return (current_tick - start_tick);
    }
    else
    {
        /* Counter has wrapped around */
        return ((0xFFFFFFFF - start_tick) + current_tick + 1);
    }
}

/**
 * @brief Initialize a timeout handle
 * @details Sets up timeout structure for non-blocking timeout checking
 * @param timeout: Pointer to timeout handle
 * @param timeout_ms: Timeout duration in milliseconds
 * @return None
 */
void SysTick_TimeoutStart(SysTick_TimeoutTypeDef *timeout, uint32_t timeout_ms)
{
    if (timeout != NULL)
    {
        timeout->StartTick = system_ticks;
        timeout->TimeoutMs = timeout_ms;
        timeout->IsActive = true;
    }
}

/**
 * @brief Check if timeout has expired
 * @details Non-blocking check if timeout duration has elapsed
 * @param timeout: Pointer to timeout handle
 * @return true if timeout expired, false if still within timeout period
 */
bool SysTick_TimeoutExpired(SysTick_TimeoutTypeDef *timeout)
{
    if (timeout == NULL || !timeout->IsActive)
    {
        return false;
    }
    
    return (SysTick_GetElapsedMs(timeout->StartTick) >= timeout->TimeoutMs);
}

/**
 * @brief Stop/cancel a timeout
 * @details Deactivates timeout handle
 * @param timeout: Pointer to timeout handle
 * @return None
 */
void SysTick_TimeoutStop(SysTick_TimeoutTypeDef *timeout)
{
    if (timeout != NULL)
    {
        timeout->IsActive = false;
    }
}

/**
 * @brief Precise microsecond delay (approximate)
 * @details Software delay for short microsecond delays
 * @param us: Delay time in microseconds (1-1000)
 * @return None
 */
void SysTick_DelayUs(uint32_t us)
{
    volatile uint32_t count;
    
    /* Limit microsecond delay to reasonable range */
    if (us > 1000)
    {
        us = 1000;
    }
    
    /* Calculate loop count based on system clock */
    /* Approximate: 1us = SystemCoreClock/1000000 cycles */
    count = (us * (SystemCoreClock / 8000000));  /* Rough calibration */
    
    while(count--)
    {
        __NOP();
    }
}

/**
 * @brief Low power delay using WFI (Wait For Interrupt)
 * @details Power-saving delay that puts CPU to sleep between ticks
 * @param ms: Delay time in milliseconds
 * @return None
 */
void SysTick_DelayMsLowPower(uint32_t ms)
{
    uint32_t start_tick;
    
    if (ms == 0 || !systick_initialized)
    {
        return;
    }
    
    start_tick = system_ticks;
    
    while ((system_ticks - start_tick) < ms)
    {
        __WFI();    /* Wait for interrupt - reduces power consumption */
    }
}

/**
 * @brief Check if SysTick is initialized and running
 * @details Verifies SysTick timer is properly configured
 * @param None
 * @return true if SysTick is running, false otherwise
 */
bool SysTick_IsInitialized(void)
{
    return systick_initialized && ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0);
}

/*==================================================================================================
*                                     STATISTICS FUNCTIONS
==================================================================================================*/

#if (SYSTICK_ENABLE_STATISTICS == 1)
/**
 * @brief Update SysTick usage statistics
 * @details Internal function to track delay usage patterns
 * @param delay_ms: Delay time that was executed
 * @return None
 */
static void SysTick_UpdateStatistics(uint32_t delay_ms)
{
    systick_stats.TotalDelays++;
    systick_stats.TotalDelayTimeMs += delay_ms;
    
    if (delay_ms > systick_stats.MaxDelayMs)
    {
        systick_stats.MaxDelayMs = delay_ms;
    }
    
    /* Calculate average delay */
    systick_stats.AverageDelayMs = systick_stats.TotalDelayTimeMs / systick_stats.TotalDelays;
}

/**
 * @brief Get SysTick usage statistics
 * @details Returns timing statistics if enabled
 * @param stats: Pointer to statistics structure
 * @return None
 */
void SysTick_GetStatistics(SysTick_StatsTypeDef *stats)
{
    if (stats != NULL)
    {
        *stats = systick_stats;
    }
}

/**
 * @brief Reset SysTick statistics
 * @details Clears all timing statistics
 * @param None
 * @return None
 */
void SysTick_ResetStatistics(void)
{
    systick_stats.TotalDelays = 0;
    systick_stats.MaxDelayMs = 0;
    systick_stats.TotalDelayTimeMs = 0;
    systick_stats.AverageDelayMs = 0;
}
#endif

/*==================================================================================================
*                                  COMPATIBILITY FUNCTIONS
==================================================================================================*/

/**
 * @brief Legacy delay function (for backward compatibility)
 * @details Wrapper for SysTick_DelayMs()
 * @param ms: Delay time in milliseconds
 * @return None
 */
void delay_ms(uint32_t ms)
{
    SysTick_DelayMs(ms);
}

/**
 * @brief Legacy get tick function (for backward compatibility)
 * @details Wrapper for SysTick_GetTick()
 * @param None
 * @return Current system time in milliseconds
 */
uint32_t get_system_ms(void)
{
    return SysTick_GetTick();
}