/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: systick.h
 * Author: hoangphuc540202@gmail.com
 * Date: 25/9/2025
 *
 * Description: SysTick timer driver for precise timing and delay functions
 * Features:
 *   - 1ms tick resolution
 *   - System uptime tracking
 *   - Precise millisecond delays
 *   - Timeout support for UART operations
 *   - Low power sleep delays (optional)
 * =============================================================================
 */

#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

/*==================================================================================================
*                                        DEFINES AND MACROS
==================================================================================================*/

/* SysTick Configuration */
#define SYSTICK_FREQUENCY_HZ        1000U           /* 1ms tick (1000 Hz) */
#define SYSTICK_MAX_DELAY_MS        0x0FFFFFFFU     /* Maximum delay in ms */

/* Timeout values for different operations */
#define SYSTICK_UART_TIMEOUT_MS     1000U           /* Default UART timeout */
#define SYSTICK_I2C_TIMEOUT_MS      100U            /* I2C operation timeout */
#define SYSTICK_SPI_TIMEOUT_MS      100U            /* SPI operation timeout */

/* Enable/Disable features */
#define SYSTICK_USE_WFI_DELAY       0               /* Use WFI in delay for power saving */
#define SYSTICK_ENABLE_OVERFLOW     1               /* Handle tick counter overflow */
#define SYSTICK_ENABLE_STATISTICS   0               /* Enable timing statistics */

/*==================================================================================================
*                                 STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

/**
 * @brief SysTick Statistics Structure (if enabled)
 */
#if (SYSTICK_ENABLE_STATISTICS == 1)
typedef struct
{
    uint32_t TotalDelays;           /* Total number of delays performed */
    uint32_t MaxDelayMs;            /* Maximum delay requested */
    uint32_t TotalDelayTimeMs;      /* Total time spent in delays */
    uint32_t AverageDelayMs;        /* Average delay time */
} SysTick_StatsTypeDef;
#endif

/**
 * @brief Timeout handle for non-blocking operations
 */
typedef struct
{
    uint32_t StartTick;             /* Start time in ticks */
    uint32_t TimeoutMs;             /* Timeout duration in ms */
    bool     IsActive;              /* Timeout is active */
} SysTick_TimeoutTypeDef;

/*==================================================================================================
*                                 GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/* System tick counter - incremented every 1ms by SysTick interrupt */
extern volatile uint32_t system_ticks;

#if (SYSTICK_ENABLE_STATISTICS == 1)
/* SysTick statistics (if enabled) */
extern SysTick_StatsTypeDef systick_stats;
#endif

/*==================================================================================================
*                                     FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief Initialize SysTick timer for 1ms interrupts
 * @details Configures SysTick to generate interrupts every 1ms based on system clock
 * @note Must be called after system clock configuration
 * @param None
 * @return None
 */
void SysTick_Init(void);

/**
 * @brief Blocking delay function
 * @details Blocks execution for specified milliseconds using SysTick counter
 * @param ms: Delay time in milliseconds (0 to SYSTICK_MAX_DELAY_MS)
 * @return None
 * @note This function blocks CPU execution. Use timeout functions for non-blocking delays
 */
void SysTick_DelayMs(uint32_t ms);

/**
 * @brief Get current system uptime in milliseconds
 * @details Returns the number of milliseconds since SysTick initialization
 * @param None
 * @return Current system time in milliseconds
 * @note Handles 32-bit overflow (wraps around after ~49.7 days)
 */
uint32_t SysTick_GetTick(void);

/**
 * @brief Get elapsed time since a reference tick
 * @details Calculates time difference handling counter overflow
 * @param start_tick: Reference tick value from SysTick_GetTick()
 * @return Elapsed time in milliseconds
 * @note Handles overflow correctly for differences up to 2^31 ms (~24.8 days)
 */
uint32_t SysTick_GetElapsedMs(uint32_t start_tick);

/**
 * @brief Initialize a timeout handle
 * @details Sets up timeout structure for non-blocking timeout checking
 * @param timeout: Pointer to timeout handle
 * @param timeout_ms: Timeout duration in milliseconds
 * @return None
 */
void SysTick_TimeoutStart(SysTick_TimeoutTypeDef *timeout, uint32_t timeout_ms);

/**
 * @brief Check if timeout has expired
 * @details Non-blocking check if timeout duration has elapsed
 * @param timeout: Pointer to timeout handle
 * @return true if timeout expired, false if still within timeout period
 */
bool SysTick_TimeoutExpired(SysTick_TimeoutTypeDef *timeout);

/**
 * @brief Stop/cancel a timeout
 * @details Deactivates timeout handle
 * @param timeout: Pointer to timeout handle
 * @return None
 */
void SysTick_TimeoutStop(SysTick_TimeoutTypeDef *timeout);

/**
 * @brief Precise microsecond delay (approximate)
 * @details Software delay for short microsecond delays
 * @param us: Delay time in microseconds (1-1000)
 * @return None
 * @note Less accurate than millisecond delays, depends on system clock
 */
void SysTick_DelayUs(uint32_t us);

/**
 * @brief Low power delay using WFI (Wait For Interrupt)
 * @details Power-saving delay that puts CPU to sleep between ticks
 * @param ms: Delay time in milliseconds
 * @return None
 * @note CPU wakes up on any interrupt, may affect real-time performance
 */
void SysTick_DelayMsLowPower(uint32_t ms);

/**
 * @brief Check if SysTick is initialized and running
 * @details Verifies SysTick timer is properly configured
 * @param None
 * @return true if SysTick is running, false otherwise
 */
bool SysTick_IsInitialized(void);

#if (SYSTICK_ENABLE_STATISTICS == 1)
/**
 * @brief Get SysTick usage statistics
 * @details Returns timing statistics if enabled
 * @param stats: Pointer to statistics structure
 * @return None
 */
void SysTick_GetStatistics(SysTick_StatsTypeDef *stats);

/**
 * @brief Reset SysTick statistics
 * @details Clears all timing statistics
 * @param None
 * @return None
 */
void SysTick_ResetStatistics(void);
#endif

/*==================================================================================================
*                                       INLINE FUNCTIONS
==================================================================================================*/

/**
 * @brief Fast inline version of SysTick_GetTick()
 * @details Inline function for performance-critical code
 * @param None
 * @return Current system tick count
 */
static inline uint32_t SysTick_GetTickFast(void)
{
    return system_ticks;
}

/**
 * @brief Check if enough time has elapsed (inline version)
 * @details Fast inline check for time elapsed since reference
 * @param start_tick: Reference tick value
 * @param timeout_ms: Timeout value in milliseconds
 * @return true if timeout elapsed, false otherwise
 */
static inline bool SysTick_IsTimeoutFast(uint32_t start_tick, uint32_t timeout_ms)
{
    return ((system_ticks - start_tick) >= timeout_ms);
}

/*==================================================================================================
*                                        COMPATIBILITY
==================================================================================================*/

/* Backward compatibility macros */
#define delay_ms(ms)            SysTick_DelayMs(ms)
#define get_system_ms()         SysTick_GetTick()
#define get_tick()              SysTick_GetTick()

/* HAL compatibility */
#define HAL_GetTick()           SysTick_GetTick()
#define HAL_Delay(ms)           SysTick_DelayMs(ms)

#endif /* __SYSTICK_H */