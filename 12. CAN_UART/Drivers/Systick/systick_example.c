/*
 * =============================================================================
 * Project: STM32F103 CAN UART 
 * File: systick_example.c
 * Author: hoangphuc540202@gmail.com
 * Date: 25/9/2025
 *
 * Description: Example usage of SysTick driver with UART integration
 * Features:
 *   - Basic delay functions
 *   - Non-blocking timeout handling
 *   - System uptime tracking
 *   - UART timeout integration
 * =============================================================================
 */

#include "systick.h"
#include "uart.h"
#include "stm32f10x.h"

/*==================================================================================================
*                                       EXAMPLE FUNCTIONS
==================================================================================================*/

/**
 * @brief Example 1: Basic delay usage
 * @details Demonstrates simple blocking delays
 */
void SysTick_Example_BasicDelays(void)
{
    /* Initialize SysTick */
    SysTick_Init();
    
    /* Basic millisecond delays */
    SysTick_DelayMs(1000);      /* 1 second delay */
    SysTick_DelayMs(500);       /* 0.5 second delay */
    
    /* Microsecond delays for precise timing */
    SysTick_DelayUs(100);       /* 100 microseconds */
    SysTick_DelayUs(500);       /* 500 microseconds */
    
    /* Low power delay (uses WFI - Wait For Interrupt) */
    SysTick_DelayMsLowPower(2000);  /* 2 second power-saving delay */
}

/**
 * @brief Example 2: System uptime and elapsed time
 * @details Shows how to measure elapsed time
 */
void SysTick_Example_Timing(void)
{
    uint32_t start_time;
    uint32_t elapsed_time;
    uint32_t current_uptime;
    
    /* Get current system uptime */
    current_uptime = SysTick_GetTick();
    printf("System uptime: %lu ms\r\n", current_uptime);
    
    /* Measure elapsed time for an operation */
    start_time = SysTick_GetTick();
    
    /* Simulate some work */
    SysTick_DelayMs(1500);
    
    elapsed_time = SysTick_GetElapsedMs(start_time);
    printf("Operation took: %lu ms\r\n", elapsed_time);
}

/**
 * @brief Example 3: Non-blocking timeout handling
 * @details Shows timeout usage for peripheral operations
 */
void SysTick_Example_Timeouts(void)
{
    SysTick_TimeoutTypeDef uart_timeout;
    SysTick_TimeoutTypeDef general_timeout;
    bool operation_complete = false;
    uint32_t retry_count = 0;
    
    /* Example: UART transmission with timeout */
    SysTick_TimeoutStart(&uart_timeout, SYSTICK_UART_TIMEOUT_MS);
    
    while (!SysTick_TimeoutExpired(&uart_timeout))
    {
        /* Try to send data via UART */
        if (UART_TransmitReady(UART1))
        {
            operation_complete = true;
            break;
        }
        
        /* Small delay to prevent busy waiting */
        SysTick_DelayMs(10);
        retry_count++;
    }
    
    if (operation_complete)
    {
        printf("UART transmission successful after %lu retries\r\n", retry_count);
    }
    else
    {
        printf("UART transmission timeout after %d ms\r\n", SYSTICK_UART_TIMEOUT_MS);
    }
    
    /* Stop timeout when done */
    SysTick_TimeoutStop(&uart_timeout);
    
    /* Example: General operation timeout */
    SysTick_TimeoutStart(&general_timeout, 5000);  /* 5 second timeout */
    
    while (!SysTick_TimeoutExpired(&general_timeout))
    {
        /* Perform non-blocking operation */
        if (/* some condition */)
        {
            break;
        }
        
        /* Yield CPU briefly */
        SysTick_DelayMs(1);
    }
    
    SysTick_TimeoutStop(&general_timeout);
}

/**
 * @brief Example 4: UART integration with SysTick timing
 * @details Shows how UART driver uses SysTick for timeouts
 */
void SysTick_Example_UARTIntegration(void)
{
    UART_ConfigTypeDef uart_config;
    uint8_t tx_data[] = "Hello from UART with SysTick timing!\r\n";
    uint8_t rx_buffer[64];
    uint32_t bytes_received;
    uint32_t start_tick;
    
    /* Initialize SysTick first */
    SysTick_Init();
    
    /* Configure UART with timeout support */
    uart_config.UartNumber = UART1;
    uart_config.BaudRate = 115200;
    uart_config.DataBits = UART_DATABITS_8;
    uart_config.StopBits = UART_STOPBITS_1;
    uart_config.Parity = UART_PARITY_NONE;
    uart_config.FlowControl = UART_FLOWCONTROL_NONE;
    uart_config.Mode = UART_MODE_TX_RX;
    uart_config.TransmissionMode = UART_MODE_INTERRUPT;
    uart_config.TimeoutMs = 1000;  /* 1 second timeout */
    
    UART_Init(&uart_config);
    
    /* Transmit with timing measurement */
    start_tick = SysTick_GetTick();
    
    if (UART_Transmit(UART1, tx_data, sizeof(tx_data) - 1, 1000) == UART_OK)
    {
        uint32_t tx_time = SysTick_GetElapsedMs(start_tick);
        printf("Transmission completed in %lu ms\r\n", tx_time);
    }
    
    /* Receive with timeout */
    start_tick = SysTick_GetTick();
    
    if (UART_Receive(UART1, rx_buffer, sizeof(rx_buffer), &bytes_received, 2000) == UART_OK)
    {
        uint32_t rx_time = SysTick_GetElapsedMs(start_tick);
        printf("Received %lu bytes in %lu ms\r\n", bytes_received, rx_time);
    }
    else
    {
        printf("UART receive timeout\r\n");
    }
}

/**
 * @brief Example 5: Fast inline functions for performance-critical code
 * @details Shows usage of inline functions for minimal overhead
 */
void SysTick_Example_FastFunctions(void)
{
    uint32_t start_tick;
    uint32_t current_tick;
    
    /* Use fast inline version for performance-critical sections */
    start_tick = SysTick_GetTickFast();
    
    /* Critical timing section */
    /* ... time-sensitive code ... */
    
    current_tick = SysTick_GetTickFast();
    
    /* Fast timeout check */
    if (SysTick_IsTimeoutFast(start_tick, 100))
    {
        printf("Fast timeout detected\r\n");
    }
}

/**
 * @brief Example 6: SysTick status checking
 * @details Shows how to verify SysTick initialization
 */
void SysTick_Example_StatusCheck(void)
{
    /* Check if SysTick is initialized */
    if (SysTick_IsInitialized())
    {
        printf("SysTick is initialized and running\r\n");
        
        /* Show current system uptime */
        printf("System uptime: %lu ms\r\n", SysTick_GetTick());
    }
    else
    {
        printf("SysTick not initialized - initializing now...\r\n");
        SysTick_Init();
    }
}

#if (SYSTICK_ENABLE_STATISTICS == 1)
/**
 * @brief Example 7: SysTick statistics (if enabled)
 * @details Shows how to use timing statistics
 */
void SysTick_Example_Statistics(void)
{
    SysTick_StatsTypeDef stats;
    
    /* Perform some delays to generate statistics */
    SysTick_DelayMs(100);
    SysTick_DelayMs(500);
    SysTick_DelayMs(1000);
    SysTick_DelayMs(250);
    
    /* Get statistics */
    SysTick_GetStatistics(&stats);
    
    printf("SysTick Statistics:\r\n");
    printf("- Total delays: %lu\r\n", stats.TotalDelays);
    printf("- Max delay: %lu ms\r\n", stats.MaxDelayMs);
    printf("- Total delay time: %lu ms\r\n", stats.TotalDelayTimeMs);
    printf("- Average delay: %lu ms\r\n", stats.AverageDelayMs);
    
    /* Reset statistics */
    SysTick_ResetStatistics();
    printf("Statistics reset\r\n");
}
#endif

/**
 * @brief Main example function
 * @details Demonstrates all SysTick features
 */
void SysTick_RunAllExamples(void)
{
    printf("\r\n=== SysTick Driver Examples ===\r\n");
    
    /* Example 1: Basic delays */
    printf("\r\n1. Basic Delays Example\r\n");
    SysTick_Example_BasicDelays();
    
    /* Example 2: Timing measurements */
    printf("\r\n2. Timing Measurement Example\r\n");
    SysTick_Example_Timing();
    
    /* Example 3: Timeout handling */
    printf("\r\n3. Timeout Handling Example\r\n");
    SysTick_Example_Timeouts();
    
    /* Example 4: UART integration */
    printf("\r\n4. UART Integration Example\r\n");
    SysTick_Example_UARTIntegration();
    
    /* Example 5: Fast functions */
    printf("\r\n5. Fast Functions Example\r\n");
    SysTick_Example_FastFunctions();
    
    /* Example 6: Status checking */
    printf("\r\n6. Status Check Example\r\n");
    SysTick_Example_StatusCheck();
    
#if (SYSTICK_ENABLE_STATISTICS == 1)
    /* Example 7: Statistics (if enabled) */
    printf("\r\n7. Statistics Example\r\n");
    SysTick_Example_Statistics();
#endif
    
    printf("\r\n=== All Examples Complete ===\r\n");
}

/*==================================================================================================
*                                      UTILITY FUNCTIONS
==================================================================================================*/

/**
 * @brief Demonstrate SysTick overflow handling
 * @details Shows how the driver handles 32-bit counter overflow
 */
void SysTick_Example_OverflowTest(void)
{
    uint32_t start_tick;
    uint32_t elapsed;
    
    printf("Testing overflow handling...\r\n");
    
    /* Simulate near-overflow condition */
    system_ticks = 0xFFFFFFFE;  /* Set to near maximum value */
    
    start_tick = SysTick_GetTick();
    printf("Start tick: 0x%08lX\r\n", start_tick);
    
    /* Wait for overflow to occur */
    SysTick_DelayMs(10);
    
    elapsed = SysTick_GetElapsedMs(start_tick);
    printf("Elapsed time across overflow: %lu ms\r\n", elapsed);
    
    printf("Current tick after overflow: 0x%08lX\r\n", SysTick_GetTick());
}

/**
 * @brief Performance test for SysTick functions
 * @details Measures performance of different SysTick functions
 */
void SysTick_Example_PerformanceTest(void)
{
    uint32_t i;
    uint32_t start_tick;
    uint32_t test_iterations = 10000;
    
    printf("SysTick Performance Test (%lu iterations):\r\n", test_iterations);
    
    /* Test SysTick_GetTick() performance */
    start_tick = SysTick_GetTick();
    for (i = 0; i < test_iterations; i++)
    {
        volatile uint32_t tick = SysTick_GetTick();
        (void)tick;  /* Prevent optimization */
    }
    printf("SysTick_GetTick(): %lu ms\r\n", SysTick_GetElapsedMs(start_tick));
    
    /* Test SysTick_GetTickFast() performance */
    start_tick = SysTick_GetTick();
    for (i = 0; i < test_iterations; i++)
    {
        volatile uint32_t tick = SysTick_GetTickFast();
        (void)tick;  /* Prevent optimization */
    }
    printf("SysTick_GetTickFast(): %lu ms\r\n", SysTick_GetElapsedMs(start_tick));
    
    /* Test SysTick_GetElapsedMs() performance */
    uint32_t ref_tick = SysTick_GetTick();
    start_tick = SysTick_GetTick();
    for (i = 0; i < test_iterations; i++)
    {
        volatile uint32_t elapsed = SysTick_GetElapsedMs(ref_tick);
        (void)elapsed;  /* Prevent optimization */
    }
    printf("SysTick_GetElapsedMs(): %lu ms\r\n", SysTick_GetElapsedMs(start_tick));
}