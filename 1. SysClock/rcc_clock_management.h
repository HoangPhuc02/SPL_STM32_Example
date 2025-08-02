/**
 * @file    rcc_clock_management.h
 * @brief   Comprehensive RCC Clock Management Header for STM32F103C8T6
 * @details This header file contains function prototypes and definitions for
 *          comprehensive clock management using STM32 Standard Peripheral Library.
 *          It demonstrates HSE+PLL configuration, peripheral clock control,
 *          clock monitoring, and power management features.
 * 
 * @author  STM32 Development Team
 * @date    2025-01-08
 * @version 1.0.0
 * 
 * @note    Target: STM32F103C8T6 (72MHz max, 64KB Flash, 20KB RAM)
 *          External Crystal: 8MHz HSE
 *          System Clock: 72MHz (HSE + PLL)
 * 
 * Hardware Requirements:
 * - 8MHz external crystal oscillator
 * - Proper crystal loading capacitors (typically 18-22pF)
 * - Decoupling capacitors for power supply
 */

#ifndef RCC_CLOCK_MANAGEMENT_H
#define RCC_CLOCK_MANAGEMENT_H

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/**
 * @brief   Clock Configuration Constants
 */
#define HSE_FREQUENCY_HZ                    (8000000UL)     /* 8MHz external crystal */
#define SYSTEM_CLOCK_TARGET_HZ              (72000000UL)    /* Target 72MHz system clock */
#define PLL_MULTIPLIER                      (RCC_PLLMul_9)  /* 8MHz * 9 = 72MHz */
#define FLASH_LATENCY_CYCLES                (FLASH_Latency_2) /* 2 wait states for 72MHz */

/**
 * @brief   Timeout Values for Clock Stabilization
 */
#define HSE_STARTUP_TIMEOUT                 (0x5000U)       /* HSE startup timeout */
#define PLL_STARTUP_TIMEOUT                 (0x5000U)       /* PLL startup timeout */
#define CLOCK_SWITCH_TIMEOUT                (0x5000U)       /* Clock switch timeout */

/**
 * @brief   Clock Monitoring Intervals
 */
#define CLOCK_MONITOR_INTERVAL_MS           (1000U)         /* 1 second monitoring interval */

/**
 * @brief   Power Management Modes
 */
typedef enum {
    POWER_MODE_FULL_PERFORMANCE = 0,    /* All peripherals enabled */
    POWER_MODE_NORMAL,                  /* Essential peripherals only */
    POWER_MODE_LOW_POWER,               /* Minimal peripherals */
    POWER_MODE_SLEEP_READY              /* Ready for sleep mode */
} PowerMode_t;

/**
 * @brief   Clock Configuration Status
 */
typedef enum {
    CLOCK_CONFIG_SUCCESS = 0,           /* Configuration successful */
    CLOCK_CONFIG_HSE_FAIL,              /* HSE startup failed */
    CLOCK_CONFIG_PLL_FAIL,              /* PLL startup failed */
    CLOCK_CONFIG_SWITCH_FAIL,           /* Clock switch failed */
    CLOCK_CONFIG_TIMEOUT                /* Configuration timeout */
} ClockConfigStatus_t;

/**
 * @brief   System Clock Information Structure
 */
typedef struct {
    uint32_t SystemClock_Hz;            /* System clock frequency */
    uint32_t HCLK_Hz;                   /* AHB clock frequency */
    uint32_t PCLK1_Hz;                  /* APB1 clock frequency */
    uint32_t PCLK2_Hz;                  /* APB2 clock frequency */
    uint32_t ADCCLK_Hz;                 /* ADC clock frequency */
    uint32_t TIMCLK1_Hz;                /* Timer clock 1 frequency */
    uint32_t TIMCLK2_Hz;                /* Timer clock 2 frequency */
    RCC_ClocksTypeDef RCC_Clocks;       /* STM32 RCC clock structure */
} SystemClockInfo_t;

/**
 * @brief   Peripheral Clock Control Structure
 */
typedef struct {
    bool GPIO_A_Enabled;                /* GPIOA clock status */
    bool GPIO_B_Enabled;                /* GPIOB clock status */
    bool GPIO_C_Enabled;                /* GPIOC clock status */
    bool USART1_Enabled;                /* USART1 clock status */
    bool SPI1_Enabled;                  /* SPI1 clock status */
    bool I2C1_Enabled;                  /* I2C1 clock status */
    bool TIM2_Enabled;                  /* TIM2 clock status */
    bool TIM3_Enabled;                  /* TIM3 clock status */
    bool ADC1_Enabled;                  /* ADC1 clock status */
    bool AFIO_Enabled;                  /* AFIO clock status */
} PeripheralClockStatus_t;

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief   Initialize system clock to 72MHz using HSE + PLL
 * @details Configures the system clock using external 8MHz crystal with PLL
 *          to achieve maximum performance of 72MHz. Includes proper flash
 *          wait states and bus clock prescalers.
 * 
 * @return  ClockConfigStatus_t - Configuration status
 * 
 * @note    This function should be called early in system initialization
 */
ClockConfigStatus_t RCC_SystemClockConfig(void);

/**
 * @brief   Configure peripheral clocks for specific power mode
 * @details Enables or disables peripheral clocks based on the selected
 *          power management mode to optimize power consumption.
 * 
 * @param   mode - Power management mode
 * 
 * @return  None
 */
void RCC_PeripheralClockConfig(PowerMode_t mode);

/**
 * @brief   Enable specific peripheral clocks
 * @details Provides granular control over individual peripheral clocks
 *          for fine-tuned power management.
 * 
 * @param   None
 * 
 * @return  None
 */
void RCC_EnablePeripheralClocks(void);

/**
 * @brief   Disable specific peripheral clocks
 * @details Disables peripheral clocks to save power when peripherals
 *          are not in use.
 * 
 * @param   None
 * 
 * @return  None
 */
void RCC_DisablePeripheralClocks(void);

/**
 * @brief   Get current system clock information
 * @details Reads and calculates all system clock frequencies and
 *          populates the SystemClockInfo structure.
 * 
 * @param   clockInfo - Pointer to SystemClockInfo structure
 * 
 * @return  None
 */
void RCC_GetSystemClockInfo(SystemClockInfo_t* clockInfo);

/**
 * @brief   Get peripheral clock status
 * @details Reads the current status of all peripheral clocks and
 *          populates the PeripheralClockStatus structure.
 * 
 * @param   status - Pointer to PeripheralClockStatus structure
 * 
 * @return  None
 */
void RCC_GetPeripheralClockStatus(PeripheralClockStatus_t* status);

/**
 * @brief   Configure ADC clock prescaler
 * @details Sets the ADC clock prescaler to achieve optimal ADC
 *          performance while staying within specifications.
 * 
 * @param   prescaler - ADC prescaler value (RCC_PCLK2_Div2, RCC_PCLK2_Div4, etc.)
 * 
 * @return  None
 */
void RCC_ConfigureADCClock(uint32_t prescaler);

/**
 * @brief   Monitor system clocks
 * @details Continuously monitors system clock frequencies and detects
 *          any clock failures or deviations from expected values.
 * 
 * @param   None
 * 
 * @return  bool - true if all clocks are stable, false if issues detected
 */
bool RCC_MonitorSystemClocks(void);

/**
 * @brief   Display clock information via UART
 * @details Formats and transmits current clock information through
 *          UART for debugging and monitoring purposes.
 * 
 * @param   clockInfo - Pointer to SystemClockInfo structure
 * 
 * @return  None
 * 
 * @note    Requires UART to be initialized
 */
void RCC_DisplayClockInfo(const SystemClockInfo_t* clockInfo);

/**
 * @brief   Handle clock failure
 * @details Implements clock failure recovery mechanisms including
 *          fallback to HSI and error reporting.
 * 
 * @param   None
 * 
 * @return  ClockConfigStatus_t - Recovery status
 */
ClockConfigStatus_t RCC_HandleClockFailure(void);

/**
 * @brief   Reset to default clock configuration
 * @details Resets the clock system to default HSI configuration
 *          for recovery purposes.
 * 
 * @param   None
 * 
 * @return  None
 */
void RCC_ResetToDefaultClock(void);

/**
 * @brief   Enable clock security system
 * @details Enables the Clock Security System (CSS) to detect HSE
 *          failures and automatically switch to HSI.
 * 
 * @param   None
 * 
 * @return  None
 */
void RCC_EnableClockSecurity(void);

/**
 * @brief   Calculate timer clock frequencies
 * @details Calculates the actual timer clock frequencies based on
 *          current APB prescaler settings.
 * 
 * @param   clockInfo - Pointer to SystemClockInfo structure
 * 
 * @return  None
 */
void RCC_CalculateTimerClocks(SystemClockInfo_t* clockInfo);

/*==================================================================================================
*                                       GLOBAL VARIABLES
==================================================================================================*/

/* External variables for clock monitoring */
extern SystemClockInfo_t g_SystemClockInfo;
extern PeripheralClockStatus_t g_PeripheralClockStatus;
extern volatile bool g_ClockMonitoringEnabled;

#endif /* RCC_CLOCK_MANAGEMENT_H */

/**
 * @}
 */
