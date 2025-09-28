# SysTick Driver Documentation

## Table of Contents
1. [Overview](#overview)
2. [Features](#features)
3. [Configuration](#configuration)
4. [API Reference](#api-reference)
5. [Usage Examples](#usage-examples)
6. [Integration with UART](#integration-with-uart)
7. [Performance Considerations](#performance-considerations)
8. [Troubleshooting](#troubleshooting)

## Overview

The SysTick driver provides precise timing and delay functions for STM32F103 microcontrollers. It leverages the ARM Cortex-M3 SysTick timer to generate 1ms interrupts for system timing.

### Key Components
- **systick.h**: Header file with function prototypes and configuration
- **systick.c**: Implementation of SysTick driver functions
- **systick_example.c**: Comprehensive usage examples

### Architecture
```
Application Layer
    ↓
SysTick Driver API
    ↓
ARM Cortex-M3 SysTick Timer
    ↓
System Clock (HCLK)
```

## Features

### Core Timing Functions
- ✅ **1ms tick resolution** - Precise millisecond timing
- ✅ **System uptime tracking** - 32-bit counter with overflow handling
- ✅ **Blocking delays** - Simple `delay_ms()` function
- ✅ **Non-blocking timeouts** - For peripheral operations
- ✅ **Elapsed time calculation** - With overflow protection

### Advanced Features
- ✅ **Microsecond delays** - Approximate short delays
- ✅ **Low power delays** - Using WFI (Wait For Interrupt)
- ✅ **Fast inline functions** - For performance-critical code
- ✅ **Statistics tracking** - Optional delay usage statistics
- ✅ **Overflow handling** - Correct wraparound behavior

### UART Integration
- ✅ **Timeout support** - For UART transmit/receive operations
- ✅ **Timing measurements** - For performance monitoring
- ✅ **Backward compatibility** - With existing UART code

## Configuration

### Compile-Time Options

```c
/* SysTick Configuration */
#define SYSTICK_FREQUENCY_HZ        1000U           /* 1ms tick (1000 Hz) */
#define SYSTICK_MAX_DELAY_MS        0x0FFFFFFFU     /* Maximum delay in ms */

/* Timeout values */
#define SYSTICK_UART_TIMEOUT_MS     1000U           /* Default UART timeout */
#define SYSTICK_I2C_TIMEOUT_MS      100U            /* I2C operation timeout */
#define SYSTICK_SPI_TIMEOUT_MS      100U            /* SPI operation timeout */

/* Feature enables */
#define SYSTICK_USE_WFI_DELAY       0               /* Use WFI in delay */
#define SYSTICK_ENABLE_OVERFLOW     1               /* Handle overflow */
#define SYSTICK_ENABLE_STATISTICS   0               /* Enable statistics */
```

### Clock Configuration

SysTick uses the processor clock (HCLK) as the clock source:
- **STM32F103 at 72MHz**: SysTick reload value = 71999 (for 1ms)
- **Other frequencies**: Automatically calculated using `SystemCoreClock`

## API Reference

### Initialization Functions

#### `SysTick_Init(void)`
Initializes SysTick timer for 1ms interrupts.

**Parameters:** None  
**Returns:** None  
**Note:** Must be called before using any other SysTick functions

```c
void SysTick_Init(void);

// Usage
SysTick_Init();
```

#### `SysTick_IsInitialized(void)`
Checks if SysTick is properly initialized and running.

**Parameters:** None  
**Returns:** `true` if initialized, `false` otherwise

```c
bool SysTick_IsInitialized(void);

// Usage
if (SysTick_IsInitialized()) {
    // SysTick is ready
}
```

### Basic Timing Functions

#### `SysTick_DelayMs(uint32_t ms)`
Blocking delay function in milliseconds.

**Parameters:**
- `ms`: Delay time in milliseconds (0 to `SYSTICK_MAX_DELAY_MS`)

**Returns:** None

```c
void SysTick_DelayMs(uint32_t ms);

// Usage
SysTick_DelayMs(1000);  // 1 second delay
SysTick_DelayMs(500);   // 0.5 second delay
```

#### `SysTick_GetTick(void)`
Gets current system uptime in milliseconds.

**Parameters:** None  
**Returns:** Current system time in milliseconds

```c
uint32_t SysTick_GetTick(void);

// Usage
uint32_t current_time = SysTick_GetTick();
printf("Uptime: %lu ms\n", current_time);
```

#### `SysTick_GetElapsedMs(uint32_t start_tick)`
Calculates elapsed time since a reference tick.

**Parameters:**
- `start_tick`: Reference tick value from `SysTick_GetTick()`

**Returns:** Elapsed time in milliseconds

```c
uint32_t SysTick_GetElapsedMs(uint32_t start_tick);

// Usage
uint32_t start = SysTick_GetTick();
// ... some operation ...
uint32_t elapsed = SysTick_GetElapsedMs(start);
printf("Operation took: %lu ms\n", elapsed);
```

### Timeout Functions

#### `SysTick_TimeoutStart(SysTick_TimeoutTypeDef *timeout, uint32_t timeout_ms)`
Initializes a timeout handle for non-blocking operations.

**Parameters:**
- `timeout`: Pointer to timeout handle
- `timeout_ms`: Timeout duration in milliseconds

**Returns:** None

```c
void SysTick_TimeoutStart(SysTick_TimeoutTypeDef *timeout, uint32_t timeout_ms);

// Usage
SysTick_TimeoutTypeDef uart_timeout;
SysTick_TimeoutStart(&uart_timeout, 1000);  // 1 second timeout
```

#### `SysTick_TimeoutExpired(SysTick_TimeoutTypeDef *timeout)`
Checks if timeout has expired.

**Parameters:**
- `timeout`: Pointer to timeout handle

**Returns:** `true` if timeout expired, `false` otherwise

```c
bool SysTick_TimeoutExpired(SysTick_TimeoutTypeDef *timeout);

// Usage
while (!SysTick_TimeoutExpired(&uart_timeout)) {
    // Wait for operation or timeout
    if (operation_complete) break;
}
```

#### `SysTick_TimeoutStop(SysTick_TimeoutTypeDef *timeout)`
Stops/cancels a timeout.

**Parameters:**
- `timeout`: Pointer to timeout handle

**Returns:** None

```c
void SysTick_TimeoutStop(SysTick_TimeoutTypeDef *timeout);

// Usage
SysTick_TimeoutStop(&uart_timeout);
```

### Advanced Functions

#### `SysTick_DelayUs(uint32_t us)`
Approximate microsecond delay using software loops.

**Parameters:**
- `us`: Delay time in microseconds (1-1000)

**Returns:** None  
**Note:** Less accurate than millisecond delays

```c
void SysTick_DelayUs(uint32_t us);

// Usage
SysTick_DelayUs(100);   // ~100 microseconds
SysTick_DelayUs(500);   // ~500 microseconds
```

#### `SysTick_DelayMsLowPower(uint32_t ms)`
Low power delay using WFI (Wait For Interrupt).

**Parameters:**
- `ms`: Delay time in milliseconds

**Returns:** None  
**Note:** CPU sleeps between ticks to save power

```c
void SysTick_DelayMsLowPower(uint32_t ms);

// Usage
SysTick_DelayMsLowPower(2000);  // 2 second power-saving delay
```

### Inline Functions (Performance)

#### `SysTick_GetTickFast(void)`
Fast inline version of `SysTick_GetTick()`.

**Parameters:** None  
**Returns:** Current system tick count

```c
static inline uint32_t SysTick_GetTickFast(void);

// Usage (in performance-critical sections)
uint32_t tick = SysTick_GetTickFast();
```

#### `SysTick_IsTimeoutFast(uint32_t start_tick, uint32_t timeout_ms)`
Fast inline timeout check.

**Parameters:**
- `start_tick`: Reference tick value
- `timeout_ms`: Timeout value in milliseconds

**Returns:** `true` if timeout elapsed, `false` otherwise

```c
static inline bool SysTick_IsTimeoutFast(uint32_t start_tick, uint32_t timeout_ms);

// Usage
uint32_t start = SysTick_GetTickFast();
// ... operation ...
if (SysTick_IsTimeoutFast(start, 100)) {
    // Timeout occurred
}
```

### Backward Compatibility

Legacy functions for existing code:

```c
/* Legacy functions (wrappers) */
void delay_ms(uint32_t ms);              // → SysTick_DelayMs()
uint32_t get_system_ms(void);            // → SysTick_GetTick()

/* HAL compatibility */
uint32_t HAL_GetTick(void);              // → SysTick_GetTick()
void HAL_Delay(uint32_t ms);             // → SysTick_DelayMs()
```

## Usage Examples

### Basic Usage

```c
#include "systick.h"

int main(void) {
    // Initialize system
    SystemInit();
    
    // Initialize SysTick
    SysTick_Init();
    
    while (1) {
        // Blink LED every 1 second
        LED_Toggle();
        SysTick_DelayMs(1000);
    }
}
```

### Timeout Handling

```c
void uart_transmit_with_timeout(void) {
    SysTick_TimeoutTypeDef timeout;
    bool success = false;
    
    // Start 1 second timeout
    SysTick_TimeoutStart(&timeout, 1000);
    
    while (!SysTick_TimeoutExpired(&timeout)) {
        if (UART_TransmitReady()) {
            UART_SendByte(0x55);
            success = true;
            break;
        }
        SysTick_DelayMs(1);  // Small delay
    }
    
    SysTick_TimeoutStop(&timeout);
    
    if (!success) {
        printf("UART transmit timeout\n");
    }
}
```

### Performance Measurement

```c
void measure_function_performance(void) {
    uint32_t start_time = SysTick_GetTick();
    
    // Function to measure
    complex_calculation();
    
    uint32_t elapsed = SysTick_GetElapsedMs(start_time);
    printf("Function took: %lu ms\n", elapsed);
}
```

### Overflow Handling

```c
void demonstrate_overflow_handling(void) {
    uint32_t start_tick = SysTick_GetTick();
    
    // Even if counter overflows, elapsed time is calculated correctly
    while (SysTick_GetElapsedMs(start_tick) < 5000) {
        // Wait for 5 seconds (handles overflow correctly)
        do_work();
    }
}
```

## Integration with UART

The SysTick driver is seamlessly integrated with the UART driver:

### UART Driver Integration Points

1. **Timeout Support**: UART operations use SysTick timeouts
2. **Timing Functions**: UART driver includes `systick.h`
3. **Backward Compatibility**: Existing `delay_ms()` and `get_system_ms()` calls work

### UART Configuration with SysTick

```c
// UART driver automatically uses SysTick for timeouts
UART_ConfigTypeDef config = {
    .UartNumber = UART1,
    .BaudRate = 115200,
    .TimeoutMs = 1000,      // Uses SysTick timeout
    // ... other config
};

UART_Init(&config);
```

### UART Timing Examples

```c
// Transmit with timing measurement
uint32_t start = SysTick_GetTick();
UART_StatusTypeDef result = UART_Transmit(UART1, data, len, 1000);
uint32_t tx_time = SysTick_GetElapsedMs(start);

printf("UART TX took: %lu ms, Status: %d\n", tx_time, result);

// Receive with timeout
uint8_t rx_buffer[64];
uint32_t bytes_received;
result = UART_Receive(UART1, rx_buffer, sizeof(rx_buffer), &bytes_received, 2000);

if (result == UART_TIMEOUT) {
    printf("UART receive timeout\n");
}
```

## Performance Considerations

### Timing Accuracy

| Function | Accuracy | Overhead | Use Case |
|----------|----------|----------|----------|
| `SysTick_DelayMs()` | ±1ms | Medium | General delays |
| `SysTick_DelayUs()` | ±10% | Low | Short delays |
| `SysTick_GetTick()` | 1ms resolution | Low | Timing measurements |
| `SysTick_GetTickFast()` | 1ms resolution | Minimal | Performance-critical |

### Memory Usage

- **RAM**: ~16 bytes (system_ticks + flags)
- **Flash**: ~2KB (driver code)
- **Stack**: <32 bytes per function call

### Interrupt Overhead

- **SysTick ISR**: ~10 CPU cycles (very fast)
- **Frequency**: 1000 Hz (1ms intervals)
- **CPU Load**: <0.1% at 72MHz

### Optimization Tips

1. **Use inline functions** for performance-critical code
2. **Enable WFI delays** for power savings (if real-time not critical)
3. **Disable statistics** if not needed (saves RAM and cycles)
4. **Use timeout objects** instead of polling for non-blocking operations

## Troubleshooting

### Common Issues

#### SysTick not working
**Symptoms**: Delays don't work, timeouts fail
**Solutions**:
- Call `SysTick_Init()` after `SystemInit()`
- Check system clock configuration
- Verify `SystemCoreClock` variable is correct

#### Inaccurate delays
**Symptoms**: Delays are too short/long
**Solutions**:
- Ensure correct system clock frequency
- Check for interrupt conflicts
- Verify `SystemCoreClock` matches actual frequency

#### Timeout not working
**Symptoms**: Timeouts never expire or expire immediately
**Solutions**:
- Initialize timeout properly with `SysTick_TimeoutStart()`
- Check if timeout handle is valid
- Ensure SysTick is running

#### High CPU usage
**Symptoms**: System becomes unresponsive
**Solutions**:
- Don't use very short delays in tight loops
- Use `SysTick_DelayMsLowPower()` for long delays
- Consider using timeout objects instead of polling

### Debug Functions

```c
// Check SysTick status
if (!SysTick_IsInitialized()) {
    printf("ERROR: SysTick not initialized\n");
}

// Monitor system uptime
printf("System uptime: %lu ms\n", SysTick_GetTick());

// Test delay accuracy
uint32_t start = SysTick_GetTick();
SysTick_DelayMs(1000);
uint32_t actual = SysTick_GetElapsedMs(start);
printf("Delay accuracy: %ld ms error\n", (int32_t)actual - 1000);
```

### Integration Checklist

- [ ] `SysTick_Init()` called after system initialization
- [ ] UART driver includes `systick.h`
- [ ] Timeout values are reasonable (not too small/large)
- [ ] System clock is properly configured
- [ ] No conflicts with other timer interrupts
- [ ] Memory allocation for timeout handles

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-09-25 | Initial release with full feature set |
| | | - 1ms tick resolution |
| | | - Timeout support |
| | | - UART integration |
| | | - Performance optimizations |
| | | - Comprehensive documentation |

## License

This SysTick driver is part of the STM32F103 CAN UART project.  
Author: hoangphuc540202@gmail.com  
Date: 25/9/2025