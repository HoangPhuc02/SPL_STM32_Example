# STM32F103C8T6 RCC Comprehensive Example

## Overview

This comprehensive example demonstrates advanced RCC (Reset and Clock Control) management for the STM32F103C8T6 microcontroller using the Standard Peripheral Library (SPL). It showcases professional-grade clock configuration, monitoring, and power management techniques commonly used in production embedded systems.

## 🎯 **Key Features Demonstrated**

### ✅ **High-Performance Clock Configuration**
- **HSE + PLL Setup**: Configures 8MHz external crystal with PLL to achieve 72MHz system clock
- **Optimal Bus Clocks**: AHB=72MHz, APB1=36MHz, APB2=72MHz for maximum performance
- **Flash Optimization**: Proper flash latency and prefetch buffer configuration
- **ADC Clock**: Optimized 12MHz ADC clock for accurate conversions

### ✅ **Comprehensive Peripheral Clock Management**
- **Granular Control**: Individual peripheral clock enable/disable
- **Power Mode Management**: Four distinct power modes for different scenarios
- **Dynamic Switching**: Runtime power mode transitions
- **Clock Gating**: Selective peripheral clock control for power savings

### ✅ **Advanced Clock Monitoring**
- **Real-time Monitoring**: Continuous clock frequency verification
- **Failure Detection**: HSE failure detection with Clock Security System (CSS)
- **Error Counting**: Statistical tracking of clock-related errors
- **Recovery Mechanisms**: Automatic fallback to HSI and recovery procedures

### ✅ **Professional Error Handling**
- **Timeout Protection**: All clock operations include timeout mechanisms
- **Graceful Degradation**: Fallback to HSI if HSE fails
- **Status Reporting**: Comprehensive error reporting and diagnostics
- **Recovery Procedures**: Automatic and manual recovery options

## 📁 **Project Structure**

```
RCC_Comprehensive_Example/
├── rcc_clock_management.h      # Header with all function prototypes
├── rcc_clock_management.c      # Complete RCC implementation
├── main.c                      # Demonstration application
├── README.md                   # This documentation
└── Makefile                    # Build configuration (optional)
```

## 🔧 **Hardware Requirements**

### **Minimum Setup**
- **STM32F103C8T6** microcontroller (Blue Pill board recommended)
- **8MHz Crystal Oscillator** with proper loading capacitors (18-22pF)
- **Power Supply**: 3.3V regulated supply with decoupling capacitors

### **Full Feature Setup**
- **UART Connection**: PA9 (TX) and PA10 (RX) for debug output
- **Status LED**: PC13 (built-in LED on Blue Pill)
- **User Button**: PA0 with external pull-up resistor
- **USB-to-Serial Converter**: For viewing debug output (115200 baud)

### **Crystal Oscillator Circuit**
```
    STM32F103C8T6
         |
    OSC_IN |---- [18pF] ---- 8MHz Crystal ---- [18pF] ---- OSC_OUT
         |                                                    |
        GND                                                  GND
```

## 🚀 **Getting Started**

### **1. Hardware Setup**
```c
// Pin Connections:
// PC13 -> LED (built-in on Blue Pill)
// PA0  -> Button (with pull-up resistor)
// PA9  -> UART TX (connect to USB-Serial RX)
// PA10 -> UART RX (connect to USB-Serial TX)
// OSC_IN/OSC_OUT -> 8MHz crystal with loading capacitors
```

### **2. Software Setup**
```c
#include "rcc_clock_management.h"

int main(void)
{
    // Configure system clock to 72MHz
    ClockConfigStatus_t status = RCC_SystemClockConfig();
    
    if (status == CLOCK_CONFIG_SUCCESS)
    {
        // Clock configuration successful
        // Initialize your application
    }
    else
    {
        // Handle clock configuration failure
        RCC_HandleClockFailure();
    }
    
    return 0;
}
```

### **3. Compile and Flash**
- Use your preferred STM32 development environment
- Include STM32F10x Standard Peripheral Library
- Flash the compiled binary to STM32F103C8T6
- Connect UART to view debug output

## 📊 **Clock Configuration Details**

### **System Clock Tree**
```
HSE (8MHz) -> PLL (x9) -> SYSCLK (72MHz)
                            |
                            ├── AHB Prescaler (/1) -> HCLK (72MHz)
                            |     |
                            |     ├── APB1 Prescaler (/2) -> PCLK1 (36MHz)
                            |     |     └── Timer Multiplier (x2) -> TIM2,3,4 (72MHz)
                            |     |
                            |     └── APB2 Prescaler (/1) -> PCLK2 (72MHz)
                            |           ├── Timer Clock -> TIM1 (72MHz)
                            |           └── ADC Prescaler (/6) -> ADCCLK (12MHz)
                            |
                            └── USB Prescaler (/1.5) -> USBCLK (48MHz)
```

### **Power Modes**

#### **FULL_PERFORMANCE Mode**
```c
// All peripherals enabled
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                      RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO |
                      RCC_APB2Periph_USART1 | RCC_APB2Periph_SPI1 |
                      RCC_APB2Periph_ADC1, ENABLE);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2 |
                      RCC_APB1Periph_TIM3, ENABLE);
```

#### **NORMAL Mode**
```c
// Essential peripherals only
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                      RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
```

#### **LOW_POWER Mode**
```c
// Minimal peripherals
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
```

#### **SLEEP_READY Mode**
```c
// Only wakeup-capable peripherals
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
```

## 🔍 **Key Functions Explained**

### **Clock Configuration**
```c
ClockConfigStatus_t RCC_SystemClockConfig(void)
{
    // 1. Enable HSE and wait for stabilization
    RCC_HSEConfig(RCC_HSE_ON);
    if (!RCC_WaitForHSEReady()) return CLOCK_CONFIG_HSE_FAIL;
    
    // 2. Configure Flash latency for 72MHz
    FLASH_SetLatency(FLASH_Latency_2);
    
    // 3. Configure bus prescalers
    RCC_HCLKConfig(RCC_SYSCLK_Div1);    // AHB = 72MHz
    RCC_PCLK1Config(RCC_HCLK_Div2);     // APB1 = 36MHz
    RCC_PCLK2Config(RCC_HCLK_Div1);     // APB2 = 72MHz
    
    // 4. Configure and enable PLL
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    if (!RCC_WaitForPLLReady()) return CLOCK_CONFIG_PLL_FAIL;
    
    // 5. Switch system clock to PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    // 6. Enable Clock Security System
    RCC_ClockSecuritySystemCmd(ENABLE);
    
    return CLOCK_CONFIG_SUCCESS;
}
```

### **Clock Monitoring**
```c
bool RCC_MonitorSystemClocks(void)
{
    SystemClockInfo_t currentClocks;
    RCC_GetSystemClockInfo(&currentClocks);
    
    // Check system clock frequency (within 1% tolerance)
    if ((currentClocks.SystemClock_Hz < (72000000 * 99 / 100)) ||
        (currentClocks.SystemClock_Hz > (72000000 * 101 / 100)))
    {
        return false;  // Clock frequency out of range
    }
    
    // Check HSE and PLL status
    if ((RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) ||
        (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET))
    {
        return false;  // Clock source not ready
    }
    
    return true;  // All clocks stable
}
```

### **Power Management**
```c
void RCC_PeripheralClockConfig(PowerMode_t mode)
{
    // Disable all peripheral clocks first
    RCC_DisablePeripheralClocks();
    
    switch (mode)
    {
        case POWER_MODE_FULL_PERFORMANCE:
            // Enable all peripherals for maximum functionality
            break;
            
        case POWER_MODE_NORMAL:
            // Enable essential peripherals only
            break;
            
        case POWER_MODE_LOW_POWER:
            // Enable minimal peripherals
            break;
            
        case POWER_MODE_SLEEP_READY:
            // Prepare for sleep mode
            break;
    }
}
```

## 📈 **Performance Characteristics**

### **Clock Frequencies Achieved**
- **System Clock**: 72MHz (maximum for STM32F103C8T6)
- **AHB Bus**: 72MHz (full speed)
- **APB1 Bus**: 36MHz (maximum allowed)
- **APB2 Bus**: 72MHz (full speed)
- **ADC Clock**: 12MHz (optimal for 12-bit conversion)
- **Timer Clocks**: 72MHz (maximum resolution)

### **Power Consumption Estimates**
- **Full Performance**: ~25mA @ 72MHz
- **Normal Mode**: ~20mA @ 72MHz
- **Low Power**: ~15mA @ 72MHz
- **Sleep Ready**: ~10mA @ 72MHz

### **Clock Stability**
- **HSE Accuracy**: ±20ppm (with quality crystal)
- **Temperature Stability**: ±50ppm over -40°C to +85°C
- **Aging**: <±5ppm per year

## 🛠 **Troubleshooting**

### **Common Issues**

#### **HSE Won't Start**
```c
// Check crystal connections and loading capacitors
// Verify crystal frequency (must be 8MHz for this example)
// Check power supply stability
// Ensure proper PCB layout for crystal circuit
```

#### **Clock Frequency Wrong**
```c
// Verify PLL multiplier settings
// Check bus prescaler configuration
// Ensure SystemCoreClockUpdate() is called after changes
```

#### **Peripheral Not Working**
```c
// Check if peripheral clock is enabled
RCC_GetPeripheralClockStatus(&status);
// Verify correct power mode is selected
// Check GPIO alternate function configuration
```

### **Debug Output Examples**

#### **Successful Startup**
```
=====================================
STM32F103C8T6 RCC Management Example
=====================================
Clock Configuration Status: SUCCESS

=== STM32F103C8T6 Clock Information ===
System Clock (SYSCLK): 72000000 Hz (72.00 MHz)
AHB Clock (HCLK):      72000000 Hz (72.00 MHz)
APB1 Clock (PCLK1):    36000000 Hz (36.00 MHz)
APB2 Clock (PCLK2):    72000000 Hz (72.00 MHz)
ADC Clock (ADCCLK):    12000000 Hz (12.00 MHz)
Timer Clock 1:         72000000 Hz (72.00 MHz)
Timer Clock 2:         72000000 Hz (72.00 MHz)
Clock Source: PLL
HSE Status: Ready
PLL Status: Ready
CSS Status: Normal
========================================
```

#### **HSE Failure Recovery**
```
WARNING: Clock instability detected!
Error count: 1
HSE failure detected - attempting recovery...
Switching to HSI as emergency clock source...
Attempting to reconfigure HSE + PLL...
Recovery status: SUCCESS
```

## 📚 **Educational Value**

This example teaches:

1. **Professional Clock Management**: Industry-standard practices for clock configuration
2. **Error Handling**: Robust error detection and recovery mechanisms
3. **Power Optimization**: Practical power management techniques
4. **Real-time Monitoring**: Continuous system health monitoring
5. **Embedded Best Practices**: Timeout handling, status checking, graceful degradation

## 🔗 **Integration with Other Systems**

This RCC management system can be easily integrated with:
- **RTOS**: FreeRTOS, RT-Thread, or other real-time operating systems
- **Communication Stacks**: USB, CAN, Ethernet protocols
- **Sensor Systems**: ADC-based measurement systems
- **Motor Control**: PWM and timer-based control systems
- **IoT Applications**: Low-power wireless communication systems

## 📄 **License**

This example is provided for educational and development purposes. Feel free to use and modify for your projects.

---

**Happy Clock Management! ⏰💻**
