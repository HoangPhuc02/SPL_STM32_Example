# STM32F103 PWM Examples using SPL Library

This directory contains comprehensive PWM examples for STM32F103 microcontrollers using the Standard Peripheral Library (SPL). Each example is thoroughly commented and explains both theory and practical implementation.

## 📁 Examples Overview

### 1. **Basic_PWM_LED_Control.c**
**Purpose**: Introduction to PWM generation for LED brightness control
**Hardware**: LED + resistor connected to PA6
**Features**:
- Basic TIM3 PWM configuration
- LED fade in/out effect
- Detailed PWM theory and calculations
- Step-by-step hardware setup guide

**Key Learning Points**:
- Timer configuration for PWM
- GPIO alternate function setup
- Duty cycle calculations
- PWM frequency selection

### 2. **Multi_Channel_PWM.c** 
**Purpose**: Demonstrates multi-channel PWM using all 4 channels of TIM3
**Hardware**: 4 LEDs connected to PA6, PA7, PB0, PB1
**Features**:
- Simultaneous control of 4 PWM channels
- Different patterns per channel (breathing, blinking, constant)
- Independent duty cycle control
- Synchronized PWM generation

**Key Learning Points**:
- Multi-channel timer configuration
- Channel independence
- Pattern generation
- Resource sharing concepts

### 3. **Servo_Motor_Control.c**
**Purpose**: PWM generation for servo motor position control
**Hardware**: Standard hobby servo connected to PA0
**Features**:
- 50Hz PWM for servo control
- Angle to pulse width conversion
- Servo sweep demonstration
- Precise timing requirements

**Key Learning Points**:
- Servo PWM specifications (1ms-2ms pulse width)
- High-precision timing (1µs resolution)
- Position control algorithms
- Real-world timing constraints

### 4. **Motor_Speed_Control.c**
**Purpose**: DC motor speed and direction control using H-bridge
**Hardware**: DC motor with L298N H-bridge driver
**Features**:
- Variable speed control (0-100%)
- Direction control (forward/reverse/stop)
- Smooth acceleration/deceleration
- H-bridge interfacing

**Key Learning Points**:
- H-bridge operation principles
- Motor control theory
- PWM frequency selection for motors
- Safety considerations

## 🔧 Hardware Requirements

### Common Components:
- STM32F103C8T6 development board (Blue Pill)
- Breadboard and jumper wires
- External power supply (5V/12V for motors)
- Oscilloscope (recommended for PWM verification)

### Per Example:
1. **Basic LED Control**: LED + 220Ω resistor
2. **Multi-Channel**: 4x LEDs + 4x 220Ω resistors  
3. **Servo Control**: Standard hobby servo (SG90 or similar)
4. **Motor Control**: DC motor + L298N H-bridge module

## ⚙️ Timer and Pin Assignments

| Example | Timer | Channels | GPIO Pins | Frequency |
|---------|-------|----------|-----------|-----------|
| Basic LED | TIM3 | CH1 | PA6 | 1kHz |
| Multi-Channel | TIM3 | CH1-CH4 | PA6,PA7,PB0,PB1 | 2kHz |
| Servo Control | TIM2 | CH1 | PA0 | 50Hz |
| Motor Control | TIM3 | CH1 | PA6 + PA1,PA2 | 1kHz |

## 📚 PWM Theory Covered

### Fundamental Concepts:
- **Frequency**: How fast PWM switches (Hz)
- **Duty Cycle**: Percentage of time signal is HIGH
- **Resolution**: Number of discrete duty cycle steps
- **Period**: Time for one complete PWM cycle

### Timer Calculations:
```
Timer_Frequency = APB_Clock / (Prescaler + 1)
PWM_Frequency = Timer_Frequency / (Period + 1)  
Duty_Cycle_% = (Compare_Value / Period) × 100%
```

### Applications Explained:
- **LED Dimming**: Linear brightness control
- **Motor Speed**: Average voltage control
- **Servo Position**: Precise timing control
- **Audio**: Tone generation

## 🚀 Getting Started

### 1. **Hardware Setup**
```
1. Connect STM32F103C8T6 to development environment
2. Wire components according to example schematics
3. Ensure proper power supply connections
4. Verify ground connections between all components
```

### 2. **Software Setup**
```
1. Install STM32 SPL library
2. Configure toolchain (GCC, Keil, or IAR)
3. Set up project with SPL includes
4. Flash example code to microcontroller
```

### 3. **Testing**
```
1. Compile and flash example code
2. Observe PWM output with oscilloscope
3. Verify expected behavior (LED dimming, servo movement, etc.)
4. Experiment with parameter changes
```

## 🔍 Debugging Tips

### PWM Signal Verification:
- **Oscilloscope**: Best tool for PWM analysis
- **LED Indicator**: Visual feedback for duty cycle changes
- **Multimeter**: Average voltage measurement
- **Logic Analyzer**: Digital signal timing

### Common Issues:
1. **No PWM Output**:
   - Check GPIO alternate function configuration
   - Verify timer clock enable
   - Confirm timer start command

2. **Wrong Frequency**:
   - Recalculate prescaler and period values
   - Check APB1 clock frequency
   - Verify timer base configuration

3. **Incorrect Duty Cycle**:
   - Check compare value calculations
   - Verify PWM mode settings
   - Ensure compare value ≤ period

### Test Points:
- **PA6**: Basic LED PWM output
- **PA0**: Servo control signal  
- **PA1,PA2**: Motor direction signals
- **Power Rails**: Verify voltage levels

## 📖 Advanced Topics

### PWM Optimization:
- **DMA Integration**: Automatic duty cycle updates
- **Interrupt Handling**: Event-driven PWM control
- **Dead Time**: Prevention of shoot-through in H-bridges
- **Complementary Outputs**: Advanced motor control

### Real-World Applications:
- **BLDC Motor Control**: Three-phase PWM generation
- **Power Converters**: Buck/boost converter control
- **Audio Amplifiers**: Class-D amplifier design
- **LED Drivers**: Constant current control

### Performance Considerations:
- **CPU Overhead**: Timer vs DMA vs interrupt methods
- **Power Efficiency**: PWM frequency vs switching losses
- **EMI Reduction**: Spread spectrum and dithering
- **Thermal Management**: Heat dissipation in power applications

## 🔧 Customization Guide

### Frequency Modification:
```c
// Change these values to modify PWM frequency
#define PRESCALER    72    // Adjust for different timer clock
#define PERIOD       1000  // Adjust for different PWM frequency

// New frequency = 72MHz / PRESCALER / PERIOD
```

### Duty Cycle Control:
```c
// Convert percentage to compare value
uint16_t duty_compare = (percentage * PERIOD) / 100;
TIM_SetCompare1(TIM3, duty_compare);
```

### Pin Reassignment:
```c
// To use different GPIO pins:
// 1. Check timer channel to pin mapping in datasheet
// 2. Update GPIO_InitStructure.GPIO_Pin
// 3. Modify GPIO port if necessary
// 4. Update timer channel initialization
```

## ⚠️ Safety Notes

### Electrical Safety:
- **Isolation**: Separate microcontroller and motor power supplies
- **Protection**: Use fuses and current limiting
- **Grounding**: Ensure proper common ground connections
- **Voltage Levels**: Verify logic level compatibility

### Motor Control Safety:
- **Emergency Stop**: Implement software and hardware emergency stops
- **Gradual Changes**: Avoid sudden speed/direction changes
- **Overcurrent Protection**: Monitor motor current
- **Thermal Protection**: Prevent overheating

### Development Safety:
- **Code Review**: Verify PWM parameters before testing
- **Incremental Testing**: Start with low power/speed
- **Backup Plans**: Have emergency stop procedures
- **Documentation**: Record all modifications and tests

## 📞 Support and Resources

### Documentation:
- **STM32F103 Reference Manual**: Complete register descriptions
- **SPL Documentation**: Library function references  
- **Application Notes**: STM32 PWM application examples
- **Datasheet**: Electrical characteristics and pin descriptions

### Online Resources:
- **STM32 Community**: Forums and discussion groups
- **GitHub Repositories**: Open source PWM projects
- **YouTube Tutorials**: Visual learning resources
- **Application Notes**: Official STM technical documents

### Tools:
- **STM32CubeMX**: Graphical configuration tool
- **STM32CubeIDE**: Integrated development environment
- **STLink Utility**: Programming and debugging tool
- **Oscilloscope Software**: Signal analysis tools

---

## 📝 Example Usage

### Quick Start - Basic LED Control:
```c
#include "stm32f10x.h"

// Initialize PWM for LED on PA6
PWM_GPIO_Config();    // Configure PA6 as TIM3_CH1
PWM_Timer_Config();   // Setup TIM3 for 1kHz PWM

// Set 50% brightness
PWM_SetDutyCycle(500); // 500/1000 = 50%

// Fade effect
for(int duty = 0; duty <= 1000; duty += 10) {
    PWM_SetDutyCycle(duty);
    Delay(10000);
}
```

This collection provides a complete foundation for understanding and implementing PWM control in STM32F103 projects using the SPL library.
