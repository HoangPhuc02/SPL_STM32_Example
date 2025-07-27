# STM32F103 ADC Examples

This directory contains various example implementations for ADC operations on the STM32F103 microcontroller. Each file demonstrates a different ADC configuration and usage scenario.

## Available Examples

1. **Group_Oneshot_Scan_HW.c** - Group-based ADC sampling in Scan Mode with Hardware Trigger
   - Uses Timer2 as hardware trigger
   - Samples 3 channels (PA0, PA1, PA2) in sequence
   - Oneshot mode (one full scan of the group, then stop)
   - Interrupt-based result reading

2. **Continuous_DMA.c** - Continuous ADC sampling with DMA Transfer
   - Software trigger to start conversions
   - Continuous conversion mode
   - DMA to transfer values to a buffer without CPU intervention
   - Uses DMA Transfer Complete interrupt

3. **Single_Oneshot_SW.c** - Single Channel ADC with Software Trigger
   - Simple single channel conversion
   - Software trigger
   - Oneshot mode (needs to trigger each conversion)
   - Polling method to check conversion completion
   - LED control based on ADC value

## How to Build and Run Examples

### Using Make Command

To build and flash a specific example, use:

```bash
# For Unix/Linux/WSL:
make example EXAMPLE=example_name

# For Windows (PowerShell):
make example-win EXAMPLE=example_name
```

Where `example_name` is the filename without the `.c` extension. For example:

```bash
make example EXAMPLE=Group_Oneshot_Scan_HW
```

### Using VS Code Tasks

1. Press `Ctrl+Shift+P` to open the command palette
2. Type "Tasks: Run Task" and select it
3. Choose either:
   - "Build Example" to only build the example
   - "Build and Flash Example" to build and flash the example
4. Enter the example name when prompted (without the .c extension)

## Creating Your Own Examples

To create a new example:

1. Create a new .c file in the `examples` directory
2. Include necessary headers (stm32f10x.h, stm32f10x_rcc.h, etc.)
3. Implement GPIO configuration, ADC configuration, and main logic
4. Build and flash using the methods described above

## Hardware Configuration

All examples assume the following hardware setup:

- STM32F103 microcontroller (Blue Pill or similar)
- ADC input channels connected to analog signals
- For examples with LED indicators: PC13 connected to an onboard LED (active low)
