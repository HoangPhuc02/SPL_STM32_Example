# LIN Communication Examples for STM32F103C8T6

This directory contains comprehensive examples for LIN (Local Interconnect Network) communication using the STM32F103C8T6 microcontroller and Standard Peripheral Library (SPL).

## Overview

LIN is a serial communication protocol used primarily in automotive applications for connecting sensors, actuators, and control units. These examples demonstrate various aspects of LIN communication including basic transmission/reception, interrupt handling, master/slave configurations, diagnostics, and bridging.

## Hardware Requirements

- STM32F103C8T6 (Blue Pill board)
- LIN transceiver (e.g., TJA1020, MCP2003B)
- LEDs for status indication
- Optional: Potentiometer for analog input (slave example)
- Optional: UART-to-USB converter for bridge example

## Pin Configuration

### LIN Interface (USART1)
- **PA9**: LIN TX (connect to LIN transceiver TX)
- **PA10**: LIN RX (connect to LIN transceiver RX)

### Status LEDs
- **PC13**: General status/activity LED
- **PC14**: Error indicator LED (some examples)
- **PC15**: Additional status LED (some examples)

### Bridge Example Additional Pins (USART2)
- **PA2**: UART TX to PC
- **PA3**: UART RX from PC

### Slave Example Additional Pins
- **PA0**: ADC input for sensor simulation
- **PB0, PB1**: Digital inputs
- **PC13, PC14, PC15**: Digital outputs (LEDs)

## Examples Description

### 1. basic_transmit.c
**Purpose**: Simple LIN frame transmission
**Features**:
- Basic LIN frame structure
- PID calculation with parity bits
- Checksum calculation
- Periodic frame transmission

### 2. basic_receiver.c
**Purpose**: Basic LIN frame reception
**Features**:
- Frame reception with validation
- Break detection
- Sync byte verification
- Checksum validation

### 3. lin_interrupt_example.c
**Purpose**: Advanced interrupt-based LIN communication
**Features**:
- Interrupt-driven frame handling
- State machine for frame processing
- Timeout handling
- Frame validation and error detection
- System tick timer integration

### 4. lin_receive_example.c
**Purpose**: Dedicated LIN receiver with frame filtering
**Features**:
- Frame buffering system
- ID-based frame filtering
- Multiple LED status indicators
- Frame processing based on ID
- Statistics tracking

### 5. lin_master_example.c
**Purpose**: LIN Master node implementation
**Features**:
- Schedule-based frame transmission
- Slave node management
- Master request/slave response handling
- Network monitoring
- Diagnostic capabilities

### 6. lin_slave_example.c
**Purpose**: LIN Slave node implementation
**Features**:
- Automatic response to master requests
- Sensor data simulation (ADC)
- Digital I/O control
- Status reporting
- Diagnostic support

### 7. lin_diagnostic_example.c
**Purpose**: LIN network diagnostic and monitoring
**Features**:
- Network scanning
- Node discovery
- Diagnostic trouble code (DTC) reading
- Node status monitoring
- Statistics collection

### 8. lin_bridge_example.c
**Purpose**: LIN to UART bridge for PC monitoring
**Features**:
- Dual UART interfaces (LIN + PC)
- Frame logging and forwarding
- PC control interface
- Frame filtering
- Real-time monitoring

## LIN Protocol Implementation

### Frame Structure
```
Break | Sync | PID | Data[0-8] | Checksum
```

### PID Calculation
- 6-bit ID + 2 parity bits
- P0 = ID0 ⊕ ID1 ⊕ ID2 ⊕ ID4
- P1 = !(ID1 ⊕ ID3 ⊕ ID4 ⊕ ID5)

### Checksum Calculation (LIN 2.0)
- Sum of PID + all data bytes
- Carry handling: if sum > 255, subtract 255
- Final result: ~sum (bitwise NOT)

## Usage Instructions

### Building Examples
1. Copy the desired example to your main.c
2. Ensure all SPL libraries are included
3. Configure your build system (Makefile/CMake)
4. Compile and flash to STM32F103C8T6

### Testing Setup
1. Connect LIN transceiver to STM32
2. Connect LIN bus between nodes
3. Power up all nodes
4. Monitor communication via LEDs or bridge example

### Bridge Example PC Interface
The bridge example provides a UART interface for PC monitoring:

**Command Format**: `0xAA | Command | Length | Data[0-15] | Checksum`

**Commands**:
- `0x01`: Send LIN frame
- `0x02`: Set frame filter
- `0x03`: Get statistics
- `0x04`: Reset bridge
- `0x05`: Set monitor mode

## Frame ID Conventions

| ID Range | Purpose | Examples |
|----------|---------|----------|
| 0x10-0x1F | Master commands | 0x10: Master control |
| 0x20-0x2F | Slave status | 0x20-0x22: Slave 1-3 status |
| 0x30-0x3B | Sensor/Actuator | 0x30: Sensor data, 0x31: Actuator cmd |
| 0x3C | Diagnostic | Diagnostic requests/responses |
| 0x3D | Diagnostic response | Slave diagnostic responses |
| 0x3F | Broadcast | Network-wide commands |

## Error Handling

All examples include comprehensive error handling:
- Frame timeout detection
- Checksum validation
- PID parity verification
- Break detection
- Buffer overflow protection

## Performance Characteristics

- **Baud Rate**: 9600 bps (standard LIN)
- **Frame Rate**: Configurable (typically 10-100ms intervals)
- **Response Time**: < 10ms for slave responses
- **Error Rate**: < 0.1% under normal conditions

## Troubleshooting

### Common Issues
1. **No communication**: Check LIN transceiver connections
2. **Checksum errors**: Verify data integrity and timing
3. **Missing frames**: Check break detection configuration
4. **Timing issues**: Ensure proper system clock configuration

### Debug Features
- LED status indicators
- Error counters
- Frame logging (bridge example)
- Statistics tracking

## Advanced Features

### Diagnostic Capabilities
- Node discovery and enumeration
- Software/hardware version reading
- Diagnostic trouble code (DTC) management
- Network health monitoring

### Bridge Functionality
- Real-time frame monitoring
- PC-based control and analysis
- Frame filtering and logging
- Statistics and error reporting

## License

These examples are provided for educational and development purposes. Please refer to the main project license for usage terms.

## Author

- **Author**: hoangphuc540202@gmail.com
- **Github**: https://github.com/HoangPhuc02
- **Date**: 01/08/2025

## References

- LIN Specification 2.2A
- STM32F103 Reference Manual
- STM32F10x Standard Peripheral Library Documentation
