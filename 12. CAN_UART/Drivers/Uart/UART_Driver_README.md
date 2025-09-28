# STM32F103 UART Driver

🚀 **Comprehensive UART driver cho STM32F103C8T6 hỗ trợ 3 chế độ truyền dữ liệu**

## 📋 Tính Năng

### 🔧 **Chế độ hoạt động**
- ✅ **Polling Mode** - Truyền/nhận đồng bộ với timeout
- ✅ **Interrupt Mode** - Truyền/nhận bất đồng bộ với interrupt
- ✅ **DMA Mode** - Truyền/nhận tốc độ cao với DMA

### 🌟 **Tính năng nâng cao**
- ✅ Multiple UART support (UART1, UART2, UART3)
- ✅ Configurable baud rates (9600 - 115200)
- ✅ Error handling và callback functions
- ✅ Printf implementation
- ✅ Echo và string utilities
- ✅ Hardware flow control support

### ⚙️ **Cấu hình linh hoạt**
- ✅ Word length: 8-bit, 9-bit
- ✅ Stop bits: 1, 1.5, 2
- ✅ Parity: None, Even, Odd
- ✅ Circular buffer support
- ✅ Timeout configuration

## 📁 Cấu Trúc Project

```
STM32F103_UART_Driver/
├── Config/
│   └── uart_cfg.h          # Cấu hình UART
├── Drivers/
│   └── Uart/
│       ├── uart.h          # Header chính
│       └── uart.c          # Implementation
├── SPL/                    # Standard Peripheral Library
├── CMSIS/                  # CMSIS headers
├── Startup/                # Startup files
├── Linker/                 # Linker scripts
├── uart_example.c          # Example usage
├── Makefile               # Build configuration
└── README.md              # Hướng dẫn này
```

## 🛠️ Hardware Configuration

### **UART1 (Default)**
```c
Pin TX:  PA9  (Alternate Function Push-Pull)
Pin RX:  PA10 (Input Floating)
Baud:    115200
DMA TX:  DMA1 Channel 4
DMA RX:  DMA1 Channel 5
IRQ:     USART1_IRQn
```

### **UART2 (Optional)**
```c
Pin TX:  PA2  (Alternate Function Push-Pull)  
Pin RX:  PA3  (Input Floating)
Baud:    9600 (configurable)
DMA TX:  DMA1 Channel 7
DMA RX:  DMA1 Channel 6
IRQ:     USART2_IRQn
```

### **UART3 (Optional)**
```c
Pin TX:  PB10 (Alternate Function Push-Pull)
Pin RX:  PB11 (Input Floating)
Baud:    9600 (configurable)
DMA TX:  DMA1 Channel 2
DMA RX:  DMA1 Channel 3
IRQ:     USART3_IRQn
```

## 🚀 Quick Start

### **1. Khởi tạo cơ bản**
```c
#include "uart.h"

// Khởi tạo UART1 với cấu hình mặc định
UART_HandleTypeDef huart1;
huart1.Instance = USART1;
huart1.BaudRate = 115200;

// Chọn chế độ hoạt động
UART_Init(&huart1, UART_MODE_POLLING);    // Polling
UART_Init(&huart1, UART_MODE_INTERRUPT);  // Interrupt  
UART_Init(&huart1, UART_MODE_DMA);        // DMA
```

### **2. Truyền dữ liệu**
```c
uint8_t data[] = "Hello STM32!";

// Polling mode
UART_Transmit_Polling(&huart1, data, strlen(data), 1000);

// Interrupt mode
UART_Transmit_IT(&huart1, data, strlen(data));
while(!uart_tx_complete); // Wait completion

// DMA mode
UART_Transmit_DMA(&huart1, data, strlen(data));
while(uart_dma_busy); // Wait completion
```

### **3. Nhận dữ liệu**
```c
uint8_t buffer[100];

// Polling mode
UART_Receive_Polling(&huart1, buffer, 10, 5000);

// Interrupt mode
UART_Receive_IT(&huart1, buffer, 10);
while(!uart_rx_complete); // Wait completion

// DMA mode  
UART_Receive_DMA(&huart1, buffer, 10);
while(!uart_rx_complete); // Wait completion
```

### **4. Utility functions**
```c
// Send character
UART_SendChar(&huart1, 'A');

// Send string
UART_SendString(&huart1, "Hello World!\r\n");

// Printf implementation
UART_Printf(&huart1, "Value: %d, Voltage: %.2f\r\n", 123, 3.14);

// Receive character
char ch = UART_ReceiveChar(&huart1);
```

## 🔧 Configuration Options

### **uart_cfg.h - Cấu hình chính**
```c
// Enable/Disable UART instances
#define UART1_ENABLE              1
#define UART2_ENABLE              0  
#define UART3_ENABLE              0

// Default transmission mode
#define UART_DEFAULT_MODE         UART_MODE_POLLING

// Buffer sizes
#define UART_TX_BUFFER_SIZE       256
#define UART_RX_BUFFER_SIZE       256
#define UART_PRINTF_BUFFER_SIZE   128

// Enable features
#define UART_DMA_ENABLE           1
#define UART_IT_ENABLE            1
#define UART_HW_FLOW_CONTROL      0
#define UART_PARITY_ENABLE        0
#define UART_DEBUG_ENABLE         1
```

## 💾 Memory Usage

| Configuration | Flash (KB) | RAM (KB) |
|---------------|-----------|----------|
| Polling only  | ~8        | ~1       |
| + Interrupt   | ~12       | ~1.5     |
| + DMA         | ~15       | ~2       |
| Full features | ~18       | ~2.5     |

## 📊 Performance Comparison

| Mode      | CPU Usage | Speed    | Reliability | Use Case |
|-----------|-----------|----------|-------------|----------|
| Polling   | High      | Low      | High        | Simple apps |
| Interrupt | Medium    | Medium   | High        | Real-time |
| DMA       | Low       | High     | Very High   | High throughput |

## 🔧 Build & Flash

### **Build Project**
```bash
# Build
make

# Clean and rebuild
make clean all

# Show size information
make size

# Generate disassembly
make disasm
```

### **Flash to MCU**
```bash
# Using st-link
make flash

# Using STM32CubeProgrammer
make flash-cube

# Erase flash
make erase
```

### **Debug**
```bash
# Start debug session (OpenOCD + GDB)
make debug
```

## 🧪 Testing

### **Run Example**
1. Connect UART1 (PA9/PA10) to USB-Serial converter
2. Open terminal at 115200 baud
3. Flash firmware: `make flash`
4. Select test options from menu:
   - `1` - Test Polling Mode
   - `2` - Test Interrupt Mode  
   - `3` - Test DMA Mode
   - `4` - Echo Test
   - `5` - Printf Test

### **Expected Output**
```
========================================
  STM32F103 UART Driver Demo
  Author: hoangphuc540202@gmail.com
  Date: 22/9/2025
========================================

=== STM32F103 UART Driver Test Menu ===
1. Test Polling Mode
2. Test Interrupt Mode
3. Test DMA Mode
4. Echo Test (Type and press Enter)
5. Printf Test
Select option (1-5): 
```

## 👤 Author

**hoangphuc540202@gmail.com**
- 📧 Email: hoangphuc540202@gmail.com  
- 🐙 GitHub: [HoangPhuc02](https://github.com/HoangPhuc02)
- 📅 Date: 22/9/2025

---
⭐ **Don't forget to star this repository if it helps you!**