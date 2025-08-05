# Giải thích chi tiết về CAN với DMA trên STM32F103

## Tổng quan (Overview)

Dự án này minh họa việc sử dụng CAN (Controller Area Network) kết hợp với DMA (Direct Memory Access) trên STM32F103C8T6. Hệ thống thực hiện truyền thông CAN liên tục và hiển thị dữ liệu qua UART sử dụng DMA để tối ưu hiệu suất.

## Kiến trúc hệ thống (System Architecture)

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   CAN Module    │    │   DMA Engine    │    │  UART Module    │
│                 │    │                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │   TX FIFO   │ │    │ │  Channel 4  │ │    │ │  Data Reg   │ │
│ └─────────────┘ │    │ │ (UART TX)   │ │    │ │   (DR)      │ │
│ ┌─────────────┐ │    │ └─────────────┘ │    │ └─────────────┘ │
│ │   RX FIFO   │◄┼────┼─► Memory Buffer ◄┼────┤                 │
│ └─────────────┘ │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                        │                        │
        ▼                        ▼                        ▼
   Loopback Mode           Automatic Transfer        Serial Output
   (PB8/PB9)              (No CPU intervention)      (PA9 - 115200)
```

## Chi tiết cấu hình (Detailed Configuration)

### 1. Clock Configuration

```c
// System Clock: 72MHz
// APB1 Clock: 36MHz (for CAN peripheral)
// APB2 Clock: 72MHz (for GPIO, UART)
// AHB Clock: 72MHz (for DMA)

RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // 8MHz * 9 = 72MHz
RCC_PCLK1Config(RCC_HCLK_Div2);    // APB1 = 36MHz
RCC_PCLK2Config(RCC_HCLK_Div1);    // APB2 = 72MHz
```

**Giải thích**: 
- CAN cần APB1 clock = 36MHz để tính toán bit timing
- DMA hoạt động trên AHB clock = 72MHz để đảm bảo tốc độ transfer cao
- UART trên APB2 = 72MHz để hỗ trợ baudrate cao

### 2. CAN Configuration chi tiết

#### 2.1 GPIO Configuration
```c
// PB8: CAN_RX (Input Pull-up)
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // Input with internal pull-up
GPIO_Init(GPIOB, &GPIO_InitStructure);

// PB9: CAN_TX (Alternate Function Push-Pull)
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // Alternate function
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // High speed for signal integrity
GPIO_Init(GPIOB, &GPIO_InitStructure);
```

**Giải thích**:
- CAN_RX cần pull-up để đảm bảo logic level ổn định khi không có tín hiệu
- CAN_TX ở chế độ Alternate Function để CAN peripheral điều khiển trực tiếp
- Tốc độ 50MHz đảm bảo edge time nhanh cho tín hiệu CAN

#### 2.2 CAN Bit Timing Calculation

```c
// Target: 1Mbps CAN baudrate
// Formula: Baudrate = APB1_Clock / (Prescaler × (1 + BS1 + BS2))
// 1,000,000 = 36,000,000 / (Prescaler × (1 + 8 + 1))
// 1,000,000 = 36,000,000 / (Prescaler × 10)
// Prescaler = 36,000,000 / (1,000,000 × 10) = 3.6 ≈ 4

CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;     // Synchronization Jump Width
CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;     // Bit Segment 1 = 8 time quanta
CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;     // Bit Segment 2 = 1 time quantum
CAN_InitStructure.CAN_Prescaler = 4;         // Clock prescaler
```

**Giải thích**:
- **SJW (Synchronization Jump Width)**: Độ lệch tối đa cho phép khi đồng bộ
- **BS1**: Thời gian sampling (8 tq = 80% của bit time)
- **BS2**: Thời gian sau sampling point
- **Prescaler**: Chia clock để có tần số bit time mong muốn

#### 2.3 CAN Filter Configuration

```c
// Accept all CAN messages (no filtering)
CAN_FilterInitStructure.CAN_FilterNumber = 0;
CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;      // Accept any ID
CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  // Mask = 0 means don't care
CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
```

**Giải thích**:
- Filter 0 được cấu hình để chấp nhận tất cả message
- Mask = 0x0000 có nghĩa là không quan tâm đến ID
- Tất cả message được đưa vào FIFO0

### 3. DMA Configuration chi tiết

#### 3.1 DMA Channel Assignment
```c
// STM32F103 DMA1 Channel mapping:
// Channel 1: ADC1, TIM2_CH3, TIM4_CH1
// Channel 2: SPI1_RX, USART3_TX, TIM1_CH1
// Channel 3: SPI1_TX, USART3_RX, TIM1_CH2
// Channel 4: SPI2/I2S2_RX, USART1_TX, TIM1_CH4  ← Used for UART TX
// Channel 5: SPI2/I2S2_TX, USART1_RX, TIM1_TRIG
// Channel 6: USART2_RX, TIM1_CH3, TIM3_CH1
// Channel 7: USART2_TX, TIM2_CH2, TIM4_CH2
```

#### 3.2 DMA Transfer Configuration
```c
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;  // Destination
DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_tx_buffer;   // Source
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                 // Memory to Peripheral
DMA_InitStructure.DMA_BufferSize = 0;                              // Set dynamically
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // DR address fixed
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;            // Buffer address increments
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // 8-bit transfer
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;          // 8-bit transfer
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                            // One-shot transfer
DMA_InitStructure.DMA_Priority = DMA_Priority_High;                      // High priority
```

**Giải thích**:
- **PeripheralBaseAddr**: Địa chỉ USART1 Data Register (DR)
- **MemoryBaseAddr**: Địa chỉ buffer chứa dữ liệu cần gửi
- **DIR**: Hướng transfer từ Memory → Peripheral
- **PeripheralInc = Disable**: DR address không thay đổi
- **MemoryInc = Enable**: Buffer address tăng dần sau mỗi byte
- **Mode = Normal**: Transfer một lần rồi dừng (không circular)

### 4. Interrupt Configuration

#### 4.1 CAN Interrupt
```c
// Enable CAN RX interrupt
CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  // FIFO0 Message Pending

// NVIC Configuration
NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // Highest priority
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
```

#### 4.2 DMA Interrupt
```c
// Enable DMA transfer complete interrupt
DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

// NVIC Configuration
NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // Lower than CAN
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
```

**Giải thích Priority**:
- CAN RX có priority cao hơn (0) để đảm bảo không mất data
- DMA TC có priority thấp hơn (1) vì không time-critical

## Data Flow chi tiết

### 1. CAN Message Transmission
```
CPU → TxMessage Structure → CAN Peripheral → Loopback → RX FIFO
```

### 2. CAN Message Reception
```
RX FIFO → Interrupt → CPU copies to can_rx_buffer → Set flag
```

### 3. Data Formatting
```
can_rx_buffer → Format_CAN_Data_For_UART() → uart_tx_buffer
```

### 4. UART Transmission via DMA
```
uart_tx_buffer → DMA Channel 4 → USART1 DR → Serial Output
```

## Memory Layout

```c
// Global Variables in RAM
├── CAN_InitStructure        (36 bytes)    - CAN configuration
├── TxMessage                (16 bytes)    - CAN TX message structure
├── RxMessage                (16 bytes)    - CAN RX message structure
├── can_rx_buffer[8]         (8 bytes)     - CAN received data
├── uart_tx_buffer[20]       (20 bytes)    - UART formatted data
├── can_data_ready           (1 byte)      - Flag: new data available
├── uart_dma_busy            (1 byte)      - Flag: DMA transfer in progress
└── transmission_counter     (4 bytes)     - Message counter

Total RAM usage: ~102 bytes + stack
```

## Timing Analysis

### 1. CAN Transmission Timing
```
Bit Time = 1 µs (for 1Mbps)
Frame Time = ~108 µs (for 8-byte data frame)
```

### 2. UART Transmission Timing
```
Baud Rate = 115200 bps
Bit Time = 8.68 µs
Byte Time = 86.8 µs (including start/stop bits)
Full Message Time = ~1.74 ms (for 20-byte message)
```

### 3. DMA Transfer Timing
```
DMA Clock = 72MHz
Transfer Rate = 1 byte per clock cycle
20-byte transfer = 20 × 13.9 ns = 278 ns (practically instantaneous)
```

## Debugging Tips

### 1. CAN Issues
```c
// Check CAN status
uint32_t can_status = CAN1->MSR;
if (can_status & CAN_MSR_ERRI) {
    // Error interrupt flag set
}

// Check transmission status
uint8_t tx_status = CAN_TransmitStatus(CAN1, mailbox);
// CAN_TxStatus_Ok, CAN_TxStatus_Failed, CAN_TxStatus_Pending
```

### 2. DMA Issues
```c
// Check DMA flags
if (DMA_GetFlagStatus(DMA1_FLAG_TC4)) {
    // Transfer complete
}
if (DMA_GetFlagStatus(DMA1_FLAG_TE4)) {
    // Transfer error
}

// Check remaining transfer count
uint16_t remaining = DMA_GetCurrDataCounter(DMA1_Channel4);
```

### 3. UART Issues
```c
// Check UART flags
if (USART_GetFlagStatus(USART1, USART_FLAG_TXE)) {
    // Transmit data register empty
}
if (USART_GetFlagStatus(USART1, USART_FLAG_TC)) {
    // Transmission complete
}
```

## Performance Benefits of DMA

### Without DMA (Polling method):
```c
void UART_SendString_Polling(char* str) {
    while (*str) {
        while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));  // Wait for TXE
        USART_SendData(USART1, *str++);                        // Send byte
    }
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TC));       // Wait for TC
}
// CPU Usage: 100% during transmission (~1.74ms for 20 bytes)
```

### With DMA:
```c
void UART_SendString_DMA(char* str, uint16_t length) {
    DMA1_Channel4->CNDTR = length;     // Set transfer count
    DMA_Cmd(DMA1_Channel4, ENABLE);    // Start transfer
}
// CPU Usage: ~1% (only for setup), CPU free during transfer
```

## Advanced Features

### 1. Circular DMA for Continuous Transmission
```c
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
// Buffer will automatically restart from beginning when complete
```

### 2. Multiple CAN ID Filtering
```c
// Filter for specific CAN IDs
CAN_FilterInitStructure.CAN_FilterIdHigh = (0x123 << 5);      // Accept ID 0x123
CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (0x7FF << 5);  // Exact match
```

### 3. Error Handling
```c
// CAN Error Handling
if (CAN_GetLastErrorCode(CAN1) != CAN_ErrorCode_NoErr) {
    CAN_ClearFlag(CAN1, CAN_FLAG_LEC);  // Clear error flags
}

// DMA Error Handling
if (DMA_GetITStatus(DMA1_IT_TE4)) {
    DMA_ClearITPendingBit(DMA1_IT_TE4);  // Clear transfer error
}
```

## Kết luận (Conclusion)

Việc sử dụng DMA cho UART transmission mang lại những lợi ích sau:

1. **Giải phóng CPU**: CPU có thể xử lý các tác vụ khác trong khi DMA transfer
2. **Tốc độ cao**: DMA transfer với tốc độ bus clock (72MHz)
3. **Độ tin cậy**: Giảm thiểu việc mất dữ liệu do interrupt latency
4. **Tiết kiệm năng lượng**: CPU có thể vào sleep mode trong khi DMA hoạt động

Dự án này là nền tảng tốt để phát triển các ứng dụng CAN phức tạp hơn như:
- CAN Gateway/Router
- Vehicle diagnostic system
- Industrial automation networks
- Real-time data logging systems
