# SysTick Timer trong STM32F103 SPL - Tương đương HAL_GetTick()

## Tổng quan về SysTick Timer

SysTick là một timer 24-bit có sẵn trong tất cả ARM Cortex-M cores, được thiết kế để cung cấp time base cho hệ thống. Trong HAL library, nó được sử dụng để implement `HAL_GetTick()` và `HAL_Delay()`.

## So sánh SPL vs HAL

| Chức năng | HAL Library | SPL Library |
|-----------|-------------|-------------|
| Khởi tạo SysTick | `HAL_Init()` | `SysTick_Config_1ms()` |
| Lấy thời gian | `HAL_GetTick()` | `GetTick()` |
| Delay | `HAL_Delay(ms)` | `Delay_ms(ms)` |
| Interrupt Handler | `HAL_IncTick()` | `SysTick_Handler()` |

## 1. Cấu hình SysTick Timer

### Cách 1: Sử dụng hàm CMSIS có sẵn
```c
#include "core_cm3.h"
#include "system_stm32f10x.h"

// Biến đếm millisecond (global)
volatile uint32_t msTicks = 0;

void SysTick_Config_1ms(void)
{
    // Cấu hình SysTick để interrupt mỗi 1ms
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1); // Cấu hình thất bại
    }
}

// SysTick interrupt handler
void SysTick_Handler(void)
{
    msTicks++;
}
```

### Cách 2: Cấu hình thủ công
```c
void SysTick_Config_Manual(void)
{
    uint32_t ticks = SystemCoreClock / 1000; // 1ms
    
    SysTick->LOAD = ticks - 1;                    // Set reload value
    SysTick->VAL = 0;                             // Clear current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |  // Use processor clock
                    SysTick_CTRL_TICKINT_Msk |    // Enable interrupt
                    SysTick_CTRL_ENABLE_Msk;      // Enable SysTick
}
```

## 2. Các hàm utility

### GetTick() - Tương đương HAL_GetTick()
```c
uint32_t GetTick(void)
{
    return msTicks;
}
```

### Delay_ms() - Tương đương HAL_Delay()
```c
void Delay_ms(uint32_t delay)
{
    uint32_t start = GetTick();
    while ((GetTick() - start) < delay);
}
```

### Kiểm tra timeout
```c
uint8_t HasTimeElapsed(uint32_t start_time, uint32_t timeout_ms)
{
    return ((GetTick() - start_time) >= timeout_ms);
}

// Sử dụng:
uint32_t start = GetTick();
// Thực hiện operation
if (HasTimeElapsed(start, 1000)) {
    // Timeout after 1 second
}
```

## 3. Ứng dụng thực tế

### Button Debouncing với SysTick
```c
uint8_t Button_Debounced_Press(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    static uint8_t button_state = 0;
    static uint32_t debounce_timer = 0;
    const uint32_t DEBOUNCE_DELAY = 20;  // 20ms
    
    uint8_t current_pin = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
    
    switch (button_state) {
        case 0: // Idle
            if (current_pin == Bit_SET) {
                button_state = 1;
                debounce_timer = GetTick();
            }
            break;
            
        case 1: // Debouncing press
            if ((GetTick() - debounce_timer) > DEBOUNCE_DELAY) {
                if (current_pin == Bit_SET) {
                    button_state = 2;
                    return 1; // Button pressed
                } else {
                    button_state = 0;
                }
            }
            break;
            
        case 2: // Button held
            if (current_pin == Bit_RESET) {
                button_state = 3;
                debounce_timer = GetTick();
            }
            break;
            
        case 3: // Debouncing release
            if ((GetTick() - debounce_timer) > DEBOUNCE_DELAY) {
                button_state = 0;
            }
            break;
    }
    return 0;
}
```

### Periodic Task với SysTick
```c
void Periodic_Tasks(void)
{
    static uint32_t last_led_toggle = 0;
    static uint32_t last_data_send = 0;
    
    // Nhấp nháy LED mỗi 500ms
    if (HasTimeElapsed(last_led_toggle, 500)) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, 
                     (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));
        last_led_toggle = GetTick();
    }
    
    // Gửi dữ liệu mỗi 1 giây
    if (HasTimeElapsed(last_data_send, 1000)) {
        // Send data
        last_data_send = GetTick();
    }
}
```

## 4. Tính toán SysTick Values

### Công thức tính toán:
```
SysTick_Value = (SystemCoreClock / Desired_Frequency) - 1
```

### Ví dụ với STM32F103 (72MHz):
- **1ms (1000Hz)**: 72,000,000 / 1000 - 1 = 71,999
- **100μs (10000Hz)**: 72,000,000 / 10000 - 1 = 7,199
- **10μs (100000Hz)**: 72,000,000 / 100000 - 1 = 719

### Giới hạn SysTick:
- **Maximum Value**: 2^24 - 1 = 16,777,215
- **Minimum Frequency**: 72MHz / 16,777,215 ≈ 4.3Hz
- **Maximum Frequency**: 72MHz (1 tick)

## 5. Ưu điểm SysTick Timer

1. **Không cần cấu hình peripheral clock** - là core peripheral
2. **Độ chính xác cao** - sử dụng system clock
3. **Ít overhead** - hardware counter tự động
4. **Tương thích** - có sẵn trên tất cả Cortex-M
5. **Non-blocking** - chạy trong background

## 6. Lưu ý khi sử dụng

### Overflow handling:
```c
// SysTick sẽ overflow sau ~49 ngày với 1ms tick
// Để xử lý overflow an toàn:
uint8_t IsTimeElapsed(uint32_t start, uint32_t duration)
{
    uint32_t current = GetTick();
    
    // Xử lý overflow bằng cách dùng unsigned arithmetic
    return ((current - start) >= duration);
}
```

### Power saving:
```c
void Delay_ms_LowPower(uint32_t delay)
{
    uint32_t start = GetTick();
    
    while ((GetTick() - start) < delay) {
        __WFI(); // Wait For Interrupt - tiết kiệm điện
    }
}
```

## 7. Integration với main.c

```c
int main(void)
{
    // Khởi tạo system
    SystemInit();
    SysTick_Config_1ms();  // Quan trọng: phải gọi sau SystemInit()
    
    // Cấu hình peripherals
    GPIO_Config();
    // ... other configs
    
    while (1) {
        // Main loop với timing chính xác
        Periodic_Tasks();
        Delay_ms(1); // 1ms loop time
    }
}
```

## 8. Debug và Troubleshooting

### Kiểm tra SysTick hoạt động:
```c
void Test_SysTick(void)
{
    uint32_t start = GetTick();
    uint32_t count = 0;
    
    while (count < 10) {
        if (GetTick() != start) {
            count++;
            start = GetTick();
            // printf("Tick: %lu\\n", start);
        }
    }
}
```

### Common Issues:
1. **SystemCoreClock chưa được update** - gọi `SystemCoreClockUpdate()`
2. **SysTick_Handler không được gọi** - kiểm tra interrupt enable
3. **Timing không chính xác** - kiểm tra system clock configuration

SysTick timer là giải pháp tối ưu để tạo time base trong embedded systems, cung cấp timing chính xác và dễ sử dụng tương tự như HAL library.
