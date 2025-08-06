/*
 * =============================================================================
 * Project: STM32F103 SysTick Timer Example (SPL)
 * File: systick_example.c
 * Description: SysTick timer configuration and usage like HAL_GetTick()
 * Author: SPL Example Team
 * Date: August 2025
 * 
 * SysTick Timer Overview:
 * - SysTick là timer 24-bit có sẵn trong tất cả Cortex-M cores
 * - Thường được dùng để tạo time base cho RTOS hoặc delay functions
 * - Có thể tạo interrupt mỗi 1ms giống HAL_GetTick()
 * - Không cần cấu hình peripheral clock vì là core peripheral
 * 
 * Công thức tính SysTick:
 * SysTick_Value = (SystemCoreClock / Frequency) - 1
 * Ví dụ: 72MHz / 1000Hz = 72000 - 1 = 71999 (cho interrupt 1ms)
 * =============================================================================
 */

#include "stm32f10x.h"
#include "core_cm3.h"

/* ======================= Global Variables ======================= */
// Biến đếm millisecond - tương tự HAL_GetTick()
volatile uint32_t msTicks = 0;

// Biến đếm microsecond (optional)
volatile uint32_t usTicks = 0;

/* ======================= Function Prototypes ======================= */
void SysTick_Config_1ms(void);
void SysTick_Config_Custom(uint32_t frequency_hz);
uint32_t GetTick(void);
uint32_t GetMicroTick(void);
void Delay_ms(uint32_t delay);
void Delay_us(uint32_t delay);
uint8_t HasTimeElapsed(uint32_t start_time, uint32_t timeout_ms);

/* ======================= SysTick Configuration ======================= */
/**
 * @brief Cấu hình SysTick cho interrupt 1ms (giống HAL)
 * @note  Tương đương với HAL_Init() trong HAL library
 */
void SysTick_Config_1ms(void)
{
    // Cấu hình SysTick để interrupt mỗi 1ms
    // SystemCoreClock thường là 72MHz cho STM32F103
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        // Cấu hình thất bại - xử lý lỗi
        while (1);
    }
    
    // Set priority cho SysTick interrupt (optional)
    NVIC_SetPriority(SysTick_IRQn, 0);
}

/**
 * @brief Cấu hình SysTick với tần số tùy chỉnh
 * @param frequency_hz: Tần số interrupt mong muốn (Hz)
 */
void SysTick_Config_Custom(uint32_t frequency_hz)
{
    uint32_t ticks = SystemCoreClock / frequency_hz;
    
    if (ticks > SysTick_LOAD_RELOAD_Msk)
    {
        // Tần số quá thấp, SysTick không thể đếm
        return;
    }
    
    SysTick->LOAD = ticks - 1;                    // Set reload value
    SysTick->VAL = 0;                             // Clear current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |  // Use processor clock
                    SysTick_CTRL_TICKINT_Msk |    // Enable interrupt
                    SysTick_CTRL_ENABLE_Msk;      // Enable SysTick
}

/* ======================= SysTick Interrupt Handler ======================= */
/**
 * @brief SysTick interrupt handler - được gọi mỗi 1ms
 * @note  Tương tự như HAL_IncTick() trong HAL library
 */
void SysTick_Handler(void)
{
    msTicks++;           // Tăng biến đếm millisecond
    
    // Optional: đếm microsecond (ước tính)
    // Chú ý: không chính xác 100% do overhead của interrupt
    usTicks += 1000;
}

/* ======================= Time Functions ======================= */
/**
 * @brief Lấy thời gian hiện tại tính bằng millisecond
 * @return Thời gian tính từ lúc khởi động (ms)
 * @note  Tương đương với HAL_GetTick()
 */
uint32_t GetTick(void)
{
    return msTicks;
}

/**
 * @brief Lấy thời gian hiện tại tính bằng microsecond (ước tính)
 * @return Thời gian tính từ lúc khởi động (μs)
 */
uint32_t GetMicroTick(void)
{
    return usTicks;
}

/**
 * @brief Delay chính xác bằng millisecond
 * @param delay: Thời gian delay (ms)
 * @note  Tương đương với HAL_Delay()
 */
void Delay_ms(uint32_t delay)
{
    uint32_t start = GetTick();
    
    // Đợi cho đến khi đủ thời gian
    while ((GetTick() - start) < delay)
    {
        // Có thể thêm __WFI() để tiết kiệm điện
        // __WFI(); // Wait For Interrupt
    }
}

/**
 * @brief Delay microsecond (không chính xác như ms)
 * @param delay: Thời gian delay (μs)
 * @note  Chỉ ước tính, không chính xác như delay_ms
 */
void Delay_us(uint32_t delay)
{
    uint32_t start = usTicks;
    
    while ((usTicks - start) < delay)
    {
        // Busy wait
    }
}

/**
 * @brief Kiểm tra xem thời gian timeout đã trôi qua chưa
 * @param start_time: Thời điểm bắt đầu (từ GetTick())
 * @param timeout_ms: Thời gian timeout (ms)
 * @return 1 nếu đã timeout, 0 nếu chưa
 */
uint8_t HasTimeElapsed(uint32_t start_time, uint32_t timeout_ms)
{
    return ((GetTick() - start_time) >= timeout_ms);
}

/* ======================= Example Usage ======================= */
/**
 * @brief Ví dụ sử dụng SysTick timer
 */
void SysTick_Example(void)
{
    uint32_t last_blink = 0;
    uint32_t last_print = 0;
    uint32_t start_time = 0;
    
    // Khởi tạo SysTick (1ms interrupt)
    SysTick_Config_1ms();
    
    while (1)
    {
        // Nhấp nháy LED mỗi 500ms
        if (HasTimeElapsed(last_blink, 500))
        {
            // Toggle LED
            GPIOC->ODR ^= GPIO_Pin_13;
            last_blink = GetTick();
        }
        
        // In thông tin mỗi 1 giây
        if (HasTimeElapsed(last_print, 1000))
        {
            // printf("Current time: %lu ms\n", GetTick());
            last_print = GetTick();
        }
        
        // Ví dụ timeout operation
        start_time = GetTick();
        // Thực hiện một operation nào đó
        if (HasTimeElapsed(start_time, 100))
        {
            // Operation timeout sau 100ms
            // Xử lý timeout
        }
        
        // Delay 10ms giữa các lần kiểm tra
        Delay_ms(10);
    }
}

/* ======================= Advanced SysTick Functions ======================= */
/**
 * @brief Lấy thời gian chính xác hơn với SysTick counter
 * @return Thời gian với độ phân giải cao hơn
 */
uint32_t GetTickHighRes(void)
{
    uint32_t ms, ticks;
    
    // Đọc giá trị hiện tại (cần disable interrupt để đồng bộ)
    __disable_irq();
    ms = msTicks;
    ticks = SysTick->VAL;
    __enable_irq();
    
    // Tính thời gian với độ phân giải cao
    // SysTick đếm ngược từ LOAD về 0
    uint32_t elapsed_ticks = SysTick->LOAD - ticks;
    uint32_t us_in_tick = (elapsed_ticks * 1000) / SysTick->LOAD;
    
    return (ms * 1000 + us_in_tick); // Return in microseconds
}

/**
 * @brief Đo thời gian thực thi của một đoạn code
 * @param func: Con trỏ hàm cần đo
 * @return Thời gian thực thi (μs)
 */
uint32_t MeasureExecutionTime(void (*func)(void))
{
    uint32_t start = GetTickHighRes();
    func();
    return (GetTickHighRes() - start);
}

/**
 * @brief Tạo delay chính xác với SysTick counter
 * @param us: Thời gian delay (microseconds)
 */
void Delay_us_Precise(uint32_t us)
{
    uint32_t start = GetTickHighRes();
    while ((GetTickHighRes() - start) < us);
}
