/*
 * =============================================================================
 * Project: STM32F103 CAN Transmitter (SPL)
 * File: can_normal_mode_tx.c
 * Description: CAN Transmitter using Normal Mode with button trigger
 * Author: CAN Driver Team
 * Date: August 2025
 * 
 * Hardware Setup:
 * - STM32F103C8T6 Blue Pill board
 * - CAN pins: PA11 (RX), PA12 (TX)
 * - Button: PA0 (with internal pull-down)
 * - LED: PC13 (indicates transmission)
 * - CAN Transceiver: TJA1050 or similar
 * 
 * Features:
 * - Button-triggered CAN transmission
 * - Debounced button input
 * - LED indication for transmission status
 * - 250kbps CAN baudrate
 * - Standard 11-bit CAN ID
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "system_stm32f10x.h"
#include "misc.h"

/* ======================= Global Variables ======================= */
CanTxMsg TxMessage;
volatile uint8_t button_pressed = 0;
uint32_t message_counter = 0;

/* ======================= Function Prototypes ======================= */
void SystemClock_Config(void);
void GPIO_Config(void);
void CAN_Config(void);
void CAN_TransmitMessage(void);
void Delay(uint32_t count);
uint8_t Button_Debounced_Press(void);

/* ======================= CAN Transmission Function ======================= */
/**
 * @brief Transmit CAN message with incrementing counter
 * @note  Sends 8-byte message with counter and test pattern
 */
void CAN_TransmitMessage(void)
{
    uint8_t transmit_mailbox;
    uint32_t timeout = 0;

    /* Prepare CAN message */
    TxMessage.StdId = 0x123;                    // Standard ID
    TxMessage.ExtId = 0x00;                     // Extended ID (not used)
    TxMessage.IDE = CAN_Id_Standard;            // Use standard ID
    TxMessage.RTR = CAN_RTR_Data;               // Data frame
    TxMessage.DLC = 8;                          // Data length: 8 bytes
    
    /* Fill data payload */
    TxMessage.Data[0] = (message_counter >> 24) & 0xFF;  // Counter MSB
    TxMessage.Data[1] = (message_counter >> 16) & 0xFF;
    TxMessage.Data[2] = (message_counter >> 8) & 0xFF;
    TxMessage.Data[3] = message_counter & 0xFF;          // Counter LSB
    TxMessage.Data[4] = 0xAA;                            // Test pattern
    TxMessage.Data[5] = 0x55;                            // Test pattern
    TxMessage.Data[6] = 0xCC;                            // Test pattern
    TxMessage.Data[7] = 0x33;                            // Test pattern

    /* Turn on LED to indicate transmission */
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);

    /* Transmit message */
    transmit_mailbox = CAN_Transmit(CAN1, &TxMessage);
    
    /* Wait for transmission completion with timeout */
    while((CAN_TransmitStatus(CAN1, transmit_mailbox) != CAN_TxStatus_Ok) && (timeout < 1000000))
    {
        timeout++;
    }
    
    /* Turn off LED */
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    /* Increment message counter */
    message_counter++;
    
    /* Brief delay to show LED */
    Delay(100000);
}


/* ======================= Button Debouncing ======================= */
/**
 * @brief Debounced button press detection
 * @return 1 if button was just pressed, 0 otherwise
 */
uint8_t Button_Debounced_Press(void)
{
    static uint8_t button_state = 0;
    static uint32_t debounce_timer = 0;
    const uint32_t DEBOUNCE_DELAY = 20000;  // Adjust based on loop timing
    
    uint8_t current_pin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    
    switch (button_state) {
        case 0: // Idle - waiting for press
            if (current_pin == Bit_SET) {
                button_state = 1;
                debounce_timer = 0;
            }
            break;
            
        case 1: // Debouncing press
            debounce_timer++;
            if (debounce_timer > DEBOUNCE_DELAY) {
                if (current_pin == Bit_SET) {
                    button_state = 2;
                    return 1; // Button press confirmed
                } else {
                    button_state = 0; // False trigger
                }
            }
            break;
            
        case 2: // Button held - waiting for release
            if (current_pin == Bit_RESET) {
                button_state = 3;
                debounce_timer = 0;
            }
            break;
            
        case 3: // Debouncing release
            debounce_timer++;
            if (debounce_timer > DEBOUNCE_DELAY) {
                if (current_pin == Bit_RESET) {
                    button_state = 0; // Button release confirmed
                } else {
                    button_state = 2; // Still pressed
                }
            }
            break;
    }
    
    return 0;
}

/* =============================================================================
 * GLOBAL VARIABLES - Remove old variables
 * =============================================================================
 */

/* ======================= Main Function ======================= */
/**
 * @brief Main application function - CAN Transmitter
 * @note  Button-triggered CAN transmission with debouncing
 * 
 * Program Flow:
 * 1. Initialize system clock (72MHz)
 * 2. Configure GPIO (CAN, Button, LED)
 * 3. Configure CAN peripheral
 * 4. Main loop: Wait for button press and transmit CAN message
 */
int main(void)
{
    /* System Initialization */
    SystemInit();            // 1. Setup system clock to 72MHz
    CAN_GPIO_Config();       // 2. Configure GPIO pins
    CAN_Config();            // 3. Configure CAN peripheral
    
    /* Startup indication - blink LED 3 times */
    for (uint8_t i = 0; i < 3; i++) {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // LED on
        Delay(200000);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);    // LED off
        Delay(200000);
    }
    
    /* Main application loop */
    while (1)
    {
        /* Check for button press */
        if (Button_Debounced_Press()) {
            /* Transmit CAN message */
            CAN_TransmitMessage();
        }
        
        /* Small delay for debouncing timing */
        Delay(1000);
    }
}
/* =============================================================================
 * HÀM CẤU HÌNH XUNG NHỊP HỆ THỐNG (72MHz)
 * =============================================================================
 * Thiết lập xung nhịp hệ thống sử dụng HSE (thạch anh ngoài) và PLL để đạt 72MHz.
 * Các bước:
 * - Reset cấu hình RCC
 * - Bật HSE
 * - Chờ HSE sẵn sàng
 * - Cấu hình Flash latency
 * - Cấu hình các bus: AHB, APB1, APB2
 * - Cấu hình PLL nhân xung lên 72MHz
 * - Chọn PLL làm nguồn xung nhịp hệ thống
 */
void SystemClock_Config(void)
{
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit(); // Reset cấu hình RCC về mặc định
    RCC_HSEConfig(RCC_HSE_ON); // Bật thạch anh ngoài (HSE)
    HSEStartUpStatus = RCC_WaitForHSEStartUp(); // Chờ HSE sẵn sàng

    if(HSEStartUpStatus == SUCCESS)
    {
        FLASH_SetLatency(FLASH_Latency_2); // Thiết lập độ trễ cho Flash (phù hợp với 72MHz)
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); // Bật bộ đệm Flash
        RCC_HCLKConfig(RCC_SYSCLK_Div1);    // AHB = SYSCLK (không chia)
        RCC_PCLK1Config(RCC_HCLK_Div2);     // APB1 = AHB/2 = 36MHz
        RCC_PCLK2Config(RCC_HCLK_Div1);     // APB2 = AHB = 72MHz
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // PLL: 8MHz * 9 = 72MHz
        RCC_PLLCmd(ENABLE); // Bật PLL
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); // Chờ PLL sẵn sàng
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // Chọn PLL làm nguồn xung nhịp hệ thống
        while(RCC_GetSYSCLKSource() != 0x08); // Đảm bảo PLL đã được chọn
    }
    else
    {
        while(1); // Nếu HSE lỗi, dừng chương trình tại đây
    }
}
/*
 * CAN GPIO CONFIGURATION
 * =============================================================================
 * CAN RX: PA11, CAN TX: PA12
 */
void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    // CAN RX (PA11) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // CAN TX (PA12) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PC13 for LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* =============================================================================
 * CAN CONFIGURATION
 * =============================================================================
 */
void CAN_Config(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    CAN_InitStructure.CAN_TTCM = DISABLE;      // TTCM: Time Triggered Communication Mode. DISABLE: không dùng chế độ truyền theo thời gian (ít dùng trong ứng dụng phổ thông)
    CAN_InitStructure.CAN_ABOM = DISABLE;      // ABOM: Automatic Bus-Off Management. DISABLE: không tự động phục hồi khi bus-off (lỗi bus CAN)
    CAN_InitStructure.CAN_AWUM = DISABLE;      // AWUM: Automatic WakeUp Mode. DISABLE: không tự động đánh thức khi có hoạt động trên bus CAN
    CAN_InitStructure.CAN_NART = ENABLE;       // NART: No Automatic Retransmission. ENABLE: không tự động gửi lại nếu gửi thất bại (giúp kiểm soát việc gửi)
    CAN_InitStructure.CAN_RFLM = DISABLE;      // RFLM: Receive FIFO Locked Mode. DISABLE: FIFO không bị khóa, dữ liệu cũ sẽ bị ghi đè nếu đầy
    CAN_InitStructure.CAN_TXFP = DISABLE;      // TXFP: Transmit FIFO Priority. DISABLE: ưu tiên gửi theo thứ tự yêu cầu (không theo độ ưu tiên ID)
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // Chế độ hoạt động CAN. CAN_Mode_Normal: chế độ bình thường. Có thể dùng CAN_Mode_LoopBack để tự kiểm tra
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;   // SJW: Synchronization Jump Width. Độ rộng nhảy đồng bộ, thường chọn 1tq
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;  // BS1: Bit Segment 1. Độ dài bit segment 1, ảnh hưởng đến tốc độ và độ ổn định CAN
    CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;  // BS2: Bit Segment 2. Độ dài bit segment 2, ảnh hưởng đến tốc độ và độ ổn định CAN
    CAN_InitStructure.CAN_Prescaler = 4;      // Prescaler: hệ số chia xung clock cho CAN. Giá trị này quyết định baudrate CAN (ví dụ: 250kbps với APB1=36MHz)

    CAN_Init(CAN1, &CAN_InitStructure);

    // CAN filter configuration
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // Enable FIFO0 message pending interrupt
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/* =============================================================================
 * NVIC CONFIGURATION
 * =============================================================================
 */
void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



/* =============================================================================
 * SIMPLE DELAY FUNCTION
 * =============================================================================
 */
void Delay(uint32_t count)
{
    volatile uint32_t i;
    for (i = 0; i < count; i++);
}
