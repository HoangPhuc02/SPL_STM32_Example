/*
 * =============================================================================
 * Project: STM32F103 CAN Example (SPL)
 * File: main.c
 * Description: Basic CAN communication using SPL (Standard Peripheral Library)
 * Author: CAN Driver Team
 * Date: July 2025
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "misc.h"

/* =============================================================================
 * FUNCTION PROTOTYPES
 * =============================================================================
 */
void CAN_GPIO_Config(void);
void CAN_Config(void);
void NVIC_Config(void);
void Delay(uint32_t count);
void SystemClock_Config(void);

/* =============================================================================
 * GLOBAL VARIABLES
 * =============================================================================
 */
CanRxMsg RxMessage;
CanTxMsg TxMessage;
volatile uint8_t CAN_Received = 0;

/* =============================================================================
 * HÀM MAIN
 * =============================================================================
 * Đây là điểm bắt đầu của chương trình. Các bước thực hiện:
 * 1. Cấu hình xung nhịp hệ thống lên 72MHz (SystemClock_Config)
 * 2. Cấu hình các chân GPIO cho CAN và LED (CAN_GPIO_Config)
 * 3. Cấu hình module CAN (CAN_Config)
 * 4. Cấu hình ngắt cho CAN (NVIC_Config)
 * 5. Chuẩn bị dữ liệu gửi CAN (TxMessage)
 * 6. Vòng lặp chính: gửi dữ liệu CAN, chờ nhận phản hồi, kiểm tra dữ liệu nhận và nhấp nháy LED
 */
int main(void)
{
    SystemClock_Config(); // 1. Thiết lập xung nhịp hệ thống 72MHz
    CAN_GPIO_Config();    // 2. Cấu hình chân CAN và LED
    CAN_Config();         // 3. Cấu hình module CAN
    NVIC_Config();        // 4. Cấu hình ngắt cho CAN

    // 5. Chuẩn bị dữ liệu gửi CAN
    TxMessage.StdId = 0x321;           // Địa chỉ ID chuẩn (Standard ID) cho khung CAN
    TxMessage.ExtId = 0x00;            // Không dùng ID mở rộng
    TxMessage.IDE = CAN_Id_Standard;   // Chọn kiểu ID chuẩn
    TxMessage.RTR = CAN_RTR_Data;      // Khung dữ liệu (Data frame)
    TxMessage.DLC = 1;                 // Độ dài dữ liệu: 1 byte
    TxMessage.Data[0] = 0xAB;          // Dữ liệu gửi: 0xAB

    while (1)
    {
        // 6. Gửi khung CAN
        CAN_Transmit(CAN1, &TxMessage);

        // Chờ đến khi nhận được dữ liệu CAN (biến CAN_Received được set trong ngắt)
        while (!CAN_Received);

        // Kiểm tra dữ liệu nhận được
        if (RxMessage.StdId == 0x321 && RxMessage.Data[0] == 0xAB)
        {
            // Nếu đúng dữ liệu, nhấp nháy LED trên chân PC13
            GPIO_SetBits(GPIOC, GPIO_Pin_13); // Bật LED
            Delay(1000000);                    // Đợi
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);// Tắt LED
            Delay(1000000);                    // Đợi
        }

        CAN_Received = 0; // Reset trạng thái nhận để tiếp tục vòng lặp
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

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // Use CAN_Mode_LoopBack for self-test
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
    CAN_InitStructure.CAN_Prescaler = 4; // Set baudrate (e.g., 250kbps for 36MHz APB1)

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
 * CAN RX INTERRUPT HANDLER
 * =============================================================================
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    CAN_Received = 1;
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
