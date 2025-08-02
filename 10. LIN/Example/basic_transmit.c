/*
 * =============================================================================
 * Project: STM32F103 LIN Example (SPL)
 * File: main.c
 * Description: Giao tiếp LIN cơ bản sử dụng SPL (Standard Peripheral Library)
 * Author: LIN Driver Team
 * Date: 30/07/2025
 * Author      : hoangphuc540202@gmail.com
 * Github      : https://github.com/HoangPhuc02
 * =============================================================================
 */

/* =============================================================================
 * INCLUDES - Khai báo các thư viện cần thiết
 * =============================================================================
 */
#include "stm32f10x.h"          // Thư viện chính cho STM32F10x
#include "stm32f10x_gpio.h"     // Điều khiển GPIO
#include "stm32f10x_rcc.h"      // Quản lý clock
#include "stm32f10x_usart.h"    // Giao tiếp UART/USART
#include "system_stm32f10x.h"   // Cấu hình hệ thống

/* =============================================================================
 * MACROS - Các định nghĩa và hằng số
 * =============================================================================
 */
#define LIN_BREAK_DELAY 13      // Số bit thời gian cho break detection trong LIN

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void GPIO_Config(void);           // Khởi tạo GPIO interface
void LIN_Init(void);            // Khởi tạo LIN interface
uint8_t LIN_CheckSum(uint8_t pid, uint8_t *data, uint8_t data_length);       // tính checksum
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t data_length); // Gửi frame LIN
uint8_t LIN_CheckPID(uint8_t pid); // Tính toán PID với parity

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
// uint8_t txData[11] = {0};
/* =============================================================================
 * HÀM MAIN - Chương trình chính
 * =============================================================================
 * Chương trình demo giao tiếp LIN thực hiện các chức năng:
 * 1. Khởi tạo hệ thống (SystemInit)
 * 2. Khởi tạo giao tiếp LIN (LIN_Init)
 * 3. Vòng lặp chính:
 *    - Gửi frame LIN với ID 0x30 và 2 byte dữ liệu
 *    - Tạo độ trễ giữa các frame
 */
int main(void)
{
    // Khởi tạo mảng dữ liệu mẫu
    uint8_t data[8] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
    
    // Khởi tạo hệ thống và LIN
    SystemInit();               // Cấu hình clock hệ thống
    GPIO_Config();
    LIN_Init();                // Khởi tạo LIN interface
    
    // Vòng lặp chính
    while(1)
    {
        // Gửi frame LIN với ID 0x30 và 2 byte dữ liệu
        LIN_SendFrame(0x30, data, 8);
        GPIOC->ODR ^= 1<<13;
        // Tạo độ trễ giữa các frame
        for(int i = 0; i < 1000000; i++);
    }
}

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Bật clock cho GPIOA và USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
    
    // Cấu hình chân TX (PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Cấu hình chân RX (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Cấu hình chân Led (PA13)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/* =============================================================================
 * LIN_Init - Khởi tạo giao tiếp LIN
 * =============================================================================
 * Chức năng:
 * - Cấu hình GPIO cho USART1 (TX: PA9, RX: PA10)
 * - Cấu hình USART1 cho giao tiếp LIN:
 *   + Baud rate: 9600 (chuẩn LIN)
 *   + 8 bit data, 1 stop bit, không parity
 *   + Không sử dụng flow control
 */
void LIN_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    
    // Bật clock cho GPIOA và USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    // Cấu hình USART1 cho LIN
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure);
    // USART_LINCmd(USART1, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

/* =============================================================================
 * LIN_SendFrame - Gửi một frame LIN hoàn chỉnh
 * =============================================================================
 * Tham số:
 * - pid: ID của frame (6 bit)
 * - data: Con trỏ đến mảng dữ liệu cần gửi
 * - data_length: Độ dài dữ liệu
 * 
 * Chức năng:
 * 1. Gửi break và sync
 * 2. Tính toán PID với parity
 * 3. Gửi PID
 * 4. Gửi dữ liệu
 * 5. Tính và gửi checksum
 */
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t data_length)
{
    const uint8_t frame_length = 3 + data_length;
    uint8_t txData[frame_length];
    txData[0] = 0x55;
    // Tính PID với parity
    txData[1] = LIN_CheckPID(pid);

    // Gửi dữ liệu
    for(int i = 0; i < data_length; i++)
    {
        txData[i+2] = data[i];
    }
    // CRC
    txData[frame_length - 1] = LIN_CheckSum(pid, data, data_length);

    // Gửi break và sync
    USART_SendBreak(USART1);
    for(int i = 0; i < frame_length; i++)
    {
        USART_SendData(USART1, txData[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }

}

/* =============================================================================
 * LIN_CheckPID - Tính toán PID với parity
 * =============================================================================
 * Tham số:
 * - pid: ID gốc (6 bit)
 * 
 * Trả về:
 * - PID 8 bit với 2 bit parity
 * 
 * Chức năng:
 * - Tính toán 2 bit parity cho PID theo chuẩn LIN
 * - P0 = ID0 ⊕ ID1 ⊕ ID2 ⊕ ID4
 * - P1 = !(ID1 ⊕ ID3 ⊕ ID4 ⊕ ID5)
 */
uint8_t LIN_CheckPID(uint8_t pid)
{
    uint8_t p0 = 0, p1 = 0;
    uint8_t masked_pid = pid & 0x3F; // Lấy 6 bit thấp
    
    // Tính bit parity P0
    p0 = ((masked_pid >> 0) & 1) ^ ((masked_pid >> 1) & 1) ^ 
         ((masked_pid >> 2) & 1) ^ ((masked_pid >> 4) & 1);
    
    // Tính bit parity P1
    p1 = !(((masked_pid >> 1) & 1) ^ ((masked_pid >> 3) & 1) ^ 
           ((masked_pid >> 4) & 1) ^ ((masked_pid >> 5) & 1));
    
    // Kết hợp PID với parity
    return masked_pid | (p0 << 6) | (p1 << 7);
}


uint8_t LIN_CheckSum(uint8_t pid, uint8_t *data, uint8_t data_length)
{
    // Tính checksum (LIN 2.0)
    uint8_t checksum = pid;
    for(int i = 0; i < data_length; i++)
    {
        checksum += data[i];
        if(checksum > 255)
            checksum -= 255;
    }
    return ~checksum; // Đảo bit
}