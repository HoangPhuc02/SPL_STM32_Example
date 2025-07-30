/*
 * =============================================================================
 * Project: STM32F103 LIN Example (SPL)
 * File: basic_transmit.c
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
void LIN_Init(void);            // Khởi tạo LIN interface
void LIN_SendBreak(void);       // Gửi tín hiệu break
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t length); // Gửi frame LIN
uint8_t LIN_CheckPID(uint8_t pid); // Tính toán PID với parity

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */

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
    uint8_t data[2] = {0x55, 0xAA};
    
    // Khởi tạo hệ thống và LIN
    SystemInit();               // Cấu hình clock hệ thống
    LIN_Init();                // Khởi tạo LIN interface
    
    // Vòng lặp chính
    while(1)
    {
        // Gửi frame LIN với ID 0x30 và 2 byte dữ liệu
        LIN_SendFrame(0x30, data, 2);
        
        // Tạo độ trễ giữa các frame
        for(int i = 0; i < 1000000; i++);
    }
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
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    // Bật clock cho GPIOA và USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    // Cấu hình chân TX (PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Cấu hình chân RX (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Cấu hình USART1 cho LIN
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

/* =============================================================================
 * LIN_SendBreak - Gửi tín hiệu break LIN
 * =============================================================================
 * Chức năng:
 * - Tạo tín hiệu break LIN (13 bit 0)
 * - Gửi byte đồng bộ (0x55)
 * Các bước thực hiện:
 * 1. Tắt USART
 * 2. Cấu hình TX pin thành GPIO output
 * 3. Gửi break (13 bit 0)
 * 4. Khôi phục TX pin về chế độ USART
 * 5. Gửi byte đồng bộ
 */
void LIN_SendBreak(void)
{
    // Tắt USART tạm thời
    USART_Cmd(USART1, DISABLE);
    
    // Cấu hình TX pin thành GPIO output
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Gửi break (kéo xuống mức thấp)
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
    
    // Duy trì break trong 13 bit time
    // Tại 9600 baud: 1/9600 * 13 ≈ 1.354ms
    for(int i = 0; i < LIN_BREAK_DELAY; i++)
    {
        USART_SendData(USART1, 0x00);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    
    // Khôi phục TX pin về chế độ USART
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Bật lại USART
    USART_Cmd(USART1, ENABLE);
    
    // Gửi byte đồng bộ (0x55)
    USART_SendData(USART1, 0x55);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/* =============================================================================
 * LIN_SendFrame - Gửi một frame LIN hoàn chỉnh
 * =============================================================================
 * Tham số:
 * - pid: ID của frame (6 bit)
 * - data: Con trỏ đến mảng dữ liệu cần gửi
 * - length: Độ dài dữ liệu
 * 
 * Chức năng:
 * 1. Gửi break và sync
 * 2. Tính toán PID với parity
 * 3. Gửi PID
 * 4. Gửi dữ liệu
 * 5. Tính và gửi checksum
 */
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0;
    
    // Gửi break và sync
    LIN_SendBreak();
    
    // Tính PID với parity
    pid = LIN_CheckPID(pid);
    
    // Gửi PID
    USART_SendData(USART1, pid);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    
    // Tính checksum (LIN 2.0)
    checksum = pid;
    for(int i = 0; i < length; i++)
    {
        checksum += data[i];
        if(checksum > 255)
            checksum -= 255;
    }
    checksum = ~checksum; // Đảo bit
    
    // Gửi dữ liệu
    for(int i = 0; i < length; i++)
    {
        USART_SendData(USART1, data[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    
    // Gửi checksum
    USART_SendData(USART1, checksum);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
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
