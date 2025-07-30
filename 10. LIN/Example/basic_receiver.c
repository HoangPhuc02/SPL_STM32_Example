/*
 * =============================================================================
 * Project: STM32F103 LIN Example (SPL)
 * File: basic_receiver.c
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

/* =============================================================================
 * DEFINES - Các định nghĩa và hằng số
 * =============================================================================
 */
// Định nghĩa các thông số LIN
#define LIN_BREAK_DELAY      13      // Số bit thời gian cho break detection
#define LIN_MAX_FRAME_SIZE   10      // Kích thước tối đa frame: Sync + PID + 8 data + checksum
#define LIN_TIMEOUT          1000    // Thời gian timeout cho việc nhận frame

/* =============================================================================
 * STRUCTURES - Cấu trúc dữ liệu
 * =============================================================================
 */
// Cấu trúc frame LIN
typedef struct {
    uint8_t sync;                    // Byte đồng bộ (0x55)
    uint8_t pid;                     // ID gói tin với parity
    uint8_t data[8];                 // Dữ liệu (tối đa 8 byte)
    uint8_t length;                  // Độ dài dữ liệu thực tế
    uint8_t checksum;                // Checksum kiểm tra
    uint8_t status;                  // Trạng thái frame: 0-idle, 1-receiving, 2-complete, 3-error
} LIN_Frame;

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
volatile LIN_Frame lin_rx_frame;     // Frame LIN đang xử lý
volatile uint8_t lin_rx_buffer[LIN_MAX_FRAME_SIZE];  // Buffer lưu dữ liệu tạm
volatile uint8_t lin_rx_index = 0;   // Chỉ số hiện tại trong buffer
volatile uint32_t lin_rx_timer = 0;  // Timer cho timeout

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void LIN_Init(void);                 // Khởi tạo LIN interface
void LIN_SendBreak(void);           // Gửi tín hiệu break
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t length); // Gửi frame
uint8_t LIN_CheckPID(uint8_t pid);  // Kiểm tra PID
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length); // Tính checksum
void LIN_ReceiveHandler(void);      // Xử lý nhận dữ liệu
uint8_t LIN_CheckFrame(LIN_Frame *frame); // Kiểm tra frame

/* =============================================================================
 * MAIN FUNCTION - Chương trình chính
 * =============================================================================
 */
int main(void)
{
    // Khởi tạo mảng dữ liệu mẫu để gửi
    uint8_t data[2] = {0x55, 0xAA};
    
    // Khởi tạo hệ thống và LIN
    SystemInit();                    // Cấu hình clock hệ thống
    LIN_Init();                     // Khởi tạo LIN interface
    
    // Cấu hình ngắt USART1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    
    // Vòng lặp chính
    while(1)
    {
        // Kiểm tra nếu nhận được frame hoàn chỉnh
        if(lin_rx_frame.status == 2) // Frame hoàn thành
        {
            if(LIN_CheckFrame(&lin_rx_frame))
            {
                // Frame hợp lệ - xử lý tại đây
                // Ví dụ: Điều khiển LED dựa trên dữ liệu nhận được
                if(lin_rx_frame.pid == 0x30)
                {
                    GPIO_WriteBit(GPIOC, GPIO_Pin_13, 
                                (lin_rx_frame.data[0] == 0x55) ? Bit_RESET : Bit_SET);
                }
            }
            
            // Reset trạng thái frame
            lin_rx_frame.status = 0;
        }
        
        // Gửi frame LIN định kỳ
        static uint32_t last_send = 0;
        if(SystemCoreClock - last_send > 1000000)
        {
            LIN_SendFrame(0x30, data, 2);
            last_send = SystemCoreClock;
            
            // Đảo bit dữ liệu cho lần gửi tiếp theo
            data[0] ^= 0xFF;
            data[1] ^= 0xFF;
        }
    }
}

/* =============================================================================
 * INTERRUPT HANDLER - Xử lý ngắt USART1
 * =============================================================================
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received = USART_ReceiveData(USART1);
        
        // Reset timer timeout
        lin_rx_timer = SystemCoreClock;
        
        // Kiểm tra break (0x00) - bắt đầu frame
        if(received == 0x00 && lin_rx_frame.status == 0)
        {
            lin_rx_frame.status = 1; // Bắt đầu nhận
            lin_rx_index = 0;
            lin_rx_buffer[lin_rx_index++] = received;
        }
        else if(lin_rx_frame.status == 1) // Đang nhận frame
        {
            lin_rx_buffer[lin_rx_index++] = received;
            
            // Kiểm tra frame hoàn chỉnh (tối thiểu: sync + pid + checksum)
            if(lin_rx_index >= 3)
            {
                // Kiểm tra đã nhận đủ byte dữ liệu
                uint8_t expected_length = 3; // sync + pid + checksum
                if(lin_rx_index >= expected_length)
                {
                    // Phân tích frame nhận được
                    lin_rx_frame.sync = lin_rx_buffer[0];
                    lin_rx_frame.pid = lin_rx_buffer[1];
                    
                    // Độ dài dữ liệu phụ thuộc ứng dụng (giả sử 2 byte)
                    lin_rx_frame.length = 2;
                    lin_rx_frame.data[0] = lin_rx_buffer[2];
                    lin_rx_frame.data[1] = lin_rx_buffer[3];
                    lin_rx_frame.checksum = lin_rx_buffer[4];
                    
                    lin_rx_frame.status = 2; // Frame hoàn thành
                }
            }
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/* =============================================================================
 * LIN_CheckFrame - Kiểm tra tính hợp lệ của frame
 * =============================================================================
 * Tham số:
 * - frame: Con trỏ đến frame cần kiểm tra
 * 
 * Trả về:
 * - 1: Frame hợp lệ
 * - 0: Frame không hợp lệ
 */
uint8_t LIN_CheckFrame(LIN_Frame *frame)
{
    // Kiểm tra byte sync
    if(frame->sync != 0x55)
        return 0;
    
    // Kiểm tra parity của PID
    uint8_t pid = LIN_CheckPID(frame->pid & 0x3F);
    if(pid != frame->pid)
        return 0;
    
    // Kiểm tra checksum
    uint8_t calc_checksum = LIN_CalculateChecksum(frame->pid, frame->data, frame->length);
    if(calc_checksum != frame->checksum)
        return 0;
    
    return 1; // Frame hợp lệ
}

/* =============================================================================
 * LIN_CalculateChecksum - Tính checksum cho frame LIN
 * =============================================================================
 * Tham số:
 * - pid: ID của frame
 * - data: Con trỏ đến mảng dữ liệu
 * - length: Độ dài dữ liệu
 * 
 * Trả về:
 * - Giá trị checksum đã tính toán
 */
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length)
{
    uint16_t checksum = pid;
    
    // Tính checksum theo chuẩn LIN 2.0 (PID + data)
    for(int i = 0; i < length; i++)
    {
        checksum += data[i];
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }
    
    return ~checksum; // Đảo bit kết quả
}

/* =============================================================================
 * LIN_Init - Khởi tạo giao tiếp LIN
 * =============================================================================
 * Chức năng:
 * - Cấu hình GPIO cho USART1 (TX: PA9, RX: PA10)
 * - Cấu hình LED debug (PC13)
 * - Cấu hình USART1 cho giao tiếp LIN
 * - Cấu hình ngắt nhận USART1
 */
void LIN_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // Bật clock cho các ngoại vi
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    // Cấu hình LED trên PC13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    // Cấu hình USART1 Tx (PA9) và Rx (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Cấu hình USART1 cho LIN
    USART_InitStructure.USART_BaudRate = 9600;  // Tốc độ chuẩn LIN
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure);
    
    // Cấu hình ngắt USART1 RX
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
}
