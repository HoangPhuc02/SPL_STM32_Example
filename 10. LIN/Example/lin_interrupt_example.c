/*
 * =============================================================================
 * Project: STM32F103 LIN Interrupt Example (SPL)
 * File: lin_interrupt_example.c
 * Description: LIN communication with interrupt-based frame handling
 * Author: LIN Driver Team
 * Date: 01/08/2025
 * Author      : hoangphuc540202@gmail.com
 * Github      : https://github.com/HoangPhuc02
 * =============================================================================
 */

/* =============================================================================
 * INCLUDES - Khai báo các thư viện cần thiết
 * =============================================================================
 */
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "system_stm32f10x.h"

/* =============================================================================
 * DEFINES - Các định nghĩa và hằng số
 * =============================================================================
 */
#define LIN_BREAK_DELAY         13
#define LIN_MAX_FRAME_SIZE      10
#define LIN_TIMEOUT_MS          100
#define LIN_SYNC_BYTE           0x55
#define LIN_BREAK_BYTE          0x00

// LIN Frame States
#define LIN_STATE_IDLE          0
#define LIN_STATE_BREAK         1
#define LIN_STATE_SYNC          2
#define LIN_STATE_PID           3
#define LIN_STATE_DATA          4
#define LIN_STATE_CHECKSUM      5
#define LIN_STATE_COMPLETE      6
#define LIN_STATE_ERROR         7

/* =============================================================================
 * STRUCTURES - Cấu trúc dữ liệu
 * =============================================================================
 */
typedef struct {
    uint8_t sync;
    uint8_t pid;
    uint8_t data[8];
    uint8_t length;
    uint8_t checksum;
    uint8_t state;
    uint8_t error_code;
    uint32_t timestamp;
} LIN_Frame_t;

typedef struct {
    LIN_Frame_t rx_frame;
    LIN_Frame_t tx_frame;
    uint8_t rx_buffer[LIN_MAX_FRAME_SIZE];
    uint8_t rx_index;
    uint8_t expected_length;
    uint32_t timeout_counter;
    uint8_t frame_ready;
    uint8_t error_flag;
} LIN_Handler_t;

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
volatile LIN_Handler_t lin_handler;
volatile uint32_t system_tick = 0;

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void LIN_Init(void);
void LIN_Timer_Init(void);
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t length);
void LIN_ProcessRxByte(uint8_t byte);
uint8_t LIN_CalculatePID(uint8_t id);
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length);
uint8_t LIN_ValidateFrame(LIN_Frame_t *frame);
void LIN_ResetRx(void);
void LIN_HandleTimeout(void);
void LIN_ProcessFrame(LIN_Frame_t *frame);

/* =============================================================================
 * MAIN FUNCTION - Chương trình chính
 * =============================================================================
 */
int main(void)
{
    uint8_t test_data[4] = {0x01, 0x02, 0x03, 0x04};
    uint32_t last_send = 0;
    
    // Khởi tạo hệ thống
    SystemInit();
    LIN_Init();
    LIN_Timer_Init();
    
    // Khởi tạo LED debug
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    while(1)
    {
        // Kiểm tra frame nhận được
        if(lin_handler.frame_ready)
        {
            LIN_ProcessFrame(&lin_handler.rx_frame);
            lin_handler.frame_ready = 0;
        }
        
        // Kiểm tra lỗi
        if(lin_handler.error_flag)
        {
            // Xử lý lỗi - nhấp nháy LED
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            for(volatile int i = 0; i < 100000; i++);
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            
            lin_handler.error_flag = 0;
            LIN_ResetRx();
        }
        
        // Gửi frame định kỳ (mỗi 500ms)
        if(system_tick - last_send > 500)
        {
            LIN_SendFrame(0x30, test_data, 4);
            last_send = system_tick;
            
            // Thay đổi dữ liệu cho lần gửi tiếp theo
            test_data[0]++;
        }
        
        // Kiểm tra timeout
        LIN_HandleTimeout();
    }
}

/* =============================================================================
 * INTERRUPT HANDLERS - Xử lý ngắt
 * =============================================================================
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART1);
        LIN_ProcessRxByte(received_byte);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    
    if(USART_GetITStatus(USART1, USART_IT_LBD) != RESET)
    {
        // Break detected
        LIN_ResetRx();
        lin_handler.rx_frame.state = LIN_STATE_BREAK;
        lin_handler.timeout_counter = system_tick;
        USART_ClearITPendingBit(USART1, USART_IT_LBD);
    }
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        system_tick++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

/* =============================================================================
 * LIN_ProcessRxByte - Xử lý byte nhận được
 * =============================================================================
 */
void LIN_ProcessRxByte(uint8_t byte)
{
    lin_handler.timeout_counter = system_tick;
    
    switch(lin_handler.rx_frame.state)
    {
        case LIN_STATE_BREAK:
            if(byte == LIN_SYNC_BYTE)
            {
                lin_handler.rx_frame.sync = byte;
                lin_handler.rx_frame.state = LIN_STATE_SYNC;
                lin_handler.rx_index = 0;
            }
            else
            {
                lin_handler.rx_frame.state = LIN_STATE_ERROR;
                lin_handler.rx_frame.error_code = 1; // Sync error
            }
            break;
            
        case LIN_STATE_SYNC:
            lin_handler.rx_frame.pid = byte;
            lin_handler.rx_frame.state = LIN_STATE_PID;
            
            // Xác định độ dài dữ liệu dựa trên PID
            uint8_t id = byte & 0x3F;
            if(id <= 0x1F) lin_handler.expected_length = 2;
            else if(id <= 0x2F) lin_handler.expected_length = 4;
            else lin_handler.expected_length = 8;
            break;
            
        case LIN_STATE_PID:
            if(lin_handler.rx_index < lin_handler.expected_length)
            {
                lin_handler.rx_frame.data[lin_handler.rx_index++] = byte;
                if(lin_handler.rx_index >= lin_handler.expected_length)
                {
                    lin_handler.rx_frame.length = lin_handler.expected_length;
                    lin_handler.rx_frame.state = LIN_STATE_DATA;
                }
            }
            break;
            
        case LIN_STATE_DATA:
            lin_handler.rx_frame.checksum = byte;
            lin_handler.rx_frame.state = LIN_STATE_CHECKSUM;
            
            // Validate frame
            if(LIN_ValidateFrame(&lin_handler.rx_frame))
            {
                lin_handler.rx_frame.state = LIN_STATE_COMPLETE;
                lin_handler.rx_frame.timestamp = system_tick;
                lin_handler.frame_ready = 1;
            }
            else
            {
                lin_handler.rx_frame.state = LIN_STATE_ERROR;
                lin_handler.rx_frame.error_code = 2; // Checksum error
                lin_handler.error_flag = 1;
            }
            break;
            
        default:
            LIN_ResetRx();
            break;
    }
}

/* =============================================================================
 * LIN_ProcessFrame - Xử lý frame hoàn chỉnh
 * =============================================================================
 */
void LIN_ProcessFrame(LIN_Frame_t *frame)
{
    uint8_t id = frame->pid & 0x3F;
    
    switch(id)
    {
        case 0x30: // LED control frame
            if(frame->length >= 1)
            {
                if(frame->data[0] & 0x01)
                    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // LED ON
                else
                    GPIO_SetBits(GPIOC, GPIO_Pin_13);   // LED OFF
            }
            break;
            
        case 0x31: // Status request frame
            // Respond with status data
            uint8_t status_data[2] = {0xAA, 0x55};
            LIN_SendFrame(0x32, status_data, 2);
            break;
            
        default:
            // Unknown frame ID
            break;
    }
}

/* =============================================================================
 * LIN_Init - Khởi tạo LIN interface
 * =============================================================================
 */
void LIN_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO |
                          RCC_APB2Periph_USART1, ENABLE);

    // Configure USART1 pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART1
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // Enable LIN mode and break detection
    USART_LINCmd(USART1, ENABLE);
    USART_LINBreakDetectLengthConfig(USART1, USART_LINBreakDetectLength_11b);

    // Configure interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable interrupts
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_LBD, ENABLE);

    USART_Cmd(USART1, ENABLE);

    // Initialize handler
    LIN_ResetRx();
}

/* =============================================================================
 * LIN_Timer_Init - Khởi tạo timer cho system tick
 * =============================================================================
 */
void LIN_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Configure timer for 1ms tick
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/* =============================================================================
 * LIN_SendFrame - Gửi frame LIN
 * =============================================================================
 */
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t length)
{
    uint8_t calculated_pid = LIN_CalculatePID(pid);
    uint8_t checksum = LIN_CalculateChecksum(calculated_pid, data, length);

    // Send break
    USART_SendBreak(USART1);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Send sync byte
    USART_SendData(USART1, LIN_SYNC_BYTE);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Send PID
    USART_SendData(USART1, calculated_pid);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Send data
    for(uint8_t i = 0; i < length; i++)
    {
        USART_SendData(USART1, data[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }

    // Send checksum
    USART_SendData(USART1, checksum);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/* =============================================================================
 * LIN_CalculatePID - Tính PID với parity bits
 * =============================================================================
 */
uint8_t LIN_CalculatePID(uint8_t id)
{
    uint8_t p0, p1;
    uint8_t masked_id = id & 0x3F;

    p0 = ((masked_id >> 0) & 1) ^ ((masked_id >> 1) & 1) ^
         ((masked_id >> 2) & 1) ^ ((masked_id >> 4) & 1);

    p1 = !(((masked_id >> 1) & 1) ^ ((masked_id >> 3) & 1) ^
           ((masked_id >> 4) & 1) ^ ((masked_id >> 5) & 1));

    return masked_id | (p0 << 6) | (p1 << 7);
}

/* =============================================================================
 * LIN_CalculateChecksum - Tính checksum
 * =============================================================================
 */
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length)
{
    uint16_t checksum = pid;

    for(uint8_t i = 0; i < length; i++)
    {
        checksum += data[i];
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }

    return ~checksum;
}

/* =============================================================================
 * LIN_ValidateFrame - Kiểm tra tính hợp lệ của frame
 * =============================================================================
 */
uint8_t LIN_ValidateFrame(LIN_Frame_t *frame)
{
    // Check sync byte
    if(frame->sync != LIN_SYNC_BYTE)
        return 0;

    // Check PID parity
    uint8_t expected_pid = LIN_CalculatePID(frame->pid & 0x3F);
    if(expected_pid != frame->pid)
        return 0;

    // Check checksum
    uint8_t expected_checksum = LIN_CalculateChecksum(frame->pid, frame->data, frame->length);
    if(expected_checksum != frame->checksum)
        return 0;

    return 1;
}

/* =============================================================================
 * LIN_ResetRx - Reset receiver state
 * =============================================================================
 */
void LIN_ResetRx(void)
{
    lin_handler.rx_frame.state = LIN_STATE_IDLE;
    lin_handler.rx_index = 0;
    lin_handler.expected_length = 0;
    lin_handler.frame_ready = 0;
    lin_handler.rx_frame.error_code = 0;
}

/* =============================================================================
 * LIN_HandleTimeout - Xử lý timeout
 * =============================================================================
 */
void LIN_HandleTimeout(void)
{
    if(lin_handler.rx_frame.state != LIN_STATE_IDLE &&
       lin_handler.rx_frame.state != LIN_STATE_COMPLETE)
    {
        if(system_tick - lin_handler.timeout_counter > LIN_TIMEOUT_MS)
        {
            lin_handler.rx_frame.state = LIN_STATE_ERROR;
            lin_handler.rx_frame.error_code = 3; // Timeout error
            lin_handler.error_flag = 1;
        }
    }
}
