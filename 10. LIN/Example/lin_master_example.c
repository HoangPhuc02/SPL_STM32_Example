/*
 * =============================================================================
 * Project: STM32F103 LIN Master Example (SPL)
 * File: lin_master_example.c
 * Description: LIN Master node implementation with scheduling and slave management
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
#define LIN_SYNC_BYTE           0x55
#define LIN_MAX_SLAVES          8
#define LIN_SCHEDULE_SIZE       16
#define LIN_FRAME_TIMEOUT_MS    50
#define LIN_SCHEDULE_PERIOD_MS  100

// LIN Frame Types
#define LIN_FRAME_TYPE_MASTER_TX    0
#define LIN_FRAME_TYPE_SLAVE_TX     1
#define LIN_FRAME_TYPE_SLAVE_RX     2

// LIN Frame IDs
#define LIN_ID_MASTER_CMD       0x10
#define LIN_ID_SLAVE1_STATUS    0x20
#define LIN_ID_SLAVE2_STATUS    0x21
#define LIN_ID_SLAVE3_STATUS    0x22
#define LIN_ID_SENSOR_DATA      0x30
#define LIN_ID_ACTUATOR_CMD     0x31
#define LIN_ID_DIAGNOSTIC       0x3C
#define LIN_ID_BROADCAST        0x3F

/* =============================================================================
 * STRUCTURES - Cấu trúc dữ liệu
 * =============================================================================
 */
typedef struct {
    uint8_t id;
    uint8_t type;
    uint8_t data[8];
    uint8_t length;
    uint8_t period_ms;
    uint32_t last_sent;
    uint8_t active;
} LIN_ScheduleEntry_t;

typedef struct {
    uint8_t node_id;
    uint8_t status;
    uint32_t last_response;
    uint8_t error_count;
    uint8_t online;
} LIN_SlaveNode_t;

typedef struct {
    LIN_ScheduleEntry_t schedule[LIN_SCHEDULE_SIZE];
    uint8_t schedule_count;
    uint8_t current_entry;
    uint32_t schedule_timer;
    
    LIN_SlaveNode_t slaves[LIN_MAX_SLAVES];
    uint8_t slave_count;
    
    uint8_t tx_busy;
    uint8_t rx_pending;
    uint32_t frame_timeout;
    
    uint32_t total_frames_sent;
    uint32_t total_responses_received;
    uint32_t total_errors;
} LIN_Master_t;

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
volatile LIN_Master_t lin_master;
volatile uint32_t system_ms = 0;

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void LIN_Master_Init(void);
void LIN_Timer_Init(void);
void LIN_Master_AddScheduleEntry(uint8_t id, uint8_t type, uint8_t *data, 
                                uint8_t length, uint8_t period_ms);
void LIN_Master_AddSlave(uint8_t node_id);
void LIN_Master_SendFrame(uint8_t id, uint8_t *data, uint8_t length);
void LIN_Master_SendHeader(uint8_t id);
void LIN_Master_ProcessSchedule(void);
void LIN_Master_CheckSlaveStatus(void);
void LIN_Master_HandleResponse(uint8_t *data, uint8_t length);
uint8_t LIN_CalculatePID(uint8_t id);
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length);
void LIN_Master_DiagnosticScan(void);

/* =============================================================================
 * MAIN FUNCTION - Chương trình chính
 * =============================================================================
 */
int main(void)
{
    uint8_t master_cmd_data[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t actuator_data[2] = {0x01, 0x00}; // Actuator ID, Command
    
    // Khởi tạo hệ thống
    SystemInit();
    LIN_Master_Init();
    LIN_Timer_Init();
    
    // Khởi tạo LED debug
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    // Cấu hình schedule
    LIN_Master_AddScheduleEntry(LIN_ID_MASTER_CMD, LIN_FRAME_TYPE_MASTER_TX, 
                               master_cmd_data, 4, 200);
    LIN_Master_AddScheduleEntry(LIN_ID_SLAVE1_STATUS, LIN_FRAME_TYPE_SLAVE_TX, 
                               NULL, 4, 100);
    LIN_Master_AddScheduleEntry(LIN_ID_SLAVE2_STATUS, LIN_FRAME_TYPE_SLAVE_TX, 
                               NULL, 2, 150);
    LIN_Master_AddScheduleEntry(LIN_ID_ACTUATOR_CMD, LIN_FRAME_TYPE_MASTER_TX, 
                               actuator_data, 2, 300);
    
    // Thêm slave nodes
    LIN_Master_AddSlave(0x01);
    LIN_Master_AddSlave(0x02);
    LIN_Master_AddSlave(0x03);
    
    // Khởi tạo master
    lin_master.schedule_timer = system_ms;
    lin_master.current_entry = 0;
    
    while(1)
    {
        // Xử lý schedule
        LIN_Master_ProcessSchedule();
        
        // Kiểm tra trạng thái slaves
        static uint32_t last_slave_check = 0;
        if(system_ms - last_slave_check > 1000)
        {
            LIN_Master_CheckSlaveStatus();
            last_slave_check = system_ms;
        }
        
        // Diagnostic scan định kỳ
        static uint32_t last_diagnostic = 0;
        if(system_ms - last_diagnostic > 5000)
        {
            LIN_Master_DiagnosticScan();
            last_diagnostic = system_ms;
        }
        
        // Kiểm tra timeout
        if(lin_master.rx_pending && 
           (system_ms - lin_master.frame_timeout) > LIN_FRAME_TIMEOUT_MS)
        {
            lin_master.rx_pending = 0;
            lin_master.total_errors++;
            
            // Blink LED để báo lỗi
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            for(volatile int i = 0; i < 50000; i++);
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
        }
        
        // Update actuator command
        static uint32_t last_actuator_update = 0;
        if(system_ms - last_actuator_update > 1000)
        {
            actuator_data[1] = (actuator_data[1] == 0x00) ? 0x01 : 0x00;
            last_actuator_update = system_ms;
        }
    }
}

/* =============================================================================
 * INTERRUPT HANDLERS - Xử lý ngắt
 * =============================================================================
 */
void USART1_IRQHandler(void)
{
    static uint8_t rx_buffer[16];
    static uint8_t rx_index = 0;
    static uint8_t expected_length = 0;
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART1);
        
        if(lin_master.rx_pending)
        {
            rx_buffer[rx_index++] = received_byte;
            
            if(rx_index == 1)
            {
                // First byte after header - determine expected length
                // This is simplified - in real implementation, length would be
                // determined from the frame ID
                expected_length = 4; // Assume 4 bytes for this example
            }
            else if(rx_index >= expected_length + 1) // +1 for checksum
            {
                // Frame complete
                LIN_Master_HandleResponse(rx_buffer, expected_length);
                lin_master.rx_pending = 0;
                rx_index = 0;
            }
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        system_ms++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

/* =============================================================================
 * LIN_Master_ProcessSchedule - Xử lý schedule
 * =============================================================================
 */
void LIN_Master_ProcessSchedule(void)
{
    if(lin_master.schedule_count == 0 || lin_master.tx_busy || lin_master.rx_pending)
        return;
    
    LIN_ScheduleEntry_t *entry = &lin_master.schedule[lin_master.current_entry];
    
    if(!entry->active)
    {
        lin_master.current_entry = (lin_master.current_entry + 1) % lin_master.schedule_count;
        return;
    }
    
    if((system_ms - entry->last_sent) >= entry->period_ms)
    {
        switch(entry->type)
        {
            case LIN_FRAME_TYPE_MASTER_TX:
                LIN_Master_SendFrame(entry->id, entry->data, entry->length);
                break;
                
            case LIN_FRAME_TYPE_SLAVE_TX:
                LIN_Master_SendHeader(entry->id);
                lin_master.rx_pending = 1;
                lin_master.frame_timeout = system_ms;
                break;
                
            default:
                break;
        }
        
        entry->last_sent = system_ms;
        lin_master.total_frames_sent++;
        
        // Move to next entry
        lin_master.current_entry = (lin_master.current_entry + 1) % lin_master.schedule_count;
    }
}

/* =============================================================================
 * LIN MASTER FUNCTIONS - Các hàm master
 * =============================================================================
 */
void LIN_Master_Init(void)
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

    // Enable LIN mode
    USART_LINCmd(USART1, ENABLE);
    USART_LINBreakDetectLengthConfig(USART1, USART_LINBreakDetectLength_11b);

    // Configure interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    // Initialize master structure
    lin_master.schedule_count = 0;
    lin_master.slave_count = 0;
    lin_master.tx_busy = 0;
    lin_master.rx_pending = 0;
    lin_master.total_frames_sent = 0;
    lin_master.total_responses_received = 0;
    lin_master.total_errors = 0;
}

void LIN_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
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

void LIN_Master_AddScheduleEntry(uint8_t id, uint8_t type, uint8_t *data,
                                uint8_t length, uint8_t period_ms)
{
    if(lin_master.schedule_count < LIN_SCHEDULE_SIZE)
    {
        LIN_ScheduleEntry_t *entry = &lin_master.schedule[lin_master.schedule_count];

        entry->id = id;
        entry->type = type;
        entry->length = length;
        entry->period_ms = period_ms;
        entry->last_sent = 0;
        entry->active = 1;

        if(data != NULL)
        {
            for(uint8_t i = 0; i < length && i < 8; i++)
            {
                entry->data[i] = data[i];
            }
        }

        lin_master.schedule_count++;
    }
}

void LIN_Master_AddSlave(uint8_t node_id)
{
    if(lin_master.slave_count < LIN_MAX_SLAVES)
    {
        LIN_SlaveNode_t *slave = &lin_master.slaves[lin_master.slave_count];

        slave->node_id = node_id;
        slave->status = 0;
        slave->last_response = 0;
        slave->error_count = 0;
        slave->online = 0;

        lin_master.slave_count++;
    }
}

void LIN_Master_SendFrame(uint8_t id, uint8_t *data, uint8_t length)
{
    uint8_t pid = LIN_CalculatePID(id);
    uint8_t checksum = LIN_CalculateChecksum(pid, data, length);

    lin_master.tx_busy = 1;

    // Send break
    USART_SendBreak(USART1);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Send sync
    USART_SendData(USART1, LIN_SYNC_BYTE);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Send PID
    USART_SendData(USART1, pid);
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

    lin_master.tx_busy = 0;
}

void LIN_Master_SendHeader(uint8_t id)
{
    uint8_t pid = LIN_CalculatePID(id);

    lin_master.tx_busy = 1;

    // Send break
    USART_SendBreak(USART1);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Send sync
    USART_SendData(USART1, LIN_SYNC_BYTE);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Send PID
    USART_SendData(USART1, pid);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    lin_master.tx_busy = 0;
}

/* =============================================================================
 * UTILITY FUNCTIONS - Các hàm tiện ích
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

void LIN_Master_HandleResponse(uint8_t *data, uint8_t length)
{
    lin_master.total_responses_received++;

    // Process response based on current frame ID
    // This is simplified - in real implementation, you would track
    // which frame ID was sent and process accordingly

    // Example: Update slave status
    for(uint8_t i = 0; i < lin_master.slave_count; i++)
    {
        lin_master.slaves[i].last_response = system_ms;
        lin_master.slaves[i].online = 1;
        lin_master.slaves[i].error_count = 0;
    }
}

void LIN_Master_CheckSlaveStatus(void)
{
    for(uint8_t i = 0; i < lin_master.slave_count; i++)
    {
        LIN_SlaveNode_t *slave = &lin_master.slaves[i];

        // Check if slave has responded recently
        if((system_ms - slave->last_response) > 2000) // 2 seconds timeout
        {
            if(slave->online)
            {
                slave->online = 0;
                slave->error_count++;
                lin_master.total_errors++;
            }
        }
    }
}

void LIN_Master_DiagnosticScan(void)
{
    // Send diagnostic frame to check network health
    uint8_t diagnostic_data[1] = {0x01}; // Diagnostic command
    LIN_Master_SendFrame(LIN_ID_DIAGNOSTIC, diagnostic_data, 1);
}
