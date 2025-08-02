/*
 * =============================================================================
 * Project: STM32F103 LIN Diagnostic Example (SPL)
 * File: lin_diagnostic_example.c
 * Description: LIN network diagnostic and monitoring tool
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
#define LIN_SYNC_BYTE               0x55
#define LIN_DIAGNOSTIC_TIMEOUT      1000
#define LIN_MAX_NODES               16
#define LIN_DIAGNOSTIC_BUFFER_SIZE  32

// Diagnostic commands
#define LIN_DIAG_READ_BY_ID         0x22
#define LIN_DIAG_SESSION_CONTROL    0x10
#define LIN_DIAG_TESTER_PRESENT     0x3E
#define LIN_DIAG_READ_DTC           0x19
#define LIN_DIAG_CLEAR_DTC          0x14

// Diagnostic frame IDs
#define LIN_DIAG_MASTER_REQ         0x3C
#define LIN_DIAG_SLAVE_RESP         0x3D

// Node status
#define NODE_STATUS_UNKNOWN         0
#define NODE_STATUS_ONLINE          1
#define NODE_STATUS_OFFLINE         2
#define NODE_STATUS_ERROR           3

/* =============================================================================
 * STRUCTURES - Cấu trúc dữ liệu
 * =============================================================================
 */
typedef struct {
    uint8_t node_id;
    uint8_t status;
    uint32_t last_response;
    uint16_t response_time_ms;
    uint32_t total_requests;
    uint32_t successful_responses;
    uint32_t error_count;
    uint8_t software_version[3];
    uint8_t hardware_version[3];
    uint16_t diagnostic_trouble_codes[8];
    uint8_t dtc_count;
} LIN_NodeInfo_t;

typedef struct {
    uint8_t command;
    uint8_t data[7];
    uint8_t length;
    uint32_t timestamp;
    uint8_t target_node;
} LIN_DiagRequest_t;

typedef struct {
    LIN_NodeInfo_t nodes[LIN_MAX_NODES];
    uint8_t node_count;
    uint8_t current_scan_node;
    uint8_t scan_active;
    uint32_t scan_start_time;
    
    LIN_DiagRequest_t request_buffer[LIN_DIAGNOSTIC_BUFFER_SIZE];
    uint8_t request_write_index;
    uint8_t request_read_index;
    uint8_t request_count;
    
    uint8_t waiting_for_response;
    uint32_t request_timestamp;
    uint8_t current_request_node;
    
    uint32_t total_scans;
    uint32_t successful_scans;
    uint32_t network_errors;
} LIN_Diagnostic_t;

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
volatile LIN_Diagnostic_t lin_diag;
volatile uint32_t system_ms = 0;

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void LIN_Diagnostic_Init(void);
void LIN_Timer_Init(void);
void LIN_Diagnostic_StartScan(void);
void LIN_Diagnostic_ProcessScan(void);
void LIN_Diagnostic_SendRequest(uint8_t node_id, uint8_t command, uint8_t *data, uint8_t length);
void LIN_Diagnostic_ProcessResponse(uint8_t *data, uint8_t length);
void LIN_Diagnostic_AddNode(uint8_t node_id);
LIN_NodeInfo_t* LIN_Diagnostic_FindNode(uint8_t node_id);
void LIN_Diagnostic_UpdateNodeStatus(uint8_t node_id, uint8_t status);
void LIN_Diagnostic_ReadNodeInfo(uint8_t node_id);
void LIN_Diagnostic_ReadDTC(uint8_t node_id);
void LIN_Diagnostic_ClearDTC(uint8_t node_id);
uint8_t LIN_CalculatePID(uint8_t id);
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length);

/* =============================================================================
 * MAIN FUNCTION - Chương trình chính
 * =============================================================================
 */
int main(void)
{
    // Khởi tạo hệ thống
    SystemInit();
    LIN_Diagnostic_Init();
    LIN_Timer_Init();
    
    // Khởi tạo LED status
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    
    // Khởi tạo diagnostic
    lin_diag.node_count = 0;
    lin_diag.scan_active = 0;
    lin_diag.waiting_for_response = 0;
    lin_diag.request_count = 0;
    lin_diag.total_scans = 0;
    lin_diag.successful_scans = 0;
    lin_diag.network_errors = 0;
    
    // Thêm các node để scan
    LIN_Diagnostic_AddNode(0x01);
    LIN_Diagnostic_AddNode(0x02);
    LIN_Diagnostic_AddNode(0x03);
    
    // Bắt đầu scan đầu tiên
    LIN_Diagnostic_StartScan();
    
    while(1)
    {
        // Xử lý scan
        LIN_Diagnostic_ProcessScan();
        
        // Kiểm tra timeout cho response
        if(lin_diag.waiting_for_response && 
           (system_ms - lin_diag.request_timestamp) > LIN_DIAGNOSTIC_TIMEOUT)
        {
            // Timeout - mark node as offline
            LIN_Diagnostic_UpdateNodeStatus(lin_diag.current_request_node, NODE_STATUS_OFFLINE);
            lin_diag.waiting_for_response = 0;
            lin_diag.network_errors++;
        }
        
        // Scan định kỳ mỗi 5 giây
        static uint32_t last_scan = 0;
        if(!lin_diag.scan_active && (system_ms - last_scan) > 5000)
        {
            LIN_Diagnostic_StartScan();
            last_scan = system_ms;
        }
        
        // Đọc DTC định kỳ cho các node online
        static uint32_t last_dtc_read = 0;
        if((system_ms - last_dtc_read) > 10000)
        {
            for(uint8_t i = 0; i < lin_diag.node_count; i++)
            {
                if(lin_diag.nodes[i].status == NODE_STATUS_ONLINE)
                {
                    LIN_Diagnostic_ReadDTC(lin_diag.nodes[i].node_id);
                }
            }
            last_dtc_read = system_ms;
        }
        
        // Status LEDs
        // PC13: Scan active
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, lin_diag.scan_active ? Bit_RESET : Bit_SET);
        
        // PC14: Network error indicator
        static uint32_t last_error_blink = 0;
        if(lin_diag.network_errors > 0 && (system_ms - last_error_blink) > 500)
        {
            GPIO_WriteBit(GPIOC, GPIO_Pin_14, 
                         (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_14) == Bit_SET) ? Bit_RESET : Bit_SET);
            last_error_blink = system_ms;
        }
        
        // PC15: Activity indicator
        static uint32_t last_activity = 0;
        if((system_ms - last_activity) > 1000)
        {
            GPIO_WriteBit(GPIOC, GPIO_Pin_15, 
                         (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15) == Bit_SET) ? Bit_RESET : Bit_SET);
            last_activity = system_ms;
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
    static uint8_t receiving_response = 0;
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART1);
        
        if(lin_diag.waiting_for_response)
        {
            if(!receiving_response)
            {
                // First byte - determine expected length
                expected_length = 8; // Assume maximum length for diagnostic
                receiving_response = 1;
                rx_index = 0;
            }
            
            rx_buffer[rx_index++] = received_byte;
            
            if(rx_index >= expected_length + 1) // +1 for checksum
            {
                // Process response
                LIN_Diagnostic_ProcessResponse(rx_buffer, expected_length);
                lin_diag.waiting_for_response = 0;
                receiving_response = 0;
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
 * DIAGNOSTIC FUNCTIONS - Các hàm diagnostic
 * =============================================================================
 */
void LIN_Diagnostic_StartScan(void)
{
    if(!lin_diag.scan_active && lin_diag.node_count > 0)
    {
        lin_diag.scan_active = 1;
        lin_diag.current_scan_node = 0;
        lin_diag.scan_start_time = system_ms;
        lin_diag.total_scans++;
        
        // Start with first node
        LIN_Diagnostic_ReadNodeInfo(lin_diag.nodes[0].node_id);
    }
}

void LIN_Diagnostic_ProcessScan(void)
{
    if(!lin_diag.scan_active || lin_diag.waiting_for_response)
        return;
    
    // Move to next node
    lin_diag.current_scan_node++;
    
    if(lin_diag.current_scan_node >= lin_diag.node_count)
    {
        // Scan complete
        lin_diag.scan_active = 0;
        lin_diag.successful_scans++;
    }
    else
    {
        // Scan next node
        LIN_Diagnostic_ReadNodeInfo(lin_diag.nodes[lin_diag.current_scan_node].node_id);
    }
}

void LIN_Diagnostic_SendRequest(uint8_t node_id, uint8_t command, uint8_t *data, uint8_t length)
{
    uint8_t request_data[8];
    uint8_t request_length = 0;

    // Build diagnostic request
    request_data[0] = node_id;      // Target node
    request_data[1] = command;      // Diagnostic command

    // Add data if provided
    for(uint8_t i = 0; i < length && i < 6; i++)
    {
        request_data[2 + i] = data[i];
    }

    request_length = 2 + length;

    // Send LIN frame
    uint8_t pid = LIN_CalculatePID(LIN_DIAG_MASTER_REQ);
    uint8_t checksum = LIN_CalculateChecksum(pid, request_data, request_length);

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
    for(uint8_t i = 0; i < request_length; i++)
    {
        USART_SendData(USART1, request_data[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }

    // Send checksum
    USART_SendData(USART1, checksum);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // Set waiting for response
    lin_diag.waiting_for_response = 1;
    lin_diag.request_timestamp = system_ms;
    lin_diag.current_request_node = node_id;

    // Update node statistics
    LIN_NodeInfo_t *node = LIN_Diagnostic_FindNode(node_id);
    if(node != NULL)
    {
        node->total_requests++;
    }
}

void LIN_Diagnostic_ProcessResponse(uint8_t *data, uint8_t length)
{
    if(length < 2) return;

    uint8_t responding_node = data[0];
    uint8_t response_command = data[1];

    LIN_NodeInfo_t *node = LIN_Diagnostic_FindNode(responding_node);
    if(node == NULL) return;

    // Update node status
    node->status = NODE_STATUS_ONLINE;
    node->last_response = system_ms;
    node->response_time_ms = system_ms - lin_diag.request_timestamp;
    node->successful_responses++;

    // Process response based on command
    switch(response_command)
    {
        case LIN_DIAG_READ_BY_ID:
            if(length >= 8)
            {
                // Software version
                node->software_version[0] = data[2];
                node->software_version[1] = data[3];
                node->software_version[2] = data[4];

                // Hardware version
                node->hardware_version[0] = data[5];
                node->hardware_version[1] = data[6];
                node->hardware_version[2] = data[7];
            }
            break;

        case LIN_DIAG_READ_DTC:
            if(length >= 4)
            {
                node->dtc_count = data[2];
                uint8_t dtc_index = 0;

                for(uint8_t i = 3; i < length && dtc_index < 8; i += 2)
                {
                    if(i + 1 < length)
                    {
                        node->diagnostic_trouble_codes[dtc_index] = (data[i] << 8) | data[i + 1];
                        dtc_index++;
                    }
                }
            }
            break;

        case LIN_DIAG_TESTER_PRESENT:
            // Node is alive
            break;

        default:
            break;
    }
}

void LIN_Diagnostic_AddNode(uint8_t node_id)
{
    if(lin_diag.node_count < LIN_MAX_NODES)
    {
        LIN_NodeInfo_t *node = &lin_diag.nodes[lin_diag.node_count];

        node->node_id = node_id;
        node->status = NODE_STATUS_UNKNOWN;
        node->last_response = 0;
        node->response_time_ms = 0;
        node->total_requests = 0;
        node->successful_responses = 0;
        node->error_count = 0;
        node->dtc_count = 0;

        // Initialize version info
        for(uint8_t i = 0; i < 3; i++)
        {
            node->software_version[i] = 0;
            node->hardware_version[i] = 0;
        }

        // Initialize DTC array
        for(uint8_t i = 0; i < 8; i++)
        {
            node->diagnostic_trouble_codes[i] = 0;
        }

        lin_diag.node_count++;
    }
}

LIN_NodeInfo_t* LIN_Diagnostic_FindNode(uint8_t node_id)
{
    for(uint8_t i = 0; i < lin_diag.node_count; i++)
    {
        if(lin_diag.nodes[i].node_id == node_id)
        {
            return &lin_diag.nodes[i];
        }
    }
    return NULL;
}

void LIN_Diagnostic_UpdateNodeStatus(uint8_t node_id, uint8_t status)
{
    LIN_NodeInfo_t *node = LIN_Diagnostic_FindNode(node_id);
    if(node != NULL)
    {
        node->status = status;
        if(status == NODE_STATUS_OFFLINE || status == NODE_STATUS_ERROR)
        {
            node->error_count++;
        }
    }
}

void LIN_Diagnostic_ReadNodeInfo(uint8_t node_id)
{
    uint8_t data[2] = {0x01, 0x00}; // Read software/hardware version
    LIN_Diagnostic_SendRequest(node_id, LIN_DIAG_READ_BY_ID, data, 2);
}

void LIN_Diagnostic_ReadDTC(uint8_t node_id)
{
    uint8_t data[1] = {0x02}; // Read stored DTCs
    LIN_Diagnostic_SendRequest(node_id, LIN_DIAG_READ_DTC, data, 1);
}

void LIN_Diagnostic_ClearDTC(uint8_t node_id)
{
    uint8_t data[1] = {0xFF}; // Clear all DTCs
    LIN_Diagnostic_SendRequest(node_id, LIN_DIAG_CLEAR_DTC, data, 1);
}

/* =============================================================================
 * INITIALIZATION FUNCTIONS - Các hàm khởi tạo
 * =============================================================================
 */
void LIN_Diagnostic_Init(void)
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
