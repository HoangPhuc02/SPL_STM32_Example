/*
 * =============================================================================
 * Project: STM32F103 LIN Bridge/Gateway Example (SPL)
 * File: lin_bridge_example.c
 * Description: LIN to UART bridge for PC monitoring and control
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
#include "system_stm32f10x.h"

/* =============================================================================
 * DEFINES - Các định nghĩa và hằng số
 * =============================================================================
 */
#define LIN_SYNC_BYTE           0x55
#define BRIDGE_BUFFER_SIZE      64
#define UART_BAUD_RATE          115200
#define LIN_BAUD_RATE           9600

// Bridge commands from PC
#define CMD_SEND_LIN_FRAME      0x01
#define CMD_SET_LIN_FILTER      0x02
#define CMD_GET_STATISTICS      0x03
#define CMD_RESET_BRIDGE        0x04
#define CMD_SET_MONITOR_MODE    0x05

// Response codes to PC
#define RESP_LIN_FRAME_RX       0x81
#define RESP_LIN_FRAME_TX       0x82
#define RESP_STATISTICS         0x83
#define RESP_ERROR              0x84
#define RESP_ACK                0x85

/* =============================================================================
 * STRUCTURES - Cấu trúc dữ liệu
 * =============================================================================
 */
typedef struct {
    uint8_t start_byte;     // 0xAA
    uint8_t command;
    uint8_t length;
    uint8_t data[16];
    uint8_t checksum;
} UART_Packet_t;

typedef struct {
    uint8_t id;
    uint8_t data[8];
    uint8_t length;
    uint8_t checksum;
    uint32_t timestamp;
    uint8_t direction;      // 0=RX, 1=TX
} LIN_LogEntry_t;

typedef struct {
    // UART communication
    uint8_t uart_rx_buffer[BRIDGE_BUFFER_SIZE];
    uint8_t uart_rx_index;
    uint8_t uart_packet_ready;
    
    // LIN communication
    uint8_t lin_rx_buffer[16];
    uint8_t lin_rx_index;
    uint8_t lin_rx_state;
    uint8_t lin_frame_ready;
    
    // Filtering
    uint8_t filter_enabled;
    uint8_t filter_ids[16];
    uint8_t filter_count;
    
    // Monitoring
    uint8_t monitor_mode;   // 0=bridge, 1=monitor only
    
    // Statistics
    uint32_t lin_frames_rx;
    uint32_t lin_frames_tx;
    uint32_t uart_packets_rx;
    uint32_t uart_packets_tx;
    uint32_t errors;
    
    // Log buffer
    LIN_LogEntry_t log_buffer[32];
    uint8_t log_write_index;
    uint8_t log_read_index;
    uint8_t log_count;
} LIN_Bridge_t;

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
volatile LIN_Bridge_t bridge;
volatile uint32_t system_ms = 0;

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void Bridge_Init(void);
void UART_PC_Init(void);
void LIN_Interface_Init(void);
void SysTick_Init(void);
void Bridge_ProcessUARTPacket(UART_Packet_t *packet);
void Bridge_ProcessLINFrame(uint8_t *data, uint8_t length);
void Bridge_SendUARTPacket(uint8_t command, uint8_t *data, uint8_t length);
void Bridge_SendLINFrame(uint8_t id, uint8_t *data, uint8_t length);
void Bridge_AddLogEntry(uint8_t id, uint8_t *data, uint8_t length, uint8_t direction);
uint8_t Bridge_CheckFilter(uint8_t id);
uint8_t Bridge_CalculateUARTChecksum(UART_Packet_t *packet);
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
    SysTick_Init();
    Bridge_Init();
    UART_PC_Init();
    LIN_Interface_Init();
    
    // Khởi tạo LED status
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    // Khởi tạo bridge
    bridge.uart_rx_index = 0;
    bridge.uart_packet_ready = 0;
    bridge.lin_rx_index = 0;
    bridge.lin_rx_state = 0;
    bridge.lin_frame_ready = 0;
    bridge.filter_enabled = 0;
    bridge.filter_count = 0;
    bridge.monitor_mode = 0;
    bridge.lin_frames_rx = 0;
    bridge.lin_frames_tx = 0;
    bridge.uart_packets_rx = 0;
    bridge.uart_packets_tx = 0;
    bridge.errors = 0;
    bridge.log_write_index = 0;
    bridge.log_read_index = 0;
    bridge.log_count = 0;
    
    // Send startup message to PC
    uint8_t startup_msg[] = "LIN Bridge Ready";
    Bridge_SendUARTPacket(RESP_ACK, startup_msg, sizeof(startup_msg) - 1);
    
    while(1)
    {
        // Process UART packets from PC
        if(bridge.uart_packet_ready)
        {
            UART_Packet_t packet;
            packet.start_byte = 0xAA;
            packet.command = bridge.uart_rx_buffer[1];
            packet.length = bridge.uart_rx_buffer[2];
            
            for(uint8_t i = 0; i < packet.length && i < 16; i++)
            {
                packet.data[i] = bridge.uart_rx_buffer[3 + i];
            }
            
            packet.checksum = bridge.uart_rx_buffer[3 + packet.length];
            
            if(Bridge_CalculateUARTChecksum(&packet) == packet.checksum)
            {
                Bridge_ProcessUARTPacket(&packet);
                bridge.uart_packets_rx++;
            }
            else
            {
                bridge.errors++;
                Bridge_SendUARTPacket(RESP_ERROR, NULL, 0);
            }
            
            bridge.uart_packet_ready = 0;
            bridge.uart_rx_index = 0;
        }
        
        // Process LIN frames
        if(bridge.lin_frame_ready)
        {
            Bridge_ProcessLINFrame(bridge.lin_rx_buffer, bridge.lin_rx_index);
            bridge.lin_frame_ready = 0;
            bridge.lin_rx_index = 0;
            bridge.lin_rx_state = 0;
        }
        
        // Send periodic statistics
        static uint32_t last_stats = 0;
        if(system_ms - last_stats > 5000)
        {
            uint8_t stats_data[16];
            stats_data[0] = (uint8_t)(bridge.lin_frames_rx >> 24);
            stats_data[1] = (uint8_t)(bridge.lin_frames_rx >> 16);
            stats_data[2] = (uint8_t)(bridge.lin_frames_rx >> 8);
            stats_data[3] = (uint8_t)(bridge.lin_frames_rx);
            stats_data[4] = (uint8_t)(bridge.lin_frames_tx >> 24);
            stats_data[5] = (uint8_t)(bridge.lin_frames_tx >> 16);
            stats_data[6] = (uint8_t)(bridge.lin_frames_tx >> 8);
            stats_data[7] = (uint8_t)(bridge.lin_frames_tx);
            stats_data[8] = (uint8_t)(bridge.errors >> 8);
            stats_data[9] = (uint8_t)(bridge.errors);
            
            Bridge_SendUARTPacket(RESP_STATISTICS, stats_data, 10);
            last_stats = system_ms;
        }
        
        // Activity LED
        static uint32_t last_blink = 0;
        if(system_ms - last_blink > 1000)
        {
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, 
                         (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET) ? Bit_RESET : Bit_SET);
            last_blink = system_ms;
        }
    }
}

/* =============================================================================
 * INTERRUPT HANDLERS - Xử lý ngắt
 * =============================================================================
 */
void USART1_IRQHandler(void) // LIN Interface
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART1);
        
        switch(bridge.lin_rx_state)
        {
            case 0: // Waiting for sync after break
                if(received_byte == LIN_SYNC_BYTE)
                {
                    bridge.lin_rx_buffer[0] = received_byte;
                    bridge.lin_rx_index = 1;
                    bridge.lin_rx_state = 1;
                }
                break;
                
            case 1: // Receiving frame
                bridge.lin_rx_buffer[bridge.lin_rx_index++] = received_byte;
                
                if(bridge.lin_rx_index >= 10) // Max frame size
                {
                    bridge.lin_frame_ready = 1;
                }
                break;
                
            default:
                bridge.lin_rx_state = 0;
                break;
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    
    if(USART_GetITStatus(USART1, USART_IT_LBD) != RESET)
    {
        bridge.lin_rx_state = 0;
        bridge.lin_rx_index = 0;
        USART_ClearITPendingBit(USART1, USART_IT_LBD);
    }
}

void USART2_IRQHandler(void) // PC Interface
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART2);
        
        if(bridge.uart_rx_index == 0 && received_byte == 0xAA)
        {
            bridge.uart_rx_buffer[bridge.uart_rx_index++] = received_byte;
        }
        else if(bridge.uart_rx_index > 0)
        {
            bridge.uart_rx_buffer[bridge.uart_rx_index++] = received_byte;
            
            // Check if packet is complete
            if(bridge.uart_rx_index >= 3)
            {
                uint8_t expected_length = bridge.uart_rx_buffer[2];
                if(bridge.uart_rx_index >= (4 + expected_length))
                {
                    bridge.uart_packet_ready = 1;
                }
            }
            
            // Prevent buffer overflow
            if(bridge.uart_rx_index >= BRIDGE_BUFFER_SIZE)
            {
                bridge.uart_rx_index = 0;
                bridge.errors++;
            }
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void SysTick_Handler(void)
{
    system_ms++;
}

/* =============================================================================
 * BRIDGE FUNCTIONS - Các hàm bridge
 * =============================================================================
 */
void Bridge_ProcessUARTPacket(UART_Packet_t *packet)
{
    switch(packet->command)
    {
        case CMD_SEND_LIN_FRAME:
            if(packet->length >= 2 && !bridge.monitor_mode)
            {
                uint8_t lin_id = packet->data[0];
                uint8_t lin_length = packet->data[1];

                if(lin_length <= 8 && packet->length >= (2 + lin_length))
                {
                    Bridge_SendLINFrame(lin_id, &packet->data[2], lin_length);
                    Bridge_SendUARTPacket(RESP_ACK, NULL, 0);
                }
                else
                {
                    Bridge_SendUARTPacket(RESP_ERROR, NULL, 0);
                }
            }
            break;

        case CMD_SET_LIN_FILTER:
            bridge.filter_enabled = packet->data[0];
            bridge.filter_count = packet->data[1];

            for(uint8_t i = 0; i < bridge.filter_count && i < 16; i++)
            {
                bridge.filter_ids[i] = packet->data[2 + i];
            }

            Bridge_SendUARTPacket(RESP_ACK, NULL, 0);
            break;

        case CMD_GET_STATISTICS:
            {
                uint8_t stats_data[16];
                stats_data[0] = (uint8_t)(bridge.lin_frames_rx >> 24);
                stats_data[1] = (uint8_t)(bridge.lin_frames_rx >> 16);
                stats_data[2] = (uint8_t)(bridge.lin_frames_rx >> 8);
                stats_data[3] = (uint8_t)(bridge.lin_frames_rx);
                stats_data[4] = (uint8_t)(bridge.lin_frames_tx >> 24);
                stats_data[5] = (uint8_t)(bridge.lin_frames_tx >> 16);
                stats_data[6] = (uint8_t)(bridge.lin_frames_tx >> 8);
                stats_data[7] = (uint8_t)(bridge.lin_frames_tx);
                stats_data[8] = (uint8_t)(bridge.uart_packets_rx >> 8);
                stats_data[9] = (uint8_t)(bridge.uart_packets_rx);
                stats_data[10] = (uint8_t)(bridge.errors >> 8);
                stats_data[11] = (uint8_t)(bridge.errors);
                stats_data[12] = bridge.filter_enabled;
                stats_data[13] = bridge.monitor_mode;

                Bridge_SendUARTPacket(RESP_STATISTICS, stats_data, 14);
            }
            break;

        case CMD_RESET_BRIDGE:
            bridge.lin_frames_rx = 0;
            bridge.lin_frames_tx = 0;
            bridge.uart_packets_rx = 0;
            bridge.uart_packets_tx = 0;
            bridge.errors = 0;
            bridge.log_write_index = 0;
            bridge.log_read_index = 0;
            bridge.log_count = 0;

            Bridge_SendUARTPacket(RESP_ACK, NULL, 0);
            break;

        case CMD_SET_MONITOR_MODE:
            bridge.monitor_mode = packet->data[0];
            Bridge_SendUARTPacket(RESP_ACK, NULL, 0);
            break;

        default:
            Bridge_SendUARTPacket(RESP_ERROR, NULL, 0);
            break;
    }
}

void Bridge_ProcessLINFrame(uint8_t *data, uint8_t length)
{
    if(length < 3) return; // Minimum: sync + pid + checksum

    uint8_t sync = data[0];
    uint8_t pid = data[1];
    uint8_t lin_id = pid & 0x3F;

    if(sync != LIN_SYNC_BYTE) return;

    // Check filter
    if(bridge.filter_enabled && !Bridge_CheckFilter(lin_id))
        return;

    // Extract data and checksum
    uint8_t data_length = length - 3; // Exclude sync, pid, checksum
    uint8_t lin_data[8];
    uint8_t checksum = data[length - 1];

    for(uint8_t i = 0; i < data_length && i < 8; i++)
    {
        lin_data[i] = data[2 + i];
    }

    // Validate checksum
    uint8_t calc_checksum = LIN_CalculateChecksum(pid, lin_data, data_length);
    if(calc_checksum != checksum)
    {
        bridge.errors++;
        return;
    }

    // Add to log
    Bridge_AddLogEntry(lin_id, lin_data, data_length, 0); // 0 = RX

    // Send to PC
    uint8_t uart_data[16];
    uart_data[0] = lin_id;
    uart_data[1] = data_length;
    uart_data[2] = (uint8_t)(system_ms >> 24);
    uart_data[3] = (uint8_t)(system_ms >> 16);
    uart_data[4] = (uint8_t)(system_ms >> 8);
    uart_data[5] = (uint8_t)(system_ms);

    for(uint8_t i = 0; i < data_length; i++)
    {
        uart_data[6 + i] = lin_data[i];
    }

    Bridge_SendUARTPacket(RESP_LIN_FRAME_RX, uart_data, 6 + data_length);
    bridge.lin_frames_rx++;
}

void Bridge_SendUARTPacket(uint8_t command, uint8_t *data, uint8_t length)
{
    UART_Packet_t packet;
    packet.start_byte = 0xAA;
    packet.command = command;
    packet.length = length;

    if(data != NULL && length > 0)
    {
        for(uint8_t i = 0; i < length && i < 16; i++)
        {
            packet.data[i] = data[i];
        }
    }

    packet.checksum = Bridge_CalculateUARTChecksum(&packet);

    // Send packet
    USART_SendData(USART2, packet.start_byte);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    USART_SendData(USART2, packet.command);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    USART_SendData(USART2, packet.length);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    for(uint8_t i = 0; i < packet.length; i++)
    {
        USART_SendData(USART2, packet.data[i]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    }

    USART_SendData(USART2, packet.checksum);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    bridge.uart_packets_tx++;
}

void Bridge_SendLINFrame(uint8_t id, uint8_t *data, uint8_t length)
{
    uint8_t pid = LIN_CalculatePID(id);
    uint8_t checksum = LIN_CalculateChecksum(pid, data, length);

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

    // Add to log
    Bridge_AddLogEntry(id, data, length, 1); // 1 = TX
    bridge.lin_frames_tx++;
}

/* =============================================================================
 * UTILITY FUNCTIONS - Các hàm tiện ích
 * =============================================================================
 */
void Bridge_AddLogEntry(uint8_t id, uint8_t *data, uint8_t length, uint8_t direction)
{
    if(bridge.log_count < 32)
    {
        LIN_LogEntry_t *entry = &bridge.log_buffer[bridge.log_write_index];

        entry->id = id;
        entry->length = length;
        entry->direction = direction;
        entry->timestamp = system_ms;

        for(uint8_t i = 0; i < length && i < 8; i++)
        {
            entry->data[i] = data[i];
        }

        bridge.log_write_index = (bridge.log_write_index + 1) % 32;
        bridge.log_count++;
    }
    else
    {
        // Buffer full - overwrite oldest entry
        bridge.log_read_index = (bridge.log_read_index + 1) % 32;
        bridge.log_write_index = (bridge.log_write_index + 1) % 32;
    }
}

uint8_t Bridge_CheckFilter(uint8_t id)
{
    if(!bridge.filter_enabled)
        return 1; // No filter - accept all

    for(uint8_t i = 0; i < bridge.filter_count; i++)
    {
        if(bridge.filter_ids[i] == id)
            return 1; // ID found in filter
    }

    return 0; // ID not in filter
}

uint8_t Bridge_CalculateUARTChecksum(UART_Packet_t *packet)
{
    uint8_t checksum = packet->start_byte + packet->command + packet->length;

    for(uint8_t i = 0; i < packet->length; i++)
    {
        checksum += packet->data[i];
    }

    return ~checksum;
}

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

/* =============================================================================
 * INITIALIZATION FUNCTIONS - Các hàm khởi tạo
 * =============================================================================
 */
void Bridge_Init(void)
{
    // Initialize bridge structure
    bridge.uart_rx_index = 0;
    bridge.uart_packet_ready = 0;
    bridge.lin_rx_index = 0;
    bridge.lin_rx_state = 0;
    bridge.lin_frame_ready = 0;
    bridge.filter_enabled = 0;
    bridge.filter_count = 0;
    bridge.monitor_mode = 0;
}

void UART_PC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure USART2 pins (PA2=TX, PA3=RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART2
    USART_InitStructure.USART_BaudRate = UART_BAUD_RATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    // Configure interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void LIN_Interface_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO |
                          RCC_APB2Periph_USART1, ENABLE);

    // Configure USART1 pins (PA9=TX, PA10=RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART1
    USART_InitStructure.USART_BaudRate = LIN_BAUD_RATE;
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
    USART_ITConfig(USART1, USART_IT_LBD, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000); // 1ms tick
}
