/*
 * =============================================================================
 * Project: STM32F103 LIN Receive Example (SPL)
 * File: lin_receive_example.c
 * Description: Dedicated LIN receiver with frame filtering and processing
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
#define LIN_MAX_DATA_LENGTH     8
#define LIN_BUFFER_SIZE         16

// Frame IDs to monitor
#define LIN_ID_SENSOR_DATA      0x20
#define LIN_ID_ACTUATOR_CMD     0x21
#define LIN_ID_STATUS_REQ       0x22
#define LIN_ID_DIAGNOSTIC       0x3C

// LED pins for status indication
#define LED_RX_PIN              GPIO_Pin_13  // PC13 - Frame received
#define LED_ERROR_PIN           GPIO_Pin_14  // PC14 - Error indicator
#define LED_ACTIVITY_PIN        GPIO_Pin_15  // PC15 - Activity indicator

/* =============================================================================
 * STRUCTURES - Cấu trúc dữ liệu
 * =============================================================================
 */
typedef struct {
    uint8_t id;
    uint8_t data[LIN_MAX_DATA_LENGTH];
    uint8_t length;
    uint8_t checksum;
    uint32_t timestamp;
    uint8_t valid;
} LIN_RxFrame_t;

typedef struct {
    LIN_RxFrame_t frames[LIN_BUFFER_SIZE];
    uint8_t write_index;
    uint8_t read_index;
    uint8_t count;
    uint8_t overflow;
} LIN_RxBuffer_t;

typedef struct {
    uint8_t state;
    uint8_t rx_buffer[16];
    uint8_t rx_index;
    uint8_t expected_length;
    uint32_t last_byte_time;
    uint8_t error_count;
    uint32_t frame_count;
} LIN_RxHandler_t;

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
volatile LIN_RxBuffer_t lin_rx_buffer;
volatile LIN_RxHandler_t lin_rx_handler;
volatile uint32_t system_ms = 0;

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void LIN_Receiver_Init(void);
void LED_Init(void);
void SysTick_Init(void);
uint8_t LIN_ValidatePID(uint8_t pid);
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length);
void LIN_ProcessReceivedByte(uint8_t byte);
void LIN_AddFrameToBuffer(LIN_RxFrame_t *frame);
uint8_t LIN_GetFrameFromBuffer(LIN_RxFrame_t *frame);
void LIN_ProcessFrame(LIN_RxFrame_t *frame);
void LIN_HandleSensorData(uint8_t *data, uint8_t length);
void LIN_HandleActuatorCommand(uint8_t *data, uint8_t length);
void LIN_HandleStatusRequest(void);
void LIN_HandleDiagnostic(uint8_t *data, uint8_t length);
void LED_Blink(uint16_t pin, uint32_t duration);

/* =============================================================================
 * MAIN FUNCTION - Chương trình chính
 * =============================================================================
 */
int main(void)
{
    LIN_RxFrame_t received_frame;
    uint32_t last_activity = 0;
    
    // Khởi tạo hệ thống
    SystemInit();
    SysTick_Init();
    LED_Init();
    LIN_Receiver_Init();
    
    // Khởi tạo buffer
    lin_rx_buffer.write_index = 0;
    lin_rx_buffer.read_index = 0;
    lin_rx_buffer.count = 0;
    lin_rx_buffer.overflow = 0;
    
    // Khởi tạo handler
    lin_rx_handler.state = 0;
    lin_rx_handler.rx_index = 0;
    lin_rx_handler.error_count = 0;
    lin_rx_handler.frame_count = 0;
    
    // Bật LED activity để báo hiệu hệ thống đã sẵn sàng
    GPIO_ResetBits(GPIOC, LED_ACTIVITY_PIN);
    
    while(1)
    {
        // Xử lý frame nhận được
        if(LIN_GetFrameFromBuffer(&received_frame))
        {
            LIN_ProcessFrame(&received_frame);
            last_activity = system_ms;
            
            // Blink LED RX
            LED_Blink(LED_RX_PIN, 100);
        }
        
        // Kiểm tra buffer overflow
        if(lin_rx_buffer.overflow)
        {
            LED_Blink(LED_ERROR_PIN, 500);
            lin_rx_buffer.overflow = 0;
        }
        
        // Activity LED - nhấp nháy mỗi giây khi không có hoạt động
        if(system_ms - last_activity > 1000)
        {
            GPIO_WriteBit(GPIOC, LED_ACTIVITY_PIN, 
                         (system_ms % 2000 < 1000) ? Bit_RESET : Bit_SET);
        }
        else
        {
            GPIO_ResetBits(GPIOC, LED_ACTIVITY_PIN);
        }
        
        // Reset error count định kỳ
        static uint32_t last_reset = 0;
        if(system_ms - last_reset > 10000)
        {
            lin_rx_handler.error_count = 0;
            last_reset = system_ms;
        }
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
        LIN_ProcessReceivedByte(received_byte);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    
    if(USART_GetITStatus(USART1, USART_IT_LBD) != RESET)
    {
        // LIN Break detected - reset receiver
        lin_rx_handler.state = 1; // Waiting for sync
        lin_rx_handler.rx_index = 0;
        lin_rx_handler.last_byte_time = system_ms;
        USART_ClearITPendingBit(USART1, USART_IT_LBD);
    }
}

void SysTick_Handler(void)
{
    system_ms++;
    
    // Timeout check for incomplete frames
    if(lin_rx_handler.state > 0 && 
       (system_ms - lin_rx_handler.last_byte_time) > 100)
    {
        lin_rx_handler.state = 0; // Reset to idle
        lin_rx_handler.error_count++;
    }
}

/* =============================================================================
 * LIN_ProcessReceivedByte - Xử lý byte nhận được
 * =============================================================================
 */
void LIN_ProcessReceivedByte(uint8_t byte)
{
    lin_rx_handler.last_byte_time = system_ms;
    
    switch(lin_rx_handler.state)
    {
        case 0: // Idle - waiting for break (handled by interrupt)
            break;
            
        case 1: // Waiting for sync byte
            if(byte == LIN_SYNC_BYTE)
            {
                lin_rx_handler.rx_buffer[0] = byte;
                lin_rx_handler.rx_index = 1;
                lin_rx_handler.state = 2; // Waiting for PID
            }
            else
            {
                lin_rx_handler.state = 0;
                lin_rx_handler.error_count++;
            }
            break;
            
        case 2: // Waiting for PID
            if(LIN_ValidatePID(byte))
            {
                lin_rx_handler.rx_buffer[1] = byte;
                lin_rx_handler.rx_index = 2;
                lin_rx_handler.state = 3; // Receiving data
                
                // Determine expected data length based on ID
                uint8_t id = byte & 0x3F;
                if(id <= 0x1F) lin_rx_handler.expected_length = 2;
                else if(id <= 0x2F) lin_rx_handler.expected_length = 4;
                else lin_rx_handler.expected_length = 8;
            }
            else
            {
                lin_rx_handler.state = 0;
                lin_rx_handler.error_count++;
            }
            break;
            
        case 3: // Receiving data and checksum
            lin_rx_handler.rx_buffer[lin_rx_handler.rx_index++] = byte;
            
            // Check if we have received all data + checksum
            if(lin_rx_handler.rx_index >= (2 + lin_rx_handler.expected_length + 1))
            {
                // Validate and process frame
                LIN_RxFrame_t frame;
                frame.id = lin_rx_handler.rx_buffer[1] & 0x3F;
                frame.length = lin_rx_handler.expected_length;
                
                for(uint8_t i = 0; i < frame.length; i++)
                {
                    frame.data[i] = lin_rx_handler.rx_buffer[2 + i];
                }
                
                frame.checksum = lin_rx_handler.rx_buffer[2 + frame.length];
                frame.timestamp = system_ms;
                
                // Validate checksum
                uint8_t calc_checksum = LIN_CalculateChecksum(lin_rx_handler.rx_buffer[1], 
                                                            frame.data, frame.length);
                
                if(calc_checksum == frame.checksum)
                {
                    frame.valid = 1;
                    LIN_AddFrameToBuffer(&frame);
                    lin_rx_handler.frame_count++;
                }
                else
                {
                    frame.valid = 0;
                    lin_rx_handler.error_count++;
                }
                
                lin_rx_handler.state = 0; // Return to idle
            }
            break;
            
        default:
            lin_rx_handler.state = 0;
            break;
    }
}

/* =============================================================================
 * INITIALIZATION FUNCTIONS - Các hàm khởi tạo
 * =============================================================================
 */
void LIN_Receiver_Init(void)
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

    // Configure USART1 for LIN
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

    // Configure NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable interrupts
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_LBD, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_RX_PIN | LED_ERROR_PIN | LED_ACTIVITY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Turn off all LEDs initially
    GPIO_SetBits(GPIOC, LED_RX_PIN | LED_ERROR_PIN | LED_ACTIVITY_PIN);
}

void SysTick_Init(void)
{
    // Configure SysTick for 1ms tick
    SysTick_Config(SystemCoreClock / 1000);
}

/* =============================================================================
 * UTILITY FUNCTIONS - Các hàm tiện ích
 * =============================================================================
 */
uint8_t LIN_ValidatePID(uint8_t pid)
{
    uint8_t id = pid & 0x3F;
    uint8_t p0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^ ((id >> 2) & 1) ^ ((id >> 4) & 1);
    uint8_t p1 = !(((id >> 1) & 1) ^ ((id >> 3) & 1) ^ ((id >> 4) & 1) ^ ((id >> 5) & 1));

    uint8_t expected_pid = id | (p0 << 6) | (p1 << 7);
    return (expected_pid == pid);
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

void LED_Blink(uint16_t pin, uint32_t duration)
{
    static uint32_t led_timers[3] = {0, 0, 0};
    static uint8_t led_states[3] = {0, 0, 0};
    uint8_t led_index = 0;

    // Determine LED index
    if(pin == LED_RX_PIN) led_index = 0;
    else if(pin == LED_ERROR_PIN) led_index = 1;
    else if(pin == LED_ACTIVITY_PIN) led_index = 2;

    if(led_states[led_index] == 0)
    {
        GPIO_ResetBits(GPIOC, pin);
        led_timers[led_index] = system_ms;
        led_states[led_index] = 1;
    }
    else if(system_ms - led_timers[led_index] >= duration)
    {
        GPIO_SetBits(GPIOC, pin);
        led_states[led_index] = 0;
    }
}

/* =============================================================================
 * BUFFER MANAGEMENT FUNCTIONS - Quản lý buffer
 * =============================================================================
 */
void LIN_AddFrameToBuffer(LIN_RxFrame_t *frame)
{
    if(lin_rx_buffer.count < LIN_BUFFER_SIZE)
    {
        lin_rx_buffer.frames[lin_rx_buffer.write_index] = *frame;
        lin_rx_buffer.write_index = (lin_rx_buffer.write_index + 1) % LIN_BUFFER_SIZE;
        lin_rx_buffer.count++;
    }
    else
    {
        lin_rx_buffer.overflow = 1;
    }
}

uint8_t LIN_GetFrameFromBuffer(LIN_RxFrame_t *frame)
{
    if(lin_rx_buffer.count > 0)
    {
        *frame = lin_rx_buffer.frames[lin_rx_buffer.read_index];
        lin_rx_buffer.read_index = (lin_rx_buffer.read_index + 1) % LIN_BUFFER_SIZE;
        lin_rx_buffer.count--;
        return 1;
    }
    return 0;
}

/* =============================================================================
 * FRAME PROCESSING FUNCTIONS - Xử lý frame
 * =============================================================================
 */
void LIN_ProcessFrame(LIN_RxFrame_t *frame)
{
    if(!frame->valid) return;

    switch(frame->id)
    {
        case LIN_ID_SENSOR_DATA:
            LIN_HandleSensorData(frame->data, frame->length);
            break;

        case LIN_ID_ACTUATOR_CMD:
            LIN_HandleActuatorCommand(frame->data, frame->length);
            break;

        case LIN_ID_STATUS_REQ:
            LIN_HandleStatusRequest();
            break;

        case LIN_ID_DIAGNOSTIC:
            LIN_HandleDiagnostic(frame->data, frame->length);
            break;

        default:
            // Unknown frame ID - log or ignore
            break;
    }
}

void LIN_HandleSensorData(uint8_t *data, uint8_t length)
{
    if(length >= 4)
    {
        // Example: Temperature and humidity sensor data
        uint16_t temperature = (data[0] << 8) | data[1];
        uint16_t humidity = (data[2] << 8) | data[3];

        // Process sensor data
        // Example: Control LED based on temperature
        if(temperature > 2500) // 25.0°C
        {
            GPIO_ResetBits(GPIOC, LED_RX_PIN); // LED ON for high temp
        }
        else
        {
            GPIO_SetBits(GPIOC, LED_RX_PIN);   // LED OFF for normal temp
        }
    }
}

void LIN_HandleActuatorCommand(uint8_t *data, uint8_t length)
{
    if(length >= 2)
    {
        uint8_t actuator_id = data[0];
        uint8_t command = data[1];

        switch(actuator_id)
        {
            case 0x01: // LED actuator
                if(command == 0x01)
                    GPIO_ResetBits(GPIOC, LED_ACTIVITY_PIN);
                else
                    GPIO_SetBits(GPIOC, LED_ACTIVITY_PIN);
                break;

            case 0x02: // Motor control
                // Implement motor control logic
                break;

            default:
                break;
        }
    }
}

void LIN_HandleStatusRequest(void)
{
    // Prepare status response
    // This would typically send a response frame
    // For this example, just blink error LED to indicate status request received
    LED_Blink(LED_ERROR_PIN, 200);
}

void LIN_HandleDiagnostic(uint8_t *data, uint8_t length)
{
    if(length >= 1)
    {
        uint8_t diagnostic_cmd = data[0];

        switch(diagnostic_cmd)
        {
            case 0x01: // Reset error counters
                lin_rx_handler.error_count = 0;
                break;

            case 0x02: // Get statistics
                // Prepare statistics response
                break;

            case 0x03: // Self test
                // Perform self test
                LED_Blink(LED_RX_PIN, 100);
                LED_Blink(LED_ERROR_PIN, 100);
                LED_Blink(LED_ACTIVITY_PIN, 100);
                break;

            default:
                break;
        }
    }
}
