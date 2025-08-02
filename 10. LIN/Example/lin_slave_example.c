/*
 * =============================================================================
 * Project: STM32F103 LIN Slave Example (SPL)
 * File: lin_slave_example.c
 * Description: LIN Slave node implementation with automatic response handling
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
#include "stm32f10x_adc.h"
#include "system_stm32f10x.h"

/* =============================================================================
 * DEFINES - Các định nghĩa và hằng số
 * =============================================================================
 */
#define LIN_SYNC_BYTE           0x55
#define LIN_SLAVE_NODE_ID       0x01
#define LIN_MAX_RESPONSE_TIME   10  // ms

// LIN Frame IDs this slave responds to
#define LIN_ID_SLAVE1_STATUS    0x20
#define LIN_ID_SENSOR_DATA      0x30
#define LIN_ID_ACTUATOR_CMD     0x31
#define LIN_ID_DIAGNOSTIC       0x3C

// Slave states
#define LIN_SLAVE_IDLE          0
#define LIN_SLAVE_HEADER_RX     1
#define LIN_SLAVE_DATA_RX       2
#define LIN_SLAVE_RESPONSE_TX   3

// Response types
#define LIN_RESPONSE_STATUS     0
#define LIN_RESPONSE_SENSOR     1
#define LIN_RESPONSE_ACK        2

/* =============================================================================
 * STRUCTURES - Cấu trúc dữ liệu
 * =============================================================================
 */
typedef struct {
    uint8_t id;
    uint8_t data[8];
    uint8_t length;
    uint8_t checksum;
    uint8_t valid;
} LIN_Frame_t;

typedef struct {
    uint8_t node_id;
    uint8_t state;
    uint8_t current_frame_id;
    uint8_t response_pending;
    uint32_t response_timer;
    
    // Sensor data
    uint16_t temperature;
    uint16_t voltage;
    uint8_t digital_inputs;
    uint8_t digital_outputs;
    
    // Statistics
    uint32_t frames_received;
    uint32_t responses_sent;
    uint32_t errors;
    
    // Receive buffer
    uint8_t rx_buffer[16];
    uint8_t rx_index;
    uint8_t expected_length;
} LIN_Slave_t;

/* =============================================================================
 * GLOBAL VARIABLES - Biến toàn cục
 * =============================================================================
 */
volatile LIN_Slave_t lin_slave;
volatile uint32_t system_ms = 0;

/* =============================================================================
 * FUNCTION PROTOTYPES - Khai báo prototype các hàm
 * =============================================================================
 */
void LIN_Slave_Init(void);
void ADC_Init(void);
void SysTick_Init(void);
void LIN_Slave_ProcessHeader(uint8_t pid);
void LIN_Slave_ProcessData(uint8_t *data, uint8_t length);
void LIN_Slave_SendResponse(uint8_t frame_id);
void LIN_Slave_UpdateSensorData(void);
uint8_t LIN_ValidatePID(uint8_t pid);
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length);
void LIN_Slave_HandleActuatorCommand(uint8_t *data, uint8_t length);
void LIN_Slave_HandleDiagnostic(uint8_t *data, uint8_t length);

/* =============================================================================
 * MAIN FUNCTION - Chương trình chính
 * =============================================================================
 */
int main(void)
{
    // Khởi tạo hệ thống
    SystemInit();
    SysTick_Init();
    LIN_Slave_Init();
    ADC_Init();
    
    // Khởi tạo LED và digital I/O
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // LED outputs (PC13, PC14, PC15)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    
    // Digital inputs (PB0, PB1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // Khởi tạo slave
    lin_slave.node_id = LIN_SLAVE_NODE_ID;
    lin_slave.state = LIN_SLAVE_IDLE;
    lin_slave.response_pending = 0;
    lin_slave.frames_received = 0;
    lin_slave.responses_sent = 0;
    lin_slave.errors = 0;
    
    while(1)
    {
        // Update sensor data
        LIN_Slave_UpdateSensorData();
        
        // Check for response timeout
        if(lin_slave.response_pending && 
           (system_ms - lin_slave.response_timer) > LIN_MAX_RESPONSE_TIME)
        {
            LIN_Slave_SendResponse(lin_slave.current_frame_id);
            lin_slave.response_pending = 0;
        }
        
        // Update digital outputs based on received commands
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, (lin_slave.digital_outputs & 0x01) ? Bit_RESET : Bit_SET);
        GPIO_WriteBit(GPIOC, GPIO_Pin_14, (lin_slave.digital_outputs & 0x02) ? Bit_RESET : Bit_SET);
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, (lin_slave.digital_outputs & 0x04) ? Bit_RESET : Bit_SET);
        
        // Heartbeat LED (PC13) - blink every second
        static uint32_t last_heartbeat = 0;
        if(system_ms - last_heartbeat > 1000)
        {
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, 
                         (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET) ? Bit_RESET : Bit_SET);
            last_heartbeat = system_ms;
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
        
        switch(lin_slave.state)
        {
            case LIN_SLAVE_IDLE:
                // Waiting for sync byte after break
                if(received_byte == LIN_SYNC_BYTE)
                {
                    lin_slave.state = LIN_SLAVE_HEADER_RX;
                    lin_slave.rx_index = 0;
                }
                break;
                
            case LIN_SLAVE_HEADER_RX:
                // Receiving PID
                if(LIN_ValidatePID(received_byte))
                {
                    LIN_Slave_ProcessHeader(received_byte);
                }
                else
                {
                    lin_slave.state = LIN_SLAVE_IDLE;
                    lin_slave.errors++;
                }
                break;
                
            case LIN_SLAVE_DATA_RX:
                // Receiving data
                lin_slave.rx_buffer[lin_slave.rx_index++] = received_byte;
                
                if(lin_slave.rx_index >= lin_slave.expected_length + 1) // +1 for checksum
                {
                    // Validate and process frame
                    uint8_t checksum = lin_slave.rx_buffer[lin_slave.expected_length];
                    uint8_t calc_checksum = LIN_CalculateChecksum(lin_slave.current_frame_id, 
                                                                lin_slave.rx_buffer, 
                                                                lin_slave.expected_length);
                    
                    if(calc_checksum == checksum)
                    {
                        LIN_Slave_ProcessData(lin_slave.rx_buffer, lin_slave.expected_length);
                        lin_slave.frames_received++;
                    }
                    else
                    {
                        lin_slave.errors++;
                    }
                    
                    lin_slave.state = LIN_SLAVE_IDLE;
                }
                break;
                
            default:
                lin_slave.state = LIN_SLAVE_IDLE;
                break;
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    
    if(USART_GetITStatus(USART1, USART_IT_LBD) != RESET)
    {
        // LIN Break detected
        lin_slave.state = LIN_SLAVE_IDLE;
        USART_ClearITPendingBit(USART1, USART_IT_LBD);
    }
}

void SysTick_Handler(void)
{
    system_ms++;
}

/* =============================================================================
 * LIN SLAVE FUNCTIONS - Các hàm slave
 * =============================================================================
 */
void LIN_Slave_ProcessHeader(uint8_t pid)
{
    uint8_t frame_id = pid & 0x3F;
    lin_slave.current_frame_id = frame_id;
    
    switch(frame_id)
    {
        case LIN_ID_SLAVE1_STATUS:
        case LIN_ID_SENSOR_DATA:
            // Master requests data from slave - prepare response
            lin_slave.response_pending = 1;
            lin_slave.response_timer = system_ms;
            lin_slave.state = LIN_SLAVE_IDLE;
            break;
            
        case LIN_ID_ACTUATOR_CMD:
        case LIN_ID_DIAGNOSTIC:
            // Master sends data to slave - prepare to receive
            lin_slave.state = LIN_SLAVE_DATA_RX;
            lin_slave.rx_index = 0;
            lin_slave.expected_length = (frame_id == LIN_ID_ACTUATOR_CMD) ? 2 : 4;
            break;
            
        default:
            // Unknown frame ID
            lin_slave.state = LIN_SLAVE_IDLE;
            break;
    }
}

void LIN_Slave_ProcessData(uint8_t *data, uint8_t length)
{
    switch(lin_slave.current_frame_id)
    {
        case LIN_ID_ACTUATOR_CMD:
            LIN_Slave_HandleActuatorCommand(data, length);
            break;

        case LIN_ID_DIAGNOSTIC:
            LIN_Slave_HandleDiagnostic(data, length);
            break;

        default:
            break;
    }
}

void LIN_Slave_SendResponse(uint8_t frame_id)
{
    uint8_t response_data[8];
    uint8_t response_length = 0;
    uint8_t pid = frame_id | 0x40; // Add parity bits (simplified)

    switch(frame_id)
    {
        case LIN_ID_SLAVE1_STATUS:
            // Send status data
            response_data[0] = lin_slave.node_id;
            response_data[1] = (lin_slave.digital_inputs << 4) | (lin_slave.digital_outputs & 0x0F);
            response_data[2] = (uint8_t)(lin_slave.frames_received & 0xFF);
            response_data[3] = (uint8_t)(lin_slave.errors & 0xFF);
            response_length = 4;
            break;

        case LIN_ID_SENSOR_DATA:
            // Send sensor data
            response_data[0] = (uint8_t)(lin_slave.temperature >> 8);
            response_data[1] = (uint8_t)(lin_slave.temperature & 0xFF);
            response_data[2] = (uint8_t)(lin_slave.voltage >> 8);
            response_data[3] = (uint8_t)(lin_slave.voltage & 0xFF);
            response_data[4] = lin_slave.digital_inputs;
            response_data[5] = 0x00; // Reserved
            response_length = 6;
            break;

        default:
            return; // Unknown frame ID
    }

    // Calculate checksum
    uint8_t checksum = LIN_CalculateChecksum(pid, response_data, response_length);

    // Send response
    for(uint8_t i = 0; i < response_length; i++)
    {
        USART_SendData(USART1, response_data[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }

    USART_SendData(USART1, checksum);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    lin_slave.responses_sent++;
}

void LIN_Slave_UpdateSensorData(void)
{
    static uint32_t last_update = 0;

    if(system_ms - last_update > 100) // Update every 100ms
    {
        // Read ADC for temperature and voltage
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        lin_slave.temperature = ADC_GetConversionValue(ADC1);

        // Read digital inputs
        lin_slave.digital_inputs = 0;
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_RESET)
            lin_slave.digital_inputs |= 0x01;
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
            lin_slave.digital_inputs |= 0x02;

        // Simulate voltage reading (in practice, use another ADC channel)
        lin_slave.voltage = 3300; // 3.3V in mV

        last_update = system_ms;
    }
}

void LIN_Slave_HandleActuatorCommand(uint8_t *data, uint8_t length)
{
    if(length >= 2)
    {
        uint8_t actuator_id = data[0];
        uint8_t command = data[1];

        switch(actuator_id)
        {
            case 0x01: // Digital outputs
                lin_slave.digital_outputs = command;
                break;

            case 0x02: // LED pattern
                if(command & 0x01) lin_slave.digital_outputs |= 0x01;
                else lin_slave.digital_outputs &= ~0x01;

                if(command & 0x02) lin_slave.digital_outputs |= 0x02;
                else lin_slave.digital_outputs &= ~0x02;

                if(command & 0x04) lin_slave.digital_outputs |= 0x04;
                else lin_slave.digital_outputs &= ~0x04;
                break;

            default:
                break;
        }
    }
}

void LIN_Slave_HandleDiagnostic(uint8_t *data, uint8_t length)
{
    if(length >= 1)
    {
        uint8_t diagnostic_cmd = data[0];

        switch(diagnostic_cmd)
        {
            case 0x01: // Reset counters
                lin_slave.frames_received = 0;
                lin_slave.responses_sent = 0;
                lin_slave.errors = 0;
                break;

            case 0x02: // Self test
                // Blink all LEDs
                GPIO_ResetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
                for(volatile int i = 0; i < 100000; i++);
                GPIO_SetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
                break;

            case 0x03: // Get node info
                // This would typically send a response with node information
                break;

            default:
                break;
        }
    }
}

/* =============================================================================
 * INITIALIZATION FUNCTIONS - Các hàm khởi tạo
 * =============================================================================
 */
void LIN_Slave_Init(void)
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
    USART_ITConfig(USART1, USART_IT_LBD, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

void ADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    // Configure ADC pin (PA0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure ADC
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // Configure ADC channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

    // Enable ADC
    ADC_Cmd(ADC1, ENABLE);

    // Calibrate ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000); // 1ms tick
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
