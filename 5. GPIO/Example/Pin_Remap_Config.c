#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#define LIN_BREAK_DELAY      13      // 13 bit times for break detection
#define LIN_MAX_FRAME_SIZE   10      // Sync + PID + 8 data + checksum
#define LIN_TIMEOUT          1000    // Timeout for frame reception

typedef struct {
    uint8_t sync;
    uint8_t pid;
    uint8_t data[8];
    uint8_t length;
    uint8_t checksum;
    uint8_t status; // 0: idle, 1: receiving, 2: complete, 3: error
} LIN_Frame;

volatile LIN_Frame lin_rx_frame;
volatile uint8_t lin_rx_buffer[LIN_MAX_FRAME_SIZE];
volatile uint8_t lin_rx_index = 0;
volatile uint32_t lin_rx_timer = 0;

void LIN_Init(void);
void LIN_SendBreak(void);
void LIN_SendFrame(uint8_t pid, uint8_t *data, uint8_t length);
uint8_t LIN_CheckPID(uint8_t pid);
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length);
void LIN_ReceiveHandler(void);
uint8_t LIN_CheckFrame(LIN_Frame *frame);

int main(void)
{
    uint8_t data[2] = {0x55, 0xAA};
    
    // Initialize system and LIN
    SystemInit();
    LIN_Init();
    
    // Enable USART1 interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    
    while(1)
    {
        // Check if a complete frame was received
        if(lin_rx_frame.status == 2) // Complete frame
        {
            if(LIN_CheckFrame(&lin_rx_frame))
            {
                // Valid frame received - process it here
                // For example: toggle LED based on received data
                if(lin_rx_frame.pid == 0x30)
                {
                    GPIO_WriteBit(GPIOC, GPIO_Pin_13, 
                                (lin_rx_frame.data[0] == 0x55) ? Bit_RESET : Bit_SET);
                }
            }
            
            // Reset frame status
            lin_rx_frame.status = 0;
        }
        
        // Send LIN frame periodically
        static uint32_t last_send = 0;
        if(SystemCoreClock - last_send > 1000000)
        {
            LIN_SendFrame(0x30, data, 2);
            last_send = SystemCoreClock;
            
            // Toggle data for next send
            data[0] ^= 0xFF;
            data[1] ^= 0xFF;
        }
    }
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received = USART_ReceiveData(USART1);
        
        // Reset timeout timer
        lin_rx_timer = SystemCoreClock;
        
        // Check for break (0x00) - start of frame
        if(received == 0x00 && lin_rx_frame.status == 0)
        {
            lin_rx_frame.status = 1; // Start receiving
            lin_rx_index = 0;
            lin_rx_buffer[lin_rx_index++] = received;
        }
        else if(lin_rx_frame.status == 1) // Receiving frame
        {
            lin_rx_buffer[lin_rx_index++] = received;
            
            // Check for complete frame (minimum: sync + pid + checksum)
            if(lin_rx_index >= 3)
            {
                // Check if we've received all data bytes
                uint8_t expected_length = 3; // sync + pid + checksum
                if(lin_rx_index >= expected_length)
                {
                    // Parse received frame
                    lin_rx_frame.sync = lin_rx_buffer[0];
                    lin_rx_frame.pid = lin_rx_buffer[1];
                    
                    // Data length depends on application (assuming 2 bytes here)
                    lin_rx_frame.length = 2;
                    lin_rx_frame.data[0] = lin_rx_buffer[2];
                    lin_rx_frame.data[1] = lin_rx_buffer[3];
                    lin_rx_frame.checksum = lin_rx_buffer[4];
                    
                    lin_rx_frame.status = 2; // Frame complete
                }
            }
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

uint8_t LIN_CheckFrame(LIN_Frame *frame)
{
    // Verify sync byte
    if(frame->sync != 0x55)
        return 0;
    
    // Verify PID parity
    uint8_t pid = LIN_CheckPID(frame->pid & 0x3F);
    if(pid != frame->pid)
        return 0;
    
    // Verify checksum
    uint8_t calc_checksum = LIN_CalculateChecksum(frame->pid, frame->data, frame->length);
    if(calc_checksum != frame->checksum)
        return 0;
    
    return 1; // Frame is valid
}

uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t length)
{
    uint16_t checksum = pid;
    
    // Classic LIN 2.0 checksum (PID + data)
    for(int i = 0; i < length; i++)
    {
        checksum += data[i];
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }
    
    return ~checksum; // Invert the result
}

void LIN_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    // Configure LED on PC13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    // Configure USART1 Tx (PA9) and Rx (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART configuration for LIN
    USART_InitStructure.USART_BaudRate = 9600; // LIN standard baud rate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure);
    
    // Enable USART1 RX interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
}

// Các hàm LIN_SendBreak(), LIN_SendFrame(), LIN_CheckPID() giữ nguyên như ví dụ trước