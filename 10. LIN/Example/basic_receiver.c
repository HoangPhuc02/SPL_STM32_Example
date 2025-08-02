#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

// LIN Protocol defines
#define LIN_SYNC_BYTE           0x55                // LIN sync byte value
#define LIN_BREAK_THRESHOLD     10                  // Break detection threshold (not used directly)
#define LIN_FRAME_MAX_SIZE      8                   // Maximum number of data bytes in a LIN frame
#define TARGET_ID               0x30                // Target ID to trigger LED

// LIN Frame States
typedef enum {
    LIN_STATE_IDLE,                 // Waiting for LIN frame
    LIN_STATE_BREAK_DETECTED,       // Break detected, start of frame
    LIN_STATE_SYNC_RECEIVED,        // Sync byte received
    LIN_STATE_PID_RECEIVED,         // PID received, waiting for data
    LIN_STATE_DATA_RECEIVING,       // Receiving data bytes
    LIN_STATE_CHECKSUM_RECEIVED     // Checksum received, frame complete
} LIN_State_t;

// LIN Frame Structure
typedef struct {
    uint8_t sync;                       // Sync byte (0x55)
    uint8_t pid;                        // Protected Identifier (PID)
    uint8_t data[LIN_FRAME_MAX_SIZE];   // Data bytes
    uint8_t data_length;                // Number of data bytes
    uint8_t checksum;                   // Received checksum
    uint8_t calculated_checksum;        // Calculated checksum for verification
} LIN_Frame_t;

// Global variables
static volatile LIN_State_t lin_state = LIN_STATE_IDLE; // Current LIN state
static volatile LIN_Frame_t lin_frame;                  // Current LIN frame
static volatile uint8_t data_index = 0;                 // Index for data bytes
static volatile uint32_t break_timer = 0;               // Timer for break detection (not used)

// Function prototypes
void System_Init(void);                                 // System initialization
void GPIO_Configuration(void);                          // GPIO setup
void UART_Configuration(void);                          // USART setup
void NVIC_Configuration(void);                          // NVIC interrupt setup
void Toggle_LED_C13(void);                              // Toggle LED on PC13
uint8_t Calculate_LIN_Checksum(uint8_t pid, uint8_t *data, uint8_t length); // LIN checksum calculation
uint8_t Check_PID_Parity(uint8_t pid);                  // Check PID parity bits
void Process_LIN_Frame(void);                           // Process received LIN frame

int main(void)
{
    System_Init();      // Initialize system peripherals
    
    while (1)
    {
        // Main loop - everything handled in interrupt
        __WFI();        // Wait for interrupt (low power)
    }
}

// Initialize clocks, GPIO, UART, NVIC
void System_Init(void)
{
    // Enable clocks for GPIOA, GPIOC, USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | 
                          RCC_APB2Periph_USART1, ENABLE);
    
    GPIO_Configuration();   // Setup GPIO pins
    UART_Configuration();   // Setup USART1 for LIN
    NVIC_Configuration();   // Setup NVIC for USART1 interrupt
}

// Configure GPIO pins for LED and USART1
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Configure LED PC13 as output push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // Turn off LED initially (active low)
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    // Configure USART1 TX (PA9) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure USART1 RX (PA10) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// Configure USART1 for LIN communication
void UART_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    
    // Set baud rate to 9600 (typical for LIN)
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure);   // Apply configuration
    
    // Enable LIN mode and set break detection length to 10 bits
    USART_LINCmd(USART1, ENABLE);
    // USART_LINBreakDetectLengthConfig(USART1, USART_LINBreakDetectLength_10b);
    // Enable RXNE (receive) and LBD (LIN break detection) interrupts
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_LBD, ENABLE);
    
    // Enable USART1 peripheral
    USART_Cmd(USART1, ENABLE);
}

// Configure NVIC for USART1 interrupt
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // Set USART1 interrupt priority and enable
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// USART1 interrupt handler: handles LIN frame reception
void USART1_IRQHandler(void)
{
    // Check for LIN Break Detection interrupt
    if (USART_GetITStatus(USART1, USART_IT_LBD) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_LBD); // Clear interrupt flag
        
        // Break detected - start of new LIN frame
        lin_state = LIN_STATE_BREAK_DETECTED;
        data_index = 0;
        
        // Clear frame data
        lin_frame.sync = 0;
        lin_frame.pid = 0;
        lin_frame.data_length = 0;
        lin_frame.checksum = 0;
        lin_frame.calculated_checksum = 0;
    }
    
    // Check for received data interrupt
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART1); // Read received byte
        
        switch (lin_state)
        {
            case LIN_STATE_BREAK_DETECTED:
                // After break, expect sync byte
                lin_state = LIN_STATE_SYNC_RECEIVED;
                break;
                
            case LIN_STATE_SYNC_RECEIVED:
                // After sync, expect PID
                lin_frame.pid = received_byte;
                
                // Check PID parity
                if (Check_PID_Parity(received_byte))
                {
                    lin_frame.data_length = LIN_FRAME_MAX_SIZE; // Get expected data length
                    lin_state = LIN_STATE_PID_RECEIVED;
                    data_index = 0;
                }
                else
                {
                    lin_state = LIN_STATE_IDLE; // Invalid PID, reset state
                }
                break;
                
            case LIN_STATE_PID_RECEIVED:
            case LIN_STATE_DATA_RECEIVING:
                // Receive data bytes
                if (data_index < lin_frame.data_length)
                {
                    lin_frame.data[data_index] = received_byte;
                    data_index++;
                    lin_state = LIN_STATE_DATA_RECEIVING;
                }
                else
                {
                    // After data, next byte is checksum
                    lin_frame.checksum = received_byte;
                    lin_state = LIN_STATE_CHECKSUM_RECEIVED;
                    Process_LIN_Frame(); // Process complete frame
                }
                break;
                
            default:
                lin_state = LIN_STATE_IDLE; // Any other state, reset
                break;
        }
    }
}

// Process received LIN frame: verify checksum and toggle LED if target ID
void Process_LIN_Frame(void)
{
    // Calculate expected checksum (LIN 2.x: includes PID)
    lin_frame.calculated_checksum = Calculate_LIN_Checksum(lin_frame.pid, 
                                                          (uint8_t*)lin_frame.data, 
                                                          lin_frame.data_length);
    
    // Check if received checksum matches calculated checksum
    if (lin_frame.checksum == lin_frame.calculated_checksum)
    {
        // Extract ID from PID
        uint8_t id = Extract_ID_From_PID(lin_frame.pid);
        
        // If ID matches target, toggle LED
        if (id == TARGET_ID)
        {
            Toggle_LED_C13();
        }
    }
    
    // Reset state for next frame
    lin_state = LIN_STATE_IDLE;
}

// Calculate LIN checksum (enhanced for LIN 2.x, includes PID)
uint8_t Calculate_LIN_Checksum(uint8_t pid, uint8_t *data, uint8_t length)
{
    uint16_t sum = 0;
    uint8_t i;
    
    // LIN 2.x uses enhanced checksum (includes PID)
    // For LIN 1.x, comment out the next line
    sum += Extract_ID_From_PID(pid);
    
    // Add all data bytes
    for (i = 0; i < length; i++)
    {
        sum += data[i];
        
        // Handle carry (add carry to sum)
        if (sum > 0xFF)
        {
            sum = (sum & 0xFF) + 1;
        }
    }
    
    // Invert result for checksum
    return (uint8_t)(~sum);
}



// Extract ID from PID (bits 5-0)
uint8_t Extract_ID_From_PID(uint8_t pid)
{
    return (pid & 0x3F);
}

// Check PID parity bits (bits 6 and 7)
uint8_t Check_PID_Parity(uint8_t pid)
{
    uint8_t id = Extract_ID_From_PID(pid);
    uint8_t p0, p1;
    
    // Calculate parity bits according to LIN specification
    p0 = (id & 0x01) ^ ((id >> 1) & 0x01) ^ ((id >> 2) & 0x01) ^ ((id >> 4) & 0x01);
    p1 = ~(((id >> 1) & 0x01) ^ ((id >> 3) & 0x01) ^ ((id >> 4) & 0x01) ^ ((id >> 5) & 0x01)) & 0x01;
    
    // Check if calculated parity matches received parity bits
    uint8_t expected_pid = id | (p0 << 6) | (p1 << 7);
    
    return (pid == expected_pid);
}

// Toggle LED on PC13 (active low)
void Toggle_LED_C13(void)
{
    static uint8_t led_state = 0;
    
    if (led_state)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);   // Turn off LED
        led_state = 0;
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Turn on LED
        led_state = 1;
    }
}
