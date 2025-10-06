#include "uart.h"
#include "systick.h"
#include "system_stm32f10x.h"

uint8_t* data = (uint8_t *)"Hello, UART!\r\n";
extern UART_HandleTypeDef huart1;
int main(void)
{
    // Initialize system
    SystemInit();
    SysTick_Init();
    
    // Initialize UART
    UART_Config(&huart1);
    
    // Main loop
    while (1)
    {
        // Example: Send a number every second
        UART_SendString(&huart1, "Current Time (ms): ");
        UART_SendNumber(&huart1, SysTick_GetTick());
        UART_SendString(&huart1, "\r\n");
        
        SysTick_DelayMs(1000);
    }
}