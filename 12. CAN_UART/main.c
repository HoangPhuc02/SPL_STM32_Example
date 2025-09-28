#include "uart.h"
#include "systick.h"


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
        UART_SendString("Current Time (ms): ");
        UART_SendNumber(SysTick_GetTick());
        UART_SendString("\r\n");
        
        SysTick_DelayMs(1000);
    }
}