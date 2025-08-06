/************************************************************************
 *                          Utility Functions                           *
*************************************************************************/

/* ======================= UART Utility Functions ======================= */
/**
 * @brief Send a byte via UART
 */
void UART_SendByte(uint8_t byte)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

/**
 * @brief Send a string via UART
 */
void UART_SendString(char* str)
{
    while(*str)
    {
        UART_SendByte(*str++);
    }
}

/**
 * @brief Send a hex value via UART
 */
void UART_SendHex(uint8_t value)
{
    uint8_t high = (value >> 4) & 0x0F;
    uint8_t low = value & 0x0F;

    UART_SendByte(high < 10 ? '0' + high : 'A' + high - 10);
    UART_SendByte(low < 10 ? '0' + low : 'A' + low - 10);
}