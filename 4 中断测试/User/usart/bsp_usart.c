#include "bsp_usart.h"

/**
  * @brief 串口重定义
  */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&USART_X, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&USART_X, &ch, 1, 0xffff);
    return ch;
}
