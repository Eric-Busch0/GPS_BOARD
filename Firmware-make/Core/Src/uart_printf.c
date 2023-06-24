#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "uart_printf.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

char print_buf[256] = {0};

uint8_t uart_printf(const char *fmt, ...)
{

    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(print_buf, 128, fmt, ap);
    va_end(ap);
    return HAL_UART_Transmit(&huart1, (uint8_t *)print_buf, strlen(print_buf), HAL_MAX_DELAY);
}