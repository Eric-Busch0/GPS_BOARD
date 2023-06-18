#ifndef UART_PRINTF_H
#define UART_PRINTF_H

#include <stdint.h>
#ifdef __cplusplus
extern "C"{
#endif

//Place c code here

uint8_t uart_printf(const char *fmt, ...);



#ifdef __cplusplus
}
#endif

#endif