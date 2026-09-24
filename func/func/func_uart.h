#ifndef __FUNC_UART_H__
#define __FUNC_UART_H__

#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define UART_1 &huart6
#define UART_2 &huart1

void func_uart_transmit(uint8_t* txBuffer,uint16_t len);
void func_uart_init(void);
uint8_t func_uart_getValue(float *f1, float *f2, float *f3, float *f4, uint8_t *val , uint8_t *mod);

#endif
