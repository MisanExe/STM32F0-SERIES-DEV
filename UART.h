/*
 * UART.h
 *
 *  Created on: Dec 4, 2021
 *      Author: wizzp
 */

#ifndef UART_H_
#define UART_H_
#include "stm32f072xb.h"
#include <stdint.h>
void uart2_tx_init();
char uart2_read(void);
void uart2_rxtx_init();



#endif /* UART_H_ */
