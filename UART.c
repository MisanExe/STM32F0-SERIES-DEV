#include "UART.h"

#define GPIOAEN (1U<<17)

#define PIN5	(1U<<5)

#define LED_PIN		PIN5

#define UART2ENR (1U<<17)


#define  SYS_FREQ	16000000
#define   APB1_CLK  SYS_FREQ
#define  UART_BAUDRATE 115200

static void uart_set_Baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_BD(uint32_t PeriphClk, uint32_t Baudrate);
void uart2_write(int ch);


int __io_putchar(int ch){

	uart2_write(ch);
	return ch;
}
void uart2_rxtx_init(){
	//configure  uart tx for f072 it is : PA2 = UART2_TX  @ AF1

	//1 enable clock access to gpioA
		RCC->AHBENR	|= GPIOAEN;
	//2 change mode to alternate mode on PA2 and PA3
		GPIOA->MODER &= ~(1U<<4);
		GPIOA->MODER |= (1U<<5);

		GPIOA->MODER &= ~(1U<<6);
		GPIOA->MODER |= (1U<<7);
	//3 SET PA2 ALTERNATE FUNCTION TYPE TO UART_TX : find ALTERNATE FUNCTION LOW REGISTER IN REFERNCE MANUAL

		GPIOA->AFR[0] |= (1U<<8);
		GPIOA->AFR[0] &= ~(1U<<9);
		GPIOA->AFR[0] &= ~(1U<<10);
		GPIOA->AFR[0] &= ~(1U<<11);

		//SET PA3 ALTERERNATE FUNCTION REG

		GPIOA->AFR[0] |= (1U<<12);
		GPIOA->AFR[0] &= ~(1U<<13);
		GPIOA->AFR[0] &= ~(1U<<14);
		GPIOA->AFR[0] &= ~(1U<<15);

		/*********CONFIGURE UART***************/
		//ENABLE CLOCK ACCESS
		RCC->APB1ENR |= UART2ENR;
		//BAUD RATE CONFIG
		uart_set_Baudrate(USART2, APB1_CLK, UART_BAUDRATE);

		//Configure transfer direction : UART CONTROL REGISTER, BIT 3 enables transfer. we're enabling the transmitter AND RECEIVER
		USART2->CR1 = (1U<<3) | (1U<<2);

		//configure word length at the CR2 we can specify start and stop bit
			// NOT REQUIRED FOR OUR APPLICATION

		// THEN WE NEED TO ENABLE THE UART AT CR1, UE(UART ENABLE)
		USART2->CR1 |= (1U<<0);


}


char uart2_read(void){
	while( !(USART2->ISR & (1U<<5)) ){}

	return USART2->RDR;
}

void uart2_write(int ch){
	//make sure tx data register is empty : USART2 ISR, TXE TO KNOW IF IT IS EMPTY
		while( !(USART2->ISR & (1U<<7) ) ){}
	//write to transmit data register
		USART2->TDR  = (ch);
}
static void uart_set_Baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate){

	USARTx->BRR = compute_uart_BD(PeriphClk, BaudRate);
}


static uint16_t compute_uart_BD(uint32_t PeriphClk, uint32_t Baudrate){

	return (PeriphClk + (Baudrate/2U)/ Baudrate);
}
