
#include "UART.h"
#include <stdio.h>

#define GPIOAEN (1U<<17)
#define PIN5	(1U<<5)
#define LED_PIN		PIN5


char key;

int main(void){

	RCC->AHBENR |= GPIOAEN;

	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &= ~(1U<<11);

	uart2_rxtx_init();

	while(1){
		key = uart2_read();
		if (key == '1'){
			GPIOA->ODR |= LED_PIN;
		}else{
			GPIOA->ODR &= ~LED_PIN;
		}
	}
}


