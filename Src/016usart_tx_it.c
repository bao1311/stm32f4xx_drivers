/*
 * 016usart_tx_it.c
 *
 *  Created on: Sep 29, 2025
 *      Author: gphi1
 */

// Code for testing USART interrupt mode
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

USART_Handle_t USARTHandle;
GPIO_Handle_t GPIOHandle;
char message[][50] = {"I am Bao", "I learn embedded system", "how are you"};
int main()
{
	USART2_GPIOInits();
	USART_PeripheralControl(USART2, ENABLE);
	USART_PeriClockControl(USART2, ENABLE);

	GPIOBtn_Init();
	while (1)
	{
		while (GPIO_ReadFromInputPin(GPIOA, 0));

		while (USART_MasterReceiveDataIT(&USARTHandle, message, strlen(message)));
	}

}

