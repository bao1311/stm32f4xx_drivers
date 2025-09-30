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
#include <string.h>

USART_Handle_t USARTHandle;
GPIO_Handle_t GPIOHandle;
char message[][50] = {"I am Bao", "I learn embedded system", "how are you"};
char* rcv_buf = "";
void delay()
{
	for (int i = 0; i < 1000000; i++)
	{
		;
	}
}
void USART2_Init()
{
	USARTHandle.pUSARTx = USART2;
	USARTHandle.USART_Config.USART_Baud = 115200;
	USARTHandle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USARTHandle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USARTHandle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USARTHandle.USART_Config.USART_ParityControl = USART_PARITY_DI;
	USARTHandle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Init(&USARTHandle);

}
void USART2_GPIOInits()
{
	// Common declaration
	GPIOHandle.pGPIOx = GPIOA;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	// PA2
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = 2;
	GPIO_Init(&GPIOHandle);
	// PA3
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = 3;
	GPIO_Init(&GPIOHandle);
}

void GPIOBtn_Init()
{
	GPIOHandle.pGPIOx = GPIOA;

	GPIOHandle.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&GPIOHandle);

}
void USART2_IRQHandler(void)
{
	return USART_EV_IRQHandling(&USARTHandle);
}
int main()
{
	USART2_GPIOInits();
	USART2_Init();
	USART_PeripheralControl(USART2, ENABLE);
	USART_PeriClockControl(USART2, ENABLE);

	GPIOBtn_Init();
	while (1)
	{
		while (!GPIO_ReadFromInputPin(GPIOA, 0));
		// For debouncing
		delay();

		while (USART_MasterReceiveDataIT(&USARTHandle, rcv_buf, strlen(message[0])) != USART_READY);

		USART_MasterSendData(&USARTHandle, message[0], strlen(message[0]));
	}

}

