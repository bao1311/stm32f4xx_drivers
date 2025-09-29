/*
 * 015usart_tx.c
 *
 *  Created on: Sep 29, 2025
 *      Author: gphi1
 */
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_gpio_driver.h"
//#include "stm32f407xx.h"

char *message = "Hello, I am Bao";
USART_Handle_t USARTHandle;

GPIO_Handle_t GPIOHandle;
void USART_GPIOInits()
{
	/*
	 * Set up GPIO Pin for TX and RX pin of USART peripheral
	 * USART2
	 * TX: PA2
	 * RX: PA3
	 */
	// Set up PA2 alternate function mode for USART2's TX
	GPIOHandle.pGPIOx = GPIOA;

	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&GPIOHandle);

	// PA3
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&GPIOHandle);

}
//void USART_Init();


//void GPIO_Init();
int main(void)
{
	// USART_GPIO Pin Initialization
	USART_GPIOInits();

	// USART configuration
	USARTHandle.pUSARTx = USART2;
	// USART Init
	USART_Init(&USARTHandle);


//	GPIO_BtnInit();

	while (1)
	{
		while (!GPIO_ReadFromInputPin(GPIOA, 8));

		USART_MasterSendData(pUSARTHandle, message, sizeof(message), SLAVE_ADDR);
	}


}

