/*
 * 006spi_tx_testing.c
 *
 *  Created on: Aug 17, 2025
 *      Author: gphi1
 */
#include <stdio.h>
#include <stm32f407xx.h>
/*
 * Requirements: Print hello world string through SPI2 peripheral
 */

/*
 * Alternate function:
 * PB12 		-> SPI2_NSS
 * PB13			-> SPI2_SCK
 * PB14			-> SPI2_MISO
 * PB15			-> SPI2_MOSI
 * Alternate function mode 5
 */

void SPI_GPIOInits()
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;



	// SPI2_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(SPIPins);

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(SPIPins);

	// SPI2_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(SPIPins);

	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(SPIPins);
}

int main(void)
{
	SPI_GPIOInits();
	printf("Hello world");
	return 0;
}

