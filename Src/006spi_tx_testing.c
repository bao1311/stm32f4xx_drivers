/*
 * 006spi_tx_testing.c
 *
 *  Created on: Aug 17, 2025
 *      Author: gphi1
 */
#include <stdio.h>
#include <string.h>
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

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	// 1. Enable GPIO peripheral clock
	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;



	// SPI2_NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(SPIPins.pGPIOx);

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
//	GPIO_Init(SPIPins.pGPIOx);
//	GPIO_Init(pGPIOHandle);
	GPIO_Init(&SPIPins);

	// SPI2_MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(SPIPins.pGPIOx);

	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	// 1. Configure SPI2 peri
	SPI2Handle.pSPIx = SPI2;
	// 2. Configure SPI2 config
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_DFF  = SPI_DFF_8BITS;
	// Generate clock speed of 8 MHz
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	// Software slave management enable for NSS pin
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	// 3. Call SPI Init
	SPI_Init(&SPI2Handle);

}

int main(void)
{
	char user_data[] = "Hello, world";
	SPI2_GPIOInits();
	SPI2_Inits();
	// Send data
	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	printf("Hello world");

	while(1);
	return 0;
}

