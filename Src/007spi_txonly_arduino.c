/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Aug 26, 2025
 *      Author: gphi1
 */


#include <stdio.h>
#include <string.h>
#include <stm32f407xx.h>
void delay()
{
	for (uint32_t i = 0; i < 100000; i += 1)
	{

	}
}

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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

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
	// Generate clock speed of 2 MHz
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	// Hardware slave management enable for NSS pin
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	// 3. Call SPI Init
	SPI_Init(&SPI2Handle);

}

void GPIOBtn_Init(void)
{
	// Configure the button
	GPIO_Handle_t gpioButton;
	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);
}

int main(void)
{
	char user_data[] = "Hello, world";
	SPI2_GPIOInits();

	SPI2_Inits();
	// Avoid MODF error by toggling SSI bit
//	SPI_SSIConfig(SPI2, ENABLE);
	// Configure SSOE bit for NSS configuration (NSS output enable (SSM=0,SSOE=1))
	SPI_SSOEConfig(SPI2, ENABLE);
	// Send data
	SPI_PeripheralControl(SPI2, ENABLE);
	while (1)
	{
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

