/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 16, 2025
 *      Author: gphi1
 */

#include "stm32f407xx_spi_driver.h"
#ifndef SRC_STM32F407XX_SPI_DRIVER_C_
#define SRC_STM32F407XX_SPI_DRIVER_C_

void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	// 0. Enable SPI Peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	uint32_t tempreg = 0;
	// 1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	// 2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// bidi should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// bidi should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the serial clock
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	// 4. Configure the data format frame
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	// 5. Configure the clock polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	// 6. Configure the clock phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	pSPIHandle->pSPIx->CR1 = tempreg;
}
void SPI_DeInit(SPI_RegDef_t* pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_PCLK_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_PCLK_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_PCLK_RESET();
	}

}
/*
 * API for SPI Peripheral Clock Setup
 */

/******************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enable or disable for the given SPI
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx== SPI3) {
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if (pSPIx== SPI3) {
			SPI3_PCLK_DI();
		}
	}
}
/*
 * Data send and receive
 */


/******************************************
 * @fn			- SPI_GetFlagStatus
 *
 * @brief		- This function get the flag status of a specific SPI register
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Flag name
 *
 * @return
 * @Note:
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint8_t FlagName)
{
	if (pSPIx->SR & FlagName)
		return FLAG_SET;
	return FLAG_RESET;

}

/******************************************
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- This function enable SPI, no more change in configuration from here
 * @param[in]	- SPI1, 2 or 3
 * @param[in]	- Enable or Disable variable
 *
 * @return
 */
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/******************************************
 * @fn			- SPI_SendData
 *
 * @brief		- This function sends data through SPI peripheral
 * @param[in]	- SPI1, 2 or 3
 * @param[in]	- pTxBuffer
 * @param[in]	- Length of the data being transmitted
 *
 * @return
 * @Note:	This is a blocking call
 */
/*
 * API for SPI Data Send
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// 1. Wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);


		// 2. Check the DFF bit
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			// 3. This section is for sending 16 bits of data
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			--Len;
			--Len;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 3. This section is for sending 8 bits of data
			pSPIx->DR = *((uint8_t*) pTxBuffer);
			--Len;
			pTxBuffer++;
		}


	}

}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQPriorityHandling(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t* pHandle);



#endif /* SRC_STM32F407XX_SPI_DRIVER_C_ */
