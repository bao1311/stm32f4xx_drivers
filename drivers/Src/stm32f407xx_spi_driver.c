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

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t Len)
{
	// 1. Check if there is still data to receive
	while (Len > 0)
	{
		// 2. Wait until RXNE is set (Receive buffer is not empty)
		while (SPI_GetFlagStatus(pSPIx, 1 << SPI_SR_RXNE) == FLAG_SET);
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// DFF == 1bit => transfer 16 bits at a time
			*pRxBuffer = *((uint16_t*)pSPIx->DR);
			(uint16_t*)pRxBuffer++;
			--Len;
			--Len;

		}
		else
		{
			// bit DFF == 0 => Transfer 8 bits at a time
			*pRxBuffer = (pSPIx->DR);
			pRxBuffer++;
			Len--;

		}
	}

}

/*
 * Data send and receive (Interrupt version)
 */
uint8_t SPI_SendDataIT(SPI_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len)
{
	if (pHandle->TxState != SPI_BUSY_IN_TX)
	{

		// 1. Save pTxBuffer address and Len information in some global info

		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no
		// other code can take over same SPI peripheral until transmission is over
		pHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable TXEIE control bit to get the interrupt whenever
		// TXE flag is set in SR
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data Transmission will be handled by ISR code (implement later)

	}
	return pHandle->TxState;

}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t Len)
{
	if (pHandle->RxState != SPI_BUSY_IN_RX)
	{

		// 1. Save pRxBuffer address and Len information in some global info

		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no
		// other code can take over same SPI peripheral until transmission is over
		pHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable RXEIE control bit to get the interrupt whenever
		// RXE flag is set in SR
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data Transmission will be handled by ISR code (implement later)

	}
	return pHandle->RxState;
}

static void spi_txe_interrupt_handle(SPI_Handle_t* pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t* pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t* pHandle);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t* pHandle)
{
	// 1. Finding the root cause of the interrupt first
	uint32_t temp1, temp2;
	// 2. Check if root cause of interrupt is due to TXE
	temp1 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	temp2 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	if (temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}
	// 3. Check if root cause of interrupt is due to RXE
	temp1 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	temp2 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	if (temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}
	// 4. Check if root cause of interrupt is due to overrun error
	temp1 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	temp2 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	if (temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

/*
 * Some helper functions for SPI driver code
 */

static void spi_txe_interrupt_handle(SPI_Handle_t* pHandle)
{
	if (pHandle->TxLen > 0)
	{
		uint32_t CR1 = pHandle->pSPIx->CR1;
		if (CR1 & (1 << SPI_CR1_DFF))
		{
			// DFF bit == 1 which means the data is
			// transfered 16 bits at a time
			pHandle->pSPIx->DR = *((uint16_t*)pHandle->pTxBuffer);
			pHandle->TxLen -= 2;
			(uint16_t*)pHandle->pTxBuffer++;
		}
		else
		{
			// DFF bit == 0 which means the data is transferred
			// 8 bits at a time
			pHandle->pSPIx->DR = *(pHandle->pTxBuffer);
			pHandle->TxLen -= 1;
			pHandle->pTxBuffer++;
		}
	}
	// 2. Reset pHandle for TX and Application Event call back
	if (pHandle->TxLen == 0)
	{
		SPI_CloseTransmission(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);


	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t* pHandle)
{
	// 1. Check if RxLen > 0 or equal to 0
	if (pHandle->RxLen > 0)
	{
		// 2. Check if DFF is 1 or 0
		if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 3. In case DFF is 1, data transfer will be 16 bits
			*((uint16_t*)pHandle->pRxBuffer) = (uint16_t)pHandle->pSPIx->DR;
			(uint16_t*)pHandle->pRxBuffer++;
			pHandle->RxLen -= 2;

		}
		else
		{
			*(pHandle->pRxBuffer) = pHandle->pSPIx->DR;
			pHandle->pRxBuffer++;
			pHandle->RxLen -= 1;

		}

	}
	if (!pHandle->RxLen)
	{
		// 4. Reset pHandle for RX and application event call back
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX_CMPLT);

	}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t* pHandle)
{
	/*
	 * Handle overrun by  clearing the OVR bit
	 */
	uint8_t temp;
	// Only clear the OVR flag if the device is not busy transmitting
	if (pHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_OVR_ERR);

}

void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}
void SPI_CloseTransmission(SPI_Handle_t* pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pHandle->pTxBuffer = NULL;
	pHandle->TxState = SPI_READY;
	pHandle->TxLen = 0;

}
void SPI_CloseReception(SPI_Handle_t* pHandle)
{
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
	pHandle->pRxBuffer = NULL;
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t* pHandle, uint8_t AppEv)
{

}


#endif /* SRC_STM32F407XX_SPI_DRIVER_C_ */
