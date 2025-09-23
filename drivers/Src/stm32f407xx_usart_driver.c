/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Sep 16, 2025
 *      Author: gphi1
 */
#include "stm32f407xx_usart_driver.h"


/*
 * ******************************************
 * @fn			- USART_SetBaudRate
 *
 * @brief 		- This function configure the
 * BRR register so USART peripheral can get
 * its desired baud rate
 *
 * @param[in]	- USART Handle pointer
 * @param[in]	- Baud Rate value
 *
 * @return
 * @Note:
 * ******************************************
 */
void USART_SetBaudRate(USART_Handle_t* pUSARTHandle, uint32_t BaudRate)
{
	/*
	 * In order to configure the Fraction and Mantissa of the BRR
	 * register, we need to figure out the USART_DIV
	 */
	// APB2: USART1 and USART6
	// APB1; USART2, USART3, UART4, and UART5
	uint32_t PCLKx;
	if (pUSARTHandle->pUSARTx == USART1 || pUSARTHandle->pUSARTx == USART6)
	{
		// APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		// APB1 bus
		PCLKx = RCC_GetPCLK1Value();
	}
	// Fraction part
	uint8_t F;
	// Mantissa part
	uint16_t M;
	// USARTDIV part
	uint32_t USARTDIV;
	// OVER8 bit
	uint8_t OVER8 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_OVER8);
	// tempreg for setting up BRR register
	uint32_t tempreg = 0;
	// Calculate USARTDIV
	USARTDIV = 100 * PCLKx / (8 * (2 - OVER8) * BaudRate);

	// Calculate Mantissa part (M)
	M = USARTDIV / 100;
	// Set up the Mantissa part of tempreg
	tempreg |= (M << 4);
	// Calculate Fraction part (F)
	F = USARTDIV % 100;
	// Set up the Fraction part of tempreg
	tempreg |= F;
	// Set up the BRR register
	pUSARTHandle->pUSARTx->BRR = tempreg;
}
/******************************************
 * @fn			- USART_Init
 *
 * @brief		- This function initialise the USART
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */
void USART_Init(USART_Handle_t* pUSARTHandle)
{
	uint32_t tempreg = 0;
	// Set up USART_MODE
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= (1 << USART_CR1_TE);
		tempreg |= (1 << USART_CR1_TE);
	}
	pUSARTHandle->pUSARTx->CR1 |= tempreg;

	// Set up USART_Baud
	USART_SetBaudRate(pUSARTHandle, pUSARTHandle->USART_Config.USART_Baud);

	// Set up USART_NoOfStopBits
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_1)
	{
		tempreg |= (0 << USART_CR2_STOP);
	}
	else if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_0_5)
	{
		tempreg |= (1 << USART_CR2_STOP);
	}
	else if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_2)
	{
		tempreg |= (2 << USART_CR2_STOP);
	}
	else if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_1_5)
	{
		tempreg |= (3 << USART_CR2_STOP);
	}
	pUSARTHandle->pUSARTx->CR2 |= tempreg;


	// USART_WordLength
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
	{
		tempreg |= (0 << USART_CR1_M);
	}
	else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		tempreg |= (1 << USART_CR1_M);
	}
	pUSARTHandle->pUSARTx->CR1 |= tempreg;
	// USART_ParityControl
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DI)
	{
		tempreg |= (0 << USART_CR1_PCE);
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		// Even Parity
		tempreg |= (1 << USART_CR1_PCE);
		tempreg |= (0 << USART_CR1_PS);
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		// Odd Parity
		tempreg |= (1 << USART_CR1_PCE);
		tempreg |= (1 << USART_CR1_PS);
	}
	pUSARTHandle->pUSARTx->CR1 |= tempreg;

	// USART_HWFlowControl
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_NONE)
	{
		tempreg &= ~(1 << USART_CR3_RTSE);
		tempreg &= ~(1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// CTS enable
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// RTS enable
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// Both CTS and RTS enable
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);

	}
	pUSARTHandle->pUSARTx->CR3 |= tempreg;

}
void USART_DeInit(USART_RegDef_t* pUSARTx); 		// Cando
/*
 * API for USART Peripheral Clock Setup
 */

/******************************************
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This function enable or disable for the given USART
 * @param[in]	- Base address of the USART peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */

void USART_PeriClockControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}


/*
 * Data send and receive (Blocking version)
 */
void USART_MasterSendData(USART_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	while (Len > 0)
	{
		// Check if Transmit DR is empty
		while (!USART_GetFlagStatus(pHandle->pUSARTx, USART_TXE_FLAG));

		// 2 Cases for M Word bit: 8 or 9 bits
		if (pHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
		{
			/*
			 * WORDLEN 8 BITS CASE
			 */
			pHandle->pUSARTx->DR = *pTxBuffer;
			pTxBuffer++;
		}
		else if (pHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/*
			 * WORDLEN 9 BITS CASE
			 */

			// Check for parity configuration
			uint16_t *pValue;
			// Loading 2 bytes into DR register and masking it
			pValue = (uint16_t*) pTxBuffer;
			pHandle->pUSARTx->DR = *pValue & (0x1FF);
			if (pHandle->USART_Config.USART_ParityControl == USART_PARITY_DI)
			{
				// If Parity Enabled
				// Data format: 9 bits of data
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// Data format: 8 bits of data + 1 bit of Parity Control (given by hardware)
				pTxBuffer++;

			}

		}

		Len--;
	}
	// Wait until Transaction complete by checking TC bit in SR

}
void USART_MasterReceiveData(USART_Handle_t* pUSARTx, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr);


/*
 * Data send and receive (Interrupt version)
 */
uint8_t USART_MasterSendDataIT(USART_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t USART_MasterReceiveDataIT(USART_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Data send and receive close communication
 */
void USART_CloseSendData(USART_Handle_t* pUSARTHandle);
void USART_CloseReceiveData(USART_Handle_t* pUSARTHandle);

/*                     /
 * USART Application/ States Macros
 */
#define USART_READY				0
#define USART_BUSY_IN_RX			1
#define USART_BUSY_IN_TX			2


/*
 * IRQ Configuration and ISR Handling
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shiftedAmt = 8 * iprx_section + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority) << (shiftedAmt);
}
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) //Cando
{
	if (EnorDi == ENABLE)
	{
		uint8_t pos = IRQNumber % 32;
		uint8_t offset = IRQNumber / 32;
		if (offset == 0)
		{
			*NVIC_ISER0 |= (1 << pos);
		}
		else if (offset == 1)
		{
			*NVIC_ISER1 |= (1 << pos);
		}
		else if (offset == 2)
		{
			*NVIC_ISER2 |= (1 << pos);
		}
		else if (offset == 3)
		{
			*NVIC_ISER3 |= (1 << pos);
		}
	}
	else
	{
		uint8_t pos = IRQNumber % 32;
		uint8_t offset = IRQNumber / 32;
		if (offset == 0)
		{
			*NVIC_ISER0 &= ~(1 << pos);
		}
		else if (offset == 1)
		{
			*NVIC_ISER1 &= ~(1 << pos);
		}
		else if (offset == 2)
		{
			*NVIC_ISER2 &= ~(1 << pos);
		}
		else if (offset == 3)
		{
			*NVIC_ISER3 &= ~(1 << pos);
		}

	}
}
void USART_EV_IRQHandling(USART_Handle_t* pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}

}
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint16_t FlagName)
{
	if (pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void USART_ClearFlag(USART_RegDef_t* pUSARTx, uint16_t FlagName)
{
	pUSARTx->SR &= ~FlagName;
}
void USART_ManageAcking(USART_RegDef_t* pUSARTx, uint8_t EnorDi);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t* pHandle, uint8_t AppEv);







