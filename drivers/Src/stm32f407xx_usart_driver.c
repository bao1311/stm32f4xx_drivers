/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Sep 16, 2025
 *      Author: gphi1
 */
#include "stm32f407xx_usart_driver.h"
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

	// Set up USART_Baud

	// Set up USART_NoOfStopBits
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOP_BITS_1)
	{
		tempreg |= (0 << USART_CR2_STOP);
	}
	else if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOP_BITS_0.5)
	{
		tempreg |= (1 << USART_CR2_STOP);
	}
	else if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOP_BITS_2)
	{
		tempreg |= (2 << USART_CR2_STOP);
	}
	else if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOP_BITS_1.5)
	{
		tempreg |= (3 << USART_CR2_STOP);
	}
	pUSARTHandle->pUSARTx->CR2 |= tempreg;


	// USART_WordLength
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9)
	{
		tempreg |= (1 << USART_CR1_M);
	}
	pUSARTHandle->pUSARTx->CR1 |= tempreg;
	// USART_ParityControl
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN)
	{
		tempreg |= (1 << USART_CR1_PCE);
	}
	pUSARTHandle->pUSARTx->CR1 |= tempreg;
	// USART_HWFlowControl
	tempreg = 0;
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_CTRL_EN)
	{
		temreg |= (1 << USART_CR3_RTSE);
		temreg |= (1 << USART_CR3_CTSE);
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
	if (EnorDi == USART_CLK_EN)
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
void USART_MasterSendData(USART_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority); // Cando
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); //Cando
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
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint8_t FlagName);
void USART_ManageAcking(USART_RegDef_t* pUSARTx, uint8_t EnorDi);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t* pHandle, uint8_t AppEv);







