/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Sep 16, 2025
 *      Author: gphi1
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"
/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t* pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;

/*
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * This is the USART API that this driver supports
 * Please look at the implementation for more details
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 */

/*
 * API for USART Init and Del Init
 */

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
void USART_Init(USART_Handle_t* pUSARTHandle);
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

void USART_PeriClockControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi);


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
void USART_PeripheralControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint8_t FlagName);
void USART_ManageAcking(USART_RegDef_t* pUSARTx, uint8_t EnorDi);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t* pHandle, uint8_t AppEv);




#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
