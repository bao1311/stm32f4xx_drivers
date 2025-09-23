/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Sep 16, 2025
 *      Author: gphi1
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"
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
 * @USART_HWFlowControl
 * Possible options of USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * @USART_ParityControl
 * Possible options of USART_ParityControl
 */
#define USART_PARITY_DI			0
#define USART_PARITY_EN_EVEN	1
#define USART_PARITY_EN_ODD		2
/*
 * @USART_WordLength
 * Possible options of USART_WordLength
 *
 */
#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1
/*
 * @USART_NoOfStopBits
 * Possible options of USART_NoOfStopBits
 */
#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5			3

/*
 * @USART_Baud
 * Possible options of USART_Baud
 */
#define USART_STD_BAUD_1200			1200
#define USART_STD_BAUD_2400			2400
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200		19200
#define USART_STD_BAUD_38400		38400
#define USART_STD_BAUD_57600		57600
#define USART_STD_BAUD_115200		115200
#define USART_STD_BAUD_230400		230400
#define USART_STD_BAUD_460800		460800
#define USART_STD_BAUD_921600		921600
#define USART_STD_BAUD_2000000		2000000
#define USART_STD_BAUD_3000000		3000000

/*
 * @USART_Mode
 * Possible options of USART_Mode
 */
#define USART_MODE_TX		0
#define USART_MODE_RX		1
#define USART_MODE_TXRX		2

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
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint16_t FlagName);
void USART_ManageAcking(USART_RegDef_t* pUSARTx, uint8_t EnorDi);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t* pHandle, uint8_t AppEv);

/*
 * USART Flag
 */
#define USART_TXE_FLAG 				(1 << 7)
#define USART_RXNE_FLAG				(1 << 5)




#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
