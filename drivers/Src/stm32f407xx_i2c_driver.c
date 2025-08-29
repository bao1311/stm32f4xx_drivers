/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Aug 29, 2025
 *      Author: gphi1
 */

#include "stm32f407xx_i2c_driver.h"



void I2C_DeInit(I2C_RegDef_t* pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_PCLK_DI();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_PCLK_DI();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_PCLK_DI();
	}
}


void I2C_Init(I2C_Handle_t* pI2CHandle);
/*
 * API for I2C Peripheral Clock Setup
 */

/******************************************
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- This function enable or disable for the given I2C
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */

void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if (EnorDi == DISABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*
 * Data send and receive (Blocking version)
 */
void I2C_SendData(I2C_RegDef_t* pI2Cx, uint8_t* pTxBuffer, uint32_t Len);
void I2C_ReceiveData(I2C_RegDef_t* pI2Cx, uint8_t* pRxBuffer, uint32_t Len);


/*
 * Data send and receive (Interrupt version)
 */
uint8_t I2C_SendDataIT(I2C_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len);
uint8_t I2C_ReceiveDataIT(I2C_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t Len);
/*
 * I2C Application States Macros
 */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2


/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t irpx, irpx_section;
	irpx = IRQNumber / 4;
	irpx_section = IRQNumber % 4;
	// STM32F407 only have 16 priority level
	uint8_t shiftAmt = irpx_section * 8 + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + irpx) |= (IRQPriority << shiftAmt);
}
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); //Cando


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t FlagName);
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t* pHandle, uint8_t AppEv);


