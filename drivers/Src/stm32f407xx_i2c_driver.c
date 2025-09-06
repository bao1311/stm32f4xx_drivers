/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Aug 29, 2025
 *      Author: gphi1
 */

#include "stm32f407xx_i2c_driver.h"


void i2c_sb_handle(I2C_Handle_t* pI2CHandle);

static void I2C_GenerateStopSignal(I2C_RegDef_t* pI2Cx);

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

uint32_t RCC_GetPLLOutputClk()
{
	// Not mentioned inside the lecture
	return 0;
}
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint32_t RCC_GetPCLK1Value()
{
	uint32_t SystemClk;
	uint8_t ClkSource;
	ClkSource = (RCC->CFGR >> 2) & (0x3);
	if (ClkSource == 0)
	{
		// HSI oscillator as system clock
		SystemClk = 16000000U;
	}
	else if (ClkSource == 1)
	{
		// HSE oscillator as system clock
		SystemClk = 8000000U;
	}
	else if (ClkSource == 2)
	{
		// PLL selected as system clock
		SystemClk = RCC_GetPLLOutputClk();
	}
	uint8_t temp = (RCC->CFGR >> 4) & (0xF);
	uint16_t ahbp = 0;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}
	temp = (RCC->CFGR >> 10) & (0x7);
	uint8_t apb1 = 0;
	if (temp < 4)
	{
		apb1 = 1;
	}
	else
	{
		apb1 = APB1_PreScaler[temp - 4];
	}
	uint32_t pclk1 = (SystemClk/ahbp) / apb1;
	return pclk1;
}

void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	uint32_t tempreg = 0;
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	// Configure CR1 register of I2C
	// ack control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;
	// Configure CR2 register of I2C
	// configurethe freq field of CR2
	tempreg = 0;
	tempreg |= ((RCC_GetPCLK1Value()/1000000U) << I2C_CR2_FREQ);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);
	// Configure OAR1 (Own address register 1) of I2C
	tempreg = 0;
	tempreg |= (1 << 14);
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	pI2CHandle->pI2Cx->OAR1 = tempreg;
	// Configure the CCR register of I2C
	tempreg = 0;
	uint16_t ccr = 0;
	// CCR calculation
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// I2C SCL standard mode
		// Note: In standard mode, Thigh = Tlow
		ccr = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr & 0xFFF);
	}
	else
	{
		// I2C SCL fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			// DUTY_2 mode
			// Note: In this mode, Tlow = 2 * Thigh
			ccr =  (RCC_GetPCLK1Value())/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			// DUTY_16_9 mode
			ccr =  (RCC_GetPCLK1Value())/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		tempreg |= (ccr & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	uint8_t trise = 0;
	tempreg = 0;
	// Trise Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode trise
		// Note: Max of trise in sm = 1000ns
		trise = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// Fast mode trise
		trise = ((RCC_GetPCLK1Value() * 3) / 10000000U) + 1;
	}
	tempreg |= (trise & 0x3F);
	pI2CHandle->pI2Cx->TRISE = tempreg;
}


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


static void I2C_GenerateStartSignal(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/*
 * *******************************
 * @fn				- I2C_ClearSB
 * @param[in]		- I2C_RegDef_t*
 * @return: void
 * @Note: This function is not needed.
 * Why? The SB bit is cleared by reading SR1 followed by
 * write into DR register. We are doing that by checking if the
 * SB bit is set, then we write into the DR register with the address
 * of the slave
 */
/*
static void I2C_ClearSB(I2C_RegDef_t* pI2Cx)
{
	uint32_t temp;
	temp = pI2Cx->SR1;
	temp = pI2Cx->DR;
	(void)temp;

}
*/

/*
 * *************************************
 * @fn						- I2C_ExecuteAddressPhaseWrite
 * @brief					- This function send the address of slave
 *							and the read/write bit to the DR register
 * @param[in]				- I2C_RegDef_t*
 * @param[in]				- SlaveAddress
 * @return: void
 * @note:
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddress)
{
	uint8_t address = 0;
	address |= (SlaveAddress << 1);
	// read/write bit == 0 => Transmitter
	address &= ~(1);
	pI2Cx->DR |= address;
}

/*
 * *************************************
 * @fn						- I2C_ExecuteAddressPhaseRead
 * @brief					- This function send the address of slave
 *							and the read/write bit to the DR register
 * @param[in]				- I2C_RegDef_t*
 * @param[in]				- SlaveAddress
 * @return: void
 * @note:
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddress)
{
	uint8_t address = 0;
	address |= (SlaveAddress << 1);
	// read/write bit == 1 => Receiver
	address |= 1;
	pI2Cx->DR |= address;
}
//void I2C_Ack(I2C_Handle_t* pHandle)
//{
////	pHandle->pI2Cx->
//}

static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle)
{
	/*
	 * Two cases for pI2CHandle.
	 * 1. RxLen == 1
	 * 2. RxLen > 1
	 */
	uint8_t dummyRead;
	if (pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR2_MSL))
	{
		// Master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				// Send ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
				// Clear ADDR
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else
		{
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}
	else
	{
		// Slave mode
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_EN)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


void I2C_MasterReceiveData(I2C_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate start signal (SB)
	I2C_GenerateStartSignal(pHandle->pI2Cx);
	// 2. Ensure the SB Flag is set
	while (!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_SB_FLAG));
	// 3. Execute Address Phase for Receiver
	I2C_ExecuteAddressPhaseRead(pHandle->pI2Cx, SlaveAddr);
	// 4. Confirm the ADDR bit is set by the slave
	while (!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_ADDR_FLAG));

	if (Len == 1)
	{
		// 5. Manage ACK, turn ACK to 0
		I2C_ManageAcking(pHandle->pI2Cx, I2C_ACK_DI);
		// 6. Turn STOP -> 1
		I2C_GenerateStopSignal(pHandle->pI2Cx);
		// Clear ADDR Flag
		I2C_ClearADDRFlag(pHandle);
		// Wait until RXNE turn 1
		while (!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_RXNE_FLAG));
		// 7. Read DR
		*pRxBuffer = pHandle->pI2Cx->DR;
	}
	else if (Len > 1)
	{
		// Clear ADDR Flag
		I2C_ClearADDRFlag(pHandle);
		I2C_ManageAcking(pHandle->pI2Cx, I2C_ACK_EN);
		while (Len)
		{
			// When Len == 2, a special event occurs
			/*
			 * Read Data, then do the same as the first case
			 * ACK->0
			 * STOP->1
			 * read DR
			 */
			if (Len == 2)
			{
				while (!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_RXNE_FLAG));
				// Read DR
				*pRxBuffer = pHandle->pI2Cx->DR;
				Len--;
				pRxBuffer++;
				I2C_ManageAcking(pHandle->pI2Cx, I2C_ACK_DI);
				I2C_GenerateStopSignal(pHandle->pI2Cx);
				// read DR
				*pRxBuffer = pHandle->pI2Cx->DR;
				Len--;
				pRxBuffer++;

			}
			else
			{
				// 6. Wait until RXNE bit
				while (!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_RXNE_FLAG));
				// 7. Read DR
				*pRxBuffer = pHandle->pI2Cx->DR;
				Len--;
				pRxBuffer++;
				// 5. ACK -> 1
				I2C_ManageAcking(pHandle->pI2Cx, I2C_ACK_EN);
			}
		}

	}
	// 4. Clear Address bit
	I2C_ClearADDRFlag(pHandle);
	// 5. Receive data
	while (Len)
	{
		while (I2C_GetFlagStatus(pHandle->pI2Cx, I2C_RXNE_FLAG));
		pHandle->pI2Cx->DR = *pRxBuffer;
		pRxBuffer++;
		Len--;
	}

	// Reenable the acking
	if (pHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pHandle->pI2Cx, I2C_ACK_EN);
	}


}



void I2C_MasterSendData(I2C_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate start signal
	I2C_GenerateStartSignal(pHandle->pI2Cx);
	// 2. Ensure that SB FLAG is set
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_SB_FLAG));
	// 2. EV5: SB = 1. We will clear it by reading SR1 register
	// followed by reading SR2 register (NOT NEEDED)
//	I2C_ClearSB(pHandle->pI2Cx);
	// 3. Send Address
	I2C_ExecuteAddressPhaseWrite(pHandle->pI2Cx, pHandle->I2C_Config.I2C_DeviceAddress);
	// 4. Clear SR1 ADDR bit
	I2C_ClearADDRFlag(pHandle);
	// 4. Acknowledge will be done by the receiver (ACK/NACK)
	// 5. Send data while Len > 0
	while (Len > 0)
	{
		while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_TXE_FLAG));

		*pTxBuffer = pHandle->pI2Cx->DR;
		pTxBuffer++;
		Len--;
	}
	// 6. Wait until TXE (DR empty for transmitters) and BTF (Byte transfer finished) flag is set to 1 (Hardware will set that)
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_TXE_FLAG));
	while (! I2C_GetFlagStatus(pHandle->pI2Cx, I2C_BTF_FLAG));
	I2C_GenerateStopSignal(pHandle->pI2Cx);
}
static void I2C_GenerateStopSignal(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

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
/*
 * @fn 			- I2C_IRQInterruptConfig
 *
 * @brief		- This function enable the interrupt based on the IRQ number
 * @param[in]	- IRQNumber
 * @param[in]	- EnorDi
 *
 * @return
 * @Note: Use ISER (Set-enable register). STM32F407 only has 82 maskable
 * interrupt channels
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber < 32)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else
		{
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}

	}
	else
	{

		if (IRQNumber < 32)
		{
			*NVIC_ISER0 &= ~(1 << IRQNumber);
		}
		else if (IRQNumber < 64)
		{
			*NVIC_ISER1 &= ~(1 << (IRQNumber%32));
		}
		else
		{
			*NVIC_ISER2 &= ~(1 << (IRQNumber%64));
		}
	}

}


/*
 * Data send and receive (Interrupt version)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pHandle->TxRxState;
	// Enable I2C_BUSY_IN_TX state
	if (busystate != I2C_BUSY_IN_TX && busystate != I2C_BUSY_IN_RX)
	{
		pHandle->TxRxState = I2C_BUSY_IN_TX;
		pHandle->TxBuffer = pTxBuffer;
		pHandle->TxLen = Len;
		pHandle->DevAddr = SlaveAddr;
		pHandle->Sr = Sr;


		// Implement code to Generate Start condition
		I2C_GenerateStartSignal(pHandle->pI2Cx);

		// Enable ITEVTEN and ITBUFEN and ITERREN bit
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return pHandle->TxRxState;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pHandle->TxRxState;
	// Enable I2C_BUSY_IN_RX state
	if (busystate != I2C_BUSY_IN_RX && busystate != I2C_BUSY_IN_TX)
	{
		pHandle->TxRxState = I2C_BUSY_IN_RX;
		pHandle->RxBuffer = pRxBuffer;
		pHandle->RxLen = Len;
		pHandle->DevAddr = SlaveAddr;
		pHandle->Sr = Sr;

		// Implement code to Generate Start condition
		I2C_GenerateStartSignal(pHandle->pI2Cx);

		// Enable ITEVTEN and ITBUFEN and ITERREN bit
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;

}

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority); // Cando
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); //Cando


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
		return FLAG_SET;
	return FLAG_RESET;

}


void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle)
{
	// TxE is set
	if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	{
		if (pI2CHandle->TxLen > 0)
		{
			*(pI2CHandle->TxBuffer) = pI2CHandle->pI2Cx->DR;
			pI2CHandle->TxBuffer++;
			pI2CHandle->TxLen--;
		}
	}

}

void I2C_CloseSendData(I2C_Handle_t* pI2CHandle)
{
	/*
	 * Disable the interrupt first
	 */
	// For TXE, we have to disable ITEVTEN and ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->RxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->TxRxState = I2C_READY;
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}
void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle)
{
	/*
	 * Disable the interrupt first
	 */
	// For RXNE, we have to disable ITEVTEN and ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->TxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;

}
void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle)
{
	// RxNE is set
	if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	{

		if (pI2CHandle->RxLen == 1)
		{
			/*
			 * In case TxLen is equal to 1, we would want to disable
			 * ACK and turn STOP bit to 1 so Slave won't send
			 * any extra data
			 */
			*(pI2CHandle->RxBuffer) = pI2CHandle->pI2Cx->DR;
			pI2CHandle->RxLen--;
		}
		else if (pI2CHandle->RxLen > 1)
		{
			if (pI2CHandle->RxLen == 2)
			{
				/*
				 * When Len is equal to 2
				 * Read DR,
				 * ACK -> 0
				 * STOP -> 1
				 * Read DR
				 */
				// Send NACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DI);

			}

			*(pI2CHandle->RxBuffer) = pI2CHandle->pI2Cx->DR;
			pI2CHandle->RxBuffer++;
			pI2CHandle->RxLen--;
		}
		if (pI2CHandle->RxLen == 0)
		{
			if (pI2CHandle->Sr == I2C_DISABLE_SR)
			{
				// Stop signal generated
				I2C_GenerateStopSignal(pI2CHandle->pI2Cx);
			}
			// Close the I2C communication
			I2C_CloseReceiveData(pI2CHandle);
			// Event call back
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
		}

	}

}

/*
 * *****************************************
 * @fn					- I2C_EV_IRQHandling
 *
 * @brief				-
 *
 * @param[in]
 * @param[in]
 *
 *
 * @return
 * @Note				- Interrupt handling for different I2C Event
 * *****************************************
 */
void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device
	uint8_t temp1, temp2, temp3;

	// temp1 -> ITVEFEN
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	// temp2 -> ITBUFEN
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	// 1. Handle for Interrupt generated by SB event
	// SB flag can only be generated in Master mode
	// temp3 -> SB
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if (temp1 & temp3)
	{
		// SB flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}
	// 2. Handle interrupt generated by ADDR event
	/*
	 * 2 cases:
	 * - For master: Check if end of address transmission
	 * - For slave: Check if addr == slave address
	 */
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if (temp1 & temp3)
	{
		// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}
	// 3. Handle interrupt generated by BTF(Byte transfer finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if (temp1 & temp3)
	{
		// BTF flag is set
		/*
		 * Root cause: BTF flag is set because data is written to DR
		 * already but we have not taken any action to read/write the
		 * DR
		 */
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if (pI2CHandle->TxLen == 0)
			{
				if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE))
				{
					// Generate STOP condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopSignal(pI2CHandle->pI2Cx);
					}

					I2C_CloseSendData(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Nothing to do here
			;
		}
	}
	// 4. Handle interrupt generated by STOPF(Stop Detection)
	// This interrupt is handled by Slave only. For master, this flag will
	// never be set
	// The below code block will not be executed by master since STOPF
	// will not be set in the Master Mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if (temp1 & temp3)
	{
		// STOPF is set
		// We have to clear the STOPF flag
		pI2CHandle->pI2Cx->CR1 |= 0;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	// 5. Handle interrupt generated by TXE(Data register empty(transmitters))
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	if (temp1 & temp2 & temp3)
	{
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}

	}
	// 6. Handle interrupt generated by RXNE(Data register not empty(receiver))
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	if (temp1 & temp2 & temp3)
	{
		if (pI2CHandle->pI2Cx->SR2 & (I2C_SR2_MSL))
		{

			I2C_MasterHandleRXNEInterrupt(pI2CHandle);
		}
	}

}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= (1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}



/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t* pHandle, uint8_t AppEv);


