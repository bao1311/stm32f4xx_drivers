/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Aug 28, 2025
 *      Author: gphi1
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_
#include "stm32f407xx.h"
/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACK;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct {
	I2C_RegDef_t* pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t* TxBuffer;		   /* |< To store the app. Tx buffer address > */
	uint8_t* RxBuffer;		   /* |< To store the app. Rx buffer address > */
	uint32_t RxLen;			   /* |< To store the app. Rx len > */
	uint32_t TxLen;				/* |< To store the app. Tx len > */
	uint8_t TxRxState;			/* |< To store the app. TxRx Ready/Busy State > */
	uint8_t DevAddr;			/* |< To store the Device Address > */
	uint32_t RxSize;			/* |< To store the RxSize > */
	uint8_t Sr;					/* |< To store the Repeated Start signal > */
} I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM				100000
#define I2C_SCL_SPEED_FM_2k				200000
#define I2C_SCL_SPEED_FM_4k				400000

/*
 * @I2C_ACK
 */
#define I2C_ACK_DISABLE					0
#define I2C_ACK_ENABLE					1

/*
 * @I2C_FLAG
 */
#define I2C_SB_FLAG						(1 << 0)
#define I2C_ADDR_FLAG					(1 << 1)
#define I2C_BTF_FLAG					(1 << 2)
#define I2C_ADD10_FLAG					(1 << 3)
#define I2C_STOPF_FLAG					(1 << 4)
#define I2C_RXNE_FLAG					(1 << 6)
#define I2C_TXE_FLAG					(1 << 7)
#define I2C_BERR_FLAG					(1 << 8)
#define I2C_ARLO_FLAG					(1 << 9)
#define I2C_AF_FLAG						(1 << 10)
#define I2C_OVR_FLAG					(1 << 11)
#define I2C_PECERR_FLAG					(1 << 12)
#define I2C_TIMEOUT_FLAG				(1 << 14)
#define I2C_SMBALERT_FLAG				(1 << 15)
/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2					0
#define I2C_FM_DUTY_16_9				1

/*
 * @I2C_ACK_FLAG
 */
#define I2C_ACK_EN						1
#define I2C_ACK_DI						0


/*
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * This is the I2C API that this driver supports
 * Please look at the implementation for more details
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 * **********************************************************
 */

/*
 * API for I2C Init and Del Init
 */

/******************************************
 * @fn			- I2C_Init
 *
 * @brief		- This function initialise the I2C
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2Cx); 		// Cando
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

void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);


/*
 * Data send and receive (Blocking version)
 */
void I2C_MasterSendData(I2C_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t* pI2Cx, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr);


/*
 * Data send and receive (Interrupt version)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t Len);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t Len);
/*                     /
 * I2C Application/ States Macros
 */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2


/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority); // Cando
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); //Cando


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t* pHandle, uint8_t AppEv);




#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
