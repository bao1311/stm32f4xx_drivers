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
} I2C_Handle_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct {
	I2C_RegDef_t* pI2Cx;
	I2C_Handle_t I2C_Config;
} I2C_Config_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM				100000
#define I2C_SCL_SPEED_FM_2k				200000
#define I2C_SCL_SPEED_FM				400000

/*
 * @I2C_ACK
 */
#define I2C_ACK_DISABLE					0
#define I2C_ACK_ENABLE					1

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2					0
#define I2C_FM_DUTY_16_9				1


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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority); // Cando
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




#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
