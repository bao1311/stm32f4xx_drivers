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


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
