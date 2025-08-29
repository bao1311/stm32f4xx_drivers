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
