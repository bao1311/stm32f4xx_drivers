/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Aug 31, 2025
 *      Author: gphi1
 */

// PB6 -> SCL
// PB9 -> SDA
#include "stm32f407xx_i2c_driver.h"
void I2C1_GPIOInits()
{
	GPIO_Handle_t I2C1Pins;
	// 1. Enable GPIO peripheral clock
	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	// 2. Configure SCL line of STM32 (PB6)
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2C1Pins);
	// 3. Configure SDA line of STM32 (PB9)
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2C1Pins);

}

void I2C1_Inits()
{
	I2C_Handle_t I2C1Handle;
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACK = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
//	I2C1Handle.I2C_Config.I2C_DeviceAddress =
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Init(&I2C1Handle);

}

int main(void)
{
	char data[] = "Hello, world!\n";
	I2C1_GPIOInits();

	I2C1_Inits();

}

