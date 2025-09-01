/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Aug 31, 2025
 *      Author: gphi1
 */

// PB6 -> SCL
// PB9 -> SDA
#include "stm32f407xx_i2c_driver.h"
void I2C_GPIOInits()
{
	GPIO_Handle_t I2CPins;
	// 1. Enable GPIO peripheral clock
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	// 2. Configure SCL line of STM32 (PB6)
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);
	// 3. Configure SDA line of STM32 (PB9)
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

int main(void)
{
	char data[] = "Hello, world!\n";
	I2C_GPIOInits();

}

