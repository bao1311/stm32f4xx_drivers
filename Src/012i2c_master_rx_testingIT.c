/*
 * 012i2c_master_rx_testingIT.c
 *
 *  Created on: Sep 6, 2025
 *      Author: gphi1
 */


/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Aug 31, 2025
 *      Author: gphi1
 */

// PB6 -> SCL
// PB9 -> SDA
#include "stm32f407xx_i2c_driver.h"
#include "string.h"
#define MY_ADDRESS 0x61
#define SLAVE_ADDRESS 0x68
I2C_Handle_t I2C1Handle;
void I2C1_GPIOInits(void)
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

void GPIOBtn_Init(void)
{
	GPIO_Handle_t gpioBtn;
	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = 0;
	gpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&gpioBtn);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Init(&I2C1Handle);

}

void delay()
{
	for (uint32_t i = 0; i < 100000; i += 1)
	{

	}
}

int main(void)
{
	uint8_t data[] = "Hello, world!\n";
	I2C1_GPIOInits();

	I2C1_Inits();
	GPIOBtn_Init();

	I2C_PeripheralControl(I2C1, ENABLE);

	while (1)
	{
		while (!GPIO_ReadFromInputPin(GPIOA, 0));
		delay();
		I2C_MasterSendData(&I2C1Handle, data, strlen((char*)data), SLAVE_ADDRESS);
	}

}




