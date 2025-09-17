/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: Sep 6, 2025
 *      Author: gphi1
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx.h"
#include "string.h"
#include "stm32f407xx_gpio_driver.h"

#define CMD_READ_LEN_DATA 		0x51
#define CMD_READ_CMPLT_DATA 	0x52

#define SLAVE_ADDRESS			0x68
#define MY_ADDRESS 				0x61

I2C_Handle_t I2CHandle;
GPIO_Handle_t GPIOHandle;
extern void initialise_monitor_handles();

void I2C1_GPIOInit()
{
	GPIOHandle.pGPIOx = GPIOB;
	// SCL -> PB6
	// SDA -> PB7
	GPIOHandle.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	GPIOHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// Configure pin for SCL => PB6
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&GPIOHandle);

	// Configure pin for SDA => PB7
	GPIOHandle.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&GPIOHandle);

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

void I2C1_Inits()
{
	I2CHandle.pI2Cx = I2C1;
	I2CHandle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2CHandle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
	I2CHandle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2CHandle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

//	I2C_PeripheralControl(I2CHandle.pI2Cx, ENABLE);
	I2C_Init(&I2CHandle);
}
void delay()
{
	for (int i = 0; i < 100000; ++i)
	{
		;
	}
}

int main()
{
	// GPIO Init for SCL, SDA line
	// SCL->PB6
	// SDA->PB7
	// I2C1 Pin init
	I2C1_GPIOInit();
	// GPIO Button pin init
	GPIOBtn_Init();
	// I2C1 Init code
	initialise_monitor_handles();

	// Enable PE right here
	// Only after we enable that, we are allowed to modify the peripheral register
	I2C1_Inits();
	// Enable I2C Peripheral
//	I2C_PeriClockControl(I2C1, ENABLE);


	while (1)
	{
		while (!GPIO_ReadFromInputPin(GPIOA, 0));
		// Handle debouncing
		delay();
		// Master send command code 0x51 to read length data from Slave
		uint8_t commandCode = 0x51;
		I2C_MasterSendData(&I2CHandle, &commandCode, 1, SLAVE_ADDRESS);

		// Close Send Data
//		I2C_CloseSendData(&I2CHandle);
		uint8_t Len = 0;
		// Master receive length data from slave
		I2C_MasterReceiveData(&I2CHandle, &Len, 1, SLAVE_ADDRESS);

		// Master send 0x52 to receive the data from slave
		commandCode = 0x52;
		I2C_MasterSendData(&I2CHandle, &commandCode, 1, SLAVE_ADDRESS);

		uint8_t rcv_buf[100];
		// Master read complete data from slave
		I2C_MasterReceiveData(&I2CHandle, rcv_buf, Len, SLAVE_ADDRESS);
		rcv_buf[Len+1] = '\0';
		printf("Data is: %s\n", rcv_buf);
		// Close Receive Data
//		I2C_CloseReceiveData(&I2CHandle);
	}

}


