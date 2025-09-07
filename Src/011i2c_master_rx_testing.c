/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: Sep 6, 2025
 *      Author: gphi1
 */

#include "stm32f407xx_i2c_driver.h"
#include "string.h"

#define CMD_READ_LEN_DATA 		0x51
#define CMD_READ_CMPLT_DATA 	0x52

void GPIO_Init()
{

}


int main()
{
	// GPIO Init for SCL, SDA line
	// SCL->PB6
	// SDA->PB7
	GPIO_Init();


	// Enable I2C Peripheral
	I2C_PeriClockControl(I2C1, ENABLE);

	// Master send command code 0x51 to read length data from Slave
	I2C_MasterSendData(pHandle, pTxBuffer, Len, SlaveAddr);

	// Close Send Data
	I2C_CloseSendData(pI2CHandle);

	// Master receive length data from slave
	I2C_MasterReceiveData(pHandle, pRxBuffer, Len, SlaveAddr);

	// Master read complete data from slave
	I2C_MasterReceiveData(pHandle, pRxBuffer, Len, SlaveAddr);

	// Close Receive Data
	I2C_CloseReceiveData(pI2CHandle);

}


