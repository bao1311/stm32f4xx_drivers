/*
 * stm32f407xx_gpio_spi_driver.h
 *
 *  Created on: Aug 16, 2025
 *      Author: gphi1
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include "stm32f407xx.h"
/*
 * Configuration structure for SPIx peripheral
 */
typedef struct {
	uint8_t SPI_DeviceMode;			// Slave or Master
	uint8_t SPI_BusConfig;			// Full duplex, half duplex, simplex
	uint8_t SPI_SclkSpeed;			// set up prescaler mode like /2,/4,/8
	uint8_t SPI_DFF;				// data in format 8 bits or 16 bits
	uint8_t SPI_CPOL;				// Clock polarity high or low
	uint8_t SPI_CPHA;				// Clock phase
	uint8_t SPI_SSM;				// Software slave management
} SPI_Config_t;

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI 			0
#define SPI_SSM_EN 			1
/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH			1
#define SPI_CPHA_LOW			0

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * SPI related status flags definitions related to SPI_SR register
 */
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 				(1 << SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG				(1 << SPI_SR_CHSIDE)


/*
 * Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t* pSPIx;		/*|< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_Config_t SPIConfig;

} SPI_Handle_t;

/*
 * =====================================================
 * =====================================================
 */

/*
 * This is the SPI API that this driver supports
 * Please look at the implementation for more details
 */

/*
 * API for SPI Init and Del Init
 */

/******************************************
 * @fn			- SPI_Init
 *
 * @brief		- This function initialise the SPI
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);
/*
 * API for SPI Peripheral Clock Setup
 */

/******************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enable or disable for the given SPI
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t Len);

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQPriorityHandling(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t* pHandle);


/*
 * Other Peripheral Control APIs
 */

#endif /*  INC_STM32F407XX_SPI_DRIVER_H_ */
