/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Jul 27, 2025
 *      Author: gphi1
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * GPIO Convert To Binary function
 */
#define GPIO_BASEADDR_TO_CODE(x)	(x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :0



/*
 * @GPIO_PIN_MODES
 * Modes of GPIO pin input type
 *
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_OP_TYPE
 * Mode of output type of GPIO port
 *
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 *
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD
 * GPIO pin pull up and pull down configuration macros
 *
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * This is configuration structure for a GPIO Pin
 */
typedef struct
{
	// Configurable Pin configuration for user application
	uint8_t GPIO_PinNumber;				/* Possible values in @GPIO_PIN_NUMBER	*/
	uint8_t GPIO_PinMode; 				/* Possible values in @GPIO_PIN_MODES	*/
	uint8_t GPIO_PinSpeed;				/* Possible values in @GPIO_PIN_SPEED	*/
	uint8_t GPIO_PinOPType;				/* Possible values in @GPIO_OP_TYPE		*/
	uint8_t GPIO_PinPuPdControl;		/* Possible values in @gPIO_PUPD		*/
	uint8_t GPIO_PinAltFunMode;




}GPIO_PinConfig_t;

/*
 * This is Handle Structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t* pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * This is the GPIO that this driver supports
 * Please look at the implementation for more details
 */

/*
 * API for GPIO Init and Del Init
 */

/******************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function initialise the
 * @param[in]	- Base address of the gpio peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);
/*
 * API for GPIO Peripheral Clock Setup
 */

/******************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enable or disable for the given GPIO port
 * @param[in]	- Base address of the gpio peripheral
 * @param[in]	- Enable or Disable macros
 *
 * @return
 * @Note:
 */

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

/*
 * API for read and write data to input pin and port
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint8_t Value);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
