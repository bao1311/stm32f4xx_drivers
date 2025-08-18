/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 27, 2025
 *      Author: gphi1
 */
#include "stm32f407xx_gpio_driver.h"

/*
 * API for GPIO Init and Del Init
 */

/******************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function initialise the pin number in the given gpio port
 * @param[in]	- GPIO port and pin configuration
 * @param[in]
 *
 * @return		- None
 * @Note:
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;
	// 1. Configure the mode of gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non-interrupt mode configuration
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else
	{
		// Interrupt mode configuration
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure the FTSR( Falling trigger selection register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the RTSR (Rising trigger selection register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint32_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint32_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portCode << (4 * temp2);

		// 3. Enable EXTI interrupt using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	// 2. Configure the speed
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;



	// 3. Configure pupd settings
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. Configure the output type
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	// 5. Configure the alt function if required
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}

}

/******************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function resets the GPIOx register
 * @param[in]	- GPIO_x port register
 * @param[in]
 *
 * @return		- None
 * @Note:
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{

	if (pGPIOx == GPIOA)
		GPIOA_PCLK_RESET();
	else if (pGPIOx == GPIOB) {
		GPIOB_PCLK_RESET();
	}
	else if (pGPIOx == GPIOC) {
		GPIOC_PCLK_RESET();
	}
	else if (pGPIOx == GPIOD) {
		GPIOD_PCLK_RESET();
	}
	else if (pGPIOx == GPIOE) {
		GPIOE_PCLK_RESET();
	}
	else if (pGPIOx == GPIOF) {
		GPIOF_PCLK_RESET();
	}
	else if (pGPIOx == GPIOG) {
		GPIOG_PCLK_RESET();
	}
	else if (pGPIOx == GPIOH) {
		GPIOH_PCLK_RESET();
	}
	else if (pGPIOx == GPIOI) {
		GPIOI_PCLK_RESET();
	}
}

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
 * @return		- None
 * @Note:
 */

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG) {
			GPIOJ_PCLK_EN();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}


	}
	else
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}

	}

}

/*
 * API for read and write data to input pin and port
 */

/******************************************
 * @fn 			- GPIO_WriteToOutputPin
 *
 * @brief		- This function write value to the configured pin of the GPIO port
 * @param[in] 	- GPIOx
 * @param[in]	- Pin Number
 * @param[in]	- Value to write into the output pin
 *
 * @return		- None
 * @Note:
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		// Write 1 to the GPIO pin
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// Write 0 to the GPIO pin
		pGPIOx->ODR &= ~(1 << PinNumber);

	}

}

/******************************************
 * @fn 			- GPIO_WriteToOutputPort
 *
 * @brief		- This function write value to the port of the GPIO
 * @param[in] 	- GPIOx
 * @param[in]	- Value to write into the output port
 *
 * @return		- None
 * @Note:
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint8_t Value)
{
	pGPIOx->ODR |= Value;
}

/******************************************
 * @fn 			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read value from the configured input pin of GPIOx
 * @param[in] 	- GPIOx
 * @param[in]	- Pin Number
 *
 * @return		- 0 or 1
 * @Note:
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t res;
	res = (uint8_t)(pGPIOx->IDR >> PinNumber) & (0x1);
	return res;
}

/******************************************
 * @fn 			- GPIO_ReadFromInputPort
 *
 * @brief		- This function read value from the 16 pins of GPIOx port
 * @param[in] 	- GPIOx
 *
 * @return		- uint16_t
 * @Note:		- A port consists of 16 pins
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t res;
	res = (uint16_t)pGPIOx->IDR;
	return res;
}

/******************************************
 * @fn 			- GPIO_ToggleOutputPin
 *
 * @brief		- This function toggles the output configured pin
 * @param[in] 	- GPIOx
 * @param[in]	- Pin Number
 *
 * @return		- None
 * @Note:		- Toggle from 0 to 1 and vice versa
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}

/*
 * IRQ Configuration and ISR handling
 */

/******************************************
 * @fn 			- GPIO_IRQInterruptConfig
 *
 * @brief		- This function configured the interrupt configuration
 * @param[in] 	- IRQNumber
 * @param[in]	- IRQPriority
 * @param[in]	- EnorDi
 *
 * @return		- None
 * @Note:		- ???
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		// Enable the IRQ based on the NVIC EXTI register
		if (IRQNumber >= 0 && IRQNumber < 32)
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 64));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 96));
		}
	}
	else
	{
		// Disable the IRQ based on the NVIC EXTI register
		if (IRQNumber >= 0 && IRQNumber < 32)
		{
			// Program ICER0 register
			*NVIC_ICER0 &= ~(1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			// Program ICER1 register
			*NVIC_ICER1 &= ~(1 << (IRQNumber % 64));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ICER2 register
			*NVIC_ICER2 &= ~(1 << (IRQNumber % 96));
		}

	}
}

void GPIO_IRQPriorityHandling(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shiftAmt = 8 * (iprx_section) + 8 - NO_PR_BITS_IMPLEMENTED;
	*(NVIC_PR_BASEADDR + iprx) |= IRQPriority << shiftAmt;

}

/******************************************
 * @fn 			- GPIO_IRQHandling
 *
 * @brief		- This function handles the iterrupt
 * @param[in]	- Pin Number
 *
 * @return		- None
 * @Note:		- ???
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		// Clear
		EXTI->PR |= ( 1 << PinNumber);

	}
}




