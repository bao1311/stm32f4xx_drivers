/*
 * 005button_interrupt.c
 *
 *  Created on: Jul 30, 2025
 *      Author: gphi1
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>


#define HIGH 		1
#define LOW			0
#define BTN_PRESSED	LOW



void delay()
{
	for (uint32_t i = 0; i < 500000; i += 1)
	{

	}
}
int main(void)
{
	// We want to turn on the led of the GPIO
	// Target: turn on the LED
	// Info: LED 2 is connected to PD13 (GPIOD pin 12)
	GPIO_Handle_t gpioLed;
	memset(&gpioLed,0,sizeof(gpioLed));
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);


	// Configure the button
	GPIO_Handle_t gpioButton;
	memset(&gpioButton,0,sizeof(gpioButton));
	gpioButton.pGPIOx = GPIOD;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_Init(&gpioButton);

//	GPIO_WriteToOutputPin(GPIOA, 12, GPIO_PIN_RESET);
	GPIO_IRQPriorityHandling(IRQ_NO_EXTI9_5, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	delay();
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
