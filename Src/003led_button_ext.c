/*
 * 003led_button_ext.c
 *
 *  Created on: Jul 29, 2025
 *      Author: gphi1
 */



#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"



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
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);


	// Configure the button
	GPIO_Handle_t gpioButton;
	gpioButton.pGPIOx = GPIOB;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&gpioButton);

	while (1)
	{

		if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
			delay();
		}
	}


	return 0;
}

