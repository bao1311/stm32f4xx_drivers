/*
 * 001led_toggle.c
 *
 *  Created on: Jul 28, 2025
 *      Author: gphi1
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay()
{
	for (uint32_t i = 0; i < 100000; i += 1)
	{

	}
}
int main(void)
{
	// We want to turn on the led of the GPIO
	// Target: turn on the LED 3
	// Info: LED 3 is connected to PD13 (GPIOD pin 13)
	GPIO_Handle_t gpioLed;
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);


	// Configure the button
	GPIO_Handle_t gpioButton;
	gpioButton.pGPIOx = GPIOD;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioButton);

	while(1)
	{
		delay();
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	}


	return 0;
}
