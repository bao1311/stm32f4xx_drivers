/*
 * main.c
 *
 *  Created on: Jul 30, 2025
 *      Author: gphi1
 */

#include "stm32f407xx.h"

int main(void)
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
}
