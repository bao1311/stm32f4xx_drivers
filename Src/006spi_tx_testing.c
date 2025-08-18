/*
 * 006spi_tx_testing.c
 *
 *  Created on: Aug 17, 2025
 *      Author: gphi1
 */
#include <stdio.h>

/*
 * Requirements: Print hello world string through SPI2 peripheral
 */

/*
 * Alternate function:
 * PB12 		-> SPI2_NSS
 * PB13			-> SPI2_SCK
 * PB14			-> SPI2_MISO
 * PB15			-> SPI2_MOSI
 * Alternate function mode 5
 */
int main(void)
{
	printf("Hello world");
	return 0;
}

