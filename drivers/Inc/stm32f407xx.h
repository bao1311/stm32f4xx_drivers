/*
 * stm32f407xx.h
 *
 *  Created on: Jul 24, 2025
 *      Author: gphi1
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * Type Macros
 */
#define __vo volatile
#define __weak __attribute__((weak))



#include <stddef.h>
#include <stdint.h>
/*
 * ARM Cortex M4 Address Section
 */

/*
 * Define NVIC Priority level
 */
#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15

// NVIC ISER(Interrupt Set-enable Register) address
#define NVIC_ISER0		(__vo uint32_t*)0xE000E100
#define NVIC_ISER1		(__vo uint32_t*)0xE000E104
#define NVIC_ISER2		(__vo uint32_t*)0xE000E108
#define NVIC_ISER3		(__vo uint32_t*)0xE000E10C

// NVIC ICER(Interrupt Clear-enable Register) address
#define NVIC_ICER0		(__vo uint32_t*)0xE000E180
#define NVIC_ICER1		(__vo uint32_t*)0xE000E184
#define NVIC_ICER2		(__vo uint32_t*)0xE000E188
#define NVIC_ICER3		(__vo uint32_t*)0xE000E18C

// NVIC IPR(Interrupt Priority Register) Address
#define NVIC_PR_BASEADDR		(__vo uint32_t*)0xE000E400

/*
 * ARM Cortex Mx Processor number of implemented in Priority register
 */
#define NO_PR_BITS_IMPLEMENTED		4
/*
 * *****************************************************************
 */

#define FLASH_BASEADDR	0x08000000U
#define SRAM1_BASEADDR 	0x20000000U
#define SRAM2_BASEADDR	0x2001C000U
#define ROM 			0x1FFF7800U
#define SRAM			SRAM1_BASEADDR

/*
 * Define SET and RESET value
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/*
 * *******************************************************
 * Bit position definitions of I2C peripheral
 * *******************************************************
 */
/*
 * I2C_CR1 register
 */
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

/*
 * I2C_CR2 register
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12


/*
 * I2C_SR1 register
 */
#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define	I2C_SR1_RxNE					6
#define	I2C_SR1_TxE						7
#define	I2C_SR1_BERR					8
#define	I2C_SR1_ARLO					9
#define	I2C_SR1_AF						10
#define	I2C_SR1_OVR						11
#define	I2C_SR1_PECERR					12
#define	I2C_SR1_TIMEOUT					14
#define	I2C_SR1_SMBALERT				15


/*
 *
 *********************************************************
 *Bit position definitions of SPI peripheral
 *********************************************************
 */

// SPI_CR1 register
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

// SPI_CR2 register
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

// SPI_SR register
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

// SPI_DR register
#define SPI_DR					0

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR	0x40000000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000U
#define RCC_BASEADDR 		0x40023800U

/*
 * Define base address for each peripherals in each bus
 */
// AHB1
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + 0x2000)
// APB1
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00)

// APB2
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)
#define UART5_BASEADDR (APB1PERIPH_BASEADDR + 0X5000)

typedef struct
{
	__vo uint32_t MEMRMP;		// SYSCFG memory remap register
	__vo uint32_t PMC;			// SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];	// SYSCFG external interrupt configuration register 1,2,3,4
	uint32_t RESERVED1[2];		// Reserved from 0x18-0x1C
	__vo uint32_t CMPCR;		// Compensation cell control register
	uint32_t RESERVED2[2];		// Reserved from 0x24-0x28
	__vo uint32_t CFGR; 		// Address offset: 0x2C


}SYSCFG_RegDef_t;
#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)
typedef struct
{
	__vo uint32_t IMR;			// EXTI interrupt mask register
	__vo uint32_t EMR;			// EXTI event mask register
	__vo uint32_t RTSR;			// EXTI rising trigger selection register
	__vo uint32_t FTSR;			// EXTI falling trigger selection register
	__vo uint32_t SWIER;		// EXTI software interrupt event register
	__vo uint32_t PR;			// EXTI pending register

}EXTI_RegDef_t;
#define EXTI ((EXTI_RegDef_t*) EXTI_BASEADDR)

/*
 * I2C register mappings
 */
typedef struct
{
	__vo uint32_t CR1;			// I2C Control register 1
	__vo uint32_t CR2;			// I2C Control register 2
	__vo uint32_t OAR1;			// I2C Own address register 1
	__vo uint32_t OAR2;			// I2C Own address register 2
	__vo uint32_t DR;			// I2C Data register
	__vo uint32_t SR1;			// I2C Status register 1
	__vo uint32_t SR2;			// I2C Status register 2
	__vo uint32_t CCR;			// I2C Clock control register
	__vo uint32_t TRISE;		// I2C TRISE register
	__vo uint32_t FLTR;			// I2C FLTR register
}I2C_RegDef_t;
#define I2C1 ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t*) I2C3_BASEADDR)
/*
 * SPI register mappings
 */
typedef struct
{
	__vo uint32_t CR1;		// SPI control register 1
	__vo uint32_t CR2;		// SPI control register 2
	__vo uint32_t SR;		// SPI status register
	__vo uint32_t DR;		// SPI data register
	__vo uint32_t CRCPR;	// SPI CRC polynomial register
	__vo uint32_t RXCRCR;	// SPI RX CRC register
	__vo uint32_t TXCRCR;	// SPI TX CRC register
	__vo uint32_t I2SCFGR;	// SPI_I2S configuration register
	__vo uint32_t I2SPR;	// SPI_I2S prescaler register

}SPI_RegDef_t;
#define SPI1 ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*) SPI3_BASEADDR)

/*
 * GPIO register mappings
 */
typedef struct
{
	__vo uint32_t MODER;		// GPIO port mode register
	__vo uint32_t OTYPER; 		// GPIO port output type register
	__vo uint32_t OSPEEDR;		// GPIO port output speed register
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			// GPIO port input data register
	__vo uint32_t ODR;			// GPIO port output data register
	__vo uint32_t BSRR;			// GPIO port bit set/reset register
	__vo uint32_t LCKR;			// GPIO port configuration lock register
	__vo uint32_t AFR[2];		// GPIO alternate function register (array[2]) one for low and one for high
}GPIO_RegDef_t;


typedef struct
{
 	__vo uint32_t CR;			// RCC clock control register
 	__vo uint32_t PLLCFGR;		// RCC PLL configuration register
 	__vo uint32_t CFGR;			// RCC clock configuration register
	__vo uint32_t CIR;			// RCC clock interrupt register
	__vo uint32_t AHB1RSTR; 	// RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register
	uint32_t RESERVED0; 			// Reserved
	__vo uint32_t APB1RSTR; 	// RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR; 	// RCC APB2 peripheral reset register
	uint32_t RESERVED1[2]; 		// 2 block of reserved
	__vo uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR; 		// RCC AHB3 peripheral clock enable register
	uint32_t RESERVED2;			// Reserved
	__vo uint32_t APB1ENR; 		// RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;  	// RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2]; 		// 2 block of reserved
	__vo uint32_t AHB1LPENR; 	// RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR; 	// RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR; 	// RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4; 			// Reserved block
	__vo uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR; 	// RCC APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5[2]; 		// 2 blocks of reserved
	__vo uint32_t BDCR; 		// RCC Back up domain control register
	__vo uint32_t CSR; 			// RCC clock control & status register
	uint32_t RESERVED6[2];		// 2 blocks of reserved
	__vo uint32_t SSCGR;		// RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register


}RCC_RegDef_t;


/*
 * peripheral definitions
 */
#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC ((RCC_RegDef_t*) RCC_BASEADDR)

/*
 * SPI Peripheral Reset
 */
#define SPI1_PCLK_RESET()		do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_PCLK_RESET()		do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_PCLK_RESET()		do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)

/*
 * GPIO Peripheral Reset
 */
#define GPIOA_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_PCLK_RESET() 		do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()			(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()			(RCC->AHB1ENR |= (1 << 10))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))



/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))






/*
 * Clock Disable Macros for GPIOx peripherals
 */


#define GPIOA_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 10))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))
/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))


/*
 * IRQ (Interrupt Request) Numbers of STM32f407 MCU
 * Can complete this for other peripherals
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * IRQ Number of STM32F407 MCU for SPI
 */
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
