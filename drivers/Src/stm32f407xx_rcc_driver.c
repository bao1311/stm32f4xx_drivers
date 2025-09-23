#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx.h"
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};


uint32_t RCC_GetPCLK2Value()
{
	// Define SystemClk (HSI, HSE,..) and try to find ClkSource
	uint32_t SystemClk;
	uint8_t ClkSource;

	ClkSource = (RCC->CFGR >> 2) & (0x3); // Find SWS bit

	// Find SystemClk
	if (ClkSource == 0)
	{
		// HSI Clock Source
		SystemClk = 16000000U;
	}
	else if (ClkSource == 1)
	{
		// HSE Clock Source
		SystemClk = 8000000U;
	}
	else if (ClkSource == 2)
	{
		// PLL Clock Source
		SystemClk = RCC_GetPLLOutputClk();
	}

	// Find AHB Prescaler
	uint16_t ahb_prescaler = 0;
	// Find HPRE bit for ahb prescaler
	uint8_t hpre = (RCC->CFGR >> 4) & (0xF);
	// Assign ahb_prescaler
	if (hpre < 8)
	{
		ahb_prescaler = 1;
	}
	else
	{
		ahb_prescaler = AHB_PreScaler[hpre % 8];
	}
	// Find the ppre2 bit for APB2 (High-speed prescaler)
	uint8_t ppre2 = (RCC->CFGR >> 13) & (0x7);
	// Define ABP2 Prescaler
	uint16_t apb2_prescaler = 0;
	// Find APB2 Prescaler
	if (ppre2 < 4)
	{
		apb2_prescaler = 1;
	}
	else
	{
		apb2_prescaler = APB2_PreScaler[ppre2 % 4];
	}

	uint32_t pclk2 = (SystemClk / ahb_prescaler) / apb2_prescaler;
	return pclk2;

}

uint32_t RCC_GetPLLOutputClk()
{
	// Not mentioned inside the lecture
	return 0;
}
uint32_t RCC_GetPCLK1Value()
{
	uint32_t SystemClk;
	uint8_t ClkSource;
	ClkSource = (RCC->CFGR >> 2) & (0x3);
	if (ClkSource == 0)
	{
		// HSI oscillator as system clock
		SystemClk = 16000000U;
	}
	else if (ClkSource == 1)
	{
		// HSE oscillator as system clock
		SystemClk = 8000000U;
	}
	else if (ClkSource == 2)
	{
		// PLL selected as system clock
		SystemClk = RCC_GetPLLOutputClk();
	}
	uint8_t temp = (RCC->CFGR >> 4) & (0xF);
	uint16_t ahbp = 0;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}
	temp = (RCC->CFGR >> 10) & (0x7);
	uint8_t apb1 = 0;
	if (temp < 4)
	{
		apb1 = 1;
	}
	else
	{
		apb1 = APB1_PreScaler[temp - 4];
	}
	uint32_t pclk1 = (SystemClk/ahbp) / apb1;
	return pclk1;
}

