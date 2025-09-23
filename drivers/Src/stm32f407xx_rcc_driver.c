#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};


uint32_t RCC_GetPCLK2Value()
{

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

