//#include <ch32v00x.h>
#include <ch32v003fun.h>
#include <ch32v00x_misc.h>
#include <system_ch32v00x.h>
#include <ch32v00x_gpio.h>
#include <ch32v00x_rcc.h>
#include <debug.h>

#include <ch32v00x_adc.h>
#include <ch32v00x_dbgmcu.h>
#include <ch32v00x_dma.h>
#include <ch32v00x_exti.h>
#include <ch32v00x_flash.h>
#include <ch32v00x_gpio.h>
#include <ch32v00x_i2c.h>
#include <ch32v00x_iwdg.h>
#include <ch32v00x_misc.h>
#include <ch32v00x_opa.h>
#include <ch32v00x_pwr.h>
#include <ch32v00x_rcc.h>
#include <ch32v00x_spi.h>
#include <ch32v00x_tim.h>
#include <ch32v00x_usart.h>
#include <ch32v00x_wwdg.h>



// WCH-Interrupt-fast can be properly emulated in standard GCC but this is a minimal example
void NMI_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));


int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();
	
	funGpioInitAll();

	/* Set up pins for Alternate Function*/
	// PC5 is SCK, 50MHz Output, alt func, p-p
	funPinMode( PC5, GPIO_CFGLR_OUT_50Mhz_AF_PP );
	
	// PC6 is MOSI, 50MHz Output, alt func, p-p
	funPinMode( PC6, GPIO_CFGLR_OUT_50Mhz_AF_PP );

	// PC0 is Software NSS, 50MHz Output, GPIO, p-p
	funPinMode( PC0, GPIO_CFGLR_OUT_50Mhz_PP );
	funDigitalWrite(PC0, FUN_HIGH);

	RCC->APB2PRSTR = RCC_SPI1RST;
	RCC->APB2PRSTR = 0;
	RCC->APB2PCENR |= RCC_APB2Periph_SPI1;
	//RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	// Configure SPI
	SPI1->CTLR1 = 
		SPI_NSS_Soft | SPI_CPHA_1Edge | SPI_CPOL_Low | SPI_DataSize_8b |
		SPI_Mode_Master | SPI_Direction_1Line_Tx | SPI_BaudRatePrescaler_16;

	// Enable TX DMA
	//SPI1->CTLR2 = SPI_CTLR2_TXDMAEN;

	//SPI1->HSCR = 1; // High-speed enable.
	SPI1->CTLR1 |= CTLR1_SPE_Set;

	//DMA1_Channel3 is for SPI1TX
	//DMA1_Channel3->PADDR = (uint32_t)&SPI1->DATAR;
	//DMA1_Channel3->MADDR = (uint32_t)sendbuff;
	//DMA1_Channel3->CFGR  =
//		DMA_M2M_Disable |
//		DMA_Priority_VeryHigh |
//		DMA_MemoryDataSize_Byte | // 8-bit mode
//		DMA_PeripheralDataSize_Byte |
//		DMA_MemoryInc_Enable |
//		DMA_Mode_Normal | // DMA_Mode_Normal
//		DMA_DIR_PeripheralDST |
//		DMA_IT_TC; // Transmission Complete

//	NVIC_EnableIRQ( DMA1_Channel3_IRQn );

	//uint32_t timestamp = 0;

	while(1)
	{
		if (!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY))
			SPI1->DATAR = 0xAA;
		/*
		if(SysTick->CNT - timestamp > 10000)
		{
			timestamp = SysTick->CNT;

			// Send repeatedly
			//initiate_transfer();
		}
		*/
/*
		// When DMA is done and last byte is clocked out, set CS high
		if(spi_state == DMA_DONE && !(SPI1->STATR & SPI_I2S_FLAG_BSY)){
			funDigitalWrite(PC0, FUN_HIGH); // Set CS high
			spi_state = IDLE;
		}
*/		
	}	
	
/*
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	BLINKY_CLOCK_ENABLE;
	GPIO_InitStructure.GPIO_Pin = BLINKY_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLINKY_GPIO_PORT, &GPIO_InitStructure);

	Delay_Ms(1000);
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	IWDG_SetReload(4095);
	IWDG_Enable();
	IWDG_ReloadCounter();

	uint8_t ledState = 0;
	while (1)
	{
		//IWDG_ReloadCounter(); //uncomment to prevent resetting
		GPIO_WriteBit(BLINKY_GPIO_PORT, BLINKY_GPIO_PIN, ledState);
		ledState ^= 1; // invert for the next run
		Delay_Ms(100);
	}
*/	

}

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
	while (1)
	{
	}
}
