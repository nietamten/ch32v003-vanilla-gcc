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

/*!! CHANGE HERE IF NEEDED !! */
#define BLINKY_GPIO_PORT GPIOD
#define BLINKY_GPIO_PIN GPIO_Pin_4
#define BLINKY_CLOCK_ENABLE RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE)

// WCH-Interrupt-fast can be properly emulated in standard GCC but this is a minimal example
void NMI_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));

volatile uint8_t ledState = 0;

void EXTI7_0_IRQHandler( void ) __attribute__((interrupt));
void EXTI7_0_IRQHandler( void ) 
{
	{
		GPIO_WriteBit(BLINKY_GPIO_PORT, BLINKY_GPIO_PIN, ledState);
		ledState ^= 1; // invert for the next run
		Delay_Ms(100);
	}
		
	// Acknowledge the interrupt
	EXTI_ClearFlag( EXTI_Line3 );
}

int main(void)
{
	SystemInit();

	SystemCoreClockUpdate();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	BLINKY_CLOCK_ENABLE;
	GPIO_InitStructure.GPIO_Pin = BLINKY_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLINKY_GPIO_PORT, &GPIO_InitStructure);

	RCC->APB2PCENR = RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	

	AFIO->EXTICR = AFIO_EXTICR_EXTI3_PD;

	EXTI_InitTypeDef exti_a;
	EXTI_Init(&exti_a);
	exti_a.EXTI_Line = EXTI_Line3;
	exti_a.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_a.EXTI_Trigger = EXTI_Trigger_Rising;
	exti_a.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_a);

	NVIC_EnableIRQ( EXTI7_0_IRQn );


	while (1)
	{
		asm volatile( "nop" );
	}

}

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
	while (1)
	{
	}
}
