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

#include "driver_llcc68_lora.h"

#define UART_BR 115200

void initUART(){
		// Enable UART and GPIOD
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;

	// Push-Pull, 10MHz Output on D5, with AutoFunction
	GPIOD->CFGLR = (GPIOD->CFGLR & ~(0xF<<(4*5))) |
			((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5));

	// Setup UART for Tx 8n1
	USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx | USART_Mode_Rx;;
	USART1->CTLR2 = USART_StopBits_1;

	// Set baud rate and enable UART
	USART1->BRR = ((FUNCONF_SYSTEM_CORE_CLOCK) + (UART_BR)/2) / (UART_BR);
	USART1->CTLR1 |= CTLR1_UE_Set;
}

// WCH-Interrupt-fast can be properly emulated in standard GCC but this is a minimal example
void NMI_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));


int gpio_interrupt_init();
void gpio_interrupt_deinit();

//////////////////////////////
uint8_t (*g_gpio_irq)(void) = NULL;
uint8_t res;
static uint8_t gs_rx_done;
uint32_t timeout;


static void a_callback(uint16_t type, uint8_t *buf, uint16_t len)
{
    switch (type)
    {
        case LLCC68_IRQ_TX_DONE :
        {
            llcc68_interface_debug_print("llcc68: irq tx done.\n");
            

            break;
        }
        case LLCC68_IRQ_RX_DONE :
        {
            uint16_t i;
            llcc68_bool_t enable;
            float rssi;
            float snr;
            
            llcc68_interface_debug_print("llcc68: irq rx done.\n");
            
            /* get the status */
            if (llcc68_lora_get_status((float *)&rssi, (float *)&snr) != 0)
            {
                return;
            }
            llcc68_interface_debug_print("llcc68: rssi is %0.1f.\n", rssi);
            llcc68_interface_debug_print("llcc68: snr is %0.2f.\n", snr);
            
            /* check the error */
            if (llcc68_lora_check_packet_error(&enable) != 0)
            {
                return;
            }
            if ((enable == LLCC68_BOOL_FALSE) && len)
            {
                for (i = 0; i < len; i++)
                {
                    llcc68_interface_debug_print("x %d %d",8, buf[i]);
                }
                llcc68_interface_debug_print("\n");
                gs_rx_done = 1;
            }
            
            break;
        }
        case LLCC68_IRQ_PREAMBLE_DETECTED :
        {
            llcc68_interface_debug_print("llcc68: irq preamble detected.\n");
            
            break;
        }
        case LLCC68_IRQ_SYNC_WORD_VALID :
        {
            llcc68_interface_debug_print("llcc68: irq valid sync word detected.\n");
            
            break;
        }
        case LLCC68_IRQ_HEADER_VALID :
        {
            llcc68_interface_debug_print("llcc68: irq valid header.\n");
            
            break;
        }
        case LLCC68_IRQ_HEADER_ERR :
        {
            llcc68_interface_debug_print("llcc68: irq header error.\n");
            
            break;
        }
        case LLCC68_IRQ_CRC_ERR :
        {
            llcc68_interface_debug_print("llcc68: irq crc error.\n");
            
            break;
        }
        case LLCC68_IRQ_CAD_DONE :
        {
            llcc68_interface_debug_print("llcc68: irq cad done.\n");
            
            break;
        }
        case LLCC68_IRQ_CAD_DETECTED :
        {
            llcc68_interface_debug_print("llcc68: irq cad detected.\n");
            
            break;
        }
        case LLCC68_IRQ_TIMEOUT :
        {
            llcc68_interface_debug_print("llcc68: irq timeout.\n");
            
            break;
        }
        default :
        {
            break;
        }
    }
}

int lora_send()
{
	/* gpio init */
	res = gpio_interrupt_init();
	if (res != 0)
	{
		return 1;
	}
	g_gpio_irq = llcc68_lora_irq_handler;

	llcc68_interface_debug_print("preinit");
	res = llcc68_lora_init(a_callback);
	if (res != 0)
	{
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}
	llcc68_interface_debug_print("postinit");

	/* set send mode */
	res = llcc68_lora_set_send_mode();
	if (res != 0)
	{
		(void)llcc68_lora_deinit();
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}

	llcc68_interface_debug_print("llcc68: send %s.\n", "123");

	/* send data */
	res = llcc68_lora_send((uint8_t *)"123", strlen("123"));
	if (res != 0)
	{
		(void)llcc68_lora_deinit();
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}

	/* deinit */
	res = llcc68_lora_deinit();
	if (res != 0)
	{
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}
	(void)gpio_interrupt_deinit();
	g_gpio_irq = NULL;

	return 0;
}

int lora_receive()
{

	/* gpio init */
	res = gpio_interrupt_init();
	if (res != 0)
	{
		return 1;
	}
	g_gpio_irq = llcc68_lora_irq_handler;

	/* lora init */
	res = llcc68_lora_init(a_callback);
	if (res != 0)
	{
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}

	/* start receiving */
	llcc68_interface_debug_print("llcc68: start receiving...\n");
	gs_rx_done = 0;
	timeout = 3000;

	/* start receive */
	res = llcc68_lora_set_continuous_receive_mode();
	if (res != 0)
	{
		(void)llcc68_lora_deinit();
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}

	while ((timeout != 0) && (gs_rx_done == 0))
	{
		timeout--;
		llcc68_interface_delay_ms(1000);
	}
	if (gs_rx_done == 0)
	{
		/* receive timeout */
		llcc68_interface_debug_print("llcc68: receive timeout.\n");
		(void)llcc68_lora_deinit();
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}

	/* deinit */
	res = llcc68_lora_deinit();
	if (res != 0)
	{
		(void)gpio_interrupt_deinit();
		g_gpio_irq = NULL;

		return 1;

	}
	(void)gpio_interrupt_deinit();
	g_gpio_irq = NULL;

	return 0;
}
//////////////////////////////


void EXTI7_0_IRQHandler( void ) __attribute__((interrupt));
void EXTI7_0_IRQHandler( void ) 
{
	if (g_gpio_irq)
		g_gpio_irq();	
	// Acknowledge the interrupt
	EXTI_ClearFlag( EXTI_Line3 );
}


int gpio_interrupt_init(){
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;

	GPIO_InitTypeDef GPIO_InitStructure = {0};
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
	return 0;
}

void gpio_interrupt_deinit(){
	EXTI_InitTypeDef exti_a;
	EXTI_Init(&exti_a);
	exti_a.EXTI_Line = EXTI_Line3;
	exti_a.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_a.EXTI_Trigger = EXTI_Trigger_Rising;
	exti_a.EXTI_LineCmd = DISABLE;
	EXTI_Init(&exti_a);
}

#define BLINKY_GPIO_PORT GPIOA
#define BLINKY_GPIO_PIN GPIO_Pin_1
#define BLINKY_CLOCK_ENABLE RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)

int __io_putchar(int ch) {
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, ch);
    return ch;      
}

uint8_t receiver = 0;

int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();	
/*
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	BLINKY_CLOCK_ENABLE;
	GPIO_InitStructure.GPIO_Pin = BLINKY_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLINKY_GPIO_PORT, &GPIO_InitStructure);

	uint8_t ledState = 0;
*/



	funGpioInitAll();
	initUART();

	while(1)
	{
		if (receiver)
		{
			lora_receive();	
		}
		else {
			lora_send();
		}
				
/*
		GPIO_WriteBit(BLINKY_GPIO_PORT, BLINKY_GPIO_PIN, ledState);
		ledState ^= 1; // invert for the next run
		Delay_Ms(1000);
*/
	}	


}

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
	while (1)
	{
	}
}
