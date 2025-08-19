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
#define BLINKY_GPIO_PORT GPIOC
#define BLINKY_GPIO_PIN GPIO_Pin_2
#define BLINKY_CLOCK_ENABLE RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)

// WCH-Interrupt-fast can be properly emulated in standard GCC but this is a minimal example
void NMI_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void); //__attribute__((interrupt("WCH-Interrupt-fast")));


void initializeTimerPWM(uint16_t CCR)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure );

    TIM_Cmd( TIM1, DISABLE );
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0}; 
	
    TIM_TimeBaseInitStructure.TIM_Period = 480;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = CCR; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM1, &TIM_OCInitStructure );

    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Enable );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );
}

void InitializeADC()
{
     ADC_InitTypeDef ADC_InitStructure = {0};
     GPIO_InitTypeDef GPIO_InitStructure = {0};

     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
     RCC_ADCCLKConfig(RCC_PCLK2_Div8);

     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
     GPIO_Init(GPIOD, &GPIO_InitStructure); 

     ADC_DeInit(ADC1);
     ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
     ADC_InitStructure.ADC_ScanConvMode = DISABLE;
     ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
     ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
     ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
     ADC_InitStructure.ADC_NbrOfChannel = 1; 
     ADC_Init(ADC1, &ADC_InitStructure);

     ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_241Cycles);


     ADC_Cmd(ADC1, ENABLE);
     ADC_ResetCalibration(ADC1);
     while(ADC_GetResetCalibrationStatus(ADC1));
     ADC_StartCalibration(ADC1);
     while(ADC_GetCalibrationStatus(ADC1));

     ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


int main(void)
{
	SystemInit();

	InitializeADC();

	SystemCoreClockUpdate();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	initializeTimerPWM(1);

	uint8_t ledState = 0;
	int i=0;
	uint16_t adcValue = 0;
	while (1)
	{
		//GPIO_WriteBit(BLINKY_GPIO_PORT, BLINKY_GPIO_PIN, ledState);
		//ledState ^= 1; // invert for the next run
		Delay_Ms(1);
		   uint32_t tmp;// = ADC_GetConversionValue(ADC1);
		   for(int j=0;j<100000;j++)
		   {
				tmp += ADC_GetConversionValue(ADC1);
		   }
		   tmp = tmp/135000;
		   if ( tmp != adcValue)
		   {
			adcValue = tmp;
			initializeTimerPWM(adcValue);
		   }
/*			
		if (adcValue > 100)
		{
		initializeTimerPWM(i++);
		if (i==480 ) i=0;
		}
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
