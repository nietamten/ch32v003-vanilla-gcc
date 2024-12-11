/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_llcc68_interface_template.c
 * @brief     driver llcc68 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2023-04-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/04/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */
 
#include "driver_llcc68_interface.h"


#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>
#include <stdint.h>

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

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t llcc68_interface_spi_init(void)
{
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
/*
	SPI1->CTLR1 = 
		SPI_NSS_Soft | SPI_CPHA_1Edge | SPI_CPOL_Low | SPI_DataSize_8b |
		SPI_Mode_Master | SPI_Direction_2Lines_FullDuplex | SPI_BaudRatePrescaler_16;

    SPI1->CTLR1 |= 1<<8;
    SPI1->CTLR2 |= SPI_CTLR2_SSOE; 
    
    
    SPI1->CTLR1 |= CTLR1_SPE_Set;
*/

{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);    
    GPIO_WriteBit(GPIOC,GPIO_Pin_1,1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    SPI_InitTypeDef  SPI_InitStructure = {0};

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_SSOutputCmd(SPI1, DISABLE);

    
    //SPI1->CTLR2 |= SPI_CTLR2_SSOE; 
    //SPI1->CTLR1 |= 1<<8;
    //SPI_SSOutputCmd(SPI1,ENABLE);
    //SPI1->CTLR1 &= ~(1<<8);
    //SPI1->CTLR1 |= 1<<8;

    SPI_Cmd(SPI1, ENABLE);

}

    //SPI_NSSInternalSoftwareConfig(SPI1,SPI_NSSInternalSoft_Set);
    //SPI1->CTLR1 |=  SPI_NSSInternalSoft_Reset;
    
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t llcc68_interface_spi_deinit(void)
{
   	SPI1->CTLR1 &= ~CTLR1_SPE_Set;
    return 0;
}

/**
 * @brief      interface spi bus write read
 * @param[in]  *in_buf points to a input buffer
 * @param[in]  in_len is the input length
 * @param[out] *out_buf points to a output buffer
 * @param[in]  out_len is the output length
 * @return     status code
 *             - 0 success
 *             - 1 write read failed
 * @note       none
 */
uint8_t llcc68_interface_spi_write_read(uint8_t *in_buf, uint32_t in_len,
                                        uint8_t *out_buf, uint32_t out_len)
{
    GPIO_WriteBit(GPIOC,GPIO_Pin_1,0);

    for(uint32_t i=0; i<in_len  ;i++){
        while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY));
        SPI1->DATAR = in_buf[i];
        llcc68_interface_debug_print("SPI s %d %d \r\n",1,in_buf[i]);
    }
    for(uint32_t i=0; i<out_len  ;i++){
        while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY));
        SPI1->DATAR = 0;
        while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY));
        out_buf[i] = SPI1->DATAR;
        llcc68_interface_debug_print("SPI r %d %d \r\n",1,out_buf[i]);
    }    
    GPIO_WriteBit(GPIOC,GPIO_Pin_1,1);
    /*
    for(uint32_t i=0; i<in_len || i<out_len ;i++){
        while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY));
        if (i < out_len)
            SPI1->DATAR = out_buf[i];
        else
            SPI1->DATAR = 0x00;
        while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY));
        if (i<in_len)
            out_buf[i] = SPI1->DATAR;
    }
    if (out_len)
    {
        int x = out_buf[0];
        llcc68_interface_debug_print("SPI %d %d %d\n",out_len, out_len, x);
    }
    */
    return 0;
}

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t llcc68_interface_reset_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

    return 0;
}

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t llcc68_interface_reset_gpio_deinit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

    return 0;
}

/**
 * @brief     interface reset gpio write
 * @param[in] data is the written data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t llcc68_interface_reset_gpio_write(uint8_t data)
{
    GPIO_WriteBit(GPIOC,GPIO_Pin_3,data);
    return 0;
}

/**
 * @brief  interface busy gpio init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t llcc68_interface_busy_gpio_init(void)
{
   	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	

    return 0;
}

/**
 * @brief  interface busy gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t llcc68_interface_busy_gpio_deinit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	

    return 0;
}

/**
 * @brief      interface busy gpio read
 * @param[out] *value points to a value buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t llcc68_interface_busy_gpio_read(uint8_t *value)
{
    *value =     GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void llcc68_interface_delay_ms(uint32_t ms)
{
    Delay_Ms(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void llcc68_interface_debug_print(const char *const fmt, ...)
{
	va_list args;
	va_start( args, fmt );
	printf(fmt, args);
	va_end( args );
}

/**
 * @brief     interface receive callback
 * @param[in] type is the receive callback type
 * @param[in] *buf points to a buffer address
 * @param[in] len is the buffer length
 * @note      none
 */
void llcc68_interface_receive_callback(uint16_t type, uint8_t *buf, uint16_t len)
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
            llcc68_interface_debug_print("llcc68: irq rx done.\n");
            
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
            llcc68_interface_debug_print("llcc68: unknown code.\n");
            
            break;
        }
    }
}
