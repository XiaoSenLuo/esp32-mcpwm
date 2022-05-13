/*
 * @Author: your name
 * @Date: 2022-03-24 15:36:31
 * @LastEditTime: 2022-05-10 19:39:14
 * @LastEditors: xiaosenluo xiaosenluo@yandex.com
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \orsystem\main\orsystem_gpio.h
 */

#ifndef _ORSYSTEM_GPIO_H__
#define _ORSYSTEM_GPIO_H__

#include "sdkconfig.h"


#define IO_UART0_TX     1
#define IO_UART0_RX     3


#if(CONFIG_ORSYSTEM_MODE)
/*!<DQC IO*/
#define IO_CAN_TX       21
#define IO_CAN_RX       22
#define IO_EN_BZ        25
#define IO_VSPI_CLK     18
#define IO_VSPI_MISO    19
#define IO_VSPI_MOSI    23
#define IO_VSPI_CS      5
#define IO_I2C_SCL      4
#define IO_I2C_SDA      0

#define IO_ADS_ADRDY    35
#define IO_EN_CH        12
#define IO_ADS_RESET    16
#define IO_EN_AP        17


#else

/*!<Signsl Generator IO*/
#define IO_CAN_TX         25
#define IO_CAN_RX         26
#define IO_PWMA_OUTPUT    22
#define IO_PWMB_OUTPUT    21
#define IO_PWM_CS         19

#define IO_S1             12
#define IO_S2             2
#define IO_K1             34
#define IO_K2             35
#define IO_K3             27
#define IO_K4             14
#define IO_ST             4

#endif

#endif