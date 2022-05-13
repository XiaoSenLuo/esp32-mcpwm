/*
 * @Author: your name
 * @Date: 2021-10-06 17:12:14
 * @LastEditTime: 2022-04-29 08:25:53
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \orsystem\components\ads131a\ads131a.h
 */


#ifndef _ADS131A_H__
#define _ADS131A_H__

#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"

#include "ads131a_driver.h"

#ifdef __cplusplus
extern "C" {
#endif



/* Register 0x00 (ID_MSB) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |                                           NU_CH[7:0]                                          |
 * -------------------------------------------------------------------------------------------------
 */

    /* ID_MSB register address */
    #define ID_MSB_ADDRESS													((uint8_t) 0x00)

    /* ID_MSB register field masks */
    #define ID_MSB_NU_CH_MASK												((uint8_t) 0xFF)

    /* NU_CH field values */
    #define ID_MSB_NU_CH_2													((uint8_t) 0x02)
    #define ID_MSB_NU_CH_4													((uint8_t) 0x04)

/* Register 0x01 (ID_LSB) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |                                          REV_ID[7:0]                                          |
 * -------------------------------------------------------------------------------------------------
 */

    /* ID_LSB register address */
    #define ID_LSB_ADDRESS													((uint8_t) 0x01)

    /* ID_LSB register field masks */
    #define ID_LSB_REV_ID_MASK												((uint8_t) 0xFF)



/* Register 0x02 (STAT_1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |   F_OPC   |   F_SPI   |  F_ADCIN  |   F_WDT   |  F_RESYNC |   F_DRDY  |  F_CHECK  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_1 register address */
    #define STAT_1_ADDRESS													((uint8_t) 0x02)

    /* STAT_1 default (reset) value */
    #define STAT_1_DEFAULT													((uint8_t) 0x00)

    /* STAT_1 register field masks */
    #define STAT_1_F_OPC_MASK												((uint8_t) 0x40)
    #define STAT_1_F_SPI_MASK												((uint8_t) 0x20)
    #define STAT_1_F_ADCIN_MASK												((uint8_t) 0x10)
    #define STAT_1_F_WDT_MASK												((uint8_t) 0x08)
    #define STAT_1_F_RESYNC_MASK											((uint8_t) 0x04)
    #define STAT_1_F_DRDY_MASK												((uint8_t) 0x02)
    #define STAT_1_F_CHECK_MASK												((uint8_t) 0x01)



/* Register 0x03 (STAT_P) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |   F_IN4P  |   F_IN3P  |   F_IN2P  |   F_IN1P  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_P register address */
    #define STAT_P_ADDRESS													((uint8_t) 0x03)

    /* STAT_P default (reset) value */
    #define STAT_P_DEFAULT													((uint8_t) 0x00)

    /* STAT_P register field masks */
    #define STAT_P_F_IN4P_MASK												((uint8_t) 0x08)
    #define STAT_P_F_IN3P_MASK												((uint8_t) 0x04)
    #define STAT_P_F_IN2P_MASK												((uint8_t) 0x02)
    #define STAT_P_F_IN1P_MASK												((uint8_t) 0x01)



/* Register 0x04 (STAT_N) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |   F_IN4N  |   F_IN3N  |   F_IN2N  |   F_IN1N  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_N register address */
    #define STAT_N_ADDRESS													((uint8_t) 0x04)

    /* STAT_N default (reset) value */
    #define STAT_N_DEFAULT													((uint8_t) 0x00)

    /* STAT_N register field masks */
    #define STAT_N_F_IN4N_MASK												((uint8_t) 0x08)
    #define STAT_N_F_IN3N_MASK												((uint8_t) 0x04)
    #define STAT_N_F_IN2N_MASK												((uint8_t) 0x02)
    #define STAT_N_F_IN1N_MASK												((uint8_t) 0x01)



/* Register 0x05 (STAT_S) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     | F_STARTUP |    F_CS   |  F_FRAME  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_S register address */
    #define STAT_S_ADDRESS													((uint8_t) 0x05)

    /* STAT_S default (reset) value */
    #define STAT_S_DEFAULT													((uint8_t) 0x00)

    /* STAT_S register field masks */
    #define STAT_S_F_STARTUP_MASK											((uint8_t) 0x04)
    #define STAT_S_F_CS_MASK												((uint8_t) 0x02)
    #define STAT_S_F_FRAME_MASK												((uint8_t) 0x01)



/* Register 0x06 (ERROR_CNT) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |                                            ER[7:0]                                            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ERROR_CNT register address */
    #define ERROR_CNT_ADDRESS												((uint8_t) 0x06)

    /* ERROR_CNT default (reset) value */
    #define ERROR_CNT_DEFAULT												((uint8_t) 0x00)

    /* ERROR_CNT register field masks */
    #define ERROR_CNT_ER_MASK												((uint8_t) 0xFF)



/* Register 0x07 (STAT_M2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |       M2PIN[1:0]      |       M1PIN[1:0]      |       M0PIN[1:0]      |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_M2 register address */
    #define STAT_M2_ADDRESS													((uint8_t) 0x07)

    /* STAT_M2 default (reset) value */
    #define STAT_M2_DEFAULT													((uint8_t) 0x00)
    #define STAT_M2_DEFAULT_MASK											((uint8_t) 0xC0)

    /* STAT_M2 register field masks */
    #define STAT_M2_M2PIN_MASK												((uint8_t) 0x30)
    #define STAT_M2_M1PIN_MASK												((uint8_t) 0x0C)
    #define STAT_M2_M0PIN_MASK												((uint8_t) 0x03)

    /* M2PIN field values */
    #define STAT_M2_M2PIN_M2_HAMMING_OFF									((uint8_t) 0x00)
    #define STAT_M2_M2PIN_M2_HAMMING_ON										((uint8_t) 0x10)
    #define STAT_M2_M2PIN_M2_NC												((uint8_t) 0x20)

    /* M1PIN field values */
    #define STAT_M2_M1PIN_M1_24BIT											((uint8_t) 0x00)
    #define STAT_M2_M1PIN_M1_32BIT											((uint8_t) 0x04)
    #define STAT_M2_M1PIN_M1_16BIT											((uint8_t) 0x08)

    /* M0PIN field values */
    #define STAT_M2_M0PIN_M0_SYNC_MASTER									((uint8_t) 0x00)
    #define STAT_M2_M0PIN_M0_ASYNC_SLAVE									((uint8_t) 0x01)
    #define STAT_M2_M0PIN_M0_SYNC_SLAVE										((uint8_t) 0x02)



/* Register 0x08 (RESERVED0) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */


/* Register 0x09 (RESERVED1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x0A (RESERVED2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x0B (A_SYS_CFG) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |   VNCPEN  |    HRM    |     0     |  VREF_4V  | INT_REFEN |            COMP_TH[2:0]           |
 * -------------------------------------------------------------------------------------------------
 */

    /* A_SYS_CFG register address */
    #define A_SYS_CFG_ADDRESS												((uint8_t) 0x0B)

    /* A_SYS_CFG default (reset) value */
    #define A_SYS_CFG_DEFAULT												((uint8_t) 0x60)

    /* A_SYS_CFG register field masks */
    #define A_SYS_CFG_VNCPEN_MASK											((uint8_t) 0x80)
    #define A_SYS_CFG_HRM_MASK												((uint8_t) 0x40)
    #define A_SYS_CFG_VREF_4V_MASK											((uint8_t) 0x10)
    #define A_SYS_CFG_INT_REFEN_MASK										((uint8_t) 0x08)
    #define A_SYS_CFG_COMP_TH_MASK											((uint8_t) 0x07)

    /* COMP_TH field values */
    #define A_SYS_CFG_COMP_TH_HIGH_95_LOW_5									((uint8_t) 0x00)
    #define A_SYS_CFG_COMP_TH_HIGH_92p5_LOW_7p5								((uint8_t) 0x01)
    #define A_SYS_CFG_COMP_TH_HIGH_90_LOW_10								((uint8_t) 0x02)
    #define A_SYS_CFG_COMP_TH_HIGH_87p5_LOW_12p5							((uint8_t) 0x03)
    #define A_SYS_CFG_COMP_TH_HIGH_85_LOW_15								((uint8_t) 0x04)
    #define A_SYS_CFG_COMP_TH_HIGH_80_LOW_20								((uint8_t) 0x05)
    #define A_SYS_CFG_COMP_TH_HIGH_75_LOW_25								((uint8_t) 0x06)
    #define A_SYS_CFG_COMP_TH_HIGH_70_LOW_30								((uint8_t) 0x07)



/* Register 0x0C (D_SYS_CFG) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |   WDT_EN  |  CRC_MODE |       DNDLY[1:0]      |      HIZDLY[1:0]      |   FIXED   |   CRC_EN  |
 * -------------------------------------------------------------------------------------------------
 */

    /* D_SYS_CFG register address */
    #define D_SYS_CFG_ADDRESS												((uint8_t) 0x0C)

    /* D_SYS_CFG default (reset) value */
    #define D_SYS_CFG_DEFAULT												((uint8_t) 0x3C)

    /* D_SYS_CFG register field masks */
    #define D_SYS_CFG_WDT_EN_MASK											((uint8_t) 0x80)
    #define D_SYS_CFG_CRC_MODE_MASK											((uint8_t) 0x40)
    #define D_SYS_CFG_DNDLY_MASK											((uint8_t) 0x30)
    #define D_SYS_CFG_HIZDLY_MASK											((uint8_t) 0x0C)
    #define D_SYS_CFG_FIXED_MASK											((uint8_t) 0x02)
    #define D_SYS_CFG_CRC_EN_MASK											((uint8_t) 0x01)

    /* DNDLY field values */
    #define D_SYS_CFG_DNDLY_6ns												((uint8_t) 0x00)
    #define D_SYS_CFG_DNDLY_8ns												((uint8_t) 0x10)
    #define D_SYS_CFG_DNDLY_10ns											((uint8_t) 0x20)
    #define D_SYS_CFG_DNDLY_12ns											((uint8_t) 0x30)

    /* HIZDLY field values */
    #define D_SYS_CFG_HIZDLY_6ns											((uint8_t) 0x00)
    #define D_SYS_CFG_HIZDLY_8ns											((uint8_t) 0x04)
    #define D_SYS_CFG_HIZDLY_10ns											((uint8_t) 0x08)
    #define D_SYS_CFG_HIZDLY_12ns											((uint8_t) 0x0C)



/* Register 0x0D (CLK1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |   CLKSRC  |     0     |     0     |     0     |            CLK_DIV[2:0]           |     0     |
 * -------------------------------------------------------------------------------------------------
 */

    /* CLK1 register address */
    #define CLK1_ADDRESS													((uint8_t) 0x0D)

    /* CLK1 default (reset) value */
    #define CLK1_DEFAULT													((uint8_t) 0x08)

    /* CLK1 register field masks */
    #define CLK1_CLKSRC_MASK												((uint8_t) 0x80)
    #define CLK1_CLK_DIV_MASK												((uint8_t) 0x0E)

    /* CLK_DIV field values */
    #define CLK1_CLK_DIV_2													((uint8_t) 0x02)
    #define CLK1_CLK_DIV_4													((uint8_t) 0x04)
    #define CLK1_CLK_DIV_6													((uint8_t) 0x06)
    #define CLK1_CLK_DIV_8													((uint8_t) 0x08)
    #define CLK1_CLK_DIV_10													((uint8_t) 0x0A)
    #define CLK1_CLK_DIV_12													((uint8_t) 0x0C)
    #define CLK1_CLK_DIV_14													((uint8_t) 0x0E)



/* Register 0x0E (CLK2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |           ICLK_DIV[2:0]           |     0     |                    OSR[3:0]                   |
 * -------------------------------------------------------------------------------------------------
 */

    /* CLK2 register address */
    #define CLK2_ADDRESS													((uint8_t) 0x0E)

    /* CLK2 default (reset) value */
    #define CLK2_DEFAULT													((uint8_t) 0x86)

    /* CLK2 register field masks */
    #define CLK2_ICLK_DIV_MASK												((uint8_t) 0xE0)
    #define CLK2_OSR_MASK													((uint8_t) 0x0F)

    /* ICLK_DIV field values */
    #define CLK2_ICLK_DIV_2													((uint8_t) 0x20)
    #define CLK2_ICLK_DIV_4													((uint8_t) 0x40)
    #define CLK2_ICLK_DIV_6													((uint8_t) 0x60)
    #define CLK2_ICLK_DIV_8													((uint8_t) 0x80)
    #define CLK2_ICLK_DIV_10												((uint8_t) 0xA0)
    #define CLK2_ICLK_DIV_12												((uint8_t) 0xC0)
    #define CLK2_ICLK_DIV_14												((uint8_t) 0xE0)

    /* OSR field values */
    #define CLK2_OSR_4096													((uint8_t) 0x00)
    #define CLK2_OSR_2048													((uint8_t) 0x01)
    #define CLK2_OSR_1024													((uint8_t) 0x02)
    #define CLK2_OSR_800													((uint8_t) 0x03)
    #define CLK2_OSR_768													((uint8_t) 0x04)
    #define CLK2_OSR_512													((uint8_t) 0x05)
    #define CLK2_OSR_400													((uint8_t) 0x06)
    #define CLK2_OSR_384													((uint8_t) 0x07)
    #define CLK2_OSR_256													((uint8_t) 0x08)
    #define CLK2_OSR_200													((uint8_t) 0x09)
    #define CLK2_OSR_192													((uint8_t) 0x0A)
    #define CLK2_OSR_128													((uint8_t) 0x0B)
    #define CLK2_OSR_96														((uint8_t) 0x0C)
    #define CLK2_OSR_64														((uint8_t) 0x0D)
    #define CLK2_OSR_48														((uint8_t) 0x0E)
    #define CLK2_OSR_32														((uint8_t) 0x0F)



/* Register 0x0F (ADC_ENA) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |                    ENA[3:0]                   |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC_ENA register address */
    #define ADC_ENA_ADDRESS													((uint8_t) 0x0F)

    /* ADC_ENA default (reset) value */
    #define ADC_ENA_DEFAULT													((uint8_t) 0x00)

    /* ADC_ENA register field masks */
    #define ADC_ENA_ENA_MASK												((uint8_t) 0x0F)

    /* ENA field values */
    #define ADC_ENA_ENA_ALL_CH_PWDN											((uint8_t) 0x00)
    #define ADC_ENA_ENA_ALL_CH_PWUP											((uint8_t) 0x0F)



/* Register 0x10 (RESERVED3) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x11 (ADC1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN1[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC1 register address */
    #define ADC1_ADDRESS													((uint8_t) 0x11)

    /* ADC1 default (reset) value */
    #define ADC1_DEFAULT													((uint8_t) 0x00)

    /* ADC1 register field masks */
    #define ADC1_GAIN1_MASK													((uint8_t) 0x07)

    /* GAIN1 field values */
    #define ADC1_GAIN1_1													((uint8_t) 0x00)
    #define ADC1_GAIN1_2													((uint8_t) 0x01)
    #define ADC1_GAIN1_4													((uint8_t) 0x02)
    #define ADC1_GAIN1_8													((uint8_t) 0x03)
    #define ADC1_GAIN1_16													((uint8_t) 0x04)



/* Register 0x12 (ADC2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN2[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC2 register address */
    #define ADC2_ADDRESS													((uint8_t) 0x12)

    /* ADC2 default (reset) value */
    #define ADC2_DEFAULT													((uint8_t) 0x00)

    /* ADC2 register field masks */
    #define ADC2_GAIN2_MASK													((uint8_t) 0x07)

    /* GAIN2 field values */
    #define ADC2_GAIN2_1													((uint8_t) 0x00)
    #define ADC2_GAIN2_2													((uint8_t) 0x01)
    #define ADC2_GAIN2_4													((uint8_t) 0x02)
    #define ADC2_GAIN2_8													((uint8_t) 0x03)
    #define ADC2_GAIN2_16													((uint8_t) 0x04)



/* Register 0x13 (ADC3) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN3[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC3 register address */
    #define ADC3_ADDRESS													((uint8_t) 0x13)

    /* ADC3 default (reset) value */
    #define ADC3_DEFAULT													((uint8_t) 0x00)

    /* ADC3 register field masks */
    #define ADC3_GAIN3_MASK													((uint8_t) 0x07)

    /* GAIN3 field values */
    #define ADC3_GAIN3_1													((uint8_t) 0x00)
    #define ADC3_GAIN3_2													((uint8_t) 0x01)
    #define ADC3_GAIN3_4													((uint8_t) 0x02)
    #define ADC3_GAIN3_8													((uint8_t) 0x03)
    #define ADC3_GAIN3_16													((uint8_t) 0x04)



/* Register 0x14 (ADC4) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN4[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC4 register address */
    #define ADC4_ADDRESS													((uint8_t) 0x14)

    /* ADC4 default (reset) value */
    #define ADC4_DEFAULT													((uint8_t) 0x00)

    /* ADC4 register field masks */
    #define ADC4_GAIN4_MASK													((uint8_t) 0x07)

    /* GAIN4 field values */
    #define ADC4_GAIN4_1													((uint8_t) 0x00)
    #define ADC4_GAIN4_2													((uint8_t) 0x01)
    #define ADC4_GAIN4_4													((uint8_t) 0x02)
    #define ADC4_GAIN4_8													((uint8_t) 0x03)
    #define ADC4_GAIN4_16													((uint8_t) 0x04)

    

//****************************************************************************
//
// Register settings macros
//
//****************************************************************************

#define ADS_SPI_ASYNS_MODE                         1
#define ADS_SPI_SYNS_MODE                          0
#define ADS_SPI_SIZE_32                            1
#define ADS_SPI_SIZE_24                            0
#define ADS_ENABLE_HAMMING                         1
#define ADS_DISABLE_HAMMING                        0

#define OPCODE_NULL                             ((uint16_t) 0x0000)
#define OPCODE_RESET                            ((uint16_t) 0x0011)
#define OPCODE_STANDBY                          ((uint16_t) 0x0022)
#define OPCODE_WAKEUP                           ((uint16_t) 0x0033)
#define OPCODE_LOCK                             ((uint16_t) 0x0555)
#define OPCODE_UNLOCK                           ((uint16_t) 0x0655)
#define OPCODE_RREG                             ((uint16_t) 0x2000)
#define OPCODE_WREG                             ((uint16_t) 0x4000)
#define OPCODE_RREGS                            ((uint16_t) 0x2000)
#define OPCODE_WREGS                            ((uint16_t) 0x6000)

#if(0)
typedef struct _ads131a_state_t{
    uint8_t state1;
    uint8_t state_p;
    uint8_t state_n;
    uint8_t state_s;
    uint8_t error_count;
    uint8_t state_m2;
    uint8_t reserved_08h;
    uint8_t reserved_09h;
}ads131a_state_t;

typedef struct _ads131a_config_t{
    uint8_t reserved_0ah;
    uint8_t analog_config;
    uint8_t digital_config;
    uint8_t clk1;
    uint8_t clk2;
    uint8_t adc_en;
    uint8_t reserved_10h;
    uint8_t adc_gain[4];
}ads131a_config_t;

#endif

#define ADS131A_OSR    {4096, 2048, 1024, 800, 768, 512, 400, 384, 256, 200, 192, 128, 96, 64, 48, 32}

typedef volatile struct ads131a_dev_s{
    union{
        uint8_t val;
    }id_msb;
    union{
        uint8_t val;
    }id_lsb;
    union{
        struct{
            uint8_t f_check : 1;
            uint8_t f_drdy : 1;
            uint8_t f_resync : 1;
            uint8_t f_wdt : 1;
            uint8_t f_adcin : 1;
            uint8_t f_spi : 1;
            uint8_t f_opc : 1;
            uint8_t bit7 : 1;
        };
        uint8_t val;
    }stat_1;
    union{
        struct{
            uint8_t f_in1p : 1;
            uint8_t f_in2p : 1;
            uint8_t f_in3p : 1;
            uint8_t f_in4p : 1;
            uint8_t bit4_7 : 4;
        };
        uint8_t val;
    }stat_p;
    union{
        struct{
            uint8_t f_in1n : 1;
            uint8_t f_in2n : 1;
            uint8_t f_in3n : 1;
            uint8_t f_in4n : 1;
            uint8_t bit4_7 : 4;
        };
        uint8_t val;
    }stat_n;
    union{
        struct{
            uint8_t f_frame : 1;
            uint8_t f_cs : 1;
            uint8_t f_startup : 1;
            uint8_t bit3_7 : 5;
        };
        uint8_t val;
    }stat_s;
    union{
        uint8_t val;
    }error_cnt;
    union{
        struct{
            uint8_t m0pin: 2;
            uint8_t m1pin: 2;
            uint8_t m2pin: 2;
            uint8_t bit5_7: 2;
        };
        uint8_t val;
    }stat_m2;
    union{
        uint8_t val;
    }reserved_08h;
    union{
        uint8_t val;
    }reserved_09h;
    union{
        uint8_t val;
    }reserved_0ah;
    union{
        struct{
            uint8_t comp_th : 3;
            uint8_t int_refen : 1;
            uint8_t vref_4v : 1;
            uint8_t bit5 : 1;
            uint8_t hrm : 1;
            uint8_t vncpen : 1;
        };
        uint8_t val;
    }a_sys_cfg;
    union{
        struct{
            uint8_t crc_en : 1; /// 0:CRC disabled (default), 1:CRC enabled
            uint8_t fixed : 1;
            uint8_t hizdly : 2;
            uint8_t dndly : 2;
            uint8_t crc_mode : 1; /// 0:CRC is valid on only the device words being sent and received (default), 1:CRC is valid on all bits received and transmitted
            uint8_t wdt_en : 1;
        };
        uint8_t val;
    }d_sys_cfg;
    union{
        struct{
            uint8_t bit0 : 1;
            uint8_t clk_div : 3;
            uint8_t bit4 : 1;
            uint8_t bit5 : 1;
            uint8_t bit6 : 1;
            uint8_t clksrc : 1;
        };
        uint8_t val;
    }clk1;
    union{
        struct{
            uint8_t osr : 4;
            uint8_t bit4 : 1;
            uint8_t iclk_div : 3;
        };
        uint8_t val;
    }clk2;
    union{
        struct{
            uint8_t ena : 4;
            uint8_t bit4_7 : 4;
        };
        uint8_t val;
    }adc_ena;
    union{
        uint8_t val;
    }reserved_10h;
    union{
        struct{
            uint8_t gain : 3;
            uint8_t bit3_7 : 5;
        };
        uint8_t val;
    }adc[4];
}ads131a_dev_t;
//extern ads131a_dev_t ads131a;


typedef struct ads131a_udata_s{
    uint32_t channel_data[4];
    uint16_t rawcrc;
    uint16_t calcrc;
}ads131a_udata_t;

typedef struct ads131a_idata_s{
    int32_t channel_data[4];
}ads131a_idata_t;

typedef struct ads131a_fdata_s{
    float channel_data[4];
}ads131a_fdata_t;

typedef union{
    struct{
        uint8_t reset : 1;
        uint8_t standby : 1;
        uint8_t lock : 1;
        uint8_t reserved : 3;
    };
    uint8_t val;
}ads131a_state_t;

typedef struct _ads131a_device_t{
//    uint16_t id;
//    ads131a_state_t state;
//    ads131a_config_t config;
    ads131a_dev_t dev;
    ads131a_state_t state;
//    uint16_t opc;              // cmd
//    uint8_t word_length;       // word length, unit: byte
//    ads131a_udata_t channel_data;
    uint32_t vref;       // Unit: mV

}ads131a_device_t;

typedef ads131a_device_t* ads131a_device_handle_t;


/**
 * @brief 默认配置
 */
#if(0)
#define ADS131A_CONFIGURE_DEFAULT()     {.analog_config = 0x60, .digital_config = 0x3C, .clk1 = 0x08, .clk2 = 0x86, .adc_en = 0x00, .adc_gain = {0x00, 0x00, 0x00, 0x00}}
#endif

#if(0)
/**
 * @brief M0, M1, M2 全都浮空
 */
#define ADS131A_DEFAULT_DEVICE()                  {.id = 0, \
                                                    .state = {0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00} \
                                                    .config = {0x00, 0x60, 0x3C, 0x08, 0x86, 0x00, 0x00, {0x00, 0x00, 0x00, 0x00}}, \
                                                    .word_length = 3, \
                                                    .verf = 2500 }


#define ADS131A_M0_M1_M2_DEVICE(m0, m1, m2)     {.id = 0, \
                                                    .state = {0x00, 0x00, 0x00, 0x00, 0x00, ((m2 << 4) | (m1 << 2) | m0), 0x00, 0x00}, \
                                                    .config = {0x00, 0x60, 0x3C, 0x08, 0x86, 0x00, 0x00, {0x00, 0x00, 0x00, 0x00}}, \
                                                    .vref = 2500 }

#else

#define ADS131A_M0_M1_M2_DEVICE(m0, m1, m2)        { \
        .id_msb.val = 0x00, \
        .id_lsb.val = 0x00, \
        .stat_1.val = 0x00, \
        .stat_p.val = 0x00, \
        .stat_n.val = 0x00, \
        .stat_s.val = 0x00, \
        .error_cnt.val = 0x00, \
        .stat_m2.val = ((m0) | ((m1) << 2) | ((m2) << 4)), \
        .reserved_08h.val = 0x00, \
        .reserved_09h.val = 0x00, \
        .reserved_0ah.val = 0x00, \
        .a_sys_cfg.val = 0x60, \
        .d_sys_cfg.val = 0x3C, \
        .clk1.val = 0x08, \
        .clk2.val = 0x86, \
        .adc_ena.val = 0x00, \
        .reserved_10h.val = 0x00, \
        .adc[0].val = 0x00, \
        .adc[1].val = 0x00, \
        .adc[2].val = 0x00, \
        .adc[3].val = 0x00, \
    }


#endif

const static uint8_t WORD_BITS[4] = {24, 32, 16, 0};
#if(0)
#define ADS131A_WORD_BITS(device)        WORD_BITS[(((device)->state.state_m2) & 0x0C) >> 2]

#define ADS131A_GET_CHANNEL_NUMBERA(device) (((device)->id >> 8) & 0xFF)

#define ADS131A_GET_ERROR_COUNT(device) ((device)->state.error_count)

#define ADS131A_ENABLE_CHANNEL(device) do{ (device)->config.adc_en |= ADC_ENA_ENA_ALL_CH_PWUP; }while(0)
#define ADS131A_DISABLE_CHANNEL(device) do{ (device)->config.adc_en = 0; }while(0)
#define ADS131A_IS_CHANNEL_ENABLE(device) (((device)->config.adc_en & ADC_ENA_ENA_ALL_CH_PWUP) ? 1 : 0)

#define ADS131A_SET_CHANNEL_GAIN(device, channel, gain) do{ (device)->config.adc_gain[channel] = gain; }while(0)
#define ADS131A_GET_CHANNEL_GAIN(device, channel) ((device)->config.adc_gain[channel])

#define ADS131A_ENABLE_NEGATIVE_PUMP(device) do{ (device)->config.analog_config |= A_SYS_CFG_VNCPEN_MASK; }while(0)
#define ADS131A_DISABLE_NEGATIVE_PUMP(device) do{ (device)->config.analog_config &= (~A_SYS_CFG_VNCPEN_MASK); }while(0)

#define ADS131A_ENABLE_HRM(device) do{ (device)->config.analog_config |= A_SYS_CFG_HRM_MASK; }while(0)
#define ADS131A_DISABLE_HRM(device) do{ (device)->config.analog_config &= (~A_SYS_CFG_HRM_MASK); }while(0)

#define ADS131A_ENABLE_VREF_4V(device) do{ (device)->config.analog_config |= A_SYS_CFG_VREF_4V_MASK; }while(0)
#define ADS131A_DISABLE_VREF_4V(device) do{ (device)->config.analog_config &= (~A_SYS_CFG_VREF_4V_MASK); }while(0)

#define ADS131A_ENABLE_INT_REF(device) do{ (device)->config.analog_config |= A_SYS_CFG_INT_REFEN_MASK; }while(0)
#define ADS131A_DISABLE_INT_REF(device) do{ (device)->config.analog_config &= (~A_SYS_CFG_INT_REFEN_MASK); }while(0)

#define ADS131A_SET_COMP_THRESHOLD(device, th) do{ (device)->config.analog_config &= ~(A_SYS_CFG_COMP_TH_MASK); (device)->config.analog_config |= (th); }while(0)

#define ADS131A_ENABLE_WDT(device) do{ (device)->config.digital_config |= D_SYS_CFG_WDT_EN_MASK; }while(0)
#define ADS131A_DISABLE_WDT(device) do{ (device)->config.digital_config &= (~D_SYS_CFG_WDT_EN_MASK); }while(0)

#define ADS131A_SET_CRC_MODE(device, mode) do{(device)->config.digital_config &= ~(D_SYS_CFG_CRC_MODE_MASK); (device)->config.digital_config |= (mode << 6); }while(0)
#define ADS131A_GET_CRC_MODE(device) ((((device))->config.digital_config & D_SYS_CFG_CRC_MODE_MASK) ? 1 : 0)

#define ADS131A_SET_DONE_DELAY(device, delay) do{(device)->config.digital_config &= ~(D_SYS_CFG_DNDLY_MASK); (device)->config.digital_config |= (delay); }while(0)

#define ADS131A_SET_HIZ_DELAY(device, delay) do{(device)->config.digital_config &= ~(D_SYS_CFG_HIZDLY_MASK); (device)->config.digital_config |= (delay); }while(0)

#define ADS131A_ENABLE_FIXED_WORD_SIZE(device) do{ (device)->config.digital_config |= D_SYS_CFG_FIXED_MASK; }while(0)
#define ADS131A_DISABLE_FIXED_WORD_SIZE(device) do{ (device)->config.digital_config &= (~D_SYS_CFG_FIXED_MASK); }while(0)
#define ADS131A_IS_FIXED_WORD_SIZE(device) ((((device))->config.digital_config & D_SYS_CFG_FIXED_MASK) ? 1 : 0)

#define ADS131A_ENABLE_CRC(device) do{ (device)->config.digital_config |= D_SYS_CFG_CRC_EN_MASK; }while(0)
#define ADS131A_DISABLE_CRC(device) do{ (device)->config.digital_config &= (~D_SYS_CFG_CRC_EN_MASK); }while(0)
#define ADS131A_IS_CRC_ENABLE(device) ((((device))->config.digital_config & D_SYS_CFG_CRC_EN_MASK) ? 1 : 0)

#define ADS131A_SET_ADC_CLK_DIV(device, div) do{ (device)->config.clk1 &= (~(CLK1_CLK_DIV_MASK)); (device)->config.clk1 |= (div); }while(0)

#define ADS131A_SET_MODULATOR_CLK_DIV(device, div) do{ (device)->config.clk2 &= (~(CLK2_ICLK_DIV_MASK)); (device)->config.clk2 |= (div); }while(0)

#define ADS131A_SET_OSR(device, osr) do{ (device)->config.clk2 &= (~(CLK2_OSR_MASK)); (device)->config.clk2 |= (osr); }while(0)

#else

#define CLK_DIV(div) (div >> 1)

#define LL_ADS_MODIFY_U8_REG_FIELD(base_reg, reg_field, field_val)    \
{                                                           \
    uint8_t temp_val = base_reg.val;                       \
    typeof(base_reg) temp_reg;                              \
    temp_reg.val = temp_val;                                \
    temp_reg.reg_field = (field_val);                       \
    (base_reg).val = temp_reg.val;                          \
}

#define LL_ADS_READ_U8_REG_FIELD(base_reg, reg_field) ({    \
    uint8_t temp_val = base_reg.val;                       \
    typeof(base_reg) temp_reg;                              \
    temp_reg.val = temp_val;                                \
    temp_reg.reg_field;                                     \
})

#define LL_ADS_WRITE_U8_REG(base_reg, reg_val)   {base_reg.val = reg_val;}

static inline uint8_t ll_ads_get_device_channel(const ads131a_device_t *device){
    return device->dev.id_msb.val;
}

static inline uint8_t ll_ads_get_word_bits(const ads131a_device_t *device){
    return WORD_BITS[device->dev.stat_m2.m1pin];
}

static inline uint8_t ll_ads_is_hamming_code_enable(const ads131a_device_t *device){
    return device->dev.stat_m2.m2pin;
}

static inline bool ll_ads_is_crc_en(const ads131a_device_t *device){
    return (device->dev.d_sys_cfg.crc_en);
}

static inline bool ll_ads_get_crc_mode(const ads131a_device_t *device){
    return device->dev.d_sys_cfg.crc_mode;
}

static inline bool ll_ads_is_fixed_mode(const ads131a_device_t *device){
    return device->dev.d_sys_cfg.fixed;
}

static inline bool ll_ads_is_enable_adc(const ads131a_device_t *device){
    return (device->dev.adc_ena.ena ? 1 : 0);
}

static inline uint8_t ll_ads_get_channel_gain(const ads131a_device_t *device, uint8_t channel){
    return (1 << (device->dev.adc[channel].gain));
}

#endif

#ifndef UPPER_BYTE
#define UPPER_BYTE(_uint16)        ((uint8_t)(((_uint16) & 0xFF00) >> 8))
#endif

#ifndef LOWER_BYTE
#define LOWER_BYTE(_uint16)       ((uint8_t)((_uint16) & 0x00FF))
#endif

#ifndef COMBINE_BYTES
#define COMBINE_BYTES(hb, lb)     (((uint16_t)(hb) << 8) | ((uint16_t)(lb) & 0x00FF))
#endif

static __inline int32_t complement_to_decimal(uint32_t com, uint8_t bits){
//	if(com & (0x01 << (n - 1))){    // 负数
//		return (signed int)(com | (~((1 << n) - 1)));
//	}else{
//		return (int32_t)(com);
//	}
	return ((com & (0x00000001 << (bits - 1))) ? (int32_t)(com | (~((1 << bits) - 1))) : (int32_t)(com));
}

static __inline uint32_t decimal_to_complement(int32_t dec, uint8_t bits){
    return (((dec) & (0x00000001 << (bits - 1))) ? (uint32_t)((((0x01 << bits) - 1) & (dec)) | (0x01 << (bits - 1))) : (uint32_t)(dec));
}

#define COM_TO_DEC(complements, n)  ((complements & (0x00000001 << (n - 1))) ? (int32_t)(complements | (~((1 << n) - 1))) : (complements))

#if(0)
void ads131a_assert_printf_state(const ads131a_state_t state);
void ads131a_assert_printf_configure(const ads131a_config_t config);
#endif

uint8_t ads131a_read_register(ads131a_device_t* device, uint8_t reg);
uint8_t ads131a_write_register(const ads131a_device_t* device, uint8_t reg, uint8_t data);

uint16_t ads131a_send_command(const ads131a_device_t *device, uint16_t opc);

static inline uint16_t ads131a_get_command_response(const ads131a_device_t *device){
    return ads131a_send_command(device, OPCODE_NULL);
}

/**
 * @description: 上电后启动 ADS131A, 获取信息
 * @param {ads131a_device_t} *device
 * @return {*}
 */
uint8_t ads131a_startup(ads131a_device_t *device);

uint8_t ads131a_adcen(ads131a_device_t *device);

uint8_t ads131a_adcoff(ads131a_device_t *device);

static inline uint16_t ads131a_opc_reset(ads131a_device_t *device){
    uint16_t response = 0;
    device->state.reset = 1;
    ads131a_send_command(device, OPCODE_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    response = ads131a_get_command_response(device);
    if(response == (0xFF00 + device->dev.id_msb.val)) device->state.val = 0x04;
    return response;
}

static inline uint16_t ads131a_opc_wakeup(ads131a_device_t *device){
    uint16_t response = 0;
    ads131a_send_command(device, OPCODE_WAKEUP);
    response = ads131a_get_command_response(device);
    if(response == OPCODE_WAKEUP) device->state.standby = 0;
    return response;
}

static inline uint16_t ads131a_opc_standby(ads131a_device_t *device){
    uint16_t response = 0;
    ads131a_send_command(device, OPCODE_STANDBY);
    response = ads131a_get_command_response(device);
    if(response == OPCODE_STANDBY){
        device->state.standby = 1;
    }
    return response;
}

static inline uint16_t ads131a_opc_lock(ads131a_device_t *device){
    uint16_t response = 0;
    ads131a_send_command(device, OPCODE_LOCK);
    response = ads131a_get_command_response(device);
    if(response == OPCODE_LOCK) device->state.lock = 1;
    return response;
}

static inline uint16_t ads131a_opc_unlock(ads131a_device_t *device){
    uint16_t response = 0;
    ads131a_send_command(device, OPCODE_UNLOCK);
    response = ads131a_get_command_response(device);
    if(response == OPCODE_UNLOCK){
        device->state.lock = 0;
    }
    return response;
}


/**
 *
 * @param device
 * @param buffer
 * @return state_1 register's value
 */
uint16_t ads131a_get_channel_udata(ads131a_device_t *device, ads131a_udata_t *ptr);

uint16_t ads131a_get_channel_idata(ads131a_device_t *device, ads131a_idata_t *idata);

void ads131a_get_channel_voltage(const ads131a_device_t *device, const ads131a_udata_t *udata, ads131a_fdata_t *fdata);




uint16_t ads131a_get_device_id(ads131a_device_t *device);

static inline uint8_t ads131a_get_device_channel(ads131a_device_t *device){
    uint8_t reg_val = 0;
    reg_val = ads131a_read_register(device, ID_MSB_ADDRESS);
    return reg_val;
}

uint16_t ads131a_get_device_status(ads131a_device_t *device);

uint16_t ads131a_configure_device(const ads131a_device_t *device);
uint16_t ads131a_get_device_configure(ads131a_device_t *device);

#ifdef __cplusplus
}
#endif

#endif