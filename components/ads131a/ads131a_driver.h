//
// Created by XIAO on 2022/3/30.
//

#ifndef ORSYSTEM_ADS131A_DRIVER_H
#define ORSYSTEM_ADS131A_DRIVER_H

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "stdbool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


typedef struct _ads131a_dataframe_t{
    uint8_t *tx_ptr;
    uint8_t *rx_ptr;
    uint8_t dataframe_size;   // unit: byte
    bool fixed;
    uint8_t word_bits;
    uint16_t response;
}ads131a_dataframe_t;

typedef spi_device_handle_t ads_spi_handle_t;

esp_err_t ads131a_drv_init(const spi_device_handle_t spi, int cs);

esp_err_t ads131a_drv_dataframe(ads131a_dataframe_t *dataframe);

// void io_ads131a_reset(uint8_t level);
// void io_ads131a_cs(uint8_t level);

#ifdef USE_DRV_OPC
uint16_t ads131a_drv_null(void);
uint16_t ads131a_drv_unlock(void);
uint16_t ads131a_drv_rreg(uint8_t reg);
uint16_t ads131a_drv_wreg(uint8_t reg, uint8_t data);
#endif
#endif //ORSYSTEM_ADS131A_DRIVER_H
