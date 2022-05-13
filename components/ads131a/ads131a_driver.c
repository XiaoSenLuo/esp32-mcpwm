//
// Created by XIAO on 2022/3/30.
//

#include "ads131a_driver.h"

static const char *TAG = "_ADS131A_DRIVER_C";

static spi_device_handle_t ads_vspi = NULL;

static int io_cs = -1;


static inline void io_ads131a_cs(uint8_t level){
    gpio_set_level(io_cs, level);
}



esp_err_t ads131a_drv_init(const spi_device_handle_t spi, int cs){
    esp_err_t err = ESP_FAIL;
/**
    spi_bus_config_t buscfg = {
            .miso_io_num = IO_VSPI_MISO,
            .mosi_io_num = IO_VSPI_MOSI,
            .sclk_io_num = IO_VSPI_CLK,
            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,  // 主机模式, IO 功能复用
//            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 32 * 6
    };

    spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 1000000, // MHz
            .mode = 1, // (CPOL=0, CPHA=1)
            .spics_io_num = -1,
            .queue_size = 3,
            .flags = 0,
            .cs_ena_posttrans = 1,
//            .pre_cb = NULL
    };

    err = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(err);

    err = spi_bus_add_device(VSPI_HOST, &devcfg, &ads_vspi);
    ESP_ERROR_CHECK(err);
**/
    io_cs = cs;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pin_bit_mask = (1ULL << io_cs);
    gpio_config(&io_conf);
    io_ads131a_cs(1);

    ads_vspi = spi;
    ESP_LOGI(TAG, "ADS131a SPI device added, SPI handle address: %#X\r\n", (uint32_t)ads_vspi);

    return err;
}

esp_err_t ads131a_drv_dataframe(ads131a_dataframe_t *dataframe){
    esp_err_t err = ESP_FAIL;
    spi_transaction_t t = {
            .flags = 0,
            .cmd = 0,
            .addr = 0,
            .user = 0,
            .tx_buffer = dataframe->tx_ptr,
            .rx_buffer = dataframe->rx_ptr,
            .length = dataframe->dataframe_size << 3,
            .rxlength = 0
    };

//    ESP_LOGD(TAG, "tx_buffer address: 0x%X, rx_buffer address: 0x%X\r\n", (uint32_t)(uint8_t*)t.tx_buffer, (uint32_t)(uint8_t*)t.rx_buffer);

//    err = spi_device_acquire_bus(ads_vspi, portMAX_DELAY);
//    ESP_ERROR_CHECK(err);
    io_ads131a_cs(0);
    err = spi_device_polling_transmit(ads_vspi, &t);
    io_ads131a_cs(1);
//    ESP_ERROR_CHECK(err);
//    spi_device_release_bus(ads_vspi);
    if(err == ESP_OK){
        dataframe->response = ((uint16_t)(((uint8_t*)t.rx_buffer)[0]) << 8) | (((uint8_t*)t.rx_buffer)[1]);
    }
    return err;
}

#ifdef USE_DRV_OPC
uint16_t ads131a_drv_null(void){
    esp_err_t err = ESP_FAIL;
    uint16_t response = 0;
    spi_transaction_t t = {
            .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
            .cmd = 0,
            .addr = 0,
            .user = 0,
            .tx_data = {0x00, 0x00, 0x00, 0x00},
            .rx_data = {0x00, 0x00, 0x00, 0x00},
            .length = 4 << 3,
            .rxlength = 0
    };
    io_ads131a_cs(0);
    err = spi_device_polling_transmit(ads_vspi, &t);
    io_ads131a_cs(1);
    ESP_ERROR_CHECK(err);
//    spi_device_release_bus(ads_vspi);
    if(err == ESP_OK){
        response = (uint16_t)(t.rx_data[0] << 8) | (t.rx_data[1]);
        ESP_LOGD(TAG, "null response:0x%X\r\n", response);
    }
    return response;
}

uint16_t ads131a_drv_unlock(void){
    esp_err_t err = ESP_FAIL;
    uint16_t response = 0;
    spi_transaction_t t = {
            .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
            .cmd = 0,
            .addr = 0,
            .user = 0,
            .tx_data = {0x06, 0x55, 0x00, 0x00},
            .rx_data = {0x00, 0x00, 0x00, 0x00},
            .length = 4 << 3,
            .rxlength = 0
    };
    io_ads131a_cs(0);
    err = spi_device_polling_transmit(ads_vspi, &t);
    io_ads131a_cs(1);
    ESP_ERROR_CHECK(err);
//    spi_device_release_bus(ads_vspi);
    if(err == ESP_OK){
        response = (uint16_t)(t.rx_data[0] << 8) | (t.rx_data[1]);
        ESP_LOGD(TAG, "unlock response:0x%X\r\n", response);
    }
//    response = ads131a_drv_null();
    return response;
}


uint16_t ads131a_drv_rreg(uint8_t reg){
    esp_err_t err = ESP_FAIL;
    uint16_t response = 0;
    spi_transaction_t t = {
            .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
            .cmd = 0,
            .addr = 0,
            .user = 0,
            .tx_data = {(0x20 + reg), 0x00, 0x00, 0x00},
            .rx_data = {0x00, 0x00, 0x00, 0x00},
            .length = 4 << 3,
            .rxlength = 0
    };
    io_ads131a_cs(0);
    err = spi_device_polling_transmit(ads_vspi, &t);
    io_ads131a_cs(1);
    ESP_ERROR_CHECK(err);
//    spi_device_release_bus(ads_vspi);
    if(err == ESP_OK){
        response = (uint16_t)(t.rx_data[0] << 8) | (t.rx_data[1]);
        ESP_LOGD(TAG, "rreg[%xh] response:0x%X\r\n", 0x20 + reg, response);
    }
    response = ads131a_drv_null();
    return response;
}

uint16_t ads131a_drv_wreg(uint8_t reg, uint8_t data){
    esp_err_t err = ESP_FAIL;
    uint16_t response = 0;
    spi_transaction_t t = {
            .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
            .cmd = 0,
            .addr = 0,
            .user = 0,
            .tx_data = {(0x40 + reg), data, 0x00, 0x00},
            .rx_data = {0x00, 0x00, 0x00, 0x00},
            .length = 4 << 3,
            .rxlength = 0
    };
    io_ads131a_cs(0);
    err = spi_device_polling_transmit(ads_vspi, &t);
    io_ads131a_cs(1);
    ESP_ERROR_CHECK(err);
//    spi_device_release_bus(ads_vspi);
    if(err == ESP_OK){
        response = (uint16_t)(t.rx_data[0] << 8) | (t.rx_data[1]);
        ESP_LOGD(TAG, "wreg[%xh] response:0x%X\r\n", 0x40 + reg, response);
    }
    return response;
}

#endif