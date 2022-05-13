/*
 * @Author: your name
 * @Date: 2022-01-24 08:40:10
 * @LastEditTime: 2022-05-05 09:31:41
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \orsystem\main\hardware_access.h
 */

#ifndef _HARDWAER_ACCESS_H__
#define _HARDWAER_ACCESS_H__

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/timer.h"


#include "../components/ll_mcpwm/ll_mcpwm.h"

#include "orsystem_gpio.h"

esp_err_t esp32_uart_init(const int uart_index, const uart_config_t* config);
esp_err_t esp32_uart_deinit(const int uart_index);

#if(CONFIG_ORSYSTEM_MODE)
esp_err_t esp32_i2c_init(i2c_port_t port, uint32_t clk);
esp_err_t esp32_i2c_deinit(i2c_port_t port);

int esp32_take_i2c_access(i2c_port_t port);
int esp32_release_i2c_access(i2c_port_t port);


esp_err_t esp32_vspi_init(spi_device_handle_t *vspi);
esp_err_t vspi_wr_buffer(uint8_t *tx_buffer, uint8_t *rx_buffer, uint32_t length);

extern spi_device_handle_t vspi;

#endif

#if(CONFIG_ORSYSTEM_MODE)
static inline void io_ads131a_reset(uint8_t level){
    gpio_set_level(IO_ADS_RESET, level);
}

static inline void io_set_analog_power(uint8_t enable){
    gpio_set_level(IO_EN_AP, enable);
}

static inline void io_set_buzzer(uint8_t enable){
    gpio_set_level(IO_EN_BZ, enable);
}

static inline void io_set_analog_channel(uint8_t enable){
    gpio_set_level(IO_EN_CH, enable);
}

static inline void io_ads131a_set_drdy_intr(uint8_t enable){
    if(enable) gpio_intr_enable(IO_ADS_ADRDY);
    else gpio_intr_disable(IO_ADS_ADRDY);
}

#else
static inline void io_mcpwm_cs(uint8_t level){
    gpio_set_level(IO_PWM_CS, level);
}


#endif



void gpio_init(void);

esp_err_t esp32_can_install(twai_general_config_t gconf, twai_timing_config_t tconf, twai_filter_config_t fconf);
esp_err_t esp32_can_uninstall(void);

esp_err_t esp32_can_self_test(void);

esp_err_t nvs_init(void);
esp_err_t nvs_save_blob_data(const char *name, const char *key, const void* data, size_t length);
esp_err_t nvs_read_blob_data(const char *name, const char *key, void* data, size_t length);

static inline esp_err_t nvs_save_ads_device(const void* device, const uint8_t length){
    return nvs_save_blob_data("ads_device", "device", device, length);
}
static inline esp_err_t nvs_read_ads_device(void* device, const uint8_t length){
    return nvs_read_blob_data("ads_device", "device", device, length);
}
static inline esp_err_t nvs_save_orsystem_config(const void* config, uint8_t length){
    return nvs_save_blob_data("orsystem", "config", config, length);
}
static inline esp_err_t nvs_read_orsystem_config(void* config, uint8_t length){
    return nvs_read_blob_data("orsystem", "config", config, length);
}

#ifndef CONFIG_SPIFFS_STORAGE_BASE_PATH
#define CONFIG_SPIFFS_STORAGE_BASE_PATH    "/spiffs"
#endif
#ifndef CONFIG_SPIFFS_STORATE_LABEL
#define CONFIG_SPIFFS_STORATE_LABEL        "storage"
#endif

esp_err_t spiffs_initializer(const char* base_path, const char *label);

esp_err_t esp32_mcpwm_init(mcpwm_dev_t *mcpwm, mcpwm_operator_t op, const ll_mcpwm_config_t* config);
esp_err_t esp32_mcpwm_start(mcpwm_dev_t *mcpwm, mcpwm_operator_t op);
esp_err_t esp32_mcpwm_stop(mcpwm_dev_t *mcpwm, mcpwm_operator_t op);
esp_err_t esp32_mcpwm_set_duty(mcpwm_dev_t *mcpwm, mcpwm_operator_t op, const float duty);
esp_err_t esp32_mcpwm_set_frequency(mcpwm_dev_t *mcpwm, mcpwm_operator_t op, const uint32_t frequency);


#endif