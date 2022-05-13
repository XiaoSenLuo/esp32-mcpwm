


#include "hardware_access.h"

#include "nvs_flash.h"

static const char TAG[] = "HARDWARE_ACCESS";

#if(CONFIG_ORSYSTEM_MODE)
static SemaphoreHandle_t i2c_mutex[I2C_NUM_MAX] = {NULL};
#endif


esp_err_t esp32_uart_init(const int uart_index, const uart_config_t* config){
    esp_err_t err;

    err = uart_driver_install(uart_index, 512, 512, 4, NULL, 0);
    ESP_ERROR_CHECK(err);
    err = uart_param_config(uart_index, config);
    ESP_ERROR_CHECK(err);
    err = uart_set_pin(uart_index, IO_UART0_TX, IO_UART0_RX, -1, -1);
    ESP_ERROR_CHECK(err);

    return err;
}


esp_err_t esp32_uart_deinit(const int uart_index){
    esp_err_t err;
    err = uart_driver_delete(uart_index);
    return err;
}

#if(CONFIG_ORSYSTEM_MODE)
esp_err_t esp32_i2c_init(i2c_port_t port, uint32_t clk){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IO_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = IO_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = clk,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    
    if(i2c_mutex[port] != NULL) return ESP_OK;

    esp_err_t err = i2c_param_config(port, &conf);
    if(err != ESP_OK){
        return err;
    }
    err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if(err != ESP_OK){
        return err;
    }
    if(i2c_mutex[port] == NULL) i2c_mutex[port] = xSemaphoreCreateMutex();

    return err;
}


esp_err_t esp32_i2c_deinit(i2c_port_t port){

    if(i2c_mutex[port]) vSemaphoreDelete(i2c_mutex[port]);
    return i2c_driver_delete(port);
}


int esp32_take_i2c_access(i2c_port_t port){
    
    if(i2c_mutex[port]) return xSemaphoreTake(i2c_mutex[port], portMAX_DELAY);
    
    return pdFAIL;
}

int esp32_release_i2c_access(i2c_port_t port){
    
    if(i2c_mutex[port]) return xSemaphoreGive(i2c_mutex[port]);

    return pdFAIL;
}

spi_device_handle_t vspi = NULL;

esp_err_t esp32_vspi_init(spi_device_handle_t* spi){
    esp_err_t ret;

    spi_bus_config_t buscfg = {
            .miso_io_num = IO_VSPI_MISO,
            .mosi_io_num = IO_VSPI_MOSI,
            .sclk_io_num = IO_VSPI_CLK,
            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,  // 主机模式, IO 功能复用
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 32 * 6
    };

    spi_device_interface_config_t devcfg = {
            .clock_speed_hz = SPI_MASTER_FREQ_20M, // MHz
            .mode = 1, // (CPOL=0, CPHA=1)
            .spics_io_num = -1,
            .queue_size = 3,
//            .pre_cb = NULL
    };

    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus

    ret = spi_bus_add_device(VSPI_HOST, &devcfg, spi);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "VSPI device added, SPI handle address: %#X\r\n", (uint32_t)*spi);
    vspi = *spi;
    return ret;
}


esp_err_t vspi_wr_buffer(uint8_t *tx_buffer, uint8_t *rx_buffer, uint32_t length){
    esp_err_t err = ESP_FAIL;
    spi_transaction_t tr;
    memset(&tr, 0, sizeof(spi_transaction_t));
    tr.length = (length << 3);
    tr.tx_buffer = (const uint8_t *)tx_buffer;
//    tr.rx_buffer = (const uint8_t *)rx_buffer;
    tr.rxlength = tr.length;
    err = spi_device_polling_transmit(vspi, &tr);
    ESP_ERROR_CHECK(err);
    return err;
}

#endif


void gpio_init(void){
    gpio_config_t io_conf = {0};
    // TODO @ 输出管脚初始化
#if(CONFIG_ORSYSTEM_MODE)

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << IO_EN_BZ);
    gpio_config(&io_conf);

    // TODO @ 输出管脚初始化
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << IO_EN_AP) | (1ULL << IO_EN_CH) | (1ULL << IO_ADS_RESET);
    gpio_config(&io_conf);

    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << IO_VSPI_CS);
    gpio_config(&io_conf);
#else
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pin_bit_mask = (1ULL << IO_PWM_CS);
    gpio_config(&io_conf);
    io_mcpwm_cs(1);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << IO_S1) | (1ULL << IO_S2) | (1ULL << IO_K1) | (1ULL << IO_K2) | (1ULL << IO_K3) | (1ULL << IO_K4);
    gpio_config(&io_conf);
#endif
}


esp_err_t esp32_can_install(twai_general_config_t gconf, twai_timing_config_t tconf, twai_filter_config_t fconf){
    esp_err_t err = ESP_FAIL;

    twai_general_config_t twai_g_cfg = gconf;
    twai_filter_config_t  twai_f_cfg = fconf;
    twai_timing_config_t twai_t_cfg = tconf;

    err = twai_driver_install(&twai_g_cfg, &twai_t_cfg, &twai_f_cfg);
    // ESP_ERROR_CHECK(err);
    if(err == ESP_OK) ESP_LOGI(TAG, "CAN driver installed!\r\n");
    else return err;
    err = twai_start();
    // ESP_ERROR_CHECK(err);
    if(err == ESP_OK){
        ESP_LOGI(TAG, "CAN driver started!\r\n");
    }else{
        ESP_LOGE(TAG, "CAN driver start failed!\r\n");
    }
    return err;
}

esp_err_t esp32_can_uninstall(void){
    esp_err_t err = ESP_FAIL;

    twai_stop();
    // if(err == ESP_OK){
        ESP_LOGI(TAG, "CAN driver stop!\r\n");
        err = twai_driver_uninstall();
        // ESP_ERROR_CHECK(err);
        // if(err == ESP_OK){
            ESP_LOGI(TAG, "CAN uninstalled!\r\n");
        // }
    // }
    return err;
}


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define MSG_ID          0x233
#define TRY_TIMES       100

static SemaphoreHandle_t tx_sem = NULL;
static SemaphoreHandle_t rx_sem = NULL;
static SemaphoreHandle_t done_sem = NULL;

static void _twai_transmit_task(void *arg)
{
    twai_message_t tx_msg = {.data_length_code = 1, .data = {'T'}, .identifier = MSG_ID, .self = 1};
    for(int i = 0; i < TRY_TIMES; i++){
        xSemaphoreTake(tx_sem, portMAX_DELAY);
        twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
        xSemaphoreGive(done_sem);
    }
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

static void _twai_receive_task(void *arg)
{
    twai_message_t rx_message;
    for (int i = 0; i < TRY_TIMES; i++) {
        xSemaphoreTake(rx_sem, portMAX_DELAY);
        twai_receive(&rx_message, pdMS_TO_TICKS(1000));
        if((rx_message.identifier != MSG_ID) || rx_message.data[0] != 'T'){

        }else{

        }
        xSemaphoreGive(done_sem);
    }
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

esp_err_t esp32_can_self_test(void){
    twai_status_info_t info;
    uint32_t alert = 0;
    esp_err_t err = ESP_FAIL;

    err = twai_get_status_info(&info);
    if(info.state == TWAI_STATE_RECOVERING){
        while(!(alert & TWAI_ALERT_BUS_RECOVERED)){
            twai_read_alerts(&alert, pdMS_TO_TICKS(1000));
        }
    }
    if(info.state == TWAI_STATE_RUNNING){
        twai_clear_transmit_queue();
        twai_stop();
        twai_clear_receive_queue();
    }
    
    twai_driver_uninstall();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(IO_CAN_TX, IO_CAN_RX, TWAI_MODE_NO_ACK);
    g_config.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_50KBITS();
    twai_filter_config_t f_config = { .acceptance_code = (MSG_ID << 21), .acceptance_mask = ~(MSG_ID << 21), .single_filter = true};
    err = twai_driver_install(&g_config, &t_config, &f_config);
    ESP_ERROR_CHECK(err);

    tx_sem = xSemaphoreCreateBinary();
    rx_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(_twai_receive_task, "TWAI_rx", 4096, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(_twai_transmit_task, "TWAI_tx", 4096, NULL, 8, NULL, 0);

    uint8_t try_times = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));
    while((try_times++) < TRY_TIMES) {
        ESP_ERROR_CHECK(twai_start());

        xSemaphoreGive(rx_sem);
        xSemaphoreGive(tx_sem);
        xSemaphoreTake(done_sem, portMAX_DELAY);  // wait for tx done
        xSemaphoreTake(done_sem, portMAX_DELAY);  // wait for rx done

        ESP_ERROR_CHECK( twai_stop() );
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    xSemaphoreTake(done_sem, portMAX_DELAY);  // wait for tx done
    xSemaphoreTake(done_sem, portMAX_DELAY);  // wait for rx done

    vSemaphoreDelete(tx_sem);
    vSemaphoreDelete(rx_sem);
    vSemaphoreDelete(done_sem);

    twai_read_alerts(&alert, 0);
    if((alert & TWAI_ALERT_BUS_ERROR) || (alert & TWAI_ALERT_ERR_PASS) || (alert & TWAI_ALERT_ABOVE_ERR_WARN)){
        twai_status_info_t info;
        twai_get_status_info(&info);
        if(info.tx_error_counter){
            ESP_LOGE(TAG, "can self test transmit error counter:%d\r\n", info.tx_error_counter);
        }
        if(info.rx_error_counter){
            ESP_LOGE(TAG, "can self test receive error counter:%d\r\n", info.rx_error_counter);
        }
        // err = ESP_FAIL;
    }else{
        ESP_LOGI(TAG, "can bus self test pass!\r\n");
    }
    twai_driver_uninstall();
    return err;
}
#undef MSG_ID
#undef TRY_TIMES

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"

esp_err_t nvs_init(void){
    esp_err_t err;
    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    return err;
}

esp_err_t nvs_save_blob_data(const char *name, const char *key, const void* data, size_t length){
    nvs_handle_t handle = 0;
    esp_err_t err = ESP_FAIL;
    err = nvs_open(name, NVS_READWRITE, &handle);
    if(err != ESP_OK) return err;
    err = nvs_set_blob(handle, key, data, length);
    if(err != ESP_OK) goto error;
    err = nvs_commit(handle);
    if(err != ESP_OK) goto error;

    error:
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_blob_data(const char *name, const char *key, void* data, size_t length){
    nvs_handle_t handle = 0;
    esp_err_t err = ESP_FAIL;
    size_t _s = 0;

    err = nvs_open(name, NVS_READONLY, &handle);
    if(err != ESP_OK) return err;
    err = nvs_get_blob(handle, key, NULL, &_s);
    if(err != ESP_OK) goto error;

    _s = (_s >= length) ? length : _s;
    err = nvs_get_blob(handle, key, data, &_s);
    if(err != ESP_OK) goto error;
    error:
    nvs_close(handle);
    return err;
}

/***
esp_err_t nvs_save_ads_device(const void* device, const uint8_t length){
    nvs_handle_t handle = 0;
    esp_err_t err = ESP_FAIL;
    err = nvs_open("ads_device", NVS_READWRITE, &handle);
    if(err != ESP_OK) return err;
    err = nvs_set_blob(handle, "device", device, length);
    if(err != ESP_OK) goto error;
    err = nvs_commit(handle);
    if(err != ESP_OK) goto error;

    error:
    nvs_close(handle);
    return err;
}


esp_err_t nvs_read_ads_device(void* device, const uint8_t length){
    nvs_handle_t handle = 0;
    esp_err_t err = ESP_FAIL;
    size_t _s = 0;

    err = nvs_open("ads_device", NVS_READONLY, &handle);
    if(err != ESP_OK) return err;
    err = nvs_get_blob(handle, "device", NULL, &_s);
    if(err != ESP_OK) goto error;

    _s = (_s >= length) ? length : _s;
    err = nvs_get_blob(handle, "device", device, &_s);
    if(err != ESP_OK) goto error;
    error:
    nvs_close(handle);
    return err;
}
 ***/

esp_err_t spiffs_initializer(const char* base_path, const char *label){
    esp_err_t err = ESP_FAIL;
    
    esp_vfs_spiffs_conf_t conf = {
      .base_path = base_path,
      .partition_label = label,
      .max_files = 2,
      .format_if_mount_failed = false
    };
    err = esp_vfs_spiffs_register(&conf);
    if(err == ESP_OK){
        size_t total = 0, used = 0;
        ESP_LOGI(TAG, "spiffs partition find\r\n");
        err = esp_spiffs_info(conf.partition_label, &total, &used);
        if(err == ESP_OK){
            ESP_LOGI(TAG, "[%s] partition size: total: %d, used: %d", conf.partition_label, total, used);
        }else{
            esp_vfs_spiffs_unregister(conf.partition_label);
            ESP_LOGE(TAG, "failed to get SPIFFS partition information (%s)", esp_err_to_name(err));
        }
    }else{
        ESP_LOGE(TAG, "find spiffs parttition occur error\r\n");
    }
    return err;
}

#if(CONFIG_ORSYSTEM_MODE == 0)
#include "ll_mcpwm.h"

esp_err_t esp32_mcpwm_init(mcpwm_dev_t * mcpwm, mcpwm_operator_t op, const ll_mcpwm_config_t* config){
    esp_err_t err = ESP_FAIL;

    err = ll_mcpwm_init(&MCPWM0, op, config);

    ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);   /// 起始电平为低

    ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);   /// 起始电平为低

    return err;
}

esp_err_t esp32_mcpwm_start(mcpwm_dev_t *mcpwm, mcpwm_operator_t op){

    io_mcpwm_cs(0);
    mcpwm_ll_timer_start(mcpwm, op);
    return ESP_OK;
}


esp_err_t esp32_mcpwm_stop(mcpwm_dev_t *mcpwm, mcpwm_operator_t op){
    mcpwm_ll_timer_stop(mcpwm, op);
    return ESP_OK;
}


esp_err_t esp32_mcpwm_set_duty(mcpwm_dev_t *mcpwm, mcpwm_operator_t op, const float duty){

    return ESP_OK;
}


esp_err_t esp32_mcpwm_set_frequency(mcpwm_dev_t *mcpwm, mcpwm_operator_t op, const uint32_t frequency){

    return ESP_OK;
}

#endif

