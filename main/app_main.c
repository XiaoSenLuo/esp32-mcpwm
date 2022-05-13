/*
 * @Author: your name
 * @Date: 2022-03-24 17:19:23
 * @LastEditTime: 2022-05-08 21:19:15
 * @LastEditors: xiaosenluo xiaosenluo@yandex.com
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \orsystem\main\app_main.c
 */
/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include "sdkconfig.h"

#include "main.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "esp_partition.h"
#include "esp_flash_partitions.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_app_format.h"

#include "hardware_access.h"
#include "orsystem_task.h"

static const char TAG[] = "_APP_MAIN_C";

// spi_device_handle_t vspi = NULL;

orsys_config_reg_t orsystem_configure = {
    .id = (uint32_t)CONFIG_ORSYSTEM_ID,
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .uart = {
            .index = UART_NUM_0,
            .config = {
                .baud_rate = 921600,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_APB,
            },
    },
    .can = {
        // .filter = TWAI_FILTER_CONFIG_ACCEPT_ALL(),
        .timing = TWAI_TIMING_CONFIG_125KBITS(),
        // .config = TWAI_GENERAL_CONFIG_DEFAULT(IO_CAN_TX, IO_CAN_RX, TWAI_MODE_NORMAL),
    },
#if(CONFIG_ORSYSTEM_MODE)
    .analog = {
        .ads131a_dev = {
            .dev = ADS131A_M0_M1_M2_DEVICE(1, 1, 0), 
            .state.val = 0x0F,
            .vref = 2500
        },
        .atime = 1000,
        .ctrl.val = 0,
    },
#else
    .mcpwm = {
        .duty = 50,
        .frequency = 950 * 2,
        .pulse_num = 3,
    },
#endif
};


static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

void app_main(void){
     /* Print chip information */
    esp_err_t err = ESP_FAIL;

    bool twai_ok = false;

    
    nvs_init();               // 初始化 NVS 分区
    
    orsys_event_handle = xEventGroupCreate();

    // do{
    //     orsys_config_reg_t config;
    //     if(nvs_read_orsystem_config(&config, sizeof(orsys_config_reg_t)) == ESP_OK){
    //         orsystem_configure = config;
    //         orsystem_configure.mux.owner = SPINLOCK_FREE;
    //         orsystem_configure.mux.count = 0;
    //         ESP_LOGI(TAG, "read system config from file!\r\n");
    //     }
    // }while(0);

    do{
        esp32_uart_init(orsystem_configure.uart.index, &orsystem_configure.uart.config);
    }while(0);
    ESP_LOGI(TAG, "OrSystem Starting!\r\n");
    do{
        const esp_partition_t *running = esp_ota_get_running_partition();
        esp_app_desc_t running_app_info;
        esp_chip_info_t chip_info;

        if(esp_ota_get_partition_description(running, &running_app_info) == ESP_OK){
            ESP_LOGI(TAG, "Running firmware version: %s, build time: %s %s\r\n", running_app_info.version, running_app_info.date, running_app_info.time);
        }
        esp_chip_info(&chip_info);
        ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET, chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
        ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);
        ESP_LOGI(TAG, "%dMB %s flash\n", (int)(spi_flash_get_chip_size() / (1024 * 1024)), (chip_info.features & BIT(0)) ? "embedded" : "external");
        ESP_LOGI(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

//        ESP_LOGI(TAG, "can bus self test.......\r\n");
//        err = esp32_can_self_test();
//        if(err != ESP_OK){
//            ESP_LOGE(TAG, "can bus self test is fail, will restrt in 5 seconds......\r\n");
//            vTaskDelay(pdMS_TO_TICKS(5000));
//            esp_restart();
//            goto end_main;
//        }else{
            twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(IO_CAN_TX, IO_CAN_RX, TWAI_MODE_NORMAL);
#if(CONFIG_ORSYSTEM_TWAI_FILTER_ENABLE == 0)
            twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#else
#if(CONFIG_ORSYSTEM_TWAI_SINGLE_FILTER)
            twai_single_filter_code_t fcode = {
                .standard_frame.code_value = 0UL,
            };
            twai_single_filter_code_t fmark = {
                .standard_frame.code_value = 0xFFFFFFFFUL,
            };
            fcode.standard_frame.id = 0x30;   /// 接受 3xh
            fmark.standard_frame.id = 0x0F;   /// 忽略 ID低4位
            twai_filter_config_t f_config = {
                .acceptance_code = fcode.standard_frame.code_value,
                .acceptance_mask = fmark.standard_frame.code_value,
                .single_filter = true,
            };
#else
            twai_dual_filter_code_t fcode = {
                .standard_frame.code_value = 0x0UL,
            };
            twai_dual_filter_code_t fmark = {
                .standard_frame.code_value = 0xFFFFFFFFUL,
            };
            fcode.standard_frame.filter_1.id = 0x30;
            fcode.standard_frame.filter_2.id = 0x40;

            fmark.standard_frame.filter_1.id = 0x0F;   /// 忽略ID低4位
            fmark.standard_frame.filter_2.id = 0x1F;   /// 忽略ID低5位
            twai_filter_config_t f_config = {
                .acceptance_code = fcode.standard_frame.code_value,
                .acceptance_mask = fmark.standard_frame.code_value,
                .single_filter = false,
            };
#endif
#endif
            err = esp32_can_install(g_config, orsystem_configure.can.timing, f_config);
            if(err == ESP_OK) twai_ok = true;
//        }
    }while(0);
     // TODO @ 初始化硬件
    gpio_init();

#if(CONFIG_ORSYSTEM_MODE)

    io_set_buzzer(1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    io_set_buzzer(0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    io_ads131a_reset(0);
    io_set_analog_power(1);
    io_set_analog_channel(0);
#if(0)
    // TODO @ 挂载 storage 分区镜像作为存储空间
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 2,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    err = esp_vfs_fat_spiflash_mount(CONFIG_SPIFFS_STORAGE_BASE_PATH, "storage", &mount_config, &s_wl_handle);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
        goto end_main;
    }else{
        ESP_LOGI(TAG, "storage image mounted!\r\n");
    }
#elif(0)
    err = spiffs_initializer(CONFIG_SPIFFS_STORAGE_BASE_PATH, CONFIG_SPIFFS_STORATE_LABEL);
    ESP_ERROR_CHECK(err);
    if(err != ESP_OK){
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
        goto end_main;
    }
#endif

    esp32_vspi_init(&vspi);   // 初始化 SPI
    ads131a_drv_init(vspi, IO_VSPI_CS);  // 初始化 ADS131A 驱动句柄
#endif

    // TODO @ 创建基本任务
    uart_create_uart_receive_task(16, 2048, NULL);

    if(twai_ok){
        twai_create_twai_task(13, 2048, NULL);

        twai_wait_for_initializer();
    }

#if(CONFIG_ORSYSTEM_MODE)
    ads131a_create_configure_task(10, 4096, NULL);  // 创建 ADS131A 设备配置任务
    ads131a_wait_for_configure_complete();
    ads131a_create_data_task(5, 4096, NULL);   // 创建数据读取任务, 优先级要比配置任务低, 以便配置任务可以停止数据读取任务
#else

    mcpwm_create_mcpwm_task(10, 4096, NULL);
    gpio_create_gpio_task(11, 2048, NULL);
#endif

#if(CONFIG_ORSYSTEM_MODE)

// end_main:
   fflush(stdout);

#endif

}

