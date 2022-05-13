#include <sys/cdefs.h>
#include <sys/timespec.h>
#include <sys/cdefs.h>
/*
 * @Author: your name
 * @Date: 2022-03-23 15:07:34
 * @LastEditTime: 2022-04-05 09:39:28
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \orsystem\main\ads131a_task.c
 */


#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "ads131a_task.h"


static SemaphoreHandle_t ads_data_sem = NULL;      // 数据读取任务同步
static QueueHandle_t     ads_action_queue = NULL;  // ADS131A 配置同步
static QueueHandle_t     ads_config_queue = NULL;  // ADS131A 配置

static TaskHandle_t      ads_configure_task_handle = NULL;
static TaskHandle_t      ads_action_task_handle = NULL;
static TaskHandle_t      ads_data_task_handle = NULL;


static const char TAG[] = "_ADS131A_TASK_C";

static ads131a_device_t ads131a = ADS131A_M0_M1_M2_DEVICE(1,1,0);

_Noreturn static void ads131a_configure_task(void* ctx){

    spi_device_handle_t spi = (spi_device_handle_t)(ctx);
    ESP_LOGD(TAG, "ads131a_configure_task(void* ctx): ctx = %#X\r\n", (uint32_t)ctx);
    BaseType_t xReturn = pdFAIL;

    ads131a_action_t action = ADS_STARTUP;
    xQueueSend(ads_action_queue, &action, portMAX_DELAY);

    // 使能内部电荷泵
    ADS131A_ENABLE_NEGATIVE_PUMP(&ads131a);
    // 使能比较器阈值检测
    ADS131A_SET_COMP_THRESHOLD(&ads131a, 0);  // H:95%, L:5%
    // 设置 ADC 输入时钟分频
    ADS131A_SET_ADC_CLK_DIV(&ads131a, CLK1_CLK_DIV_2);   // f_iclk = f_clkin / div
    // 设置 ADC 调制时钟分频
    ADS131A_SET_MODULATOR_CLK_DIV(&ads131a, CLK2_ICLK_DIV_2);  // f_mod = f_iclk / div
    // 设置 过采样因子
    ADS131A_SET_OSR(&ads131a, CLK2_OSR_400); // f_data = f_mod / osr

    action = ADS_CONFIGURE;
    xQueueSend(ads_action_queue, &action, portMAX_DELAY);
    xQueueSend(ads_config_queue, &ads131a.config, portMAX_DELAY);

    // 使能 ADC
    action = CTRL_START;
    xQueueSend(ads_action_queue, &action, portMAX_DELAY);
    // 唤醒 device
    action = ADS_WAKEUP;
    xQueueSend(ads_action_queue, &action, portMAX_DELAY);

    // 锁定设备
    action = ADS_LOCK;
    xQueueSend(ads_action_queue, &action, portMAX_DELAY);

    while(true){
        xReturn = xQueueReceive(ads_action_queue, &action, portMAX_DELAY);
        if(xReturn == pdFAIL) continue;
        if(action == ADS_RESET){
            uint16_t response = ads131a_opc_reset(&ads131a);
            if(response == OPCODE_READY(&ads131a)) ESP_LOGI(TAG, "device reset!\r\n");
        }else if(action == ADS_STANDBY){
            uint16_t response = ads131a_opc_standby(&ads131a);
            if(response == OPCODE_STANDBY) ESP_LOGI(TAG, "device standby!\r\n");
        }else if(action == ADS_WAKEUP){
            uint16_t response = ads131a_opc_wakeup(&ads131a);
            if(response == OPCODE_WAKEUP) ESP_LOGI(TAG, "device wakeup!\r\n");
        }else if(action == ADS_LOCK){
            uint16_t response = ads131a_opc_lock(&ads131a);
            if(response == OPCODE_LOCK) ESP_LOGI(TAG, "device locked!\r\n");
        }else if(action == ADS_UNLOCK){
            uint16_t response = ads131a_opc_unlock(&ads131a);
            if(response == OPCODE_UNLOCK) ESP_LOGI(TAG, "device unlocked!\r\n");
            else ESP_LOGI(TAG, "device locked!");
        }else if(action == ADS_STARTUP){
            ads131a_startup(&ads131a);
            ads131a_assert_printf_state(ads131a.state);
        }else if(action == ADS_PWRON){
            io_set_analog_power(1);
        }else if(action == ADS_PWROFF){
            io_set_analog_power(0);
        }else if(action == ADS_CONFIGURE){
            ads131a_config_t conf;
            BaseType_t xReturn = xQueueReceive(ads_config_queue, &conf, portMAX_DELAY);
            if(xReturn == pdPASS) {
                ESP_LOGI(TAG, "configure ads131a device!\r\n");
                ads131a.config = conf;
                ads131a_configure_device(&ads131a);
            }
        }else if(action == CTRL_START){
            uint16_t response = 0;
            response = ads131a_adcen(&ads131a);
            if(response != ADC_ENA_ENA_ALL_CH_PWUP){
                ESP_LOGE(TAG, "ADC channel enable fail!\r\n");
            }else{
                ESP_LOGI(TAG, "ADC channel enabled!\r\n");
            }
            ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));   // 申请独占SPI总线, 防止出现总线竞争导致无法及时读取采样数据, 以及减少等待获取总线控制权时间, 提高响应速度
            ads131a_create_data_task(9, 4096, ctx);   // 创建数据读取任务, 优先级要比配置任务低, 以便配置任务可以停止数据读取任务
        }else if(action == CTRL_STOP){
            uint16_t response = 0;
            response = ads131a_adcoff(&ads131a);
            if(response != ADC_ENA_ENA_ALL_CH_PWDN){
                ESP_LOGE(TAG, "ADC channel disable fail!\r\n");
            }else{
                ESP_LOGI(TAG, "ADC channel disabled!\r\n");
            }
            ads131a_delete_data_task();         // 删除数据读取任务
            spi_device_release_bus(spi);        // 释放 SPI 总线控制权
        }else{

        }
    }

    vTaskDelete(NULL);
    ads_action_task_handle = NULL;
}

esp_err_t ads131a_create_configure_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;

    if(ads_action_queue == NULL){
        ads_action_queue = xQueueCreate(10, sizeof(ads131a_action_t));
    }
    if(ads_config_queue == NULL){
        ads_config_queue = xQueueCreate(2, sizeof(ads131a_config_t));
    }
    if(ads_action_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(ads131a_configure_task, "ads131a_configure_task", stack_size, ctx, priority, &ads_configure_task_handle, 1);
        if(xReturn != pdPASS){
            ESP_LOGE(TAG, "\"ads131a_configure_task(%XH)\" create fail!\r\n", (uint32_t)ctx);
            return ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "\"ads131a_configure_task(%XH)\" create success!\r\n", (uint32_t)ctx);
        }
    }

    return ESP_OK;
}

esp_err_t ads131a_delete_configure_task(void){
    if(ads_configure_task_handle){
        vTaskDelete(ads_configure_task_handle);
        ads_configure_task_handle = NULL;
    }
    if(ads_action_queue != NULL){
        vQueueDelete(ads_action_queue);
        ads_action_queue = NULL;
    }
    if(ads_config_queue != NULL){
        vQueueDelete(ads_config_queue);
        ads_config_queue = NULL;
    }
    return ESP_OK;
}

/**
esp_err_t ads131a_t_configure_device(const ads131a_config_t *config){
    if(ads_config_queue){
        BaseType_t xReturn = xQueueSend(ads_config_queue, config, portMAX_DELAY);
        return (xReturn == pdPASS) ? ESP_OK : ESP_FAIL;
    }
    return ESP_FAIL;
}
**/


esp_err_t ads131a_t_action(const ads131a_action_t action, const void *ctx){
    ads131a_action_t tmp = action;
    BaseType_t xReturn = pdFAIL;
    esp_err_t err = ESP_OK;
    if(action == ADS_CONFIGURE){
        xReturn = xQueueSend(ads_config_queue, ctx, portMAX_DELAY);
        if(xReturn != pdPASS){
            err |= ESP_FAIL;
        }
    }
    if(ads_action_queue){
        xReturn = xQueueSend(ads_action_queue, &tmp, portMAX_DELAY);
        if(xReturn != pdPASS){
            ESP_LOGE(TAG, "\"ads131a_action(%d)\" send action fail!\r\n", (int)action);
            err |= ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "\"ads131a_action(%d)\" send action success!\r\n", (int)action);
        }
    }
    return err;
}

/***
 * 读取 ADS131A 采样数据
 * @param ctx: ADS131A 设备结构体指针
 */
_Noreturn static void ads131a_data_task(void* ctx){
    BaseType_t xReturn = pdFAIL;
    ads131a_device_t *device = &ads131a;
    uint32_t notify_value = 0;
    gpio_config_t io_conf;

    spi_device_handle_t spi = (spi_device_handle_t)(ctx);
    ESP_LOGD(TAG, "ads131a_data_task(void* ctx): ctx = %#X\r\n", (uint32_t)ctx);

    ads131a_rawdata_t data = {.channel_data = {0, 0, 0, 0}, .rawcrc = 0, .calcrc = 0};

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;   // 下降沿触发
    io_conf.pin_bit_mask = 1ULL << IO_ADS_ADRDY;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);

    TaskHandle_t t_handle = xTaskGetCurrentTaskHandle();
    gpio_isr_handler_add(IO_ADS_ADRDY, ads131a_drdy_callback, &t_handle);

    while(true){
#if(0)
        xReturn = xSemaphoreTake(ads_data_sem, portMAX_DELAY);
        if(xReturn != pdPASS) continue;
        // TODO @ 读取采样数据
#elif(1)
        notify_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(notify_value > 0){
            // TODO @ 读取采样数据
            uint16_t response = 0;
            response = ads131a_get_channel_data(device, &data);

        }
#endif
    }
    vTaskDelete(NULL);
    ads_data_task_handle = NULL;
}

/**
 *
 * @param priority
 * @param stack_size
 * @param ctx
 * @return
 */
esp_err_t ads131a_create_data_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;
#if(0)
    if(ads_data_sem == NULL){
        ads_data_sem = xSemaphoreCreateBinary();
    }
#elif (1)
    if(ads_data_task_handle == NULL){
        // 在核心1创建任务
        xReturn = xTaskCreatePinnedToCore(ads131a_data_task, "ads131a_data_task", stack_size, ctx, priority,
                                          &ads_data_task_handle, 1);
        if(xReturn != pdPASS){
            ESP_LOGE(TAG, "ads131a_data_task() create fail!\r\n");
            return ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "ads131a_data_task() create success!\r\n");
        }
    }
#endif
    return ESP_OK;
}

esp_err_t ads131a_delete_data_task(){
    gpio_isr_handler_remove(IO_ADS_ADRDY);

    gpio_isr_handler_remove(IO_ADS_ADRDY);
    gpio_uninstall_isr_service();

    if(ads_data_task_handle){
        vTaskDelete(ads_data_task_handle);
        ads_data_task_handle = NULL;
    }
#if(0)
    if(ads_data_sem){
        vSemaphoreDelete(ads_data_sem);
        ads_data_sem = NULL;
    }
#endif
    return ESP_OK;
}

void ads131a_drdy_callback(void* ctx){
//    BaseType_t err = pdFAIL;
    BaseType_t token = pdFAIL;

    TaskHandle_t *t_handle = (TaskHandle_t*)ctx;
#if(0)
    if(ads_data_sem){
        xSemaphoreGiveFromISR(ads_data_sem, &err);
    }
#elif(1)
    if(ads_data_task_handle){
        vTaskNotifyGiveFromISR(ads_data_task_handle, &token);
        portYIELD_FROM_ISR(token);  // token == true, 则执行上下文切换
    }
#endif
}

