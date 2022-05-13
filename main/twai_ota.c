/*
 * @Author: xiaosenluo xiaosenluo@yandex.com
 * @Date: 2022-05-08 21:28:29
 * @LastEditors: xiaosenluo xiaosenluo@yandex.com
 * @LastEditTime: 2022-05-09 00:21:49
 * @FilePath: \orsystem\main\twai_ota.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "errno.h"
#include "driver/twai.h"
#include "esp_log.h"

#include "twai_ota.h"
#include "twai_id_def.h"


static const char TAG[] = "_TWAI_OTA_C";

static TaskHandle_t twai_ota_task_handle = NULL;

#define OTA_EVENT_OTA_START_BIT               BIT(0)
#define OTA_EVENT_OTA_CPLT_BIT                BIT(1)
#define OTA_EVENT_OTA_ERROR_BIT               BIT(2)

static EventGroupHandle_t ota_event_handle = NULL;



#define OTA_WRITE_BUFFER_SIZE                        4096
static uint8_t * ota_write_buffer_ptr = NULL;



static void task_fatal_error(void)
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    xEventGroupSetBits(ota_event_handle, OTA_EVENT_OTA_ERROR_BIT);
}

static size_t twai_receive_data_block(void * data_ptr, size_t data_size){
    size_t rec_length = 0;
    esp_err_t err = ESP_OK;
    twai_message_t r_message = {0};
    
    while(rec_length <= data_size){
        err = twai_receive(&r_message, pdMS_TO_TICKS(500));  /// 超时 500ms
        if((err == ESP_OK) && (r_message.rtr == 0) && (r_message.identifier == ID_DATA_OTA)){
            if(rec_length + r_message.data_length_code <= data_size){
                memcpy((void*)(data_ptr + rec_length), r_message.data, r_message.data_length_code);
                rec_length += r_message.data_length_code;
            }else{
                if((data_size - rec_length)) memcpy((void*)(data_ptr + rec_length), r_message.data, (data_size - rec_length));
                rec_length = data_size;
                break;
            }
        }
        if((err != ESP_OK) && (r_message.identifier == ID_DATA_OTA)){
            break;
        }
    }
    
    return rec_length;
}



static __attribute__((noreturn)) void twai_ota_task(void* ctx){
    
    bool image_header_was_checked = false;
    size_t image_length = 0;
    size_t receive_file_length = 0;
    twai_message_t message;
    esp_err_t err = ESP_FAIL;

    err = twai_receive(&message, portMAX_DELAY);
    if(err != ESP_OK){
        task_fatal_error();
        goto task_end_section;
    }

    u64_twai_ota_data_header_handle handle = (u64_twai_ota_data_header_handle)message.data;
    if(handle->id != CONFIG_ORSYSTEM_ID){
        task_fatal_error();
        goto task_end_section;
    }
    image_length = handle->message;
    ESP_LOGI(TAG, "new image size:%d byte\r\n", image_length);
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);
    if(ota_write_buffer_ptr == NULL){
        ota_write_buffer_ptr = (uint8_t*)malloc(4096 + 1);
        if(ota_write_buffer_ptr == NULL){
            task_fatal_error();
            goto task_end_section;
        }
    }
    memset(&message, 0, sizeof(twai_message_t));
    message.identifier = ID_DATA_OTA_REQUEST;
    message.data_length_code = 8;
    handle->id = CONFIG_ORSYSTEM_ID;
    handle->message = OTA_WRITE_BUFFER_SIZE;
    
    xEventGroupSetBits(ota_event_handle, OTA_EVENT_OTA_START_BIT);

    while(1){
        size_t data_read = 0;
        twai_transmit(&message, portMAX_DELAY);   /// 请求数据
        data_read = twai_receive_data_block(ota_write_buffer_ptr, OTA_WRITE_BUFFER_SIZE);
        if(data_read > 0){
            if(!image_header_was_checked){
                esp_app_desc_t new_app_info = {0};
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)){
                    memcpy(&new_app_info, &ota_write_buffer_ptr[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);
                    esp_app_desc_t running_app_info = {0};
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            break;
                        }
                    }
                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                        goto task_end_section;
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");
                }else{
                    ESP_LOGE(TAG, "received package is not fit len");
                    esp_ota_abort(update_handle);
                    task_fatal_error();
                    goto task_end_section;
                }
            }
            err = esp_ota_write( update_handle, (const void *)ota_write_buffer_ptr, data_read);
            if (err != ESP_OK) {
                esp_ota_abort(update_handle);
                task_fatal_error();
                goto task_end_section;
            }
            receive_file_length += data_read;
            ESP_LOGD(TAG, "Written image length %d", receive_file_length);
        }else{
            break;
        }
    }
    ESP_LOGI(TAG, "Total Write binary data length: %d", receive_file_length);
    if(receive_file_length != image_length){
        ESP_LOGE(TAG, "Error in receiving complete file");
        esp_ota_abort(update_handle);
        task_fatal_error();
        goto task_end_section;
    }
    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        task_fatal_error();
        goto task_end_section;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        task_fatal_error();
        goto task_end_section;
    }

    task_end_section:
    xEventGroupSetBits(ota_event_handle, OTA_EVENT_OTA_CPLT_BIT);

    while(1){
        vTaskDelay(portMAX_DELAY);
    }
}

#undef OTA_WRITE_BUFFER_SIZE

esp_err_t ota_create_ota_task(int priority, uint32_t stack_size, void* ctx){
    esp_err_t err = ESP_FAIL;
    BaseType_t xReturn = pdFALSE;
    
    if(ota_event_handle == NULL){
        ota_event_handle = xEventGroupCreate();
    }

    xReturn = xTaskCreatePinnedToCore(twai_ota_task, "twai_ota_task", stack_size, ctx, priority, &twai_ota_task_handle, 0);
    
    if(xReturn != pdTRUE){
        ESP_LOGE(TAG, "twai_ota_task(%X) create fail\r\n", (uint32_t)ctx);
        err = ESP_FAIL;
    }else{
        ESP_LOGI(TAG, "twai_ota_task(%X) create success\r\n", (uint32_t)ctx);
        err = ESP_OK;
    }
    return err;
}


void ota_delete_ota_task(void){
    
    if(twai_ota_task_handle){
        vTaskDelete(twai_ota_task_handle);
        twai_ota_task_handle = NULL;
    }
    if(ota_event_handle){
        vEventGroupDelete(ota_event_handle);
        ota_event_handle = NULL;
    }
    if(ota_write_buffer_ptr){
        free(ota_write_buffer_ptr);
        ota_write_buffer_ptr = NULL;
    }
}

void ota_wait_ota_complete(void){
    EventBits_t gbit = 0;
    gbit = xEventGroupGetBits(ota_event_handle);
    while((!(gbit & OTA_EVENT_OTA_CPLT_BIT)) || (!(gbit & OTA_EVENT_OTA_ERROR_BIT))){
        gbit = xEventGroupWaitBits(ota_event_handle, OTA_EVENT_OTA_CPLT_BIT | OTA_EVENT_OTA_ERROR_BIT, true, false, portMAX_DELAY);
    }
    ota_delete_ota_task();
}