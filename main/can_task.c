#include <sys/cdefs.h>
#include <sys/timespec.h>
#include <sys/cdefs.h>
//
// Created by XIAO on 2022/3/25.
//


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "can_task.h"


typedef enum {
    ID_SLAVE_CONFIG_DATA = 0x0061,
    ID_SLAVE_PING,
    ID_SLAVE_PING_RESP,
    ID_SLAVE_TRANSMIT_START,
    ID_SLAVE_TRANSMIT_END,
    ID_SLAVE_TIME_SYNCHRONOUS,
}twai_slave_id_t;

typedef enum {
    ID_MASTER_CONFIG_DATA = 0x0081,
    ID_PING,
    ID_MASTER_PING_RESP,
    ID_MASTER_TRANSMIT_START,
    ID_MASTER_TRANSMIT_END,
    ID_MASTER_TIME_SYNCHRONOUS,
}twai_master_id_t;


static const char TAG[] = "CAN_TASK";

static TaskHandle_t twai_transmit_task_handle = NULL;
static TaskHandle_t twai_receive_task_handle = NULL;
static TaskHandle_t twai_task_handle = NULL;

#define TWAI_ID_QUEUE_LENGTH                    10
static QueueHandle_t twai_transmit_id_queue = NULL;
static QueueHandle_t twai_transmit_ctrl_queue = NULL;
static QueueHandle_t twai_receive_id_queue = NULL;


_Noreturn static void twai_check_task(void* ctx){
    esp_err_t err = ESP_FAIL;
    uint32_t alert = TWAI_ALERT_BELOW_ERR_WARN | TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_RECOVERY_IN_PROGRESS | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ARB_LOST | TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_AND_LOG;
    err = twai_reconfigure_alerts(alert, NULL);
    ESP_ERROR_CHECK(err);

    while(true){
        alert = 0;
        err = twai_read_alerts(&alert, portMAX_DELAY);
        if(err == ESP_OK){
            if(alert & TWAI_ALERT_BUS_OFF){

            }else if(alert & TWAI_ALERT_ERR_PASS){

            }else if(alert & TWAI_ALERT_RX_QUEUE_FULL){

            }else if(alert & TWAI_ALERT_TX_FAILED){

            }else if(alert & TWAI_ALERT_BUS_ERROR){

            }else if(alert & TWAI_ALERT_ABOVE_ERR_WARN){

            }else if(alert & TWAI_ALERT_ARB_LOST){

            }else if(alert & TWAI_ALERT_BUS_RECOVERED){

            }else if(alert & TWAI_ALERT_RECOVERY_IN_PROGRESS){

            }else if(alert & TWAI_ALERT_ERR_ACTIVE){

            }else if(alert & TWAI_ALERT_BELOW_ERR_WARN){

            }else{

            }
        }
    }
    vTaskDelete(NULL);
    twai_task_handle = NULL;
}


esp_err_t twai_create_check_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;
    if(twai_transmit_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(twai_check_task, "twai_check_task", stack_size, ctx, priority,
                                          &twai_task_handle, tskNO_AFFINITY);
        if(xReturn != pdPASS){
            ESP_LOGE(TAG, "twai_check_task() create fail!\r\n");
            return ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "twai_check_task() create success!\r\n");
        }
    }
    return ESP_OK;
}

esp_err_t twai_delete_check_task(){
    if(twai_task_handle){
        vTaskDelete(twai_task_handle);
        twai_task_handle = NULL;
    }
    return ESP_OK;
}


_Noreturn static void twai_transmit_task(void* ctx){
    esp_err_t err = ESP_FAIL;
    BaseType_t xReturn = pdFAIL;
    uint32_t id = 0;

    while(true){
        id = 0;
        xQueueReceive(twai_transmit_id_queue, &id, 0);

        if(id){
            if(id == ID_SLAVE_PING_RESP){
                twai_message_t t_message = {.identifier = id, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
                twai_transmit(&t_message, portMAX_DELAY);
            }else if(id == ID_SLAVE_PING){
                twai_message_t t_message = {.identifier = id, .ss = 1, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
                twai_transmit(&t_message, portMAX_DELAY);
            }else if(id == ID_SLAVE_CONFIG_DATA){

            }else if(id == ID_SLAVE_TIME_SYNCHRONOUS){

            }else if(id == ID_SLAVE_TRANSMIT_START){

            }else if(id == ID_SLAVE_TRANSMIT_END){

            }else{

            }
        }
    }
    vTaskDelete(NULL);
    twai_transmit_task_handle = NULL;
}

esp_err_t twai_create_transmit_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;

    if(twai_transmit_id_queue == NULL) twai_transmit_id_queue = xQueueCreate(TWAI_ID_QUEUE_LENGTH, sizeof(uint32_t));
    if(twai_receive_id_queue == NULL) twai_receive_id_queue = xQueueCreate(TWAI_ID_QUEUE_LENGTH, sizeof(uint32_t));

    if(twai_transmit_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(twai_transmit_task, "twai_transmit_task", stack_size, ctx, priority, &twai_transmit_task_handle, tskNO_AFFINITY);
        if(xReturn != pdPASS){
            ESP_LOGE(TAG, "twai_transmit_task() create fail!\r\n");
            return ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "twai_transmit_task() create success!\r\n");
        }
    }
    return ESP_OK;
}



esp_err_t twai_delete_transmit_task(){
    if(twai_transmit_task_handle){
        vTaskDelete(twai_transmit_task_handle);
        twai_transmit_task_handle = NULL;
    }
    return ESP_OK;
}


_Noreturn static void twai_receive_task(void* ctx){
    esp_err_t err = ESP_FAIL;

    while(true){
        twai_message_t r_message;
        err = twai_receive(&r_message, portMAX_DELAY);
        if(err == ESP_OK){
            // TODO @ 处理来自CAN主机的数据
            if(r_message.identifier == ID_PING){
                uint32_t id = ID_SLAVE_PING_RESP;
                xQueueSend(twai_transmit_id_queue, &id, portMAX_DELAY);
            }else if(r_message.identifier == ID_MASTER_CONFIG_DATA){

            }else if(r_message.identifier == ID_MASTER_PING_RESP){

            }else if(r_message.identifier == ID_MASTER_TIME_SYNCHRONOUS){

            }else if(r_message.identifier == ID_MASTER_TRANSMIT_START){

            }else if(r_message.identifier == ID_MASTER_TRANSMIT_END){

            }else{

            }
        }
    }
    vTaskDelete(NULL);
    twai_receive_task_handle = NULL;
}

esp_err_t twai_create_receive_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;

    if(twai_transmit_id_queue == NULL) twai_transmit_id_queue = xQueueCreate(TWAI_ID_QUEUE_LENGTH, sizeof(uint32_t));
    if(twai_receive_id_queue == NULL) twai_receive_id_queue = xQueueCreate(TWAI_ID_QUEUE_LENGTH, sizeof(uint32_t));

    if(twai_receive_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(twai_receive_task, "twai_receive_task", stack_size, ctx, priority, &twai_receive_task_handle, tskNO_AFFINITY);
        if(xReturn != pdPASS){
            ESP_LOGE(TAG, "twai_receive_task() create fail!\r\n");
            return ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "twai_receive_task() create success!\r\n");
        }
    }
    return ESP_OK;
}


esp_err_t twai_delete_receive_task(){
    if(twai_receive_task_handle){
        vTaskDelete(twai_receive_task_handle);
        twai_receive_task_handle = NULL;
    }
    return ESP_OK;
}


