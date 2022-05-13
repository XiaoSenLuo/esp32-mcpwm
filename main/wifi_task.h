//
// Created by XIAOSENLUO on 2022/4/6.
//

#ifndef ORSYSTEM_WIFI_TASK_H
#define ORSYSTEM_WIFI_TASK_H

#include <stdio.h>
#include "stdint.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"



esp_err_t ads131a_create_wifi_task(int priority, uint32_t stack_size, void* ctx);

esp_err_t ads131a_delete_wifi_task(void);







#endif //ORSYSTEM_WIFI_TASK_H
