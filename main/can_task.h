//
// Created by XIAO on 2022/3/25.
//

#ifndef ORSYSTEM_CAN_TASK_H
#define ORSYSTEM_CAN_TASK_H


#include "esp_system.h"

#include "orsystem_gpio.h"
#include "hardware_access.h"

esp_err_t twai_create_check_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t twai_delete_check_task();

esp_err_t twai_create_transmit_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t twai_delete_transmit_task();

esp_err_t twai_create_receive_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t twai_delete_receive_task();




#endif //ORSYSTEM_CAN_TASK_H
