/*
 * @Author: xiaosenluo xiaosenluo@yandex.com
 * @Date: 2022-05-08 21:28:14
 * @LastEditors: xiaosenluo xiaosenluo@yandex.com
 * @LastEditTime: 2022-05-09 00:05:26
 * @FilePath: \orsystem\main\twai_ota.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _TWAI_OTA_H__
#define _TWAI_OTA_H__


#include <stdint.h>
#include <stdio.h>
#include "esp_system.h"

typedef struct u64_twai_ota_data_header_s{
    uint32_t id;
    uint32_t message;
};

typedef struct u64_twai_ota_data_header_s * u64_twai_ota_data_header_handle;

esp_err_t ota_create_ota_task(int priority, uint32_t stack_size, void* ctx);
void ota_delete_ota_task(void); 

void ota_wait_ota_complete(void);



#endif
