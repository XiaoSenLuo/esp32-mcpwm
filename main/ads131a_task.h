/*
 * @Author: your name
 * @Date: 2022-03-23 15:15:22
 * @LastEditTime: 2022-03-24 13:53:51
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \orsystem\main\ads131a_task.h
 */


#ifndef _ADS131A_TASK_H__
#define _ADS131A_TASK_H__


#include "esp_system.h"

#include "orsystem_gpio.h"
#include "hardware_access.h"
#include "include/ads131a.h"

typedef enum{
    ADS_RESET,
    ADS_STANDBY,
    ADS_WAKEUP,
    ADS_LOCK,
    ADS_UNLOCK,
    ADS_STARTUP,
    ADS_PWRON,
    ADS_PWROFF,
    ADS_CONFIGURE,
    CTRL_START,
    CTRL_STOP
}ads131a_action_t;


esp_err_t ads131a_create_configure_task(int priority, uint32_t stack_size, void* ctx);

esp_err_t ads131a_delete_configure_task(void);

esp_err_t ads131a_t_action(const ads131a_action_t action, const void *ctx);

/**
 * 创建读取采样数据任务, 启动后会阻塞等待 DRDY 中断响应服务函数发送通知
 * @param priority
 * @param stack_size
 * @param ctx
 * @return
 */
esp_err_t ads131a_create_data_task(int priority, uint32_t stack_size, void* ctx);

esp_err_t ads131a_delete_data_task();

/**
 * 发送通知, 读取采样数据, 此函数应该在 DRDY 中断服务函数中调用
 * @return
 */
void ads131a_drdy_callback(void* ctx);


#endif

