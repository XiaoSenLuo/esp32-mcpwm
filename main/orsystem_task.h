/*
 * @Author: your name
 * @Date: 2022-01-24 08:40:10
 * @LastEditTime: 2022-05-10 20:38:08
 * @LastEditors: xiaosenluo xiaosenluo@yandex.com
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \orsystem\main\orsystem_control.h
 */


#ifndef _ORSYSTEM_H__
#define _ORSYSTEM_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_system.h"

#include "hardware_access.h"

#if(CONFIG_ORSYSTEM_MODE)
#include "ads131a.h"
#endif

#include "twai_id_def.h"

#define IO_SET                                   1
#define IO_RESET                                 0

typedef struct u64_orsys_reg_config_format_s{
    uint8_t base_addr;
    uint8_t offset;
    union{
        struct{
            uint8_t data_length : 3;
            uint8_t bit3_7 : 5;
        }; 
    };
    union {
        struct{
            uint8_t bit0_6 : 7;
            uint8_t mode : 1;
        };
    };
    uint32_t data;
}u64_orsys_reg_config_format_t;

typedef struct u64_orsys_reg_config_format_s * u64_orsys_reg_config_format_handle_t;

typedef union u32_twai_timing_config_format_u{
    struct{
        uint32_t brp : 16;
        uint32_t tseg_1 : 5;
        uint32_t reserved : 3;
        uint32_t tseg_2 : 4;
        uint32_t sjw : 3;
        uint32_t triple_sampling : 1;
    };
    uint32_t val;
}u32_twai_timing_config_format_t;
typedef union u32_twai_timing_config_format_u * u32_twai_timing_config_format_handle;

typedef struct orsys_config_reg_s{
    uint32_t id;
    struct {
        uint8_t index;
        uart_config_t config;
    }uart __attribute__((deprecated));
    struct {
        // twai_general_config_t config;
        twai_timing_config_t timing;
        // twai_filter_config_t filter;
    }can;
    #if(CONFIG_ORSYSTEM_MODE)
    struct {
        ads131a_device_t ads131a_dev;
        int32_t atime; /// unit: ms
        union{
            struct{
                uint32_t data_return : 1;      /// 数据控制, 0: 响应式, 当接收到请求时返回数据. 1: 自动式, 当采集结束时自动返回数据.
                uint32_t ctrl_reserved : 31;
            };
            uint32_t val;
        }ctrl;
    }analog;
    #else
    struct {
        uint32_t frequency;
        uint32_t duty;
        int32_t pulse_num;
    }mcpwm;
    #endif
    portMUX_TYPE mux;
}orsys_config_reg_t;

typedef struct orsys_config_reg_s * orsys_config_reg_handle_t;

#define ORSYS_FORCE_WRITE_REG(orsys_config, reg, reg_value) do{ \
                                                                taskENTER_CRITICAL(&orsys_config.mux); \
                                                                orsys_config.reg = reg_value; \
                                                                taskEXIT_CRITICAL(&orsys_config.mux); \
                                                            }while(0)

#define ORSYS_FORCE_READ_REG(orsys_config, reg) { \
                                                    typeof(orsys_config) tmp; \
                                                    taskENTER_CRITICAL(&orsys_config.mux); \
                                                    tmp = orsys_config; \
                                                    taskEXIT_CRITICAL(&orsys_config.mux); \
                                                    tmp.reg; \
                                                }

extern orsys_config_reg_t orsystem_configure;

#define ORSYS_EVENT_OTA_BIT                    BIT(0)
#define ORSYS_EVENT_OTA_CPLT_BIT               BIT(1)

extern EventGroupHandle_t orsys_event_handle;

typedef union {
    union{
        struct {
            uint32_t data : 16;
            uint32_t unuse : 4;
            uint32_t rtr : 1;
            uint32_t id : 11;
        };
        uint32_t code_value;
    }standard_frame;
    union {
        struct {
            uint32_t unuse : 2;
            uint32_t rtr : 1;
            uint32_t id : 29;
        };
        uint32_t code_value;
    }extend_frame;
}twai_single_filter_code_t;

typedef union {
    union{
        struct {
            union {
                struct {
                    uint16_t data_lbit : 4;
                    uint16_t rtr : 1;
                    uint16_t id : 11;
                };
                uint16_t code_value;
            }filter_2;
            union {
                struct {
                    uint16_t data_hbit : 4;
                    uint16_t rtr : 1;
                    uint16_t id : 11;
                };
                uint16_t code_value;
            }filter_1;
        };
        uint32_t code_value;
    }standard_frame;
    union {
        struct {
            union {
                uint16_t id_13_28_bit;
            }filter_2;
            union {
                uint16_t id_13_28_bit;
            }filter_1;
        };
        uint32_t code_value;
    }extend_frame;
}twai_dual_filter_code_t;


/**********************************************************************************************************/





#if(CONFIG_ORSYSTEM_MODE)

/**  f_clkin = 16MHz  **/
/**
 * @brief D_SYS_CFG:FIXED=0,CRC_EN=0; CLK1:CLK_DIV[2:0]=1; CLK2:ICLK_DIV[2:0]=1; CLK2:OSR[3:0]=6
 *        动态帧模式, 关闭CRC输出, f_iclk = f_clkin / 2, f_mod = f_iclk / 2, f_data = f_mod / 400 = 10KHz
 */
#define ADS131A_CONFIGURE_DEFAULT_10KHZ()           {.analog_config = 0x60, .digital_config = 0x0C, .clk1 = 0x02, .clk2 = 0x26, .adc_en = 0x00, .adc_gain = {0x00, 0x00, 0x00, 0x00}}

/**
 * @brief D_SYS_CFG:FIXED=0,CRC_EN=0; CLK1:CLK_DIV[2:0]=1; CLK2:ICLK_DIV[2:0]=1; CLK2:OSR[3:0]=3
 *        动态帧模式, 关闭CRC输出, f_iclk = f_clkin / 2, f_mod = f_iclk / 2, f_data = f_mod / 800 = 5KHz
 */
#define ADS131A_CONFIGURE_DEFAULT_5KHZ()           {.analog_config = 0x60, .digital_config = 0x0C, .clk1 = 0x02, .clk2 = 0x23, .adc_en = 0x00, .adc_gain = {0x00, 0x00, 0x00, 0x00}}

/**
 * @brief D_SYS_CFG:FIXED=0,CRC_EN=0; CLK1:CLK_DIV[2:0]=1; CLK2:ICLK_DIV[2:0]=1; CLK2:OSR[3:0]=9
 *        动态帧模式, 关闭CRC输出, f_iclk = f_clkin / 2, f_mod = f_iclk / 2, f_data = f_mod / 200 = 20KHz
 */
#define ADS131A_CONFIGURE_DEFAULT_20KHZ()           {.analog_config = 0x60, .digital_config = 0x0C, .clk1 = 0x02, .clk2 = 0x29, .adc_en = 0x00, .adc_gain = {0x00, 0x00, 0x00, 0x00}}


/**
 * @brief D_SYS_CFG:FIXED=1,CRC_EN=1; CLK1:CLK_DIV[2:0]=1; CLK2:ICLK_DIV[2:0]=1; CLK2:OSR[3:0]=6
 *        固定帧模式, 使能CRC输出, f_iclk = f_clkin / 2, f_mod = f_iclk / 2, f_data = f_mod / 400 = 10KHz
 */
#define ADS131A_CONFIGURE_FIXED_CRCEN_10KHZ()           {.analog_config = 0x60, .digital_config = 0x3C, .clk1 = 0x02, .clk2 = 0x26, .adc_en = 0x00, .adc_gain = {0x00, 0x00, 0x00, 0x00}}

/**
 * @brief D_SYS_CFG:FIXED=1,CRC_EN=1; CLK1:CLK_DIV[2:0]=1; CLK2:ICLK_DIV[2:0]=1; CLK2:OSR[3:0]=3
 *        固定帧模式, 使能CRC输出, f_iclk = f_clkin / 2, f_mod = f_iclk / 2, f_data = f_mod / 800 = 5KHz
 */
#define ADS131A_CONFIGURE_FIXED_CRCEN_5KHZ()           {.analog_config = 0x60, .digital_config = 0x3C, .clk1 = 0x02, .clk2 = 0x23, .adc_en = 0x00, .adc_gain = {0x00, 0x00, 0x00, 0x00}}

/**
 * @brief D_SYS_CFG:FIXED=1,CRC_EN=1; CLK1:CLK_DIV[2:0]=1; CLK2:ICLK_DIV[2:0]=1; CLK2:OSR[3:0]=9
 *        固定帧模式, 使能CRC输出, f_iclk = f_clkin / 2, f_mod = f_iclk / 2, f_data = f_mod / 200 = 20KHz
 */
#define ADS131A_CONFIGURE_FIXED_CRCEN_20KHZ()           {.analog_config = 0x60, .digital_config = 0x3C, .clk1 = 0x02, .clk2 = 0x29, .adc_en = 0x00, .adc_gain = {0x00, 0x00, 0x00, 0x00}}

#endif

esp_err_t uart_create_uart_receive_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t uart_delete_uart_receive_task(void);

void twai_t_action(uint8_t action, uint32_t data);

esp_err_t twai_create_twai_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t twai_delete_twai_task();

esp_err_t twai_create_transmit_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t twai_delete_transmit_task();

esp_err_t twai_create_receive_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t twai_delete_receive_task();

void twai_wait_for_initializer(void);

#if(CONFIG_ORSYSTEM_MODE)
/**************************************************************************************************/


esp_err_t ads131a_create_configure_task(int priority, uint32_t stack_size, void* ctx);

esp_err_t ads131a_delete_configure_task(void);

typedef union{
    struct{
        uint32_t action : 8;
        uint32_t reserved : 8;
        uint32_t data : 8;
        uint32_t address : 8;
    };
    uint32_t val;
}u32_ads_action_t;
#endif


#if(CONFIG_ORSYSTEM_MODE)
typedef struct asd_data_ptr_s{
    uint8_t *ptr_isr;
    uint8_t *ptr;
    uint32_t ptr_size;
    uint32_t index;
    uint32_t data_length;
    portMUX_TYPE mux;

}ads_data_ptr_t;

typedef struct ads_data_ptr_s * ads_data_ptr_handle_t;


extern ads_data_ptr_t ads_data_ptr;

esp_err_t ads131a_t_action(uint32_t u32);

void ads131a_wait_for_configure_complete(void);

/**
 * 创建读取采样数据任务, 启动后会阻塞等待 DRDY 中断响应服务函数发送通知
 * @param priority
 * @param stack_size
 * @param ctx
 * @return
 */
esp_err_t ads131a_create_data_task(int priority, uint32_t stack_size, void* ctx);

esp_err_t ads131a_delete_data_task();

#else

typedef struct {
    mcpwm_config_t mcpwmConfig;
    uint32_t pulse_number;
}mcpwm_param_t;

esp_err_t mcpwm_create_mcpwm_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t mcpwm_delete_mcpwm_task(void);

void mcpwm_start_output(void);

esp_err_t gpio_create_gpio_task(int priority, uint32_t stack_size, void* ctx);
esp_err_t gpio_delete_gpio_task(void);

#endif

#endif