/**
 * @Author: your name
 * @Date: 2022-03-24 13:22:37
 * @LastEditTime: 2022-04-05 07:37:48
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \orsystem\main\orsystem_control.c
 **/




#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "esp_crc.h"
#include "esp_heap_caps.h"

#include "orsystem_task.h"
#include "twai_ota.h"



const static char TAG[] = "_ORSYSTEM_TASK_C";

#if(CONFIG_ORSYSTEM_MODE)
typedef union{
    struct{
        uint8_t start_address : 5;
        uint8_t reg_num : 3;
    };
    uint8_t val;
}u8_register_configure_format_t;

typedef union{
    struct{
        uint8_t ch1 : 1;
        uint8_t ch2 : 1;
        uint8_t ch3 : 1;
        uint8_t ch4 : 1;
    };
    uint8_t val;
}u8_req_analog_data_format_t;
#endif

enum {
    CMD_CLASS_ADS = 1,
    CMD_CLASS_TWAI,
    CMD_CLASS_MCPWM
};
#if(CONFIG_ORSYSTEM_MODE)
typedef enum{
    ADS_PIN_RESET,
    ADS_RESET,
    ADS_STANDBY,
    ADS_WAKEUP,
    ADS_LOCK,
    ADS_UNLOCK,
    ADS_STARTUP,
    ADS_PWRON,
    ADS_PWROFF,
    ADS_CONFIGURE,
    ADS_INITIALIZER,
    ADS_CTRL_START,
    ADS_CTRL_STOP
}ads131a_action_t;
#endif
enum {
    TWAI_CTRL_SELF_TEST = 1,
    TWAI_CTRL_INIT,
    TWAI_CTRL_ALERT
};
#if(CONFIG_ORSYSTEM_MODE == 0)
enum {
    MCPWM_CTRL_ACTION_START_READY = 1,
    MCPWM_CTRL_ACTION_STOP,
    MCPWM_CTRL_ACTION_SET_FREQUENCY,
    MCPWM_CTRL_ACTION_SET_DUTY,
    MCPWM_CTRL_ACTION_SET_PULSE_NUMBER,
};
#endif

typedef struct u64_cmd_action_s {
    union {
        struct {
            uint32_t cmd_class : 4;
            uint32_t cmd_type : 4;
            uint32_t cmd : 8;
#if(CONFIG_ORSYSTEM_MODE)
            uint32_t bit16_31 : 16;
#else
            uint32_t channel : 3;
            uint32_t enable : 1;
            uint32_t mcpwm_index : 1;
            uint32_t bit21_31 : 11;
#endif
        };
        uint32_t val;
    }action;
    uint32_t data;
}u64_cmd_action_t;
typedef struct u64_cmd_action_s * u64_cmd_action_handle_t;

typedef struct{
    twai_timing_config_t timing;
    portMUX_TYPE mux;
}twai_timing_config_content_t;

enum {
    TWAI_TRANS_CTRL_PING = 1,
    TWAI_TRANS_CTRL_PING_RESPONSE,
    TWAI_TRANS_CTRL_RETURN_REGISTER_DATA,
    TWAI_TRANS_CTRL_RETURN_ANALOG_DATA,
};
typedef union{
    struct{
        uint32_t action : 8;
        uint32_t reserved : 8;
        uint32_t data: 16;
    };
    uint32_t val;
}u32_twai_trans_ctrl_t;

static TaskHandle_t twai_transmit_task_handle = NULL;
static TaskHandle_t twai_receive_task_handle = NULL;
static TaskHandle_t twai_task_handle = NULL;
static TaskHandle_t uart_receive_task_handle = NULL;

static TaskHandle_t gpio_task_handle  = NULL;
static TaskHandle_t mcpwm_task_handle = NULL;

#define TWAI_ID_QUEUE_LENGTH                    16
static QueueHandle_t twai_transmit_control_queue = NULL;
static QueueHandle_t twai_ctrl_queue = NULL;
static twai_timing_config_content_t twai_timing = {.mux = portMUX_INITIALIZER_UNLOCKED, .timing = TWAI_TIMING_CONFIG_125KBITS()};


#define TWAI_EVENT_INIT_BIT                             BIT(0)
#define TWAI_EVENT_TRANS_BIT                            BIT(1)
#define TWAI_EVENT_RECE_BIT                             BIT(2)
#define TWAI_EVENT_OK_BIT                               BIT(3)
#define TWAI_EVENT_SELF_PASS_BIT                        BIT(4)
static EventGroupHandle_t twai_event_handle = NULL;


// typedef struct uart_data_frame_s{
//     uint32_t identifier;

//     uint8_t data_length_code;
//     uint8_t data[TWAI_FRAME_MAX_DLC];
// }uart_data_frame_t;
typedef twai_message_t * uart_data_frame_handle_t;

#define UART_CTRL_CHANGE_BR_BIT                         BIT(0)
static EventGroupHandle_t uart_event_handle = NULL;

EventGroupHandle_t orsys_event_handle = NULL;

#if(CONFIG_ORSYSTEM_MODE)
static QueueHandle_t     ads_action_queue = NULL;  // ADS131A 配置同步

static TaskHandle_t      ads_configure_task_handle = NULL;
static TaskHandle_t      ads_action_task_handle = NULL;
static TaskHandle_t      ads_data_task_handle = NULL;

#define ADS_EVENT_DATA_TASK_START_BIT                      BIT(0)
#define ADS_EVENT_READY_BIT                                BIT(1)
#define ADS_EVENT_START_BIT                                BIT(2)
#define ADS_EVENT_STOP_BIT                                 BIT(3)
#define ADS_EVENT_STARTUP_BIT                              BIT(4)
#define ADS_EVENT_CONFIG_CPLT_BIT                          BIT(5)
#define ADS_EVENT_DRDY_BIT                                 BIT(6)
#define ADS_EVENT_SAVE_DATA                                BIT(7)
static EventGroupHandle_t ads_event_group_handle = NULL;

#ifndef CONFIG_ADS_DATA_STATIC_BUFFER_SIZE
#define CONFIG_ADS_DATA_STATIC_BUFFER_SIZE                           (4 * 4 * 5000)
#endif
ads_data_ptr_t ads_data_ptr = {
       .ptr_isr = NULL,
       .ptr = NULL,
       .index = 0,
       .ptr_size = CONFIG_ADS_DATA_STATIC_BUFFER_SIZE,
       .mux = portMUX_INITIALIZER_UNLOCKED
};
static FILE *ads_data_cache_file = NULL;
// static ads131a_device_t ads131a = {.dev = ADS131A_M0_M1_M2_DEVICE(1, 1, 0), .vref = 2500, .state.val = 0x04};
static ads131a_device_handle_t ads131a = &orsystem_configure.analog.ads131a_dev;

static StreamBufferHandle_t data_streambuffer_handle = NULL;

#else

#define MCPWM_EVENT_START_BIT           BIT(0)
#define MCPWM_EVENT_STOP_BIT            BIT(1)
#define MCPWM_EVENT_READY_BIT           BIT(2)
#define MCPWM_EVENT_SET_FRERQUENCY_BIT  BIT(3)
#define MCPWM_EVENT_SET_DUTY_BIT        BIT(4)
#define MCPWM_EVENT_SET_PULSE_BIT       BIT(5)


static EventGroupHandle_t mcpwm_event_handle = NULL;
static QueueHandle_t mcpwm_action_queue_handle = NULL;
// static const mcpwm_dev_t *MCPWM_DEV[2] = {&MCPWM0, &MCPWM1};

// typedef struct {
//     uint32_t mcpwm;
//     uint8_t channel;
//     int8_t start;
//     bool stop;
//     float duty;
//     uint32_t frequency;
// //    ll_mcpwm_config_t config;
// }mcpwm_control_t;

typedef struct mcpwm_pulse_s{
    int32_t pulse_number;
    uint32_t counter;
}mcpwm_pulse_t;


static mcpwm_pulse_t pulse = {.counter = 0, .pulse_number = 3};



#endif


#if(CONFIG_ORSYSTEM_MODE)

//static volatile uint8_t ads_data_static_buffer[CONFIG_ADS_DATA_STATIC_BUFFER_SIZE];


static esp_err_t ads_data_ptr_malloc(ads_data_ptr_t *ptr_t, size_t length){
    esp_err_t err = ESP_OK;

    taskENTER_CRITICAL(&ptr_t->mux);
    if(!ptr_t){
        err = ESP_FAIL;
        goto end_section;
    }
    
    if(ptr_t->ptr == NULL) ptr_t->ptr = (uint8_t*)heap_caps_malloc(length, MALLOC_CAP_8BIT);
    if(!ptr_t->ptr){
        err = ESP_FAIL;
        goto end_section;
    }
    if(ptr_t->ptr_isr == NULL) ptr_t->ptr_isr = (uint8_t*)heap_caps_malloc(length, MALLOC_CAP_8BIT);
    if(!ptr_t->ptr_isr){
        free(ptr_t->ptr);
        ptr_t->ptr = NULL;
        err = ESP_FAIL;
        goto end_section;
    }
    ptr_t->index = 0;
    ptr_t->ptr_size = length;
end_section:
    taskEXIT_CRITICAL(&ptr_t->mux);
    return err;
}

static void ads_data_ptr_free(ads_data_ptr_t *ptr_t){
    if(!ptr_t){
       return;
    }
    taskENTER_CRITICAL(&ptr_t->mux);
    if(ptr_t->ptr) free(ptr_t->ptr);
    if(ptr_t->ptr_isr) free(ptr_t->ptr_isr);
    ptr_t->index = 0;
    ptr_t->data_length = 0;
    ptr_t->ptr = NULL;
    ptr_t->ptr_isr = NULL;
    taskEXIT_CRITICAL(&ptr_t->mux);
}

static void ads_data_ptr_swap_from_isr(ads_data_ptr_t *ptr_t){
    uint8_t *tmp_ptr = ptr_t->ptr_isr;
    ptr_t->ptr_isr = ptr_t->ptr;
    ptr_t->ptr = tmp_ptr;
    ptr_t->data_length = ptr_t->index;
    ptr_t->index = 0;
}


static bool ads_data_ptr_write_from_isr(ads_data_ptr_t *ptr_t, const void* data, uint32_t length){
    uint32_t remain = 0, len = 0;
    bool swap = false;
    if((ptr_t->ptr == NULL) || (ptr_t->ptr_isr == NULL)) return swap; 
    if((ptr_t->index + length) <= ptr_t->ptr_size){
        remain = 0;
        len = length;
        if(!(ptr_t->ptr_size - ptr_t->index - length)) swap = true; /// 空间将满
    }else{   /// 数据长度大于剩余空间
        len = ptr_t->ptr_size - ptr_t->index;
        remain = length - len;
        swap = true;
    }
    memcpy((void*)(ptr_t->ptr_isr + ptr_t->index), data, len);
    ptr_t->index += len;
    if(swap){
        ads_data_ptr_swap_from_isr(ptr_t);
    }
    if(remain){
        memcpy((void*)(ptr_t->ptr_isr), &data[len], remain);
        ptr_t->index += remain;
    }
    return swap;
}

static uint8_t* ads_data_ptr_get(ads_data_ptr_t *ptr_t){
    return ptr_t->ptr;
}

static void ads_start_response_drdy(void){
    gpio_intr_enable(IO_ADS_ADRDY);
    xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_START_BIT);
    xEventGroupClearBits(ads_event_group_handle, ADS_EVENT_STOP_BIT | ADS_EVENT_READY_BIT);
}

static void ads_start_ready(void){

}

static void ads_stop_response_drdy(void){
    gpio_intr_disable(IO_ADS_ADRDY);
    xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_STOP_BIT);
    xEventGroupClearBits(ads_event_group_handle, ADS_EVENT_START_BIT | ADS_EVENT_READY_BIT);
}

#endif

static void orsystem_config_write(orsys_config_reg_t * orsys_config, const u64_orsys_reg_config_format_handle_t data){
    size_t start_addr = (size_t)(orsys_config);
    size_t uart_addr = (size_t)(&orsys_config->uart);
    size_t can_addr = (size_t)(&orsys_config->can);

    
    
    if(data->base_addr == (uart_addr - start_addr)){
        int br = (int)data->data;

        // uart_config_t uart_config = {
        //     .baud_rate = br,
        //     .data_bits = UART_DATA_8_BITS,
        //     .parity = UART_PARITY_DISABLE,
        //     .stop_bits = UART_STOP_BITS_1,
        //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //     .source_clk = UART_SCLK_APB
        // };

        taskENTER_CRITICAL(&orsys_config->mux);
        *(int*)(uart_addr + data->offset) = br;
        taskEXIT_CRITICAL(&orsys_config->mux);
        xEventGroupSetBits(uart_event_handle, UART_CTRL_CHANGE_BR_BIT);
        return;
    }

    if(data->base_addr == (can_addr - start_addr)){
        u32_twai_timing_config_format_handle timing_data = (u32_twai_timing_config_format_handle)(&data->data);
        twai_timing_config_t timing = {
            .brp = (uint32_t)timing_data->brp,
            .tseg_1 = (uint8_t)timing_data->tseg_1,
            .tseg_2 = (uint8_t)timing_data->tseg_2,
            .sjw = (uint8_t)timing_data->sjw,
            .triple_sampling = (bool)timing_data->triple_sampling,
        };

        taskENTER_CRITICAL(&orsys_config->mux);
        orsys_config->can.timing = timing;
        taskEXIT_CRITICAL(&orsys_config->mux);
        twai_t_action(TWAI_CTRL_INIT, 0);
        return;
    }
#if(CONFIG_ORSYSTEM_MODE)
    size_t analog_addr = (size_t)(&orsys_config->analog);
    if(data->base_addr == (analog_addr - start_addr)){    /// 模拟采集设置
        if(data->offset < sizeof(ads131a_device_t)){
            u32_ads_action_t action = {.action = ADS_CONFIGURE};
            action.address = data->offset;
            action.data = data->data;
            
            taskENTER_CRITICAL(&orsys_config->mux);
            *(uint8_t*)(&orsys_config->analog.ads131a_dev + data->offset) = (uint8_t)data->data;
            taskEXIT_CRITICAL(&orsys_config->mux);

            ads131a_t_action(action.val);
        }else{
            switch(data->offset - sizeof(ads131a_device_t))
            {
                case 0:
                    taskENTER_CRITICAL(&orsys_config->mux);
                    orsystem_configure.analog.atime = (int32_t)data->data;
                    taskEXIT_CRITICAL(&orsys_config->mux);
                break;
                default:
                break;
            }
        }
        return;
    }
#else
    size_t mcpwm_addr = (size_t)(&orsystem_configure.mcpwm);
    if(data->base_addr == (mcpwm_addr - start_addr)){
        taskENTER_CRITICAL(&orsys_config->mux);
        *(uint32_t*)(mcpwm_addr + data->offset) = data->data;
        taskEXIT_CRITICAL(&orsys_config->mux);
        return;
    }
#endif
}

static void twai_receive_data_parse(const twai_message_t *r_message){
    if(r_message->rtr){   // 远程帧
        // ESP_LOGI(TAG, "receive remote frame[%x]\r\n", r_message.identifier);
        if(r_message->identifier == ID_REMOTE_PING){    // 远程PING
            u32_twai_trans_ctrl_t tmp = {.action = TWAI_TRANS_CTRL_PING_RESPONSE};
            xQueueSend(twai_transmit_control_queue, &tmp.val, portMAX_DELAY);
            return;
        }
        if(r_message->identifier == ID_REMOTE_SYNCHRONOUS_START){   // 接受到同步信号, 开始采样
        #if(CONFIG_ORSYSTEM_MODE)
            // ads131a_t_action(ADS_CTRL_START);
            // gpio_intr_enable(IO_ADS_ADRDY);
            ads_start_response_drdy();
        #else
            // io_mcpwm_cs(0);
            // mcpwm_ll_timer_start(&MCPWM0, MCPWM_TIMER_0);
            // xEventGroupSetBits(mcpwm_event_handle, MCPWM_EVENT_START_BIT);
            // xEventGroupClearBits(mcpwm_event_handle, MCPWM_EVENT_STOP_BIT | MCPWM_EVENT_READY_BIT);
            mcpwm_start_output();
            ESP_LOGI(TAG, "mcpwm start output\r\n");
        #endif
            return;
        }
        if(r_message->identifier == ID_REMOTE_SYNCHRONOUS_STOP){   // 接受到同步信号, 停止采样
        #if(CONFIG_ORSYSTEM_MODE)
            // gpio_intr_disable(IO_ADS_ADRDY);
            // ads131a_t_action(ADS_CTRL_STOP);
            ads_stop_response_drdy();
        #else
            // io_mcpwm_cs(1);
            ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
            ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
            mcpwm_ll_timer_stop(&MCPWM0, MCPWM_TIMER_0);
            ESP_LOGI(TAG, "mcpwm stop output\r\n");
            xEventGroupSetBits(mcpwm_event_handle, MCPWM_EVENT_STOP_BIT);
            xEventGroupClearBits(mcpwm_event_handle, MCPWM_EVENT_START_BIT | MCPWM_EVENT_READY_BIT);
        #endif
            return;
        }
        if(r_message->identifier == ID_REMOTE_REQUEST_ALL_ANALOG_DATA){
            u32_twai_trans_ctrl_t tmp = {.action = ID_REMOTE_REQUEST_ALL_ANALOG_DATA};
            xQueueSend(twai_transmit_control_queue, &tmp.val, portMAX_DELAY);
            return;
        }
        return;
    }else{   // 数据帧
        if(r_message->identifier == ID_DATA_PING_DATA){
            
            return;
        }
        if(r_message->identifier == ID_DATA_CONFIGURE_REGISTER_DATA){  /// 接受到配置数据
            const u64_orsys_reg_config_format_handle_t data = (u64_orsys_reg_config_format_handle_t)r_message->data;
            orsystem_config_write(&orsystem_configure, data);
            return;
        }
        if(r_message->identifier == ID_DATA_COMMAND){   /// 命令
            u64_cmd_action_handle_t cmd = (u64_cmd_action_handle_t)r_message->data;
            ESP_LOGD(TAG, "action:%#x, data:%#x\r\n", cmd->action.val, cmd->data);
#if(CONFIG_ORSYSTEM_MODE)
            if(cmd->action.cmd_class == CMD_CLASS_ADS){   /// ADS131A
                ads131a_t_action(cmd->action.cmd);
            }
#else
            if(cmd->action.cmd_class == CMD_CLASS_MCPWM){
                if(mcpwm_action_queue_handle) xQueueSend(mcpwm_action_queue_handle, cmd, portMAX_DELAY);
            }
#endif
            if(cmd->action.cmd_class == CMD_CLASS_TWAI){   /// TWAI
                if(twai_ctrl_queue) xQueueSend(twai_ctrl_queue, cmd, 0);
            }
            return;
        }
#if CONFIG_ORSYSTEM_MODE
        if(r_message->identifier == ID_DATA_REQUEST_REGISTER_DATA){    // 请求ADS配置数据
            const u64_orsys_reg_config_format_handle_t data = (u64_orsys_reg_config_format_handle_t)r_message->data;
            size_t start_addr = (size_t)(&orsystem_configure);
            size_t analog_addr = (size_t)(&orsystem_configure.analog);

            if(data->base_addr == (analog_addr - start_addr)){
                u32_twai_trans_ctrl_t tmp = {.val = 0};
                u8_register_configure_format_t u8 = {.val = 0};
                u8.start_address = data->offset;
                u8.reg_num = 1;
                tmp.action = TWAI_TRANS_CTRL_RETURN_REGISTER_DATA;
                tmp.data = u8.val;  /// 第一个字节指明开始地址和请求寄存器个数
                xQueueSend((QueueHandle_t)twai_transmit_control_queue, &tmp.val, portMAX_DELAY);
            }
            return;
        }
        if(r_message->identifier == ID_DATA_REQUEST_ANALOG_DATA){
            u32_twai_trans_ctrl_t tmp = {.val = 0};
            tmp.action = TWAI_TRANS_CTRL_RETURN_ANALOG_DATA;
            tmp.data = r_message->data[0];   /// 第一字节指明请求通道
            xQueueSend((QueueHandle_t)twai_transmit_control_queue, &tmp.val, portMAX_DELAY);
            return;
        }
#else

#endif
        if(r_message->identifier == ID_DATA_OTA){
            u64_twai_ota_data_header_handle handle = (u64_twai_ota_data_header_handle)r_message->data;
            if(handle->id != CONFIG_ORSYSTEM_ID) return; /// ID 检查

            xEventGroupSetBits(orsys_event_handle, ORSYS_EVENT_OTA_BIT);
            ota_create_ota_task(15, 4096, NULL);
            ota_wait_ota_complete();
            xEventGroupClearBits(orsys_event_handle, ORSYS_EVENT_OTA_BIT);
            ESP_LOGI(TAG, "OTA END\r\n");
            esp_restart();
            return;
        }
   }
}

#if(CONFIG_ORSYSTEM_MODE)
_Noreturn static void ads131a_configure_task(void* ctx){

    // orsys_config_reg_handle_t orsys_config = (orsys_config_reg_handle_t)ctx;
    ESP_LOGD(TAG, "ads131a_configure_task(void* ctx): ctx = %#X\r\n", (uint32_t)ctx);
    BaseType_t xReturn = pdFAIL;
    ads131a_action_t action = ADS_STARTUP;
    
    if(0){
        esp_err_t err = ESP_FAIL;
        ads131a_device_t device;
        err = nvs_read_ads_device(&device, sizeof(ads131a_device_t));
        if(err == ESP_OK){ // 存在配置
            // taskENTER_CRITICAL(&orsystem_configure.mux);
            orsystem_configure.analog.ads131a_dev = device;
            // taskEXIT_CRITICAL(&orsystem_configure.mux);
        }else{
            ESP_LOGE(TAG, "can't read saved device configure!\r\n");
        }
        orsystem_configure.analog.ads131a_dev.state.val = 0xFF;
    }
    ads131a_t_action(ADS_STARTUP);

    orsystem_configure.analog.ads131a_dev.state.val = 0x0F;
    orsystem_configure.analog.ads131a_dev.dev.clk1.clk_div = CLK_DIV(2); /// 设置 ADC 输入时钟分频
    orsystem_configure.analog.ads131a_dev.dev.clk2.iclk_div = CLK_DIV(2); /// 设置 ADC 调制时钟分频
    orsystem_configure.analog.ads131a_dev.dev.clk2.osr = CLK2_OSR_400; /// 设置 过采样因子
    orsystem_configure.analog.ads131a_dev.dev.adc_ena.ena = 0x00;
    // taskEXIT_CRITICAL(&orsystem_configure.mux);

    ads131a_t_action(ADS_INITIALIZER);
    // 使能 ADC

    // 唤醒 device

    // 待机模式
    ads131a_t_action(ADS_STANDBY);
    // 锁定设备
    xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_STOP_BIT);
    while(true){
        u32_ads_action_t tmp = {.val = 0};
        xReturn = xQueueReceive(ads_action_queue, &tmp.val, portMAX_DELAY);
        if(xReturn != pdTRUE) continue;
        action = tmp.action;
        if(action == ADS_PIN_RESET){
            io_ads131a_reset(0);
            vTaskDelay(pdMS_TO_TICKS(100));
            io_ads131a_reset(1);               // 复位
            vTaskDelay(pdMS_TO_TICKS(100));             // wait for POR complete
        }
        if(action == ADS_RESET){
            uint16_t response = ads131a_opc_reset(&orsystem_configure.analog.ads131a_dev);
            ESP_LOGI(TAG, "device reset[%#x]!\r\n", response);
            continue;
        }
        if(action == ADS_STANDBY){
            uint16_t response = ads131a_opc_standby(&orsystem_configure.analog.ads131a_dev);
            if(response == OPCODE_STANDBY) ESP_LOGI(TAG, "device standby!\r\n");
            continue;
        }
        if(action == ADS_WAKEUP){
           ads131a_adcen(&orsystem_configure.analog.ads131a_dev);
            uint16_t response = ads131a_opc_wakeup(&orsystem_configure.analog.ads131a_dev);
            if(response == OPCODE_WAKEUP) ESP_LOGI(TAG, "device wakeup!\r\n");
            continue;
        }
        if(action == ADS_LOCK){
            uint16_t response = ads131a_opc_lock(&orsystem_configure.analog.ads131a_dev);
            if(response == OPCODE_LOCK){
                ESP_LOGI(TAG, "device locked!\r\n");
            }
            nvs_save_orsystem_config(&orsystem_configure, sizeof(orsys_config_reg_t));
            continue;
        }
        if(action == ADS_UNLOCK){
            // taskENTER_CRITICAL(&orsystem_configure.mux);
            uint16_t response = ads131a_opc_unlock(&orsystem_configure.analog.ads131a_dev);
            // taskEXIT_CRITICAL(&orsystem_configure.mux);
            if(response == OPCODE_UNLOCK) ESP_LOGI(TAG, "device unlocked!\r\n");
            else ESP_LOGI(TAG, "device locked!");
            continue;
        }
        if(action == ADS_STARTUP){
            io_ads131a_reset(0);
            io_set_analog_power(1);  // 供电使能
            vTaskDelay(pdMS_TO_TICKS(100));
            io_ads131a_reset(1);               // 复位
            vTaskDelay(pdMS_TO_TICKS(100));             // wait for POR complete
            ESP_LOGI(TAG, "ads131a startup......\r\n");
            // taskENTER_CRITICAL(&orsystem_configure.mux);
            ads131a_startup(&orsystem_configure.analog.ads131a_dev);
            // taskEXIT_CRITICAL(&orsystem_configure.mux);
            continue;
        }
        if(action == ADS_PWRON){
            io_set_analog_power(1);
            ESP_LOGI(TAG, "ads131a power on......\r\n");
            continue;
        }
        if(action == ADS_PWROFF){
            ESP_LOGI(TAG, "ads131a power off......\r\n");
            io_set_analog_power(0);
            continue;
        }
        if(action == ADS_CONFIGURE){   /// CONFIGURE后面紧接着 (uint32_t)(address + data)
            uint32_t conf = tmp.data;
            uint8_t addr = 0, reg_value = 0;
            uint8_t err = 0;
            orsys_config_reg_t config = {0};

            addr = UPPER_BYTE(conf);
            reg_value = LOWER_BYTE(conf);
            ESP_LOGI(TAG, "configure ads131a device[address=%#x|data=%#x]!\r\n", addr, reg_value);

            taskENTER_CRITICAL(&orsystem_configure.mux);
            config = orsystem_configure;
            taskEXIT_CRITICAL(&orsystem_configure.mux);

            err = ads131a_write_register(&config.analog.ads131a_dev, addr, reg_value);
            if(err == reg_value){
                taskENTER_CRITICAL(&orsystem_configure.mux);
                config = orsystem_configure;
                taskEXIT_CRITICAL(&orsystem_configure.mux);
                nvs_save_orsystem_config(&config, sizeof(orsys_config_reg_t));
            }
            continue;
        }
        if(action == ADS_INITIALIZER){
            ESP_LOGI(TAG, "ads131a initialaizer......\r\n");
            // taskENTER_CRITICAL(&orsystem_configure.mux);
            ads131a_configure_device(&orsystem_configure.analog.ads131a_dev);
            // taskEXIT_CRITICAL(&orsystem_configure.mux);
            xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_CONFIG_CPLT_BIT);
            continue;
        }
        if(action == ADS_CTRL_START){    /// 初始化启动准备
            uint16_t response = 0;
            uint8_t div1 = 0, div2 = 0, ch = 0;
            uint16_t osr = 0, osr_num[] = ADS131A_OSR;
            int32_t atime = 0;
            size_t free_mem = 0, need_mem = 0;
            if(!(xEventGroupGetBits(ads_event_group_handle) & ADS_EVENT_STOP_BIT)) continue;  /// 防止重复调用
            gpio_intr_disable(IO_ADS_ADRDY);
            // ads_stop_response_drdy();

            taskENTER_CRITICAL(&orsystem_configure.mux);
            div1 = 2 * orsystem_configure.analog.ads131a_dev.dev.clk1.clk_div;
            div2 = 2 * orsystem_configure.analog.ads131a_dev.dev.clk2.iclk_div;
            osr = orsystem_configure.analog.ads131a_dev.dev.clk2.osr;
            atime = orsystem_configure.analog.atime;
            ch = ll_ads_get_device_channel(&orsystem_configure.analog.ads131a_dev);
            taskEXIT_CRITICAL(&orsystem_configure.mux);

            ESP_LOGI(TAG, "clk1:%d, clk2:%d, osr:%d, atime:%d\r\n", div1, div2, osr, atime);
            osr = osr_num[osr];
            free_mem = esp_get_minimum_free_heap_size();
            // ESP_LOGI(TAG, "free heap size:%d Byte\r\n", free_mem);
            heap_caps_print_heap_info(MALLOC_CAP_8BIT);

            free_mem -= 32 * 1024;  /// 保留 32KB内存给系统用
            free_mem >>= 1;       /// 剩余内存分两份

            need_mem = ((16000000 / div1 / div2 / osr) / 1000) * ch * 4 * (atime >> 1);
            need_mem = (need_mem > free_mem) ? free_mem : need_mem;
            ESP_LOGI(TAG, "need heap size:%d Byte x 2\r\n", need_mem);
            ads_data_ptr_free(&ads_data_ptr);    /// 释放 data cache 内存
            esp_err_t err = ads_data_ptr_malloc(&ads_data_ptr, need_mem);   /// 开辟 data cache 内存
            if(err != ESP_OK){
                ESP_LOGE(TAG, "data cache memery malloc fail!\r\n");
                continue;
            }

            spi_device_acquire_bus(vspi, portMAX_DELAY);   // 申请独占SPI总线, 防止出现总线竞争导致无法及时读取采样数据, 以及减少等待获取总线控制权时间, 提高响应速度
            /// 计算采样率
            xEventGroupWaitBits(ads_event_group_handle, ADS_EVENT_DATA_TASK_START_BIT, false, false, portMAX_DELAY);

            // taskENTER_CRITICAL(&orsystem_configure.mux);
            if(orsystem_configure.analog.ads131a_dev.state.lock) ads131a_opc_unlock(&orsystem_configure.analog.ads131a_dev);
            response = ads131a_adcen(&orsystem_configure.analog.ads131a_dev);
            // taskEXIT_CRITICAL(&orsystem_configure.mux);
            if(response != ADC_ENA_ENA_ALL_CH_PWUP){
                ESP_LOGE(TAG, "ADC channel enable fail!\r\n");
                // io_set_buzzer(0);
                // vTaskDelay(1000);
                // io_set_buzzer(0);
                spi_device_release_bus(vspi);
                continue;
            }else{
                ESP_LOGI(TAG, "ADC channel enabled!\r\n");
//                xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_START_BIT);
                // io_ads131a_set_drdy_intr(1);
            }
            // taskENTER_CRITICAL(&orsystem_configure.mux);
            response = ads131a_opc_wakeup(&orsystem_configure.analog.ads131a_dev);
            if(response == OPCODE_WAKEUP) ESP_LOGI(TAG, "device wakeup!\r\n");
            ads131a_opc_lock(&orsystem_configure.analog.ads131a_dev);
            // taskEXIT_CRITICAL(&orsystem_configure.mux);

            xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_READY_BIT);
            xEventGroupClearBits(ads_event_group_handle, ADS_EVENT_STOP_BIT | ADS_EVENT_START_BIT);
            /// 一切就绪
            // gpio_intr_enable(IO_ADS_ADRDY);   /// 启用中断要在同步信号回调中启用
            continue;
        }
        if(action == ADS_CTRL_STOP){
            uint16_t response = 0;
            if(!(xEventGroupGetBits(ads_event_group_handle) & ADS_EVENT_START_BIT)) continue;    /// 防止重复调用
            ads_stop_response_drdy();
#if(1)
            // taskENTER_CRITICAL(&orsystem_configure.mux);
            if(orsystem_configure.analog.ads131a_dev.state.lock) ads131a_opc_unlock(&orsystem_configure.analog.ads131a_dev);
            response = ads131a_adcoff(&orsystem_configure.analog.ads131a_dev);
            if(response != ADC_ENA_ENA_ALL_CH_PWDN){
                ESP_LOGE(TAG, "ADC channel disable fail!\r\n");
            }else{
                ESP_LOGI(TAG, "ADC channel disabled!\r\n");
            }
            response = ads131a_opc_standby(&orsystem_configure.analog.ads131a_dev);
            // taskEXIT_CRITICAL(&orsystem_configure.mux);
            if(response == OPCODE_STANDBY) ESP_LOGI(TAG, "device standby!\r\n");
#endif
            spi_device_release_bus(vspi);        // 释放 SPI 总线控制权

            xTaskGenericNotify(ads_data_task_handle, 0, eSetValueWithOverwrite, NULL);   /// 清除
            xTaskNotifyStateClear(ads_data_task_handle);
            continue;
        }
    }

    vTaskDelete(NULL);
    ads_action_task_handle = NULL;
}

esp_err_t ads131a_create_configure_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;

    if(ads_action_queue == NULL){
        ads_action_queue = xQueueCreate(16, sizeof(uint32_t));
    }
    if(ads_event_group_handle == NULL){
        ads_event_group_handle = xEventGroupCreate();
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
    return ESP_OK;
}

void ads131a_wait_for_configure_complete(void){
    EventBits_t ubit = 0;
    while(!(ubit & ADS_EVENT_CONFIG_CPLT_BIT)){
        ubit = xEventGroupWaitBits(ads_event_group_handle, ADS_EVENT_CONFIG_CPLT_BIT, true, false, portMAX_DELAY);
    }
}

esp_err_t ads131a_t_action(uint32_t u32){
    uint32_t tmp = u32;
    BaseType_t xReturn = pdFAIL;
    esp_err_t err = ESP_OK;
    xReturn = xQueueSend(ads_action_queue, &tmp, pdMS_TO_TICKS(3000));
    if(xReturn != pdTRUE){
        err = ESP_FAIL;
    }
    return err;
}


static void ads131a_drdy_fault(void){

}

static void IRAM_ATTR ads131a_drdy_callback(void* ctx){
    BaseType_t token = pdFAIL;
//    EventGroupHandle_t event_handle = (EventGroupHandle_t)ctx;
#if(0)
    if(ads_event_group_handle){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE, xReturn;
        xReturn = xEventGroupSetBitsFromISR(ads_event_group_handle, ADS_EVENT_DRDY_BIT, &xHigherPriorityTaskWoken);
//        if(xReturn == pdPASS){
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        }
    }
#elif(0)
    if(ads_data_task_handle){
        vTaskNotifyGiveFromISR(ads_data_task_handle, &token);
        portYIELD_FROM_ISR(token);  // token == true, 则执行上下文切换
    }
#elif(0)
    ads131a_udata_t data = {0};
    uint16_t response = 0;
    response = ads131a_get_channel_udata((ads131a_device_t*)&orsystem_configure.analog.ads131a_dev, &data);
    if(!(response & STAT_1_F_DRDY_MASK)){     // 没有发生错误
        // 发送数据到数据流缓冲区
        BaseType_t token = pdFALSE;
        xStreamBufferSendFromISR(data_streambuffer_handle, (void*)data.channel_data, sizeof(data.channel_data) * sizeof(uint32_t), &token);
        portYIELD_FROM_ISR(&token);
    }else{
//        ads131a_drdy_fault();
    }
#elif(0)
     ads131a_idata_t idata = {0};
     uint16_t response = 0;
     response = ads131a_get_channel_idata((ads131a_device_t*)&orsystem_configure.analog.ads131a_dev, &idata);
     if(!(response & STAT_1_F_DRDY_MASK)){     // 没有发生错误
         // 发送数据到数据流缓冲区
         BaseType_t token = pdFALSE;
         xStreamBufferSendFromISR(data_streambuffer_handle, (void*)idata.channel_data, sizeof(idata.channel_data) * sizeof(int32_t), &token);
         portYIELD_FROM_ISR(&token);
     }else{
//        ads131a_drdy_fault();
     }
#elif(1)
     ads_data_ptr_t *ptr = (ads_data_ptr_t*)ctx;
     ads131a_idata_t idata = {0};
     uint16_t response = 0;
     bool swap = false;

     response = ads131a_get_channel_idata((ads131a_device_t*)&orsystem_configure.analog.ads131a_dev, &idata);
     if((response & STAT_1_F_DRDY_MASK)) return;
     swap = ads_data_ptr_write_from_isr(ptr, idata.channel_data, sizeof(idata.channel_data));
     if(swap){
        vTaskNotifyGiveFromISR(ads_data_task_handle, NULL);   /// 通知值自增
        // portYIELD_FROM_ISR(token);  // token == true, 则执行上下文切换
     }

#endif
}


_Noreturn static void ads131a_data_task(void* ctx){
    FILE *file = NULL;
    size_t file_limit_size = 1000, save_limit = 1;
    EventBits_t ubit = 0;
    BaseType_t xReturn = pdFALSE;
    ESP_LOGD(TAG, "ads131a_data_task(void* ctx): ctx = %#X\r\n", (uint32_t)ctx);

    if(1){
#if(0)
        char file_name[16];
        strcpy(file_name,  CONFIG_SPIFFS_STORAGE_BASE_PATH);
        strcat(file_name, "/tmp.bin");
        file = fopen(file_name, "w+b");
        if(file == NULL){
            ESP_LOGE(TAG, "data cache file open failed, will delete data task......\r\n");
            goto task_end_section;
        }else{
            ESP_LOGI(TAG, "data cache file open success\r\n");
        }
#endif
//        size_t total = 0, used = 0;
//        esp_spiffs_info(CONFIG_SPIFFS_STORATE_LABEL, &total, &used);
        // save_limit = ((total - used) >> 12);
        // save_limit = 100;
        // file_limit_size = save_limit << 12;
    }

    if(1){
        // uint32_t free_heap = esp_get_minimum_free_heap_size();
        // ESP_LOGI(TAG, "free heap size:%d Byte\r\n", free_heap);
        
        gpio_set_intr_type(IO_ADS_ADRDY, GPIO_INTR_NEGEDGE);
        gpio_set_direction(IO_ADS_ADRDY, GPIO_MODE_INPUT);
        gpio_set_pull_mode(IO_ADS_ADRDY, GPIO_PULLUP_ONLY);
        gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        gpio_isr_handler_add(IO_ADS_ADRDY, ads131a_drdy_callback, (void*)&ads_data_ptr);
        gpio_intr_disable(IO_ADS_ADRDY);
    }
    xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_STOP_BIT);
    xEventGroupSetBits(ads_event_group_handle, ADS_EVENT_DATA_TASK_START_BIT);
    while(true){
#if(1)
        uint32_t notify_value = 0;

        xReturn = xTaskNotifyWait(0, 0, &notify_value, portMAX_DELAY);
        if(xReturn != pdTRUE) continue;

        ESP_LOGI(TAG, "%d:%d\r\n", (int)notify_value, (int)ads_data_ptr.data_length);
        if(notify_value == 2){
            gpio_intr_disable(IO_ADS_ADRDY);  /// 关闭中断
            bool rt = false;
            taskENTER_CRITICAL(&orsystem_configure.mux);
            rt = orsystem_configure.analog.ctrl.data_return;
            taskEXIT_CRITICAL(&orsystem_configure.mux);
            if(!rt) goto stop_section;
            vTaskSuspend(twai_transmit_task_handle);  /// 挂起
            int32_t *ptr = NULL;
            ptr = (int32_t *)ads_data_ptr.ptr;
            twai_message_t t_message = {.data_length_code = 8, .extd = 0, .rtr = 0, .ss = 0};
            for(int i = 0; i < (ads_data_ptr.data_length / sizeof(int32_t));){
                for(int j = 0; j < 4; j++){
                    t_message.identifier = ID_DATA_CH1_ANALOG_DATA + j;
                    memcpy(&t_message.data[4], (void*)(ptr + i + j), sizeof(int32_t));
                    twai_transmit(&t_message, portMAX_DELAY);
                }
                i += 4;
            }
            ptr = (int32_t*)ads_data_ptr.ptr_isr;
            for(int i = 0; i < (ads_data_ptr.data_length / sizeof(int32_t));){
                for(int j = 0; j < 4; j++){
                    t_message.identifier = ID_DATA_CH1_ANALOG_DATA + j;
                    memcpy(&t_message.data[4], (void*)(ptr + i + j), sizeof(int32_t));
                    twai_transmit(&t_message, portMAX_DELAY);
                }
                i += 4;
            }
            vTaskResume(twai_transmit_task_handle);   /// 唤醒
    stop_section:
            ads131a_t_action(ADS_CTRL_STOP);
        }
#endif
    }
    // task_end_section:
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
#if(0)
    // 创建数据流缓冲区
    if(data_streambuffer_handle == NULL){
        // uint8_t i = 0, _tr_size = 4 * 4;
        uint32_t _s = 1024 * 4 * 4;
        uint32_t free_heap = 0;
        // free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
        free_heap = esp_get_minimum_free_heap_size();
        ESP_LOGI(TAG, "free heap size:%d Byte\r\n", free_heap);
        _s = (free_heap > (128 << 10)) ? (96 << 10) : ((free_heap > (96 << 10)) ? (80 << 10) : ((free_heap > (80 << 10)) ? (64 << 10) : ((free_heap > (64 << 10)) ? (48 << 10) : ((free_heap > (48 << 10)) ? (32 << 10) : ((free_heap > (32 << 10)) ? (16 << 10) : (16 << 10))))));
//        _s = 32 << 10;
        do{
            data_streambuffer_handle = xStreamBufferCreate(_s , CONFIG_ADS_DATA_STATIC_BUFFER_SIZE);
            if(data_streambuffer_handle == NULL) break;
        }while(data_streambuffer_handle == NULL);
        if(!data_streambuffer_handle){
            ESP_LOGE(TAG, "'ads131a_create_data_task' can not create stream buffer!\r\n");
            return ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "stream buffer size:%d byte\r\n", _s);
        }
    }
#endif
    if(ads_data_task_handle == NULL){
        // 在核心1创建任务
        xReturn = xTaskCreatePinnedToCore(ads131a_data_task, "ads131a_data_task", stack_size, ctx, priority, &ads_data_task_handle, 1);
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
    gpio_uninstall_isr_service();

    if(ads_data_task_handle){
        vTaskDelete(ads_data_task_handle);
        ads_data_task_handle = NULL;
    }
    xEventGroupClearBits(ads_event_group_handle, ADS_EVENT_DATA_TASK_START_BIT);
#if(0)
    if(ads_data_sem){
        vSemaphoreDelete(ads_data_sem);
        ads_data_sem = NULL;
    }
#endif
    return ESP_OK;
}

#endif

#if(CONFIG_ORSYSTEM_MODE == 0)
#include "hal/mcpwm_hal.h"
#include "ll_mcpwm.h"
#include "driver/timer.h"

void mcpwm_start_output(void){
    io_mcpwm_cs(0);
    mcpwm_ll_timer_start(&MCPWM0, MCPWM_TIMER_0);
    xEventGroupSetBits(mcpwm_event_handle, MCPWM_EVENT_START_BIT);
    xEventGroupClearBits(mcpwm_event_handle, MCPWM_EVENT_STOP_BIT | MCPWM_EVENT_READY_BIT);
    // ESP_LOGI(TAG, "mcpwm start output\r\n");
}

void mcpwm_t_action(uint8_t action, uint32_t data){
    u64_cmd_action_t tmp = {.action.cmd_class = CMD_CLASS_MCPWM};

    tmp.action.cmd = action;
    tmp.data = data;
    if(mcpwm_action_queue_handle) xQueueSend(mcpwm_action_queue_handle, &tmp, portMAX_DELAY);
}

/**
 *  定时事件优先级:
 *  1. 递增计数: 1(最高)-软件强制事件, 2-UTEP(计数器值等于周期值), 3-UT0, 4-UT1, 5-UTEB(计数器值等于比较寄存器B的值), 6-UTEA(计数器值等于比较寄存器A的值), 7(最低)-UTEZ(计数器值等于0)
 *  2. 递减计数: 1(最高)-软件强制事件, 2-DTEZ(计数器值等于0), 3-DT0, 4-DT1, 5-DTEB(计数器值等于比较寄存器B的值), 6-DTEA(计数器值等于比较寄存器A的值), 7(最低)-DTEP(计数器值等于周期值)
 *
 *  说明: UTEP和UTEZ不同时发生, 处于递增递减循环计数模式时, UTEP不会发生, DTEP和DTEZ不同时发生, 处于递增递减循环模式时, DTEZ不会发生
 *
 */


static void IRAM_ATTR mcpwm_isr_handler(void* ctx){
    uint32_t mcpwm_intr_status = 0;
    /**
     * @brief BIT   0      1      2     3     4     5     6     7     8      15      16     17     18     19     20
     *           TOSTOP T1STOP T2STOP T0TEZ T1TEZ T2TEZ T0TEP T1TEP T2TEP  OP0TEA OP1TEA OP2TEA OP0TEB OP1TEB OP2TEB
     */
    mcpwm_dev_t *mcpwm = &MCPWM0;
    mcpwm_intr_status = mcpwm->int_st.val;
    mcpwm_pulse_t *p = (mcpwm_pulse_t*)ctx;

    if((mcpwm_intr_status & MCPWM_INTR_T0_TEP_FLAG) && (p->pulse_number > 0)){   //
        p->counter += 1;
        if(p->counter >= p->pulse_number){
            ll_mcpwm_gen_set_cntu_force_output(mcpwm, 0, 0, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_AT_TEZ);
            ll_mcpwm_gen_set_cntu_force_output(mcpwm, 0, 1, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_AT_TEZ);
            mcpwm_ll_timer_stop(mcpwm, 0);
            ll_mcpwm_disable_intr(mcpwm, MCPWM_INTR_T0_TEP_FLAG);
        }
    }
    if(mcpwm_intr_status & MCPWM_INTR_T0_STOP_FLAG){  // 停止事件
        io_mcpwm_cs(1);
        p->counter = 0;
        xEventGroupSetBitsFromISR(mcpwm_event_handle, MCPWM_EVENT_STOP_BIT, NULL);
        xEventGroupClearBitsFromISR(mcpwm_event_handle, MCPWM_EVENT_START_BIT | MCPWM_EVENT_READY_BIT);
    }
    mcpwm_ll_clear_intr(mcpwm, mcpwm_intr_status);
}


_Noreturn static void mcpwm_task(void* ctx){
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, IO_PWMA_OUTPUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, IO_PWMB_OUTPUT);
    uint32_t freq = 950 * 2;
    uint32_t duty = 50;
    if(1){
        gpio_config_t io_conf;

        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pin_bit_mask = (1ULL << IO_PWM_CS);
        gpio_config(&io_conf);
        gpio_set_level(IO_PWM_CS, 1);

        taskENTER_CRITICAL(&orsystem_configure.mux);
        freq = orsystem_configure.mcpwm.frequency;
        duty = orsystem_configure.mcpwm.duty;
        pulse.pulse_number = orsystem_configure.mcpwm.pulse_num - 1;
        taskEXIT_CRITICAL(&orsystem_configure.mux);

        ll_mcpwm_config_t config = {
                .clk_div = 1,
                .timer_div = 1,
                .frequency = freq,
                .duty = {duty, 100 - duty},
                .cmpr_up = MCPWM_COMPARE_UP_AT_TEZ,
                .period_up = MCPWM_PERIOD_UP_AT_TEZ,
                .counter_mode = MCPWM_UP_DOWN_COUNTER,
                .at_zero_action = {MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_LOW},
                .at_cmpr_up_action = {MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_NO_CHANGE},
                .at_cmpr_dwon_action = {MCPWM_ACTION_NO_CHANGE, MCPWM_ACTION_FORCE_LOW},
                .at_period_action = {MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_HIGH},
        };
        ll_mcpwm_init(&MCPWM0, 0, &config);
    }

    ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);   /// 起始电平为低

    ll_mcpwm_enable_intr(&MCPWM0, MCPWM_INTR_T0_STOP_FLAG | MCPWM_INTR_T0_TEP_FLAG);
    mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_isr_handler, (void*)&pulse, ESP_INTR_FLAG_IRAM, NULL);

    xEventGroupSetBits(mcpwm_event_handle, MCPWM_EVENT_STOP_BIT);

    while(1){
        u64_cmd_action_t action;
        BaseType_t xReturn = xQueueReceive(mcpwm_action_queue_handle, &action, portMAX_DELAY);
        if((xReturn != pdTRUE) || (action.action.cmd_class != CMD_CLASS_MCPWM)) continue;
        if(action.action.cmd == MCPWM_CTRL_ACTION_START_READY){
            uint32_t freq_tmp = 1000;
            uint32_t duty_tmp = 50;
            bool stop = false;
            mcpwm_ll_timer_stop(&MCPWM0, 0);
            xEventGroupWaitBits(mcpwm_event_handle, MCPWM_EVENT_STOP_BIT, false, false, portMAX_DELAY);

            taskENTER_CRITICAL(&orsystem_configure.mux);
            freq_tmp = orsystem_configure.mcpwm.frequency;
            duty_tmp = orsystem_configure.mcpwm.duty;
            pulse.pulse_number = orsystem_configure.mcpwm.pulse_num;
            if(pulse.pulse_number == 0) stop = true;
            pulse.counter = 0;
            taskEXIT_CRITICAL(&orsystem_configure.mux);
            
            if(stop){
                u64_cmd_action_t tmp = {.action.cmd_class = CMD_CLASS_MCPWM, .action.cmd = MCPWM_CTRL_ACTION_STOP};
                xQueueSend(mcpwm_action_queue_handle, &tmp, portMAX_DELAY);
            }

            if(freq_tmp != freq){
                freq = freq_tmp;
                ll_mcpwm_set_frequency(&MCPWM0, 0, freq);
            } 
            if(duty != duty_tmp){
                duty = duty_tmp;
                if (duty == 0) {
                    ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
                    ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
                } else {
                    ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_DISABLE, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
                    ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_DISABLE, MCPWM_CNTUFORCE_UP_IMMEDIATELY);

                    ll_mcpwm_set_duty(&MCPWM0, 0, 0, (float) duty);
                    ll_mcpwm_set_duty(&MCPWM0, 0, 1, (float) (100 - duty));
                }
            }

            ll_mcpwm_enable_intr(&MCPWM0, MCPWM_INTR_T0_TEP_FLAG);
            ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_DISABLE, MCPWM_CNTUFORCE_UP_AT_TEZ);
            ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_DISABLE, MCPWM_CNTUFORCE_UP_AT_TEZ);
            
            xEventGroupSetBits(mcpwm_event_handle, MCPWM_EVENT_READY_BIT);
            xEventGroupClearBits(mcpwm_event_handle, MCPWM_EVENT_START_BIT | MCPWM_EVENT_STOP_BIT);
            // mcpwm_ll_timer_start(&MCPWM0, MCPWM_TIMER_0);
            // io_mcpwm_cs(0);
            ESP_LOGI(TAG, "mcpwm ready outout\r\n");
            continue;
        }
        if(action.action.cmd == MCPWM_CTRL_ACTION_STOP){
            ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
            ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
            mcpwm_ll_timer_stop(&MCPWM0, 0);
            continue;
        }
        if(action.action.cmd == MCPWM_CTRL_ACTION_SET_DUTY){
            orsys_config_reg_t or_config = { 0 };
#if(1)
            taskENTER_CRITICAL(&orsystem_configure.mux);
            duty = (uint32_t)action.data;
            orsystem_configure.mcpwm.duty = (uint32_t)action.data;
            or_config = orsystem_configure;
            taskEXIT_CRITICAL(&orsystem_configure.mux);
#else
            duty = (uint32_t)action.data;
#endif
            if (duty == 0) {
                ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
                ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_LOW, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
            } else {
                if(duty > 100) duty = 100;
                ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 0, MCPWM_FORCE_OUTPUT_DISABLE, MCPWM_CNTUFORCE_UP_IMMEDIATELY);
                ll_mcpwm_gen_set_cntu_force_output(&MCPWM0, 0, 1, MCPWM_FORCE_OUTPUT_DISABLE, MCPWM_CNTUFORCE_UP_IMMEDIATELY);

                ll_mcpwm_set_duty(&MCPWM0, 0, 0, (float) duty);
                ll_mcpwm_set_duty(&MCPWM0, 0, 1, (float) (100 - duty));
            }
            ESP_LOGI(TAG, "set mcpwm duty = %d\r\n", duty);
            nvs_save_orsystem_config(&or_config, sizeof(orsys_config_reg_t));
            continue;
        }
        if(action.action.cmd == MCPWM_CTRL_ACTION_SET_FREQUENCY){
            orsys_config_reg_t or_config = { 0 };
#if(1)
            taskENTER_CRITICAL(&orsystem_configure.mux);
            freq = (uint32_t)action.data;
            orsystem_configure.mcpwm.frequency = (uint32_t)action.data;
            or_config = orsystem_configure;
            taskEXIT_CRITICAL(&orsystem_configure.mux);
#else
            freq = (uint32_t)action.data;
#endif
            ll_mcpwm_set_frequency(&MCPWM0, 0, freq);
            nvs_save_orsystem_config(&or_config, sizeof(orsys_config_reg_t));
            ESP_LOGI(TAG, "set mcpwm frequency = %d\r\n", freq);
            continue;
        }
        if(action.action.cmd == MCPWM_CTRL_ACTION_SET_PULSE_NUMBER){
            orsys_config_reg_t or_config = { 0 };
#if(0)
            taskENTER_CRITICAL(&orsystem_configure.mux);
            pulse.pulse_number = orsystem_configure.mcpwm.pulse_num - 1;
            taskEXIT_CRITICAL(&orsystem_configure.mux);
#else
            bool stop = false;
            taskENTER_CRITICAL(&orsystem_configure.mux);
            pulse.pulse_number = (int32_t)action.data;
            orsystem_configure.mcpwm.pulse_num = (int32_t)action.data;
            or_config = orsystem_configure;
            if(pulse.pulse_number == 0) stop = true;
            taskEXIT_CRITICAL(&orsystem_configure.mux);
            if(stop){
                u64_cmd_action_t tmp = {.action.cmd_class = CMD_CLASS_MCPWM, .action.cmd = MCPWM_CTRL_ACTION_STOP};
                xQueueSend(mcpwm_action_queue_handle, &tmp, portMAX_DELAY);
            }
#endif
            nvs_save_orsystem_config(&or_config, sizeof(orsys_config_reg_t));
            ESP_LOGI(TAG, "set mcpwm pulse number = %d\r\n", (int)action.data);
            continue;
        }
    }
    vTaskDelete(NULL);
    mcpwm_task_handle = NULL;
}


esp_err_t mcpwm_create_mcpwm_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFALSE;
    if(mcpwm_action_queue_handle == NULL){
        mcpwm_action_queue_handle = xQueueCreate(6, sizeof(u64_cmd_action_t));
    }

    if(mcpwm_event_handle == NULL){
        mcpwm_event_handle = xEventGroupCreate();
    }
    if(mcpwm_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(mcpwm_task, "mcpwm_task", stack_size, ctx, priority, &mcpwm_task_handle, 0);
        if(xReturn){
            ESP_LOGI(TAG, "mcpwm_task(%X) create success\r\n", (uint32_t)ctx);
        }else{
            ESP_LOGI(TAG, "mcpwm_task(%X) create fail\r\n", (uint32_t)ctx);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}


esp_err_t mcpwm_delete_mcpwm_task(void){
    if(mcpwm_task_handle){
        vTaskDelete(mcpwm_task_handle);
        mcpwm_task_handle = NULL;
    }
    if(mcpwm_action_queue_handle){
        vQueueDelete(mcpwm_action_queue_handle);
        mcpwm_action_queue_handle = NULL;
    }
    if(mcpwm_event_handle){
        vEventGroupDelete(mcpwm_event_handle);
        mcpwm_event_handle = NULL;
    }
    return ESP_OK;
}

typedef struct switch_scan_s{
    struct{
        const int key[12];
        const uint8_t key_number;
    }key;
    struct{
        uint16_t resual[12];
    }scan_resual;
    union{
        struct {
            uint8_t scan_index : 4;
            uint8_t scan_limit : 4;
        };
    };
}switch_scan_t;

typedef struct switch_scan_s * switch_scan_handle_t;

static QueueHandle_t switch_scan_queue_handle = NULL;

static bool IRAM_ATTR timer_timeout_callback(void* ctx){
    BaseType_t high_task_awoken = pdFALSE;
    switch_scan_handle_t scan = (switch_scan_handle_t)(ctx);
    
    if(scan->scan_index != scan->scan_limit){
        for(uint8_t i = 0; i < scan->key.key_number; i++){
            scan->scan_resual.resual[i] |= ((uint16_t)gpio_get_level(scan->key.key[i])) << scan->scan_index;
        }
        scan->scan_index += 1;
    }else{
        scan->scan_index = 0;
        if(switch_scan_queue_handle){
            xQueueSendFromISR(switch_scan_queue_handle, scan, &high_task_awoken);
            memset(&scan->scan_resual, 0, sizeof(scan->scan_resual));
        }
    }
    
    return (high_task_awoken == pdTRUE);
}

static void timer_initializa(int group, int timer, size_t freq, void* ctx){
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = 16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */

    uint64_t alarm_value = TIMER_BASE_CLK / freq / config.divider;

    timer_set_alarm_value(group, timer, alarm_value);
    timer_enable_intr(group, timer);

    timer_isr_callback_add(group, timer, timer_timeout_callback, ctx, 0);

    timer_start(group, timer);
}

static _Noreturn void gpio_task(void* ctx){
#define IO_ST_TR_LEVEL                 0
#define IO_ST_INDEX                    6
    switch_scan_t switch_scan = {
        .scan_resual = {{0}},
        .key = {
            .key = {IO_S1, IO_S2, IO_K1, IO_K2, IO_K3, IO_K4, IO_ST, -1},
            .key_number = 7,
        },
        .scan_limit = 10,
    };
    uint8_t scan_data_buffer[sizeof(switch_scan_t) + 1] = {0};
    uint32_t sw_value = 0, sw_value_his = 0;
    do{
        gpio_config_t io_conf = {0};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pin_bit_mask = (1ULL << IO_S1) | (1ULL << IO_S2) | (1ULL << IO_K1) | (1ULL << IO_K2) | (1ULL << IO_K3) | (1ULL << IO_K4);
        gpio_config(&io_conf);

        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pin_bit_mask = (1ULL << IO_ST);
        gpio_config(&io_conf);
    }while(0);

    timer_initializa(0, 1, 500, &switch_scan);
    
    while(1){
        BaseType_t xReturn = xQueueReceive(switch_scan_queue_handle, scan_data_buffer, portMAX_DELAY);
        if(xReturn != pdTRUE) continue;
        switch_scan_handle_t scan_handle = (switch_scan_handle_t)scan_data_buffer;
        for(int i = 0; i < scan_handle->key.key_number; i++){
            if(scan_handle->scan_resual.resual[i] >= (0x1UL << (scan_handle->scan_limit - 2))){   /// 为高
                sw_value |= 1UL << i;
                continue;
            }
            if(scan_handle->scan_resual.resual[i] <= (0x1UL << (scan_handle->scan_limit - 8))){    /// 为低
                sw_value &= (~(1UL << i));
            }
            // ESP_LOGD(TAG, "key[%d] scan resual: %d", i, (uint16_t)*(resual_ptr + i));
        }
        if(sw_value != sw_value_his){
            sw_value_his = sw_value;
            uint8_t k_value = 0, s_value = 0, st_value = 0;
            uint32_t freq_arr[16] = {550 * 2, 950 * 2, 1300 * 2, 
                                     285 * 1000 * 2, 300 * 1000 * 2, 315 * 1000 * 2,
                                     380 * 1000 * 2, 400 * 1000 * 2, 420 * 1000 * 2,
                                     800 * 1000 * 2, 900 * 1000 * 2, 1000 * 1000 * 2,
                                     10 * 1000 * 2, 20 * 1000 * 2, 30 * 1000 * 2, 50 * 2};
            k_value = (sw_value >> 2) & 0x0F;
            s_value = sw_value & 0x03;
            switch(s_value){   /// S 开关
                case 1:
                    mcpwm_t_action(MCPWM_CTRL_ACTION_SET_DUTY, k_value * 7);  /// 设置占空比
                break;
                case 2:
                    mcpwm_t_action(MCPWM_CTRL_ACTION_SET_FREQUENCY, freq_arr[k_value]);   /// 设置频率
                break;
                case 3:
                    mcpwm_t_action(MCPWM_CTRL_ACTION_SET_PULSE_NUMBER, k_value);   /// 设置脉冲个数
                break;
                case 0:
                default:
                    
                break;
            };
            if((sw_value & (1UL << (IO_ST_INDEX))) == IO_ST_TR_LEVEL){
                EventBits_t gbit = xEventGroupGetBits(mcpwm_event_handle);
                if(gbit & MCPWM_EVENT_START_BIT){
                    mcpwm_t_action(MCPWM_CTRL_ACTION_STOP, 0);
                }else if(gbit & MCPWM_EVENT_READY_BIT){
                    mcpwm_start_output();
                    ESP_LOGI(TAG, "mcpwm start output\r\n");
                }else if(gbit & MCPWM_EVENT_STOP_BIT){
                    mcpwm_t_action(MCPWM_CTRL_ACTION_START_READY, 0);
                }
            }
            ESP_LOGD(TAG, "switch value: sx[%d], kx[%d]\r\n", s_value, k_value);
        }
    }
#undef IO_ST_TR_LEVEL
#undef IO_ST_INDEX
    vTaskDelete(NULL);
    gpio_task_handle = NULL;
}

esp_err_t gpio_create_gpio_task(int priority, uint32_t stack_size, void* ctx){
    esp_err_t err = ESP_FAIL;
    BaseType_t xReturn = pdFALSE;
    
    if(switch_scan_queue_handle == NULL){
        switch_scan_queue_handle = xQueueCreate(4, sizeof(switch_scan_t));
    }

    if(gpio_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(gpio_task, "gpio_task", stack_size, ctx, priority, &gpio_task_handle, 0);
        if(xReturn){
            ESP_LOGI(TAG, "gpio_task(%X) create success\r\n", (uint32_t)ctx);
            err = ESP_OK;
        }else{
            ESP_LOGI(TAG, "gpio_task(%X) create fail\r\n", (uint32_t)ctx);
            err = ESP_FAIL;
        }
    }
    return err;
}


esp_err_t gpio_delete_gpio_task(void){
    if(gpio_task_handle){
        vTaskDelete(NULL);
        gpio_task_handle = NULL;
    }
    if(switch_scan_queue_handle){
        vQueueDelete(switch_scan_queue_handle);
        switch_scan_queue_handle = NULL;
    }
    return ESP_OK;
}


#endif


_Noreturn static void uart_receive_task(void* ctx){
//    esp_err_t err = ESP_FAIL;
//    BaseType_t xReturn = pdFAIL;
#define UART_RX_BUFFER_SIZE               sizeof(twai_message_t)
    uint8_t data_buffer_ptr[UART_RX_BUFFER_SIZE + 1] = {0};
    int read_byte_number = 0;
    EventBits_t wbit = 0;
    uint8_t uart_num = UART_NUM_0;

    while(1){
        wbit = xEventGroupWaitBits(uart_event_handle, UART_CTRL_CHANGE_BR_BIT, true, false, 0);
        if(wbit == UART_CTRL_CHANGE_BR_BIT){
            uart_config_t config = {0};
            taskENTER_CRITICAL(&orsystem_configure.mux);
            uart_num = orsystem_configure.uart.index;
            config = orsystem_configure.uart.config;
            taskEXIT_CRITICAL(&orsystem_configure.mux);

            esp32_uart_deinit(uart_num);
            esp32_uart_init(uart_num, &config);
        }
        read_byte_number = uart_read_bytes(uart_num, data_buffer_ptr, UART_RX_BUFFER_SIZE, pdMS_TO_TICKS(500));
        if(read_byte_number < 1) continue;
        uart_data_frame_handle_t data_handle = (uart_data_frame_handle_t)data_buffer_ptr;
        
        // ESP_LOGD(TAG, "uart receive data: %#x %#x %#x %#x %#x %#x %#x %#x\r\n", );
        twai_receive_data_parse(data_handle);
    }
    vTaskDelete(NULL);
    uart_receive_task_handle = NULL;
#undef UART_RX_BUFFER_SIZE
}

esp_err_t uart_create_uart_receive_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;
    if(uart_event_handle == NULL){
        uart_event_handle = xEventGroupCreate();
    }
    if(uart_receive_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(uart_receive_task, "", stack_size, ctx, priority, &uart_receive_task_handle, 0);
        if(xReturn == pdPASS){
            ESP_LOGI(TAG, "uart_receive_task(%#X) create success\r\n", (uint32_t)ctx);
        }else{
            ESP_LOGE(TAG, "uart_receive_task(%#X) create fail\r\n", (uint32_t)ctx);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_err_t uart_delete_uart_receive_task(void){
    if(uart_receive_task_handle){
        vTaskDelete(uart_receive_task_handle);
        uart_receive_task_handle = NULL;
        ESP_LOGI(TAG, "uart_receive_task() delete success\r\n");
    }
    if(uart_event_handle){
        vEventGroupDelete(uart_event_handle);
        uart_event_handle = NULL;
    }
    return ESP_OK;
}


void twai_t_action(uint8_t action, uint32_t data){
    u64_cmd_action_t tmp = {.action.cmd_class = CMD_CLASS_TWAI, .data = 0UL};
    
    tmp.action.cmd = action;
    tmp.data = data;
    if(twai_ctrl_queue) xQueueSend(twai_ctrl_queue, &tmp, portMAX_DELAY);
}

_Noreturn static void twai_task(void* ctx){
    esp_err_t err = ESP_FAIL;
    BaseType_t xReturn = pdFAIL;

    uint32_t alert =  TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_RECOVERY_IN_PROGRESS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_AND_LOG;

   err = twai_reconfigure_alerts(alert, NULL);
    do{
        twai_t_action(TWAI_CTRL_INIT, 0);
    }while(0);

    while(true){
        u64_cmd_action_t cmd;
        xReturn = xQueueReceive(twai_ctrl_queue, &cmd, portMAX_DELAY);
        if((xReturn != pdTRUE) || (cmd.action.cmd_class != CMD_CLASS_TWAI)) continue;
        if(cmd.action.cmd == TWAI_CTRL_SELF_TEST){
            ESP_LOGI(TAG, "can bus self test......\r\n");
            twai_delete_receive_task();
            twai_delete_transmit_task();
            esp_err_t err = esp32_can_self_test();
            if(err != ESP_OK){
                xEventGroupClearBits(twai_event_handle, TWAI_EVENT_SELF_PASS_BIT);
                ESP_LOGE(TAG, "can bus can't pass the self test\r\n");
            }else{
                // TODO @ CAN 总线自检通过, 创建发送, 接受任务
                xEventGroupSetBits(twai_event_handle, TWAI_EVENT_SELF_PASS_BIT);
                continue;
            }
        }
        if(cmd.action.cmd == TWAI_CTRL_INIT){
            xEventGroupClearBits(twai_event_handle, TWAI_EVENT_OK_BIT);
            twai_timing_config_t timing = { 0 };
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
            taskENTER_CRITICAL(&orsystem_configure.mux);
            timing = orsystem_configure.can.timing;
            taskEXIT_CRITICAL(&orsystem_configure.mux);
            ESP_LOGD(TAG, "can bus initialaize %d, %d, %d, %d\r\n", timing.brp, timing.tseg_1, timing.tseg_2, timing.sjw);
            twai_status_info_t stat;
            err = twai_get_status_info(&stat);
            if(err == ESP_ERR_INVALID_STATE) goto twai_install_driver_section;
            if(stat.state != TWAI_STATE_RUNNING) goto twai_install_driver_section;
            while(stat.msgs_to_tx){
                vTaskDelay(pdMS_TO_TICKS(100));
                twai_get_status_info(&stat);
            }
twai_install_driver_section:
            esp32_can_uninstall();
            if(esp32_can_install(g_config, timing, f_config) != ESP_OK){  // 初始化 CAN 驱动
                ESP_LOGE(TAG, "can driver install fail! will reboot in 5 second\r\n");
                uint8_t i = 5, s = 1;
                while(--i){
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    s = !s;
                }
                esp_restart();
            }
            orsys_config_reg_t config = {0};
            taskENTER_CRITICAL(&orsystem_configure.mux);
            config = orsystem_configure;
            taskEXIT_CRITICAL(&orsystem_configure.mux);

            xEventGroupSetBits(twai_event_handle, TWAI_EVENT_OK_BIT);

            nvs_save_orsystem_config(&config, sizeof(orsys_config_reg_t));

            twai_create_transmit_task(11, 4096, NULL);
            twai_create_receive_task(12, 4096, NULL);
            
            twai_t_action(TWAI_CTRL_ALERT, 0);
            continue;
        }
        if(cmd.action.cmd == TWAI_CTRL_ALERT){
            alert = 0;
            err = twai_read_alerts(&alert, 0);
            if(err != ESP_OK) continue;
//            if(err == ESP_OK){
                if(alert & TWAI_ALERT_BUS_OFF){
                    esp_err_t err = ESP_FAIL;
                    xEventGroupClearBits(twai_event_handle, TWAI_EVENT_OK_BIT);
                    err = twai_initiate_recovery();
                    if(err == ESP_OK){
                        twai_t_action(TWAI_CTRL_ALERT, 0);
                    }else{
                        twai_t_action(TWAI_CTRL_INIT, 0);
                    }
                }else if(alert & TWAI_ALERT_ERR_PASS){

                }else if(alert & TWAI_ALERT_RX_QUEUE_FULL){

                }else if(alert & TWAI_ALERT_TX_FAILED){

                }else if(alert & TWAI_ALERT_BUS_ERROR){

                }else if(alert & TWAI_ALERT_ABOVE_ERR_WARN){

                }else if(alert & TWAI_ALERT_ARB_LOST){

                }else if(alert & TWAI_ALERT_BUS_RECOVERED){
                    esp_err_t err = ESP_FAIL;
                    err = twai_start();
                    if(err == ESP_OK){
                        xEventGroupSetBits(twai_event_handle, TWAI_EVENT_OK_BIT);
                    }else{
                        twai_t_action(TWAI_CTRL_INIT, 0);
                    }
                }else if(alert & TWAI_ALERT_RECOVERY_IN_PROGRESS){
                    vTaskDelay(pdMS_TO_TICKS(500));
                    twai_t_action(TWAI_CTRL_ALERT, 0);
                }else if(alert & TWAI_ALERT_ERR_ACTIVE){

                }else if(alert & TWAI_ALERT_BELOW_ERR_WARN){

                }else{

                }
//            }
            continue;
        }
    }
    vTaskDelete(NULL);
    twai_task_handle = NULL;
}


esp_err_t twai_create_twai_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;
    if(twai_ctrl_queue == NULL){
        twai_ctrl_queue = xQueueCreate(16, sizeof(u64_cmd_action_t));
    }
    if(twai_event_handle == NULL){
        twai_event_handle = xEventGroupCreate();
    }
    if(twai_transmit_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(twai_task, "twai_task", stack_size, ctx, priority,
                                          &twai_task_handle, 0);
        if(xReturn != pdPASS){
            ESP_LOGE(TAG, "twai_task() create fail!\r\n");
            return ESP_FAIL;
        }else{
            ESP_LOGI(TAG, "twai_task() create success!\r\n");
        }
    }
    return ESP_OK;
}

esp_err_t twai_delete_twai_task(){
    if(twai_task_handle){
        vTaskDelete(twai_task_handle);
        twai_task_handle = NULL;
    }
    if(twai_ctrl_queue){
        vQueueDelete(twai_ctrl_queue);
        twai_ctrl_queue = NULL;
    }
    return ESP_OK;
}


_Noreturn static void twai_transmit_task(void* ctx){
//    esp_err_t err = ESP_FAIL;
    BaseType_t xReturn = pdFAIL;
    uint32_t action = 0;

    while(true){
        u32_twai_trans_ctrl_t tmp = {.val = 0};
        do{
            EventBits_t gbit = xEventGroupGetBits(twai_event_handle);
            if(!(gbit & TWAI_EVENT_OK_BIT)){
                xEventGroupWaitBits(twai_event_handle, TWAI_EVENT_OK_BIT, false, false, portMAX_DELAY);
            }
        }while(0);

        xReturn = xQueueReceive(twai_transmit_control_queue, &tmp.val, pdMS_TO_TICKS(10000));
        if(xReturn != pdTRUE) goto error_preccess;
        action = tmp.action;
        if(action == 0) continue;
//        if(action){
            if(action == TWAI_TRANS_CTRL_PING){
                // TODO @ twai ping
                twai_message_t t_message = {.identifier = ID_REMOTE_PING, .rtr = 1, .ss = 1, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
                twai_transmit(&t_message, portMAX_DELAY);
                continue;
            }
            if(action == TWAI_TRANS_CTRL_PING_RESPONSE){
                // TODO @ twai return ping response
                twai_message_t t_message = {.identifier = ID_DATA_PING_DATA, .rtr = 0, .ss = 1, .data_length_code = 8, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
                TickType_t tick = xTaskGetTickCount();
               *(uint32_t*)t_message.data = tick;
               *(uint32_t*)(t_message.data + 4) = CONFIG_ORSYSTEM_ID;
                twai_transmit(&t_message, portMAX_DELAY);
                continue;
            }
#if (CONFIG_ORSYSTEM_MODE)
             if(action == TWAI_TRANS_CTRL_RETURN_REGISTER_DATA){   /// 返回配置数据
                // TODO @ twai return configure data
                u8_register_configure_format_t u8 = {.val = tmp.data};
                twai_message_t t_message = {.identifier = ID_DATA_RETURN_REGISTER_DATA, .rtr = 0, .ss = 0};
                ads131a_dev_t ads_dev = {0};
                uint8_t *ptr = NULL;
                u64_orsys_reg_config_format_handle_t h = NULL;
                taskENTER_CRITICAL(&orsystem_configure.mux);
                ads_dev = orsystem_configure.analog.ads131a_dev.dev;  // 获取配置列表
                ptr = (uint8_t*)&ads_dev.id_msb.val; // 获取起始地址
                h = (u64_orsys_reg_config_format_handle_t)t_message.data;
                t_message.data_length_code = sizeof(u64_orsys_reg_config_format_t);   // 数据长度
                h->base_addr = (size_t)(&orsystem_configure.analog) - (size_t)(&orsystem_configure);
                h->offset = u8.start_address;
                h->data_length = 1;
                h->data = *(uint8_t*)(ptr + u8.start_address);
                taskEXIT_CRITICAL(&orsystem_configure.mux);

                twai_transmit(&t_message, pdMS_TO_TICKS(3000));
                continue;
            }
            if(action == TWAI_TRANS_CTRL_RETURN_ANALOG_DATA){  /// 返回采样数据
#if(1)
                EventBits_t gbit = 0;
                bool rt = false;
                taskENTER_CRITICAL(&orsystem_configure.mux);
                rt = orsystem_configure.analog.ctrl.data_return;    /// 检查数据控制方式
                taskEXIT_CRITICAL(&orsystem_configure.mux);
                if(rt) continue;
                gbit = xEventGroupGetBits(ads_event_group_handle);
                if(!(gbit & ADS_EVENT_STOP_BIT)) continue;
                int32_t *ptr = NULL;
                ptr = (int32_t *)ads_data_ptr.ptr;
                twai_message_t t_message = {.data_length_code = 8, .extd = 0, .rtr = 0, .ss = 0};
                for(int i = 0; i < (ads_data_ptr.data_length / sizeof(int32_t));){
                    for(int j = 0; j < 4; j++){
                        t_message.identifier = ID_DATA_CH1_ANALOG_DATA + j;
                        memcpy(&t_message.data[4], (void*)(ptr + i + j), sizeof(int32_t));
                        twai_transmit(&t_message, portMAX_DELAY);
                    }
                    i += 4;
                }
                ptr = (int32_t*)ads_data_ptr.ptr_isr;
                for(int i = 0; i < (ads_data_ptr.data_length / sizeof(int32_t));){
                    for(int j = 0; j < 4; j++){
                        t_message.identifier = ID_DATA_CH1_ANALOG_DATA + j;
                        memcpy(&t_message.data[4], (void*)(ptr + i + j), sizeof(int32_t));
                        twai_transmit(&t_message, portMAX_DELAY);
                    }
                    i += 4;
                }

#endif
            #if(0)
                // TODO @ twai return analog data
                uint8_t ch = (uint8_t)tmp.data;
                FILE *file = NULL;
                struct stat file_stat;
                size_t remain_bytes = 0;
                uint32_t *data_buffer_ptr = NULL;
                twai_message_t t_message = {.extd = 0, .rtr = 0, .ss = 0};

                char file_name[16];
                strcpy(file_name,  CONFIG_SPIFFS_STORAGE_BASE_PATH);
                strcat(file_name, "/tmp.bin");
                if(stat(file_name, &file_stat) != 0) continue;

                data_buffer_ptr = (uint32_t*)heap_caps_malloc(4096, MALLOC_CAP_32BIT);
                if(data_buffer_ptr == NULL) continue;
                file = fopen("/spiffs/tmp.bin", "rb");
                if(file == NULL) goto return_analog_data_end_section;
                
                ESP_LOGI(TAG, "%s file size:%ld byte\r\n", "/spiffs/tmp.bin", file_stat.st_size);
                remain_bytes = file_stat.st_size;
                t_message.data_length_code = 8;
#define MAX_CHANNEL_NUMBER            4
                while(remain_bytes){
                    size_t read_bytes = fread(data_buffer_ptr, sizeof(uint8_t), CONFIG_ADS_DATA_STATIC_BUFFER_SIZE, file);
                    if(read_bytes == 0) break;
                    remain_bytes -= read_bytes;
                    for(uint32_t i = 0; i < (CONFIG_ADS_DATA_STATIC_BUFFER_SIZE / sizeof(uint32_t));){
                        for(int j = 0; j < MAX_CHANNEL_NUMBER; j++){
                            if(ch & (1 << j)){  /// 选择相应的通道
                                t_message.identifier = ID_DATA_CH1_ANALOG_DATA + j;
                                uint32_t d = (uint32_t)*(data_buffer_ptr + i * MAX_CHANNEL_NUMBER + j);
                                memcpy(&t_message.data, &i, sizeof(typeof(i)));
                                memcpy(&t_message.data[sizeof(uint32_t)], &d, sizeof(typeof(d)));
                                if(!(xEventGroupGetBits(twai_event_handle) & TWAI_EVENT_OK_BIT)) goto return_analog_data_end_section;
                                esp_err_t err = twai_transmit(&t_message, pdMS_TO_TICKS(10000));
                                if(err != ESP_OK){
                                    uint32_t action = TWAI_CTRL_ALERT;
                                    xQueueSend(twai_ctrl_queue, &action, 0);
                                }
                            }
                        }
                        i += MAX_CHANNEL_NUMBER;
                    }
                }
                ESP_LOGI(TAG, "file read complete\r\n");
                return_analog_data_end_section:
                fclose(file);
                free(data_buffer_ptr);
#undef MAX_CHANNEL_NUMBER
                continue;
                #endif
            }

#else


#endif
error_preccess:
        do{
            twai_message_t t_message = {.identifier = ID_DATA_TICK, .ss = 1, .data_length_code = 8, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
            TickType_t tick = xTaskGetTickCount();
            *(uint32_t*)t_message.data = tick;
            *(uint32_t*)(t_message.data + 4) = CONFIG_ORSYSTEM_ID;
            twai_transmit(&t_message, pdMS_TO_TICKS(1000));
 
            twai_t_action(TWAI_CTRL_ALERT, 0);  /// 检查TWAIN Alert
            continue;
        }while(0);
    }
    vTaskDelete(NULL);
    twai_transmit_task_handle = NULL;
}

esp_err_t twai_create_transmit_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;

    if(twai_transmit_control_queue == NULL){
        twai_transmit_control_queue = xQueueCreate(TWAI_ID_QUEUE_LENGTH, sizeof(u32_twai_trans_ctrl_t));
    }

    if(twai_transmit_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(twai_transmit_task, "twai_transmit_task", stack_size, ctx, priority, &twai_transmit_task_handle, 0);
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
    if(twai_transmit_control_queue){
        vQueueDelete(twai_transmit_control_queue);
        twai_transmit_control_queue = NULL;
    }
    if(twai_transmit_task_handle){
        vTaskDelete(twai_transmit_task_handle);
        twai_transmit_task_handle = NULL;
    }
    return ESP_OK;
}

_Noreturn static void twai_receive_task(void* ctx){
    esp_err_t err = ESP_FAIL;

    while(true){
        twai_message_t r_message = {0};
        do{
            EventBits_t gbit = xEventGroupGetBits(twai_event_handle);
            if((!(gbit & TWAI_EVENT_OK_BIT))){
                xEventGroupWaitBits(twai_event_handle, TWAI_EVENT_OK_BIT, false, false, portMAX_DELAY);
            }
        }while(0);
        err = twai_receive(&r_message, portMAX_DELAY);
        // TODO @ 处理来自CAN主机的数据
        if(err != ESP_OK) continue;
        ESP_LOGD(TAG, "twai receive data frame id:%#x, rtr:%d, data:%#x %#x %#x %#x %#x %#x %#x %#x \r\n", r_message.identifier, r_message.rtr, r_message.data[0], r_message.data[1], r_message.data[2], r_message.data[3], r_message.data[4], r_message.data[5], r_message.data[6], r_message.data[7]);
        twai_receive_data_parse(&r_message);

    }
    vTaskDelete(NULL);
    twai_receive_task_handle = NULL;
}

esp_err_t twai_create_receive_task(int priority, uint32_t stack_size, void* ctx){
    BaseType_t xReturn = pdFAIL;

    if(twai_receive_task_handle == NULL){
        xReturn = xTaskCreatePinnedToCore(twai_receive_task, "twai_receive_task", stack_size, ctx, priority, &twai_receive_task_handle, 0);
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


void twai_wait_for_initializer(void){
    EventBits_t ubit = 0;
    while(!(ubit & TWAI_EVENT_OK_BIT)){
        ubit = xEventGroupWaitBits(twai_event_handle, TWAI_EVENT_OK_BIT, false, true, portMAX_DELAY);
    }
}