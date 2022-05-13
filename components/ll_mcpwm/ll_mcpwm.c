//
// Created by XIAOSENLUO on 2022/4/19.
//

#include "ll_mcpwm.h"
#include "string.h"



#define LL_MCPWM_CONFIGURE_INITIALIZER() { \
                                          .clk_div = 15, \
                                          .timer_div = 9, \
}\




static ll_mcpwm_config_t mcpwm_config[SOC_MCPWM_PERIPH_NUM] = {
        LL_MCPWM_CONFIGURE_INITIALIZER(),
        LL_MCPWM_CONFIGURE_INITIALIZER(),
};


esp_err_t ll_mcpwm_init(mcpwm_dev_t *mcpwm, int op, const ll_mcpwm_config_t *config){
    int mcpwm_index = 0;
    int timer_index = op;
    float duty[SOC_MCPWM_COMPARATOR_NUM] = {0.00, 0.00};

    if((size_t)mcpwm == (size_t)&MCPWM1) mcpwm_index = 1;

    periph_module_enable(PERIPH_PWM0_MODULE + mcpwm_index);   // 使能 MCPWM 模块

    // memcpy(&mcpwm_config[mcpwm_index], config, sizeof(ll_mcpwm_config_t));
    mcpwm_config[mcpwm_index] = (ll_mcpwm_config_t)*config;

    mcpwm_ll_init(mcpwm);   // 初始化 MCPWM 全局设置
    mcpwm_ll_set_clock_prescale(mcpwm, config->clk_div);   // 设置时钟分频

    mcpwm_ll_timer_set_prescale(mcpwm, timer_index, config->timer_div);   // 设置定时器分频
    uint32_t period = (2 * APB_CLK_FREQ) / (config->frequency * (config->clk_div +1) * (config->timer_div + 1));
//    mcpwm_ll_timer_set_period(mcpwm, timer_index, period);   // 设置周期
    ll_mcpwm_timer_set_period(mcpwm, timer_index, period, config->period_up);

    mcpwm_config[mcpwm_index].frequency = (2 * APB_CLK_FREQ) / (period * (config->clk_div + 1) * (config->timer_div + 1));  // 重新计算实际频率
    mcpwm_ll_timer_set_count_mode(mcpwm, timer_index, config->counter_mode);  // 设置计数模式

    mcpwm_ll_operator_select_timer(mcpwm, op, timer_index);  // 为操作器选择定时器
    for(int cmp = 0; cmp < SOC_MCPWM_COMPARATOR_NUM; cmp++){   // 设置 A, B 通道比较寄存器
        mcpwm_ll_operator_set_compare(mcpwm, op, cmp, (uint16_t)(config->duty[cmp] * period / 100.00));
    }
    ll_mcpwm_operator_set_compare_upmethod(mcpwm, op, config->cmpr_up);

    for(int gen = 0; gen < SOC_MCPWM_GENERATOR_NUM; gen++){
        mcpwm_ll_gen_set_zero_action(mcpwm, op, gen, config->at_zero_action[gen]);
        mcpwm_ll_gen_set_period_action(mcpwm, op, gen, config->at_period_action[gen]);
        mcpwm_ll_gen_set_cmp_action(mcpwm, op, gen, gen, config->at_cmpr_up_action[gen], config->at_cmpr_dwon_action[gen]);
        mcpwm_ll_gen_set_cmp_action(mcpwm, op, gen, 1 - gen, MCPWM_ACTION_NO_CHANGE, MCPWM_ACTION_NO_CHANGE);
    }

    return ESP_OK;
}


void ll_mcpwm_set_frequency(mcpwm_dev_t *mcpwm,  int op, uint32_t frequency){
    uint32_t mcpwm_index = 0;
    uint32_t timer_index = 0;
    uint32_t period = 0;
    ll_mcpwm_config_t *config = NULL;

    if((size_t)mcpwm == (size_t)(&MCPWM1)) mcpwm_index = 1;
    timer_index = op;
    config = &mcpwm_config[mcpwm_index];

    config->frequency = frequency;

    period = (2 * APB_CLK_FREQ) / (config->frequency * (config->clk_div +1) * (config->timer_div + 1));
    ll_mcpwm_timer_set_period(mcpwm, timer_index, period, config->period_up); // 设置周期

    config->frequency = (2 * APB_CLK_FREQ) / (period * (config->clk_div + 1) * (config->timer_div + 1));  // 重新计算实际频率

    for(int cmp = 0; cmp < SOC_MCPWM_COMPARATOR_NUM; cmp++){   // 重新计算比较寄存器的值, 保持占空比不变
        mcpwm_ll_operator_set_compare(mcpwm, op, cmp, (uint32_t)(config->duty[cmp] * period / 100.00));
    }
    ll_mcpwm_operator_set_compare_upmethod(mcpwm, timer_index, config->cmpr_up);
}


uint32_t ll_mcpwm_get_frequency(mcpwm_dev_t *mcpwm,  int op){
    uint32_t mcpwm_index = 0;

    if((size_t)mcpwm == (size_t)(&MCPWM1)) mcpwm_index = 1;

    return mcpwm_config[mcpwm_index].frequency;
}

void ll_mcpwm_set_duty(mcpwm_dev_t *mcpwm, int op, int gen, float duty){
    uint32_t mcpwm_index = 0;
    uint32_t timer_index = 0;
    uint32_t period = 0;
    ll_mcpwm_config_t *config = NULL;

    if((size_t)mcpwm == (size_t)(&MCPWM1)) mcpwm_index = 1;
    timer_index = op;
    config = &mcpwm_config[mcpwm_index];

    config->duty[gen] = duty;

//    period = (2 * APB_CLK_FREQ) / (config->frequency * (config->clk_div +1) * (config->timer_div + 1));

    period = HAL_FORCE_READ_U32_REG_FIELD(mcpwm->timer[timer_index].period, period);

    mcpwm_ll_operator_set_compare(mcpwm, op, gen, (uint32_t)(duty * period / 100.00));
    ll_mcpwm_operator_set_compare_upmethod(mcpwm, timer_index, config->cmpr_up);
}


float ll_mcpwm_get_duty(mcpwm_dev_t *mcpwm, int op, int gen){
    uint32_t mcpwm_index = 0;

    if((size_t)mcpwm == (size_t)(&MCPWM1)) mcpwm_index = 1;

    return mcpwm_config[mcpwm_index].duty[gen];
}
