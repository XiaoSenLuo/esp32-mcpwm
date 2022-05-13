//
// Created by XIAOSENLUO on 2022/4/19.
//

#ifndef ORSYSTEM_LL_MCPWM_H
#define ORSYSTEM_LL_MCPWM_H


#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "soc/mcpwm_periph.h"
#include "soc/mcpwm_struct.h"
#include "hal/hal_defs.h"
#include "hal/mcpwm_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MCPWM_TIMER_START = 2,
    MCPWM_TIMER_START_AND_STOP_AT_NEXT_TEZ = 3,
    MCPWM_TIMER_START_AND_STOP_AT_NEXT_TEP = 4,
}mcpwm_start_t;

typedef enum {
    MCPWM_TIMER_STOP_AT_TEZ = 0,
    MCPWM_TIMER_STOP_AT_TEP = 1,
}mcpwm_stop_t;

typedef enum {
    MCPWM_PERIOD_UP_IMMEDIATELY = 0,
    MCPWM_PERIOD_UP_AT_TEZ,
    MCPWM_PERIOD_UP_AT_SYC,
    MCPWM_PERIOD_UP_AT_TEZ_SYC
}mcpwm_timer_period_upmethod_t;


typedef enum {
    MCPWM_TIMER_DIR_UP = 0,
    MCPWM_TIMER_DIR_DWON = 1,
}mcpwm_timer_direction_t;

typedef enum {
    MCPWM_COMPARE_UP_IMMEDIATELY = 0,
    MCPWM_COMPARE_UP_AT_TEZ = BIT(0),
    MCPWM_COMPARE_UP_AT_TEP = BIT(1),
    MCPWM_COMPARE_UP_AT_SYC = BIT(2),
    MCPWM_COMPARE_UP_CLOSE = BIT(3)
}mcpwm_operator_compare_upmethod_t;

typedef enum {
    MCPWM_FORCE_OUTPUT_DISABLE = 0,
    MCPWM_FORCE_OUTPUT_LOW = 1,
    MCPWM_FORCE_OUTPUT_HIGHT = 2,
    MCPWM_FORCE_OUTPUT_CLOSE = 3
}mcpwm_operator_force_output_mode_t;

typedef enum {
    MCPWM_CNTUFORCE_UP_IMMEDIATELY = 0,
    MCPWM_CNTUFORCE_UP_AT_TEZ = BIT(0),
    MCPWM_CNTUFORCE_UP_AT_TEP = BIT(1),
    MCPWM_CNTUFORCE_UP_AT_TEA = BIT(2),
    MCPWM_CNTUFORCE_UP_AT_TEB = BIT(3),
    MCPWM_CNTUFORCE_UP_AT_SYC = BIT(4),
    MCPWM_CNTUFORCE_UP_CLOSE = BIT(5)
}mcpwm_cntuforce_upmethod_t;

typedef enum {
    MCPWM_INTR_T0_STOP_FLAG = BIT(0),
    MCPWM_INTR_T1_STOP_FLAG = BIT(1),
    MCPWM_INTR_T2_STOP_FLAG = BIT(2),
    MCPWM_INTR_T0_TEZ_FLAG = BIT(3),
    MCPWM_INTR_T1_TEZ_FLAG = BIT(4),
    MCPWM_INTR_T2_TEZ_FLAG = BIT(5),
    MCPWM_INTR_T0_TEP_FLAG = BIT(6),
    MCPWM_INTR_T1_TEP_FLAG = BIT(7),
    MCPWM_INTR_T2_TEP_FLAG = BIT(8),
    MCPWM_INTR_FAULT0_FLAG = BIT(9),
    MCPWM_INTR_FAULT1_FLAG = BIT(10),
    MCPWM_INTR_FAULT2_FLAG = BIT(11),
    MCPWM_INTR_FAULT0_CLR_FLAG = BIT(12),
    MCPWM_INTR_FAULT1_CLR_FLAG = BIT(13),
    MCPWM_INTR_FAULT2_CLR_FLAG = BIT(14),
    MCPWM_INTR_OP0_TEA_FLAG = BIT(15),
    MCPWM_INTR_OP1_TEA_FLAG = BIT(16),
    MCPWM_INTR_OP2_TEA_FLAG = BIT(17),
    MCPWM_INTR_OP0_TEB_FLAG = BIT(18),
    MCPWM_INTR_OP1_TEB_FLAG = BIT(19),
    MCPWM_INTR_OP2_TEB_FLAG = BIT(20),
    MCPWM_INTR_FH0_CBC_FLAG = BIT(21),
    MCPWM_INTR_FH1_CBC_FLAG = BIT(22),
    MCPWM_INTR_FH2_CBC_FLAG = BIT(23),
    MCPWM_INTR_FH0_OST_FLAG = BIT(24),
    MCPWM_INTR_FH1_OST_FLAG = BIT(25),
    MCPWM_INTR_FH2_OST_FLAG = BIT(26),
    MCPWM_INTR_CAP0_FLAG = BIT(27),
    MCPWM_INTR_CAP1_FLAG = BIT(28),
    MCPWM_INTR_CAP2_FLAG = BIT(29),
}mcpwm_intr_flag_t;

typedef struct {
    int clk_div;                     /*!<set the clk prescale what is input to mcpwm unit*/
    int timer_div;                   /*!< set timer input clk prescale*/
    uint32_t frequency;              /*!<Set frequency of MCPWM in Hz*/
    float duty[2];                    /*!<index = 0: Set % duty cycle for operator a(MCPWMXA), i.e for 62.3% duty cycle, duty_a = 62.3*/
                                      /*!<index = 1: Set % duty cycle for operator b(MCPWMXB), i.e for 48% duty cycle, duty_b = 48.0*/
    mcpwm_counter_type_t counter_mode;  /*!<Set  type of MCPWM counter*/
    mcpwm_timer_period_upmethod_t period_up;
    mcpwm_operator_compare_upmethod_t cmpr_up;
    mcpwm_output_action_t at_zero_action[2];
    mcpwm_output_action_t at_period_action[2];
    mcpwm_output_action_t at_cmpr_up_action[2];  /*!< index=0:PWMA, index=1:PWMBs*/
    mcpwm_output_action_t at_cmpr_dwon_action[2];
} ll_mcpwm_config_t;

//esp_err_t mcpwm180p_init(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, const mcpwm_config_t *mcpwm_config);

#define LL_MCPWM_DEFAULT_CONFIGURE() { \
                                .clk_div = 15, \
                                .timer_div = 9, \
                                .frequency = 1000, \
                                .duty = {50.0, 50.0}, \
                                .cmpr_up = MCPWM_COMPARE_UP_AT_TEZ, \
                                .period_up = MCPWM_PERIOD_UP_AT_TEZ, \
                                .counter_mode = MCPWM_UP_COUNTER, \
                                .at_zero_action = {MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH}, \
                                .at_cmpr_up_action = {MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_LOW}, \
                                .at_cmpr_dn_action = {MCPWM_ACTION_NO_CHANGE, MCPWM_ACTION_NO_CHANGE}, \
                                .at_period_action = {MCPWM_ACTION_NO_CHANGE, MCPWM_ACTION_NO_CHANGE}, \
}

static inline void ll_mcpwm_timer_start(mcpwm_dev_t *mcpwm, int timer, mcpwm_start_t start){
    mcpwm->timer[timer].mode.start = start;
}

static inline void ll_mcpwm_timer_set_period(mcpwm_dev_t *mcpwm, int timer, uint32_t period, mcpwm_timer_period_upmethod_t method){
    HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->timer[timer].period, period, period - 1);
    mcpwm->timer[timer].period.upmethod = method;
}

static inline uint16_t ll_mcpwm_timer_get_count_value(mcpwm_dev_t *mcpwm, int timer){
    return HAL_FORCE_READ_U32_REG_FIELD(mcpwm->timer[timer].status, value);
}

static inline mcpwm_timer_direction_t ll_mcpwm_timer_get_direction(mcpwm_dev_t *mcpwm, int op) {
    return (mcpwm_timer_direction_t)(HAL_FORCE_READ_U32_REG_FIELD(mcpwm->timer[op].status, direction));
}

static inline void ll_mcpwm_operator_set_compare_upmethod(mcpwm_dev_t *mcpwm, int op, mcpwm_operator_compare_upmethod_t method){
    mcpwm->channel[op].cmpr_cfg.a_upmethod = method;
    mcpwm->channel[op].cmpr_cfg.b_upmethod = method;
}

//static inline void ll_mcpwm_gen_set_tea_action(mcpwm_dev_t *mcpwm, int op, int gen, )

static inline void ll_mcpwm_gen_set_nci_force_output(mcpwm_dev_t *mcpwm, int op, int gen, mcpwm_operator_force_output_mode_t level){
   if(gen == 0){
       HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->channel[op].gen_force, a_nciforce_mode, level);
   }else{
       HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->channel[op].gen_force, b_nciforce_mode, level);
   }
}

static inline void ll_mcpwm_gen_enable_nci_force_output(mcpwm_dev_t *mcpwm, int op, int gen){
    if(gen == 0){
        uint32_t tmp = HAL_FORCE_READ_U32_REG_FIELD(mcpwm->channel[op].gen_force, a_nciforce);
        HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->channel[op].gen_force, a_nciforce, !tmp);
    }else{
        uint32_t tmp = HAL_FORCE_READ_U32_REG_FIELD(mcpwm->channel[op].gen_force, b_nciforce);
        HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->channel[op].gen_force, b_nciforce, !tmp);
    }
}

static inline void ll_mcpwm_gen_set_cntu_force_output(mcpwm_dev_t *mcpwm, int op, int gen, mcpwm_operator_force_output_mode_t level, mcpwm_cntuforce_upmethod_t method){
    if(gen == 0){
        HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->channel[op].gen_force, a_cntuforce_mode, level);
    }else{
        HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->channel[op].gen_force, b_cntuforce_mode, level);
    }
    HAL_FORCE_MODIFY_U32_REG_FIELD(mcpwm->channel[op].gen_force, cntu_force_upmethod, method);
}

static inline void ll_mcpwm_enable_intr(mcpwm_dev_t *mcpwm, mcpwm_intr_flag_t intr_flag){
    mcpwm->int_ena.val |= (intr_flag);
}

static inline void ll_mcpwm_disable_intr(mcpwm_dev_t *mcpwm, mcpwm_intr_flag_t intr_flag){
    mcpwm->int_ena.val &= ~(intr_flag);
}

esp_err_t ll_mcpwm_init(mcpwm_dev_t *mcpwm, int op, const ll_mcpwm_config_t *config);

void ll_mcpwm_set_frequency(mcpwm_dev_t *mcpwm,  int op, uint32_t frequency);
uint32_t ll_mcpwm_get_frequency(mcpwm_dev_t *mcpwm,  int op);

void ll_mcpwm_set_duty(mcpwm_dev_t *mcpwm, int op, int gen, float duty);
float ll_mcpwm_get_duty(mcpwm_dev_t *mcpwm, int op, int gen);



#ifdef __cplusplus
}
#endif
#endif //ORSYSTEM_LL_MCPWM_H
