

#include "include/ads131a.h"

static const char *TAG = "_ADS131A_C";

static uint16_t calculate_crc(const uint8_t *data, size_t length, uint16_t initial){

    uint16_t crc = initial;
    int bitIndex, byteIndex;
	bool dataMSb;					// Most significant bit of data byte
	bool crcMSb;					// Most significant bit of crc byte

    // CRC16-CCITT polynomial
    // NOTE: The polynomial's MSB is generally assumed to be high (and is handled by
    // the "dataMSb ^ crcMSb" operation below) and so it is excluded from this value.
    const uint16_t poly = 0x1021;

    //
    // CRC algorithm...
    //

    // Loop through all bytes in the dataBytes[] array
	for (byteIndex = 0; byteIndex < length; byteIndex++)
	{
	    // Point to MSb in byte
	    bitIndex = 0x80u;

	    // Loop through all bits in the current byte
	    while (bitIndex > 0)
	    {
	        // Check MSB's of data and crc
	        dataMSb = (bool) (data[byteIndex] & bitIndex);
	        crcMSb  = (bool) (crc & 0x8000u);

			// Left-shift CRC register
	        crc <<= 1;

	        // Check if XOR operation of MSBs results in additional XOR operations
	        if (dataMSb ^ crcMSb)
	        {
	            // XOR crc with polynomial
				crc ^= poly;
	        }

	        // Shift MSb pointer to the next data bit
	        bitIndex >>= 1;
	    }
	}

	return crc;
}

#if(0)
static const char* stat_m2_io[] = {"GND", "IOVDD", "NC", "Reserved"};
void ads131a_assert_printf_state(const ads131a_state_t state){
    /**
    printf("------------------------------STATE------------------------------\n");
    if(state.state1){
        printf("adc_in_fault: %u\n", state.state1 & STAT_1_F_ADCIN_MASK);
        printf("din_check_fault: %u\n", state.state1 & STAT_1_F_CHECK_MASK);
        printf("data_ready_fault: %u\n", state.state1 & STAT_1_F_DRDY_MASK);
        printf("command_fault: %u\n", state.state1 & STAT_1_F_OPC_MASK);
        printf("resynchronization_fault: %u\n", state.state1 & STAT_1_F_RESYNC_MASK);
        printf("spi_fault: %u\n", state.state1 & STAT_1_F_SPI_MASK);
        printf("watchdog_timer_fault: %u\n", state.state1 & STAT_1_F_WDT_MASK);
    }
    if(state.state_s){
        printf("chip_selete_fault: %u\n", state.state_s & STAT_S_F_CS_MASK);
        printf("frame_fault: %u\n", state.state_s & STAT_S_F_FRAME_MASK);
        printf("adc_startup_fault: %u\n", state.state_s & STAT_S_F_STARTUP_MASK);
    }
    if(state.state_m2){
        printf("hamming_code(GND[OFF], IOVDD[ON]): %s\n", stat_m2_io[(state.state_m2 & STAT_M2_M2PIN_MASK) >> 4]);
        printf("device_word_length(GND[24-Bit], IOVDD[32-Bit], NC[16-Bit]): %s\n", stat_m2_io[(state.state_m2 & STAT_M2_M1PIN_MASK) >> 2]);
        printf("connect_mode(GND[SYNC Master Mode], IOVDD[ASYNC Slave Mode], NC[SYNC Slave Mode]): %s\n", stat_m2_io[(state.state_m2 & STAT_M2_M0PIN_MASK)]);
    }
    if(state.state_p || state.state_n){
        printf("ain0_threshold:[+:%u][-:%u]\n", state.state_p & STAT_P_F_IN1P_MASK, state.state_n & STAT_N_F_IN1N_MASK);
        printf("ain1_threshold:[+:%u][-:%u]\n", state.state_p & STAT_P_F_IN2P_MASK, state.state_n & STAT_N_F_IN2N_MASK);
        printf("ain2_threshold:[+:%u][-:%u]\n", state.state_p & STAT_P_F_IN3P_MASK, state.state_n & STAT_N_F_IN3N_MASK);
        printf("ain3_threshold:[+:%u][-:%u]\n", state.state_p & STAT_P_F_IN4P_MASK, state.state_n & STAT_N_F_IN4N_MASK);
    }
     **/

    ESP_LOGI(TAG, "\r\n");
}

void ads131a_assert_printf_configure(const ads131a_config_t config){
    printf("----------------------------CONFIGURE----------------------------\n");

}
#endif

static uint8_t frame_length(const ads131a_device_t *device){
#if(0)
    uint8_t fl = 1;   // unit : word
    uint8_t _crc_en = ADS131A_IS_CRC_ENABLE(device);
    uint8_t _is_fixed = ADS131A_IS_FIXED_WORD_SIZE(device);
    uint8_t _ch_nums = ADS131A_GET_CHANNEL_NUMBERA(device);
    if(_is_fixed){   // 固定帧模式
        fl = 2 + _ch_nums;
    }else{       // 动态帧模式
        fl = 1 + _ch_nums * ADS131A_IS_CHANNEL_ENABLE(device) + _crc_en;
    }
    return fl;
#else
    uint8_t fl = 1;   // unit : word
    uint8_t _crc_en = ll_ads_is_crc_en(device);
    uint8_t _is_fixed = ll_ads_is_fixed_mode(device);
    uint8_t _ch_nums = ll_ads_get_device_channel(device);
    if(_is_fixed){   // 固定帧模式
        fl = 2 + _ch_nums;
    }else{       // 动态帧模式
        fl = 1 + _ch_nums * (uint8_t)ll_ads_is_enable_adc(device) + _crc_en;
    }
    return fl;

#endif
}


uint8_t ads131a_read_register(ads131a_device_t* device, uint8_t reg){

    uint16_t opc = OPCODE_RREG | ((uint16_t)(reg) << 8);
    uint16_t response = 0x0000;
    uint8_t *ptr = (uint8_t*)(&device->dev);

    ads131a_send_command(device, opc);
    response = ads131a_get_command_response(device);

    if(UPPER_BYTE(response) == UPPER_BYTE(opc)) *(ptr + (reg)) = LOWER_BYTE(response);

    return LOWER_BYTE(response);
}

uint8_t ads131a_write_register(const ads131a_device_t* device, uint8_t reg, uint8_t data){
    uint16_t opc = OPCODE_WREG | ((uint16_t)(reg) << 8) | data;
    uint16_t response = 0x0000;
    uint8_t *ptr = (uint8_t*)(&device->dev);

    ads131a_send_command(device, opc);
    response = ads131a_get_command_response(device);
    if((opc - 0x2000) == response) *(ptr + (reg)) = data;
    return LOWER_BYTE(response);   // return data
}


#define STATIC_BUFFER 1
uint16_t ads131a_send_command(const ads131a_device_t *device, uint16_t opc){
    uint8_t fl = 1, wb = 4,  crc_en = 0;

#if STATIC_BUFFER
    uint8_t tx_buffer[6 * 4] = { 0 }, rx_buffer[6 * 4] = { 0 };
#else
    uint8_t *tx_buffer = NULL, *rx_buffer = NULL;
#endif
    wb = ll_ads_get_word_bits(device) >> 3;
    fl = frame_length(device);
    crc_en = ll_ads_is_crc_en(device);
//    ESP_LOGD(TAG, "frame length=%d, word size=%d\r\n", fl, wb);
#if STATIC_BUFFER
//    memset(tx_buffer, 0, sizeof(tx_buffer));
//    memset(rx_buffer, 0, sizeof(rx_buffer));
#else
    tx_buffer = (uint8_t*)malloc(fl * wb);
    rx_buffer = (uint8_t*)malloc(fl * wb);
    memset(tx_buffer, 0, fl * wb);
    memset(rx_buffer, 0, fl * wb);
#endif
    tx_buffer[0] = UPPER_BYTE(opc);
    tx_buffer[1] = LOWER_BYTE(opc);

    if(crc_en){
        bool crc_mode = 0;
        crc_mode = ll_ads_get_crc_mode(device);
        uint16_t crc = calculate_crc(tx_buffer, crc_mode ? ((fl - 1) * wb) : (1 * wb), 0xFFFF);
        tx_buffer[(fl - 1) * wb] = UPPER_BYTE(crc);
        tx_buffer[(fl - 1) * wb + 1] = LOWER_BYTE(crc);
        ESP_LOGD(TAG, "crc=%XH\r\n", crc);
    }

    ads131a_dataframe_t dataframe = {.tx_ptr = tx_buffer,
                                    .rx_ptr = rx_buffer,
                                    .dataframe_size = fl * wb,
                                    .fixed = ll_ads_is_fixed_mode(device),
                                    .word_bits = wb << 3};
    ads131a_drv_dataframe(&dataframe);      // 发送命令
#if(0)
    tx_buffer[0] = UPPER_BYTE(OPCODE_NULL);
    tx_buffer[1] = LOWER_BYTE(OPCODE_NULL);
    tx_buffer[(fl - 1) * wb] = 0x00;
    ads131a_drv_dataframe(&dataframe);   // 获取响应
#endif
    ESP_LOGD(TAG, "ads131a0x device return [%#X] response: %#X\r\n", opc, dataframe.response);
#if (!STATIC_BUFFER)
    free(tx_buffer);
    free(rx_buffer);
#endif
    return dataframe.response;
}

static void printf_status(const ads131a_device_t *device){
    printf("STAT_1:%#x, STAT_P:%#x, STAT_N:%#x, STAT_S:%#x, STAT_M2:%#x\r\n", device->dev.stat_1.val, device->dev.stat_p.val, device->dev.stat_n.val, device->dev.stat_s.val, device->dev.stat_m2.val);
}

uint8_t ads131a_startup(ads131a_device_t *device){
    uint16_t response = 0x0000;

    ads131a_send_command(device, OPCODE_NULL);   // 必须的, 验证 SPI 接口通信
    response = ads131a_send_command(device, OPCODE_NULL);   /* (OPTIONAL) Verify READY response */
    ESP_LOGI(TAG, "adc device is ads131a0%d\r\n", response & 0x000F);

    ads131a_send_command(device, OPCODE_UNLOCK);
    response = ads131a_get_command_response(device);
    if(response == OPCODE_UNLOCK) ESP_LOGI(TAG, "device unlocked!\r\n");
    else ESP_LOGI(TAG, "device locked!");

    ads131a_get_device_id(device);
    ads131a_get_device_status(device);
    printf_status(device);

    return response;
}

uint8_t ads131a_adcen(ads131a_device_t *device){
    uint8_t _new = 0x0F;
    uint16_t response = 0;

    response = ads131a_write_register(device, ADC_ENA_ADDRESS, _new);
    if(LOWER_BYTE(response) == _new) device->dev.adc_ena.ena = _new;
    return LOWER_BYTE(response);
}

uint8_t ads131a_adcoff(ads131a_device_t *device){
    uint8_t _new = 0x00;
    uint16_t response = 0;

    response = ads131a_write_register(device, ADC_ENA_ADDRESS, _new);
    if(LOWER_BYTE(response) == _new) device->dev.adc_ena.ena = _new;
    return LOWER_BYTE(response);
}


uint16_t ads131a_configure_device(const ads131a_device_t *device){
    uint8_t ch_nums = 0, regs_num = 0, reg_val = 0;
    uint16_t response = 0;
    
    uint8_t *ptr = (uint8_t*)&device->dev.a_sys_cfg.val;

    ch_nums = ll_ads_get_device_channel(device);
    regs_num = (ADC_ENA_ADDRESS - A_SYS_CFG_ADDRESS) + 1 + ch_nums + 1;  // 区分 ads131a02 和 ads131a04

    for(int i = 0; i < regs_num; i++){
        reg_val = (uint8_t)*(ptr + i);
        response = ads131a_write_register(device, A_SYS_CFG_ADDRESS + i, reg_val);
        if(response != reg_val){
            ESP_LOGE(TAG, "write register occur error!");
            continue;
        }
    }
    return response;
}


uint16_t ads131a_get_channel_udata(ads131a_device_t *device, ads131a_udata_t *ptr){
    uint8_t fl = 1, wb = 4, crc_mode = 0, crc_en = 0;
    uint8_t b_crc[2][3][2] = {
            {
                /* word bits = 16 */{0x1D, 0x0F}, /* word bits = 24 */{0xCC, 0x9C}, /*word bits = 32*/{0x84, 0xC0}    // crc mode = 0
            },
            {
                {0xE1, 0x39}, {0x4E, 0xC3}, {0xF6, 0xB8}    // crc mode = 1
            }
    };
#if STATIC_BUFFER
    uint8_t tx_buffer[6 * 4] = { 0 }, rx_buffer[6 * 4] = { 0 };
#else
    uint8_t *tx_buffer = NULL, *rx_buffer = NULL;
#endif
    wb = ll_ads_get_word_bits(device) >> 3;
    fl = frame_length(device);
    crc_mode = ll_ads_get_crc_mode(device);
    crc_en = ll_ads_is_crc_en(device);
//    ESP_LOGD(TAG, "frame length=%d, word size=%d\r\n", fl, wb);
#if STATIC_BUFFER
//    memset(tx_buffer, 0, sizeof(tx_buffer));
//    memset(rx_buffer, 0, sizeof(rx_buffer));
#else
    tx_buffer = (uint8_t*)malloc(fl * wb);
    rx_buffer = (uint8_t*)malloc(fl * wb);
    memset(tx_buffer, 0, fl * wb);
    memset(rx_buffer, 0, fl * wb);
#endif
    tx_buffer[0] = UPPER_BYTE(OPCODE_NULL);
    tx_buffer[1] = LOWER_BYTE(OPCODE_NULL);

    if(crc_en){
        tx_buffer[(fl - 1) * wb] = b_crc[crc_mode][wb - 2][0];
        tx_buffer[(fl - 1) * wb + 1] = b_crc[crc_mode][wb - 2][1];
    }

    ads131a_dataframe_t dataframe = {.tx_ptr = tx_buffer,
            .rx_ptr = rx_buffer,
            .dataframe_size = fl * wb,
            .fixed = ll_ads_is_fixed_mode(device),
            .word_bits = wb << 3};
    esp_err_t err = ads131a_drv_dataframe(&dataframe);      // 读取数据
    if(err != ESP_OK) goto return_section;
    for(int i = 0; i < ll_ads_get_device_channel(device); i++){
        uint32_t td = 0;
        for(int j = 0; j < (wb); j++){
            td |= ((uint32_t)dataframe.rx_ptr[(i + 1) * wb + j]) << ((wb - j - 1) << 3);
        }
//        ptr->channel_data[i] = (td >> ((wb - 2) ? 8 : 16));
        ptr->channel_data[i] = td;
    }
    ptr->rawcrc = COMBINE_BYTES(dataframe.rx_ptr[(fl - 1) * wb], dataframe.rx_ptr[(fl - 1) * wb + 1]);
    device->dev.stat_1.val = LOWER_BYTE(dataframe.response);
//    if(crc_en){
//        ptr->calcrc = calculate_crc(dataframe.rx_ptr, fl * wb, 0xFFFF);
//    }else{
//        ptr->calcrc = 0;
//    }
#if (!STATIC_BUFFER)
    free(tx_buffer);
    free(rx_buffer);
#endif
    return_section:
    return dataframe.response;
}

uint16_t ads131a_get_channel_idata(ads131a_device_t *device, ads131a_idata_t *idata){
    ads131a_udata_t udata;
    uint16_t response = 0;
    uint8_t wB = ll_ads_get_word_bits(device);
    uint8_t hamming = ll_ads_is_hamming_code_enable(device);
    uint8_t ch = ll_ads_get_device_channel(device);
    uint8_t shift_bit = 0;
    response = ads131a_get_channel_udata(device, &udata);
    if(((hamming == 1) && (wB == 24)) || ((hamming == 0) && (wB == 16))) shift_bit = 16;
    else if(((hamming == 1) && (wB == 32)) || ((hamming == 0) && (wB == 24))) shift_bit = 8;
    else shift_bit = 8;
    for(int i = 0; i < ch; i++){
        idata->channel_data[i] = ((int32_t)udata.channel_data[i]) >> shift_bit;
    }
    return response;
}

void ads131a_get_channel_voltage(const ads131a_device_t *device, const ads131a_udata_t *udata, ads131a_fdata_t *fdata){

}

uint16_t ads131a_get_device_id(ads131a_device_t* device){
    uint8_t reg_val = 0;
    uint16_t id = 0;
    reg_val = ads131a_read_register(device, ID_MSB_ADDRESS);
    id = reg_val;
    id <<= 8;
    reg_val = ads131a_read_register(device, ID_LSB_ADDRESS);
    id |= reg_val;
    // device->dev.id_msb.val = reg_val;
    return id;
}


uint16_t ads131a_get_device_status(ads131a_device_t* device){
#if(0)
    uint8_t regs_num = (STAT_M2_ADDRESS - STAT_1_ADDRESS + 1);
    uint16_t opc = OPCODE_RREGS | ((uint16_t)STAT_1_ADDRESS << 8) | (regs_num - 1);
    uint16_t response = 0;

    ads131a_send_command(device, opc);
    response = ads131a_get_command_response(device);   // return ACK
    if(response == (opc + 0x4000)){
        for(int i = 0; i < (regs_num >> 1); i++){
            response = ads131a_get_command_response(device);   // return state register value
            *(((uint8_t*)&device->state.state1) + i * 2) = UPPER_BYTE(response);
            *(((uint8_t*)&device->state.state1) + i * 2 + 1) = LOWER_BYTE(response);
        }
    }else return 0;
#endif
    uint16_t response = 0;
    uint8_t reg_val = 0;
    for(int i = 0; i < 6; i++){
        reg_val = ads131a_read_register(device, STAT_1_ADDRESS + i);
    }
    return response;
}

uint16_t ads131a_get_device_configure(ads131a_device_t *device){
#if(0)
    uint8_t ch_nums = 0, regs_num = 0;
    uint16_t opc = 0, response = 0;

    ch_nums = ADS131A_GET_CHANNEL_NUMBERA(device);
    regs_num = (ADC_ENA_ADDRESS - A_SYS_CFG_ADDRESS) + 1 + ch_nums + 1;  // 区分 ads131a02 和 ads131a04
    opc = OPCODE_RREGS | ((uint16_t)A_SYS_CFG_ADDRESS << 8) | (regs_num - 1);

    ads131a_send_command(device, opc);
    response = ads131a_get_command_response(device);
    if(response == (opc + 0x4000)){
        for(int i = 0; i < (regs_num >> 1); i++){
            response = ads131a_get_command_response(device);
            *(uint8_t*)(&device->config.analog_config + i * 2) = UPPER_BYTE(response);
            *(uint8_t*)(&device->config.analog_config + i * 2 + 1) = LOWER_BYTE(response);
        }
    }
    return response;
#endif
    uint8_t reg = 0, reg_val = 0, ch_nums = 0;
    uint16_t response = 0;
    
    ch_nums = ads131a_get_device_channel(device);
    
    for(int i = 0; i < (0x10 - 0x0A + 1 + ch_nums); i++){
        reg = 0x0A + i;
        reg_val = ads131a_read_register(device, reg);
    }

    return response;

}

