/***************************************************************
** Copyright (C), 2010-2020, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - alsps_stk3a6x_island.c
** Description: Source file for oplus alsps.
** Version: 1.0
** Date : 2020/07/20
**
** ---------------------Revision History: ---------------------
** <author>                     <date>      <version>    <desc>
****************************************************************/
#ifdef SUPPORT_STK32600
#include "sns_rc.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_service_manager.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"
#include "sns_stream_service.h"
#include "sns_async_com_port.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_std_sensor.pb.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_timer.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_printf.h"
#include "sns_cal.pb.h"
#include "sns_sensor_util.h"

#include "alsps_stk3a6x.h"
#include "sns_alsps_sensor.h"
#include "oplus_alsps.h"

#ifdef OPLUS_FEATURE_SENSOR_FB
#include "oplus_list.h"
#endif


//#define ENABLE_FIFO
//#define STK_FIFO_GET_MAX_FRAME 256

static stk3x6a_driver_state stk_state = {
    .first_ps = false,
    .first_als = false,
    .als_gain = 16,
    .last_data_c = 0,
    .last_als = 0,
    .last_data_g = 0,
    .als_gain_level = 1,
    .last_c1 = 0,
    .last_c2 = 0,
    .last_ir = 0,
    .publish_sensors = 0,
    .offset = 0
};

typedef struct stk3a6x_register_table {
    uint8_t address;
    uint8_t value;
    uint8_t mask;
} stk3a6x_register_table;

/** Need to use ODR table. */
stk3a6x_register_table stk3a6x_default_register_table[] = {
    // PS DGain x 8, PS IT 1.536ms
    {STK3A6X_REG_PSCTRL, (STK3A6X_PS_PRS1 | STK3A6X_PS_GAIN8 | STK3A6X_PS_IT1600),   0xFF},
    // ALS IT 25ms
    {STK3A6X_REG_ALSCTRL1, (STK3A6X_ALS_PRS1 | STK3A6X_ALS_GAIN16 | STK3A6X_ALS_IT25), 0xFF},
    // LED 37.5mA
    {STK3A6X_REG_LEDCTRL,  STK3A6X_LED_43MA,                                           0xFF},
    // WAIT 6ms
    {STK3A6X_REG_WAIT,     STK3A6X_WAIT6,                                              0xFF},
    // ALS C2 & C1 DGain x16
    {STK3A6X_REG_GAINCTRL, 0x20,                                                       0xFF},
    // ALS C2 & C1 AGain x 2, PS AGain x 1
    {STK3A6X_REG_AGCTRL,   0x01,                                                       0xFF},
    // Enable multipluse, 24us/cyc
    {0x81,                 0x60,                                                       0xFF},
    // BGIR
    {0xA0,                 0x10,                                                       0xFF},
    // PS OFF Threshold
    {0xA8, ((STK3A6X_PS_OFF_THRESHOLD & 0xFF00) >> 8),                   0xFF},
    // PS OFF Threshold
    {0xA9, (STK3A6X_PS_OFF_THRESHOLD & 0x00FF),                        0xFF},
    // BGIR Threshold
    {0xAA,                 STK3A6X_PS_BGIR_THRESHOLD,                                  0x7F},
#ifdef ENABLE_FIFO
    //FIFO Stream mode, ALS+C1+C2+PS mode
    {0x60,                 0x00,                                                       0xFF},
    // Enable IT2 ALS, ALS IT 480us
    // {0x6F,              0x83,                                                       0xFF},
#endif
    // 2^3-1 = 7 ALS + PS
    {0x46,                 0x30,                                                       0xFF},
    // ALS = 2 pd
    {0xF1,                 0x02,                                                       0xFF},
};

stk3a6x_register_table stk3a6x_default_als_thd_table[] = {
    {STK3A6X_REG_THDH1_ALS, 0x00, 0xFF},
    {STK3A6X_REG_THDH2_ALS, 0x00, 0xFF},
    {STK3A6X_REG_THDL1_ALS, 0xFF, 0xFF},
    {STK3A6X_REG_THDL2_ALS, 0xFF, 0xFF},
};

stk3a6x_register_table stk3a6x_default_ps_thd_table[] = {
    {STK3A6X_REG_THDH1_PS,  0x00, 0xFF},
    {STK3A6X_REG_THDH2_PS,  0x00, 0xFF},
    {STK3A6X_REG_THDL1_PS,  0xFF, 0xFF},
    {STK3A6X_REG_THDL2_PS,  0xFF, 0xFF},
};

stk3a6x_register_table stk3a6x_default_otp_table[] = {
    {0x93,  0x01,   0x01},
    {0x90,  0x11,   0x3f},
    {0x92,  0x01,   0x02},
    {0x93,  0x00,   0x01},
};

uint8_t stk3a6x_pid_list[STK3A6X_PID_LIST_NUM] = {0x21, 0x2E};

static bool set_als_gain(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint8_t level);

static sns_rc stk3a6x_com_read_wrapper(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint32_t reg_addr,
    uint8_t *buffer,
    uint32_t bytes,
    uint32_t *xfer_bytes)
{
    sns_rc ret;
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = false;
    port_vec.reg_addr = reg_addr;
    ret = scp_service->api->sns_scp_register_rw(port_handle,
            &port_vec,
            1,
            false,
            xfer_bytes);
    #ifdef OPLUS_FEATURE_SENSOR_FB
    struct fb_event fb_event;
    memset(&fb_event, 0, sizeof(struct fb_event));
    if (ret != SNS_RC_SUCCESS || *xfer_bytes != bytes) {
        fb_event.event_id = PS_I2C_ERR_ID;
        fb_event.buff[0] = 0; //read
        fb_event.buff[1] = (int)reg_addr;
        oplus_add_fd_event(&fb_event);
    }
    #endif
    return ret;
}

static sns_rc stk3a6x_com_write_wrapper(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint32_t reg_addr,
    uint8_t *buffer,
    uint32_t bytes,
    uint32_t *xfer_bytes,
    bool save_write_time)
{
    sns_rc ret;
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = true;
    port_vec.reg_addr = reg_addr;
    ret = scp_service->api->sns_scp_register_rw(port_handle,
            &port_vec,
            1,
            save_write_time,
            xfer_bytes);
    #ifdef OPLUS_FEATURE_SENSOR_FB
    struct fb_event fb_event;
    memset(&fb_event, 0, sizeof(struct fb_event));
    if (ret != SNS_RC_SUCCESS || *xfer_bytes != bytes) {
        fb_event.event_id = PS_I2C_ERR_ID;
        fb_event.buff[0] = 1; //write
        fb_event.buff[1] = (int)reg_addr;
        oplus_add_fd_event(&fb_event);
    }
    #endif
    return ret;
}

sns_rc stk3a6x_read_modify_write(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint32_t reg_addr,
    uint8_t *reg_value,
    uint32_t size,
    uint32_t *xfer_bytes,
    bool save_write_time,
    uint8_t mask)
{
    uint8_t rw_buffer = 0;
    uint32_t rw_bytes = 0;

    if ((size > 1) || (mask == 0xFF) || (mask == 0x00)) {
        stk3a6x_com_write_wrapper(scp_service,
            port_handle,
            reg_addr,
            &reg_value[0],
            size,
            xfer_bytes,
            save_write_time);
    } else {
        // read current value from this register
        stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            reg_addr,
            &rw_buffer,
            1,
            &rw_bytes);
        // generate new value
        rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);
        // write new value to this register
        stk3a6x_com_write_wrapper(scp_service,
            port_handle,
            reg_addr,
            &rw_buffer,
            1,
            xfer_bytes,
            save_write_time);
    }

    return SNS_RC_SUCCESS;
}

sns_rc stk3a6x_set_waittime(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, int val)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t buffer[1] = {0};
    uint32_t xfer_bytes = 0;

    buffer[0] = val;

    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            STK3A6X_REG_WAIT,
            &buffer[0],
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        return rv;
    }

    stk3a6x_com_read_wrapper(scp_service,
        port_handle,
        STK3A6X_REG_WAIT,
        &buffer[0],
        1,
        &xfer_bytes);

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_set_waittime, after 0x%x \n", buffer[0]);

    return SNS_RC_SUCCESS;
}

uint8_t stk3a6x_get_mid(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  buffer;
    uint32_t xfer_bytes;

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            0xE0,
            &buffer,
            1,
            &xfer_bytes);

    return buffer;
}

sns_rc stk3a6x_get_who_am_i(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t buffer = 0;
    int count = 0;

    do {
        sns_busy_wait(sns_convert_ns_to_ticks(20 * 1000 * 1000));
        rv = stk3a6x_com_read_wrapper(scp_service,
                port_handle,
                STK3A6X_REG_PDT_ID,
                &buffer,
                1,
                &xfer_bytes);
        SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_get_who_am_i rv=%d, i=%d\n", rv, count);

        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
            rv = SNS_RC_FAILED;
        }

        if (buffer != 0x21) {
            rv = SNS_RC_FAILED;
        } else {
            rv = SNS_RC_SUCCESS;
        }
    } while ((++count <= 3) && (rv == SNS_RC_FAILED));

    return rv;
}

sns_rc stk3a6x_device_set_default_state(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, alsps_sensor_type sensor)
{
    UNUSED_VAR(sensor);
    uint8_t buffer[1];
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint16_t reg_count = 0;
    uint16_t reg_num = sizeof(stk3a6x_default_register_table) / sizeof(stk3a6x_register_table);

    for (reg_count = 0; reg_count < reg_num; reg_count++) {
        buffer[0] = stk3a6x_default_register_table[reg_count].value;
        rv = stk3a6x_read_modify_write(scp_service,
                port_handle,
                stk3a6x_default_register_table[reg_count].address,
                &buffer[0],
                1,
                &xfer_bytes,
                false,
                stk3a6x_default_register_table[reg_count].mask);

        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
            return SNS_RC_FAILED;
        }
    }

    return rv;
}

sns_rc stk3a6x_reset_device(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, alsps_sensor_type sensor)
{
    sns_rc rv = SNS_RC_SUCCESS;

    rv = stk3a6x_device_set_default_state(scp_service, port_handle, sensor);

    return rv;
}

void stk3a6x_reconfig_reg(uint8_t reg_num, uint8_t *reg_table)
{
    for (int i = 0; i < reg_num / 2; i++) {
        for (int j = 0; j < ARR_SIZE(stk3a6x_default_register_table); j++) {
            if (stk3a6x_default_register_table[j].address == reg_table[i * 2]) {
                OPLUS_ALS_PS_LOG("reg_table override: reg = %x, reg_value = [%d -> %d]\n",
                    stk3a6x_default_register_table[j].address,
                    stk3a6x_default_register_table[j].value,
                    reg_table[i * 2 + 1]);
                stk3a6x_default_register_table[j].value = reg_table[i * 2 + 1];
                break;
            }
        }
    }
}

static sns_rc stk3a6x_init_als(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    stk_state.first_als = false;
    stk_state.first_ps = false;
    stk_state.als_gain = 16; //128
    stk_state.als_gain_level = 1; //0
    //first init  reset the device
    stk3a6x_reset_device(scp_service, port_handle, ALS);

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_init_als \n");

    return 0;
}

static sns_rc stk3a6x_deinit_als()
{
    //SNS_PRINTF(ERROR, sns_fw_printf,"stk3a6x_deinit_als");
    return 0;
}

sns_rc stk3a6x_set_offset(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, int offset)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t buffer[2] = {0};
    uint32_t xfer_bytes = 0;

    stk_state.offset = offset;

    if (offset > 0) {
        offset |= 0x8000;
    } else {
        offset = -offset;
    }

    buffer[0] = ((offset & 0xFF00) >> 8);
    buffer[1] = (offset & 0xFF);

    SNS_PRINTF(ERROR, sns_fw_printf, "before buffer0 %x buffer1 %x \n", buffer[0], buffer[1]);

    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            STK3A6X_REG_DATA1_PS_OFFSET,
            &buffer[0],
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        return rv;
    }

    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            STK3A6X_REG_DATA2_PS_OFFSET,
            &buffer[1],
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        return rv;
    }

    sns_busy_wait(sns_convert_ns_to_ticks(3 * 1000 * 1000));

    stk3a6x_com_read_wrapper(scp_service,
        port_handle,
        STK3A6X_REG_DATA1_PS_OFFSET,
        &buffer[0],
        1,
        &xfer_bytes);
    stk3a6x_com_read_wrapper(scp_service,
        port_handle,
        STK3A6X_REG_DATA2_PS_OFFSET,
        &buffer[1],
        1,
        &xfer_bytes);

    SNS_PRINTF(ERROR, sns_fw_printf, "after buffer0 %x buffer1 %x \n", buffer[0], buffer[1]);
    SNS_PRINTF(ERROR, sns_fw_printf, "offset %d \n", ((buffer[0] << 8 | buffer[1]) & 0x7FFF));

    return SNS_RC_SUCCESS;
}

static sns_rc stk3a6x_ps_set_thd(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint16_t ps_thd_near,
    uint16_t ps_thd_far,
    alsps_ps_state status)
{
    UNUSED_VAR(status);
    uint8_t reg_high_thd[2], reg_low_thd[2];
    uint32_t xfer_bytes;
    reg_high_thd[0] = (uint8_t)((ps_thd_near >> 8) & 0xFF);
    reg_high_thd[1] = (uint8_t)(ps_thd_near & 0xFF);
    reg_low_thd[0] = (uint8_t)((ps_thd_far >> 8) & 0xFF);
    reg_low_thd[1] = (uint8_t)(ps_thd_far & 0xFF);

    stk3a6x_read_modify_write(scp_service,
        port_handle,
        STK3A6X_REG_THDH1_PS,
        &reg_high_thd[0],
        2,
        &xfer_bytes,
        false,
        0xFF);
    stk3a6x_read_modify_write(scp_service,
        port_handle,
        STK3A6X_REG_THDL1_PS,
        &reg_low_thd[0],
        2,
        &xfer_bytes,
        false,
        0xFF);

    return 0;
}

static sns_rc stk3a5x_check_reset_reg(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t  read_reg[2], psctrl_val = 0, ledctrl_val = 0;
    bool     reset_reg_setting = false, result;
    uint16_t reg_count = 0, reg_num = sizeof(stk3a6x_default_register_table) / sizeof(
                stk3a6x_register_table);

    for (reg_count = 0; reg_count < reg_num; reg_count++) {
        if (stk3a6x_default_register_table[reg_count].address == STK3A6X_REG_PSCTRL) {
            psctrl_val = stk3a6x_default_register_table[reg_count].value;
        } else if (stk3a6x_default_register_table[reg_count].address == STK3A6X_REG_LEDCTRL) {
            ledctrl_val = stk3a6x_default_register_table[reg_count].value;
        }
        if ((psctrl_val != 0) && (ledctrl_val != 0)) {
            break;
        }
    }
    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_check_reset_reg: psctrl_val=%d, ledctrl_val=%d",
        psctrl_val, ledctrl_val);

    if ((psctrl_val != 0) && (ledctrl_val != 0)) {
        rv = stk3a6x_com_read_wrapper(scp_service,
                port_handle,
                STK3A6X_REG_PSCTRL,
                &read_reg[0],
                1,
                &xfer_bytes);
        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
            rv = SNS_RC_FAILED;
            return rv;
        }

        rv = stk3a6x_com_read_wrapper(scp_service,
                port_handle,
                STK3A6X_REG_LEDCTRL,
                &read_reg[1],
                1,
                &xfer_bytes);
        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
            rv = SNS_RC_FAILED;
            return rv;
        }

        if ((read_reg[0] == 0) || (read_reg[0] != psctrl_val)) {
            SNS_PRINTF(ERROR, sns_fw_printf,
                "stk3a5x_check_reset_reg: Reg[0x01] = 0x%x, Not equal to psctrl_val= 0x%x", read_reg[0],
                psctrl_val);
            reset_reg_setting = true;
        }

        if ((read_reg[1] == 0) || (read_reg[1] != ledctrl_val)) {
            SNS_PRINTF(ERROR, sns_fw_printf,
                "stk3a5x_check_reset_reg: Reg[0x03] = 0x%x, Not equal to ledctrl_val= 0x%x", read_reg[1],
                ledctrl_val);
            reset_reg_setting = true;
        }

        SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_check_reset_reg: reset_reg_setting=%d",
            reset_reg_setting);
        if (reset_reg_setting) {
            #ifdef OPLUS_FEATURE_SENSOR_FB
            struct fb_event fb_event;
            memset(&fb_event, 0, sizeof(struct fb_event));
            fb_event.event_id = PS_ESD_REST_ID;
            oplus_add_fd_event(&fb_event);
            #endif
            SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_check_reset_reg: Reset all reg setting");
            rv = stk3a6x_device_set_default_state(scp_service, port_handle, ALS);  //reset all reg setting
            result = set_als_gain(scp_service, port_handle, stk_state.als_gain_level); //reset als gain
        }
    }

    return rv;
}

static sns_rc stk3a6x_ps_enable(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, bool enable)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t  i2c_flag_reg;
    bool als_enable = false;

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_ps_enable %d \n", enable);
    rv = stk3a5x_check_reset_reg(scp_service, port_handle); //check reg setting

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_STATE,
            &i2c_flag_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    if (i2c_flag_reg & STK3A6X_STATE_EN_ALS_MASK) {
        als_enable = true;
    }

    i2c_flag_reg &= (~(STK3A6X_STATE_EN_PS_MASK | STK3A6X_STATE_EN_WAIT_MASK));

    if (enable) {
        if (als_enable) {
            stk3a6x_set_waittime(scp_service, port_handle, 0);
            i2c_flag_reg |= STK3A6X_STATE_EN_PS_MASK; // als + ps,  als+0waittime+...(7) + ps
        } else {
            stk3a6x_set_waittime(scp_service, port_handle, STK3A6X_WAIT100); //only ps, ps+100waittime
            i2c_flag_reg |= (STK3A6X_STATE_EN_WAIT_MASK | STK3A6X_STATE_EN_PS_MASK);
        }
        stk_state.publish_sensors |= STK3A6X_PROX;
    } else {
        if (als_enable) {
            //only als , als+0waittime
            stk3a6x_set_waittime(scp_service, port_handle, 0);
            i2c_flag_reg |= STK3A6X_STATE_EN_WAIT_MASK;
        }
        stk_state.publish_sensors &= ~STK3A6X_PROX;
    }

    stk3a6x_read_modify_write(scp_service,
        port_handle,
        STK3A6X_REG_STATE,
        &i2c_flag_reg,
        1,
        &xfer_bytes,
        false,
        0xFF);

    i2c_flag_reg = 0x01;

    stk3a6x_read_modify_write(scp_service,
        port_handle,
        0x5F,
        &i2c_flag_reg,
        1,
        &xfer_bytes,
        false,
        0xFF);

    return rv;
}

static sns_rc stk3a6x_als_enable(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, bool enable)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t  i2c_flag_reg;
#ifdef ENABLE_FIFO
    uint8_t  fifo_ctrl1_reg;
#endif
    bool ps_enable = false;

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_als_enable %d \n", enable);
    rv = stk3a5x_check_reset_reg(scp_service, port_handle); //check reg setting

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_STATE,
            &i2c_flag_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    if (i2c_flag_reg & STK3A6X_STATE_EN_PS_MASK) {
        ps_enable = true;
    }

    i2c_flag_reg &= (~(STK3A6X_STATE_EN_ALS_MASK | STK3A6X_STATE_EN_WAIT_MASK));

    if (enable) {
        stk3a6x_set_waittime(scp_service, port_handle, 0);
        stk_state.first_als = true;
        //only als, als+0waittime+...
        //als+ps , als+0waittime+...+als+0waittime(7 group) + ps
        i2c_flag_reg |= (STK3A6X_STATE_EN_ALS_MASK | STK3A6X_STATE_EN_WAIT_MASK);
        stk_state.publish_sensors |= STK3A6X_ALS;
    } else {
        if (ps_enable) {
            //ps only need enable waittime
            stk3a6x_set_waittime(scp_service, port_handle, STK3A6X_WAIT100);
            i2c_flag_reg |= STK3A6X_STATE_EN_WAIT_MASK;
        }
        stk_state.publish_sensors &= (~STK3A6X_ALS);
    }

    stk3a6x_read_modify_write(scp_service,
        port_handle,
        STK3A6X_REG_STATE,
        &i2c_flag_reg,
        1,
        &xfer_bytes,
        false,
        0xFF);

    stk3a6x_dump_reg(scp_service, port_handle);

    return SNS_RC_SUCCESS;
}

static sns_rc stk3a6x_clear_ps_int(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  i2c_flag_reg;
    uint8_t  valid_int_flag = STK3A6X_FLG_PSINT_MASK | STK3A6X_FLG_ALSINT_MASK;
    uint32_t xfer_bytes;

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_FLAG,
            &i2c_flag_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    i2c_flag_reg &= ~valid_int_flag;

    stk3a6x_read_modify_write(scp_service,
        port_handle,
        STK3A6X_REG_FLAG,
        &i2c_flag_reg,
        1,
        &xfer_bytes,
        false,
        0xFF);

    return rv;
}

static sns_rc stk3a6x_get_ps_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint16_t *raw_data)
{
    int rv = 0;
    uint32_t xfer_bytes = 0;
    uint8_t  data[2];
    uint16_t ps_raw_data_u16 = 0;

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_DATA1_PS,
            &data[0],
            2,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 2) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    ps_raw_data_u16 = (data[0] << 8 | data[1]);
    *raw_data = ps_raw_data_u16;

    SNS_PRINTF(ERROR, sns_fw_printf, "ps raw data = %d \n", ps_raw_data_u16);

    return 0;
}

static bool set_als_gain(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint8_t level)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  alsctrl_reg, gainctrl_reg, agctrl_reg, f1_reg, buffer, gain;
    uint32_t xfer_bytes;
    bool result = true;

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_ALSCTRL1,
            &alsctrl_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_GAINCTRL,
            &gainctrl_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_AGCTRL,
            &agctrl_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    SNS_PRINTF(ERROR, sns_fw_printf,
        "alsctrl_reg = 0x%x, gainctrl_reg = 0x%x, agctrl_reg = 0x%x, gain = %d \n",
        alsctrl_reg, gainctrl_reg, agctrl_reg, stk_state.als_gain);

    if (level == 0) {
        // Highest gain for low light: ALS Gain X 128 & C Gain X 128, Reg[0x02] don't care.
        // Write Reg[0x4E] = 0x06, GAIN_ALS_DX128 = 1, GAIN_C_DX128 = 1
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK3A6X_ALS_GAIN64;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x06; // als & c data dgain x 128
        agctrl_reg = (agctrl_reg & 0xC3) | 0x00; // als & c data again x 2
        f1_reg = 0x02; // als = 2pd
        gain = 128;
    } else if (level == 1) {
        // Middle gain for middle light:  ALS Gain X 16 & C Gain X 16
        // Write Reg[0x02] = 0x21, ALS IT = 50ms, GAIN_ALS = 2' b10
        // Write Reg[0x4E] = 0x20, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b10
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK3A6X_ALS_GAIN16;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x20; //c data dgain x 16
        agctrl_reg = (agctrl_reg & 0xC3) | 0x00; //als&c data again x 2
        f1_reg = 0x02; //als = 2pd
        gain = 16;
    } else if (level == 2) {
        // Lowest gain for high light:  ALS Gain X4 & C Gain X4
        // Write Reg[0x02] = 0x11, ALS IT = 50ms, GAIN_ALS = 2' b01
        // Write Reg[0x4E] = 0x10, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b01
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK3A6X_ALS_GAIN4;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x10; // c data dgain x 4
        agctrl_reg = (agctrl_reg & 0xC3) | 0x28; // als & c data again x 0.5
        f1_reg = 0x00; // als = 1pd
        gain = 4;
    } else {
        return false;
    }

    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            STK3A6X_REG_ALSCTRL1,
            &alsctrl_reg,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        result = false;
    }

    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            STK3A6X_REG_GAINCTRL,
            &gainctrl_reg,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        result = false;
    }

    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            STK3A6X_REG_AGCTRL,
            &agctrl_reg,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        result = false;
    }

    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            0xF1,
            &f1_reg,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        result = false;
    }

    SNS_PRINTF(ERROR, sns_fw_printf,
        "alsctrl_reg = 0x%x, gainctrl_reg = 0x%x, agctrl_reg = 0x%x, f1_reg = 0x%x, gain = %d \n",
        alsctrl_reg, gainctrl_reg, agctrl_reg, f1_reg, gain);

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            0x5F,
            &buffer,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        result = false;
    }

    buffer |= 0x01; //Restart FSM
    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            0x5F,
            &buffer,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        result = false;
    }

    buffer = 0x34;
#ifdef ENABLE_FIFO
    rv = stk3a6x_read_modify_write(scp_service,
            port_handle,
            0x60,
            &buffer,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        result = false;
    }

#endif

    if (result) {
        stk_state.als_gain = gain;
    }

    return result;
}

static bool check_auto_gain(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint16_t *als_data)
{
    bool result = false;

    if (((als_data[0]) > 65000 || (als_data[4] > 65000)) && (stk_state.als_gain_level < 2)) {
        // Reduce gain
        stk_state.als_gain_level++;
        result = set_als_gain(scp_service, port_handle, stk_state.als_gain_level);
    } else if (((als_data[0] < 1500) && (als_data[4] < 1500))
        && (stk_state.als_gain_level > 0)) { //3000
        //Raise gain
        stk_state.als_gain_level--;
        result = set_als_gain(scp_service, port_handle, stk_state.als_gain_level);
    }

    return result;
}

static sns_rc stk3a6x_get_als_data(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    float *data,
    int len,
    uint8_t als_type,
    bool is_als_dri)
{
    UNUSED_VAR(len);
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes = 0;
    uint8_t  raw_data[10];
    uint16_t als_raw_data[5];
    uint32_t ir_data = 0;
    int      loop_count = 0;
    bool     auto_gain = false;

    if (!is_als_dri) {
        // check whether als data is ready or not.
    }

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_DATA1_ALS,
            &raw_data[0],
            10,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 10) {
        rv = SNS_RC_FAILED;
        SNS_PRINTF(ERROR, sns_fw_printf, "read_data fail %d \n", xfer_bytes);
        return rv;
    }

    for (loop_count = 0; loop_count < (sizeof(als_raw_data) / sizeof(als_raw_data[0])); loop_count++) {
        *(als_raw_data + loop_count) =
            (*(raw_data + (2 * loop_count)) << 8 | *(raw_data + (2 * loop_count + 1)));
    }


    SNS_PRINTF(ERROR, sns_fw_printf, "als raw data[5] = [%d, %d, %d, %d, %d] \n",
        als_raw_data[0], als_raw_data[1], als_raw_data[2], als_raw_data[3], als_raw_data[4]);

    if (!stk_state.first_als) {
        SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_get_als_data old gain = %d, level = %d \n",
            stk_state.als_gain, stk_state.als_gain_level);  // for debug
        auto_gain = check_auto_gain(scp_service, port_handle, als_raw_data);
        SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_get_als_data new gain = %d, level = %d \n",
            stk_state.als_gain, stk_state.als_gain_level);  // for debug
    }

    // use als/G to verify
    if (auto_gain) {
        // last data
        data[0] = stk_state.last_als;
        data[2] = stk_state.last_ir;
        data[1] = stk_state.last_ir;
    } else {
        if (als_type == NORMAL) {
            //calculate normal data
        } else {
            if (stk_state.als_gain == 4) {
                // als data = als data * 8.5 for (again 0.5 + pd / 2)
                data[0] = ((als_raw_data[0] * 128 * 85) / 10) / stk_state.als_gain;
                // c2_data / als_raw_data * 1000
                ir_data = (uint32_t)(((float)(als_raw_data[4]) / (float)(als_raw_data[0] * 2)) * 1000);
            } else {
                data[0] = als_raw_data[0] * 128 / stk_state.als_gain; // als data
                // c2_data / als_raw_data * 1000
                ir_data = (uint32_t)(((float)(als_raw_data[4]) / (float)(als_raw_data[0])) * 1000);
            }

            data[1] = ir_data; // (int)(c1 / c2)
            data[2] = ir_data;

            {
                log_data_info log_data;
                uint16_t als_data    = als_raw_data[0];
                uint16_t green_data  = als_raw_data[2];
                uint16_t clear_data  = als_raw_data[4];
                memset(&log_data, 0, sizeof(log_data));
                log_data.sensor_id = SENSOR_TYPE_LIGHT;
                log_data.string_id = ALSPS_LOG_LIGHT_PARAMETER;
                log_data.argu2 = data[0]; //raw_data
                log_data.argu3 = stk_state.als_gain;
                log_data.argu4 = als_data;//full band data:0x13&0x14 register
                log_data.argu5 = green_data;//gdata(550nm):0x17&0x18
                log_data.argu6 = clear_data;//cdata(IR band):0x1B&0x1C
                sensors_log_report(log_data);
            }
        }
    }
    SNS_PRINTF(ERROR, sns_fw_printf, "als_ir_data = %d \n", ir_data);

    if (stk_state.first_als) {
        if (als_raw_data[0] == 65535) {
            uint32_t tem_lux = 0;

            if ((als_raw_data[2] / als_raw_data[4] * 100) > 40) {
                tem_lux = als_raw_data[2] * 10; // cwf
            } else {
                tem_lux = als_raw_data[2] * 40; // warm-toned
            }

            if (data[0] > tem_lux) {
                // use data 0
            } else {
                data[0] = tem_lux;
                SNS_PRINTF(ERROR, sns_fw_printf, "lux is out of range lux = %d \n", data[0]);
            }
        }
    }

    stk_state.last_als = data[0];

    if (stk_state.als_gain == 4) {
        // als data = als data * 8.5 for (again 0.5 + pd / 2)
        stk_state.last_c1 = ((als_raw_data[2] * 128 * 85) / 10) / stk_state.als_gain;
        // c2_data = c2_data * 4.25 for (again 0.5)
        stk_state.last_c2 = ((als_raw_data[4] * 128 * 425) / 100) / stk_state.als_gain;
    } else {
        stk_state.last_c1 = als_raw_data[2] * 128 / stk_state.als_gain;
        stk_state.last_c2 = als_raw_data[4] * 128 / stk_state.als_gain;
    }

    stk_state.last_ir = ir_data;

    SNS_PRINTF(ERROR, sns_fw_printf, "als data = %d, c1_data = %d, c2_data= %d, ir_data = %d \n",
        stk_state.last_als, stk_state.last_c1, stk_state.last_c2, stk_state.last_ir);
    SNS_PRINTF(ERROR, sns_fw_printf, "data[0] = %d, data[1] = %d \n", data[0], data[1]);

    if (stk_state.first_als) {
        stk_state.first_als = false;
        // stk3a6x_set_waittime(scp_service, port_handle, STK3A6X_WAIT6);
        // SNS_PRINTF(ERROR, sns_fw_printf, "first data  set the wait to 6ms \n");
    }

    data[0] = data[0] / 8;

    return rv;
}

#define DEBUG_REG_LEN 36

void stk3a6x_dump_reg(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    uint32_t xfer_bytes;
    uint8_t i = 0;
    uint8_t reg_status[DEBUG_REG_LEN];
    uint8_t stk3a6x_reg_map[DEBUG_REG_LEN] = {
        STK3A6X_REG_STATE,
        STK3A6X_REG_PSCTRL,
        STK3A6X_REG_ALSCTRL1,
        STK3A6X_REG_LEDCTRL,
        STK3A6X_REG_INTCTRL1,
        STK3A6X_REG_WAIT,
        STK3A6X_REG_THDH1_PS,
        STK3A6X_REG_THDH2_PS,
        STK3A6X_REG_THDL1_PS,
        STK3A6X_REG_THDL2_PS,
        STK3A6X_REG_THDH1_ALS,
        STK3A6X_REG_THDH2_ALS,
        STK3A6X_REG_THDL1_ALS,
        STK3A6X_REG_THDL2_ALS,
        STK3A6X_REG_FLAG,
        STK3A6X_REG_DATA1_PS,
        STK3A6X_REG_DATA2_PS,
        STK3A6X_REG_DATA1_ALS,
        STK3A6X_REG_DATA2_ALS,
        STK3A6X_REG_DATA1_ALS1,
        STK3A6X_REG_DATA2_ALS1,
        STK3A6X_REG_DATA1_C,
        STK3A6X_REG_DATA2_C,
        STK3A6X_REG_DATA1_PS_OFFSET,
        STK3A6X_REG_DATA2_PS_OFFSET,
        STK3A6X_REG_PDT_ID,
        STK3A6X_REG_RSRVD,
        STK3A6X_REG_GAINCTRL,
        STK3A6X_REG_AGCTRL,
        0x46,
        0x60,
        0x81,
        0xA0,
        0xA8,
        0xA9,
        0xAA,
    };

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_dump_reg: \n");

    for (i = 0; i < DEBUG_REG_LEN; i++) {
        stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            stk3a6x_reg_map[i],
            &reg_status[i],
            1,
            &xfer_bytes);
        SNS_PRINTF(ERROR, sns_fw_printf, "reg[0x%X] = 0x%X \n", stk3a6x_reg_map[i], reg_status[i]);
    }
}

static sns_rc stk3a6x_init_ps_irq(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  buffer[1];
    uint32_t xfer_bytes;

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_INTCTRL1,
            &buffer[0],
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    buffer[0] |= STK3A6X_INT_PS;

    stk3a6x_read_modify_write(scp_service,
        port_handle,
        STK3A6X_REG_INTCTRL1,
        &buffer[0],
        1,
        &xfer_bytes,
        false,
        0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    return rv;
}

static sns_rc stk3a6x_init_als_irq(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t buffer[1];
    uint32_t xfer_bytes;
    uint16_t reg_num = 0;
    uint16_t reg_count = 0;
    stk3a6x_register_table *selected_table = NULL;

    reg_num = sizeof(stk3a6x_default_als_thd_table) / sizeof(stk3a6x_register_table);
    selected_table = stk3a6x_default_als_thd_table;

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_INTCTRL1,
            &buffer[0],
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    buffer[0] |= STK3A6X_INT_ALS;

    stk3a6x_read_modify_write(scp_service,
        port_handle,
        STK3A6X_REG_INTCTRL1,
        &buffer[0],
        1,
        &xfer_bytes,
        false,
        0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    for (reg_count = 0; reg_count < reg_num; reg_count++) {
        rv = stk3a6x_read_modify_write(scp_service,
                port_handle,
                (selected_table + reg_count)->address,
                &(selected_table + reg_count)->value,
                1,
                &xfer_bytes,
                false,
                (selected_table + reg_count)->mask);

        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
            return SNS_RC_FAILED;
        }
    }

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            0x5F,
            &buffer[0],
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    buffer[0] |= 0x01; // Restart FSM
    stk3a6x_read_modify_write(scp_service,
        port_handle,
        0x5F,
        &buffer[0],
        1,
        &xfer_bytes,
        false,
        0xFF);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    return rv;
}

static bool stk3a6x_prox_need_ir_info()
{
    return true;
}

static sns_rc stk3a6x_get_ps_original_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, int *original_data)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t ps_raw_data[2];

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_ORIGINAL_DATA1_PS,
            &ps_raw_data[0],
            2,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 2) {
        *original_data = 65535;
    } else {
        *original_data = ((ps_raw_data[0] << 8) | ps_raw_data[1]);
    }

    return rv;
}

static sns_rc stk3a6x_get_ir_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, int *ps_off_data)
{
    sns_rc     rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint16_t psoff_data[4] = {0};
    uint8_t  data_buffer[8];

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            0x24,
            &data_buffer[0],
            8,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 8) {
        rv = SNS_RC_FAILED;
        *ps_off_data = 0;
    } else {
        psoff_data[0] = (data_buffer[0] << 8) | data_buffer[1];
        psoff_data[1] = (data_buffer[2] << 8) | data_buffer[3];
        psoff_data[2] = (data_buffer[4] << 8) | data_buffer[5];
        psoff_data[3] = (data_buffer[6] << 8) | data_buffer[7];
        *ps_off_data = psoff_data[0] + psoff_data[1] + psoff_data[2] + psoff_data[3];
    }

    return rv;
}

sns_rc stk3a6x_get_ps_device_irq_mask(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint8_t *mask)
{
    int rv = 0;
    uint8_t status = 0, temp_status = 0, retry_count = 0;
    uint32_t xfer_bytes;
    uint8_t reg_value = 0xFF;
    uint8_t valid_int_flag = STK3A6X_FLG_ALSINT_MASK | STK3A6X_FLG_PSINT_MASK;
    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_get_ps_device_irq_mask \n");

    rv = stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_FLAG,
            &status,
            1,
            &xfer_bytes);

    temp_status = status;

    while ((0 != (temp_status & valid_int_flag)) && (retry_count++ < 8)) {
        if (temp_status & STK3A6X_FLG_ALSINT_MASK) {
            reg_value &= ~STK3A6X_FLG_ALSINT_MASK;
        }

        if (temp_status & STK3A6X_FLG_PSINT_MASK) {
            reg_value &= ~STK3A6X_FLG_PSINT_MASK;
        }

        if (reg_value != 0xFF) {
            stk3a6x_read_modify_write(scp_service,
                port_handle,
                STK3A6X_REG_FLAG,
                &reg_value,
                1,
                &xfer_bytes,
                false,
                0xFF);
        }

        SNS_PRINTF(ERROR, sns_fw_printf,
            "stk3a6x_get_ps_device_irq_mask retry_count = %d, status = %x \n", retry_count, reg_value);
        reg_value = 0xFF;

        stk3a6x_com_read_wrapper(scp_service,
            port_handle,
            STK3A6X_REG_FLAG,
            &temp_status,
            1,
            &xfer_bytes);
        status = (temp_status | status);
    }

    if (retry_count == 8) {
        stk3a6x_clear_ps_int(scp_service, port_handle);
    }

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_get_ps_device_irq_mask status = %x \n", status);

    if (status & STK3A6X_FLG_ALSINT_MASK) {
        *mask |= ALS_INT;
    }

    if (status & STK3A6X_FLG_PSINT_MASK) {
        *mask |= PS_INT;
    }

    return rv;
}

sns_rc stk3a6x_recover_device(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a6x_recover_device,publish_sensors=%d \n", stk_state.publish_sensors);
    rv |= stk3a6x_reset_device(scp_service, port_handle, ALS);
    //if als enable set the als enable
    if (stk_state.publish_sensors & STK3A6X_ALS) {
        stk_state.publish_sensors &= ~STK3A6X_ALS;
        stk3a6x_init_als_irq(scp_service, port_handle);
        rv |= stk3a6x_als_enable(scp_service, port_handle, 1);
    }

    if (stk_state.publish_sensors & STK3A6X_PROX) {
        stk_state.first_ps = true;
        stk_state.publish_sensors &= ~STK3A6X_PROX;
        stk3a6x_set_offset(scp_service, port_handle, stk_state.offset);
        rv |= stk3a6x_ps_enable(scp_service, port_handle, 1);
    }

    return rv;
}


struct alsps_als_operations stk3a6x_als_ops = {
    .get_who_am_i = stk3a6x_get_who_am_i,
    .init_driver = stk3a6x_init_als,
    .als_enable = stk3a6x_als_enable,
    .clear_als_int = NULL,
    .init_irq = stk3a6x_init_als_irq,
    .get_als_data = stk3a6x_get_als_data,
    .dump_reg = stk3a6x_dump_reg,
    .get_als_device_irq_mask = stk3a6x_get_ps_device_irq_mask,
    .deinit_driver = stk3a6x_deinit_als,
    .reconfig_reg_table = stk3a6x_reconfig_reg,
    .get_als_fifo_data = NULL,
    .set_brightness = NULL,
    .enable_fifo = NULL
};

struct alsps_ps_operations stk3a6x_ps_ops = {
    .get_who_am_i = stk3a6x_get_who_am_i,
    .init_driver = stk3a6x_init_als,
    .ps_enable = stk3a6x_ps_enable,
    .clear_ps_int = stk3a6x_clear_ps_int,
    .init_irq = stk3a6x_init_ps_irq,
    .get_ps_data = stk3a6x_get_ps_data,
    .ps_set_thd = stk3a6x_ps_set_thd,
    .dump_reg = stk3a6x_dump_reg,
    .set_offset = stk3a6x_set_offset,
    .get_ps_device_irq_mask = stk3a6x_get_ps_device_irq_mask,
    .deinit_driver = stk3a6x_deinit_als,
    .prox_need_ir_info = stk3a6x_prox_need_ir_info,
    .get_ps_original_data = stk3a6x_get_ps_original_data,
    .get_ir_data = stk3a6x_get_ir_data,
    .hardware_cali = NULL,
    .recover_device = stk3a6x_recover_device,
    .reconfig_reg_table = stk3a6x_reconfig_reg,
    .special_process_before_avaliable = NULL,
    .ps_offset_cali = NULL
};

#endif
