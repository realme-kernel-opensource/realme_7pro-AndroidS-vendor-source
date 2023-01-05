/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - xxx
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/

#ifdef SUPPORT_STK33502
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

#include "alsps_stk3a5x.h"
#include "sns_alsps_sensor.h"
#include "oplus_alsps.h"
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif

//#define OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME 2

static stk3x5a_driver_state stk_state = {
    .first_ps = false,
    .first_als = false,
    .als_gain = 128,
    .last_data_c = 0,
    .last_als = 0,
    .last_data_g = 0,
    .als_gain_level = 0,
    .als_ratio_type = 0, //the default must 0
    .als_coef = {
        .A = 114.6f,
        .B = 500.4f,
        .C = 25.8,
        .D = 77.8f,
        .E = 115.2,
        .ratio = 50.0f
    },
    .ps_waittime  =  STK3A5X_WAIT200,
    .als_waittime =  STK3A5X_WAIT100,
};

typedef struct stk3a5x_register_table {
    uint8_t address;
    uint8_t value;
    uint8_t mask;
} stk3a5x_register_table;

/** Need to use ODR table. */
stk3a5x_register_table stk3a5x_default_register_table[] = {
    {STK3A5X_REG_PSCTRL,            (STK3A5X_PS_PRS1 | STK3A5X_PS_GAIN8 | STK3A5X_PS_IT400),    0xFF},
    {STK3A5X_REG_ALSCTRL1,          (STK3A5X_ALS_PRS1 | STK3A5X_ALS_GAIN64 | STK3A5X_ALS_IT100),  0xFF},
    {STK3A5X_REG_LEDCTRL,           STK3A5X_LED_150MA,                                          0xFF},
#if OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 1
    {STK3A5X_REG_WAIT,              (STK3A5X_WAIT230),                                          0xFF},
#elif OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 2
    {STK3A5X_REG_WAIT,              (STK3A5X_WAIT200),                                          0xFF},
#else
    {STK3A5X_REG_WAIT,              (STK3A5X_WAIT100),                                          0xFF},
#endif
    {STK3A5X_REG_GAINCTRL,          0x06,                                                       0xFF},//als dgain x128
    {STK3A5X_REG_AGCTRL,            0x01,                                                       0xFF},//als again x2 ps x1
    {0x81,                          0x60,                                                       0xFF},//multi-pulse
    {0xA0,                          0x10,                                                       0xFF},
    {0xA1,                          0x7F,                                                       0xFF},
    {0xA8,                          ((STK3A5X_PS_OFF_THRESHOLD & 0xFF00) >> 8),                   0xFF},
    {0xA9,                          (STK3A5X_PS_OFF_THRESHOLD & 0x00FF),                        0xFF},
    {0xAA,                          STK3A5X_PS_BGIR_THRESHOLD,                                  0x7F},
    {0xF6,                          0x82,                                                       0xFF},
    {0x45,                          0x20,                                                       0xFF}//use pd2 to check high ir light
//  {0xA4,                          0x09,                                                       0xFF},
};

stk3a5x_register_table stk3a5x_default_als_thd_table[] = {
    {STK3A5X_REG_THDH1_ALS, 0x00, 0xFF},
    {STK3A5X_REG_THDH2_ALS, 0x00, 0xFF},
    {STK3A5X_REG_THDL1_ALS, 0xFF, 0xFF},
    {STK3A5X_REG_THDL2_ALS, 0xFF, 0xFF},
};

stk3a5x_register_table stk3a5x_default_ps_thd_table[] = {
    {STK3A5X_REG_THDH1_PS,  0x00, 0xFF},
    {STK3A5X_REG_THDH2_PS,  0x00, 0xFF},
    {STK3A5X_REG_THDL1_PS,  0xFF, 0xFF},
    {STK3A5X_REG_THDL2_PS,  0xFF, 0xFF},
};

stk3a5x_register_table stk3a5x_default_otp_table[] = {
    {0x93,  0x01,   0x01},
    {0x90,  0x11,   0x3f},
    {0x92,  0x01,   0x02},
    {0x93,  0x00,   0x01},
};

uint8_t stk3a5x_pid_list[STK3A5X_PID_LIST_NUM] = {0x51};
bool low_light_compensation = false;

static sns_rc stk3a5x_com_read_wrapper(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint32_t reg_addr,
    uint8_t *buffer,
    uint32_t bytes,
    uint32_t *xfer_bytes)
{
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = false;
    port_vec.reg_addr = reg_addr;
    return scp_service->api->sns_scp_register_rw(port_handle,
            &port_vec,
            1,
            false,
            xfer_bytes);
}

static sns_rc stk3a5x_com_write_wrapper(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint32_t reg_addr,
    uint8_t *buffer,
    uint32_t bytes,
    uint32_t *xfer_bytes,
    bool save_write_time)
{
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = true;
    port_vec.reg_addr = reg_addr;
    return scp_service->api->sns_scp_register_rw(port_handle,
            &port_vec,
            1,
            save_write_time,
            xfer_bytes);
}

sns_rc stk3a5x_read_modify_write(
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
        stk3a5x_com_write_wrapper(scp_service,
            port_handle,
            reg_addr,
            &reg_value[0],
            size,
            xfer_bytes,
            save_write_time);
    } else {
        // read current value from this register
        stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            reg_addr,
            &rw_buffer,
            1,
            &rw_bytes);
        // generate new value
        rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);
        // write new value to this register
        stk3a5x_com_write_wrapper(scp_service,
            port_handle,
            reg_addr,
            &rw_buffer,
            1,
            xfer_bytes,
            save_write_time);
    }

    return SNS_RC_SUCCESS;
}

sns_rc stk3a5x_set_waittime(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    int val)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t buffer[1] = {0};
    uint32_t xfer_bytes = 0;

    buffer[0] = val;

    rv = stk3a5x_read_modify_write(
            scp_service,
            port_handle,
            STK3A5X_REG_WAIT,
            &buffer[0],
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        return rv;
    }

    stk3a5x_com_read_wrapper(
        scp_service,
        port_handle,
        STK3A5X_REG_WAIT,
        &buffer[0],
        1,
        &xfer_bytes);

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_set_waittime WAIT after 0x%x", buffer[0]);
    return SNS_RC_SUCCESS;
}

uint8_t stk3a5x_get_mid(sns_sync_com_port_service * scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  buffer;
    uint32_t xfer_bytes;
    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            0xE0,
            &buffer,
            1,
            &xfer_bytes);
    return buffer;
}


sns_rc stk3a5x_get_who_am_i(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t buffer = 0;
    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_PDT_ID,
            &buffer,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
    }

    if (buffer != 0x51) {
        rv = SNS_RC_FAILED;
    } else {
        rv = SNS_RC_SUCCESS;
        stk_state.als_ratio_type = 0;//must set to defaut 0

        #ifdef ALSPS_GET_PARAMETER_FROM_SMEM
        struct sensor_hw *als_hw = NULL;
        oppo_get_sensor_hw(OPPO_LIGHT, STK3A5X, &als_hw);

        if (als_hw != NULL) {
            stk_state.als_ratio_type  = als_hw->feature.feature[8];
            stk_state.als_coef.A      = als_hw->feature.parameter[0] * 1.0 / 100;
            stk_state.als_coef.B      = als_hw->feature.parameter[1] * 1.0 / 100;
            stk_state.als_coef.C      = als_hw->feature.parameter[2] * 1.0 / 100;
            stk_state.als_coef.D      = als_hw->feature.parameter[3] * 1.0 / 100;
            stk_state.als_coef.E      = als_hw->feature.parameter[4] * 1.0 / 100;
            stk_state.als_coef.ratio  = als_hw->feature.parameter[5] * 1.0 / 100;
            if((als_hw->feature.parameter[6] != 0) || (als_hw->feature.parameter[7] != 0)) {
                stk_state.ps_waittime     = (uint8_t)als_hw->feature.parameter[6];
                stk_state.als_waittime    = (uint8_t)als_hw->feature.parameter[7];
            }
        }
        #endif
        SNS_PRINTF(ERROR, sns_fw_printf, "als_ratio_type=%d, A=%f, B=%f", stk_state.als_ratio_type, stk_state.als_coef.A, stk_state.als_coef.B);
        SNS_PRINTF(ERROR, sns_fw_printf, "C=%f, D=%f, E=%f", stk_state.als_coef.C, stk_state.als_coef.D, stk_state.als_coef.E);
        SNS_PRINTF(ERROR, sns_fw_printf, "ps_waittime=%d, als_waittime=%d", stk_state.ps_waittime, stk_state.als_waittime);
    }

    return rv;
}

sns_rc stk3a5x_device_set_default_state(sns_sync_com_port_service * scp_service,
    sns_sync_com_port_handle *port_handle,
    alsps_sensor_type sensor)
{
    UNUSED_VAR(sensor);
    uint8_t buffer[1];
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint16_t reg_count = 0;
    uint16_t  reg_num = sizeof(stk3a5x_default_register_table) / sizeof(stk3a5x_register_table);

    for (reg_count = 0; reg_count < reg_num; reg_count++) {
        buffer[0] = stk3a5x_default_register_table[reg_count].value;
        rv = stk3a5x_read_modify_write(scp_service,
                port_handle,
                stk3a5x_default_register_table[reg_count].address,
                &buffer[0],
                1,
                &xfer_bytes,
                false,
                stk3a5x_default_register_table[reg_count].mask);

        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
            return SNS_RC_FAILED;
        }
    }

    return rv;
}

sns_rc stk3a5x_reset_device(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    alsps_sensor_type sensor)
{
    sns_rc rv = SNS_RC_SUCCESS;

    rv = stk3a5x_device_set_default_state(scp_service, port_handle, sensor);

    return rv;
}

static sns_rc stk3a5x_init_als(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    stk_state.first_als = false;
    stk_state.first_ps = false;
    stk_state.als_gain = 128;
    stk_state.als_gain_level = 0;
    low_light_compensation = false;
    //first init  reset the device
    stk3a5x_reset_device(scp_service, port_handle, ALS);

    SNS_PRINTF(ERROR, sns_fw_printf, "als_ratio_type=%d, A=%f, B=%f", stk_state.als_ratio_type, stk_state.als_coef.A, stk_state.als_coef.B);
    SNS_PRINTF(ERROR, sns_fw_printf, "C=%f, D=%f, E=%f", stk_state.als_coef.C, stk_state.als_coef.D, stk_state.als_coef.E);

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_init_als");

    return 0;
}
static sns_rc stk3a5x_deinit_als()
{
    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_deinit_als");
    return 0;
}

sns_rc stk3a5x_set_offset(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    int offset)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t buffer[2] = {0};
    uint32_t xfer_bytes = 0;

    //the first bit stand by +-
    if (offset > 0) {
        offset |= 0x8000;
    } else {
        offset = -offset;
    }

    buffer[0] = ((offset & 0xFF00) >> 8);
    buffer[1] = (offset & 0xFF);

    SNS_PRINTF(ERROR, sns_fw_printf, "before buffer0 %x buffer1 %x",
        buffer[0], buffer[1]);

    rv = stk3a5x_read_modify_write(
            scp_service,
            port_handle,
            STK3A5X_REG_DATA1_PS_OFFSET,
            &buffer[0],
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        return rv;
    }

    rv = stk3a5x_read_modify_write(
            scp_service,
            port_handle,
            STK3A5X_REG_DATA2_PS_OFFSET,
            &buffer[1],
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        return rv;
    }

    sns_busy_wait(sns_convert_ns_to_ticks(3 * 1000 * 1000));

    stk3a5x_com_read_wrapper(
        scp_service,
        port_handle,
        STK3A5X_REG_DATA1_PS_OFFSET,
        &buffer[0],
        1,
        &xfer_bytes);

    stk3a5x_com_read_wrapper(
        scp_service,
        port_handle,
        STK3A5X_REG_DATA2_PS_OFFSET,
        &buffer[1],
        1,
        &xfer_bytes);
    SNS_PRINTF(ERROR, sns_fw_printf, "after buffer0 %x buffer1 %x", buffer[0], buffer[1]);

    SNS_PRINTF(ERROR, sns_fw_printf, "offset %d", ((buffer[0] << 8 | buffer[1]) & 0x7FFF));
    return SNS_RC_SUCCESS;
}

static sns_rc stk3a5x_ps_set_thd(sns_sync_com_port_service *scp_service,
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

    stk3a5x_read_modify_write(scp_service,
        port_handle,
        STK3A5X_REG_THDH1_PS,
        &reg_high_thd[0],
        2,
        &xfer_bytes,
        false,
        0xFF);
    stk3a5x_read_modify_write(scp_service,
        port_handle,
        STK3A5X_REG_THDL1_PS,
        &reg_low_thd[0],
        2,
        &xfer_bytes,
        false,
        0xFF);
    return 0;
}

static sns_rc stk3a5x_ps_enable(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    bool enable)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t  i2c_flag_reg;
    bool als_enable = false;

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_ps_enable %d", enable);

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_STATE,
            &i2c_flag_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    if (i2c_flag_reg & STK3A5X_STATE_EN_ALS_MASK) {
        als_enable = true;
    }

#if OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 1 //according to effect of p_sensor to light sensor, l_sensor integral time is set to 100ms, and p_sensor waittime must be set to above 200ms always

    i2c_flag_reg &= (~(STK3A5X_STATE_EN_PS_MASK | STK3A5X_STATE_EN_WAIT_MASK));

    if (enable) {
        if (als_enable) {
            i2c_flag_reg |= STK3A5X_STATE_EN_PS_MASK;
        } else {
            i2c_flag_reg |= (STK3A5X_STATE_EN_WAIT_MASK | STK3A5X_STATE_EN_PS_MASK);
        }
    }

#elif OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 2

    i2c_flag_reg &= (~(STK3A5X_STATE_EN_PS_MASK | STK3A5X_STATE_EN_WAIT_MASK));

    if (enable) {
        if (als_enable) {
            stk3a5x_set_waittime(scp_service, port_handle, stk_state.als_waittime);
            i2c_flag_reg |= STK3A5X_STATE_EN_PS_MASK | STK3A5X_STATE_EN_WAIT_MASK;
        } else {
            stk3a5x_set_waittime(scp_service, port_handle, stk_state.ps_waittime);
            i2c_flag_reg |= (STK3A5X_STATE_EN_WAIT_MASK | STK3A5X_STATE_EN_PS_MASK);
        }
    } else {
        if (als_enable) {
            stk3a5x_set_waittime(scp_service, port_handle, stk_state.als_waittime);
            i2c_flag_reg |= STK3A5X_STATE_EN_WAIT_MASK;
        } else {
            stk3a5x_set_waittime(scp_service, port_handle, stk_state.ps_waittime);
            i2c_flag_reg &= (~(STK3A5X_STATE_EN_WAIT_MASK));
        }
    }

#else

    i2c_flag_reg &= (~(STK3A5X_STATE_EN_PS_MASK));

    if (enable) {
        if (als_enable) {
            stk3a5x_set_waittime(scp_service, port_handle, STK3A5X_WAIT50);
            i2c_flag_reg |= STK3A5X_STATE_EN_PS_MASK | STK3A5X_STATE_EN_WAIT_MASK;
        } else {
            stk3a5x_set_waittime(scp_service, port_handle, STK3A5X_WAIT100);
            i2c_flag_reg |= (STK3A5X_STATE_EN_WAIT_MASK | STK3A5X_STATE_EN_PS_MASK);
        }
    } else {
        if (als_enable) {
            stk3a5x_set_waittime(scp_service, port_handle, STK3A5X_WAIT50);
            i2c_flag_reg |= STK3A5X_STATE_EN_WAIT_MASK;
        } else {
            stk3a5x_set_waittime(scp_service, port_handle, STK3A5X_WAIT100);
            i2c_flag_reg &= (~(STK3A5X_STATE_EN_WAIT_MASK));
        }
    }

#endif
    stk3a5x_read_modify_write(scp_service,
        port_handle,
        STK3A5X_REG_STATE,
        &i2c_flag_reg,
        1,
        &xfer_bytes,
        false,
        0xFF);
    return rv;
}

static sns_rc stk3a5x_als_enable(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    bool enable)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t  i2c_flag_reg;
    bool ps_enable = false;

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_als_enable %d", enable);

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_STATE,
            &i2c_flag_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    if (i2c_flag_reg & STK3A5X_STATE_EN_PS_MASK) {
        ps_enable = true;
    }

#if OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 1 //according to effect of p_sensor to light sensor, l_sensor integral time is set to 100ms, and p_sensor waittime must be set to above 200ms always

    i2c_flag_reg &= (~(STK3A5X_STATE_EN_ALS_MASK | STK3A5X_STATE_EN_WAIT_MASK));

    if (enable) {
        stk_state.first_als = true;
        i2c_flag_reg |= STK3A5X_STATE_EN_ALS_MASK;
    } else {
        if (ps_enable) {
            i2c_flag_reg |= STK3A5X_STATE_EN_WAIT_MASK;
        }
    }

#elif OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 2

    i2c_flag_reg &= (~(STK3A5X_STATE_EN_ALS_MASK));

    if (enable) {
        stk3a5x_set_waittime(scp_service, port_handle, 0);
        stk_state.first_als = true;
        low_light_compensation = false;
        i2c_flag_reg |= STK3A5X_STATE_EN_ALS_MASK | STK3A5X_STATE_EN_WAIT_MASK;
    } else {
        stk3a5x_set_waittime(scp_service, port_handle, stk_state.ps_waittime);
        if (ps_enable) {
            i2c_flag_reg |= STK3A5X_STATE_EN_WAIT_MASK;
        } else {
            i2c_flag_reg &= (~(STK3A5X_STATE_EN_WAIT_MASK));
        }
    }
#else
    i2c_flag_reg &= (~(STK3A5X_STATE_EN_ALS_MASK));

    if (enable) {
        stk3a5x_set_waittime(scp_service, port_handle, 0);
        stk_state.first_als = true;
        low_light_compensation = false;
        i2c_flag_reg |= STK3A5X_STATE_EN_ALS_MASK | STK3A5X_STATE_EN_WAIT_MASK;
    } else {
        stk3a5x_set_waittime(scp_service, port_handle, STK3A5X_WAIT100);
        if (ps_enable) {
            i2c_flag_reg |= STK3A5X_STATE_EN_WAIT_MASK;
        } else {
            i2c_flag_reg &= (~(STK3A5X_STATE_EN_WAIT_MASK));
        }
    }

#endif
    stk3a5x_read_modify_write(scp_service,
        port_handle,
        STK3A5X_REG_STATE,
        &i2c_flag_reg,
        1,
        &xfer_bytes,
        false,
        0xFF);
    return rv;
}

static sns_rc stk3a5x_clear_ps_int(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  i2c_flag_reg;
    uint8_t  valid_int_flag = STK3A5X_FLG_PSINT_MASK | STK3A5X_FLG_ALSINT_MASK;
    uint32_t xfer_bytes;

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_FLAG,
            &i2c_flag_reg,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS ||
        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }


    i2c_flag_reg &= ~valid_int_flag;
    stk3a5x_read_modify_write(scp_service,
        port_handle,
        STK3A5X_REG_FLAG,
        &i2c_flag_reg,
        1,
        &xfer_bytes,
        false,
        0xFF);


    return rv;
}

static sns_rc stk3a5x_get_ps_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint16_t *raw_data)
{
    int rv = 0;
    uint32_t xfer_bytes = 0;
    uint8_t  data[2];
    uint16_t ps_raw_data_u16 = 0;

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_DATA1_PS,
            &data[0],
            2,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS ||
        xfer_bytes != 2) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    ps_raw_data_u16 = (data[0] << 8 | data[1]);

    *raw_data = ps_raw_data_u16;
    SNS_PRINTF(ERROR, sns_fw_printf, "ps raw data = %d", ps_raw_data_u16);

    return 0;
}

static bool set_als_gain(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint8_t level)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  alsctrl_reg, gainctrl_reg, buffer, gain;
    uint32_t xfer_bytes;
    bool result = true;

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_ALSCTRL1,
            &alsctrl_reg,
            1,
            &xfer_bytes);
    if (rv != SNS_RC_SUCCESS ||
            xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_GAINCTRL,
            &gainctrl_reg,
            1,
            &xfer_bytes);
    if (rv != SNS_RC_SUCCESS ||
	        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    SNS_PRINTF(ERROR, sns_fw_printf, "set_als_gain old alsctrl_reg=%x,gainctrl_reg=%x\n",
               alsctrl_reg,gainctrl_reg);
    if (level == 0) {
        //Highest gain for low light :  ALS Gain X128 & C Gain X128
        //Reg[0x02] don't care
        //Write Reg[0x4E] = 0x06, GAIN_ALS_DX128 = 1, GAIN_C_DX128 = 1
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK3A5X_ALS_GAIN64;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x06; //als&c data gain x128
        gain = 128;
    } else if (level == 1) {
        //Middle gain for middle light :  ALS Gain X16 & C Gain X16
        //Write Reg[0x02] = 0x21, ALS IT = 50ms, GAIN_ALS = 2' b10
        //Write Reg[0x4E] = 0x20, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b10
        alsctrl_reg = (alsctrl_reg & 0xCF) | STK3A5X_ALS_GAIN16;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x20; //c data gain x16
        gain = 16;
    } else if (level == 2) {
        //Lowest gain for high light :  ALS Gain X1 & C Gain X1
        //Write Reg[0x02] = 0x01, ALS IT = 50ms, GAIN_ALS = 2' b00
        //Write Reg[0x4E] = 0x00, GAIN_ALS_DX128 = 0, GAIN_C_DX128 = 0, GAIN_C = 2' b00
        alsctrl_reg = (alsctrl_reg & 0xCF) |  STK3A5X_ALS_GAIN1;
        gainctrl_reg = (gainctrl_reg & 0xC9) | 0x00; //c data gain x1
        gain = 1;
    } else {
        return false;
    }

    rv = stk3a5x_read_modify_write(scp_service,
            port_handle,
            STK3A5X_REG_ALSCTRL1,
            &alsctrl_reg,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        result = false;
    }

    rv = stk3a5x_read_modify_write(scp_service,
            port_handle,
            STK3A5X_REG_GAINCTRL,
            &gainctrl_reg,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        result = false;
    }

    SNS_PRINTF(ERROR, sns_fw_printf, "set_als_gain alsctrl_reg=%x,gainctrl_reg=%x,gain=%d,level=%d\n",
        alsctrl_reg, gainctrl_reg, gain, level);

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            0x5F,
            &buffer,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        result = false;
    }

    buffer |= 0x01;//Restart FSM
    rv = stk3a5x_read_modify_write(scp_service,
            port_handle,
            0x5F,
            &buffer,
            1,
            &xfer_bytes,
            false,
            0xFF);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        result = false;
    }

    if ( result ) {
        stk_state.als_gain = gain;
    }

    return result;
}

void stk3a5x_reconfig_reg(uint8_t reg_num, uint8_t* reg_table)
{
    for (int i = 0 ; i < reg_num / 2; i++) {
        for (int j = 0; j < ARR_SIZE(stk3a5x_default_register_table); j++) {
            if (stk3a5x_default_register_table[j].address == reg_table[i * 2]) {
                stk3a5x_default_register_table[j].value = reg_table[i * 2 + 1];
                OPLUS_ALS_PS_LOG("reg_table: reg = %x  reg_value = %d\n",
                    stk3a5x_default_register_table[j].address,
                    stk3a5x_default_register_table[j].value);
                break;
            }
        }
    }
}

static bool chec_auto_gain(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint16_t * als_data)
{
    bool result = false;

    if (((als_data[0]) > 65000 || (als_data[4] > 65000)) && (stk_state.als_gain_level < 2)) {
        //Reduce gain
        stk_state.als_gain_level++;
        result = set_als_gain(scp_service, port_handle, stk_state.als_gain_level);
    } else if (((als_data[0] < 3000) && (als_data[4] < 3000)) && (stk_state.als_gain_level > 0)) {
        //Raise gain
        stk_state.als_gain_level--;
        result = set_als_gain(scp_service, port_handle, stk_state.als_gain_level);
    }

    return result;
}
static void stk3a5x_get_als_lux(float als, float green_data, float ir, float *als_data)
{
    float lux1 = 0;
    float lux2 = 0;
    float lux3 = 0;
    float lux_temp = 0;
    float A = stk_state.als_coef.A;
    float B = stk_state.als_coef.B;
    float C = stk_state.als_coef.C;
    float D = stk_state.als_coef.D;
    float E = stk_state.als_coef.E;
    float ratio = stk_state.als_coef.ratio;
    float low_thrd_compensation = 1000;
    float high_thrd_compensation = 1400;

    als = als * 128.0 / stk_state.als_gain; //als raw
    green_data = green_data * 128.0 / stk_state.als_gain; //gdata 550
    ir = ir * 128.0 / stk_state.als_gain; //c data
    als_data[1] = green_data;
    als_data[2] = ir;

    SNS_SPRINTF(ERROR, sns_fw_printf, "start 111 als=%f, ir=%f", als, ir);

    if (als > high_thrd_compensation && low_light_compensation == false) {
        low_light_compensation = true;

    } else if (als < low_thrd_compensation && low_light_compensation == true) {
        low_light_compensation = false;
    }

    if (low_light_compensation == true && (ir * 1000 / als > 300)) {
        ir = als * 3 / 10;
    }

    if (als < 70) {
        if (als < 20.f) {
            als = (als) * 1.75f;

        } else if ((als >= 20.f) && (als < 30.f)) {
            als = (als) * 1.4f;

        } else if ((als >= 30.f) && (als < 40.f)) {
            als = (als) * 1.3f;

        } else if ((als >= 40.f) && (als < 60.f)) {
            als = (als) * 1.2f;
        }
    }

    //fenguang
    if ((ir * 1000 / als) < ratio) {
        // cold light, CWF, TL84, U30
        lux3 = E * als / 1000;
        lux_temp = lux3;
    } else {
        lux3 = 0;

        if ((A * als) > (B * ir)) {
            lux1 = (A * als - B * ir) / 1000 ;// sun light

        } else {
            lux1 = 0;
        }

        if ((C * als) > (D * ir)) {
            lux2 = (C * als - D * ir) / 1000;// D65, A, HZ

        } else {
            lux2 = 0;
        }
        lux_temp = (lux2 > lux1) ? lux2 : lux1;
    }

    //lux_temp = (lux2 > lux1) ? lux2 : lux1;
    //lux_temp = (lux_temp > lux3) ? lux_temp : lux3;


    SNS_SPRINTF(ERROR, sns_fw_printf, "start 222 als=%f, ir=%f", als, ir);
    SNS_SPRINTF(ERROR, sns_fw_printf, "lux1=%f, lu2=%f, lux3=%f, lux_temp=%f", lux1,
        lux2, lux3, lux_temp);
    als_data[0] = lux_temp;
    //lux_report = lux_temp *scale
    return ;
}

static void stk3a5x_get_als_lux_old(uint16_t als_data, uint16_t green_data,
    uint16_t clear_data, float *als_report)
{
    uint16_t low_thrd_compensation = 1000;
    uint16_t high_thrd_compensation = 1400;

    if (als_data < low_thrd_compensation && low_light_compensation == false) {
        low_light_compensation = true;

    } else if (als_data > high_thrd_compensation
        && low_light_compensation == true) {
        low_light_compensation = false;
    }

    if (low_light_compensation) {
        SNS_PRINTF(ERROR, sns_fw_printf, "low_light_compensation: [%d, %d]",
            als_data, clear_data);

        if (als_data > (2 * clear_data)) {
            als_data -= 2 * clear_data;

        } else {
            als_data = 0;
        }
    }

    if (clear_data == 0 || ((green_data * 1000 / clear_data) >= 1000) ||
        (als_data * 128 / stk_state.als_gain < 200) ||
        (green_data < 20) || (clear_data < 20)) {
        als_report[0] = als_data * 128 / stk_state.als_gain * 0.35; // For CWF, TL84, U30

    } else if (green_data * 1000 / clear_data > 100) {
        als_report[0] = als_data * 128 / stk_state.als_gain * 0.1; // For Sun

    } else if (green_data * 1000 / clear_data > 61) {
        als_report[0] = green_data * 128 / stk_state.als_gain * 1.17; // For D65

    } else {
        als_report[0] = green_data * 128 / stk_state.als_gain * 0.81; // For A, HZ
    }

    als_report[1] = green_data * 128 / stk_state.als_gain; //gdata 550
    als_report[2] = clear_data * 128 / stk_state.als_gain; //c data
    return ;
}

static sns_rc stk3a5x_get_als_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, float *data, int len,
    uint8_t als_type, bool is_als_dri)
{
    UNUSED_VAR(len);
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes = 0;
    uint8_t  raw_data[10];
    uint16_t als_raw_data[5];
    int      loop_count = 0;
    bool     auto_gain = false;
    uint8_t is_als_data_ready = 0;


    if (!is_als_dri) {
        uint8_t als_reg_flag = 0;

        rv = stk3a5x_com_read_wrapper(scp_service,
                port_handle,
                STK3A5X_REG_FLAG,
                &als_reg_flag,
                1,
                &xfer_bytes);

        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1) {
            rv = SNS_RC_FAILED;
            SNS_PRINTF(ERROR, sns_fw_printf, "read_data fail %d\n", xfer_bytes);
            return rv;
        }

        if ((als_reg_flag & STK3A5X_FLG_ALSDR_MASK)) {
            //SNS_PRINTF(ERROR, sns_fw_printf, "als data is ready, als_reg_flag = 0x%x\n", als_reg_flag);
            is_als_data_ready = 1;
        } else {
            SNS_PRINTF(ERROR, sns_fw_printf, "als data is not ready, als_reg_flag = 0x%x\n", als_reg_flag);
            is_als_data_ready = 0;
        }

        if (stk_state.first_als) {
            if (is_als_data_ready) {
                SNS_PRINTF(ERROR, sns_fw_printf, "first als data is ready\n");
            } else {
                rv = SNS_RC_FAILED;
                SNS_PRINTF(ERROR, sns_fw_printf, "first als data is not ready\n");
                return rv;
            }
        }
    }

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_DATA1_ALS,
            &raw_data[0],
            10,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS ||
        xfer_bytes != 10) {
        rv = SNS_RC_FAILED;
        SNS_PRINTF(ERROR, sns_fw_printf, "read_data fail %d\n", xfer_bytes);
        return rv;
    }

    for (loop_count = 0; loop_count < (sizeof(als_raw_data) / sizeof(als_raw_data[0])); loop_count++) {
        *(als_raw_data + loop_count) =
            (*(raw_data + (2 * loop_count)) << 8 | *(raw_data + (2 * loop_count + 1) ));
    }



    SNS_PRINTF(ERROR, sns_fw_printf, "als raw data[0] = %d,als raw data[1] = %d,als raw data[2] = %d,als raw data[3] = %d,als raw data[4] = %d",
        als_raw_data[0], als_raw_data[1], als_raw_data[2], als_raw_data[3], als_raw_data[4]);

    if (!stk_state.first_als && (is_als_dri || (!is_als_dri && is_als_data_ready))) {
        auto_gain = chec_auto_gain(scp_service, port_handle, als_raw_data);
    }

    //use als/G to verify
    if (auto_gain) {
        // last data
        data[0] = stk_state.last_als;
        data[1] = stk_state.last_data_g;
        data[2] = stk_state.last_data_c;
    } else {
        if (als_type == NORMAL) {
            uint16_t als_data    = als_raw_data[0];
            uint16_t green_data  = als_raw_data[2];
            uint16_t clear_data  = als_raw_data[4];
            if(stk_state.als_ratio_type == 1) {
                stk3a5x_get_als_lux(als_raw_data[0] *1.0 , als_raw_data[2] *1.0, als_raw_data[4] *1.0 ,data);
            }else {
                stk3a5x_get_als_lux_old(als_raw_data[0], als_raw_data[2], als_raw_data[4], data);
            }

            {
                log_data_info log_data;
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
        } else {
            uint16_t low_thrd_compensation = 1000;
            uint16_t high_thrd_compensation = 1400;

            if (als_raw_data[0] < low_thrd_compensation && low_light_compensation == false) {
                low_light_compensation = true;
            } else if (als_raw_data[0] > high_thrd_compensation && low_light_compensation == true) {
                low_light_compensation = false;
            }

            if (low_light_compensation) {
                SNS_PRINTF(ERROR, sns_fw_printf, "low_light_compensation: [%d, %d]",
                    als_raw_data[0], als_raw_data[4]);

                if (als_raw_data[0] > (2 * als_raw_data[4])) {
                    als_raw_data[0] -= 2 * als_raw_data[4];
                } else {
                    als_raw_data[0] = 0;
                }
            }

            data[0] = als_raw_data[0] * 128 / stk_state.als_gain; //als data
            data[1] = als_raw_data[2] * 128 / stk_state.als_gain; //gdata 550
            data[2] = als_raw_data[2] * 128 / stk_state.als_gain; //c data
        }
    }

    stk_state.last_als = data[0];
    stk_state.last_data_g = data[1];
    stk_state.last_data_c = data[2];

    SNS_SPRINTF(ERROR, sns_fw_printf, "raw_lux = %f, gain = %d, als raw data = %d, gdata = %d,c_data= %d",
        data[0], stk_state.als_gain, als_raw_data[0], als_raw_data[2], als_raw_data[4]);

    if (stk_state.first_als) {
        stk_state.first_als = false;
        #if OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 1
        //stk3a5x_set_waittime(scp_service,port_handle,STK3A5X_WAIT50);
        SNS_PRINTF(ERROR, sns_fw_printf, "only use ps ,need to setting this feature");

        #elif OPLUS_FEATURE_SENSOR_STK3A5X_WAITTIME == 2
        stk3a5x_set_waittime(scp_service,port_handle, stk_state.als_waittime);
        SNS_PRINTF(ERROR, sns_fw_printf, "first await=%d, pwait=%d", stk_state.als_waittime, stk_state.ps_waittime);
        #else
        stk3a5x_set_waittime(scp_service,port_handle,STK3A5X_WAIT50);
        SNS_PRINTF(ERROR, sns_fw_printf, "first data  set the wait to 50");
        #endif
    }


    return rv;
}

#define DEBUG_REG_LEN 29

void stk3a5x_dump_reg(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    uint32_t xfer_bytes;
    uint8_t reg_status[DEBUG_REG_LEN];
    uint8_t stk3a5x_reg_map[DEBUG_REG_LEN] = {
        STK3A5X_REG_STATE,
        STK3A5X_REG_PSCTRL,
        STK3A5X_REG_ALSCTRL1,
        STK3A5X_REG_LEDCTRL,
        STK3A5X_REG_INTCTRL1,
        STK3A5X_REG_WAIT,
        STK3A5X_REG_THDH1_PS,
        STK3A5X_REG_THDH2_PS,
        STK3A5X_REG_THDL1_PS,
        STK3A5X_REG_THDL2_PS,
        STK3A5X_REG_THDH1_ALS,
        STK3A5X_REG_THDH2_ALS,
        STK3A5X_REG_THDL1_ALS,
        STK3A5X_REG_THDL2_ALS,
        STK3A5X_REG_FLAG,
        STK3A5X_REG_DATA1_PS,
        STK3A5X_REG_DATA2_PS,
        STK3A5X_REG_DATA1_ALS,
        STK3A5X_REG_DATA2_ALS,
        STK3A5X_REG_DATA1_ALS1,
        STK3A5X_REG_DATA2_ALS1,
        STK3A5X_REG_DATA1_C,
        STK3A5X_REG_DATA2_C,
        STK3A5X_REG_DATA1_PS_OFFSET,
        STK3A5X_REG_DATA2_PS_OFFSET,
        STK3A5X_REG_PDT_ID,
        STK3A5X_REG_RSRVD,
        STK3A5X_REG_GAINCTRL,
        STK3A5X_REG_AGCTRL,
    };

    uint8_t i = 0;

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_dump_reg: \n");

    for (i = 0; i < DEBUG_REG_LEN; i++) {
        stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            stk3a5x_reg_map[i],
            &reg_status[i],
            1,
            &xfer_bytes);
        SNS_PRINTF(ERROR, sns_fw_printf, "reg[0x%X] = 0x%X", stk3a5x_reg_map[i], reg_status[i]);
    }
}

static sns_rc stk3a5x_init_ps_irq(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  buffer[1];
    uint32_t xfer_bytes;

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_INTCTRL1,
            &buffer[0],
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS ||
        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    buffer[0] |= STK3A5X_INT_PS;

    stk3a5x_read_modify_write(scp_service,
        port_handle,
        STK3A5X_REG_INTCTRL1,
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

static sns_rc stk3a5x_init_als_irq(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t  buffer[1];
    uint32_t xfer_bytes;
    uint16_t reg_num = 0;
    uint16_t reg_count = 0;
    stk3a5x_register_table* selected_table = NULL;

    reg_num = sizeof(stk3a5x_default_als_thd_table) / sizeof(stk3a5x_register_table);
    selected_table = stk3a5x_default_als_thd_table;

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_INTCTRL1,
            &buffer[0],
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS ||
        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    buffer[0] |= STK3A5X_INT_ALS;

    stk3a5x_read_modify_write(scp_service,
        port_handle,
        STK3A5X_REG_INTCTRL1,
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

    for (reg_count = 0; reg_count < reg_num; reg_count++) {
        rv = stk3a5x_read_modify_write(scp_service,
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

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            0x5F,
            &buffer[0],
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
        return rv;
    }

    buffer[0] |= 0x01;//Restart FSM
    stk3a5x_read_modify_write(scp_service,
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

static bool stk3a5x_prox_need_ir_info()
{
    return true;
}

static sns_rc stk3a5x_get_ps_original_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    int* original_data)
{
    sns_rc   rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t  ps_raw_data[2];

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_ORIGINAL_DATA1_PS,
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

static sns_rc stk3a5x_get_ir_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    int* ps_off_data)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint16_t psoff_data[4] = {0};
    uint8_t  data_buffer[8];

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            0x24,
            &data_buffer[0],
            8,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS ||
        xfer_bytes != 8) {
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
sns_rc stk3a5x_get_ps_device_irq_mask (sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t *mask)
{
    int rv = 0;
    uint8_t status = 0, temp_status = 0, retry_count = 0;
    uint32_t xfer_bytes;
    uint8_t reg_value = 0xFF;
    uint8_t  valid_int_flag = STK3A5X_FLG_ALSINT_MASK | STK3A5X_FLG_PSINT_MASK;
    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_get_ps_device_irq_mask");

    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_FLAG,
            &status,
            1,
            &xfer_bytes);

    temp_status = status;

    while ((0 != (temp_status & valid_int_flag)) && (++retry_count < 8)) {
        if (temp_status & STK3A5X_FLG_ALSINT_MASK) {
            reg_value &= ~STK3A5X_FLG_ALSINT_MASK;
        }

        if (temp_status & STK3A5X_FLG_PSINT_MASK) {
            reg_value &= ~STK3A5X_FLG_PSINT_MASK;
        }

        if (reg_value != 0xFF) {
            stk3a5x_read_modify_write(scp_service, port_handle, STK3A5X_REG_FLAG, &reg_value, 1, &xfer_bytes, false, 0xFF);
        }

        SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_get_ps_device_irq_mask retry_count=%d, status = %x", retry_count, reg_value);
        reg_value = 0xFF;

        stk3a5x_com_read_wrapper(scp_service, port_handle, STK3A5X_REG_FLAG, &temp_status, 1, &xfer_bytes);
        status = (temp_status | status);
    }

    if (retry_count == 8) {
        stk3a5x_clear_ps_int(scp_service, port_handle);
    }

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_get_ps_device_irq_mask status = %x", status);

    if (status & STK3A5X_FLG_ALSINT_MASK) {
        *mask |= ALS_INT;
    }

    if (status & STK3A5X_FLG_PSINT_MASK) {
        *mask |= PS_INT;
    }

    return rv;
}

static uint8_t stk3a5x_get_default_reg_value(uint8_t reg)
{
    for (int i = 0; i < ARR_SIZE(stk3a5x_default_register_table); i++) {
        if (reg == stk3a5x_default_register_table[i].address) {
            return stk3a5x_default_register_table[i].value;
        }
    }

    return 0;
}

static void stk3a5x_set_default_reg_value(uint8_t reg, uint8_t value)
{
    for (int i = 0; i < ARR_SIZE(stk3a5x_default_register_table); i++) {
        if (reg == stk3a5x_default_register_table[i].address) {
            stk3a5x_default_register_table[i].value = value;
            break;
        }
    }
}

static void stk3a5x_read_pd_value(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint16_t *pd0_val,
    uint16_t *pd1_val)
{
    uint8_t data_buf[2];
    uint32_t xfer_bytes;
    memset(&data_buf[0], 0, 2);
    sns_rc rv = SNS_RC_SUCCESS;
    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            0x2c,
            &data_buf[0],
            2,
            &xfer_bytes);

    if (SNS_RC_SUCCESS == rv && 2 == xfer_bytes) {
        *pd0_val = ((data_buf[0] << 8) | data_buf[1]);
    } else {
        *pd0_val = 0;
    }

    memset(&data_buf[0], 0, 2);
    rv = stk3a5x_com_read_wrapper(scp_service,
            port_handle,
            0x2e,
            &data_buf[0],
            2,
            &xfer_bytes);

    if (SNS_RC_SUCCESS == rv && 2 == xfer_bytes) {
        *pd1_val =  ((data_buf[0] << 8) | data_buf[1]);
    } else {
        *pd1_val = 0;
    }

    SNS_PRINTF(ERROR, sns_fw_printf, "stk3a5x_read_pd_value pd0 = %u, pd1 = %u", *pd0_val, *pd1_val);
}

static void stk3a5x_close_pd(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint16_t ps_saturation)
{
    uint8_t i2c_flag_reg;
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t stk_pd_reg_val = 0x7f;
    int count = 0;
    bool close_pd0 = true;
    bool close_pd1 = true;
    uint16_t pd0_val = 0;
    uint16_t pd1_val = 0;
    uint32_t xfer_bytes;

    // set wait time 0ms
    i2c_flag_reg = stk3a5x_get_default_reg_value(STK3A5X_REG_PSCTRL);
    rv = stk3a5x_com_write_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_PSCTRL,
            &i2c_flag_reg,
            1,
            &xfer_bytes,
            false);
    i2c_flag_reg = stk3a5x_get_default_reg_value(STK3A5X_REG_LEDCTRL);
    rv = stk3a5x_com_write_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_LEDCTRL,
            &i2c_flag_reg,
            1,
            &xfer_bytes,
            false);
    i2c_flag_reg = 0;
    rv = stk3a5x_com_write_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_WAIT,
            &i2c_flag_reg,
            1,
            &xfer_bytes,
            false);
    //enable ps
    i2c_flag_reg |= (STK3A5X_STATE_EN_WAIT_MASK | STK3A5X_STATE_EN_PS_MASK);
    rv = stk3a5x_com_write_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_STATE,
            &i2c_flag_reg,
            1,
            &xfer_bytes,
            false);

    for (count = 0; count < 3; count++) {
        sns_busy_wait(sns_convert_ns_to_ticks(5 * 1000 * 1000));
        stk3a5x_read_pd_value(scp_service, port_handle, &pd0_val, &pd1_val);

        if (pd0_val <= ps_saturation) {
            close_pd0 = false;
        }

        if (pd1_val <= ps_saturation) {
            close_pd1 = false;
        }

        if (!close_pd0 && !close_pd1) {
            break;
        }
    }

    if (close_pd0 || close_pd1) {
        if (close_pd0) {
            stk_pd_reg_val &= 0xFE;
        }

        if (close_pd1) {
            stk_pd_reg_val &= 0xFD;
        }

        stk3a5x_set_default_reg_value(0xA1, stk_pd_reg_val);
    }

    //disable ps
    i2c_flag_reg = 0;
    rv = stk3a5x_com_write_wrapper(scp_service,
            port_handle,
            STK3A5X_REG_STATE,
            &i2c_flag_reg,
            1,
            &xfer_bytes,
            false);

    SNS_PRINTF(ERROR, sns_fw_printf, "close_pd0 %d, close_pd1 %d, stk_pd_reg_val 0x%x",
        close_pd0, close_pd1, stk_pd_reg_val);
}

static void stk3a5x_special_process_before_avaliable(alsps_state * state)
{
    if (state->is_need_check_pd && state->ps_saturation != 0) {
        stk3a5x_close_pd(state->scp_service, state->com_port_info.port_handle,
            state->ps_saturation);
    }
}

struct alsps_als_operations stk3a5x_als_ops = {
    .get_who_am_i = stk3a5x_get_who_am_i,
    .init_driver = stk3a5x_init_als,
    .als_enable = stk3a5x_als_enable,
    .clear_als_int = NULL,
    .init_irq = stk3a5x_init_als_irq,
    .get_als_data = stk3a5x_get_als_data,
    .dump_reg = stk3a5x_dump_reg,
    .get_als_device_irq_mask = stk3a5x_get_ps_device_irq_mask,
    .deinit_driver = stk3a5x_deinit_als,
    .reconfig_reg_table = stk3a5x_reconfig_reg,
    .get_als_fifo_data = NULL,
    .set_brightness = NULL,
    .enable_fifo = NULL
};


struct alsps_ps_operations stk3a5x_ps_ops = {
    .get_who_am_i = stk3a5x_get_who_am_i,
    .init_driver = stk3a5x_init_als,
    .ps_enable = stk3a5x_ps_enable,
    .clear_ps_int = stk3a5x_clear_ps_int,
    .init_irq = stk3a5x_init_ps_irq,
    .get_ps_data = stk3a5x_get_ps_data,
    .ps_set_thd = stk3a5x_ps_set_thd,
    .dump_reg = stk3a5x_dump_reg,
    .set_offset = stk3a5x_set_offset,
    .get_ps_device_irq_mask = stk3a5x_get_ps_device_irq_mask,
    .deinit_driver = stk3a5x_deinit_als,
    .prox_need_ir_info = stk3a5x_prox_need_ir_info,
    .get_ps_original_data = stk3a5x_get_ps_original_data,
    .get_ir_data = stk3a5x_get_ir_data,
    .hardware_cali = NULL,
    .recover_device = NULL,
    .reconfig_reg_table = stk3a5x_reconfig_reg,
    .special_process_before_avaliable = stk3a5x_special_process_before_avaliable,
    .ps_offset_cali = NULL
};
#endif
