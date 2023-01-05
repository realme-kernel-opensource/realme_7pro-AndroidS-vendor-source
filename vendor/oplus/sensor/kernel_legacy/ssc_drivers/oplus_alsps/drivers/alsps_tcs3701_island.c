/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/

#ifdef SUPPORT_TCS3701
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

#include "alsps_tcs3701.h"
#include "sns_alsps_sensor.h"
#include "oplus_alsps.h"
static int i2c_error_count = 0;
#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
extern lb_display_coefs lb_dis_coefs;
#endif
static tcs3701_instance_state tcs_state = {
    .publish_sensors = 0,
    .first_als = false,
    .ir_ratio = 1,
    .atime = 20000,
    .again = 262,
    .lux = 0,
    .last_lux = 0,
    .far_thd = 0,
    .near_thd = 0,
    .first_prox = false,
    .ch_raw = {0},
    .offset = 0,
    .saturation = 5755,
    .low_saturation = 1438,
    .use_fifo = false,
    .brightness = 1023
};

static tcs3701_reg_setting default_setting[] = {
    {TCS3701_ATIME_REG,    0},
    {TCS3701_PTIME_REG,    PTIME_MS(200)},
    {TCS3701_WTIME_REG,    0},
    {TCS3701_CFG3_REG,     (CFG3_RVED | HXTALK_MODE1)},
    {TCS3701_CFG8_REG,     CONCURRENT_PROX_AND_ALS},
    {TCS3701_CFG11_REG,    0},
    {TCS3701_CFG14_REG,    0},
    {TCS3701_PERS_REG,     (ALS_PERSIST(0) | PROX_PERSIST(ALSPS_PERSIST_CONFIG))},
    {TCS3701_PCFG1_REG,    HXTALK_MODE2},
    {TCS3701_PCFG2_REG,    PLDRIVE_MA(50)},
    {TCS3701_PCFG4_REG,    PGAIN_2X},
    {TCS3701_PCFG5_REG,    (PPULSES(40) | PPULSE_LEN_32US)},
    {TCS3701_CALIBCFG0_REG, 0},
    {TCS3701_CALIBCFG1_REG, 0x08 | 0x02},
    {TCS3701_CALIBCFG2_REG, BINSRCH_TARGET_63},
    {TCS3701_INTENAB_REG,   0x00}
};

uint16_t tcs3701_again[] = {
    0,
    1,
    2,
    4,
    8,
    16,
    32,
    64,
    128,
    262,
    549,
    1057,
    2204
};

static uint8_t smux_data[4][20] = {
    { /* 1 channel */
        0x11, 0x01, 0x11, 0x11,
        0x11, 0x11, 0x11, 0x11,
        0x11, 0x11, 0x11, 0x11,
        0x11, 0x11, 0x01, 0x11,
        0x11, 0x11, 0x00, 0x76
    },
    //{    2 channel
        //CH0:7C+4R+8G+5B+0W  CH1:1C+2R+3B+0W
       //  0x11, 0x10, 0x01, 0x12,
       // 0x11, 0x11, 0x11, 0x22,
       // 0x01, 0x11, 0x10, 0x11,
        //0x12, 0x22, 0x01, 0x11,
        //0x11, 0x01, 0x00, 0x76
    //},
    { /*CH0:8C+8R+8G+8B CH1:4W*/
        0x11, 0x12, 0x11, 0x11,
        0x11, 0x11, 0x11, 0x11,
        0x21, 0x11, 0x12, 0x11,
        0x11, 0x11, 0x11, 0x11,
        0x11, 0x11, 0x00, 0x76,
    },
    {
        /* 3 channel */
        //CH0:7C+4R+8G+5B+0W  CH1:1C+2R+3B+0W CH2:0C+0R+0B+4W
        0x11, 0x13, 0x01, 0x12,
        0x11, 0x11, 0x11, 0x22,
        0x31, 0x11, 0x13, 0x11,
        0x12, 0x22, 0x01, 0x11,
        0x11, 0x31, 0x00, 0x76
    },
    {   /* 4 channel */
        // C, R, G, B
        0x14, 0x20, 0x23, 0x41,
        0x33, 0x12, 0x14, 0x24,
        0x03, 0x23, 0x10, 0x14,
        0x32, 0x44, 0x21, 0x23,
        0x13, 0x04, 0x00, 0x76
    }
};

static sns_rc oplus_i2c_rw(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, sns_port_vector* port_vec, bool save_write_time, uint32_t* xfer_bytes)
{
    UNUSED_VAR(save_write_time);
    sns_rc rv = SNS_RC_SUCCESS;

    rv = scp_service->api->sns_scp_register_rw(port_handle,
            port_vec,
            1,
            false,
            xfer_bytes);

    if (rv != SNS_RC_SUCCESS) {
        i2c_error_count++;
    }

    return rv;
}

static sns_rc tcs3701_com_read_wrapper(
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

    return oplus_i2c_rw(scp_service, port_handle, &port_vec, false, xfer_bytes);
}

static sns_rc tcs3701_com_write_wrapper(
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
    return oplus_i2c_rw(scp_service, port_handle, &port_vec, save_write_time, xfer_bytes);
}

sns_rc tcs3701_read_modify_write(
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
        tcs3701_com_write_wrapper(scp_service,
            port_handle,
            reg_addr,
            &reg_value[0],
            size,
            xfer_bytes,
            save_write_time);
    } else {
        // read current value from this register
        tcs3701_com_read_wrapper(scp_service,
            port_handle,
            reg_addr,
            &rw_buffer,
            1,
            &rw_bytes);
        // generate new value
        rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);
        // write new value to this register
        tcs3701_com_write_wrapper(scp_service,
            port_handle,
            reg_addr,
            &rw_buffer,
            1,
            xfer_bytes,
            save_write_time);
    }

    return SNS_RC_SUCCESS;
}

static sns_rc tcs3701_inst_i2c_write_byte(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t reg,
    uint8_t val)
{
    sns_rc rv = SNS_RC_FAILED;
    uint32_t xfer_bytes = 0;
    sns_port_vector port_vec;
    port_vec.buffer = &val;
    port_vec.bytes = 1;
    port_vec.is_write = true;
    port_vec.reg_addr = reg;

    rv = oplus_i2c_rw(scp_service, port_handle, &port_vec, false, &xfer_bytes);

    //OPLUS_ALS_PS_LOG("i2c xfter_bytes %d", xfer_bytes);

    if (SNS_RC_SUCCESS != rv || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
    }

    return rv;
}

sns_rc tcs3701_inst_i2c_read_byte(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t reg,
    uint8_t *buffer)
{
    sns_rc rv = SNS_RC_FAILED;
    uint32_t xfer_bytes = 0;
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = 1;
    port_vec.is_write = false;
    port_vec.reg_addr = reg;

    rv = oplus_i2c_rw(scp_service, port_handle, &port_vec, false, &xfer_bytes);

    if (SNS_RC_SUCCESS != rv || xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
    }

    return rv;
}

static sns_rc tcs3701_inst_i2c_modify(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t reg,
    uint8_t mask,
    uint8_t val)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t rw_buffer = 0;

    if (0xFF == mask) {
        rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, reg, val);
    } else {
        /* Read current value from this register */
        rv = tcs3701_inst_i2c_read_byte(scp_service, port_handle, reg, &rw_buffer);

        if (SNS_RC_SUCCESS == rv) {
            rw_buffer &= ~mask;
            rw_buffer |= (val & mask);
            rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, reg, rw_buffer);
        }
    }

    return rv;
}

static sns_rc tcs3701_inst_i2c_read_block(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t reg,
    uint8_t *buffer,
    uint32_t len)
{
    sns_rc rv = SNS_RC_FAILED;
    uint32_t xfer_bytes = 0;
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = len;
    port_vec.is_write = false;
    port_vec.reg_addr = reg;

    rv = oplus_i2c_rw(scp_service, port_handle, &port_vec, false, &xfer_bytes);


    if (SNS_RC_SUCCESS != rv || len != xfer_bytes) {
        rv = SNS_RC_FAILED;
    }

    return rv;
}


sns_rc tcs3701_get_who_am_i(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    uint8_t buffer = 0;
    rv = tcs3701_com_read_wrapper(scp_service,
            port_handle,
            TCS3701_ID_REG,
            &buffer,
            1,
            &xfer_bytes);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        rv = SNS_RC_FAILED;
    }

    if (buffer != 0x18) {
        rv = SNS_RC_FAILED;
    } else {
        rv = SNS_RC_SUCCESS;
    }


    return rv;
}


sns_rc tcs3701_device_sw_reset(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    alsps_sensor_type sensor)
{
    UNUSED_VAR(sensor);
    uint8_t buffer[1];
    sns_rc rv = SNS_RC_SUCCESS;

    uint32_t xfer_bytes;
    buffer[0] = 0;
    rv = tcs3701_com_write_wrapper(
            scp_service,
            port_handle,
            TCS3701_ENABLE_REG,
            &buffer[0],
            1,
            &xfer_bytes,
            false);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        return rv;
    }

    rv = tcs3701_com_write_wrapper(
            scp_service,
            port_handle,
            TCS3701_ENABLE_REG,
            &buffer[0],
            1,
            &xfer_bytes,
            false);

    if (rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1) {
        return rv;
    }

    return SNS_RC_SUCCESS;
}

sns_rc tcs3701_modify_enable(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t mask,
    uint8_t val)
{
    return tcs3701_inst_i2c_modify(scp_service, port_handle, TCS3701_ENABLE_REG, mask, val);
}


sns_rc tcs3701_modify_intenab(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t mask,
    uint8_t val)
{
    return tcs3701_inst_i2c_modify(scp_service, port_handle, TCS3701_INTENAB_REG, mask, val);
}

sns_rc tcs3701_clear_fifo(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_FAILED;

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_CONTROL_REG, 0x02);

    return rv;
}

sns_rc tcs3701_set_smux_config(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint8_t als_ch_cfg)
{
    sns_rc rv = SNS_RC_FAILED;
    uint8_t i;

    /* Write command */
    tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_ENABLE_REG, 0x00);
    tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_ENABLE_REG, 0x01);

    /* Send sequnce */
    for (i = 0; i < sizeof(smux_data[als_ch_cfg - 1]); i++) {
        rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, i, smux_data[als_ch_cfg - 1][i]);

        if (SNS_RC_SUCCESS != rv) {
            return rv;
        }
    }

    tcs3701_inst_i2c_write_byte(scp_service, port_handle, 0xAF, 0x10);
    /* Execute the command */
    tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_ENABLE_REG, (0x10 | PON));
    /* Wait 500us */
    sns_busy_wait(sns_convert_ns_to_ticks(500 * 1000));
    /* Clear command */
    tcs3701_inst_i2c_write_byte(scp_service, port_handle, 0xAF, 0x00);

#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
    if (tcs_state.use_fifo) {
        tcs3701_inst_i2c_write_byte(scp_service,port_handle, 0xD5, 0x38);
        tcs3701_inst_i2c_write_byte(scp_service,port_handle, 0xFC, 0x0E);

        tcs3701_clear_fifo(scp_service,port_handle);
        lb_algo_clear_fifo();
    }
#endif
    return SNS_RC_SUCCESS;
}

sns_rc tcs3701_set_als_gain(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint32_t again)
{
    sns_rc rv = SNS_RC_FAILED;
    uint8_t cfg1 = 0;

    switch (again) {
    case 0:
        cfg1 = AGAIN_0_5X;
        break;

    case 1:
        cfg1 = AGAIN_1X;
        break;

    case 2:
        cfg1 = AGAIN_2X;
        break;

    case 4:
        cfg1 = AGAIN_4X;
        break;

    case 8:
        cfg1 = AGAIN_8X;
        break;

    case 16:
        cfg1 = AGAIN_16X;
        break;

    case 32:
        cfg1 = AGAIN_32X;
        break;

    case 64:
        cfg1 = AGAIN_64X;
        break;

    case 128:
        cfg1 = AGAIN_128X;
        break;

    case 262:
        cfg1 = AGAIN_256X;
        break;

    case 549:
        cfg1 = AGAIN_512X;
        break;

    case 1057:
        cfg1 = AGAIN_1024X;
        break;

    case 2204:
        cfg1 = AGAIN_2048X;
        break;

    default:
        break;
    }

    rv = tcs3701_inst_i2c_modify(scp_service, port_handle, TCS3701_CFG1_REG, AGAIN_MASK, cfg1);
    tcs3701_modify_enable(scp_service, port_handle, AEN, 0);
    tcs3701_modify_enable(scp_service, port_handle, AEN, AEN);

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    return rv;
}

sns_rc tcs3701_set_als_time(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint32_t time_us)
{
    sns_rc rv = SNS_RC_FAILED;
    uint16_t astep;

    OPLUS_ALS_PS_LOG("set atime to: %d(us)\n", time_us);

    astep = ASTEP_US(time_us);
    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle,
            TCS3701_ASTEPL_REG, (uint8_t)(astep & 0xFF));

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle,
            TCS3701_ASTEPH_REG, (uint8_t)((astep & 0xFF00) >> 8));
    //add to make the als time effective immediately
    tcs3701_modify_enable(scp_service, port_handle, AEN, 0);
    tcs3701_modify_enable(scp_service, port_handle, AEN, AEN);

    return rv;
}

sns_rc tcs3701_set_prox_pers(sns_sync_com_port_service * scp_service,
    sns_sync_com_port_handle *port_handle, uint8_t persistence)
{
    return tcs3701_inst_i2c_modify(scp_service, port_handle,
            TCS3701_PERS_REG, PPERS_MASK, PROX_PERSIST(persistence));
}

sns_rc tcs3701_device_set_default_state(sns_sync_com_port_service * scp_service,
    sns_sync_com_port_handle *port_handle,
    alsps_sensor_type sensor)
{
    UNUSED_VAR(sensor);
    sns_rc rv = SNS_RC_FAILED;
    uint8_t i;

    /* Enable PON bit */
    //tcs3701_inst_i2c_write_byte(state, 0xD6, 0x00);
    tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_ENABLE_REG, PON);
    if (tcs_state.use_fifo) {//for lb algo
        tcs3701_set_smux_config(scp_service, port_handle, ALS_2CH);
    } else {
        tcs3701_set_smux_config(scp_service, port_handle, ALS_3CH);
    }


    for (i = 0; i < ARR_SIZE(default_setting); i++) {
        rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle,
                default_setting[i].reg,
                default_setting[i].value);

        if (SNS_RC_SUCCESS != rv) {
            return rv;
        }
    }

    tcs3701_set_prox_pers(scp_service, port_handle, ALSPS_PERSIST_CONFIG);
    return SNS_RC_SUCCESS;
}

sns_rc tcs3701_reset_device(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    alsps_sensor_type sensor)
{
    sns_rc rv = SNS_RC_SUCCESS;

    /** HW reset only when Accel is requested for reset. */
    if (sensor == ALS || sensor == PS) {
        rv = tcs3701_device_sw_reset(scp_service, port_handle, sensor);
    }

    if (rv == SNS_RC_SUCCESS) {
        rv = tcs3701_device_set_default_state(scp_service, port_handle, sensor);
    }

    return rv;
}

static sns_rc tcs3701_deinit_als()
{
    SNS_PRINTF(ERROR, sns_fw_printf, "tcs3701_deinit_als");
    return 0;
}

static sns_rc tcs3701_init_als(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    tcs_state.first_prox = true;
    //first init  reset the device
    tcs3701_reset_device(scp_service, port_handle, ALS);
    return 0;
}

void tcs3701_reconfig_reg(uint8_t reg_num, uint8_t* reg_table)
{
    for (int i = 0 ; i < reg_num / 2; i++) {
        for (int j = 0; j < ARR_SIZE(default_setting); j++) {
            if (default_setting[j].reg == reg_table[i * 2]) {
                default_setting[j].value = reg_table[i * 2 + 1];
                OPLUS_ALS_PS_LOG("reg_table: reg = %x  reg_value = %d\n",
                    default_setting[j].reg,
                    default_setting[j].value);
                break;
            }
        }
    }
}


sns_rc tcs3701_set_offset(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    int offset)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t offset_lr, offset_hr;

    tcs_state.offset = offset;

    if (offset > 0) {
        offset_lr = offset & 0xff;
        offset_hr = 0;
    } else {
        offset_hr = 0xff;
        offset_lr = (0xff + offset);
    }

    rv |= tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_POFFSETL_REG, offset_lr);

    rv |= tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_POFFSETH_REG, offset_hr);


    if (SNS_RC_SUCCESS == rv) {
        OPLUS_ALS_PS_LOG("tcs3701_ps_set_offset: offset = %d offset lr = %d ,offset_hr = %d \n",
            offset, offset_lr, offset_hr);
    } else {
        OPLUS_ALS_PS_LOG("tcs3701_ps_set_offset: write reg ERROR!\n");
    }

    tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_POFFSETL_REG, &offset_lr);

    tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_POFFSETH_REG, &offset_hr);

    OPLUS_ALS_PS_LOG("tcs3701_ps_set_offset: after read offset lr = %d ,offset_hr = %d \n",
        offset_lr, offset_hr);
    return rv;
}

static sns_rc tcs3701_ps_set_thd(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint16_t high_thresh,
    uint16_t low_thresh,
    alsps_ps_state status)
{
    UNUSED_VAR(status);
    sns_rc rv = SNS_RC_FAILED;
    uint8_t data[4];

    if ((low_thresh != 0) && (high_thresh != 0)) {
        tcs_state.far_thd = low_thresh;
        tcs_state.near_thd = high_thresh;
    }

    if (status == PRX_FAR_AWAY) {
        low_thresh = 0;
        tcs_state.near_thd = high_thresh;
    } else if (status == RX_NEAR_BY) {
        high_thresh = 1023;
        tcs_state.far_thd = low_thresh;
    } else {
        //do nothing
    }

    data[0] = (uint8_t)(low_thresh & 0xFF);
    data[1] = (uint8_t)((low_thresh >> 8) & 0xFF);
    data[2] = (uint8_t)(high_thresh & 0xFF);
    data[3] = (uint8_t)((high_thresh >> 8) & 0xFF);

    OPLUS_ALS_PS_LOG("tcs3701_update_prox_threshold: far_thd=%d, near_thd=%d low_thresh = %d ,high_thresh = %d  status = %d\n",
        tcs_state.far_thd, tcs_state.near_thd, low_thresh, high_thresh, status);

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PILT0L_REG, data[0]);

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PILT0H_REG, data[1]);

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PIHT0L_REG, data[2]);

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PIHT0H_REG, data[3]);


    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PILT1L_REG, data[0]);

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PILT1H_REG, data[1]);

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PIHT1L_REG, data[2]);

    if (SNS_RC_SUCCESS != rv) {
        return rv;
    }

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_PIHT1H_REG, data[3]);

    return rv;
}

static sns_rc tcs3701_ps_enable(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    bool enable)
{
    uint8_t enable_flag = 0;
    uint8_t intenab = 0;
    uint8_t check_enable = 0;
    sns_rc rv = SNS_RC_SUCCESS;

    OPLUS_ALS_PS_LOG("tcs3701_ps_enable %d", enable);

    if (enable) {
        intenab |= PIEN0;
        rv |= tcs3701_modify_intenab(scp_service, port_handle, (PIEN0 | CIEN | PIEN1), intenab);

        enable_flag = PON;
        enable_flag |= PEN;
        enable_flag |= AEN;

        if (tcs_state.first_prox == true) {
            tcs3701_set_prox_pers(scp_service, port_handle, 0);
        }

        rv |= tcs3701_modify_enable(scp_service, port_handle, (PEN | AEN | PON), enable_flag);

        tcs_state.publish_sensors |= TCS3701_PROX;
    } else {
        tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_ENABLE_REG, &check_enable);

        if (check_enable & AEN) {
            check_enable &= ~PEN;
        } else {
            check_enable &= ~(PEN | PON);
        }

        if (tcs_state.publish_sensors & TCS3701_ALS) {
            check_enable |= (AEN | PON);
            if (!tcs_state.use_fifo) {
                tcs3701_modify_intenab(scp_service, port_handle, AIEN, AIEN);
            }
        }
        tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_ENABLE_REG, check_enable);
        tcs3701_modify_intenab(scp_service, port_handle, (PIEN0 | CIEN | PIEN1), 0);

        tcs_state.publish_sensors &= ~TCS3701_PROX;
    }

    return rv;
}

static sns_rc tcs3701_als_enable(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    bool enable)
{
    uint8_t enable_flag = 0;
    uint8_t check_enable = 0;
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t intenab = 0;

    OPLUS_ALS_PS_LOG("tcs3701_als_enable %d", enable);

    if (enable) {
        if (!(tcs_state.publish_sensors & TCS3701_ALS)) {
            //init data
            tcs3701_set_als_time(scp_service, port_handle, 20000);
            tcs3701_set_als_gain(scp_service, port_handle, 549);

            memset(tcs_state.ch_raw, 0, sizeof(tcs_state.ch_raw));
            tcs_state.first_als = true;
#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
            tcs_state.algo_state = COMPENSATION_ALGO;
            lb_algo_clear_fifo();
            tcs3701_clear_fifo(scp_service,port_handle);
#endif
        }

        if (!tcs_state.use_fifo) {
            intenab |= AIEN;//als int
        }
        tcs3701_modify_intenab(scp_service, port_handle, AIEN, intenab);
        enable_flag = PON;//power on
        enable_flag |= AEN;//als enable

        rv |= tcs3701_modify_enable(scp_service, port_handle, (AEN | PON), enable_flag);
        tcs_state.publish_sensors |= TCS3701_ALS;
    } else {
        tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_ENABLE_REG, &check_enable);

        //ps on not power off
        if (check_enable & PEN) {
            check_enable &= ~AEN;
        } else {
            check_enable &= ~(AEN | PON);
        }

#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
        tcs3701_clear_fifo(scp_service,port_handle);
#endif
        tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_ENABLE_REG, check_enable);

        tcs_state.publish_sensors &= ~TCS3701_ALS;
    }

    return rv;
}


static sns_rc tcs3701_get_ps_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint16_t *raw_data)
{
    int rv = SNS_RC_SUCCESS;
    uint8_t buffer[2];

    static int ps_raw_over_range = 0;
    rv |= tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_PDATAL_REG, &buffer[0]);
    rv |= tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_PDATAH_REG, &buffer[1]);

    *raw_data = (uint16_t)buffer[1] << 8 | (uint16_t)buffer[0];

    OPLUS_ALS_PS_LOG("ps raw data = %d", *raw_data);

    if (rv != SNS_RC_SUCCESS) {
        *raw_data = 0;
    }

    if (*raw_data > 0x3FF) {
        *raw_data = 0;
        ps_raw_over_range++;

        if (ps_raw_over_range >= 4) {
            ps_raw_over_range = 0;
            tcs3701_recover_device(scp_service, port_handle);
        }

        OPLUS_ALS_PS_LOG("ps raw data = %d bigger than the range", *raw_data);
    } else {
        ps_raw_over_range = 0;
    }

    if (i2c_error_count >= 10) {
        i2c_error_count = 0;
        tcs3701_recover_device(scp_service, port_handle);
    }

    return rv;
}

uint16_t tcs3701_median_filter(uint16_t data[], uint8_t count)
{
    int8_t i, j;
    uint16_t temp;

    for (i = 1; i < count; i++) {
        temp = data[i];
        j = i - 1;

        while (data[j] > temp && j >= 0) {
            data[j + 1] = data[j];
            j--;
        }

        if (j != (i - 1)) {
            data[j + 1] = temp;
        }
    }

    return data[(count - 1) / 2];
}


float tcs3701_get_lux()
{
    float lux = 0.0;
    lux = tcs_state.ch_raw[0] + tcs_state.ch_raw[1];

    if (lux < 0.0) {
        lux = 0.0;
    }

    lux /= tcs_state.again;
    lux *= get_als_normal_value();
    return lux;
}


bool tcs3701_auto_gain(sns_sync_com_port_service *scp_service,
     sns_sync_com_port_handle *port_handle,uint16_t again, uint16_t ch_atg[3], uint32_t saturation, uint32_t low_saturation)
{
    /* Auto gain */
    bool again_adjusted = true;

    if (again == 1057 && ch_atg[0] < low_saturation) {
        tcs3701_set_als_gain(scp_service, port_handle, 2204);
    } else if ((again == 2204 && (ch_atg[0] > saturation || ch_atg[2] > saturation ))
        || (again == 549 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 1057);
    } else if ((again == 1057 && (ch_atg[0] > saturation || ch_atg[2] > saturation ))
        || (again == 262 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 549);
    } else if ((again == 549 && (ch_atg[0] > saturation  || ch_atg[2] > saturation ))
        || (again == 128 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 262);
    } else if ((again == 262 && (ch_atg[0] > saturation  || ch_atg[2] > saturation ))
        || (again == 64 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 128);
    } else if ((again == 128 && (ch_atg[0] > saturation || ch_atg[2] > saturation ))
        || (again == 32 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 64);
    } else if ((again == 64 && (ch_atg[0] > saturation || ch_atg[2] > saturation ))
        || (again == 16 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 32);
    } else if ((again == 32 && (ch_atg[0] > saturation || ch_atg[2] > saturation ))
        || (again == 8 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 16);
    } else if ((again == 16 && (ch_atg[0] > saturation || ch_atg[2] > saturation ))
        || (again == 4 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 8);
    } else if ((again == 8 && (ch_atg[0] > saturation  || ch_atg[2] > saturation ))
        || (again == 2 && ch_atg[0] < low_saturation)) {
        tcs3701_set_als_gain(scp_service, port_handle, 4);
    } else if (again == 4 && (ch_atg[0] > saturation   || ch_atg[2] > saturation )) {
        tcs3701_set_als_gain(scp_service, port_handle, 2);
    } else {
        again_adjusted = false;
    }

    return again_adjusted;
}

//select als value to report
int tcs3701_als_hb_select_reported_value(float arr[], int n)
{
    int ii = 0;
    int jj = 0;
    int count = 1;
    int max_count = 1;
    int index = 0;

    index = n - 1;

    for (ii = 0; ii < n ; ii++) {
        if (arr[ii] < 0) {
            continue;
        }

        for (jj = ii + 1; jj < n; jj++) {
            if (((int)arr[ii]) == ((int)arr[jj])) {
                count++;
                arr[jj] = -arr[jj];
            }
        }

        if (count > max_count) {
            max_count = count;
            index = ii;
        }

        count = 1;
    }

    return index;
}
#if 0
static void tcs3701_dump_reg(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    uint32_t xfer_bytes;
    uint8_t reg_map[126];

    uint8_t i = 0;
    uint16_t n = sizeof(reg_map) / sizeof(reg_map[0]);

    tcs3701_com_read_wrapper(scp_service,
        port_handle,
        0x80,
        &reg_map[0],
        n,
        &xfer_bytes);

    for (i = 0; i < n; i++) {
        OPLUS_ALS_PS_LOG("tcs3701 reg[0x%X] = 0x%X",
            i + 128, reg_map[i]);
    }
}
#endif

#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
static bool tcs3701_als_algo_adjust_atime(tcs3701_instance_state *state,
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint16_t atime_us)
{
    bool atime_adjusted = true;

    if (COMPENSATION_ALGO == state->algo_state) {
        if(state->brightness <= lb_dis_coefs.dbv_coefs->dbv_h2l) {
            state->algo_state = LOW_BRIGHTNESS_ALGO;
        }

    } else if (LOW_BRIGHTNESS_ALGO == state->algo_state) {
        if (state->brightness > lb_dis_coefs.dbv_coefs->dbv_l2h) {
            state->algo_state = COMPENSATION_ALGO;
        }
    } else {
        state->algo_state = COMPENSATION_ALGO;
    }

    if (COMPENSATION_ALGO == state->algo_state) {
        if (!FIT_ATIME(atime_us, lb_dis_coefs.dbv_coefs->atime_comp_algo)) {
            tcs3701_set_als_time(scp_service, port_handle, lb_dis_coefs.dbv_coefs->atime_comp_algo);
            tcs3701_set_als_gain(scp_service, port_handle, 262);
        } else {
            atime_adjusted = false;
        }
    }

    if (LOW_BRIGHTNESS_ALGO == state->algo_state) {
        if (state->brightness <= lb_dis_coefs.dbv_coefs->dbv_1 &&
                !FIT_ATIME(atime_us, lb_dis_coefs.dbv_coefs->atime_1)) {
            tcs3701_set_als_time(scp_service, port_handle, lb_dis_coefs.dbv_coefs->atime_1);
        } else if ((state->brightness > lb_dis_coefs.dbv_coefs->dbv_1 &&
                state->brightness <= lb_dis_coefs.dbv_coefs->dbv_2) &&
                !FIT_ATIME(atime_us, lb_dis_coefs.dbv_coefs->atime_2)) {
            tcs3701_set_als_time(scp_service, port_handle, lb_dis_coefs.dbv_coefs->atime_2);
        } else if ((state->brightness > lb_dis_coefs.dbv_coefs->dbv_2 &&
                state->brightness <= lb_dis_coefs.dbv_coefs->dbv_3) &&
                !FIT_ATIME(atime_us, lb_dis_coefs.dbv_coefs->atime_3)) {
            tcs3701_set_als_time(scp_service, port_handle, lb_dis_coefs.dbv_coefs->atime_3);
        } else if ((state->brightness > lb_dis_coefs.dbv_coefs->dbv_3 &&
                state->brightness <= lb_dis_coefs.dbv_coefs->dbv_4) &&
                !FIT_ATIME(atime_us, lb_dis_coefs.dbv_coefs->atime_4)) {
            tcs3701_set_als_time(scp_service, port_handle, lb_dis_coefs.dbv_coefs->atime_4);
        } else {
            atime_adjusted = false;
        }

    }

    if (LOW_BRIGHTNESS_ALGO == state->algo_state) {
        OPLUS_ALS_PS_LOG("brightness %d, algo_state: LB algo", state->brightness);
    } else {
        OPLUS_ALS_PS_LOG("brightness %d, algo_state: HB algo", state->brightness);
    }
    return atime_adjusted;
}

void tcs3701_fifo_get_lux(float *als_event)
{
    float lux = 0.0, lux1 = 0.0, lux2 = 0.0, ratio1 = 0.0, ratio2 = 0.0;
    static float last_ch0_raw = 0;
    static float last_ch1_raw = 0;
    uint8_t sample_cnt =  tcs_state.algo_state == COMPENSATION_ALGO ? 1 : GROUP_CNT;

    if (tcs_state.algo_state == COMPENSATION_ALGO) {
        lux = tcs_state.ch_raw[0] + tcs_state.ch_raw[1];
    lux /= tcs_state.again;
    lux *= get_als_normal_value();

    if ((tcs_state.ch_raw[1] + tcs_state.ch_raw[0]) < 0.01) {
            als_event[1] = 1.0;
    } else {
            als_event[1] = (float)(tcs_state.ch_raw[1]) / tcs_state.ch_raw[0];
    }
    als_event[1] *= 100;

    } else {
        /******  reno 2
        if (tcs_state.ch_raw[0] < 1.0) {
    tcs_state.ch_raw[0] = 1.0;
    }
    ratio1 = tcs_state.ch_raw[2] / tcs_state.ch_raw[0];
    ratio2 = tcs_state.ch_raw[1] / tcs_state.ch_raw[0];
    ratio1 = lb_dis_coefs.coef_k * ratio1 + lb_dis_coefs.coef_k1;

    if (ratio2 > (ratio1 + 0.0005) || ratio2 < (ratio1 - 0.0005))
    ratio2 = ratio1;

    tcs_state.ch_raw[1] = tcs_state.ch_raw[0] * ratio2;
    if ((tcs_state.ch_raw[0] < (last_ch0_raw + 3 * sample_cnt)
    && tcs_state.ch_raw[0] > (last_ch0_raw - 3 * sample_cnt)) && last_ch1_raw != 0) {
    tcs_state.ch_raw[1] = tcs_state.ch_raw[0]/last_ch0_raw * last_ch1_raw;
    }
    last_ch0_raw = tcs_state.ch_raw[0];
    last_ch1_raw = tcs_state.ch_raw[1];

    lux1 = tcs_state.ch_raw[0] * lb_dis_coefs.coef_a - tcs_state.ch_raw[1] * lb_dis_coefs.coef_b;
    lux1 *= lb_dis_coefs.dgf * 1.1;
    lux2 = tcs_state.ch_raw[0] * lb_dis_coefs.coef_c - tcs_state.ch_raw[1] * lb_dis_coefs.coef_d;
    lux2 *= lb_dis_coefs.dgf * 1.1;

    lux = lux1 > lux2 ? lux1 : lux2;
    lux /= sample_cnt;
    lux /= (tcs_state.again * (tcs_state.atime / 1000));
    ********/
        lux = tcs_state.ch_raw[0] / sample_cnt;
        lux = lux / (tcs_state.again * (tcs_state.atime / 1000)) * 20/*ms*/;
        UNUSED_VAR(lux1);
        UNUSED_VAR(lux2);
        UNUSED_VAR(ratio1);
        UNUSED_VAR(ratio2);
        UNUSED_VAR(last_ch0_raw);
        UNUSED_VAR(last_ch1_raw);
        if (lux < 0.0) {
            lux = 0.0;
        }

        lux = lux * 549/*gain*/ - tcs_state.ch_raw[3];
        lux *= 0.25;// coe para
        lux -= 1;
        if (lux < 0.0) {
            lux = 0.0;
        }

        als_event[2] = 0;

    }
    als_event[0] = lux;
}


static sns_rc tcs3701_handle_fifo(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, float *als_event,int len)
{
    UNUSED_VAR(len);
    uint16_t again, atime_us;
    uint8_t buffer[2] = {0};
    uint8_t data[256] = {0};
    uint16_t ch_atg[3] = {0};
    uint8_t length = 0;
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t saturation = 0;
    bool atime_adjusted = false;
    bool again_adjusted = false;
    bool fifo_overflow = false;
    bool get_sample = false;
    uint8_t get_data = 0;
    uint8_t  algo_adjust = 0;

    /* adjust atime */
    rv |= tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_ASTEPL_REG, &buffer[0]);
    rv |= tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_ASTEPH_REG, &buffer[1]);
    atime_us = (uint16_t)(((uint32_t)buffer[1] << 8 | (uint32_t)buffer[0])+1) * ASTEP_US_PER_100 / 100.0;
    tcs_state.atime = atime_us;

    if (!tcs_state.first_als) {
        atime_adjusted = tcs3701_als_algo_adjust_atime(&tcs_state, scp_service, port_handle, atime_us);
    }

    if (atime_adjusted) {
        goto exit;
    }

    saturation = (uint32_t)((uint32_t)buffer[1] << 8 | (uint32_t)buffer[0])+1;
    saturation = saturation > MAX_ALS_VALUE ? MAX_ALS_VALUE : saturation;
    tcs_state.saturation = saturation * 8 / 10;
    tcs_state.low_saturation = saturation * 2 / 10;
    /* read fifo length */
    rv |= tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_RVED2_REG, &length);
    OPLUS_ALS_PS_LOG("TCS3701 fifo length=%d\n",length);
    if (128 == length) {
        fifo_overflow = true;
        goto exit;
    }
    length = (uint16_t)(length / CHANNEL_COUNT) * CHANNEL_COUNT;
    if (length == 0) {
        SET_EVENT_UNVALID(als_event);
        goto exit;
    }

    tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_CFG1_REG, &buffer[0]);
    again = buffer[0];
    again = tcs3701_again[(again&AGAIN_MASK)>>AGAIN_SHIFT];
    tcs_state.again = again;

    OPLUS_ALS_PS_LOG("TCS3701 atime=%d, again=%d, saturation = %d",
            (int)tcs_state.atime,(int)again, saturation);

    tcs3701_inst_i2c_read_block(scp_service, port_handle, TCS3701_RVED3_REG, &data[0], (length<<1));
    if (LOW_BRIGHTNESS_ALGO == tcs_state.algo_state) {
        get_sample = als_lb_algo(data, length, &algo_adjust, atime_us, again, ch_atg, tcs_state.ch_raw, &get_data);
    } else {
        tcs_state.ch_raw[0] = (float)((data[1] << 8) | data[0]);
        tcs_state.ch_raw[1] = (float)((data[3] << 8) | data[2]);
        tcs_state.ch_raw[2] = (float)((data[5] << 8) | data[4]);
        ch_atg[0] = tcs_state.ch_raw[0];
        ch_atg[1] = tcs_state.ch_raw[1];
        ch_atg[2] = tcs_state.ch_raw[2];
        als_event[2] = ALS_TIMER_MS;
        get_sample = true;
        get_data = 1;
        OPLUS_ALS_PS_LOG("HB ch0 %d, ch1 %d, ch2 %d",
                (int)tcs_state.ch_raw[0],(int)tcs_state.ch_raw[1],(int)tcs_state.ch_raw[2]);
    }

    /* adjust again */
    if (!tcs_state.first_als && get_data) {
        again_adjusted = tcs3701_auto_gain(scp_service,
                port_handle,again,
                ch_atg,tcs_state.saturation,
                tcs_state.low_saturation);
    }
    if (tcs_state.first_als && get_data) {
        if (tcs_state.ch_raw[1] > tcs_state.saturation && 16 != again) {
            again_adjusted = true;
            tcs3701_set_als_gain(scp_service, port_handle, 16);
        }
    }

exit:

    if (atime_adjusted || again_adjusted || fifo_overflow || algo_adjust) {
        SET_EVENT_UNVALID(als_event);
        lb_algo_clear_fifo();
        get_sample = false;
        tcs3701_modify_enable(scp_service, port_handle, AEN, 0);
        tcs3701_clear_fifo(scp_service,port_handle);
        tcs3701_modify_enable(scp_service, port_handle, AEN, AEN);
    }

    if (get_sample) {
        tcs3701_fifo_get_lux(als_event);
        SET_EVENT_VALID(als_event);
        if (tcs_state.first_als)
            tcs_state.first_als = false;
    }

    EVENT_ALGO_STATE(als_event) = tcs_state.algo_state;
    return rv;
}
#endif

static sns_rc tcs3701_get_als_data(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, float *als_event, int len,
    uint8_t als_type, bool is_als_dri)
{
    UNUSED_VAR(len);
    UNUSED_VAR(als_type);
    UNUSED_VAR(is_als_dri);
    uint16_t again;
    uint8_t buffer[2] = {0};
    uint8_t data[10] = {0};
    uint16_t ch_atg[3] = {0};
    int rv = 0;
    bool again_adjusted = false;

    tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_CFG1_REG, &buffer[0]);
    again = buffer[0];
    again = tcs3701_again[(again & AGAIN_MASK) >> AGAIN_SHIFT];

    tcs_state.again = again;

    OPLUS_ALS_PS_LOG("TCS3701 atime=%d, again=%d buffer =%d n",
        (int)tcs_state.atime, (int)again, buffer[0]);


    /*get the rgb*/
    tcs3701_inst_i2c_read_block(scp_service, port_handle, TCS3701_ADATA0L_REG, &data[0], 6);

    tcs_state.ch_raw[0] = (float)((data[1] << 8) | data[0]);
    tcs_state.ch_raw[1] = (float)((data[3] << 8) | data[2]);
    tcs_state.ch_raw[2] = (float)((data[5] << 8) | data[4]);
    ch_atg[0] = tcs_state.ch_raw[0];
    ch_atg[1] = tcs_state.ch_raw[1];
    ch_atg[2] = tcs_state.ch_raw[2];
    als_event[2] = ALS_TIMER_MS;

    if (!tcs_state.first_als) {
        again_adjusted = tcs3701_auto_gain(scp_service,
                port_handle, again,
                ch_atg, tcs_state.saturation,
                tcs_state.low_saturation);
    }

    if (again_adjusted) {
        als_event[0] = (uint32_t)tcs_state.last_lux;
        als_event[1] = (uint32_t)tcs_state.ir_ratio;
        return rv;
    }

    if (tcs_state.first_als) {
        if ((ch_atg[0] > tcs_state.saturation && ch_atg[2] > tcs_state.saturation) && again != 32) {
            tcs3701_set_als_gain(scp_service, port_handle, 32);
            return SNS_RC_FAILED;
        }
        tcs_state.first_als = false;
    }

    tcs_state.lux = tcs3701_get_lux();
    /* Reserve one digit after the decimal point */
    tcs_state.lux = (float)((uint32_t)((tcs_state.lux + 0.05) * 10)) / 10.0;

    //tcs_state.ir_ratio = (float)(tcs_state.ch2_raw / tcs_state.ch0_raw);
    if ((tcs_state.ch_raw[1] + tcs_state.ch_raw[0]) < 0.01) {
        tcs_state.ir_ratio = 1.0;
    } else {
        tcs_state.ir_ratio = (float)(tcs_state.ch_raw[2] / (tcs_state.ch_raw[1] + tcs_state.ch_raw[0]));
    }

    als_event[0] = (uint32_t)tcs_state.lux;
    tcs_state.last_lux = tcs_state.lux;
    als_event[1] = (uint32_t)(tcs_state.ir_ratio * 100);
    OPLUS_ALS_PS_LOG("alsps_algo: atime: %d, again: %d, ch0: %d, ch1: %d, ch2: %d, lux: %d ir_radio :%d",
        (int)tcs_state.atime, again, (int)(tcs_state.ch_raw[0]),
        (int)(tcs_state.ch_raw[1]), (int)(tcs_state.ch_raw[2]),
        (int)(tcs_state.lux), (int)als_event[1]);
    return rv;
}


sns_rc tcs3701_hardware_cali(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    OPLUS_ALS_PS_LOG("driver hard cali");
    /* Config before start offset calibration */
    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_ENABLE_REG, PON);

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_INTENAB_REG, CIEN);

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_STATUS_REG, 0xFF);//clean the interrupt

    rv = tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_CALIB_REG, START_OFFSET_CALIB);

    return rv;
}

sns_rc tcs3701_read_irq_flag(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint8_t* status)
{
    sns_rc rv = SNS_RC_SUCCESS;

    rv = tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_STATUS_REG, status);

    return rv;
}

sns_rc tcs3701_get_offset(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, int* offset)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t offset_l = 0, offset_h = 0;

    rv = tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_POFFSETL_REG, &offset_l);
    rv |= tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_POFFSETH_REG, &offset_h);

    if (select_ps_offset_cal()) {
        *offset = (((0xff & offset_h) << 8) | (0xff & offset_l));
    } else {
        if (offset_h == 0xFF) {
            *offset = offset_l - 0xFF;
        } else {
            *offset = offset_l;
        }
    }

    OPLUS_ALS_PS_LOG("offset: %d, l: %d, h: %d, rv : %d\n", *offset, offset_l, offset_h, rv);

    return rv;
}

void tcs3701_ps_offset_cali(float data_0,float data_5,int *offset,float *temp_offset)
{
    if (offset == NULL || temp_offset == NULL) {
        OPLUS_ALS_PS_LOG("ERROR:offset OR temp_offset is NULL\n");
        return;
    }

    float temp_offset1 = 0;
    float temp_offset2 = 0;
    if (select_ps_offset_cal()) {
        OPLUS_ALS_PS_LOG("tcs3701_ps_offset_cali:select_ps_offset_cal is true");
        if (data_0 >= 0xFF00) {
            temp_offset1 = (data_0 - 0xFF00) - 0xFF;
        } else {
            temp_offset1 = data_0;
        }

        if (data_5 >= 0xFF00) {
            temp_offset2 = (data_0 - 0xFF00) - 0xFF;
        } else {
            temp_offset2 = data_5;
        }

        if (temp_offset1 > temp_offset2) {
            if (temp_offset1 < 0) {
                *offset = temp_offset1 + 0xFFFF;
            } else {
                *offset = data_0;
            }
            *temp_offset = temp_offset1;
        } else {
            if (temp_offset2 < 0) {
                *offset = temp_offset2 + 0xFFFF;
            } else {
                *offset = data_5;
            }
            *temp_offset = temp_offset2;
        }
    } else {
        OPLUS_ALS_PS_LOG("tcs3701_ps_offset_cali:select_ps_offset_cal is false");
        if (data_0 > data_5) {
            *offset = data_0;
        } else {
            *offset = data_5;
        }
        *temp_offset = *offset;
    }
    OPLUS_ALS_PS_LOG("tcs3701_ps_offset_cali:data_0 = %d,data_5 = %d,temp_offset1 %d,temp_offset2 =%d,temp_offset= %d,state->ps_info.offset = %d",
        (int)(data_0),(int)(data_5),(int)temp_offset1,(int)temp_offset2,(int)(*temp_offset),(int)(*offset));
}

static sns_rc tcs3701_clear_ps_int(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t status = 0;

    tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_STATUS_REG, &status);

    if (0 != status) {
        tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_STATUS_REG, status);
    }

    return rv;
}

sns_rc tcs3701_get_hwcali_result(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, bool* result)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t status = 0;
    int offset = 0;
    bool test_result = false;
    uint8_t enable = 0;
    uint8_t intenab = 0;

    rv = tcs3701_read_irq_flag(scp_service, port_handle, &status);

    OPLUS_ALS_PS_LOG("status: %d, rv: %d \n", status, rv);

    if (rv == SNS_RC_SUCCESS) {
        if (CINT == (status & CINT)) {
            rv = tcs3701_get_offset(scp_service, port_handle, &offset);

            if (rv == SNS_RC_SUCCESS) {
                test_result = true;
            } else {
                test_result = false;
            }
        } else {
            test_result = false;
        }
    } else {
        test_result = false;
    }

    *result = test_result;

    tcs3701_clear_ps_int(scp_service, port_handle);

    if (tcs_state.publish_sensors & TCS3701_ALS) {
        if (!tcs_state.use_fifo) {
            intenab |= AIEN;
        }
        tcs3701_modify_intenab(scp_service, port_handle, AIEN, intenab);

        enable |= AEN;
        enable |= PON;
        tcs3701_modify_enable(scp_service, port_handle, (AEN | PON), enable);
    }

    if (tcs_state.publish_sensors & TCS3701_PROX) {
        intenab |= PIEN0;
        tcs3701_modify_intenab(scp_service, port_handle, (PIEN0 | CIEN | PIEN1), intenab);

        enable |= PEN;
        enable |= PON;
        tcs3701_modify_enable(scp_service, port_handle, (PEN | PON), enable);
    }

    if (tcs_state.publish_sensors == 0) {
        /* close ps */
        tcs3701_modify_enable(scp_service, port_handle, 0xFF, 0);
    }

    return rv;
}

sns_rc tcs3701_get_ps_device_irq_mask(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle, uint8_t * mask)
{
    int rv = 0;
    uint8_t status = 0, enable = 0, intenab = 0, temp_status = 0, retry_count = 0;
    uint16_t ps_rawdata = 0;
    alsps_ps_state ps_status = 0;
    OPLUS_ALS_PS_LOG("tcs3701_get_ps_device_irq_mask");

    tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_STATUS_REG, &status);
    temp_status = status;

    while ((0 != temp_status) && (retry_count++ < 5)) {
        tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_STATUS_REG, temp_status);
        tcs3701_inst_i2c_read_byte(scp_service, port_handle, TCS3701_STATUS_REG, &temp_status);
        status = (temp_status | status);
    }

    OPLUS_ALS_PS_LOG("tcs3701_get_ps_device_irq_mask status = %d retry_count = %d temp_status = %d",
        status, retry_count, temp_status);

    if (retry_count == 5) {
        tcs3701_inst_i2c_write_byte(scp_service, port_handle, TCS3701_STATUS_REG, 0xFF);
    }

    if (status & CINT) {
        if (tcs_state.publish_sensors & TCS3701_ALS) {
            enable |= AEN;
        }

        if (tcs_state.publish_sensors & TCS3701_PROX) {
            enable |= PEN;
        }

        if ((tcs_state.publish_sensors & TCS3701_ALS) && !tcs_state.use_fifo) {
            intenab |= AIEN;
        }

        if (tcs_state.publish_sensors & TCS3701_PROX) {
            intenab |= PIEN0;
        }

        if (enable != 0) {
            tcs3701_modify_intenab(scp_service, port_handle, (PIEN0 | AIEN | CIEN), intenab);
            //to force interrupt to change the thrd
            tcs3701_ps_set_thd(scp_service, port_handle, 0, 0, PRX_NEAR_BY_UNKNOWN);
            tcs3701_modify_enable(scp_service, port_handle, (PEN | AEN), enable);
        } else {
            tcs3701_modify_enable(scp_service, port_handle, 0xFF, 0);
        }

        *mask |= PS_CALI;
    }

    if (status & AINT) {
        *mask |= ALS_INT;
    }

    if (status & PINT0) {
        *mask |= PS_INT;
        tcs3701_get_ps_data(scp_service, port_handle, &ps_rawdata);

        if (ps_rawdata > tcs_state.near_thd) {
            ps_status = RX_NEAR_BY;
        } else if (ps_rawdata < tcs_state.far_thd) {
            ps_status = PRX_FAR_AWAY;
        } else {
            ps_status = PRX_FAR_AWAY;
        }

        tcs3701_ps_set_thd(scp_service, port_handle, tcs_state.near_thd, tcs_state.far_thd, ps_status);

        if (tcs_state.first_prox) {
            tcs_state.first_prox = false;
            tcs3701_modify_enable(scp_service, port_handle, (PEN), 0);
            sns_busy_wait(sns_convert_ns_to_ticks(4* 1000 * 1000));//add for avoid the register not take effect
            tcs3701_set_prox_pers(scp_service, port_handle, ALSPS_PERSIST_CONFIG);
            tcs3701_modify_enable(scp_service, port_handle, (PEN), PEN);
        }
    }

    return rv;
}

static bool tcs3701_prox_need_ir_info()
{
    return false;
}

sns_rc tcs3701_recover_device(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle)
{
    sns_rc rv = SNS_RC_SUCCESS;
    rv |= tcs3701_reset_device(scp_service, port_handle, ALS);
    //if als enable set the als enable
    if (tcs_state.publish_sensors & TCS3701_ALS) {
        tcs_state.publish_sensors &= ~TCS3701_ALS;
        rv |= tcs3701_als_enable(scp_service, port_handle, 1);
    }

    if (tcs_state.publish_sensors & TCS3701_PROX) {
        tcs_state.first_prox = true;
        tcs3701_set_offset(scp_service, port_handle, tcs_state.offset);
        rv |= tcs3701_ps_enable(scp_service, port_handle, 1);
    }

    return rv;
}

void tcs3701_set_brightness(uint16_t brightness)
{
    tcs_state.brightness = brightness;
}

void tcs3701_fifo_enable(bool use_fifo)
{
    tcs_state.use_fifo = use_fifo;
}
struct alsps_als_operations tcs3701_als_ops = {
    .get_who_am_i = tcs3701_get_who_am_i,
    .init_driver = tcs3701_init_als,
    .deinit_driver = tcs3701_deinit_als,
    .als_enable = tcs3701_als_enable,
    .clear_als_int = NULL,
    .init_irq = NULL,
    .get_als_data = tcs3701_get_als_data,

#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
    .get_als_fifo_data = tcs3701_handle_fifo,
#else
    .get_als_fifo_data = NULL,
#endif
    .dump_reg = NULL,
    .get_als_device_irq_mask = tcs3701_get_ps_device_irq_mask,
    .reconfig_reg_table = tcs3701_reconfig_reg,
    .set_brightness = tcs3701_set_brightness,
    .enable_fifo = tcs3701_fifo_enable
};

struct alsps_ps_operations tcs3701_ps_ops = {
    .get_who_am_i = tcs3701_get_who_am_i,
    .init_driver = tcs3701_init_als,
    .ps_enable = tcs3701_ps_enable,
    .deinit_driver = tcs3701_deinit_als,
    .clear_ps_int = NULL,
    .init_irq = NULL,
    .get_ps_data = tcs3701_get_ps_data,
    .ps_set_thd = tcs3701_ps_set_thd,
    .dump_reg = NULL,
    .set_offset = tcs3701_set_offset,
    .hardware_cali = tcs3701_hardware_cali,
    .read_irq_flag = tcs3701_read_irq_flag,
    .get_offset = tcs3701_get_offset,
    .get_hwcali_result = tcs3701_get_hwcali_result,
    .get_ps_device_irq_mask = tcs3701_get_ps_device_irq_mask,
    .prox_need_ir_info = tcs3701_prox_need_ir_info,
    .get_ps_original_data = NULL,
    .get_ir_data = NULL,
    .recover_device = tcs3701_recover_device,
    .reconfig_reg_table = tcs3701_reconfig_reg,
    .special_process_before_avaliable = NULL,
    .ps_offset_cali = tcs3701_ps_offset_cali
};
#endif
