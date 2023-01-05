/*
 * Copyright (c) 2018, ams AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "sns_com_port_types.h"
#include "sns_event_service.h"
#include "sns_gpio_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_sensor_event.h"
#include "sns_sensor_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_time.h"
#include "sns_types.h"
#include "sns_memmgr.h"
#include "sns_island.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_pb_util.h"

#include "sns_std_sensor.pb.h"

#include "sns_diag.pb.h"
#include "sns_diag_service.h"
#include "sns_std.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_timer.pb.h"

#include "sns_cal_util.h"
#include "sns_printf.h"

#include "sns_time.h"
#include <math.h>
#ifdef TCS3408R_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#include "oppo_sensor_para/tcs3408_rear_algo_para.h"
#endif

#include "sns_tcs3408r_hal.h"
#include "sns_tcs3408r_sensor.h"
#include "sns_tcs3408r_sensor_instance.h"
#include "../../logger/inc/sns_logger_sensor_instance.h"

#include "kiss_fft.h"
#define FFT_BIN_COUNT		256
#define SAMPLE_FREQ			(1000 / 0.975)
#define FREQ_STEP			(SAMPLE_FREQ * 1000 / FFT_BIN_COUNT);

//#define TCS3408_AUTO_DEBUG

/* Need to use ODR table. */
//extern const tcs3408_odr_reg_map tcs3408_reg_map[TCS3408_REG_MAP_TABLE_SIZE];

typedef struct log_sensor_state_raw_info
{
	/* Pointer to diag service */
	sns_diag_service *diag;
	/* Pointer to sensor instance */
	sns_sensor_instance *instance;
	/* Pointer to sensor UID*/
	struct sns_sensor_uid *sensor_uid;
	/* Size of a single encoded sample */
	size_t encoded_sample_size;
	/* Pointer to log*/
	void *log;
	/* Size of allocated space for log*/
	uint32_t log_size;
	/* Number of actual bytes written*/
	uint32_t bytes_written;
	/* Number of batch samples written*/
	/* A batch may be composed of several logs*/
	uint32_t batch_sample_cnt;
	/* Number of log samples written*/
	uint32_t log_sample_cnt;
} log_sensor_state_raw_info;

#ifdef TCS3408R_GET_PARAMETER_FROM_SMEM
static struct sensor_hw *tcs3408r_hw = NULL;
static struct sensor_feature *tcs3408r_feature = NULL;
#endif

float ir_devers = 0.3;
float lux_coef_Hir[RAW_NUM] = {0.029260,	-0.007529,	-0.021196,	-0.001236,	-0.002619};
float lux_coef_Lir[RAW_NUM] = {0.024482,	-0.011578,	0.029794,	-0.032807,	-0.007669};
uint8_t smux_data_r[20] = {
	0x14, 0x25, 0x23, 0x41,
	0x33, 0x12, 0x14, 0x24,
	0x53, 0x23, 0x15, 0x14,
	0x32, 0x44, 0x21, 0x23,
	0x13, 0x54, 0x0, 0x67
};/* 4 channel */

float gold_scale_r[2][6] = {
	{6622.5,	3523,	1835.75,	11161.75,	4182.25, 3255}, //3000k
	{4428.75,	4024.75,	3014.75,	10913,	3984,	2704}	//6000k
};

float cct_coef_Hir[CCT_COEF_ROW][CCT_COEF_COL] = {
	{-0.176478,0.106856,	0.876815,	0.200654,	0.112535},
	{0.000000,	0.000000,	1.000000,	0.000000,	0.000000},
	{0.654255,	-1.617507,	1.059303,	0.280351,	0.133328}
};

float cct_coef_Lir[CCT_COEF_ROW][CCT_COEF_COL] = {
	{0.092433,	0.107935,	0.746845,	-0.106719,	-0.080155},
	{0.000000,	0.000000,	1.000000,	0.000000,	0.000000},
	{0.749896,	-0.783842,	-1.123275,	1.326919,	-0.042727}
};

static uint16_t const fd_gains_r[] = {
	0,
	1,
	2,
	4,
	8,
	16,
	32,
	64,
	128,
	256,
	512,
	1024,
	2048
};

uint16_t tcs3408_again_r[] =
{
	0,
	1,
	2,
	4,
	8,
	16,
	32,
	64,
	128,
	256,
	512,
	1024,
	2048
};

sns_rc tcs3408r_dump_reg(sns_sensor_instance *const instance);

#ifdef TCS3408R_GET_PARAMETER_FROM_SMEM
void tcs3408r_set_algo_para()
{
	int para = 0;

	if(oppo_get_sensor_hw(OPPO_CCT_REAR, 0x01, &tcs3408r_hw) && tcs3408r_hw != NULL) {
		oppo_get_sensor_feature(OPPO_CCT_REAR, 0x01, &tcs3408r_feature);
	}
	if (tcs3408r_feature != NULL && tcs3408r_feature->parameter[0] != NULL)
		para = tcs3408r_feature->parameter[0];

	if (para < 0 || para > (TCS3408R_PARA_MAX - 1))
		para = 0;

	memcpy(&ir_devers, &(tcs3408r_algo_param[para].ir_devers), sizeof(ir_devers));
	memcpy(lux_coef_Hir, (tcs3408r_algo_param[para].lux_coef_Hir), sizeof(lux_coef_Hir));
	memcpy(lux_coef_Lir, (tcs3408r_algo_param[para].lux_coef_Lir), sizeof(lux_coef_Lir));
	memcpy(smux_data_r, (tcs3408r_algo_param[para].smux_data), sizeof(smux_data_r));
	memcpy(gold_scale_r, (tcs3408r_algo_param[para].gold_scale), sizeof(gold_scale_r));
	memcpy(cct_coef_Hir, (tcs3408r_algo_param[para].cct_coef_Hir), sizeof(cct_coef_Hir));
	memcpy(cct_coef_Lir, (tcs3408r_algo_param[para].cct_coef_Lir), sizeof(cct_coef_Lir));
}
#endif

/**
 * Allocate Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] log_size   Optional size of log packet to
 *    be allocated. If not provided by setting to 0, will
 *    default to using maximum supported log packet size
 */
void tcs3408r_log_sensor_state_raw_alloc(
                                    log_sensor_state_raw_info *log_raw_info,
                                    uint32_t log_size)
{
	uint32_t max_log_size = 0;

	if (NULL == log_raw_info || NULL == log_raw_info->diag ||
		NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid)
	{
		return;
	}

	/* Allocate memory for sensor state - raw sensor log packet */
	max_log_size = log_raw_info->diag->api->get_max_log_size(log_raw_info->diag);

	if (0 == log_size)
	{
		/* Log size not specified.
		** Use max supported log packet size */
		log_raw_info->log_size = max_log_size;
	}
	else if (log_size <= max_log_size)
	{
		log_raw_info->log_size = log_size;
	}
	else
	{
		return;
	}

	log_raw_info->log = log_raw_info->diag->api->alloc_log(
		log_raw_info->diag,
		log_raw_info->instance,
		log_raw_info->sensor_uid,
		log_raw_info->log_size,
		SNS_DIAG_SENSOR_STATE_LOG_RAW);

	log_raw_info->log_sample_cnt = 0;
	log_raw_info->bytes_written = 0;
}

/**
 * Read wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           read buffer
 * @param[i] bytes            bytes to read
 * @param[o] xfer_bytes       bytes read
 *
 * @return sns_rc
 */
static sns_rc tcs3408r_com_read_wrapper(
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

/**
 * Read one byte value from register for sensor instance using.
 *
 * @param[i] state     state of sensor instance
 * @param[i] reg       register addr to be read
 * @param[i] val       pointer of the buffer to save value
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc tcs3408r_inst_i2c_read_byte(tcs3408_instance_state *state,
                                                   uint8_t reg,
                                                   uint8_t *buffer)
{
	sns_rc rv = SNS_RC_FAILED;
	uint32_t xfer_bytes;
	sns_port_vector port_vec;
	port_vec.buffer = buffer;
	port_vec.bytes = 1;
	port_vec.is_write = false;
	port_vec.reg_addr = reg;

	rv = state->scp_service->api->sns_scp_register_rw(
					state->com_port_info.port_handle,
					&port_vec,
					1,
					false,
					&xfer_bytes);

	if (SNS_RC_SUCCESS != rv || 1 != xfer_bytes)
	{
		rv = SNS_RC_FAILED;
	}

	return rv;
}

/**
 * Read one block value from device for sensor instance using.
 *
 * @param[i] state     state of sensor instance
 * @param[i] reg       register addr to be read
 * @param[i] val       pointer of the buffer to save value
 * @param[i] len       length of the block to be read
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc tcs3408r_inst_i2c_read_block(tcs3408_instance_state *state,
                                                     uint8_t reg,
                                                     uint8_t *buffer,
                                                     uint32_t len)
{
	sns_rc rv = SNS_RC_FAILED;
	uint32_t xfer_bytes;
	sns_port_vector port_vec;
	port_vec.buffer = buffer;
	port_vec.bytes = len;
	port_vec.is_write = false;
	port_vec.reg_addr = reg;

	rv = state->scp_service->api->sns_scp_register_rw(
					state->com_port_info.port_handle,
					&port_vec,
					1,
					false,
					&xfer_bytes);

	if (SNS_RC_SUCCESS != rv || len != xfer_bytes)
	{
		rv = SNS_RC_FAILED;
	}

	return rv;
}

/**
 * Write one byte value from register for sensor instance using.
 *
 * @param[i] state     state of sensor instance
 * @param[i] reg       register addr to be read
 * @param[i] val       the value to be written
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc tcs3408r_inst_i2c_write_byte(tcs3408_instance_state *state,
                                                     uint8_t reg,
                                                     uint8_t val)
{
	sns_rc rv = SNS_RC_FAILED;
	uint32_t xfer_bytes;
	sns_port_vector port_vec;
	port_vec.buffer = &val;
	port_vec.bytes = 1;
	port_vec.is_write = true;
	port_vec.reg_addr = reg;

	rv = state->scp_service->api->sns_scp_register_rw(
					state->com_port_info.port_handle,
					&port_vec,
					1,
					false,
					&xfer_bytes);

	if (SNS_RC_SUCCESS != rv || 1 != xfer_bytes)
	{
		rv = SNS_RC_FAILED;
	}

	return rv;
}

/**
 * Modify the bits value in register for sensor instance using.
 *
 * @param[i] state	   state of sensor instance 
 * @param[i] reg       register addr to be modified
 * @param[i] mask      mask of the bits which will be modified
 * @param[i] val       the value to be written into the mask bits
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc tcs3408r_inst_i2c_modify(tcs3408_instance_state *state,
                                               uint8_t reg,
                                               uint8_t mask,
                                               uint8_t val)
{
	sns_rc rv = SNS_RC_FAILED;
	uint8_t rw_buffer = 0;

	if (0xFF == mask)
	{
		rv = tcs3408r_inst_i2c_write_byte(state, reg, val);
	}
	else
	{
		/* Read current value from this register */
		rv = tcs3408r_inst_i2c_read_byte(state, reg, &rw_buffer);
		if (SNS_RC_SUCCESS == rv)
		{
			rw_buffer &= ~mask;
			rw_buffer |= (val&mask);
			rv = tcs3408r_inst_i2c_write_byte(state, reg, rw_buffer);
		}
	}

	return rv;
}
void tcs3408r_insert_sort(uint16_t data[], uint16_t count)  
{  
    int16_t i, j;
	uint16_t temp;
	for (i = 1; i < count; i++)
	{
		temp = data[i];
		j = i-1;

		while(data[j] > temp && j >= 0)
		{
			data[j+1] = data[j];
			j--;
		}

		if (j != (i-1))
		{
			data[j+1] = temp;
		}
	}
}

sns_rc tcs3408r_get_who_am_i(sns_sync_com_port_service *scp_service,
									sns_sync_com_port_handle *port_handle,
									uint8_t *buffer)
{
	sns_rc rv = SNS_RC_FAILED;
	uint32_t xfer_bytes;

	rv = tcs3408r_com_read_wrapper(scp_service,
			port_handle,
			TCS3408_ID_REG,
			buffer,
			1,
			&xfer_bytes);

	if (SNS_RC_SUCCESS != rv || 1 != xfer_bytes)
	{
		rv = SNS_RC_FAILED;
	}

	return rv;
}

sns_rc tcs3408r_read_enable_reg(tcs3408_instance_state *state, uint8_t *value)
{
    sns_rc rv = SNS_RC_FAILED;
    rv = tcs3408r_inst_i2c_read_byte(state, TCS3408_ENABLE_REG, value);
    if (SNS_RC_SUCCESS != rv)
    {
        return rv;
    }
    return rv;
}

sns_rc tcs3408r_set_smux_config(tcs3408_instance_state *state)
{
    sns_rc rv = SNS_RC_FAILED;
    uint8_t i;

    //20190710 read smux status to copy and write to back
    uint8_t enable_status;
    tcs3408r_read_enable_reg(state,&enable_status);
    tcs3408r_inst_i2c_write_byte(state, TCS3408_ENABLE_REG,0x00);
    tcs3408r_inst_i2c_write_byte(state, TCS3408_ENABLE_REG,0x01);


    /* Send sequnce */
    for (i = 0; i < sizeof(smux_data_r); i++) {
        rv = tcs3408r_inst_i2c_write_byte(state, i, smux_data_r[i]);
        if (SNS_RC_SUCCESS != rv) {
            return rv;
        }
    }

        /* Write command */
    tcs3408r_inst_i2c_write_byte(state, 0xAF, 0x10);

    /* Execute the command */
    tcs3408r_inst_i2c_write_byte(state, TCS3408_ENABLE_REG, (0x10|PON));
    /* Wait 500us */
    sns_busy_wait(sns_convert_ns_to_ticks(500*1000));
    /* Clear command */
    tcs3408r_inst_i2c_write_byte(state, 0xAF, 0x00);

    //write the copyed status to smux
    tcs3408r_inst_i2c_write_byte(state, TCS3408_ENABLE_REG, enable_status);

    return SNS_RC_SUCCESS;
}


/**
 * Loads default values in config registers.
 *
 * @param[i] state       state of sensor instance
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc tcs3408r_device_set_default(tcs3408_instance_state *state)
{
	sns_rc rv = SNS_RC_FAILED;
	uint8_t i;

	/* Enable PON bit */
	tcs3408r_inst_i2c_write_byte(state, TCS3408_ENABLE_REG, PON);
	tcs3408r_set_smux_config(state);//only for rgb front

	tcs3408_reg_setting default_setting[] = {
		{TCS3408_ATIME_REG,        0},
	//	{TCS3408_PTIME_REG,        PTIME_MS(50)},
	//	{TCS3408_WTIME_REG,        0},
		{TCS3408_CFG0_REG,         0x00},
		{TCS3408_CFG1_REG,         AGAIN_4X},//als gain
		{TCS3408_CFG3_REG,         (CFG3_RVED|HXTALK_MODE1)},
		{TCS3408_CFG4_REG,	       0x80},//use fd_gain as the adata5's gain value,not again
		{TCS3408_CFG8_REG,         0x98},
		{TCS3408_CFG10_REG,        0xf2},
		{TCS3408_CFG11_REG,        0x40},
		{TCS3408_CFG12_REG,        0x00},
		{TCS3408_CFG14_REG,        0x00},
		{TCS3408_PERS_REG,         (ALS_PERSIST(0)|PROX_PERSIST(0))},
	//	{TCS3408_PCFG1_REG,        (PCFG1_RVED|HXTALK_MODE2)},
	//	{TCS3408_PCFG2_REG,        PLDRIVE_MA(20)},
	//	{TCS3408_PCFG4_REG,        PGAIN_1X},
	//	{TCS3408_PCFG5_REG,        (PPULSES(16)|PPULSE_LEN_8US)},
	//	{TCS3408_AGC_GAIN_MAX_REG, 0x99}, auto gain ctrl
	//	{TCS3408_CALIBCFG0_REG,    0},
	//	{TCS3408_CALIBCFG1_REG,    0},
	//	{TCS3408_CALIBCFG2_REG,    BINSRCH_TARGET_31},
		{TCS3408_INTENAB_REG,      0x00},
		{TCS3408_AZ_CONFIG_REG,    0x00},//close auto als zero
		{TCS3408_FD_CFG0,          0x80},//enable fd_fifo_mode
		{TCS3408_FD_CFG1,          0x5E},//default fd_time = 1ms
		{TCS3408_FD_CFG3,          0x21},//default fd_time = 1ms, default fd_gain = 8x
	//	{TCS3408_FIFO_MAP,         0x00}

	};

	for (i = 0; i < ARR_SIZE(default_setting); i++)
	{
		rv = tcs3408r_inst_i2c_write_byte(state,
					default_setting[i].reg,
					default_setting[i].value);
		if (SNS_RC_SUCCESS != rv)
		{
			return rv;
		}
	}

	tcs3408r_set_als_time(state, 100000);
	tcs3408r_set_fd_time_us(state, 975);

	return SNS_RC_SUCCESS;
}

sns_rc tcs3408r_reset_device(tcs3408_instance_state *state, tcs3408_sensor_type sensor)
{
	sns_rc rv = SNS_RC_FAILED;

	/* Reset Enable reigster only when both ALS and RGB are requested */
	if ((TCS3408_ALS|TCS3408_RGB|TCS3408_FLICKER) == sensor)
	{
		rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_ENABLE_REG, 0x00);
		if (SNS_RC_SUCCESS != rv)
		{
			rv = SNS_RC_FAILED;
			goto reset_exit;
		}
		rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_ENABLE_REG, 0x00);
		if (SNS_RC_SUCCESS != rv)
		{
			rv = SNS_RC_FAILED;
		}
	}

reset_exit:
	if (SNS_RC_SUCCESS == rv)
	{
		rv = tcs3408r_device_set_default(state);
	}

	return rv;
}

sns_rc tcs3408r_modify_control(tcs3408_instance_state *state,
                                     uint8_t mask,
                                     uint8_t val)
{
	return tcs3408r_inst_i2c_modify(state, TCS3408_CONTROL_REG, mask, val);
}

sns_rc tcs3408r_modify_enable(tcs3408_instance_state *state,
                                     uint8_t mask,
                                     uint8_t val)
{
	return tcs3408r_inst_i2c_modify(state, TCS3408_ENABLE_REG, mask, val);
}

sns_rc tcs3408r_modify_intenab(tcs3408_instance_state *state,
                                      uint8_t mask,
                                      uint8_t val)
{
	return tcs3408r_inst_i2c_modify(state, TCS3408_INTENAB_REG, mask, val);
}

sns_rc tcs3408r_clear_fifo(tcs3408_instance_state *state)
{
	sns_rc rv = SNS_RC_FAILED;

	rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_CONTROL_REG, 0x02);

	return rv;
}

sns_rc tcs3408r_get_als_pers(tcs3408_instance_state *state, uint8_t *persistence)
{
	sns_rc rv = SNS_RC_FAILED;

	rv = tcs3408r_inst_i2c_read_byte(state, TCS3408_PERS_REG, persistence);
	if (SNS_RC_SUCCESS == rv)
	{
		*persistence &= APERS_MASK;
		*persistence >>= APERS_SHIFT;
	}

	return rv;
}

sns_rc tcs3408r_set_als_pers(tcs3408_instance_state *state, uint8_t persistence)
{
	return tcs3408r_inst_i2c_modify(state, TCS3408_PERS_REG, APERS_MASK, ALS_PERSIST(persistence));
}

sns_rc tcs3408r_set_als_gain(tcs3408_instance_state *state, uint32_t again)
{
	sns_rc rv = SNS_RC_FAILED;
	uint8_t cfg1 = 0;
	uint8_t enable = 0;

	switch (again)
	{
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
	case 256:
		cfg1 = AGAIN_256X;
		break;
	case 512:
		cfg1 = AGAIN_512X;
		break;
	case 1024:
		cfg1 = AGAIN_1024X;
		break;
	case 2048:
		cfg1 = AGAIN_2048X;
		break;
	default:
		break;
	}

	rv = tcs3408r_inst_i2c_modify(state, TCS3408_CFG1_REG, AGAIN_MASK, cfg1);
	//SNS_INST_PRINTF(LOW, instance, "tcs3408r_set_als_gain again = %d,cfg1 = 0x%02x\n", again,cfg1);
	if (SNS_RC_SUCCESS != rv)
	{
		return rv;
	}

	if (!state->first_als) {
		tcs3408r_inst_i2c_read_byte(state, TCS3408_ENABLE_REG, &enable);
		if (enable & AEN) {
			tcs3408r_modify_enable(state, AEN, 0);
			rv = tcs3408r_modify_enable(state, AEN, AEN);
		} else {
			//do nothing
		}
	}
	
	return rv;
}

sns_rc tcs3408r_set_fd_gain(tcs3408_instance_state *state, uint16_t fd_gain)
{
	sns_rc rv = SNS_RC_FAILED;
	//int32_t ret = 0;
	uint8_t cfg1 = 0;

#ifdef AMS_DEBUG	
	TCS3408_PORT_log_Msg_1(AMS_DEBUG, "set_fd_gain, gain = %d\n", fd_gain);
#endif

	switch (fd_gain)
	{
	case 0:
		cfg1 = FD_GAIN_0_5X;
		break;
	case 1:
		cfg1 = FD_GAIN_1X;
		break;
	case 2:
		cfg1 = FD_GAIN_2X;
		break;
	case 4:
		cfg1 = FD_GAIN_4X;
		break;
	case 8:
		cfg1 = FD_GAIN_8X;
		break;
	case 16:
		cfg1 = FD_GAIN_16X;
		break;
	case 32:
		cfg1 = FD_GAIN_32X;
		break;
	case 64:
		cfg1 = FD_GAIN_64X;
		break;
	case 128:
		cfg1 = FD_GAIN_128X;
		break;
	case 256:
		cfg1 = FD_GAIN_256X;
		break;
	case 512:
		cfg1 = FD_GAIN_512X;
		break;
	case 1024:
		cfg1 = FD_GAIN_1024X;
		break;
	case 2048:
		cfg1 = FD_GAIN_2048X;
		break;

	default:
		break;
	}
	
	tcs3408r_inst_i2c_modify(state, TCS3408_ENABLE_REG, FDEN, 0);	
	rv = tcs3408r_inst_i2c_modify(state, TCS3408_FD_CFG3, FD_GAIN_MASK, cfg1);	
	tcs3408r_inst_i2c_modify(state, TCS3408_CONTROL_REG, FIFO_CLEAR,FIFO_CLEAR);//clear fifo	
	tcs3408r_inst_i2c_modify(state, TCS3408_ENABLE_REG, FDEN, FDEN);  	
	return rv;
}

sns_rc tcs3408r_set_als_time(tcs3408_instance_state *state, uint32_t time_us)
{
	sns_rc rv = SNS_RC_FAILED;
	uint16_t astep = 0;
	uint8_t enable = 0;

	astep = ASTEP_US(time_us);
	rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_ASTEPL_REG, (uint8_t)(astep&0xFF));
	if (SNS_RC_SUCCESS != rv)
	{
		return rv;
	}
	rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_ASTEPH_REG, (uint8_t)((astep&0xFF00)>>8));
	if (!state->first_als) {
		tcs3408r_inst_i2c_read_byte(state, TCS3408_ENABLE_REG, &enable);
		if (enable & AEN) {
			tcs3408r_modify_enable(state, AEN, 0);
			rv = tcs3408r_modify_enable(state, AEN, AEN);
		} else {
			//do nothing
		}
	}
	return rv;
}

sns_rc tcs3408r_set_fd_time_us(tcs3408_instance_state *state, uint32_t fd_time_us)
{
	sns_rc rv = SNS_RC_FAILED;
	uint16_t fd_time = 0;
	uint8_t low_8_bit = 0,high_3_bit = 0;

#ifdef AMS_DEBUG	
	TCS3408_PORT_log_Msg_1(AMS_DEBUG, "set_fd_time_us, time_us: %d\n", fd_time_us);
#endif

	fd_time = FD_TIME_US(fd_time_us);
	low_8_bit = (uint8_t)(fd_time&0xFF);
	high_3_bit = (uint8_t)((fd_time&0x0700)>>8); 

	tcs3408r_inst_i2c_modify(state, TCS3408_ENABLE_REG, FDEN, 0);	
	tcs3408r_inst_i2c_write_byte(state,TCS3408_FD_CFG1,low_8_bit);
	tcs3408r_inst_i2c_modify(state, TCS3408_FD_CFG3, FD_TIME_MASK, high_3_bit);	
	
	tcs3408r_inst_i2c_modify(state, TCS3408_CONTROL_REG, FIFO_CLEAR,FIFO_CLEAR);//clear fifo	
	tcs3408r_inst_i2c_modify(state, TCS3408_ENABLE_REG, FDEN, FDEN);

	return rv;
}


sns_rc tcs3408r_update_als_threshold(tcs3408_instance_state *state,
                                               uint16_t high_thresh, uint16_t low_thresh)
{
	sns_rc rv = SNS_RC_FAILED;
	uint8_t data[4];

	data[0] = (uint8_t)(low_thresh & 0xFF);
	data[1] = (uint8_t)((low_thresh >> 8) & 0xFF);
	data[2] = (uint8_t)(high_thresh & 0xFF);
	data[3] = (uint8_t)((high_thresh >> 8) & 0xFF);

	rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_AILTL_REG, data[0]);
	if (SNS_RC_SUCCESS != rv)
	{
		return rv;
	}
	rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_AILTH_REG, data[1]);
	if (SNS_RC_SUCCESS != rv)
	{
		return rv;
	}
	rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_AIHTL_REG, data[2]);
	if (SNS_RC_SUCCESS != rv)
	{
		return rv;
	}
	rv = tcs3408r_inst_i2c_write_byte(state, TCS3408_AIHTH_REG, data[3]);
	return rv;
}

void run_kiss_fft_r(sns_sensor_instance *instance, uint16_t *data_buf)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;

	kiss_fft_cpx *buffer = NULL;
	uint16_t idx_1st = 0, idx_2nd = 0;
	long long magBf = 0, magSq = 0, temp_1st = 0, temp_2nd = 0;
	bool push2que = false, increasing_status = false;

	if(!sns_island_is_island_ptr((intptr_t)data_buf)) {
		state->island_service->api->sensor_instance_island_exit(state->island_service, instance);
		buffer = (kiss_fft_cpx*) sns_malloc(SNS_HEAP_MAIN, sizeof(kiss_fft_cpx) * FFT_BIN_COUNT);
	} else {
		buffer = (kiss_fft_cpx*) sns_malloc(SNS_HEAP_ISLAND, sizeof(kiss_fft_cpx) * FFT_BIN_COUNT);
		if(buffer == NULL) {
			state->island_service->api->sensor_instance_island_exit(state->island_service, instance);
			buffer = (kiss_fft_cpx*) sns_malloc(SNS_HEAP_MAIN, sizeof(kiss_fft_cpx) * FFT_BIN_COUNT);
		}
	}

	for (int i = 0; i < FFT_BIN_COUNT; i++) {
		buffer[i].r = (float)data_buf[i];
		buffer[i].i = 0;
	}

	kiss_fft_cfg cfg = kiss_fft_alloc(FFT_BIN_COUNT , 0 , 0 , 0);
	kiss_fft(cfg, buffer, buffer);

	magSq = (long long)(buffer[8].r * buffer[8].r) + (long long)(buffer[8].i * buffer[8].i);
	increasing_status = true;//special operation on first data
	for (int i=9; i<(FFT_BIN_COUNT/2);i++) {
		magBf = magSq;
		magSq = (long long)(buffer[i].r * buffer[i].r) + (long long)(buffer[i].i * buffer[i].i);

		// find extremum value
		if (magSq > magBf) {
			increasing_status = true;

			if ((FFT_BIN_COUNT/2 - 1) == i) {//special operation on last data
				push2que = true;
				i++;
				magBf = magSq;
			}
		} else {
			if (increasing_status) {
				push2que = true;
				increasing_status = false;
			}
		}

		//push to queue
		if (push2que) {
			if (magBf > temp_1st) {
				temp_2nd = temp_1st;
				temp_1st = magBf;
				idx_2nd = idx_1st;
				idx_1st = i - 1;
			} else if (magBf > temp_2nd) {
				temp_2nd = magBf;
				idx_2nd = i - 1;
			}
			push2que = false;
		}
	}

	temp_1st = (long long)sqrt((double)temp_1st);
	temp_2nd = (long long)sqrt((double)temp_2nd);
	state->flicker_info.freq1 = idx_1st * FREQ_STEP;
	state->flicker_info.freq2 = idx_2nd * FREQ_STEP;
	state->flicker_info.mag1 = (int)temp_1st;
	state->flicker_info.mag2 = (int)temp_2nd;

	sns_free(cfg);
	sns_free(buffer);
}

static int get_mean_r(uint16_t *buff, int size)
{
	int i;
	int sum = 0;

	for (i = 0; i < size; ++i) {
		sum += buff[i];
	}

	return sum / size;
}

static uint32_t get_stdev_r(uint16_t *buff, int mean, int size)
{
	int i;
	uint32_t sum = 0;

	for (i = 0; i < size; ++i) {
		sum += ((buff[i] - mean) * (buff[i] - mean));
	}
	sum = sum / (size - 1);

	return (uint32_t)sqrt((double)sum);
}

static void get_extremum_id_r(uint16_t data[], uint16_t length, uint16_t* minid, uint16_t* maxid)
{
	*minid = 0;
	*maxid = 0;

	for (int i=0;i<length;i++) {
		if (data[i] > data[*maxid])
			*maxid = i;
		if (data[i] < data[*minid])
			*minid = i;
	}

	return;
}

void tcs3408r_get_fd_data(sns_sensor_instance *instance)
{	
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;
	int rc = 0;

	uint8_t buf[256] = {0};
	uint16_t buf_16[128] = {0};
	int j = 0, k = 0;

	static uint8_t fifo_lvl = 0;

	uint16_t saturation = 0;
	uint16_t fd_gain_switch_high = 0, fd_gain_switch_low = 0;
	uint8_t fd_cfg1_value = 0, fd_cfg3_value = 0;
	uint16_t fd_gain = 0, fd_buf_max = 0;
	static bool fd_gain_adjusted = false;
	static uint16_t raw_data[FFT_BIN_COUNT] = {0};
	uint32_t sum = 0, stdev = 0;
	int mean = 0;
	uint16_t maxid = 0, minid = 0;

	rc = tcs3408r_inst_i2c_read_byte(state, TCS3408_FIFO_STATUS, &fifo_lvl);
	SNS_INST_PRINTF(LOW, instance, "fifo length: %d\n", fifo_lvl);

	if (fifo_lvl == 0) {
		return;
	}

	if (fifo_lvl <= 16) {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], fifo_lvl*2);
	} else if (fifo_lvl <= 32) {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[32], fifo_lvl*2-32);
	} else if (fifo_lvl <= 48) {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[32], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[64], fifo_lvl*2-64);
	} else if (fifo_lvl <= 64) {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[32], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[64], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[96], fifo_lvl*2-96);
	} else if (fifo_lvl <= 80) {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[32], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[64], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[96], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[128], fifo_lvl*2-128);
	} else if (fifo_lvl <= 96) {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[32], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[64], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[96], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[128], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[160], fifo_lvl*2-160);
	} else if (fifo_lvl <= 112) {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[32], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[64], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[96], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[128], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[160], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[192], fifo_lvl*2-192);
	} else {
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[0], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[32], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[64], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[96], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[128], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[160], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[192], 32);
		rc |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &buf[224], fifo_lvl*2-224);
	}
	for(k = 0; k < fifo_lvl; k++){
		buf_16[k] = (uint16_t)(buf[k * 2]) | ((uint16_t)(buf[k * 2 + 1]) << 8);
		if (buf_16[k] > fd_buf_max)
			fd_buf_max = buf_16[k];
	}

	tcs3408r_inst_i2c_read_byte(state, TCS3408_FD_CFG1, &fd_cfg1_value);
	tcs3408r_inst_i2c_read_byte(state, TCS3408_FD_CFG3, &fd_cfg3_value);
	fd_gain = fd_gains_r[(fd_cfg3_value & FD_GAIN_MASK) >> 3];
	saturation = (((uint16_t)(fd_cfg3_value & FD_TIME_MASK) << 8) | (uint16_t)fd_cfg1_value) + 1;
	fd_gain_switch_high = saturation * 8 / 10;
	fd_gain_switch_low = saturation *3 /10;

	// quickly fix a proper fd_gain for first fd data
	if(state->first_fd) {
		state->first_fd = false;
		/*default fd_time is 1ms, so saturation value is 360. 360*80% = 288 default fd_gain is 8x
		if change to 32, not saturated, 288/4 = 72;
		if change to 128, not saturated, 72/4 = 18;
		if change to 512, not saturated, 18/4 = 5;*/
		if (fd_buf_max <= 5) {
			tcs3408r_set_fd_gain(state, 512);
		} else if (fd_buf_max <= 18) {
			tcs3408r_set_fd_gain(state, 128);
		} else if (fd_buf_max <= 72) {
			tcs3408r_set_fd_gain(state, 32);
		} else if (fd_buf_max <= 288) {
		} else {
			tcs3408r_set_fd_gain(state, 1);
		}
		fd_gain_adjusted = true;

	} else {
		fd_gain_adjusted = true;
		//auto fd gain adjust
		if ((fd_gain == 1024) && (fd_buf_max < fd_gain_switch_low)) {
			tcs3408r_set_fd_gain(state, 2048);

		}else if (((fd_gain == 512) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 2048) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 1024);

		}else if (((fd_gain == 256) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 1024) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 512);

		}else if (((fd_gain == 128) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 512) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 256);

		}else if (((fd_gain == 64) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 256) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 128);

		}else if (((fd_gain == 32) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 128) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 64);

		}else if (((fd_gain == 16) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 64) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 32);

		}else if (((fd_gain == 8) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 32) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 16);

		}else if (((fd_gain == 4) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 16) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 8);

		}else if (((fd_gain == 2) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 8) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 4);

		}else if (((fd_gain == 1) && (fd_buf_max < fd_gain_switch_low)) || ((fd_gain == 4) && (fd_buf_max > fd_gain_switch_high))){
			tcs3408r_set_fd_gain(state, 2);

		} else if ((fd_gain == 2) && (fd_buf_max > fd_gain_switch_high)){
			tcs3408r_set_fd_gain(state, 1);
		} else {
			fd_gain_adjusted = false;
		}
	}

	if(fd_gain_adjusted == true) {
		goto get_fd_exit;
	}
	//normalize the fd data to 512x gain.
	for(j = 0; j < fifo_lvl; j++){
		buf_16[j] *= (state->chanl_scale[5] / 1000.0);
		raw_data[state->flicker_info.samples] = buf_16[j] * 512 /fd_gain;
		state->flicker_info.samples++;

		if (state->flicker_info.samples == FFT_BIN_COUNT) {
			sum = 0;
			for (k = 0; k < FFT_BIN_COUNT; k++) {
				sum += raw_data[k];
				if (0 == (k%4))
					SNS_INST_PRINTF(LOW, instance, "DATA[]: [%5d, %5d, %5d, %5d]", raw_data[k], raw_data[k+1], raw_data[k+2], raw_data[k+3]);
			}
			state->flicker_info.ave = sum / FFT_BIN_COUNT;
			state->flicker_info.samples = 0;
			/* Do data sorting */
			get_extremum_id_r(raw_data, FFT_BIN_COUNT, &minid, &maxid);
			state->flicker_info.max = raw_data[maxid];
			state->flicker_info.min = raw_data[minid];

			mean = get_mean_r(raw_data, FFT_BIN_COUNT);
			stdev = get_stdev_r(raw_data, mean, FFT_BIN_COUNT);

			//if(raw_data[maxid] > (mean + (stdev * 6))){
			if(1){
				run_kiss_fft_r(instance, raw_data);
			}else{
			    state->flicker_info.freq1 = 0;//normal data jitter, set freq to 0
				state->flicker_info.freq2 = 0;
				state->flicker_info.mag1 = 0;
				state->flicker_info.mag2 = 0;
				SNS_INST_PRINTF(LOW, instance, "dont calculate fft");
			}
			state->flicker_info.freq1 /= 1000;
			state->flicker_info.freq2 /= 1000;
			state->flicker_info.data_ready = true;
			SNS_INST_PRINTF(LOW, instance, "tcs3408_get_fd_data, freq1 = %d, freq2 = %d, mag1 = %d, mag2 = %d",
				state->flicker_info.freq1, state->flicker_info.freq2, state->flicker_info.mag1, state->flicker_info.mag2);
			SNS_INST_PRINTF(LOW, instance, "tcs3408_get_fd_data, ave = %d, max = %d, min = %d",
				state->flicker_info.ave, state->flicker_info.max, state->flicker_info.min);

#ifdef TCS3408_AUTO_DEBUG
			{
				log_data_info tcs3408_log_data;
				memset(&tcs3408_log_data, 0, sizeof(tcs3408_log_data));
				tcs3408_log_data.sensor_id = SENSOR_TYPE_FLICKER;
				tcs3408_log_data.string_id = SNS_LOGGER_AUTO_DEBUG0;
				tcs3408_log_data.argu2 = state->flicker_info.freq1;
				tcs3408_log_data.argu3 = state->flicker_info.mag1;
				tcs3408_log_data.argu4 = state->flicker_info.freq2;
				tcs3408_log_data.argu5 = state->flicker_info.mag2;
				tcs3408_log_data.argu6 = state->flicker_info.ave;
				sensors_log_report(tcs3408_log_data);
			}
#endif
		}
	}

get_fd_exit:
	if (fd_gain_adjusted) {
		state->flicker_info.samples = 0;
		tcs3408r_clear_fifo(state);
	}
	return;
}

void tcs3408r_get_lux_cct(sns_sensor_instance *instance, bool is_dri)
{
    UNUSED_VAR(is_dri);
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;
	uint16_t again = 0;
	float atime = 0.0;
	float lux = 0.0, calc_ir = 0.0;
	uint8_t data[10] = {0};
	uint32_t saturation, low_saturation = 0;
	bool again_adjusted = false;
	sns_rc rv = SNS_RC_FAILED;

	int i = 0, j = 0, m = 0;
	float X = 0,Y = 0, Z = 0;
	float sum = 0,temp = 0;
	float raw_norm_array[CCT_COEF_COL] = {0};
	float matrix[CCT_COEF_ROW] = {0};

	/* Get ATime, AGain */
	tcs3408r_inst_i2c_read_byte(state, TCS3408_ASTEPL_REG, &data[0]);
	tcs3408r_inst_i2c_read_byte(state, TCS3408_ASTEPH_REG, &data[1]);
	atime = (((uint32_t)data[1] << 8 | (uint32_t)data[0]) + 1) * ASTEP_US_PER_100 / 100.0;
	saturation = (uint32_t)((uint32_t)data[1] << 8 | (uint32_t)data[0])+1;
	saturation = saturation > MAX_ALS_VALUE ? MAX_ALS_VALUE : saturation;
	low_saturation = saturation * 2 / 10;
	saturation = saturation * 8 / 10;
	tcs3408r_inst_i2c_read_byte(state, TCS3408_CFG1_REG, &data[0]);
	again = (uint16_t)tcs3408_again_r[(data[0]&AGAIN_MASK)>>AGAIN_SHIFT];

	SNS_INST_PRINTF(LOW, instance, "TCS3408 get_lux_cct ATime:%d AGain:%d\n", (int)atime, again);
	state->rgb_sample.atime = atime;
	state->rgb_sample.again = again;

	/* Read out ALS raw data */

	rv = tcs3408r_inst_i2c_read_block(state, TCS3408_ADATA0L_REG, &data[0], 10);

	state->rgb_info.clear_raw = (uint16_t)((data[1] << 8) | data[0]);
	state->rgb_info.red_raw   = (uint16_t)((data[3] << 8) | data[2]);
	state->rgb_info.green_raw = (uint16_t)((data[5] << 8) | data[4]);
	state->rgb_info.blue_raw  = (uint16_t)((data[7] << 8) | data[6]);
	state->rgb_info.wide_raw  = (uint16_t)((data[9] << 8) | data[8]);

	/* Auto gain */
	again_adjusted = true;
	if (again == 512 && state->rgb_info.clear_raw < 100) {
		tcs3408r_set_als_gain(state, 1024);
	} else if ((again == 1024 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation) )
		|| (again == 256 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 512);
	} else if ((again == 512 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation) )
		|| (again == 128 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 256);
	} else if ((again == 256 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation) )
		|| (again == 64 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 128);
	} else if ((again == 128 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation))
		|| (again == 32 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 64);
	} else if ((again == 64 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation))
		|| (again == 16 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 32);
	} else if ((again == 32 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation))
		|| (again == 8 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 16);
	} else if ((again == 16 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation))
		|| (again == 4 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 8);
	} else if ((again == 8 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation))
		|| (again == 2 && state->rgb_info.clear_raw < low_saturation)) {
		tcs3408r_set_als_gain(state, 4);
	} else if ((again == 4 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation))
		|| (again == 1 && state->rgb_info.clear_raw < low_saturation)){
		tcs3408r_set_als_gain(state, 2);
	} else if ((again == 2 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation))
		|| (again == 0 && state->rgb_info.clear_raw < low_saturation)){
		tcs3408r_set_als_gain(state, 1);
	} else if (again == 1 && (state->rgb_info.clear_raw > saturation || state->rgb_info.wide_raw > saturation)) {
		tcs3408r_set_als_gain(state, 0);
	} else {
		again_adjusted = false;
	}

	if (again_adjusted) {
		goto exit;
	}

#ifdef TCS3408_AUTO_DEBUG
	{
		log_data_info tcs3408_log_data;
		memset(&tcs3408_log_data, 0, sizeof(tcs3408_log_data));
		tcs3408_log_data.sensor_id = SENSOR_TYPE_RGB;
		tcs3408_log_data.string_id = SNS_LOGGER_AUTO_DEBUG1;
		tcs3408_log_data.argu2 = state->rgb_info.red_raw;
		tcs3408_log_data.argu3 = state->rgb_info.green_raw;
		tcs3408_log_data.argu4 = state->rgb_info.blue_raw;
		tcs3408_log_data.argu5 = state->rgb_info.clear_raw;
		tcs3408_log_data.argu6 = 0;
		sensors_log_report(tcs3408_log_data);
	}
	{
		log_data_info tcs3408_log_data;
		memset(&tcs3408_log_data, 0, sizeof(tcs3408_log_data));
		tcs3408_log_data.sensor_id = SENSOR_TYPE_RGB;
		tcs3408_log_data.string_id = SNS_LOGGER_AUTO_DEBUG2;
		tcs3408_log_data.argu2 = state->rgb_info.wide_raw;
		tcs3408_log_data.argu3 = state->rgb_info.again;
		tcs3408_log_data.argu4 = 0;
		tcs3408_log_data.argu5 = 0;
		tcs3408_log_data.argu6 = 0;
		sensors_log_report(tcs3408_log_data);
	}
#endif

	{
		state->rgb_sample.red   = (float)state->rgb_info.red_raw * (state->chanl_scale[0] / 1000.0);
		state->rgb_sample.green = (float)state->rgb_info.green_raw * (state->chanl_scale[1] / 1000.0);
		state->rgb_sample.blue  = (float)state->rgb_info.blue_raw * (state->chanl_scale[2] / 1000.0);
		state->rgb_sample.clear = (float)state->rgb_info.clear_raw * (state->chanl_scale[3] / 1000.0);
		state->rgb_sample.wide  = (float)state->rgb_info.wide_raw * (state->chanl_scale[4] / 1000.0);
	}

	state->rgb_sample.ir_ratio = (state->rgb_sample.red + state->rgb_sample.green + state->rgb_sample.blue - state->rgb_sample.clear) / (state->rgb_sample.clear * 2);
	state->rgb_sample.ir_ratio = state->rgb_sample.ir_ratio < 0 ? 0.0 : state->rgb_sample.ir_ratio;
	calc_ir = 1.0 - ((state->rgb_sample.clear * 10.0)/(27.0 * state->rgb_sample.wide));

	if (calc_ir < 0.0) {
		calc_ir = 0.0;
	}

	//normalize the raw data to 100ms itime and 128x again
	{
		float norm_scl;
		atime /= 1000.0;
		norm_scl = 100.0 * 128.0 / atime / (float)again;
		raw_norm_array[0] = state->rgb_sample.clear * norm_scl;
		raw_norm_array[1] = state->rgb_sample.red * norm_scl;
		raw_norm_array[2] = state->rgb_sample.green * norm_scl;
		raw_norm_array[3] = state->rgb_sample.blue * norm_scl;
		raw_norm_array[4] = state->rgb_sample.wide * norm_scl;
		state->rgb_sample.clear	= raw_norm_array[0];
		state->rgb_sample.red	= raw_norm_array[1];
		state->rgb_sample.green	= raw_norm_array[2];
		state->rgb_sample.blue	= raw_norm_array[3];
		state->rgb_sample.wide	= raw_norm_array[4];
	}

	SNS_INST_PRINTF(LOW, instance, "tcs3408_get_lux_cct: R:%d, G:%d, B:%d, C:%d, W:%d, ir:%d",
		(int)state->rgb_sample.red, (int)state->rgb_sample.green, (int)state->rgb_sample.blue,
		(int)state->rgb_sample.clear, (int)state->rgb_sample.wide, (int)(1000 * state->rgb_sample.ir_ratio));

	//calculate X/Y/Z	
	for(j = 0; j < CCT_COEF_ROW; j++){
		for(i = 0; i < CCT_COEF_COL; i++){
			if(calc_ir >= ir_devers){
				matrix[j] += cct_coef_Hir[j][i] * raw_norm_array[i];
			}else{
				matrix[j] += cct_coef_Lir[j][i] * raw_norm_array[i];
			}				
		}
	}	
	X = matrix[0];
	Y = matrix[1];
	Z = matrix[2];

	//calculate x/y and cct
	sum = X + Y + Z;
	state->rgb_sample.x = X/sum;
	state->rgb_sample.y = Y/sum;
	temp = (state->rgb_sample.x - 0.332)/(state->rgb_sample.y - 0.1858);


	state->rgb_sample.cct = -499*temp*temp*temp + 3525*temp*temp - 6823.3*temp + 5520.33;

	//calculate lux
	for(m = 0; m < RAW_NUM; m++){
		if(calc_ir >= ir_devers){
			lux += lux_coef_Hir[m] * raw_norm_array[m];
		}else{
			lux += lux_coef_Lir[m] * raw_norm_array[m];
		}
	}

	/* Reserve one digit after the decimal point */
	state->als_info.lux = (float)((uint32_t)((lux + 0.05) * 10)) / 10.0;
	SNS_INST_PRINTF(LOW, instance, "tcs3408_get_lux_cct: lux:%d, x:%d, y:%d, cct:%d",
		(int)state->als_info.lux, (int)(1000 * state->rgb_sample.x), (int)(1000 * state->rgb_sample.y), (int)state->rgb_sample.cct);

	state->chanl_data_ready = true;
	if (state->first_als) {
		state->first_als = false;
		tcs3408r_set_als_time(state, 100000);
	}
exit:
	return;
}

void tcs3408r_process_sensor_data(sns_sensor_instance *const instance)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;
	uint8_t status = 0, enable = 0, intenab = 0;

	tcs3408r_inst_i2c_read_byte(state, TCS3408_STATUS_REG, &status);
	if (0 != status) {
		tcs3408r_inst_i2c_write_byte(state, TCS3408_STATUS_REG, status);
	}

	tcs3408r_inst_i2c_read_byte(state, TCS3408_ENABLE_REG , &enable);
	SNS_INST_PRINTF(LOW, instance, "TCS3408 process_sensor_data enable=0x%x, status=0x%x\n", enable, status);

	enable = 0;
	state->chanl_data_ready = false;
	if (status&CINT) {
		if (state->publish_sensors&TCS3408_ALS || state->publish_sensors&TCS3408_RGB) {
			enable |= AEN;
		//	enable |= FDEN;
		}

		//added another option
		if (state->publish_sensors&TCS3408_FLICKER) {
		//	enable |= AEN;
			enable |= FDEN;
		}

		if (state->als_registry_cfg.is_dri && (state->publish_sensors&TCS3408_ALS)) {
			intenab |= AIEN;
		}

		if (enable != 0) {
			tcs3408r_modify_intenab(state, (AIEN|CIEN), intenab);
			tcs3408r_modify_enable(state, (AEN|FDEN), enable);
		} else {
			tcs3408r_modify_enable(state, 0xFF, 0);
		}
	}
	
	if (status&AINT)
	{
		tcs3408r_get_lux_cct(instance, state->als_registry_cfg.is_dri);
	}


	/* test flicker */

	if (state->publish_sensors&TCS3408_FLICKER) {
		state->flicker_info.data_ready = false;
		tcs3408r_get_fd_data(instance);
	}

	return;
}

void tcs3408r_send_config_event(sns_sensor_instance *const instance)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;

	sns_std_sensor_physical_config_event phy_sensor_config =
		sns_std_sensor_physical_config_event_init_default;

	// TODO: Use appropriate op_mode selected by driver.
	char operating_mode[] = "NORMAL";

	pb_buffer_arg op_mode_args;

	op_mode_args.buf = &operating_mode[0];
	op_mode_args.buf_len = sizeof(operating_mode);

	if (state->publish_sensors&TCS3408_ALS)
	{
		phy_sensor_config.has_sample_rate = true;
		phy_sensor_config.sample_rate = 10.0;
		phy_sensor_config.has_dri_enabled = true;
		if (state->als_registry_cfg.is_dri)
		{
			phy_sensor_config.dri_enabled = true;
		}
		else
		{
			phy_sensor_config.dri_enabled = false;
		}

		phy_sensor_config.has_water_mark = false;
		phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
		phy_sensor_config.operation_mode.arg = &op_mode_args;
		phy_sensor_config.has_active_current = true;
		phy_sensor_config.active_current = TCS3408_ALS_ACTIVE_CURRENT;
		phy_sensor_config.has_resolution = true;
		phy_sensor_config.resolution = TCS3408_ALS_RESOLUTION;
		phy_sensor_config.range_count = 2;
		phy_sensor_config.range[0] = TCS3408_ALS_RANGE_MIN;
		phy_sensor_config.range[1] = TCS3408_ALS_RANGE_MAX;
		phy_sensor_config.has_stream_is_synchronous = false;
		phy_sensor_config.has_DAE_watermark = false;

		pb_send_event(instance,
			sns_std_sensor_physical_config_event_fields,
			&phy_sensor_config,
			sns_get_system_time(),
			SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
			&state->als_info.als_suid);
	}

	if (state->publish_sensors&TCS3408_RGB)
	{
		phy_sensor_config.has_sample_rate = true;
		phy_sensor_config.sample_rate = 10.0;
		phy_sensor_config.has_dri_enabled = true;
		if (state->rgb_registry_cfg.is_dri)
		{
			phy_sensor_config.dri_enabled = true;
		}
		else
		{
			phy_sensor_config.dri_enabled = false;
		}

		phy_sensor_config.has_water_mark = false;
		phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
		phy_sensor_config.operation_mode.arg = &op_mode_args;
		phy_sensor_config.has_active_current = true;
		phy_sensor_config.active_current = TCS3408_RGB_ACTIVE_CURRENT;
		phy_sensor_config.has_resolution = true;
		phy_sensor_config.resolution = TCS3408_RGB_RESOLUTION;
		phy_sensor_config.range_count = 2;
		phy_sensor_config.range[0] = TCS3408_RGB_RANGE_MIN;
		phy_sensor_config.range[1] = TCS3408_RGB_RANGE_MAX;
		phy_sensor_config.has_stream_is_synchronous = false;
		phy_sensor_config.has_DAE_watermark = false;

		pb_send_event(instance,
			sns_std_sensor_physical_config_event_fields,
			&phy_sensor_config,
			sns_get_system_time(),
			SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
			&state->rgb_info.rgb_suid);
	}

	if (state->publish_sensors&TCS3408_FLICKER)
	{
		phy_sensor_config.has_sample_rate = true;
		phy_sensor_config.sample_rate = 4.0;
		phy_sensor_config.has_dri_enabled = true;
		if (state->flicker_registry_cfg.is_dri)
		{
			phy_sensor_config.dri_enabled = true;
		}
		else
		{
			phy_sensor_config.dri_enabled = false;
		}

		phy_sensor_config.has_water_mark = false;
		phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
		phy_sensor_config.operation_mode.arg = &op_mode_args;
		phy_sensor_config.has_active_current = true;
		phy_sensor_config.active_current = TCS3408_FLICKER_ACTIVE_CURRENT;
		phy_sensor_config.has_resolution = true;
		phy_sensor_config.resolution = TCS3408_FLICKER_RESOLUTION;
		phy_sensor_config.range_count = 2;
		phy_sensor_config.range[0] = TCS3408_FLICKER_RANGE_MIN;
		phy_sensor_config.range[1] = TCS3408_FLICKER_RANGE_MAX;
		phy_sensor_config.has_stream_is_synchronous = false;
		phy_sensor_config.has_DAE_watermark = false;

		pb_send_event(instance,
			sns_std_sensor_physical_config_event_fields,
			&phy_sensor_config,
			sns_get_system_time(),
			SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
			&state->flicker_info.flicker_suid);
	}
}

void tcs3408r_handle_sensor_als_sample(sns_sensor_instance *const instance, uint64_t timeout_ticks)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;

	float data[ALS_EVENT_SIZE];

	data[0] = state->als_info.lux / 1.0;

	state->als_info.last_event_timestamp = timeout_ticks;
	pb_send_sensor_stream_event(instance,
		&state->als_info.als_suid,
		(sns_time)timeout_ticks,
		SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
		SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
		data,
		ALS_EVENT_SIZE,
		state->encoded_als_event_len);
}


void tcs3408r_handle_sensor_rgb_sample(sns_sensor_instance *const instance, uint64_t timeout_ticks)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;

	float data[RGB_EVENT_SIZE];

	state->als_info.last_event_timestamp = timeout_ticks;
	data[0] = state->rgb_sample.cct;
	data[1] = state->rgb_sample.red;
	data[2] = state->rgb_sample.green;
	data[3] = state->rgb_sample.blue;
	data[4] = state->rgb_sample.clear;
	data[5] = state->rgb_sample.wide;
	data[6] = state->rgb_sample.ir_ratio;
	data[7] = state->rgb_sample.x;
	data[8] = state->rgb_sample.y;
	data[9] = state->als_info.lux;
	data[10] = state->rgb_sample.again / 1.0;
	data[11] = state->rgb_sample.atime;

	pb_send_sensor_stream_event(instance,
		&state->rgb_info.rgb_suid,
		(sns_time)timeout_ticks,
		SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
		SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
		data,
		RGB_EVENT_SIZE,
		state->encoded_rgb_event_len);
}

void tcs3408r_handle_sensor_flicker_sample(sns_sensor_instance *const instance, uint64_t timeout_ticks)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;

	float data[FLICKER_EVENT_SIZE];

	data[0] = state->flicker_info.freq1;
	data[1] = state->flicker_info.freq2;
	data[2] = state->flicker_info.mag1;
	data[3] = state->flicker_info.mag2;
	data[4] = state->flicker_info.ave;
	data[5] = state->flicker_info.max;
	data[6] = state->flicker_info.min;

	pb_send_sensor_stream_event(instance,
		&state->flicker_info.flicker_suid,
		(sns_time)timeout_ticks,
		SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
		SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
		data,
		FLICKER_EVENT_SIZE,
		state->encoded_flicker_event_len);
}

sns_rc tcs3408r_reset_config(tcs3408_instance_state *const state)
{
	sns_rc rv = SNS_RC_SUCCESS;
	uint8_t enable = 0, intenab = 0;

	if (state->publish_sensors&TCS3408_ALS || state->publish_sensors&TCS3408_RGB) {
		enable |= AEN;
	//	enable |= FDEN;
	}

	//added another option
	if (state->publish_sensors&TCS3408_FLICKER) {
	//	enable |= AEN;
		enable |= FDEN;
	}

	if (state->als_registry_cfg.is_dri && (state->publish_sensors&TCS3408_ALS)) {
		intenab |= AIEN;
	}

	if (enable != 0) {
		rv |= tcs3408r_modify_intenab(state, (AIEN|CIEN), intenab);
		rv |= tcs3408r_modify_enable(state, (AEN|FDEN), enable);
	} else {
		rv |= tcs3408r_modify_enable(state, 0xFF, 0);
	}

	tcs3408r_set_als_gain(state, 64);
	//tcs3408r_modify_control(state, ALS_MANUAL_AZ, ALS_MANUAL_AZ);// manual auto zero
	//tcs3408r_set_fd_gain(state, 8);//set init gain to 8x whenever open fd
	//tcs3408r_set_fd_time_us(state, 975);

	return rv;
}
void tcs3408r_register_fac_cali_timer(sns_sensor_instance *this, uint16_t period)
{
    tcs3408_instance_state *state = (tcs3408_instance_state*)this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    uint8_t buffer[50];
    sns_request timer_req =
    {
        .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
        .request    = buffer
    };
    sns_rc rc = SNS_RC_SUCCESS;

    if (NULL == state->fac_cali_timer_data_stream)
    {
        sns_service_manager *smgr     = this->cb->get_service_manager(this);
        sns_stream_service  *strm_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
        rc = strm_svc->api->create_sensor_instance_stream(strm_svc, this, state->timer_suid, &state->fac_cali_timer_data_stream);
    }

    if ( (rc != SNS_RC_SUCCESS) || (NULL == state->fac_cali_timer_data_stream) )
    {
        SNS_INST_PRINTF(ERROR, this, "error creating stream %d", rc);
        return;
    }

    req_payload.is_periodic    = true;
    req_payload.start_time     = sns_get_system_time();
    req_payload.timeout_period = sns_convert_ns_to_ticks(period * 1000* 1000);
    timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);

    if (timer_req.request_len > 0)
    {
        state->fac_cali_timer_data_stream->api->send_request(state->fac_cali_timer_data_stream, &timer_req);
    }
}

sns_rc tcs3408r_read_channel(tcs3408_instance_state *const state, uint16_t* rawdata)
{
	sns_rc rv = SNS_RC_SUCCESS;
	uint8_t reg_val[10] = {0};

	rv |= tcs3408r_inst_i2c_read_block(state, TCS3408_ADATA0L_REG, &reg_val[0], 10);
	*(rawdata + 3)	= (uint16_t)((reg_val[1] << 8) | reg_val[0]);
	*(rawdata)		= (uint16_t)((reg_val[3] << 8) | reg_val[2]);
	*(rawdata + 1) 	= (uint16_t)((reg_val[5] << 8) | reg_val[4]);
	*(rawdata + 2)  = (uint16_t)((reg_val[7] << 8) | reg_val[6]);
	*(rawdata + 4)  = (uint16_t)((reg_val[9] << 8) | reg_val[8]);

	return rv;
}

sns_rc tcs3408r_dump_reg(sns_sensor_instance *const instance)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;
	sns_rc rv = SNS_RC_SUCCESS;
	uint8_t reg = 0;
	uint8_t val[3] = {0};


	for (reg=0x80;;reg+=3)
	{
		tcs3408r_inst_i2c_read_block(state, reg , val, 3);

		SNS_INST_PRINTF(LOW, instance, "tcs3408_dump_reg, 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x",
			reg, val[0], reg+1, val[1], reg+2, val[2]);

		if ((reg+3) > 0xFD)
			break;
	}

	return rv;
}


sns_rc tcs3408r_cali_read_fd_data(tcs3408_instance_state *state, uint32_t* fd_sum, uint16_t* data_cnt)
{
	sns_rc rv = SNS_RC_SUCCESS;
	uint8_t fifo_lvl = 0;
	uint8_t reg_val[32] = {0};
	uint16_t fd_raw = 0;

	rv |= tcs3408r_inst_i2c_read_byte(state, TCS3408_FIFO_STATUS, &fifo_lvl);
	rv |= tcs3408r_inst_i2c_read_block(state, TCS3408_FDATAL, &reg_val[0], fifo_lvl*2);

	if (SNS_RC_SUCCESS != rv)
		return rv;

	for(int k=0;k<fifo_lvl;k++) {
		fd_raw = (uint16_t)(reg_val[k * 2]) | ((uint16_t)(reg_val[k * 2 + 1]) << 8);
		if (fd_raw < 10 || 65535  == fd_raw)
			continue;

		if (*data_cnt >= 350)//colletct  300 data
			continue;

		(*data_cnt)++;
		if (*data_cnt < 51)//skip 50 datas
			continue;
		*fd_sum += fd_raw;
	}

	return rv;
}

sns_rc tcs3408r_channel_cali(sns_sensor_instance *const instance)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)instance->state->state;
	sns_rc rv = SNS_RC_SUCCESS;

	static uint8_t skip_count;
	static uint16_t fd_cnt;
	static uint32_t fd_sum;
	uint8_t status = 0;
	uint16_t channel_raw[5] = {0};
	bool test_result = false;

	SNS_INST_PRINTF(HIGH, instance, "tcs3408r_channel_cali: status %d, skip_count %d", state->fac_cali_st, skip_count);
	switch(state->fac_cali_st)
	{
		case TCS3408_CALI_IDLE:
			/*set config*/
			rv |= tcs3408r_modify_enable(state, (AEN|FDEN|PON), PON);
			rv |= tcs3408r_set_als_gain(state, 32);
			rv |= tcs3408r_set_als_time(state, 100000);
			rv |= tcs3408r_modify_control(state, ALS_MANUAL_AZ, ALS_MANUAL_AZ);// manual auto zero
			rv |= tcs3408r_set_fd_gain(state, 16);//set init gain to 8x whenever open fd
			rv |= tcs3408r_set_fd_time_us(state, 975);
			rv |= tcs3408r_modify_enable(state, (AEN|FDEN|PON), AEN|FDEN|PON);
			rv |= tcs3408r_inst_i2c_write_byte(state, TCS3408_STATUS_REG, 0xff);//clear irq

			/*reset data*/
			skip_count = 0;
			state->cali_data_num = 0;
			for(int i=0;i<6;i++)
				state->cali_chanl_ave[i] = 0;
			fd_cnt = 0;
			fd_sum = 0;

			/*start cali*/
			state->fac_cali_st = TCS3408_CALI_RUNNING;
			tcs3408r_register_fac_cali_timer(instance, 10);

			break;

		case TCS3408_CALI_RUNNING:
			////collect fd data
			tcs3408r_cali_read_fd_data(state, &fd_sum, &fd_cnt);
			SNS_INST_PRINTF(LOW, instance, "tcs3408r_channel_cali: collect fd data, sum %d, cnt+50 %d", fd_sum, fd_cnt);

			//check irq & collect channel data
			rv |= tcs3408r_inst_i2c_read_byte(state, TCS3408_STATUS_REG, &status);
			if (rv != SNS_RC_SUCCESS) {
				SNS_INST_PRINTF(ERROR, instance, "tcs3408r_channel_cali: error read irq status %d", rv);
				return rv;
			}
			if (0 != status) {
				tcs3408r_inst_i2c_write_byte(state, TCS3408_STATUS_REG, status);
			}

			if (status&AINT) {
				skip_count = 0;
				rv |= tcs3408r_read_channel(state, channel_raw);
				if (rv != SNS_RC_SUCCESS) {
					SNS_INST_PRINTF(ERROR, instance, "tcs3408r_channel_cali: error read channel data %d", rv);
					return rv;
				}

				SNS_INST_PRINTF(LOW, instance, "tcs3408r_channel_cali: collect data, R %d, G %d, B %d, C %d, W %d", 
					channel_raw[0], channel_raw[1], channel_raw[2], channel_raw[3], channel_raw[4]);

				for (int i=0;i<5;i++) {
					if (channel_raw[i] < 10 || 65535 == channel_raw[i]) {
						SNS_INST_PRINTF(ERROR, instance, "tcs3408r_channel_cali: abnormal channel data, skip");
						return SNS_RC_FAILED;
					}
				}
				for(int i=0;i<5;i++)
					state->cali_chanl_ave[i] += channel_raw[i];
				state->cali_data_num++;
			} else {
				skip_count++;

				if (skip_count > 40) {//if timer 10ms * 40 times cant read irq while Atime is 100ms,  unknown error occours
					SNS_INST_PRINTF(ERROR, instance, "tcs3408r_channel_cali: missing irq in 200ms, exit cali");
					state->fac_cali_st = TCS3408_CALI_IDLE;
					tcs3408r_reset_config(state);
					state->island_service->api->sensor_instance_island_exit(state->island_service, instance);
					tcs3408r_send_self_test_event(instance,
	                     &state->rgb_info.rgb_suid,
	                     test_result,
	                     state->rgb_info.test_info.test_type);
					return SNS_RC_FAILED;
				}
			}

			if (5 == state->cali_data_num) {//collect data complete

				if (fd_cnt < 350) {//check fd data num first
					SNS_INST_PRINTF(ERROR, instance, "tcs3408r_channel_cali: ERROR too few flicker data!");
					state->fac_cali_st = TCS3408_CALI_IDLE;
					tcs3408r_reset_config(state);
					state->island_service->api->sensor_instance_island_exit(state->island_service, instance);
					tcs3408r_send_self_test_event(instance,
	                     &state->rgb_info.rgb_suid,
	                     test_result,
	                     state->rgb_info.test_info.test_type);
					return SNS_RC_FAILED;
				}


				for(int i=0;i<5;i++)
					state->cali_chanl_ave[i] /= 5;
				state->cali_chanl_ave[5] = (uint32_t)((float)fd_sum * 32.0 / (float)(fd_cnt - 50));
				state->fac_cali_st = TCS3408_CALI_DONE;
			}

			break;

		case TCS3408_CALI_DONE:
			state->fac_cali_st = TCS3408_CALI_IDLE;
			state->update_fac_cal_in_registry = true;
			test_result = true;

			if (SNS_PHYSICAL_SENSOR_TEST_TYPE_SW == state->rgb_info.test_info.test_type) {//3000K source
				for (int i=0;i<6;i++) {
					state->chanl_scale[i] = 1000 * gold_scale_r[0][i] / state->cali_chanl_ave[i];
				}
				state->double_cali = true;
			} else if (SNS_PHYSICAL_SENSOR_TEST_TYPE_HW == state->rgb_info.test_info.test_type) {//6000K source
				if (state->double_cali) {
					SNS_INST_PRINTF(MED, instance, "tcs3408r_channel_cali: 6000K source, run DOUBLE cali");
					SNS_INST_PRINTF(MED, instance, "tcs3408r_channel_cali: 3000K scale, R_sca %d, G_sca %d, B_sca %d, C_sca %d, W_sca %d. F_cali %d",
						(int)(state->chanl_scale[0]), (int)(state->chanl_scale[1]), (int)(state->chanl_scale[2]),
						(int)(state->chanl_scale[3]), (int)(state->chanl_scale[4]), (int)(state->chanl_scale[5]));

					for (int i=0;i<6;i++) {
						state->chanl_scale[i] = 1000 * (gold_scale_r[0][i] + gold_scale_r[1][i]) / (gold_scale_r[0][i]  * 1000 / state->chanl_scale[i] + state->cali_chanl_ave[i]);
					}
				} else {
					SNS_INST_PRINTF(MED, instance, "tcs3408r_channel_cali: 6000K source, run single cali");
					for (int i=0;i<6;i++) {
						state->chanl_scale[i] = 1000 * gold_scale_r[1][i] / state->cali_chanl_ave[i];
					}
				}
				state->double_cali = false;
			}

			for (int i=0;i<6;i++) {
				state->chanl_cali_ver[i]++;
				if (i < 2)
					state->rgb_fac_cal_version[i]++;
			}
			SNS_INST_PRINTF(HIGH, instance, "tcs3408r_channel_cali: cali finish, R_raw %d, G_raw %d, B_raw %d, C_raw %d, W_raw %d, F_raw %d",
				state->cali_chanl_ave[0], state->cali_chanl_ave[1], state->cali_chanl_ave[2],
				state->cali_chanl_ave[3], state->cali_chanl_ave[4], state->cali_chanl_ave[5]);
			SNS_INST_PRINTF(HIGH, instance, "tcs3408r_channel_cali: cali finish, R_sca %d, G_sca %d, B_sca %d, C_sca %d, W_sca %d, F_sca %d",
				(int)(state->chanl_scale[0]), (int)(state->chanl_scale[1]), (int)(state->chanl_scale[2]),
				(int)(state->chanl_scale[3]), (int)(state->chanl_scale[4]), (int)(state->chanl_scale[5]));

			tcs3408r_reset_config(state);
			state->island_service->api->sensor_instance_island_exit(state->island_service, instance);
			tcs3408r_send_self_test_event(instance,
	                     &state->rgb_info.rgb_suid,
	                     test_result,
	                     state->rgb_info.test_info.test_type);
			state->rgb_info.test_info.test_client_present = false;

			break;
	}

	return rv;
}
