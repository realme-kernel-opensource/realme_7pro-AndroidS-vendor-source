#pragma once
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

#include <stdint.h>
#include "sns_com_port_types.h"
#include "sns_data_stream.h"
#include "sns_sensor_instance.h"
#include "sns_time.h"

#include "sns_sensor_uid.h"

#include "sns_async_com_port.pb.h"
#include "sns_diag_service.h"
#include "sns_interrupt.pb.h"
#include "sns_island_service.h"
#include "sns_motion_detect.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_ambient_light.pb.h"
//#include "sns_proximity.pb.h"
#include "sns_sync_com_port_service.h"

#include "sns_math_util.h"
#include "sns_registry_util.h"

/** Forward Declaration of Instance API */
sns_sensor_instance_api tcs3408_sensor_instance_api;

/** Number of registers to read for debug */
#define TCS3408_DEBUG_REGISTERS          (32)

/** Number of entries in reg_map table. */
#define TCS3408_REG_MAP_TABLE_SIZE       (11)

typedef struct tcs3408_com_port_info
{
	sns_com_port_config      com_config;
	sns_sync_com_port_handle *port_handle;
} tcs3408_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr {
	float min;
	float max;
} range_attr;

typedef enum {
	TCS3408_ALS           = 0x01,
	TCS3408_RGB 		  = 0x02,
	TCS3408_FLICKER 	  = 0x04,
} tcs3408_sensor_type;

typedef enum {
	TCS3408_CONFIG_IDLE,            /** not configuring */
	TCS3408_CONFIG_POWERING_DOWN,   /** cleaning up when no clients left */
	TCS3408_CONFIG_STOPPING_STREAM, /** stream stop initiated, waiting for completion */
	TCS3408_CONFIG_FLUSHING_HW,     /** FIFO flush initiated, waiting for completion */
	TCS3408_CONFIG_UPDATING_HW      /** updating sensor HW, when done goes back to IDLE */
} tcs3408_config_step;

typedef struct tcs3408_self_test_info
{
	sns_physical_sensor_test_type test_type;
	bool test_client_present;
} tcs3408_self_test_info;

typedef enum
{
    TCS3408_CALI_IDLE,
    TCS3408_CALI_RUNNING,
    TCS3408_CALI_DONE
} tcs3408_calibration_status;

typedef struct tcs3408_rgb_sample
{
	float			clear;
	float			red;
	float			green;
	float			blue;
	float			wide;
	float			x;
	float			y;
	float			ir_ratio;
	float			atime;
	uint16_t		again;
	float			cct;
} tcs3408_rgb_sample;

typedef struct tcs3408_als_info
{
	sns_sensor_uid	als_suid;
	float			target_lux;
	float			lux;
	uint64_t		last_event_timestamp;
	tcs3408_self_test_info 	test_info;
} tcs3408_als_info;

typedef struct tcs3408_2nd_als_info
{
	float			lux;
	sns_time 		timestamp;
} tcs3408_2nd_als;

typedef struct tcs3408_rgb_info
{
	sns_sensor_uid          rgb_suid;
	uint16_t                clear_raw;
	uint16_t                red_raw;
	uint16_t                green_raw;
	uint16_t                blue_raw;
	uint16_t                wide_raw;
	int32_t                 clear_irradiance;
	int32_t                 red_irradiance;
	int32_t                 green_irradiance;
	int32_t                 blue_irradiance;
	float                   target_cct;
	tcs3408_self_test_info  test_info;
} tcs3408_rgb_info;

typedef struct tcs3408_flicker_info
{
	sns_sensor_uid          flicker_suid;
	uint16_t				samples;
	tcs3408_self_test_info  test_info;
	bool					data_ready;

	int				freq1;
	int				freq2;
	int				mag1;
	int				mag2;
	int				ave;
	int				max;
	int				min;
} tcs3408_flicker_info;

typedef struct tcs3408_irq_info
{
	sns_interrupt_req       irq_config;
	bool                    irq_ready;
	bool                    irq_registered;
} tcs3408_irq_info;

/*avoid ps ir light effecting als channle value*/
typedef struct tcs3408_channel_bufffer
{
	uint8_t i;
	uint16_t data[2][4];
} tcs3408_chanl_buf;

/** Private state. */
typedef struct tcs3408_instance_state
{
	/* Sensor to be published */
	uint8_t                 publish_sensors;
	uint8_t                 last_publish_sensors;
	tcs3408_sensor_type     new_request_sensor;

	/* rgb sample reported */
	tcs3408_rgb_sample		rgb_sample;

	/* Ambient light HW config details*/
	tcs3408_als_info        als_info;

	/* RGB HW config details*/
	tcs3408_rgb_info        rgb_info;
	
	/* Flicker HW config details*/
	tcs3408_flicker_info    flicker_info;

	/* Interrupt dependency info. */
	tcs3408_irq_info        irq_info;

	/* COM port info */
	tcs3408_com_port_info   com_port_info;

	/*--------DAE interface---------*/
	//tcs3408_dae_if_info   dae_if;

	tcs3408_config_step     config_step;

	sns_time                interrupt_timestamp;

	/* Data streams from dependentcies. */
	sns_sensor_uid          irq_suid;
	sns_sensor_uid          timer_suid;
	sns_sensor_uid          rgb_rear_suid;
	sns_data_stream         *interrupt_data_stream;
	sns_data_stream         *timer_data_stream;
	sns_data_stream         *rgb_rear_stream;

	/**----------Sensor specific registry configuration----------*/
	sns_registry_phy_sensor_cfg als_registry_cfg;

	sns_registry_phy_sensor_cfg rgb_registry_cfg;

	sns_registry_phy_sensor_cfg flicker_registry_cfg;

	uint8_t                 reg_status[TCS3408_DEBUG_REGISTERS];

	sns_diag_service        *diag_service;
	sns_sync_com_port_service *scp_service;
	sns_island_service      *island_service;

	bool                    instance_is_ready_to_configure;
	bool                    timer_is_active;
	bool                    is_dri;
	bool                    first_als;

	bool					first_fd;
	bool                    chanl_data_ready;
	//bool                  fifo_flush_in_progress;
	bool                    new_self_test_request;
	bool                    update_fac_cal_in_registry;

	float					lux_scale;
	float                   lux_bias;
	uint32_t                als_fac_cal_version[2];

	float					rgb_scale;
	float                   rgb_bias;
	float					chanl_scale[6];//order: R, G, B, C, W, F
	uint32_t                rgb_fac_cal_version[2];
	uint32_t                chanl_cali_ver[6];

	float					flicker_scale;
	float                   flicker_bias;
	uint32_t                flicker_fac_cal_version[2];

	size_t                  log_interrupt_encoded_size;
	size_t                  log_raw_encoded_size;
	size_t                  encoded_als_event_len;

	size_t                  encoded_rgb_event_len;

	size_t                  encoded_flicker_event_len;

	/*use for fac cali*/
	bool						double_cali;
	uint8_t						time_cnt;
	uint32_t					cali_chanl_ave[6];
	uint8_t						cali_data_num;
	tcs3408_calibration_status	fac_cali_st;
	sns_data_stream				*fac_cali_timer_data_stream;

	// add for double light
	tcs3408_2nd_als				last_second_als;
	tcs3408_2nd_als				second_als;
	uint8_t						sizeof_2nd_als;
	uint8_t						sizeof_flashlight_block_als;
} tcs3408_instance_state;

typedef struct tcs3408_odr_reg_map
{
	float                  odr;
	uint8_t                discard_samples;
} tcs3408_odr_reg_map;

typedef struct sns_tcs3408_req
{
	float	desired_sample_rate;
	float	desired_report_rate;
	bool	config_is_new_request;
	tcs3408_sensor_type sensor_type;
} sns_tcs3408_req;

/**
 * Executes requested self-tests.
 *
 * @param instance   reference to the instace
 *
 * @return none
 */
void tcs3408_run_self_test(sns_sensor_instance *instance);

/**
 * Sends a self-test completion event.
 *
 * @param[i] instance  Instance reference
 * @param[i] uid       Sensor UID
 *
 * @return none
 */
void tcs3408_send_self_test_event(sns_sensor_instance *instance,
									sns_sensor_uid *uid, bool test_result,
									sns_physical_sensor_test_type type);

sns_rc tcs3408_inst_init(sns_sensor_instance *const this, sns_sensor_state const *sstate);

sns_rc tcs3408_inst_deinit(sns_sensor_instance *const this);
