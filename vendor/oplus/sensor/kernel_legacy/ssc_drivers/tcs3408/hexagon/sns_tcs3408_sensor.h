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
#include "sns_data_stream.h"
#include "sns_island_service.h"
#include "sns_pwr_rail_service.h"
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_diag_service.h"
#include "sns_math_util.h"
#include "sns_registry_util.h"

#include "sns_tcs3408_sensor_instance.h"
#include "sns_tcs3408_hal.h"

#define TCS3408_DRIVER_VERSION         0x0100

#define TCS3408_ALS_SUID \
	{  \
		.sensor_uid =  \
		{  \
			0x61, 0x6D, 0x73, 0x54, 0x43, 0x53, 0x33, 0x37,  \
			0x30, 0x31, 0x41, 0x4C, 0x53, 0x5F, 0x5F, 0x5F  \
		}  \
	}

#define TCS3408_RGB_SUID \
	{  \
		.sensor_uid =  \
		{  \
			0x61, 0x6D, 0x73, 0x54, 0x43, 0x53, 0x33, 0x37,  \
			0x30, 0x31, 0x52, 0x47, 0x42, 0x58, 0x5F, 0x5F  \
		}  \
	}

#define TCS3408_FLICKER_SUID \
	{  \
		.sensor_uid =  \
		{  \
			0x61, 0x6D, 0x73, 0x54, 0x43, 0x53, 0x33, 0x37,  \
			0x30, 0x31, 0x52, 0x51, 0x51, 0x82, 0x5F, 0x5F  \
		}  \
	}



#define ALS_FAC_CAL_NUM                2
#define ALS_EVENT_SIZE                 1
#define PROX_EVENT_SIZE                2
#define RGB_EVENT_SIZE                 12
#define FLICKER_EVENT_SIZE             7


/* TCS3408 ODR (Hz) definitions */
#define TCS3408_ODR_5                  5.0
#define TCS3408_ODR_10                 10.0
#define TCS3408_ODR_20                 20.0
#define TCS3408_ODR_100                100.0
#define TCS3408_ODR_1000               1000.0
#define TCS3408_ODR_2000               2000.0

/* TCS3408 ALS ranges */
#define TCS3408_ALS_RANGE_MIN          0.0
#define TCS3408_ALS_RANGE_MAX          65535.0

/* TCS3408 ALS resolutions */
#define TCS3408_ALS_RESOLUTION         0.1

/* TCS3408 ALS power */
#define TCS3408_ALS_ACTIVE_CURRENT     90
#define TCS3408_ALS_SLEEP_CURRENT      1

/* TCS3408 PROX ranges */
#define TCS3408_PROX_RANGE_MIN         0.0
#define TCS3408_PROX_RANGE_MAX         5.0

/* TCS3408 PROX resolutions */
#define TCS3408_PROX_RESOLUTION        1.0

/* TCS3408 PROX power */
#define TCS3408_PROX_ACTIVE_CURRENT    1000
#define TCS3408_PROX_SLEEP_CURRENT     1

/* TMD3702 RGB and CT_C ranges */
#define TCS3408_RGB_RANGE_MIN          0.0
#define TCS3408_RGB_RANGE_MAX          65535.0

/* TMD3702 RGB and CT_C resolutions */
#define TCS3408_RGB_RESOLUTION         0.1

/* TMD3702 RGB and CT_C power */
#define TCS3408_RGB_ACTIVE_CURRENT     90
#define TCS3408_RGB_SLEEP_CURRENT      1

/* TMD3702 RGB and CT_C ranges */
#define TCS3408_FLICKER_RANGE_MIN          0.0
#define TCS3408_FLICKER_RANGE_MAX          65535.0

/* TMD3702 RGB and CT_C resolutions */
#define TCS3408_FLICKER_RESOLUTION         0.1

/* TMD3702 RGB and CT_C power */
#define TCS3408_FLICKER_ACTIVE_CURRENT     90
#define TCS3408_FLICKER_SLEEP_CURRENT      1


/* Supported opertating modes */
#define TCS3408_LPM                    "LPM"
#define TCS3408_NORMAL                 "NORMAL"

/* Forward Declaration of Ambient Light Sensor API */
sns_sensor_api tcs3408_als_sensor_api;

/* Forward Declaration of RGB */
sns_sensor_api tcs3408_rgb_sensor_api;


/* Forward Declaration of flicker */
sns_sensor_api tcs3408_flicker_sensor_api;

/* Power rail timeout States for the TCS3408 Sensors.*/
typedef enum
{
	TCS3408_POWER_RAIL_PENDING_NONE,
	TCS3408_POWER_RAIL_PENDING_INIT,
	TCS3408_POWER_RAIL_PENDING_SET_CLIENT_REQ,
} tcs3408_power_rail_pending_state;

typedef struct tcs3408_pb_custom_float_parse_arg
{
	float    *data_array;
	uint32_t *version_array;
	uint8_t  *index;
	uint8_t  size;
} tcs3408_pb_custom_float_parse_arg;

/* Interrupt Sensor State. */
typedef struct tcs3408_common_state
{
	sns_sensor_uid          reg_suid;
	sns_sensor_uid          irq_suid;
	sns_sensor_uid          timer_suid;
	sns_sensor_uid          rgb_rear_suid;

	tcs3408_com_port_info   com_port_info;
	sns_interrupt_req       irq_config;

	sns_rail_config         rail_config;
	sns_power_rail_state    registry_rail_on_state;

	bool                    start_hw_detect;
	bool                    hw_is_present;
	uint16_t                who_am_i;

	/* Registry sensor config */
	bool                    als_registry_cfg_received;
	bool                    rgb_registry_cfg_received;
	bool                    flicker_registry_cfg_received;
	sns_registry_phy_sensor_cfg als_registry_cfg;

	sns_registry_phy_sensor_cfg rgb_registry_cfg;

	sns_registry_phy_sensor_cfg flicker_registry_cfg;

	/* Registry sensor platform config */
	bool                    registry_pf_cfg_received;
	sns_registry_phy_sensor_pf_cfg registry_pf_cfg;

	/* factory calibration */
	float                   fac_cal_lux_scale;
	float                   fac_cal_lux_bias;
	uint32_t                als_fac_cal_version[2];

	bool					double_cali;
	float                   fac_cal_rgb_scale;
	float                   fac_cal_rgb_bias;
	float					channel_scale[6];
	uint32_t                rgb_fac_cal_version[2];
	uint32_t                channel_cali_version[6];

	float                   fac_cal_flicker_scale;
	float                   fac_cal_flicker_bias;
	uint32_t                flicker_fac_cal_version[2];


	/* Coefficients */
	bool                    registry_coefficient_received;
	float                   coefficient[12];
} tcs3408_common_state;

typedef struct tcs3408_state
{
	tcs3408_common_state    common;

	/* Sensor Type */
	tcs3408_sensor_type     sensor;

	sns_diag_service        *diag_service;
	sns_sync_com_port_service *scp_service;
	sns_pwr_rail_service    *pwr_rail_service;
	sns_island_service      *island_service;

	sns_data_stream         *fw_stream;
	sns_data_stream         *reg_data_stream;
	sns_data_stream         *timer_stream;

	sns_sensor_uid          my_suid;

	tcs3408_power_rail_pending_state    power_rail_pend_state;

	/* factory calibration */
	bool                    registry_fac_cal_received;

	uint32_t                als_encoded_event_len;

	uint32_t                rgb_encoded_event_len;

	uint32_t                flicker_encoded_event_len;
	bool                    is_new_request;
} tcs3408_state;

/* Functions shared by all TCS3408 Sensors */
/**
 * Sends a request to the SUID Sensor to get SUID of a dependent
 * Sensor.
 *
 * @param[i] this          Sensor reference
 * @param[i] data_type     data_type of dependent Sensor
 * @param[i] data_type_len Length of the data_type string
 */
void tcs3408_send_suid_req(sns_sensor *this, char *const data_type,
									uint32_t data_type_len);

/**
 * Call this function to do common initialization for TCS3408
 * Sensor.
 *
 * @param[i] this          Sensor reference
 */
void tcs3408_common_init(sns_sensor *const this);

/**
 * This function parses the client_request list per Sensor and
 * determines final config for the Sensor Instance.
 *
 * @param[i] this          Sensor reference
 * @param[i] instance      Sensor Instance to config
 *
 * @return none
 */
void tcs3408_reval_instance_config(sns_sensor *this,
									sns_sensor_instance *instance,
									tcs3408_sensor_type sensor_type);

/**
 * Processes events from SUID Sensor.
 *
 * @param[i] this   Sensor reference
 *
 * @return none
 */
void tcs3408_process_suid_events(sns_sensor *const this);

/**
 * Request registry.
 *
 * @param[i] this   Sensor reference
 *
 * @return none
 */
void tcs3408_request_registry(sns_sensor *const this);

/**
 * Processes registry events.
 *
 * @param[i] this   Sensor reference
 * @param[i] event  Sensor event
 *
 * @return none
 */
void tcs3408_sensor_process_registry_event(sns_sensor *const this,
									sns_sensor_event *event);

bool tcs3408_discover_hw(sns_sensor *const this);

void tcs3408_publish_available(sns_sensor *const this);

void tcs3408_update_sibling_sensors(sns_sensor *const this);

void tcs3408_start_power_rail_timer(sns_sensor *const this,
									sns_time timeout_ticks,
									tcs3408_power_rail_pending_state pwr_rail_pend_state);

void tcs3408_start_hw_detect_sequence(sns_sensor *const this);

/**
 * notify_event() Sensor API common between all TCS3408 Sensors.
 *
 * @param this    Sensor reference
 *
 * @return sns_rc
 */
sns_rc tcs3408_sensor_notify_event(sns_sensor *const this);

/**
 * set_client_request() Sensor API common between all TCS3408
 * Sensors.
 *
 * @param this            Sensor reference
 * @param exist_request   existing request
 * @param new_request     new request
 * @param remove          true to remove request
 *
 * @return sns_sensor_instance*
 */
sns_sensor_instance* tcs3408_set_client_request(sns_sensor *const this,
									struct sns_request const *exist_request,
									struct sns_request const *new_request,
									bool remove);
/* ALS init & deinit */
sns_rc tcs3408_als_init(sns_sensor *const this);
sns_rc tcs3408_als_deinit(sns_sensor *const this);


/* RGB init & deinit */
sns_rc tcs3408_rgb_init(sns_sensor *const this);
sns_rc tcs3408_rgb_deinit(sns_sensor *const this);

/* flicker init & deinit */
sns_rc tcs3408_flicker_init(sns_sensor *const this);
sns_rc tcs3408_flicker_deinit(sns_sensor *const this);

/* Write back to registry */
bool tcs3408_write_calibration_to_registry(sns_sensor *const this);
