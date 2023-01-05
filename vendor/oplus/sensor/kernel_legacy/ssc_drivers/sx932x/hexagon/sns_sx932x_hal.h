#pragma once
/*******************************************************************************
* Copyright (c) 2017, Semtech
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     1. Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*     2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     3. Neither the name of Semtech nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_diag.pb.h"
#include "sns_diag_service.h"

#include "sns_sx932x_sensor.h"
#include "sns_sx932x_sensor_instance.h"

/* Constants */
#define SX_NULL              (0)

/*******ERR Code**************************************/
#define  SUCCESS      		((uint8_t)0)
#define E_NULL_PTR         	((int8_t)-127)
#define E_COMM_RES         	((int8_t)-1)
#define E_OUT_OF_RANGE     	((int8_t)-2)

/* Sensor Specific constants */
#define SLEEP_MODE          (0x00)
#define NORMAL_MODE         (0x03)

#define ULTRA_LOW_POWER_MODE          (0x00)
#define ULTRA_HIGH_RESOLUTION_MODE    (0x04)

#define SX931x_WHOAMI_REG			   0x42
#define SX931x_WHOAMI_VALUE			   0x01  /** Who Am I default value */
#define SX932x_WHOAMI_REG			   0xFA
#define SX932x_WHOAMI_VALUE			   0x23  /** Who Am I default value */
#define SX932x_WHOAMI_VALUE2		       0x22  /** Who Am I default value */


/** Off to idle time */
#define SX93xx_OFF_TO_IDLE_MS      	   100  //ms
#define SX932x_NUM_AXES         		4

/*******Interrupt and status*****************/
#define SX932x_IRQSTAT_REG    		    0x00
#define SX932x_STAT0_REG    			0x01
#define SX932x_STAT1_REG    			0x02
#define SX932x_STAT2_REG    			0x03
#define SX932x_STAT3_REG    			0x04
#define SX932x_IRQ_ENABLE_REG			0x05
#define SX932x_IRQCFG0_REG    		    0x06
#define SX932x_IRQCFG1_REG    	    	0x07
/**********General control*****************/
#define SX932x_CTRL0_REG    			0x10
#define SX932x_CTRL1_REG    			0x11
#define SX932x_I2CADDR_REG    		    0x14
/**********AFE Control*******************/
#define SX932x_AFE_CTRL0_REG   	    	0x20
#define SX932x_AFE_CTRL1_REG        	0x21
#define SX932x_AFE_CTRL2_REG          	0x22
#define SX932x_AFE_CTRL3_REG        	0x23
#define SX932x_AFE_CTRL4_REG			0x24
#define SX932x_AFE_CTRL5_REG			0x25
#define SX932x_AFE_CTRL6_REG			0x26
#define SX932x_AFE_CTRL7_REG			0x27
#define SX932x_AFE_PH0_REG				0x28
#define SX932x_AFE_PH1_REG				0x29
#define SX932x_AFE_PH2_REG				0x2A
#define SX932x_AFE_PH3_REG				0x2B
/****Main Digital Processing (Prox) control*****/
#define SX932x_PROX_CTRL0_REG			0x30
#define SX932x_PROX_CTRL1_REG			0x31
#define SX932x_PROX_CTRL2_REG			0x32
#define SX932x_PROX_CTRL3_REG			0x33
#define SX932x_PROX_CTRL4_REG			0x34
#define SX932x_PROX_CTRL5_REG			0x35
#define SX932x_PROX_CTRL6_REG			0x36
#define SX932x_PROX_CTRL7_REG			0x37
/****Advanced Digital Processing control******/
#define SX932x_ADV_CTRL0_REG			0x40
#define SX932x_ADV_CTRL1_REG			0x41
#define SX932x_ADV_CTRL2_REG			0x42
#define SX932x_ADV_CTRL3_REG			0x43
#define SX932x_ADV_CTRL4_REG			0x44
#define SX932x_ADV_CTRL5_REG			0x45
#define SX932x_ADV_CTRL6_REG			0x46
#define SX932x_ADV_CTRL7_REG			0x47
#define SX932x_ADV_CTRL8_REG			0x48
#define SX932x_ADV_CTRL9_REG			0x49
#define SX932x_ADV_CTRL10_REG			0x4A
#define SX932x_ADV_CTRL11_REG			0x4B
#define SX932x_ADV_CTRL12_REG			0x4C
#define SX932x_ADV_CTRL13_REG			0x4D
#define SX932x_ADV_CTRL14_REG			0x4E
#define SX932x_ADV_CTRL15_REG			0x4F
#define SX932x_ADV_CTRL16_REG			0x50
#define SX932x_ADV_CTRL17_REG			0x51
#define SX932x_ADV_CTRL18_REG			0x52
#define SX932x_ADV_CTRL19_REG			0x53
#define SX932x_ADV_CTRL20_REG			0x54
/*******Sensor Readback ****************/
#define SX932x_CPSRD          		0x60
#define SX932x_USEMSB         		0x61
#define SX932x_USELSB         		0x62
#define SX932x_AVGMSB         		0x63
#define SX932x_AVGLSB         		0x64
#define SX932x_DIFFMSB        		0x65
#define SX932x_DIFFLSB        		0x66
#define SX932x_OFFSETMSB			0x67
#define SX932x_OFFSETLSB			0x68
#define SX932x_SARMSB				0x69
#define SX932x_SARLSB				0x6A

#define SX932x_SOFTRESET_REG  		0x9F
#define SX932x_REV_REG				0xFB
/******IrqStat 0:Inactive 1:Active********/
#define SX932x_IRQSTAT_RESET_FLAG      	0x80
#define SX932x_IRQSTAT_TOUCH_FLAG      	0x40
#define SX932x_IRQSTAT_RELEASE_FLAG    	0x20
#define SX932x_IRQSTAT_COMPDONE_FLAG   	0x10
#define SX932x_IRQSTAT_CONV_FLAG       	0x08
#define SX932x_IRQSTAT_PROG2_FLAG		0x04
#define SX932x_IRQSTAT_PROG1_FLAG     	0x02
#define SX932x_IRQSTAT_PROG0_FLAG   	0x01
/****** RegStat0********************/
#define SX932x_PROXSTAT_PH3_FLAG    		0x08
#define SX932x_PROXSTAT_PH2_FLAG   			0x04
#define SX932x_PROXSTAT_PH1_FLAG   			0x02
#define SX932x_PROXSTAT_PH0_FLAG   			0x01
/******SoftReset *******************/
#define SX932x_SOFTRESET_VALUE  			0xDE
#define SX932x_REV_VALUE					0x11

#ifdef SX932X_GET_PARAMETER_FROM_SMEM
#define SX932X_PARAMETER_NUM 9
#define SX932X_PARAMETER_NUM_MAX 12
#endif

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

typedef struct
{
    /* Batch Sample type as defined in sns_diag.pb.h */
    sns_diag_batch_sample_type sample_type;
    /* Timestamp of the sensor state data sample */
    sns_time timestamp;
    /*Raw sensor state data sample*/
    float sample[SX932x_NUM_AXES];
    /* Data status.*/
    sns_std_sensor_sample_status status;
} sx931x_batch_sample;

typedef struct _REGISTER_SETTING
{
    uint8_t Register;
    uint8_t Value;
} REGISTER_SETTING, *PREGISTER_SETTING;

void sx932x_dump_reg(sns_sensor_instance *this);
sns_rc sx932x_reset_device(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            sx932x_sensor_type sensor);
void sx932x_send_config_event(sns_sensor_instance * const instance);
sns_rc sx932x_get_who_am_i(sns_sync_com_port_service *scp_service,
                                sns_sync_com_port_handle *port_handle,
                                uint8_t *buffer);

void sx932x_reconfig_hw(sns_sensor_instance *this, sx932x_sensor_type sensor_type);
void sx932x_convert_and_send_sar_sample(sns_sensor_instance *const instance,
                                            sns_time            timestamp,
                                            const uint8_t       data[6]);
void sx932x_convert_and_send_temp_sample(sns_sensor_instance *const instance,
                                            sns_time            timestamp,
                                            const uint8_t       data[3]);

void sx932x_set_temperature_polling_config(sns_sensor_instance *const this);
void sx932x_set_sar_polling_config(sns_sensor_instance *const this);
void sx932x_handle_sar_data_stream_timer_event(sns_sensor_instance * const instance);
void sx932x_handle_sar_data_stream_interrupt_event(sns_sensor_instance *const instance);
void sx932x_register_interrupt(sns_sensor_instance *this);
void sx932x_update_intr(sns_sensor_instance *const instance);
sns_rc sx932x_device_sw_reset(sns_sync_com_port_service *scp_service,
                                    sns_sync_com_port_handle *port_handle);
