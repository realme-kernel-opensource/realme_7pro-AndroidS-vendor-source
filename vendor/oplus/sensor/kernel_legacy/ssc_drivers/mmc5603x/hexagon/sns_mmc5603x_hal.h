#pragma once
/**
 * @file sns_mmc5603x_hal.h
 *
 * Hardware Access Layer functions.
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include <stdint.h>

#include "sns_mmc5603x_lite.h"
#include "sns_mmc5603x_sensor_instance.h"
#include "sns_diag.pb.h"
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#ifdef MMC5603X_ENABLE_DIAG_LOGGING
#include "sns_std.pb.h"
#endif
#include "sns_std_sensor.pb.h"
#include "sns_registry_util.h"



///////////////////////////////memsic///////////////////////////////////////


//MMC5603NJ Register Addresses
#define MMC5603NJ_REG_DATA         			0x00
#define MMC5603NJ_REG_TMPT     			       0x09
#define MMC5603NJ_REG_STATUS0     			0x19
#define MMC5603NJ_REG_STATUS1     			0x18
#define MMC5603NJ_REG_ODR     			       0x1A
#define MMC5603NJ_REG_CTRL0 	    			0x1B
#define MMC5603NJ_REG_CTRL1        			0x1C
#define MMC5603NJ_REG_CTRL2        			0x1D
#define MMC5603NJ_REG_WHO_AM_I             	0x39
#define MMC5603NJ_REG_THRD             	              0x3C
//MMC5603NJ  S4S 
#define MMC5603NJ_REG_TPH0             	              0x0B
#define MMC5603NJ_REG_TPH1             	              0x0A
#define MMC5603NJ_REG_TURR             	              0x0C
#define MMC5603NJ_REG_DT0             	              0x0D



/*MMC5603NJ Measurement Command*/
#define MMC5603NJ_BW_100HZ           		        0x00     
#define MMC5603NJ_BW_300HZ           		        0x01     
#define MMC5603NJ_BW_500HZ           		        0x02
#define MMC5603NJ_BW_1000HZ           		 0x03
#define MMC5603NJ_SOFT_RESET           		 0x80
#define MMC5603NJ_S4S_CONF           		        0xA0
#define MMC5603NJ_CONT_MEASURE           		 0x1D
#define MMC5603NJ_AUTO_SET           		        0x10
#define MMC5603NJ_MANU_TM           		        0x01
#define MMC5603NJ_MANU_TEMP           		 0x02
#define MMC5603NJ_MANU_SET           		        0x08
#define MMC5603NJ_MANU_RESET           		 0x10
#define MMC5603NJ_SFTEST_POSEN                     0x20


/** Default values loaded in probe function */
#define MMC5603NJ_WHOAMI_VALUE              (0x10)  /** Who Am I default value */
#define MMC5603NJ_OTP_READ_DONE		0x10


/** Off to idle time */
#define MMC5603NJ_OFF_TO_IDLE_MS      100  //ms

#define MMC5603NJ_SINGLE_TM_TIME      10  //ms
#define MMC5603NJ_SINGLE_ST_TM         2  //ms
#define MMC5603NJ_SINGLE_SET_TIME      1  //ms
#define MMC5603NJ_ST_DELTA_VALUE 	    100//count
#define MMC5603NJ_STABLE_DELAY          20

#define MMC5603NJ_NUM_AXES           3
#define MMC5603NJ_CMD_TMM                 0x01
#define MMC5603NJ_CMD_TMT                 0x02
#define MMC5603NJ_CMD_START_MDT           0x04
#define MMC5603NJ_CMD_SET                 0x08
#define MMC5603NJ_CMD_RESET               0x10
#define MMC5603NJ_CMD_AUTO_SR_EN          0x20
#define MMC5603NJ_CMD_AUTO_ST_EN          0x40
#define MMC5603NJ_CMD_CMM_FREQ_EN         0x80
#define MMC5603NJ_CMD_CMM_EN              0x10

/* Bit definition for control register 1 0x1C */
#define MMC5603NJ_CMD_BW00			0x00
#define MMC5603NJ_CMD_BW01			0x01
#define MMC5603NJ_CMD_BW10			0x02
#define MMC5603NJ_CMD_BW11			0x03
#define MMC5603NJ_CMD_ST_ENP			0x20
#define MMC5603NJ_CMD_ST_ENM			0x40
#define MMC5603NJ_CMD_SW_RST			0x80


/* Bit definition for control register ODR 0x1A */
#define MMC5603NJ_CMD_ODR_1HZ			0x01
#define MMC5603NJ_CMD_ODR_5HZ			0x05
#define MMC5603NJ_CMD_ODR_10HZ		0x0A
#define MMC5603NJ_CMD_ODR_50HZ		0x32
#define MMC5603NJ_CMD_ODR_100HZ		0x64
#define MMC5603NJ_CMD_ODR_200HZ		0xC8
#define MMC5603NJ_CMD_ODR_255HZ		0xFF



#define MMC5603NJ_REG_X_THD               0x1E
#define MMC5603NJ_REG_Y_THD               0x1F
#define MMC5603NJ_REG_Z_THD               0x20

#define MMC5603NJ_REG_ST_X_VAL            0x27
#define MMC5603NJ_REG_ST_Y_VAL            0x28
#define MMC5603NJ_REG_ST_Z_VAL            0x29
#define MMC5603NJ_SAT_SENSOR              0x20

///////////////////////////////memsic//////////////////////////////////////
#define MMC5603X_TIME_FOR_MEASURE_US                 7200 //us (TYP)

sns_rc mmc5603x_enter_i3c_mode(sns_sensor *const this,
                              sns_sensor_instance *const instance,
                              mmc5603x_com_port_info *com_port,
                              sns_sync_com_port_service * scp_service);



/**
 *  Address registers
 */

#define MEMSIC_MMC5603X_REG_ST1                         (0x10)//?????
#define MEMSIC_MMC5603X_REG_CNTL3                       (0x32)////???? sw_reset
#define MEMSIC_MMC5603X_REG_TPH1                        (0xC0)///s4s
#define MEMSIC_MMC5603X_REG_SYT                         (0xC3) ////s4s
#define MEMSIC_MMC5603X_REG_DT                          (0xC4)////s4s
/** MMC5603X number of data types*/
#define MMC5603X_NUM_READ_DEV_ID                     4
#define MMC5603X_NUM_SENSITIVITY                     3
#define MMC5603X_NUM_DATA_ST1_TO_ST2                 9
#define MMC5603X_NUM_DATA_HXL_TO_ST2                 8

/** Data ready bit */
#define MMC5603X_DRDY_BIT                            0x1

/** Data over run bit */
#define MMC5603X_DOR_BIT                             0x2

/** Magnetic sensor overflow bit */
#define MMC5603X_HOFL_BIT                            0x8

/** Soft reset */
#define MMC5603X_SOFT_RESET                          0x1

#define MMC5603X_MAX_FIFO_SIZE                      200
#define MMC5603X_MAX_PHYSICAL_FIFO_SIZE              32 // Physical mag senosr allows maximum upto 32 samples

/** Off to idle time */
#define MMC5603X_OFF_TO_IDLE_MS                      100 //ms

/** Wait time before mode setting */
#define MMC5603X_TWAIT_USEC                          100 //us

/** masurement time */


#ifdef MMC5603X_ENABLE_S4S
/** s4s configuration */
#define MMC5603X_S4S_INTERVAL_MS                     1000 //ms
#define MMC5603X_S4S_RR                              1
#endif // MMC5603X_ENABLE_S4S


/*******************************
 * Number of axes in a 3 axis sensor
 */
#define MMC5603X_NUM_AXES                            TRIAXIS_NUM

/*******************************
 * Measurement time calculation bit resolution
 */
#define MMC5603X_CALC_BIT_RESOLUTION                 13
#define MMC5603X_IRQ_NUM_FOR_OSC_ERROR_CALC          3

#ifdef MMC5603X_BOARD_HDK820
#define MMC5603X_CALC_BIT_ERROR                      4000  // HDK820 (huge jitter observed)
#else
#define MMC5603X_CALC_BIT_ERROR                      40
#endif

#ifdef MMC5603X_ENABLE_DIAG_LOGGING
/*******************************
 * Log structure definition
 */
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

/*******************************
 * Unencoded batch sample
 */
typedef struct
{
  /* Batch Sample type as defined in sns_diag.pb.h */
  sns_diag_batch_sample_type sample_type;
  /* Timestamp of the sensor state data sample */
  sns_time timestamp;
  /*Raw sensor state data sample*/
  float sample[MMC5603X_NUM_AXES];
  /* Data status.*/
  sns_std_sensor_sample_status status;
} mmc5603x_batch_sample;
#else
typedef struct log_sensor_state_raw_info
{
  // enmpty
} log_sensor_state_raw_info;
#endif

/******************* Function Declarations ***********************************/
/**
 * Write wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           write buffer
 * @param[i] bytes            bytes to write
 * @param[o] xfer_bytes       bytes written
 * @param[i] save_write_time  true to save write transfer time.
 *
 * @return sns_rc
 */
sns_rc mmc5603x_com_write_wrapper(sns_sync_com_port_service * scp_service,
                                 sns_sync_com_port_handle *port_handle,
                                 uint32_t reg_addr,
                                 uint8_t *buffer,
                                 uint32_t bytes,
                                 uint32_t *xfer_bytes,
                                 bool save_write_time );

/**
 * Resets the Sensor SW.
 * This function is used in mmc5603x drivers flow only.
 * If call from other flow directly for HW reset,
 * should also reset the SW settings like a mag_info.curr_odr.
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] scp_service   synch COM port service
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc mmc5603x_device_sw_reset(sns_sensor_instance *const this,
                               sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle);

/**
 * Enable Mag streaming. enables Mag sensor with
 * non-zero desired ODR.
 *
 * @param[i] state         Instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc mmc5603x_start_mag_streaming(sns_sensor_instance *const this);

/**
 * Disable Mag streaming.
 *
 * @param[i] state         Instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc mmc5603x_stop_mag_streaming(sns_sensor_instance *const this);

/**
 * Gets Who-Am-I register for the sensor.
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] scp_service   handle to synch COM port service
 *
 * @param[o] buffer        who am I value read from HW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc mmc5603x_get_who_am_i(sns_sync_com_port_service * scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer);

    /**
 *  get the trim  data 0x27-0x29
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] scp_service   handle to synch COM port service
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */                        
sns_rc mmc5603x_get_trim_data(sns_sync_com_port_service *scp_service,
                              sns_sync_com_port_handle *port_handle
                              ) ;                           

/**
 * Gets current ODR.
 *
 * @param[i] curr_odr       Current ODR.
 *
 * @return current ODR
 */
float mmc5603x_get_mag_odr(mmc5603x_mag_odr curr_odr);

/**
 * Provides sample interval based on current ODR
 *
 * @param[i] curr_odr       Current ODR.
 *
 * @return sampling interval time in ticks
 */
sns_time mmc5603x_get_sample_interval(mmc5603x_mag_odr curr_odr);

/**
 * Process buffer and extracts mag samples from the buffer
 * and generates event.
 *
 * @param[i] instance              Sensor instance
 * @param[i] first_ts              Timestamp of first sample in fifo
 * @param[i] interval              Sampling interval in time ticks
 * @param[i] fifo                  Buffer containing sample read from HW FIFO
 * @param[i] num_bytes             Number of bytes in fifo buffer
 *
 */
void mmc5603x_process_mag_data_buffer(sns_sensor_instance *instance,
                                     sns_time            first_ts,
                                     sns_time            interval,
                                     uint8_t             *fifo,
                                     size_t              num_bytes);


/**
 * Sends a FIFO complete event.
 *
 * @param instance   Instance reference
 */
void mmc5603x_send_fifo_flush_done(sns_sensor_instance *const instance);

/**
 * Read mag samples from the buffer
 * and generates event.
 *
 * @param instance                 Sensor Instance
 */
void mmc5603x_read_mag_samples(sns_sensor_instance *const instance);

/**
 * Sends config update event for the chosen sample_rate
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 * @param[i] instance    reference to this Instance
 */
sns_rc mmc5603x_send_config_event(sns_sensor_instance *const instance);

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] log_raw_info   Pointer to logging information
 *       pertaining to the sensor
 * @param[i] batch_complete true if submit request is for end
 *       of batch
 *  */
void mmc5603x_log_sensor_state_raw_submit(
  log_sensor_state_raw_info *log_raw_info,
  bool batch_complete);

/**
 * Add raw uncalibrated sensor data to Sensor State Raw log
 * packet
 *
 * @param[i] log_raw_info Pointer to logging information
 *                        pertaining to the sensor
 * @param[i] raw_data     Uncalibrated sensor data to be logged
 * @param[i] timestamp    Timestamp of the sensor data
 * @param[i] status       Status of the sensor data
 *
 * * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc mmc5603x_log_sensor_state_raw_add(
  log_sensor_state_raw_info *log_raw_info,
  float *raw_data,
  sns_time timestamp,
  sns_std_sensor_sample_status status);

/**
 * Allocate Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] log_size   Optional size of log packet to
 *    be allocated. If not provided by setting to 0, will
 *    default to using maximum supported log packet size
 */
void mmc5603x_log_sensor_state_raw_alloc(
  log_sensor_state_raw_info *log_raw_info,
  uint32_t log_size);

/**
 * Encode log sensor state raw packet
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *                       during encode
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc mmc5603x_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written);

/**
 * Enable interrupt if not already enabled
 *
 */
void mmc5603x_register_interrupt(sns_sensor_instance *this);

/**
 * Enable heart beat timer if not already enabled
 *
 */
void mmc5603x_register_heart_beat_timer(sns_sensor_instance *this);

/**
 * Enable timer if not already enabled
 *
 */
void mmc5603x_register_timer(sns_sensor_instance *this);

/**
 * Get time for measurement
 *
 * @param[i] device_select  Device type
 * @param[i] sdr            Drive mode setting
 * @param[o] measure_us     Measurement time in usec
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc mmc5603x_get_meas_time(memsic_device_type device_select, uint8_t sdr, sns_time* measure_us );

/**
 * Configures sensor with new/recomputed settings
 *
 * @param instance   Instance reference
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 */
sns_rc mmc5603x_reconfig_hw(sns_sensor_instance *this);

/**
 * Run a hardware self-tests.
 *
 * @param[i] instance     reference to the instance
 * @param[o] err          error code
 *
 * @return bool
 */
bool mmc5603x_hw_self_test(sns_sensor_instance *instance);



/**
 * Executes requested self-tests.
 *
 * @param instance     reference to the instance
 *
 * @return none
 */
void mmc5603x_run_self_test(sns_sensor_instance *instance);


/**
 * Send Calibration event to client
 *
 * @param[i] instance        instance reference
 * @return none
 */
void mmc5603x_send_cal_event(sns_sensor_instance * const instance);

/**
 * Reset Calibration values
 *
 * @param[i] instance        instance reference
 * @return none
 */
void mmc5603x_reset_cal_data(sns_sensor_instance *const instance);
void mmc5603x_validate_timestamp_for_dri(sns_sensor_instance *const instance);

void mmc5603x_validate_timestamp_for_polling(sns_sensor_instance *const 
instance);
void mmc5603x_continue_client_config(sns_sensor_instance *const instance);

