/*******************************************************************************
 * Copyright (c) 2017-2020, Bosch Sensortec GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
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
/**
 * @file sns_bmi26x_ver.h
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

/*==============================================================================
  DESCRIPTION

  Impelements the driver for the Bosch BMI26X part

  EDIT HISTORY FOR FILE


  when          who     what
  ----------    ---     -----------------------------------------------------------
  12/22/17      lk      0.01: driver initial draft for openssc_ver_50 based on sample driver
  12/28/17      lk      0.10: driver enumeration success
  02/12/18      lk      0.11: take changes from bmi26x 2.12.26
  02/13/18      lk      0.12: avoid false detection of HW
  02/13/18      lk      0.13: remove kw WARNING
  03/08/18      lk      0.14: merged some changes in bmi26x_3.01.01
  03/19/18      lk      0.15: [merged] 3.01.01.02
  03/19/18      lk      0.16: [merged] 3.01.01.03
  04/12/18      lk      0.17: [merged] 3.01.01.05
  04/13/18      lk      0.18: soft reset when an instance is removed
  07/03/18      fx      0.19: reset sensor state after 'reset' during instance initialization stage
  07/10/18      fx      0.20: load sensor configuration function works
  07/17/18      fx      0.21: step counter with timer works
  08/13/18      fx      0.22: [update] configure fw change on BMI26X C2 sample, driver update for it.
  08/14/18      fx      0.23: [update] MD function
  08/14/18      fx      0.24: [update] optimize the MD register configuration
  08/14/18      fx      0.25: [update] add step detector for pedo fac test
  08/20/18      fx      0.26: [fixed] fix the ACC range issue
  08/20/18      fx      0.27: [improve] load fw and keep power live before instance deinit
  08/22/18      fx      0.28: [improve] add data ready function
  08/23/18      fx      0.29: [update] OIS
  08/28/18      fx      0.30: [improve] low power mode flag assert when power configuration
  10/26/18      fx      0.31: CRT basic function supported
  11/02/18      fx      0.32: fine-tune MD parameters
  11/06/18      fx      0.33: remove the queue on command register
  11/23/18      fx      0.34: CRT over FIFO works
  11/27/18      fx      0.35: CRT in fac
  11/27/18      fx      0.36: crt specified registry r/w works
  11/30/18      fx      0.37: [update] cross sensitivity
  11/30/18      fx      0.37.1: [improve] code for sensor data debugging
  11/30/18      fx      0.38: [fixed] fac test failure issue
  12/03/18      fx      0.39: [fixed] acc data issue in the data ready mode
  12/04/18      fx      0.40: [fixed] crt gain enable register
  12/05/18      fx      0.41: [fixed] +/- 2g fac test failed
  12/07/18      fx      0.41.0.1: [improve] add time-stamp log on data ready frame report
  12/07/18      fx      0.42: burst write length resize for the configure file download
  12/10/18      fx      0.43: optimize the gyro crt parameters configuration
  12/11/18      fx      0.43.1: [fixed] parameter error in normal mode judgment
  12/11/18      fx      0.44: [improve] do a soft reset before crt
  12/12/18      fx      0.45: [improve] OIS configuration support
  12/14/18      fx      0.46: sensor h/w self-test
  01/04/19      fx      0.50: first draft version for DAE
  01/09/19      fx      0.51: acc/gyro work with DAE under DRDY mode
  01/17/19      fx      0.52: disable INT latch and keep gyro_filter_perf configuration in lpm
  01/21/19      fx      0.53: acc/gyr works in fifo mode under workaround
  01/23/19      fx      0.54: acc/gyr works in fifo mode under workaround, changing odr still work
  01/23/19      fx      0.55: fix the DAE stop/start streaming
  02/12/19      fx      0.56: [fixed] no interrupt when then build configuration has latency after the h/w configuration
  02/20/19      fx      0.57: [improve] power configuration when keep-power-on enabled
  02/20/19      fx      0.58: [fixed] send flush data request in DAE when flush request in present
  02/21/19      fx      0.59: [fixed] DAE water mark and aging limitation
  02/22/19      fx      0.60: [fixed] flush event for flush-on-request sensor
  02/23/19      fx      0.61: [improve] flush HW when check the INT pin is high
  02/23/19      fx      0.62: [fixed] sensor state is error when checking the master sensor
  02/26/19      fx      0.63: [improve] code for sensor status reset
  02/26/19      fx      0.64: [improve] improve MD
  02/28/19      fx      0.65: [improve] HW configure event for MD
  02/28/19      fx      0.66: [fixed] gyro cross sensitivity be a signed value
  03/04/19      fx      0.67: [improve] add a reset wrapper function to do the device reset operations after a device soft-reset
  03/04/19      fx      0.68: [fixed] MD feature lost the DAE connection
  03/08/19      fx      0.69: [fixed] HW/SW test restore status issue
  03/20/19      fx      0.70: [fixed] publish attribute issue for SNS_STD_SENSOR_ATTRID_API
  03/22/19      fx      0.71: [improve] double check INT status after INT handling
  04/18/19      fx      0.72: [improve] time-stamp algo improve under DAE mode
  04/18/19      fx      0.73: [improve] perform acc factory test when z-axis configured invert
  04/19/19      fx      0.74: [fixed] sensor recover after fac calibration test
  04/29/19      fx      0.75: [improve] update cfg stream block size to improve performance
  04/29/19      fx      0.76: [improve] flush IMU sensor when acc/gyro sensor flush request comes in
  05/05/19      fx      0.77: [fixed] typo issue for imu flush
  05/09/19      fx      0.78: new configure file
  05/10/19      fx      0.79: crt without sw reset
  05/13/19      fx      0.80: crt request is aborted when gyro is running
  05/16/19      fx      0.81: pending HW cfg done after fifo flush data event received
  05/20/19      fx      0.82: restructure code for island
  05/21/19      fx      0.83: add SW reset in fac test
  05/22/19      fx      0.84: [fixed] handle error when crt failure
  05/27/19      fx      1.0.0.0: multiple-crt function for release
  06/17/19      fx      1.0.0.1: sensor attributes publish optimized
  06/25/19      fx      1.0.0.2: fix the typo issue for async request
  06/28/19      fx      1.0.0.3: fix bus read on instance init and deinit
  07/08/19      fx      1.0.0.4: [improve] redefine config message structure for set_client_config
  07/10/19      fx      1.0.0.5: [improve] combine set_client_configs
  07/24/19      fx      1.0.0.6: [improve] time stamp algo against jitter issue
  07/26/19      fx      1.0.0.7: [improve] sensor request process over power rail timer
  07/30/19      fx      1.0.0.8: [improve] sensor HW init operation after power rail on
  08/08/19      fx      1.0.0.9: [fixed] gyro no output when go run enable-disable-enable due to ois configuration
  08/22/19      fx      1.0.0.10:[improve] combine FIFO water mark interrupt with a on-going FIFO reading async request
  08/23/19      fx      1.0.0.11:[fixed] timer stream should be set to NULL after destroy
  08/29/19      fx      1.0.0.12:[align] power rail latecy off when no client on sensors
  09/06/19      fx      1.0.0.13:[req] standard configure file
  09/10/19      fx      1.0.0.14:[req] lowg support on INT2
  09/16/19      fx      1.0.0.15:[improve] bus r/w delay depend on power mode
  09/16/19      fx      1.0.0.16:[improve] crt gain update
  09/18/19      fx      1.0.0.17:[improve] sbus_read/sbus_write
  09/19/19      fx      1.0.0.18:[fixed] crash on access configure stream in island mode
  09/29/19      fx      1.0.0.19:[align] power rail update before instance creation
  09/30/19      fx      1.0.0.20: free fall configuration file
  10/09/19      fx      1.0.0.21: fix lowg hangup issue
  10/09/19      fx      1.0.0.22: avoid duplicated lowg event
  10/22/19      fx      1.0.0.23: drop fifo frame before new configure valid
  10/23/19      fx      1.0.0.24: power rail should be add in the discover hw
  10/24/19      fx      1.0.0.25: fix ts estimation error
  10/26/19      fx      1.0.0.26:[improve] sdc code
  10/27/19      fx      1.0.0.27:new configure file version with lowg
  10/29/19      fx      1.0.0.28:improve power rail available checking
  10/30/19      fx      1.0.0.29:update support SPI3 for ois
  11/04/19      fx      1.0.0.30:[fixed] enable sensor temp power on sensor temperature streaming request
  11/05/19      fx      1.0.0.31:[improve] configure SPI3 base on configuration from json file
  11/06/19      fx      1.0.0.32:[fixed] burst write size for AndroidQ version on sdk7.1.2
  11/12/19      fx      1.0.0.33:[fixed] fix com_div algo mod data type lead to crash issue
  11/13/19      fx      1.0.0.34:[improve] add input parameters valid checking before reg r/w operation on bus
  11/15/19      fx      1.0.0.35:[fixed] put feature data/function into island
  12/12/19      fx      1.0.0.36:free fall function update
  12/12/19      fx      1.0.0.37:[align] free fall debug mode support
  12/16/19      fx      1.0.0.38:[fixed] free fall configure structure definition
  12/19/19      fx      1.0.0.39:ois lp configuration
  12/24/19      fx      1.0.0.40:handle flush request while enable ongoing
  12/25/19      fx      1.0.0.41:configuration file v4.14
  12/26/19      fx      1.0.0.42:support get configuration version
  12/30/19      fx      1.0.0.43:[fixed]sensor temperature keep reporting data after disable it
  12/31/19      fx      1.0.0.44:[align] registry update on request sensor ONLY
  12/31/19      fx      1.0.0.45:support filter delay
  01/02/20      fx      1.0.0.46:optimize int checking in DAE
  01/02/20      fx      1.0.0.47:[fixed] hard code in gyr ois low pass filter configuration lead to power consumption abnormal
  01/08/20      fx      1.0.0.48:[fixed] crt state issue when crt abort
  02/12/20      fx      1.0.0.49:[improve] sensor stopping while hw configuration changing
  02/15/20      fx      1.0.0.50:wait for on-going dae flush finish before the hw config
  02/15/20      fx      1.0.0.51:improve the dae gpio checking process
  02/16/20      fx      1.0.0.52:align: wait sns_dae_resp(pause sampling) before destroy
  02/20/20      fx      1.0.0.53:align: ensure advanced power save cleared before do crt
  02/22/20      fx      1.0.0.54:800hz support
  02/24/20      fx      1.0.0.55:sensor temperature dae flush process fixed.
  02/25/20      fx      1.0.0.56:[improve]multiple CRT process
  03/18/20      fx      1.0.0.57 Basic Heart beat function
  03/31/20      fx      1.0.0.58 [req]Check cfg state in Heart beat function
  04/10/20      fx      1.0.0.59 align: sensor temperature should use different name from acc/gyro
  07/06/20      fx      1.0.0.60 gyro fac precision
  07/29/20      fx      1.0.0.61 merged DAE/Non-DAE
  07/31/20      fx      1.0.0.62 some bug fix merged
  08/05/20      fx      1.0.0.63 HB WML threshold
  08/06/20      fx      1.0.0.64 check cfg status in HB context
  08/18/20      fx      1.0.0.65 support double-tap feature
  08/27/20      fx      1.0.0.66 multiple double-tap event fired event when client hold on
  09/22/20      fx      1.0.0.67 int status double check in dae
  09/23/20      fx      1.0.0.68 ignore the int which already handled in data event context
  10/10/20      mt      1.0.0.69 update the config file to the version BMI260T_Image_09_10_2020_V4.0
  10/15/20      mt      1.0.0.70 update the config file to the version 02_BMI260T_Image_14_10_2020_V4.1
  11/03/20      mt      1.0.0.71 fix the cali no data BFT-1669
  11/11/20      mt      1.0.0.72 add the work around for gyro range change
  ============================================================================*/

#ifndef __SNS_DD_BMI26X_VER_H
#define __SNS_DD_BMI26X_VER_H

#define SNS_DD_MODULE_VER_BMI26X        "BMI26X_DD_VERSION: 2020/11/11 10:00:00 1.00.00.72"
#define BMI26X_SEE_DD_ATTRIB_VERSION    0x01000048

#endif
