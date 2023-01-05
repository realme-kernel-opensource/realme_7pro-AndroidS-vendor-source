/**
 * @file sns_lsm6dso_ver.h
 *
 * Copyright (c) 2020, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
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
 *
 ******************************************************************************/

/*==============================================================================

    Change Log:

    EDIT HISTORY FOR FILE

    Oct 24 2017 ST - 1.5.0
      - Align Driver with LSM6DSM_SEE_V1.29.0
    Nov 1 2017 ST - 1.6.0
      - MD support
    Nov 3 2017 ST - 1.7.0
      - Align Driver with LSM6DSO_SEE_V1.38.0
    Nov 9 2017 ST - 1.8.0
      - Fix for bad timestamp at fastest ODR
      - Minor self test fixes
    Nov 17 2017 ST - 1.9.0
      - Batching fixes
    Nov 20 2017 ST - 1.10.0
      - MD fix for heart beat(wmk fix at 0 ODR)
      - Latency fix
    Nov 30 2017 ST - 1.11.0
      - Registry write fix
      - MD fix for WMK=1
      - Filter update to improve MD sstvt and STD-DEV error
      - This release has MD updated to match LSM6DSM driver
    Dec 13 2017 ST - 1.12.0
      - Latency Fix
      - CTS fixes
      - Bad gyro data fix
    Feb 27 2018 QC - 2.00.0
      - Restructured to have shared state amongst sibling sensors
      - Updated DAE interface module to reenable streaming via DAE sensor
    Mar 16 2018 QC - 2.1.0
      - Fixed support for non-I3C config
      - Fixed DAE related config
    Mar 21 2018 QC = 2.2.0
      - Fixed MD re-arming when there are non-gated accel streams
      - Updated max rate to 416Hz
      - Fixed non-I3C support
    Apr 10 2018 QC = 2.3.0
      - Enabled max FIFO
      - Enabled ODR_CHG virtual sensor
    Apr 19 2018 ST - 2.4.0
      - Low latency attributes updates
      - MD + Gyro spurious interrupt issue
      - Dual sensor support
      - Heart Attack fix
      - Self Test new feature and fixes
      - level triggred interrupt support
    Apr 20 2018 ST - 2.5.0
      - support for S4S sync feature
    Apr 23 2018 QC - 2.6.0
      - Fixed WM=0 in Config event
    May 08 2018 QC - 2.7.0
      - Handles Cal Reset request as the first request
      - Starts/stops DAE streaming only if necessary
      - Fixed missing Config events for Sensor Temperature and MD sensors
      - Gyro's FIFO size attribute adjusted to half of max FIFO
    May 25 2018 ST - 2.7.1
      - Add ESP features(Free_Fall and High_Shock)
    May 16 2018 QC - 2.8.0
      - Make HW FIFO WM a factor of DAE FIFO WM
    May 24 2018 QC - 2.9.0
      - Uses min ODR of 26Hz
      - Waits 500us before resetting sensor
      - Updates temperature sensor info when starting/stopping DAE streaming.
    May 29 2018 QC - 2.10.0
      - update for flush only request
    June 01 2018 QC - 2.11.0
      - Fixed delivery latency issue in DAE-enabled config
    June 06 2018 QC - 2.12.0
      - Interval calculation fix for flushed samples without any prior interrupts
      - If only data age limit changes DAE stream is updated instead of stopped and restarted
    June 11 2018 QC - 2.13.0
      - Fixed report rate calculation for concurrent streaming and batching requests
    June 19 2018 QC - 2.14.0
      - Integrated version 2.7.1
      - Cleaned up ESP featurization
    July 2 2018 ST - 2.15.0
      - Bug fix: Gyro invalid samples after config changes
      - Update ESP interrupt handling for DAE enabled cases
      - New feature: Added support for OIS
      - Created Build config file for debug and build flags
    July 17 2018 QC - 2.16.0
      - Cleaned up OIS featurization
      - Fixed timestamp calculation related to Flush on DAE stream
    July 26 2018 ST - 2.17.0
      - Bug fixes for MD, gated/non-gated accel and gyro concurrent usecases
      - Timestamp fixes for gyro and MD use cases
      - Updated timestamp logic using moving window average
      - DAE compilation flag for dae support
    July 31 2018 QC - 2.18.0
      - Fixed calibration registry update
      - Reduced max FIFO to 208
    Aug 03 2018 ST - 2.19.0
      - Tag sensor fix
    Aug 08 2018 QC - 2.20.0
      - Made unreliable sample as is ( reverted from 0 )
    Aug 27 2018 ST/QC - 2.21.0
      - Featurized S4S
      - Added sanity test debug messages
      - Enabled faster MD settle time
      - Fixed various config event issues
      - Dual Sensor support for DAE-enabled config
    Aug 30 2018 ST/QC - 2.22.0
      - Fixed QC tracker related to Temperature(#10),MD physical config (#16)
      - Fixed QC tracker related to Partial Fifo after flush(#20)(Non-DAE mode fixed)
      - Fixed issues related to Self Test
      - Fixed issues related to ESP
    Aug 31 2018 ST - 2.23.0
      - update timestamp logic for DAE
      - unify timestamp logic for DAE and DAE bypass mode
      - Fixed timestamp gap issues in DAE mode (Tkr 12)
      - update build config for 855
    Sep 10 2018 ST - 2.24.0
      - Enable timestamp correction function in DAE path
      - use BDR interrupt bit instead of WMK
    Sep 10 2018 QC - (2.22.0)
      - Changed how MIN and MAX ODRs are selected
    Sep 13 2018 QC - 2.25.0
      - Merged QC version 2.22.0 with ST version 2.24.0
      - Changed to update heartbeat timer when changing DAE WM without changing FIFO WM
    Sep 14 2018 ST - 2.26.0
      - Fix -ve timestamp issue. trk-9
      - Other minor fixes
    Sep 20 2018 QC - 2.27.0
      - No longer removing heartbeat timer when only max batch or flush only requests remain
    Sep 28 2018 QC - 2.28.0
      - Reduced frequency of heartbeat timer request
      - Made sure MIN ODR is obeyed
      - Made sure old data from DAE is consumed before new config applies
      - Made sure not to flush FIFO unnecessarily
    Oct 03 2018 QC - 2.29.0
      - Fixed DAE WM calculation to optimize power consumption
    Oct 08 2018 QC - 2.30.0
      - Reworked heartbeat interval calculation
    Oct 09 2018 ST - 2.31.0
      - update MD threshold based on Full scale setting of accel
    Oct 19 2018 QC - 2.31.0.1
      - Sends Flush event, if necessary, after processing Data event from DAE sensor
      - Waits longer before resetting sensor when creating instance
    Oct 29 2018 QC/ST - 2.31.0.2
      - QC: the combined report period is GCD of all report periods
      - QC: fixed missing Flush event for DAE-disabled config
      - ST: Made use of internal frequency fine to get accurate sampling rate
    Oct 10 2018 ST - 2.32.0
      - MD restructuring
      - Lite driver updates
      - QCOM Trk fixes
    Oct 18 2018 ST - 2.33.0
      - Fix MD issues
      - Add support for OEM config
    Nov 02 2018 ST - 2.34.0
      - Integrate 2.31.2 changes(Clients sending continuous flush)
      - Flush DAE data when new streaming starts using SEE
      - Use hardware timestamp for nominal interval
      - BDR TRG bit clearing
    Nov 09 2018 QC - 2.34.0.1
      - Consolidated get_sample_interval()
    Nov 10 2018 QC - 2.34.0.2
      - Overwrite un-reliable samples with last reliable sample,
        when there is no change in client configuration
    Nov 15 2018 QC - 2.34.0.3
      - Cleaned up FIFO WM and DAE WM calculations
    Dec 05 2018 QC - 2.34.0.4
      - Added Rigid Body Attribute to Motion Detect and ACCEL publish Rigid Body to DAE sensor	  
    Nov 15 2018 ST - 2.35.0.0
      - OIS Polling Mode support
      - One shot implementation
      - Use attempts number instead of 10ms(fixed value) for SW reset
      - reset added for ascp_req for error return types
      - reset only accel/gyro if reconfig is not required
    Dec 27 2018 ST - 2.36.0.0
      - update interrupt interval bounds when only wmk is changed
      - update timestamp logic to adjust drift when odr is changed
    Jan 10 2019 ST/QC 2.37.0.1
      - Fix timestamp latency issue
      - Fix driver logic when DAE is dropping data when FLUSH_ONLY
      - Cleanup config_sensor and odr_change logic
    Jan 24 2019 ST 2.37.0.2
      - Fix issues with sample staus when odr changed in different usecases(Ex: missing cfg tags, orphan batch etc)
    Jan 17 2019 QC 2.37.0.2
      - Updated registry defaults for Target testing without Registry support
    Jan 31 2019 QC 2.37.0.4
      - Updated registry defaults to support dual sensors
    Jan 24 2019 ST 2.38.0.0
      - support for passive sensor feature
    Feb 11 2019 ST 2.39.0.0
      - Merging IBI changes
      - restructuring esp and interrupt handlers
      - Klocwork comments
    Feb 15 2019 QC 2.39.0.1
      - Integrated ST v2.39.0.0 with QC v2.37.0.5
    Feb 21 2019 QC 2.39.0.2
      - DAE report rate set to zero for flush-only/max-batch
      - Flushes samples when increasing DAE WM
      - Correctly determines when a batch of samples belong to previous config
    Mar 8 2019 ST 2.41.0.0
      - Added support for step counter
      - Fix allways aware test accel #101,#104,#106
      - Fix for updating the heart attack counter till 6
      - Applying timestamp correction for orphan batch
    Feb 27 2019 QC 2.38.0.3
      - Low frequency CTS failures fix for DAE bypass case with level triggred.
    Mar 05 2019 QC 2.38.0.4
      - Made sure actual report rate to be at least 90% of requested rate
    Mar 07 2019 QC 2.38.0.5
      - update physical config event timestamp with current Ts for MD sensor.
    Mar 26 2019 QC 2.38.0.7
      - Revamped Passive Mode implementation
    Mar 15 2019 ST 2.42.0.0
      - Integrated Qualcomm 2.38.0.3,2.38.0.4 and 2.38.0.3 changes
      - Read sample sets from Fifo for gyro for dae-bypass mode
    Mar 28 2019 ST 2.42.0.2
      - Remove features not applicable for this device.
    Mar 29 2019 QC 2.42.0.3
      - Merged ST 2.42.0.2 and QC 2.38.0.7
    Apr 02 2019 QC 2.42.0.4
      - Fixed missing Cal event for Cal Reset request
    Apr 10 2019 QC 2.42.0.5
      - Fixed reading wakeup source register irrespective of MD and EPS
    Apr 11 2019 ST 2.43.0.0
      - Updated timestamp logic to fix latency issue during odr change
      - optimize accel/gyro data handling functions
      - move raw log packets code to a different file
    Apr 18 2019 ST 2.43.0.1
      - Changes made based on review comments
    Apr 25 2019 ST 2.43.0.2
      - Fixed Cal event timestamp for Cal Reset request
      - Fixed Config event when new request arrives before previous config generates any sample
    May 3 2019 ST 2.43.0.3
      - Clear MD from config sensor in DAE-bypass mode
      - Update the odr_index correctly
      - Update the first_timestamp correctly in case of orphan batch
      - Add the check for cal_reset request while creating new instance
      - update clear_interrupt function to include check for interrupt type and DAE
      - Update the fifo info interrupt timestamp correctly
      - Update the average sampling interval properly
      - Update the cur_wmk correctly during watermark change
      - Integrate QC release 2.43.0.1, 2.43.0.2
      - variance check update for factory test
      - Send cal_event only when factory test passed
      - update the odr value for LSM6DSO
      - Remove fifo from bypass mode when only watermark changes
    May 8 2019 ST 2.43.0.4
      - Revert the change for rejecting cal_reset request
    May 29 2019 QC 2.43.0.5
      - Config events for Temperature sensor now use ODR change timestamp
      - Cal events now use timestamp of Cal data update
      - Avoided disrupting DAE streaming unnecessarily
    Jun 10 2019 ST 2.44.0.0
      - Updated timestamp logic to minimize 2% shift in sampling interval during config change
    June 18 2019 QC 2.43.0.6
      - Restores existing request if rejecting its replacement
    June 19 2019 QC 2.43.0.7
      - Fixed wrong integer type used in HB timeout calculation
    July 02 2019 QC 2.44.0.1
      - Merged QC v2.43.0.7 with ST v2.44.0.0
    July 16 2019 ST 2.44.0.2
      - Bug Fix for missing MD disable event to FW
      - Bug Fix:Two config events with same timestamp can't have different contents
      - Updates the timestamp of physical config event once samples with previous sample rate are sent.
      - Include Qualcommm changes 2.44.0.1
      - Change the datatype to 64 bit for first_ts_gap
    July 26 2019 ST 2.44.0.3
      - Send config event if passive sensor is present
      - Send physical config event for temperature with same timestamp
        untill there is a change in configuration
      - fix factory-test zero variance issue.
      - Donot send samples if only passive client is there
    July 30 2019 QC 2.44.0.4
      - Made sure actual DAE report rate to be at least 90% of requested rate
    Aug 02 2019 QC 2.44.0.5
      - Must restart DAE streaming on interrupt event
    Aug 09 2019 ST 2.44.0.6
      - Send config event for non-dae in all cases(even if cfg tag is not there)
    Aug 15 2019 QC 2.44.0.7
      - Plug gaps in updating TS for Phy Cfg Event
    Aug 27 2019 QC 2.44.0.8
      - MD On-change requests are removed immediately following MD FIRED event
      - Updated to use new implementation of instance callback add_client_request()
    Aug 29 2019 QC 2.44.0.9
      - Batch periods of max-batch requests of a sensor no longer affect its report rate
    Aug 28 2019 ST 2.45.0.0
      - Add FSM/MLC support
      - split set_client_request into two functions for re-using in esp
      - Restructure folder structure
      - Fix zero bias issue
    Aug 30 2019 ST 2.45.0.1
      - Integrate QC 2.44.0.8 changes
      - Bug Fix:Set the odr_changed variable correctly after sending missing samples
    Sept 6 2019 ST 2.45.0.2
      - Fix step counter build/flow issue
      - Bug fix for handling esp interrupt in non-DAE mode
    Sept 18 2019 ST 2.45.0.3
      - Disable timestamp counter during power-down
      - Create MD and heart-beat timer only one time
      - Send config event when MD + gated are present
      - Send config event with correct timestamp when odr changes from odr1>odr2>odr1
      - Update the config event watermark only when there is a change in watermark
      - Send error event when invalid request is send
      - Disable FIFO interrupts while reconfiguring in DAE mode
    Sept 24 2019 ST 2.45.0.4
      - Update config event timestamp correctly
    Sept 27 2019 ST 2.45.0.5
      - Fix very fast ODR discard sample
      - Non-DAE max_wmk change from QC
      - Single data gap issue with Flush only case
    Sept 30 2019 QC 2.45.0.6
      - Send error event when new request is rejected for any reason
    Oct 03 2019 ST/QC 2.45.0.7
      - Fix for publish config event for use-cases when wmk changes
      - Fix non-DAE power rail B2B use-case
      - Fix temperature publish DAE_wmk issue
    Mon Oct 14 2019 QCOM - 2.45.0.8
      - Removed chipset specific Bus and IRQ configuration   
    Oct 22 2019 ST 2.45.0.9
      - Send config event when only passive client is there
      - Send physical config event timestamp correctly for passive clients
      - Set correct number of discard samples at 832hz
    Oct 29 2019 ST 2.45.0.10
      - Include qualcomm changes for default_reg_config files
      - Fix FSM/MLC non-DAE not working issue
      - Support to send enable/disable/fired xsensor event
      - Fix issue with config event for different payload and same timestamp
    Nov 4 2019 ST 2.45.0.11
      - Publish_config_events
        1. Seperate out MD publish config timestamp
        2. Stop overwriting passive sensors physical config event parameters when md is present
        3. Update last_sent_config timestamp for passive only when active client is present
        4. Use a boolean variable instead of state parameters to send physical config event 
        for passive sensor when active client goes away
        5. fix different payload same timestamp for active/passive temperature
        concurrency test
    Nov 19 2019 ST 2.45.0.12
        1. Send passive sensor config event when there is no active sensor
        2. Enable LPF1 for gyro for 832 hz
        3. Discard 50 samples at 832hz for Gyro if lpf1 is enabled
        4. Send config event with 0 sample rate for gated request
        5. Pass timestamp from SDC to SEE driver
        6. Write to lpf1 only during add/remove of 832 hz
        7. turn off hw timer when gated client only
    Nov 21 2019 QC 2.45.0.13
      - Processes config requests before flush requests
    Nov 26 2019 QC 2.45.0.14
     - Correctly updating 'power_rail_pend_state' in LSM6DST during/post instance removal process.
    Jan 17 2020 ST 2.45.0.15
     - Fix issue of Gyro drop contineously
     - XSensor/ESP:Enable/Disable embedded space before reading FSM_OUTS/LC
     - XSensor/ESP:No Accel Data issue non-DAE + ESP
     - Put reading current time from meta under macro
     - Donot do dae flush if odr is not set(Change from QC)
     - Latency issue fix: Fix for sample drops during odr changes
    Feb 12 2020 QC 2.45.0.16
     - Fixed implicit-int-float-conversion error found with hexagon8.4 compiler 
    Feb 12 2020 ST 2.45.0.17
     - Reload esp/xsensor after heart attack
     - Add stub function for new esp function
     - Reduce I3C bus available time to 2us instead of default 50us
     - flush dae before sending new configuration to DAE
    Feb 27 2020 ST 2.46.0.0
     - Reduce DAE timestamp in meta data to 1 byte
     - Fix latency issue on ODR change when gyro introduced 
     - Fix physical config event with zero timestamp
     - Support physical config event for OFF operation mode
     - Reduce island mode size
    Mar 05 2020 QC 2.46.0.1
     - Return error value when HB timer expired from sensor instance to framework
    Mar 24 2020 QC 2.46.0.2
     - Fixed issue of returning NULL instance for Orphan FLUSH
    Mar 26 2020 ST 2.46.0.3
     - No PCE for 0wmk/odr case(except passive/gated)
     - XSensor single output sensor updates
     - Send MD enable event only when applicable
     - Fix same ts for 1st sample issue
     - Include QC review comments for ST driver 2.46.0.2
    Apr 3 2020 ST 2.46.0.4
     - Bug fix for HA during MD + gated_flush_only + non-gated
    Apr 6 2020 QC 2.46.0.5
     - Bug fix on handling cal reset request to be marked with special pointer
    Apr 10 2020 ST 2.46.0.6
     - Bug fix for ts gap for 1st gyro sample
     - MD only case, send 0 ODR to DAE
    Apr 16 2020 QC 2.46.0.7
     - Remove TARGET NAMEs
    May 4 2020 ST 2.46.0.8
     - Support for IBI Async timing control mode 0
     - Updated temp sensor active current numbers
     - Optimize ibi irq timestamp calculation
    May 07 2020 QC 2.46.0.9
      - Return speical pointer upon handling flush request
    May 26 2020 QC 2.46.0.10 optimize comport open for instance_notify_event
    Jun 4 2020  QC 2.46.0.11 optimize comport open for set_client_config
    Jun 14 2020 QC 2.46.0.12
      - Do not forward FLUSH requests to DAE for passive only clients
    Jun 30 2020 ST 2.46.0.13
      - Fix for com_self_test failure due bus power changes
      - Fix for std-dev of accel when accel is introduced and odr changed
        while gyro is running
      - fix 1st gyro sample data 0 issue
    July 09 2020 ST 2.46.0.14
      -  Disable Accel LP mode when gyro is streaming
    July 09 2020 QC 2.46.0.15
      - Update range and resolution units for accel and gyro to be the same as
        the respective data units
    July 13 2020 ST 2.46.0.16
      - Fix publish sensor issue when accel introduced with gyro already
        streaming
    Aug 1 2020 ST 2.46.0.17
      - Gyro data gap issue
    Aug 11 2020 QC 2.46.0.18
      - No longer uses gcd()
    Aug 12 2020 ST 2.46.0.19
      - Fix PCE DAE watermark for gated request
      - Implement double tap sensor
      - Fix large timestamp diff issues when esp/xsensor client running in background
    Sept 3 2020 QC 2.46.0.20
     -Fix watermark calculation
    Sept 8 2020 QC 2.46.0.21
      - Fix for enabling fifo interrupt in DAE bypassed mode	 
    Sept 9 2020 ST 2.46.0.22
     - Implement QC review comment for 2.46.0.18
     - Fix PCE resolution issue for passive sensor
  ============================================================================*/

#ifndef _SNS_LSM6DSO_VER_H
#define _SNS_LSM6DSO_VER_H

// 32-bit version number represented as major[31:16].minor[15:8].rev[7:0]
#define LSM6DSO_MAJOR        2
#define LSM6DSO_MINOR       46
#define LSM6DSO_REV         22
#define SNS_VERSION_LSM6DSO  ((LSM6DSO_MAJOR<<16) | (LSM6DSO_MINOR<<8) | LSM6DSO_REV)

#endif //_SNS_LSM6DSO_VER_H

