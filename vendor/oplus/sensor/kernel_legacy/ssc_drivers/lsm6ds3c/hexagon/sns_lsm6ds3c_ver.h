/*******************************************************************************
* Copyright (c) 2018-2019, STMicroelectronics.
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
      - Align Driver with LSM6DS3C_SEE_V1.29.0
    Nov 1 2017 ST - 1.6.0
      - MD support
    Nov 3 2017 ST - 1.7.0
      - Align Driver with LSM6DS3C_SEE_V1.38.0
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
      - This release has MD updated to match LSM6DS3C driver
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
    Aug 08 2018 QC - 2.20.0
      - Made unreliable sample as is ( reverted from 0 )
    Aug 27 2018 ST/QC - 2.21.0
      - Added sanity test debug messages
      - Fixed various config event issues
      - Dual Sensor support for DAE-enabled config
    Aug 30 2018 ST/QC - 2.22.0
      - Fixed QC tracker related to Temperature(#10),MD physical config (#16)
      - Fixed issues related to Self Test
      - Fixed issues related to ESP
    Aug 31 2018 ST - 2.23.0
      - update timestamp logic for DAE
      - unify timestamp logic for DAE and DAE bypass mode
      - Fixed timestamp gap issues in DAE mode (Tkr 12)
      - update build config for 855
    Sep 10 2018 ST - 2.24.0
      - Enable timestamp correction function in DAE path
    Sep 10 2018 QC - (2.22.0)
      - Changed how MIN and MAX ODRs are selected
    Sep 13 2018 QC - 2.25.0
      - Merged QC version 2.22.0 with ST version 2.24.0
      - Changed to update heartbeat timer when changing DAE WM without changing FIFO WM
    Sep 14 2018 ST - 2.26.0
      - Fix -ve timestamp issue. trk-9
      - Other minot fixes
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
    Nov 09 2018 QC - 2.34.0.1
      - Consolidated get_sample_interval()
    Nov 10 2018 QC - 2.34.0.2
      - Overwrite un-reliable samples with last reliable sample,
        when there is no change in client configuration
    Nov 15 2018 QC - 2.34.0.3
      - Cleaned up FIFO WM and DAE WM calculations
    Dec 05 2018 QC - 2.34.0.4
      - Added Rigid Body Attribute to Motion Detect and ACCEL publish Rigid Body to DAE sensor
    Dec 06 2018 QC - 2.35.0.0
      - One shot implementation
      - Use attempts number instead of 10ms(fixed value) for SW reset
      - reset added for ascp_req for error return types
      - reset only accel/gyro if reconfig is not required
      - Fix heart attack issue in DAE mode after reconfig
    Jan 10 2019 ST/QC 2.37.0.1
      - Fix timestamp latency issue
      - Fix driver logic when DAE is dropping data when FLUSH_ONLY
      - Cleanup config_sensor and odr_change logic
    Jan 21 2019 ST 2.37.0.2
      - Fix crash issue when DAE does not flush HW FIFO
    Feb 14 2019 ST/QC 2.37.0.3
      - DAE report rate set to zero for flush-only/max-batch
      - Flushes samples when increasing DAE WM
      - Reduce delay in reconfiguration
      - Fix issue with last_ts invalid
    Apr 1 2019 ST/QC 2.37.0.5
      - Reduce MD filter settling time.
      - Remove wrong reset of interrupt count when interrupt is settled
      - Update physical config event with current time for MD sensro(QC change)
      - DAE change to make sure actual report rate is at least 90% of the requested rate(QC change)
    Apr 18 2019 ST 2.37.0.6
      - Start averaging after IRQ is stable
    May 31 2019 ST 2.37.0.7
      - fix issue with device suspend test
      - don't send accel sample for gated client
    Jun 21 2019 ST/QC 2.37.0.8
      - timestamp calculation updated
      - MD logic updated
      - add registry read/write nominal ratios
    July 16 2019 ST 2.37.0.9
      - fix heart attack handling esp and md issue
      - fix step counter while power rail always being on
      - revert rounding source read
    July 24 2019 ST 2.37.0.10
      - Restores existing request if rejecting its replacement
      - fix the type cast build compilation issue
    July 31 2019 ST 2.43.0.0
      - Timestamp: Updated TS logic to store and use nominal sampling interval specific for the device.
      - Timestamp: Other timestamp fixes for latency, accuracy and flush use-cases.
      - MD related fixes for interrupt, events and heart attack handling
      - Update ESP sensor for interrupt and heart attack handling
      - Pattern position check fix for DAE mode
      - Update to use correct filter delay and ODR
      - Self-test related updates
      - Config event related updates for framework requirements
      - Other minor fixes
    Sept 10 2019 ST 2.43.0.7
      - Reset Full-Scale to registry values when self-test is complete
      - Add option to adjust sensitivity
      - fix md handling while self-test alive
      - send intr_is_cleared_msg after handling esp intr
      - Integrate QC changaes for 2.43.0.3 to fix wrong data type used in
        heart beat_timer timeout calculation
      - Integrate ST 2.43.0.4 to fix timestamp synchronization issue
      - Integrate QC 2.43.0.5 Fix for interrupt registration event drop
      - Integrate QC 2.43.0.7 changes to restart DAE streaming on interrupt event
    Nov 15 2019 ST 2.43.0.8
      - fix num_sample value return issue
    Dec 10 2019 ST 2.43.0.9
      - discord sample fix
      - max batch fix
      - fix nominal ratio calculation
    Dec 12 2019 ST 2.45.0.11
      - Remove LSM5DS3 support and sorted data structure by size as per Qualcomm review comments
      - update interrupt count if it is first interrupt and Inbound check fails
      - send interrupt clear msg during heart attack usecase
      - update for ignoring heart attack handling
      - Do not skip flush even if wmk=1
      - update interrupt ts if flush and interrupt comes closer
      - Bug Fix for missing MD disable event to FW
      - Include Qualcomm Max batch changes
      - Create MD and heart-beat timers only one time
      - fix avg_to_nominal_ratio calculation
      - fix factory-test zero variance issue
      - Include QC 2.45_x changes
      - Send error event when invalid request is send
      - use gcd only incase of dae mode
      - Fixed issues relates to new framework requirement where all physical config events belonging to same
        stream should have the same timestamp unless the physical config event changes
      - During heart attack, read src registers to clear interrupt anyway
    Jan 3 2020 ST 2.45.0.12
      - Update new_config timestamp structure for md physical config event while enabling md
    Jan 22 2020 ST 2.45.0.13
      - Reset the device on every boot
      - Send config event with 0 sample rate for gated request
      - Latency Fixes
      - Fix gated accel heart attack issue
    Feb 19 2020 ST 2.45.0.14
      - Initialize the cal event timestamp when regsitry is disabled
      - Discard the samples when factory test completes
      - Allocate raw log only if the driver is sending raw logs
      - Include QC mainlined changes from 2.45.0.13
  ============================================================================*/

#ifndef _SNS_LSM6DS3C_VER_H
#define _SNS_LSM6DS3C_VER_H

// 32-bit version number represented as major[31:16].minor[15:0]
#define LSM6DS3C_MAJOR        2
#define LSM6DS3C_MINOR       45
#define LSM6DS3C_REV         14
#define SNS_VERSION_LSM6DS3C  ((LSM6DS3C_MAJOR<<16) | (LSM6DS3C_MINOR<<8) | LSM6DS3C_REV)

#endif //_SNS_LSM6DS3C_VER_H

