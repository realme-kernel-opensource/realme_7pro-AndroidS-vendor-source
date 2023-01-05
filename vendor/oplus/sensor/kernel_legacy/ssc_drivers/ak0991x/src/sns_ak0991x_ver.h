#pragma once
/**
 * @file sns_ak0991x_ver.h
 *
 * Driver version
 *
 * Copyright (c) 2017-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017-2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

/**
 * EDIT HISTORY FOR FILE
 *
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 *
 * when         version    who              what
 * --------     --------   ----------       ---------------------------------
 * 02/03/20     026209     Qualcomm         Fixed dropped Mag sample when HW runs fast
 * 01/14/20     026208     Qualcomm         Fix for limiting number of samples to max fifo size
 * 11/01/19     026207     AKM              Fixed JIRA2458 Re-evaluate a configuration when two request are made
 * 10/23/19     026206     AKM              Fixed JIRA2467 Changed a calculation method of chosen_report_rate
 * 10/18/19                Qualcomm         Fixed JIRA-2547 - Increased buffer length for encoding timer request
 * 10/14/19     026205     Qualcomm         Fixed a bug that tries to enter I3C mode even though bus is I2C
 *                                          Removed chipset specific Bus and IRQ configuration 
 * 10/11/19     026204     AKM              Fixed JIRA2458 incorrect info on 2nd physical_config_event when two request are made.
 * 10/11/19                AKM              Fixed a bug that physical_config_event is sent before buffered data is flushed to clients
 * 10/09/19     026203     AKM              Fixed a bug that didn't flush after changing config
 * 10/03/19     026202     AKM              Do flush if recieved flush request during a config
 * 09/27/19     026201     Qualcomm/AKM     Remove instance correctly upon rejecting invalid request
 *                                          Do flush if recieved flush request during a clock error procedure
 * 09/19/19     026200     AKM              Fix the numbering format of version
 * 09/19/19                AKM              Modify to set has_water_mark=true and water_mark=1 if FIFO not in use
 * 09/18/19                AKM              Modify to set MAX and MIN ODR from registry
 *                                          Fixed a bug that it didn't have sample for fastest rate request after a lower request
 * 09/12/19                AKM              Fix to send correct attribute for send_config_event
 * 09/05/19     020060.6   AKM              Fix to send correct attribute strings of operation mode
 *                                          Fixed a bug that caused an UNRELIABLE event
 *                                          Fixed a bug that power rail timer was not deleted
 * 08/16/19     020060.5   Qualcomm         dont publish has_snc_ts_anchor for non s4s
 *                                          do not call flush_hw if wm=1
 * 08/14/19     020060.3   AKM              Modified DAE driver always num_samples=1 even no samples.
 * 08/01/19     020060.2   AKM              Use FIFO for AK09917 polling+nonFIFO mode
 * 07/22/19     020060.1   Qualcomm         Reject bad request and restore old request if any
 * 07/22/19     020060     AKM              Merged Qualcomm's 020060 modification.
 * 07/22/19                                 Merged Qualcomm's S4S codes and added the definition for compile.
 * 07/16/19                AKM              Do not flush hw if no fifo is used.
 * 07/16/19     020059     AKM              Modified for WaitForEvents error on MovingRates test of CTS.
 * 06/24/19     020058.2   Qualcomm         Merged Qualcomm's 020059 modification.
 *                                          Flush done event should be sent after flush completes
 * 06/19/19                Qualcomm/AKM     No sending flush_done while DAE flushing_data is true
 * 06/19/19                Qualcomm/AKM     S4S modification after review
 * 06/13/19     020058.1   AKM              Wait for stopping DAE stream before power rail off
 * 06/07/19     020058     AKM              First tested version with new QAWA(1.0.71.0)
 * 06/04/19                AKM              Modified for last flush num_samples. 
 * 05/31/19                AKM              Modified for S4S+DAE. Test version. 
 * 05/31/19                AKM              Use the timer with is_dry_run=true to sync DAE polling timing.
 * 05/31/19                Qualcomm/AKM     Fix for avoiding timer reg event read by heart beat timer 
 * 05/23/19                AKM              Use only dae_event_time for TS on polling with <= 50Hz ODR.
 *                                          Modified Config not changed judge.
 * 05/17/19                AKM              Removed sw reset for DAE when ODR changed. Adjusted polling_offset.
*  05/15/19     020057     Qualcomm         Fix for avoiding timer reg event read by heart beat timer 
 * 05/16/19     020056     AKM              modified for data_age_limit_ticks to use if-else
 * 05/14/19                Qualcomm         Sometimes, num_samples exceeding max fifo size and causing
                                            stack corruption(Stability issue)
 * 05/10/19                AKM              add dummy data when flush only request on DRI+FIFO+nonDAE
 * 05/09/19                AKM              modified for sending only config event when "Config not changed" on nonDAE.
 * 05/08/19                AKM              modified for sending cal event on self test for DAE.
 * 05/07/19                AKM              modified for negative timestamp when polling+DAE.
 * 05/03/19     020055     AKM              modified timestamp for config event
 * 04/29/19     020054     AKM              removed dummy on DRI+FIFO+nonDAE. removed care_fifo_buffer when ODR=0
 * 04/26/19     020053     AKM              fixed for bug to prevent UNRELIABLE data.
 * 04/25/19                AKM              use last_sent_cfg for judging "Config not changed"
 * 04/25/19                AKM              update last_sent_cfg and last_config_sent_time only when it is new config
 * 04/25/19                AKM              removed dummy add sequence from DAE
 * 04/24/19                AKM              calc_fifo_wmk bug fix No calculation when desired sample rate = 0
 * 04/23/19     020052     AKM              Modified ak0991x_get_decoded_mag_request for repoort_rate and flush_period
 * 04/22/19                AKM              Modified data_age_limit_ticks calculation
 * 04/22/19                AKM              Modified flush_period when flush_only and change parameter from sns_time from uint32_t
 * 04/22/19                AKM              Modified is_orphan decision without using time
 * 04/18/19     020051     AKM              Modified data_age_limit_ticks for MAG-051 in DAE+DRI+FIFO
 * 04/17/19                AKM              Refined HB timer
 * 04/16/19                AKM              Perform flush when Config not changed. (For MAG-052 in DAE+DRI+FIFO)
 * 04/16/19                AKM              Modified for negative latency when last flush in Polling+DAE.(MAG-059/062)
 * 04/15/19                AKM              Added dummy data when last flush and orphan. (For MAG-048 in DAE+DRI+FIFO)
 * 04/12/19                AKM              Changed to use AK0991X_INST_PRINT for LOW and MED level debug messages
 * 04/11/19                AKM              Modified for not to send flush done when UNRELIABLE data.
 * 04/11/19                AKM              Fixed for cal_event sending and timestamp.
 * 04/03/19     020050     AKM              Support 1Hz ODR for AK09915 and AK09917
 * 04/01/19     020049     AKM              Modified for last flush during ASCP(MAG-062), fifo_wmk calc bug for MAG-073
 * 03/25/19                AKM              Modified self test for MAG-213,221 and 223.
 * 03/22/19                AKM              Refactor for config. Created req_cfg.
 * 03/22/19                AKM              Modified for DRI+FIFO+nonDAE gap issue
 * 03/21/19     020048     AKM              Prevent to re-initialize average interval when ODR=0
 * 03/19/19                AKM              Prevent tx cal event calling when self test
 * 03/19/19                AKM/Qualcomm     Modified for using previous offset during UNKNOWN_DEVICE_MODE state
 * 03/14/19     020044.1   Qualcomm         Added AK0991X_UNKNOWN_DEVICE_MODE status for device mode
 * 03/14/19     020047     Qualcomm         Checked in
 * 03/12/19     020046     AKM              Fixed wm value when send_config_event in DAE
 * 03/12/19                AKM              add checking when the timestamp is newer than the dae_event_time
 * 03/12/19     020045     AKM              modified for rail timer
 * 03/08/19     020045     Qualcomm         Modified not to flush samples while still warming up
 * 02/22/19     020044     AKM              add dummy for DRI+FIFO+nonDAE when detect gap at last
 * 02/22/19                AKM              timestamp adjustment for flush only testing(MAG-048) on DRI+FIFO+nonDAE
 * 02/20/19                AKM              Modified to check update of dae_watermark.
 * 02/20/19                AKM              Modified not to change config by req_wmk on Non-DAE
 * 02/20/20                AKM              Added flush_done call in deinit for Non-DAE.
 * 02/14/19     020043     AKM              Use ideal timestamp for polling
 * 02/14/19                AKM
 * 02/14/19                AKM              Fixed driver version.
 * 02/11/19     020042     Qualcomm         Enabled IBI support.
 * 02/05/19                AKM              Use ideal time for Polling.
 * 02/05/19                AKM              Modified for sending flush_done with reliable mag data samples
 * 02/05/19                AKM              Added flush_done call in deinit for DAE
 * 01/23/19     020041     AKM/Qualcomm     Use SNS_ENABLE_DAE. Set 0us for TSU_STA time.
 * 01/17/19                AKM              Revert in ak0991x_enter_i3c_mode. Not return when dynamic address is assigned.
 * 01/10/19     020040     AKM              Fixed dummy data process on DAE
 * 01/09/19     020039     AKM              Added enter_i3c when bus power reset
 * 01/07/19     020038     AKM              Tested CTS: DRI+FIFO+DAE/non-DAE.
 * 01/04/19                AKM              Added Extra tSU_STA time for I3C.
 * 01/02/19                AKM/Qualcomm     Merged Qualcomm's 020036 modification for SM8250. Still testing.
 * 12/28/18                AKM              Check negative timestamp when flush requested in DAE+DRI.
 * 12/27/18                AKM              Added dummy data at the first data when detects gap.
 * 12/27/18                AKM              Check drifting timestamp for flush only tests on DRI.
 * 12/18/18                AKM              Modified for multiple orphan on DAE+Pollng.
 * 12/17/18     020037     AKM              Revert ak0991x_send_cal_event. CTS passed on DAE+Polling/DAE+DRI+FIFO.
 * 12/17/18                AKM              Modified timestamp when flush request on DAE+Polling
 * 12/17/18     020036     AKM              Modified num_samples after flush on DAE+Polling
 * 12/13/18                AKM              Modified dae_watermark when flush_only request.
 * 12/12/18                AKM              Modified calculate dae_watermark.
 * 12/12/18                AKM              Removed ak0991x_send_cal_event from ak0991x_send_config_event
 * 12/12/18                AKM              Added patch when received SNS_STD_ERROR_INVALID_STATE on DAE+Polling
 * 12/10/18                AKM              Added streaming check when polling+DAE
 * 12/10/18                AKM              Modified for SelfTest to streaming start when AK0991X_CONFIG_UPDATING_HW
 * 12/07/18                AKM              Added patch for jitter on Polling+DAE mode.
 * 12/07/18     020036     Qualcomm         Modified for SM8250 Defaults and DAE Bypass and I2C Mode crash fixes.
 * 12/07/18     020035     AKM              Modified for flush request in DAE.
 * 12/07/18                AKM              CTS passed on DAE+I3C+DRI+FIFO.
 * 12/07/18                AKM              CTS passed on DAE+I3C+Polling excepts testGeomagneticRotationVector_fastest. Timer timestamp has jitter.
 * 12/06/18                AKM              Add macro to use ideal timestamp for DAE+Polling.
 * 12/06/18     020034     AKM              CTS passed with DAE+Polling
 * 12/05/18                AKM              Modified flush request in DAE+Polling mode
 * 12/04/18                AKM              Use last_sw_reset_time to detect orphan batch.
 * 12/04/18                AKM              Added AK0991X_ENABLE_TIMESTAMP_TYPE
 * 11/30/18                AKM              Modified for orphan batch when interrupt detected.
 * 11/29/18                AKM              Use timestamp_type for detecting DRI or Flush in DAE mode.
 * 11/29/18     020033     AKM              Modified for flush requests on DAE.
 * 11/26/18     020032     AKM              Modified for AK09917 RevA/RevB bug and clock error procedure for DAE mode.
 *                                          Modified average interval calc for DRI on non DAE mode. Now it is same as DAE mode.
 * 11/19/18     020031     AKM              Regardless the DRDY status, set UNRELIABLE flag when XYZ data is all 0.
 * 11/16/18     020030     AKM              Modified for MAG039/MAG040 for DRI+FIFO mode
 * 11/15/18     020029     AKM              Modified not to use previous data. Added Re-check num_samples when DRDY when Polling ODR=0
 * 11/14/18     020028     AKM              Modified for current time < timestamp
 * 11/14/18     020027     AKM              Confirmed the 5Hz issue. Update the version.
 * 11/13/18                AKM              Test modification for AK09918C 5Hz issue
 * 11/08/18     020026     AKM/Qualcomm     AKM: Modified to use UNRELIABLE for the first data if data is not ready.
 *                                          Qualcomm fixed a handful of changes, in this version, made by AKM.
 * 10/29/18     020025     AKM              Debugged for very first data for Polling+DAE
 * 10/19/18     020024     AKM              Debugged for I2C mode at ak0991x_device_sw_reset
 * 10/19/18     020023     AKM              Merged Qualcomm 20021 and AKM 20023
 * 10/17/18                AKM              Implemented HB timer for Polling when DAE enabled.
 * 10/15/18     020022     AKM              Debugged for I2C+Polling+DAE mode.
 * 10/12/18     020021     AKM              HB timer perform while 100Hz dummy meas. Pause HB timer while self test.
 * 10/24/18     020023     Qualcomm         Do not add a cal reset request to the instance
 * 10/19/18     020022     Qualcomm         Retry enter i3c mode if fails
 * 10/16/18     020021     Qualcomm         Fixed setting of max batch, for max-batch = True and flush-only = True use case 
 * 10/12/18     020020     Qualcomm/AKM     Fixed setting of flush period for FlushOnly = True use case.
 * 09/26/18     020019     AKM              Merged Qualcomm 010017 and AKM 020018 and modified for WM
 * 09/25/18     020018     AKM              Modified for MAG221 with AK09917D RevA parts.
 * 09/17/18     020017     AKM              Modified for MAG023/MAG025/MAG027 timing error on DAE mode
 * 09/20/18     020017     Qualcomm         Fixed setting of FIFO watermark.
 *                                          Previous incorrect logic was causing garbage values to be reported
 *                                          in the extra data samples above the 25 max for AK09917
 * 09/10/18     020016     AKM              Merged Qualcomm's 020015 and AKM's 020015.
 * 09/06/18     020015     Qualcomm         Changed when to enter i3c
 * 09/03/18     020015     AKM              Modified for Dual Sensor on DAE
 * 08/03/18     020014     AKM              Debugged for the Klocwork P1 errors(#03603537)
 * 07/28/18     020013     Qualcomm         Send CFG Event for new request even no change 
 * 07/24/18     020013     AKM/Qualcomm     Enabled device mode as default and cleaned related code.
 * 07/12/18     020012     AKM/Qualcomm     Fixed compile error when DAE is enabled
 * 07/03/18     020011     AKM              Debugged when the registry access is disabled.
 * 06/28/18                Qualcomm         Retry 5 times if sw reset fails
 * 07/02/18     020010     AKM/Qualcomm     Modified for the upgrated LLVM
 * 06/27/18                AKM/Qualcomm     Merged Qualcomm's modification and AKM's 020009
 * 06/24/18     020009     AKM              Removed macros for SEE_LIET mode
 * 06/22/18                AKM              Refactor for the device_mode.
 * 06/20/18     020008     AKM/Qualcomm     Sometimes, mag is taking scp path with number of samples 32. 
 *                                          Changed local buffer size according to physical senosor
 *                                          Debugged mul-function when the AK0991X_FORCE_MAX_ODR_50HZ is set.
 *                                          Fixed watermark calculation for max batch
 *                                          Fixed flush only request handling
 *                                          Fixed wrong report rate calculation
 *                                          Removed odr < 100 limit for polling mode
 * 06/19/18     020007     AKM              Debugged compile error when dual sensor is enabled. Modified to ignore the irq time when WM!=num_samples.
 * 06/19/18     020006     AKM              Integrated the deltas between AKM's driver versions 77 and 80.
 *                                          AKM version 010080: Applied the device_mode modification from ver1.00.62F.
 *                                          AKM version 010079: Modified for device mode sensor and timestamp for DRI+FIFO mode.
 *                                          AKM version 010078: Modified contains mode check in HW self-test
 * 06/13/18     020005     Qualcomm         Integrated the deltas between AKM's driver versions 71 and 77.
 *                                          AKM version 010077:  Add continuous mode check in HW self-test
 *                                          AKM/QCOM version 010076: Added AK0991X_ENABLE_REG_FAC_CAL macro for reading 3x3 factory calibration parameter from registry
 *                                          AKM version 010075: Changed to read registry value for rail_vote when registry access is enabled
 *                                          AKM version 010074: Added sns_suid_lookup_deinit.
 *                                          AKM version 010073: Remove DC-Lib related code.
 *                                          AKM version 010072: Modified to set RAIL_ON_NPM by registry.
 * 05/16/18     020004     Qualcomm         Integrated versions 010063 to 010071
 * 05/10/18     020004     Qualcomm         Integrated versions 010059 to 010062
 * 04/20/18     020003     Qualcomm         Integrated versions 010057 and 010058
 * 04/20/18                Qualcomm         Fixed COM selftest; Fixed batching via DAE sensor
 * 04/20/18                Qualcomm         Added DAE WM to Config event
 * 03/16/18     020002     Qualcomm         DAE availability is discovered at boot
 * 03/07/18     020001     Qualcomm         Added I3C support
 * 03/02/18     020000     Qualcomm         Re-enabled streaming via DAE
 * 04/26/18     010071     AKM              Fixed error when ENABLE_DC is defined.
 * 04/20/18                AKM              Reduced parameters for SEE-Lite mode.
 * 04/20/18     010070     AKM              Modified for initialize use_dri,use_fifo,nsf and sdr.
 * 04/16/18     010069     AKM              Modified macro definition for non SEE_LITE mode. Removed AK0991X_ENABLE_ALL_DEVICES definition.
 * 04/13/18     010068     AKM              Fixed for SEE_LITE_MODE.
 * 04/12/18     010067     AKM              Added AK0991X_ENABLE_REG_WRITE_ACCESS macro
 * 04/12/18     010066     AKM              Modified for AK09918
 * 04/12/18                AKM              Modified for 0 gap detection on MAG-024/025/026 with S4S mode.
 * 04/12/18     010065     AKM              Modified for SEE_LITE_MODE again.
 * 04/12/18     010064     AKM              Modified for SEE_LITE_MODE.
 * 04/09/18     010063     AKM              Modified DAE settings.
 * 04/09/18                AKM              Fixed AK09915C/D read samples.
 * 04/11/18     010062     AKM              Protect several code for dual SI parameter by macro.
 * 04/03/18                AKM              Implement for handling dual SI parameter using device_mode_sensor.
 * 03/30/18     010061     AKM              Modified HB timer setting for crash when system is busy.
 * 03/21/18     010060     AKM              Fixed S4S to care timestamp.
 * 03/21/18                AKM              Added registry item version number. Case# 03380028
 * 03/12/18     010059     AKM              Fixed S4S settings.
 * 03/20/18                AKM              Debugged for HB timer on DRI mode.
 * 03/12/18                AKM              Fixed S4S settings.
 * 03/12/18                AKM              Change to read AKM's DC-parameter from registry file. Case# 03380124
 * 02/27/18     010058     AKM              Added unregister heart beat stream in ak0991x_stop_mag_streaming
 * 02/16/18     010057     AKM              Remove some comments after checked in.
 * 02/14/18     010056     Qualcomm/AKM     Qualcomm checked in version.
 * 02/14/18                Qualcomm/AKM     Fixed mag stops streams when system is busy and heartbeat timer expires
 * 02/14/18                AKM              Modified for Case# 03325581. Change to keep report rate equals max sample rate while streaming is running.
 * 02/14/18     010055     AKM              Change to remove timer_data_stream on each registering heart beat timer.
 * 01/24/18     010054     AKM              Modified for Case# 03314109. Always uses SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH for polling mode.
 * 01/18/18                AKM              Modified for Case# 03314109. Added checking ST1 status on polling mode.
 * 01/16/18     010053     AKM              Merged the version 010052 and self test bug modification
 * 01/04/18                AKM              Modified a self test bug
 * 01/11/18     010052     AKM              Modify to avoid sending ASCP request by heart beat timer
 * 01/09/18     010051     Qualcomm         No longer sending Config event when deleting
 * 01/09/18                Qualcomm         Fixed DAE flush commands
 * 12/25/17     010050     AKM              Modified for DC-Lib.
 * 12/25/17                AKM              Fixed comparison of the timestamp for heart_beat_timer_event.
 * 12/20/17     010049     Qualcomm/AKM     Merged the version 010048 and dual sensor modification
 * 12/18/17                Qualcomm/AKM     Fixed dual sensor for simultaneous streaming.
 * 12/11/17                AKM              Debugged for sending SNS_STD_MSGID_SNS_STD_FLUSH_EVENT
 * 12/20/17     010048     AKM              Featurization fix for Flush
 * 12/20/17                Qualcomm         Fixed issue of unexpected stopping of streaming
 * 12/20/17                Qualcomm         Fixed to stream instead of batch when requested rate is lower than MIN ODR
 * 12/20/17                Qualcomm         Always print ERROR messages
 * 11/30/17     010047     AKM              Apply 50Hz limitation to AK09915C/D and AK09917. Fixed for DRI+FIFO mode.
 * 11/30/17                Qualcomm         When not streaming on DAE only flushes when FIFO is in use
 * 11/21/17     010046     AKM              Use requested_timeout_time for Polling system_time.
 * 11/20/17     010045     AKM              Fixed averaged interval for Polling.
 * 11/20/17                AKM              Debugged limited to select 50Hz ODR in ak0991x_mag_match_odr()
 * 11/17/17                AKM              Added AK0991X_FORCE_MAX_ODR_50HZ macro.
 * 11/17/17     010043     AKM              Refactor for clock error measurement.
 * 11/17/17                AKM              Debugged for SEE-Lite compile.
 * 11/17/17                AKM              Fixed delayed Flush response
 * 11/17/17                AKM              Fixed self test
 * 11/17/17                AKM              Modified to use 2 x 100Hz DRI measurements to calculate the clock error
 * 11/17/17                Qualcomm         Fixed race condition in DAE interface module
 * 11/09/17     010042     AKM              Fine tuned average interval calculation
 * 11/09/17                Qualcomm         Changed to disallow Flush requests without Config requests
 * 11/09/17                Qualcomm         When DAE is unavailable reconfig HW after interrupt is ready
 * 11/07/17     010041     Qualcomm         Remove 1Hz ODR. Don't delete timer stream while processing it.
 * 11/04/17     010040     Qualcomm         Added Calibration event. 
 * 11/03/17     010039     AKM              Removed AK09917_REV_A flag. Calculate averaged_interval.
 * 11/03/17                Qualcomm         Fixed flush request handling during power up
 * 10/31/17     010038     AKM              Refactor to use ASCP in flush. Added AK09917_REV_A flag. 
 * 10/31/17                AKM              Added dual sensor support
 * 10/25/17     010037     AKM              Removed averaging filter for DRI mode
 * 10/23/17     010036     Qualcomm         Sends config event to new clients immediately if already streaming
 * 10/20/17     010035     AKM              Modified for SEE-Lite. 
 * 10/20/17                AKM              Fixed negavite timestamp intervals
 * 10/19/17     010034     Qualcomm/AKM     Debugged timestamp issue. Added 1[sec] delay power rail when off. Removed GPIO check.
 * 10/18/17                AKM(M)           Modified for SEE-Lite except using new SUID handler utility
 * 10/18/17     010033     AKM(N)           Added heart beat timer function
 * 10/16/17     010032     AKM(M)           Supports flush_only.
 * 10/16/17     010031     AKM(M)           Debugged negative timestamp when DRI+FIFO/Polling+FIFO.
 * 10/13/17                AKM(M)           Changed S4S name and debugged for negative timestamp when irq->flush->acsp.
 * 10/12/17     010030     AKM(M)           Debugged of the ak0991x_get_adjusted_mag_data.
 * 10/12/17     010029     AKM(M)           Modified for the timestamp in the Polling and FIFO+Polling mode.
 * 10/12/17                AKM(N+M)         Removed duplicate functions/code paths.
 * 10/12/17                AKM(N)           Separated file for S4S.
 * 09/29/17     010028     Qualcomm/AKM     Debugged for negative timestamp issues.
 * 09/26/17     010027     Qualcomm/AKM     Merged Qualcomm's 010020 and AKM's 010026.
 * 09/24/17     010020     Qualcomm         Re-enable FIFO, ts fixes.
 * 09/22/17     010019     Qualcomm         Disable S4S, FIFO temporarily due to ts issues
 * 09/22/17     010026     Qualcomm/AKM     Added this_is_first_data=true when ak0991x_start_mag_streaming() called.
 * 09/20/17                AKM(M)           Modified debug messages(ERROR->LOW, uses %u for timestamp).
 * 09/19/17                AKM(M)           Modified debug messages.
 * 09/19/17     010025     AKM(M)           Added GPIO checking for the DRI interrupt.
 * 09/18/17     010024     AKM(M)           Modified for the timestamp in the FIFO mode
 * 09/14/17     010023     AKM(N+M)         Added compile switches for FIFO/DRI/FUSE and modified for issue#5/#23
 * 09/13/17     010022     AKM(M)           Modified check DRDY bit to ignore wrong notify event call
 * 09/13/17     010021     AKM(M)           Cont. Modified for SEE-Lite mode. Disabled registry. Not finished yet.
 * 09/09/17     010020     AKM(M)           Cont. Modified for SEE-Lite mode. Not finished yet.
 * 09/08/17     010019     AKM(M)           Modified for SEE-Lite mode. Not finished yet.
 * 09/08/17     010018     AKM(M)           Added to check DRDY bit to ignore wrong notify_event call
 * 09/07/17     010017     AKM(M)           enabled AK0991X_DAE_FORCE_NOT_AVAILABLE for second test run
 * 09/07/17     010016     AKM(M)           Merged 010014 and 010015
 * 09/07/17     010014     AKM              Update S4S in non-DAE and DAE.
 * 09/05/17     010015     AKM(M)           Modified filter for DRI/FIFO mode.
 * 09/05/17     010014     AKM(M)           Added filter for DRI mode(won't work with FIFO/Polling modes)
 * 09/05/17     010013     AKM              2nd Support S4S.
 * 09/01/17     010012     AKM              1st Support S4S.
 * 08/25/17     010011     Qualcomm         Work on OpenSSC v5.0.5.
 * 08/10/17     010010     AKM              Modify for prevention of duplicate interrupts using DRDY bit status and timestamp.
 * 08/07/17     010009     AKM              Modify for prevention of duplicate interrupts using DRDY bit status.
 * 08/04/17     010008     AKM              Modify for prevention of duplicate interrupts.
 * 07/10/17     010007     AKM              Modify for self-test which cares streaming.
 * 07/03/17     010006     AKM              Support FIFO+Polling mode.
 * 06/22/17     010005     Qualcomm/AKM     Support COM/HW self-test.
 * 06/22/17                Qualcomm/AKM     Re-do island refactoring.
 * 06/19/17     010004     Qualcomm/AKM     Fix to work on 845 platform.
 * 06/19/17                Qualcomm/AKM     Fix DAE sensor.
 * 06/19/17                Qualcomm/AKM     Fix mag streaming after flush.
 * 06/19/17                Qualcomm/AKM     Fix ODR sweep bugs.
 * 06/19/17                Qualcomm/AKM     Support registry.
 * 06/13/17     010003     Qualcomm         Work on OpenSSC v5.0.4.
 * 05/11/17     010002     AKM              Add AK09917D support.
 * 05/11/17                AKM              Add island mode support.
 * 05/11/17                AKM              Add DAE sensor support.
 * 04/04/17     010001     AKM              Fix IRQ configuration.
 * 04/04/17                AKM              Fix ODR attribute configuration.
 * 02/20/17     010000     AKM              First version.
 *
 **/

// major:02 minor:62 revision:07
#define AK0991X_DRV_VER_MAJOR    2
#define AK0991X_DRV_VER_MINOR    62
#define AK0991X_DRV_VER_REV      9
#define AK0991X_DRIVER_VERSION ( (AK0991X_DRV_VER_MAJOR<<16) | (AK0991X_DRV_VER_MINOR<<8) | AK0991X_DRV_VER_REV )
