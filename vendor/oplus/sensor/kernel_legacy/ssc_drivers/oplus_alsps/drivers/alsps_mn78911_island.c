#ifdef SUPPORT_MN78911
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

#include "alsps_mn78911.h"
#include "sns_alsps_sensor.h"
#include "oplus_alsps.h"

static mn78_optical_sensor mn_sensor;
#define PS_POLLING_MODE     0
#define ALS_POLLING_MODE    0
#define ALS_FAST_UPDATE     1
bool init_flag = true;

bool als_fac_flag;

int als_dynamic_intt_intt[] = {MN_ALS_INTT_24576, MN_ALS_INTT_12288, MN_ALS_INTT_6144};
int als_dynamic_intt_value[] = {24576, 12288, 6144}; //match als_dynamic_intt_intt table
int als_dynamic_intt_gain0[] = {MN_GAIN0_MID, MN_GAIN0_LOW, MN_GAIN0_LOW};
int als_dynamic_intt_gain1[] = {MN_GAIN1_HIGH, MN_GAIN1_MID, MN_GAIN1_LOW};
int als_dynamic_intt_cycle[] = {MN_CYCLE_2, MN_CYCLE_8, MN_CYCLE_16};
int als_dynamic_intt_enh[] = {MN_ALS_ENH_MODE_2, MN_ALS_ENH_MODE_1, MN_ALS_ENH_MODE_1};
int als_dynamic_intt_high_thr[] = {60000, 60000, 65535};
int als_dynamic_intt_low_thr[] = {0, 200, 200};
int als_dynamic_intt_rs[] = {MN_RS_8, MN_RS_4, MN_RS_0}; //2^n
int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_value)/sizeof(int);
bool als_first_flag;
bool ag_chg_flag;
uint8_t ag_last_stage = 0;
uint16_t temp_offset = 0;

typedef enum
{
    CMC_BIT_RAW   			= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
} CMC_ALS_REPORT_TYPE;

int mn78_als_intt_value[] = {48, 96, 192, 384, 576, 768, 1152, 1536, 2304, 3072, 4608, 6144, 9216, 12288, 18432, 24576};
int mn78_als_rs_value[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};
int mn78_rs_num = sizeof(mn78_als_rs_value)/sizeof(int);
int mn78_ps_intt_value[] = {8, 16, 24, 32, 48, 80, 144, 208, 272, 336, 400, 464, 528, 656, 784, 1040};
int mn78_cycle_value[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
int mn78_wait_value[] = {0, 2, 4, 8, 12, 20, 30, 40, 50, 75, 100, 150, 200, 300, 400};
/******************************************************************************
 *  ALS Light source
 ******************************************************************************/
int offset_gain;
int scale_gain;
uint32_t lsrc_als_offset = 0;
uint16_t lsrc_raw = 0;
uint32_t lsrc_lux = 0;
uint32_t lsrc_ratio = 0;
//uint16_t lsrc_w_thd = 0;
//uint16_t lsrc_m_thd = 0;

/******************************************************************************
 *  ALS_MAX_LUX_PATCH
 ******************************************************************************/
uint8_t als_nor_intt;
uint8_t als_nor_gain0;
uint8_t als_nor_gain1;
uint8_t als_nor_cycle;
uint8_t als_max_intt;
uint8_t als_max_gain0;
uint8_t als_max_gain1;
uint8_t als_max_cycle;
uint32_t als_max_lux_condiition;
uint16_t lpc_gain;
uint32_t als_lux_revocer_condiition;
bool als_max_lux_flag;
bool als_max_change_flag;

/******************************************************************************************************************/
#define LOG_TAG "[MN78xxx]"

#define I2C_RETRY   3

void mn78_sensor_dump_reg(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle);

static sns_rc mn_sensor_I2C_Write(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint32_t reg_addr, uint8_t buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint32_t xfer_bytes, k=0;
    sns_port_vector port_vec;
    port_vec.buffer = &buffer;
    port_vec.bytes = 1;
    port_vec.is_write = true;
    port_vec.reg_addr = reg_addr;

    for (k=0;k<I2C_RETRY; k++) {
        rc = scp_service->api->sns_scp_register_rw(port_handle, &port_vec, 1, false, &xfer_bytes);
        if (rc == SNS_RC_SUCCESS)
            break;
    }
    return rc;
}

static sns_rc mn_sensor_I2C_Write_Block(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint32_t reg_addr, uint8_t *buffer, uint32_t bytes)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint32_t xfer_bytes, k=0;
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = true;
    port_vec.reg_addr = reg_addr;

    for (k=0;k<I2C_RETRY; k++) {
        rc = scp_service->api->sns_scp_register_rw(port_handle, &port_vec, 1, false, &xfer_bytes);
        if (rc == SNS_RC_SUCCESS)
            break;
    }

    return rc;
}

static sns_rc mn_sensor_I2C_Read(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint32_t reg_addr, uint32_t bytes, uint8_t *buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint32_t xfer_bytes, k=0;
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = false;
    port_vec.reg_addr = reg_addr;

    for (k=0;k<I2C_RETRY; k++) {
        rc = scp_service->api->sns_scp_register_rw(port_handle, &port_vec, 1, false, &xfer_bytes);
        if (rc == SNS_RC_SUCCESS)
            break;
    }

    return rc;
}
/******************************************************************************************************************/
//sensing time
static int als_sensing_time(int intt, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_cycle;
    int i = 0, enh_value = 1;

    als_intt = mn78_als_intt_value[intt>>4];
    als_cycle = mn78_cycle_value[cycle];

    for(i=0; i<(mn_sensor.als.enh_mode) ; i++)
        enh_value *= 2;
    sensing_us_time = (enh_value*als_intt + 76) * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;
    return (sensing_ms_time + 10);
}

static int ps_sensing_time(int intt, int pulse)
{
    long sensing_us_time;
    int sensing_ms_time;
    int ps_intt, ps_pulse = 1;
    int i = 0, enh_value = 1;

    ps_intt = mn78_ps_intt_value[intt>>4];
    for(i=0; i<(pulse) ; i++)
        ps_pulse *= 2;
    for(i=0; i<(mn_sensor.ps.enh_mode) ; i++)
        enh_value *= 2;
    sensing_us_time = (enh_value*ps_intt +  76) * 2 * ps_pulse;
    sensing_ms_time = sensing_us_time / 1000;
    return (sensing_ms_time + 10);
}

static sns_rc set_psensor_intr_threshold(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint16_t pilt, uint16_t piht)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t thd[4];

    thd[0] = (uint8_t) (pilt & 0x00ff);
    thd[1] = (uint8_t) (pilt >> 8);
    thd[2] = (uint8_t) (piht & 0x00ff);
    thd[3] = (uint8_t) (piht >> 8);

    rc = mn_sensor_I2C_Write_Block( scp_service, port_handle, DEVREG_PS_ILTL, thd, 4);
    OPLUS_ALS_PS_LOG("[set_psensor_intr_threshold] - rc=%d, low_thd = %d, high_thd = %d \n", rc, pilt, piht);
    return rc;
}

static sns_rc set_lsensor_intr_threshold(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint16_t ailt, uint16_t aiht)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t thd[4];

    thd[0] = (uint8_t) (ailt & 0x00ff);
    thd[1] = (uint8_t) (ailt >> 8);
    thd[2] = (uint8_t) (aiht & 0x00ff);
    thd[3] = (uint8_t) (aiht >> 8);

    rc = mn_sensor_I2C_Write_Block( scp_service, port_handle, DEVREG_ALS_ILTL, thd, 4);
    OPLUS_ALS_PS_LOG("[set_lsensor_intr_threshold] - rc=%d, low_thd = %d, high_thd = %d \n", rc, ailt, aiht);
    return rc;
}

static sns_rc set_ag_threshold(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint32_t reg_addr, uint16_t ag_thd)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t thd[2];

    thd[0] = (uint8_t) (ag_thd & 0x00ff);
    thd[1] = (uint8_t) (ag_thd >> 8);

    rc = mn_sensor_I2C_Write_Block( scp_service, port_handle, reg_addr, thd, 2);
    OPLUS_ALS_PS_LOG("[set_ag_threshold] - rc=%d, REG0x%x: ag_thd=%d \n", rc, reg_addr, ag_thd);
    return rc;
}

static void select_pixel_scale(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    uint8_t pxl_scale[2];
    if( (mn_sensor.als.als_pixel_sel==PIXEL_IR0A0B_ALS) || (mn_sensor.als.als_pixel_sel==PIXEL_CLR_ALS) )
    {//ch0:4x, ch1:2x
        pxl_scale[0] = 0x01;
        pxl_scale[1] = 0x8A;
    }
	else if(mn_sensor.als.als_pixel_sel==PIXEL_IR0A0B_CLR)
	{//ch0:4x, ch1:4x, 0.8725
		pxl_scale[0] = 0x05;
        pxl_scale[1] = 0xAA;
	}
	else if(mn_sensor.als.als_pixel_sel==PIXEL_IR0A0B_ALS_CLR)
	{//ch0:4x, ch1:4x, 1.375
		pxl_scale[0] = 0x05;
        pxl_scale[1] = 0x6A;
	}
	else
	{
		pxl_scale[0] = 0x00;
        pxl_scale[1] = 0x8A;
	}
    mn_sensor_I2C_Write_Block( scp_service, port_handle, 0x14, pxl_scale, 2);
    OPLUS_ALS_PS_LOG("[select_pixel_scale] - scale=0x%x, factor=0x%x \n", pxl_scale[0], pxl_scale[1]);
}
//write_global_variable
static sns_rc mn78_sensor_write_global_variable(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t buf_block[7];

    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RESET | MN_UN_LOCK) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RESET | MN_UN_LOCK) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );

    mn_sensor_I2C_Write( scp_service, port_handle, 0xF0, 0x8E ); //unkey
    mn_sensor_I2C_Write( scp_service, port_handle, 0xF4, 0x21 ); // chip refresh
    sns_busy_wait(sns_convert_ns_to_ticks(5*1000*1000));
    mn_sensor_I2C_Write( scp_service, port_handle, 0xF4, 0x20 ); // chip refresh
	//mn_sensor_I2C_Write( 0xF7, 0x41 ); // EN_VOS off
    mn_sensor_I2C_Write( scp_service, port_handle, 0xF0, 0x00 ); //key
	mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET) );

    //mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_OFSL, (uint8_t)(mn_sensor.ps.ps_offset& 0xff) );
    //mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_OFSH, ((mn_sensor.ps.ps_offset & 0xff00) >> 8) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_OFSL, (uint8_t)(mn_sensor.als.als_offset& 0xff) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_OFSH, ((mn_sensor.als.als_offset & 0xff00) >> 8) );

    select_pixel_scale(scp_service, port_handle);
    set_psensor_intr_threshold( scp_service, port_handle, mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
    set_lsensor_intr_threshold( scp_service, port_handle, mn_sensor.als.low_threshold, mn_sensor.als.low_threshold);
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_INT_CTRL, (mn_sensor.als.als_ag_en | mn_sensor.interrupt_control | mn_sensor.wait_clk | mn_sensor.wait_clk | mn_sensor.als.als_ag_ch0sat_en | mn_sensor.als.als_ag_3stage) );

    //pixel selection
    buf_block[0] = (mn_sensor.als.dk_period_en | mn_sensor.als.als_pixel_mode | mn_sensor.als.dk_period);
    buf_block[1] = mn_sensor.als.als_pixel_sel;
    buf_block[2] = (mn_sensor.ps.ps_ir0b23_sel | mn_sensor.ps.ps_ir3_en | mn_sensor.ps.ps_ir2_en | mn_sensor.ps.ps_ir1_en | mn_sensor.ps.ps_ir0b_en | mn_sensor.ps.ps_ir0a_en);
    mn_sensor_I2C_Write_Block( scp_service, port_handle, DEVREG_DARK_FRMAE, buf_block, 3);

    //als config
    buf_block[0] = mn_sensor.als.integration_time | mn_sensor.als.als_gain1 | mn_sensor.als.als_gain0;
    buf_block[1] = mn_sensor.als.interrupt_channel_select | mn_sensor.als.als_std | mn_sensor.als.cycle;
    buf_block[2] = mn_sensor.als.als_rs | mn_sensor.als.persist | mn_sensor.als.interrupt_type;
    buf_block[3] = mn_sensor.als.als_cal_mode | mn_sensor.als.ma_en | mn_sensor.als.enh_mode;
    mn_sensor_I2C_Write_Block( scp_service, port_handle, DEVREG_ALS_CONFIG, buf_block, 4);

    //ps config
    buf_block[0] = mn_sensor.ps.integration_time | mn_sensor.ps.ps_gain1 | mn_sensor.ps.ps_gain0;
    buf_block[1] = mn_sensor.ps.interrupt_channel_select | mn_sensor.ps.ps_std | mn_sensor.ps.ps_pulse;
    buf_block[2] = mn_sensor.ps.ps_avg | mn_sensor.ps.persist | mn_sensor.ps.interrupt_type;
    buf_block[3] = mn_sensor.ps.enh_mode;
    buf_block[4] = mn_sensor.ps.ir_on_control | mn_sensor.ps.ir_drive_tune | mn_sensor.ps.ir_drive;
    mn_sensor_I2C_Write_Block( scp_service, port_handle, DEVREG_PS_CONFIG, buf_block, 5);

    buf_block[0] = mn_sensor.als.als_aenh_l | mn_sensor.als.als_aintt_l;
    buf_block[1] = mn_sensor.als.als1_ag_l | mn_sensor.als.als0_ag_l | mn_sensor.als.als_acycle_l;
    buf_block[2] = mn_sensor.als.als_aenh_m | mn_sensor.als.als_aintt_m;
    buf_block[3] = mn_sensor.als.als1_ag_m | mn_sensor.als.als0_ag_m | mn_sensor.als.als_acycle_m;
    buf_block[4] = mn_sensor.als.als_aenh_h | mn_sensor.als.als_aintt_h;
    buf_block[5] = mn_sensor.als.als1_ag_h | mn_sensor.als.als0_ag_h | mn_sensor.als.als_acycle_h;
    mn_sensor_I2C_Write_Block( scp_service, port_handle, DEVREG_ALS_AG_CFG_L, buf_block, 6);
    set_ag_threshold( scp_service, port_handle, DEVREG_ALS_AG_H2M_THL, mn_sensor.als.als_ag_h2m_thd);
    set_ag_threshold( scp_service, port_handle, DEVREG_ALS_AG_M2H_THL, mn_sensor.als.als_ag_m2h_thd);
    set_ag_threshold( scp_service, port_handle, DEVREG_ALS_AG_M2L_THL, mn_sensor.als.als_ag_m2l_thd);
    set_ag_threshold( scp_service, port_handle, DEVREG_ALS_AG_L2M_THL, mn_sensor.als.als_ag_l2m_thd);
    if(mn_sensor.als.als_ag_3stage == MN_AG_3STG_DIS)
    {
        buf_block[0] = mn_sensor.als.als_aenh_ll | mn_sensor.als.als_aintt_ll;
        buf_block[1] = mn_sensor.als.als1_ag_ll | mn_sensor.als.als0_ag_ll | mn_sensor.als.als_acycle_ll;
        mn_sensor_I2C_Write_Block( scp_service, port_handle, DEVREG_ALS_AG_CFG_LL, buf_block, 2);
        set_ag_threshold( scp_service, port_handle, DEVREG_ALS_AG_L2LL_THL, mn_sensor.als.als_ag_l2ll_thd);
        set_ag_threshold( scp_service, port_handle, DEVREG_ALS_AG_LL2L_THL, mn_sensor.als.als_ag_ll2l_thd);
    }
    //set mode and wait
    if(mn_sensor.mode == MN_MODE_PS)
        rc = mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, (MN_WAIT_200_MS | mn_sensor.mode) );
    else
        rc = mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, (mn_sensor.wait | mn_sensor.mode) );
    return rc;
}

static void mn78_sensor_als_ag_calc_new_thd(void)
{
	int idx=0, i=0, j=0, gain_value=0, intt_value=0, enh_value=1, enh_temp=0, total_value=0;
	als_dynamic_intt_gain0[0] = (mn_sensor.als.als0_ag_h>>4);
    als_dynamic_intt_gain0[1] = (mn_sensor.als.als0_ag_m>>4);
    als_dynamic_intt_gain0[2] = (mn_sensor.als.als0_ag_l>>4);
    als_dynamic_intt_gain1[0] = (mn_sensor.als.als1_ag_h>>4);
    als_dynamic_intt_gain1[1] = (mn_sensor.als.als1_ag_m>>4);
    als_dynamic_intt_gain1[2] = (mn_sensor.als.als1_ag_l>>4);
    als_dynamic_intt_intt[0] = (mn_sensor.als.als_aintt_h<<4);
    als_dynamic_intt_intt[1] = (mn_sensor.als.als_aintt_m<<4);
    als_dynamic_intt_intt[2] = (mn_sensor.als.als_aintt_l<<4);
    als_dynamic_intt_value[0] = mn78_als_intt_value[(als_dynamic_intt_intt[0]>>4)];
    als_dynamic_intt_value[1] = mn78_als_intt_value[(als_dynamic_intt_intt[1]>>4)];
    als_dynamic_intt_value[2] = mn78_als_intt_value[(als_dynamic_intt_intt[2]>>4)];
    als_dynamic_intt_cycle[0] = mn_sensor.als.als_acycle_h;
    als_dynamic_intt_cycle[1] = mn_sensor.als.als_acycle_m;
    als_dynamic_intt_cycle[2] = mn_sensor.als.als_acycle_l;
    als_dynamic_intt_enh[0] = (mn_sensor.als.als_aenh_h>>4);
    als_dynamic_intt_enh[1] = (mn_sensor.als.als_aenh_m>>4);
    als_dynamic_intt_enh[2] = (mn_sensor.als.als_aenh_l>>4);
	als_dynamic_intt_high_thr[0] = mn_sensor.als.als_ag_h2m_thd;
	als_dynamic_intt_high_thr[1] = mn_sensor.als.als_ag_m2l_thd;
	als_dynamic_intt_low_thr[1] = mn_sensor.als.als_ag_m2h_thd;
	als_dynamic_intt_low_thr[2] = mn_sensor.als.als_ag_l2m_thd;
	for(i = 0; i < (als_dynamic_intt_intt_num-1); i++)
	{
		if(als_dynamic_intt_gain1[i] == MN_GAIN1_HH)
			gain_value = 64;
		else if(als_dynamic_intt_gain1[i] == MN_GAIN1_HIGH)
			gain_value = 32;
		else if(als_dynamic_intt_gain1[i] == MN_GAIN1_MID)
			gain_value = 8;
		else
			gain_value = 1;

		if(als_dynamic_intt_gain1[als_dynamic_intt_intt_num-1] == MN_GAIN1_HH)
			gain_value /= 64;
		else if(als_dynamic_intt_gain1[als_dynamic_intt_intt_num-1] == MN_GAIN1_HIGH)
			gain_value /= 32;
		else if(als_dynamic_intt_gain1[als_dynamic_intt_intt_num-1] == MN_GAIN1_MID)
			gain_value /= 8;

		enh_value = 1;
		for(j=0; j<(als_dynamic_intt_enh[i]) ; j++)
			enh_value *= 2;
		enh_temp = 1;
		for(j=0; j<(als_dynamic_intt_enh[als_dynamic_intt_intt_num-1]) ; j++)
			enh_temp *= 2;
		enh_value = enh_value/enh_temp;

		intt_value = als_dynamic_intt_value[i] / als_dynamic_intt_value[(als_dynamic_intt_intt_num-1)];
		total_value = gain_value * intt_value * enh_value;
		for(idx = 0; idx < mn78_rs_num;  idx++)
		{
			if(total_value < mn78_als_rs_value[idx])
			{
				break;
			}
		}
		OPLUS_ALS_PS_LOG("[mn78_sensor_als_ag_calc_new_thd]: idx=%d, mn_als_rs_value=%d, total_value=%d, enh_value=%d\r\n", idx-1, mn78_als_rs_value[idx-1], total_value, enh_value);
		als_dynamic_intt_rs[i] = ((idx-1)<<4);
		als_dynamic_intt_high_thr[i] = als_dynamic_intt_high_thr[i]/total_value;
		als_dynamic_intt_low_thr[i] = als_dynamic_intt_low_thr[i]/total_value;
		OPLUS_ALS_PS_LOG("[mn78_sensor_als_ag_calc_new_thd]: als_dynamic_intt_low_thr[%d]=%d, als_dynamic_intt_high_thr[%d]=%d\r\n", i, als_dynamic_intt_low_thr[i], i, als_dynamic_intt_high_thr[i]);
	}
}
static void mn78_sensor_read_renvo(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    uint8_t rx_buf[2] = {0};
    mn_sensor_I2C_Read(scp_service, port_handle, DEVREG_REV_ID, 2, rx_buf);
    mn_sensor.revno = (uint16_t) rx_buf[0] | (rx_buf[1]<<8);
}
//initial_global_variable
static sns_rc initial_global_variable(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    sns_rc rc = SNS_RC_SUCCESS;
    int als_max_intt_value=0, als_nor_intt_value=0;

    //general setting
    mn_sensor.mode = MN_MODE_IDLE;
	mn_sensor.wait = MN_WAIT_2_MS;
	mn_sensor.wait_clk = WAIT_CLK_1M_EN;
    mn_sensor.interrupt_control = 	MN_INT_CTRL_ALS_OR_PS;
    //als setting
	mn_sensor.als.dk_period_en = DARK_FRMAE_EN;
	mn_sensor.als.als_pixel_mode = ALS_PIXEL_HW;
	mn_sensor.als.dk_period = DARK_FRMAE_1_32768;
	mn_sensor.als.als_pixel_sel = PIXEL_IR0B_ALS; //PIXEL_IR0B_ALS; PIXEL_IR0B_ALS_NODK
	mn_sensor.als.als_cal_mode = MN_ALS_SUB_DK_EN; //MN_ALS_SUB_DK_EN;  MN_ALS_SUB_DK_DIS
    mn_sensor.als.polling_mode = ALS_POLLING_MODE;
	mn_sensor.als.integration_time = MN_ALS_INTT_6144;
    mn_sensor.als.als_gain0 = MN_GAIN0_LOW;
    mn_sensor.als.als_gain1 = MN_GAIN1_LOW;
    mn_sensor.als.cycle = MN_CYCLE_16;
    mn_sensor.als.report_type = CMC_BIT_PRE_COUNT; //CMC_BIT_DYN_INT //CMC_BIT_RAW; //CMC_BIT_PRE_COUNT
    mn_sensor.als.factory.lux_per_count = 1000; //1
    mn_sensor.als.als_intr_percent = 5;
	//ALS AG CONFIG+
    mn_sensor.als.als_ag_en = MN_AG_EN;
    mn_sensor.als.als0_ag_h = (MN_GAIN0_MID<<4);
    mn_sensor.als.als0_ag_m = (MN_GAIN0_LOW<<4);
    mn_sensor.als.als0_ag_l = (MN_GAIN0_LOW<<4);
    mn_sensor.als.als1_ag_h = (MN_GAIN1_HIGH<<4);
    mn_sensor.als.als1_ag_m = (MN_GAIN1_MID<<4);
    mn_sensor.als.als1_ag_l = (MN_GAIN1_LOW<<4);
    mn_sensor.als.als_aintt_h = (MN_ALS_INTT_24576>>4);
    mn_sensor.als.als_aintt_m = (MN_ALS_INTT_12288>>4);
    mn_sensor.als.als_aintt_l = (MN_ALS_INTT_12288>>4);
    mn_sensor.als.als_aenh_h = (MN_ALS_ENH_MODE_4<<4);
    mn_sensor.als.als_aenh_m = (MN_ALS_ENH_MODE_1<<4);
    mn_sensor.als.als_aenh_l = (MN_ALS_ENH_MODE_1<<4);
    mn_sensor.als.als_acycle_h = MN_CYCLE_1;
    mn_sensor.als.als_acycle_m = MN_CYCLE_4;
    mn_sensor.als.als_acycle_l = MN_CYCLE_8;
    mn_sensor.als.als_ag_h2m_thd = 60000;
    mn_sensor.als.als_ag_m2h_thd = 1000;
    mn_sensor.als.als_ag_m2l_thd = 60000;
    mn_sensor.als.als_ag_l2m_thd = 5000;
    //ALS AG CONFIG-
	mn_sensor.als.factory.lux_per_lux = 1000; //1, //from mn78xxx_als_cfg()
	mn_sensor.als.persist = MN_PERIST_1;
    mn_sensor.als.enh_mode = MN_ALS_ENH_MODE_1;
	mn_sensor.als.ma_en = MN_ALS_MA_EN;
    mn_sensor.als.als_rs = MN_RS_0;
    mn_sensor.als.als_std = MN_ALS_PRE;
    mn_sensor.als.interrupt_channel_select = MN_ALS_INT_CHSEL_1;
    mn_sensor.als.als_offset = 0;
    if(mn_sensor.als.polling_mode)
        mn_sensor.als.interrupt_type = MN_INTTY_DISABLE;
    else
        mn_sensor.als.interrupt_type = MN_INTTY_ACTIVE;
	mn78_sensor_als_ag_calc_new_thd();
	mn_sensor.als.als_ag_ch0sat_en = MN_AG_CH0_SAT_EN;

	//+ if als_ag_3stage is enabled, ignore it
	mn_sensor.als.als_ag_3stage = MN_AG_3STG_DIS;
	als_max_intt = MN_ALS_INTT_384;
	als_max_cycle = MN_CYCLE_64;
	als_max_gain0 = MN_GAIN0_LOW;
	als_max_gain1 = MN_GAIN1_LOW;
	als_max_lux_condiition = 60000; //60000*0.3(lpc)=18000
	als_lux_revocer_condiition = 480; //100*4.8(new_lpc)= 480
	if(mn_sensor.als.als_ag_3stage == MN_AG_3STG_DIS)
    {
        uint32_t max_lux_lpc, nor_lux_lpc, ll2l_value, l2ll_value;

        als_nor_intt = (mn_sensor.als.als_aintt_l<<4);
        als_nor_gain0 = (mn_sensor.als.als0_ag_l>>4);
        als_nor_gain1 = (mn_sensor.als.als1_ag_l>>4);
        als_nor_cycle = mn_sensor.als.als_acycle_l;
        mn_sensor.als.als0_ag_ll = (als_max_gain0<<4);
        mn_sensor.als.als1_ag_ll = (als_max_gain1<<4);
        mn_sensor.als.als_aenh_ll = (MN_ALS_ENH_MODE_1<<4);
        mn_sensor.als.als_aintt_ll = (als_max_intt>>4);
        mn_sensor.als.als_acycle_ll = als_max_cycle;
        als_max_intt_value = mn78_als_intt_value[als_max_intt>>4];
        als_nor_intt_value = mn78_als_intt_value[als_nor_intt>>4];

        if(als_nor_gain1 == MN_GAIN1_LOW)
           lpc_gain = als_nor_intt_value / als_max_intt_value;
        else if (als_nor_gain1 == MN_GAIN1_MID)
           lpc_gain = (8 * als_nor_intt_value) / als_max_intt_value;
        else if (als_nor_gain1 == MN_GAIN1_HIGH)
           lpc_gain = (32 * als_nor_intt_value) / als_max_intt_value;
        else if (als_nor_gain1 == MN_GAIN1_HH)
           lpc_gain = (64 * als_nor_intt_value) / als_max_intt_value;

        if(als_max_gain1 == MN_GAIN1_HH)
            lpc_gain /= 64;
        else if(als_max_gain1 == MN_GAIN1_HIGH)
            lpc_gain /= 32;
        else if(als_max_gain1 == MN_GAIN1_MID)
            lpc_gain /= 8;

        max_lux_lpc = mn_sensor.als.factory.lux_per_lux * mn_sensor.als.factory.lux_per_count / 1000 * lpc_gain;
        nor_lux_lpc = mn_sensor.als.factory.lux_per_lux * mn_sensor.als.factory.lux_per_count / 1000;
        if(max_lux_lpc==0 || nor_lux_lpc==0 )
        {
            OPLUS_ALS_PS_LOG("[initial_global_variable]: Fail--> max_lux_lpc=%lu, nor_lux_lpc=%lu \n", max_lux_lpc, nor_lux_lpc);
        }
        else
        {
            l2ll_value = als_max_lux_condiition * 1000 / nor_lux_lpc;
            ll2l_value = als_lux_revocer_condiition * 1000 / max_lux_lpc;
            mn_sensor.als.als_ag_l2ll_thd = l2ll_value;
            mn_sensor.als.als_ag_ll2l_thd = ll2l_value;
        }
    }
    //- if als_ag_3stage is enabled, ignore it
    //ALS_LSRC
    offset_gain = 220;
    scale_gain = 155;
    //ps setting
    mn_sensor.ps.ps_ir0b23_sel = PS_IR_CTIA0;
	mn_sensor.ps.ps_ir3_en = PS_IR3_EN;
	mn_sensor.ps.ps_ir2_en = PS_IR2_EN;
	mn_sensor.ps.ps_ir1_en = PS_IR1_DIS;
	mn_sensor.ps.ps_ir0b_en = PS_IR0B_EN;
	mn_sensor.ps.ps_ir0a_en = PS_IR0A_DIS;
    mn_sensor.ps.polling_mode = PS_POLLING_MODE;
	mn_sensor.ps.integration_time = MN_PS_INTT_32;
    mn_sensor.ps.ps_gain0 = MN_GAIN0_MID;
    mn_sensor.ps.ps_gain1 = MN_GAIN1_MID;
    mn_sensor.ps.ps_pulse = MN_PS_PULSE_32; //match ps_avg
    mn_sensor.ps.ps_avg = MN_AVG_64;
    mn_sensor.ps.persist = MN_PERIST_1;
    mn_sensor.ps.ir_drive = MN_IR_DRIVE_100;
    mn_sensor.ps.ir_drive_tune = MN_IR_DRIVE_14_92;
    mn_sensor.ps.ps_max_ct = 10000; //enable PS, factory
	mn_sensor.ps.enh_mode = MN_PS_ENH_MODE_1;
    mn_sensor.ps.ps_std = MN_PS_PRE;
    mn_sensor.ps.interrupt_channel_select = MN_PS0_INT_CHSEL;
    mn_sensor.ps.ir_on_control = MN_IR_ON_CTRL_ON;
    mn_sensor.ps.ps_offset = 0;
    mn_sensor.ps.high_threshold = 2500;
    mn_sensor.ps.low_threshold = 1000;
    if(mn_sensor.ps.polling_mode)
        mn_sensor.ps.interrupt_type = MN_INTTY_DISABLE;
    else
        mn_sensor.ps.interrupt_type = MN_INTTY_ACTIVE;
    rc = mn78_sensor_write_global_variable(scp_service, port_handle);
    return rc;
}

static void mn78_sensor_update_mode(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    int als_time = als_sensing_time(mn_sensor.als.integration_time, mn_sensor.als.cycle);
    int ps_time = ps_sensing_time(mn_sensor.ps.integration_time, mn_sensor.ps.ps_pulse);
    UNUSED_VAR(als_time);
    UNUSED_VAR(ps_time);
    //OPLUS_ALS_PS_LOG("time: als =%dms, ps=%dms \n", als_time, ps_time);
    //OPLUS_ALS_PS_LOG("mode selection =0x%x \n", (mn78xxx_dev.ps_enable | (mn78xxx_dev.als_enable << 1)));

    //**** mode selection ****
    switch((mn_sensor.als.enable << 1) | mn_sensor.ps.enable)
    {
        case 0: //disable all
            mn_sensor.mode = MN_MODE_IDLE;
            break;
        case 1: //als = 0, ps = 1
            mn_sensor.mode = MN_MODE_PS;
            break;
        case 2: //als = 1, ps = 0
            mn_sensor.mode = MN_MODE_ALS;
            break;
        case 3: //als = 1, ps = 1
            mn_sensor.mode = MN_MODE_ALS_PS;
            break;
    }
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET) );
    if(mn_sensor.ps.ps_first_flag == true)
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, MN_MODE_PS );
    else if(mn_sensor.mode == MN_MODE_PS)
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, (MN_WAIT_200_MS | mn_sensor.mode) );
    else
    	mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, (mn_sensor.wait | mn_sensor.mode) );

    if(mn_sensor.mode != MN_MODE_IDLE)
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );
}

static void mn78_sensor_enable_als(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, bool enable)
{
    OPLUS_ALS_PS_LOG("[mn78_sensor_enable_als]: als enable=%d \n", enable);
    if(enable)
    {
        mn_sensor.als.enable = true;
        als_first_flag=true;
        als_fac_flag = true; //emi by 20200615_v2
        ag_chg_flag = false;
        ag_last_stage = 0x3;
        set_lsensor_intr_threshold(scp_service, port_handle, L_SENSOR_LTHD_TRIGER, L_SENSOR_HTHD);
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RESET | MN_UN_LOCK) );
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );
    }
    else
    {
        als_first_flag=false;
         mn_sensor.als.enable = false;
    }
	#if ALS_FAST_UPDATE
	mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET) );
	mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_LL, (mn_sensor.als.als1_ag_ll | mn_sensor.als.als0_ag_ll | MN_CYCLE_1));
	mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_L, (mn_sensor.als.als1_ag_l | mn_sensor.als.als0_ag_l | MN_CYCLE_1));
	mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_M, (mn_sensor.als.als1_ag_m | mn_sensor.als.als0_ag_m | MN_CYCLE_1));
	mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_CFG_H, ((MN_ALS_ENH_MODE_2<<4) | mn_sensor.als.als_aintt_h));
	mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_H, (mn_sensor.als.als1_ag_h | mn_sensor.als.als0_ag_h | MN_CYCLE_1));
	mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );
	#endif
    mn78_sensor_update_mode(scp_service, port_handle);
}

static void mn78_sensor_enable_ps(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, bool enable)
{
    OPLUS_ALS_PS_LOG("[mn78_sensor_enable_ps]: ps enable=%d \n", enable);
    if(enable)
    {
		set_psensor_intr_threshold( scp_service, port_handle, mn_sensor.ps.ps_max_ct, (mn_sensor.ps.ps_max_ct+1));
		mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );
		mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RESET | MN_UN_LOCK));
		mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RUN | MN_UN_LOCK));
		mn_sensor.ps.enable = true;
		mn_sensor.ps.ps_first_flag = true;
	}
    else
    {
         mn_sensor.ps.enable = false;
		 mn_sensor.ps.ps_first_flag = false;
    }
    mn78_sensor_update_mode(scp_service, port_handle);
}

static bool mn78_sensor_ag_stage_change(uint16_t ch1)
{
    bool ret = false;
    if(ch1 == als_dynamic_intt_high_thr[0] ||
       ch1 == als_dynamic_intt_high_thr[1] ||
       ch1 == als_dynamic_intt_low_thr[1] ||
       ch1 == als_dynamic_intt_low_thr[2])
    {
        ret = true;
    }

    if( (mn_sensor.als.als_ag_en == MN_AG_EN && mn_sensor.als.als_ag_3stage == MN_AG_3STG_DIS)
        && (ch1==mn_sensor.als.als_ag_l2ll_thd || ch1==mn_sensor.als.als_ag_ll2l_thd) )
    {
        ret = true;
    }

    return ret;
}

static void lsrc_raw_convert_to_adc(uint32_t ch0, uint32_t ch1, uint16_t *new_raw)
{
    uint32_t als_offset=0, als_scale=0;
    uint16_t nor_raw=0;

    if(ch1 > 0)
    {
        lsrc_ratio = ch0*1000 / ch1;
    	als_offset = (ch0 * ch0) / (ch0+ch1) * offset_gain / 1000;
		als_scale = (1000-scale_gain)*ch1 / 1000;
    	if( als_offset < als_scale )
    		nor_raw = ch1 - als_offset;
    	else
    		nor_raw = ch1 * scale_gain / 1000;
    }
    else
    {
        nor_raw = ch1;
    }
    lsrc_raw = nor_raw;
    lsrc_als_offset = als_offset;

    *new_raw = nor_raw;

    OPLUS_ALS_PS_LOG("[lsrc_raw_convert_to_adc]: ch0=%lu, ch1=%lu, nor_raw=%u \r\n", ch0, ch1, nor_raw);
}

static void mn78_sensor_ag_4stage_lpc(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint32_t *new_lpc)
{
    uint8_t i2c_read_data, ag_current_stage;

    mn_sensor_I2C_Read(scp_service, port_handle, 0x69, 1, &i2c_read_data);
    ag_current_stage = ((i2c_read_data&0x30)>>4);

    if( ((als_first_flag==false)&&(ag_last_stage==0x3)) && (ag_current_stage!=ag_last_stage) && (mn_sensor.als.data.channel[1]==mn_sensor.als.als_ag_ll2l_thd) )//LLtoL //emi by 20200716
        *new_lpc = mn_sensor.als.factory.lux_per_count*lpc_gain;
    else if( (ag_current_stage==0x3) && (ag_last_stage!=ag_current_stage) && (mn_sensor.als.data.channel[1]==mn_sensor.als.als_ag_l2ll_thd) ) //LtoLL //emi by 20200716
        *new_lpc = mn_sensor.als.factory.lux_per_count;
	else if( (ag_current_stage==0x3) && (ag_last_stage!=ag_current_stage) && (mn_sensor.als.als_sat&0x11) ) //L_stage, MN_AG_CH0_SAT_EN //emi by 20200713
    {
        *new_lpc = mn_sensor.als.factory.lux_per_count;
        ag_chg_flag = true;
    }
    else if(ag_current_stage == 0x3) //LL_stage
        *new_lpc = mn_sensor.als.factory.lux_per_count*lpc_gain;
    else
        *new_lpc = mn_sensor.als.factory.lux_per_count;
	ag_last_stage = ag_current_stage;
    OPLUS_ALS_PS_LOG("[mn78_sensor_ag_4stage_lpc]: REG0x69=0x%x, new_lpc=%lu, ch1=%u \r\n", i2c_read_data, *new_lpc, mn_sensor.als.data.channel[1]);
}

static void lux_convert_to_lux(uint32_t raw, uint32_t lpc, uint32_t *new_lux)
{
    uint32_t lux = 0, new_lpc;

    lsrc_lux = lpc * raw / 1000;
    new_lpc = mn_sensor.als.factory.lux_per_lux * lpc / 1000;
    lux =  new_lpc * raw;
    *new_lux = lux;

    OPLUS_ALS_PS_LOG("[lux_convert_to_lux]: lux_per_lux=%d, new_lpc=%lu, lux=%lu \n", mn_sensor.als.factory.lux_per_lux, new_lpc, lux/1000);
}

static uint32_t mn78_sensor_get_als_value(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint16_t als_ch0, uint16_t als_ch1)
{
    uint32_t lux = 0;
    uint32_t als_new_lpc = 0;
    uint16_t als_lsrc_raw=0;
    switch(mn_sensor.als.report_type)
    {
        case CMC_BIT_RAW:
            return als_ch1;
        break;
        case CMC_BIT_PRE_COUNT:
            lsrc_raw_convert_to_adc(als_ch0, als_ch1, &als_lsrc_raw);
            if(mn_sensor.als.als_ag_en == MN_AG_EN && mn_sensor.als.als_ag_3stage == MN_AG_3STG_DIS) //LL_stage
                mn78_sensor_ag_4stage_lpc(scp_service, port_handle, &als_new_lpc);
            else
                als_new_lpc = mn_sensor.als.factory.lux_per_count;

            if(ag_chg_flag == true)
            {
                ag_chg_flag = false;
                return mn_sensor.als.data.lux;
            }
            else
               lux_convert_to_lux(als_lsrc_raw, als_new_lpc, &lux);
            return (lux / 1000);
        break;
    }

    return 0;
}

static int32_t mn78xxx_read_als(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    int ret = 0;
    uint8_t rx_buf[8] = {0};
    ret = mn_sensor_I2C_Read(scp_service, port_handle, DEVREG_ALS_STATUS, 8, rx_buf);

    mn_sensor.als.compare_high = (rx_buf[0] & 0x10);
    mn_sensor.als.compare_low = (rx_buf[0] & 0x08);
    mn_sensor.als.interrupt_flag = (rx_buf[0] & 0x04);
    mn_sensor.als.compare_reset = (rx_buf[0] & 0x02);
    mn_sensor.als.lock = (rx_buf[0] & 0x01);
    mn_sensor.als.als_sat = rx_buf[1];
    mn_sensor.als.data.channel[0] = ((rx_buf[3]<<8) | rx_buf[2]);
    mn_sensor.als.data.channel[1] = ((rx_buf[5]<<8) | rx_buf[4]);
    mn_sensor.als.data.channel[2] = ((rx_buf[7]<<8) | rx_buf[6]);

	OPLUS_ALS_PS_LOG("[mn78xxx_read_als]: als_status=0x%x, als_sat=0x%x\n", rx_buf[0], mn_sensor.als.als_sat);
	OPLUS_ALS_PS_LOG("[mn78xxx_read_als]: ch0=%d, ch1=%d, ch2=%d \n", mn_sensor.als.data.channel[0], mn_sensor.als.data.channel[1], mn_sensor.als.data.channel[2]);
	OPLUS_ALS_PS_LOG("[mn78_sensor_read_als]: als_lock=%d \n" ,mn_sensor.als.lock);
	if((als_fac_flag == true) && (mn_sensor.als.compare_high == mn_sensor.als.compare_low))
	{
		OPLUS_ALS_PS_LOG("[mn78xxx_read_als]: als don't ready \r\n");
		return -1;
	}
	if(als_first_flag==true && mn78_sensor_ag_stage_change(mn_sensor.als.data.channel[1]) )
	{
		OPLUS_ALS_PS_LOG("[mn78xxx_read_als]: als_ag don't ready \r\n");
		return -1;
	}
	#if ALS_FAST_UPDATE
    if(als_first_flag == true)
	{
		mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET) );
		mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_LL, (mn_sensor.als.als1_ag_ll | mn_sensor.als.als0_ag_ll | mn_sensor.als.als_acycle_ll));
		mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_L, (mn_sensor.als.als1_ag_l | mn_sensor.als.als0_ag_l | mn_sensor.als.als_acycle_l));
		mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_M, (mn_sensor.als.als1_ag_m | mn_sensor.als.als0_ag_m | mn_sensor.als.als_acycle_m));
		mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_CFG_H, (mn_sensor.als.als_aenh_h | mn_sensor.als.als_aintt_h));
		mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_ALS_AG_FILT_H, (mn_sensor.als.als1_ag_h | mn_sensor.als.als0_ag_h | mn_sensor.als.als_acycle_h));
		mn_sensor_I2C_Write(scp_service, port_handle,DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );
	}
	#endif
    //als_first_flag = false;
    return mn_sensor.als.data.channel[1];
}

static int32_t mn78xxx_read_ps(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    int ret = 0;
    uint8_t rx_buf[11] = {0};
    ret = mn_sensor_I2C_Read(scp_service, port_handle, DEVREG_PS_STATUS, 11, rx_buf);

    mn_sensor.ps.compare_high = (rx_buf[0] & 0x10);
    mn_sensor.ps.compare_low = (rx_buf[0] & 0x08);
    mn_sensor.ps.interrupt_flag = (rx_buf[0] & 0x04);
    mn_sensor.ps.compare_reset = (rx_buf[0] & 0x02);
    mn_sensor.ps.lock = (rx_buf[0] & 0x01);
    mn_sensor.ps.ps0_sat = rx_buf[1];
    mn_sensor.ps.ps1_sat = rx_buf[2];

    mn_sensor.ps.compare_high = (rx_buf[0] & 0x10);
    mn_sensor.ps.compare_low = (rx_buf[0] & 0x08);
    mn_sensor.ps.interrupt_flag = (rx_buf[0] & 0x04);
    mn_sensor.ps.compare_reset = (rx_buf[0] & 0x02);
    mn_sensor.ps.lock = (rx_buf[0] & 0x01);
    mn_sensor.ps.ps0_sat = rx_buf[1];
    mn_sensor.ps.ps1_sat = rx_buf[2];
    mn_sensor.ps.data.ps0_channel[0] = (uint16_t)((rx_buf[4]<<8) | rx_buf[3]);
    mn_sensor.ps.data.ps0_channel[1] = (uint16_t)((rx_buf[6]<<8) | rx_buf[5]);
    mn_sensor.ps.data.ps1_channel[0] = (uint16_t)((rx_buf[8]<<8) | rx_buf[7]);
    mn_sensor.ps.data.ps1_channel[1] = (uint16_t)((rx_buf[10]<<8) | rx_buf[9]);

	if(mn_sensor.ps.interrupt_channel_select == MN_PS0_INT_CHSEL)
    {
        mn_sensor.ps.sat = mn_sensor.ps.ps0_sat;
        mn_sensor.ps.data.ir_data = mn_sensor.ps.data.ps0_channel[0];
        mn_sensor.ps.data.pdata = mn_sensor.ps.data.ps0_channel[1];
    }
    else
    {
        mn_sensor.ps.sat = mn_sensor.ps.ps1_sat;
        mn_sensor.ps.data.ir_data = mn_sensor.ps.data.ps1_channel[0];
        mn_sensor.ps.data.pdata = mn_sensor.ps.data.ps1_channel[1];
    }

	OPLUS_ALS_PS_LOG("[mn78_sensor_read_ps]: ps_int_chsel = 0x%x\n" , (mn_sensor.ps.interrupt_channel_select>>7));
    OPLUS_ALS_PS_LOG("[mn78_sensor_read_ps]: ps_status=0x%x, ps_sat=0x%x \n", rx_buf[0], mn_sensor.ps.sat);
	OPLUS_ALS_PS_LOG("[mn78_sensor_read_ps]: irdata=%d, pdata=%d \n" ,mn_sensor.ps.data.ir_data, mn_sensor.ps.data.pdata);
	OPLUS_ALS_PS_LOG("[mn78_sensor_read_ps]: ps_lock=%d \n" ,mn_sensor.ps.lock);
    return mn_sensor.ps.data.pdata;
}

void mn78_sensor_dump_reg(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    uint8_t reg_value;
    uint8_t reg_map[] = {
                    DEVREG_ENABLE      ,
                    DEVREG_INT_CTRL    ,
                    DEVREG_RESET       ,
                    DEVREG_DARK_FRMAE  ,
                    DEVREG_ALS_PIXEL   ,
                    DEVREG_PS_PIXEL    ,
                    DEVREG_REV_ID      ,
                    DEVREG_CHIP_ID     ,
                    DEVREG_ALS_CONFIG  ,
                    DEVREG_ALS_FILT    ,
                    DEVREG_ALS_INT     ,
                    DEVREG_ALS_ENH     ,
                    DEVREG_PS_CONFIG   ,
                    DEVREG_PS_PULSE    ,
                    DEVREG_PS_INT      ,
                    DEVREG_PS_ENH      ,
                    DEVREG_LED_CONFIG  ,
                    DEVREG_ALS_ILTL    ,
                    DEVREG_ALS_ILTH    ,
                    DEVREG_ALS_IHTL    ,
                    DEVREG_ALS_IHTH    ,
                    DEVREG_PS_ILTL     ,
                    DEVREG_PS_ILTH     ,
                    DEVREG_PS_IHTL     ,
                    DEVREG_PS_IHTH     ,
                    DEVREG_ALS_STATUS  ,
                    DEVREG_ALS_SAT     ,
                    DEVREG_C0DATAL     ,
                    DEVREG_C0DATAH     ,
                    DEVREG_C1DATAL     ,
                    DEVREG_C1DATAH     ,
                    DEVREG_C2DATAL     ,
                    DEVREG_C2DATAH     ,
                    DEVREG_PS_STATUS   ,
                    DEVREG_PS_SAT      ,
                    DEVREG_PS1_SAT     ,
                    DEVREG_PS_ADATAL   ,
                    DEVREG_PS_ADATAH   ,
                    DEVREG_PS_RDATAL   ,
                    DEVREG_PS_RDATAH   ,
                    DEVREG_PS1_ADATAL   ,
                    DEVREG_PS1_ADATAH   ,
                    DEVREG_PS1_RDATAL   ,
                    DEVREG_PS1_RDATAH   ,
                    DEVREG_ALS_AG_CFG_L ,
                    DEVREG_ALS_AG_FILT_L,
                    DEVREG_ALS_AG_CFG_M ,
                    DEVREG_ALS_AG_FILT_M,
                    DEVREG_ALS_AG_CFG_H ,
                    DEVREG_ALS_AG_FILT_H,
                    DEVREG_ALS_AG_H2M_THL ,
                    DEVREG_ALS_AG_H2M_THH ,
                    DEVREG_ALS_AG_M2H_THL ,
                    DEVREG_ALS_AG_M2H_THH ,
                    DEVREG_ALS_AG_M2L_THL ,
                    DEVREG_ALS_AG_M2L_THH ,
                    DEVREG_ALS_AG_L2M_THL ,
                    DEVREG_ALS_AG_L2M_THH ,
                    DEVREG_ALS_AG_INFO ,
                    0x66 ,
                    0x67 ,
                    0x68 ,
                    0x69 ,
                    0xF0 ,
                    0xF7 ,
                    0xF8 ,
                    0x26 ,
                    0x27 ,
    };
    uint8_t i = 0;
    uint16_t n = sizeof(reg_map)/sizeof(reg_map[0]);
    for(i=0; i<n;i++)
    {
        mn_sensor_I2C_Read(scp_service, port_handle, reg_map[i], 1, &reg_value);
        OPLUS_ALS_PS_LOG("mn78_dump_reg: reg[0x%x]=0x%x \n", reg_map[i], reg_value);
    }
}
/****************************************************************************************************/
static sns_rc mn78911_get_who_am_i(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    mn78_sensor_read_renvo(scp_service, port_handle);
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_init_als(sns_sync_com_port_service *scp_service,sns_sync_com_port_handle *port_handle)
{
    sns_rc rc = SNS_RC_SUCCESS;
    if(init_flag == true)
    rc = initial_global_variable(scp_service, port_handle);
    init_flag = false;
	rc = mn78_sensor_write_global_variable(scp_service, port_handle);
    return rc;
}

static sns_rc mn78911_deinit_als(void)
{
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_init_ps_irq(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    UNUSED_VAR(scp_service);
    UNUSED_VAR(port_handle);
	#if 1
	if(init_flag == true){
    initial_global_variable(scp_service, port_handle);
	mn78_sensor_dump_reg(scp_service, port_handle);
    }
    init_flag = false;
	mn78_sensor_write_global_variable(scp_service, port_handle);
	mn78_sensor_dump_reg(scp_service, port_handle);
	#endif
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_init_als_irq(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    UNUSED_VAR(scp_service);
    UNUSED_VAR(port_handle);
    return SNS_RC_SUCCESS;
}

static void mn78_sensor_intr_als_set_thd(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint16_t raw); //emi by 20200615_v1

static sns_rc mn78911_get_ps_device_irq_mask(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle,uint8_t * mask)
{
	#if 1
	uint8_t status = 0x0;
	uint8_t flag_als = 0x0;
	uint8_t flag_ps = 0x0;
	
    mn_sensor_I2C_Read(scp_service, port_handle, DEVREG_ALS_STATUS, 1, &status);
    flag_als = (status & 0x04);

	status = 0x0;
	
    mn_sensor_I2C_Read(scp_service, port_handle, DEVREG_PS_STATUS, 1, &status);
    flag_ps = (status & 0x04);

    if(flag_als == 0x04)
    {
        als_fac_flag = false; //emi by 20200615_v2
        *mask = 0x01;
        mn78xxx_read_als( scp_service, port_handle );
        mn78_sensor_intr_als_set_thd( scp_service, port_handle, mn_sensor.als.data.channel[1] );
        if((mn_sensor.als.als_ag_en==MN_AG_EN && mn_sensor.als.als_ag_3stage==MN_AG_3STG_DIS) && (mn_sensor.als.data.channel[1]==mn_sensor.als.als_ag_l2ll_thd) && (mn_sensor.als.high_threshold==65535))
		    set_lsensor_intr_threshold(scp_service, port_handle, L_SENSOR_LTHD_TRIGER, L_SENSOR_HTHD);
        /* ALS Interrupt */
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RESET | MN_UN_LOCK) );
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );
    }
    else if (flag_ps == 0x04)
    {
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );
        *mask = 0x02;
    }
    else
    {
        *mask = 0x08;
    }
#endif
    return SNS_RC_SUCCESS;
}

static void mn78911_dump_reg(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    mn78_sensor_dump_reg(scp_service, port_handle);
}

static sns_rc mn78911_als_enable(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, bool enable)
{
    mn78_sensor_enable_als(scp_service,port_handle,enable);
    return SNS_RC_SUCCESS;
}

//add 20200603
static void mn78_sensor_intr_als_set_thd(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint16_t raw)
{
    uint16_t thd_offset = 0;

    thd_offset = raw * mn_sensor.als.als_intr_percent / 100;
    if(thd_offset <= 0)
        thd_offset = 1;
    OPLUS_ALS_PS_LOG("[mn78_sensor_intr_als_set_thd]: thd_offset=%d \r\n", thd_offset );
    //set low threshold
    if(raw < thd_offset)    //overflow
        mn_sensor.als.low_threshold = 0;
    else
        mn_sensor.als.low_threshold = raw - thd_offset;

    //set high threshold
    if(raw > (0xffff-thd_offset)) //overflow
        mn_sensor.als.high_threshold = 0xffff;
    else
        mn_sensor.als.high_threshold = raw + thd_offset;
    //set new threshold
    set_lsensor_intr_threshold(scp_service, port_handle, mn_sensor.als.low_threshold, mn_sensor.als.high_threshold);
}

static sns_rc mn78911_get_als_data(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, float *raw_data, int len, uint8_t als_type, bool is_als_dri)
{
    int32_t als_data = 0, report_lux = 0;
    UNUSED_VAR(len); //pls check
    UNUSED_VAR(als_type);
    UNUSED_VAR(is_als_dri);
    als_data = mn78xxx_read_als(scp_service,port_handle);
#if 0 //emi by 20200615_v1
    if(mn_sensor.als.interrupt_flag == MN_INT_TRIGGER)
    {
        mn78_sensor_intr_als_set_thd( scp_service, port_handle, mn_sensor.als.data.channel[1] );
        if((mn_sensor.als.als_ag_en==MN_AG_EN && mn_sensor.als.als_ag_3stage==MN_AG_3STG_DIS) && (mn_sensor.als.data.channel[1]==mn_sensor.als.als_ag_l2ll_thd) && (mn_sensor.als.high_threshold==65535))
		    set_lsensor_intr_threshold(scp_service, port_handle, L_SENSOR_LTHD_TRIGER, L_SENSOR_HTHD);
        /* ALS Interrupt */
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RESET | MN_UN_LOCK) );
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );
    }
#else
	mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ALS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );
#endif
    if (als_data < 0) {
        //invalid lux value when debounce is on
		OPLUS_ALS_PS_LOG("invalid lux value when debounce is on, als_data:%d\n", als_data);
        return SNS_RC_FAILED;
    }
	report_lux = mn78_sensor_get_als_value(scp_service,port_handle,mn_sensor.als.data.channel[0], mn_sensor.als.data.channel[1]); //calculate lux
	als_first_flag = false; //emi by 20200716
    //report lux //pls check
    OPLUS_ALS_PS_LOG("[mn78911_get_als_data]: -------------------  lux=%u \n", report_lux);
    *raw_data = report_lux;

    return SNS_RC_SUCCESS;
}


static sns_rc mn78911_ps_enable(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, bool enable)
{
	mn78_sensor_enable_ps(scp_service,port_handle,enable);
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_clear_ps_int(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle)
{
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_ps_set_thd(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint16_t ps_thd_near, uint16_t ps_thd_far, alsps_ps_state status)
{
    UNUSED_VAR(status); //pls check
    mn_sensor.ps.low_threshold = ps_thd_far + temp_offset;
    mn_sensor.ps.high_threshold = ps_thd_near + temp_offset;
    set_psensor_intr_threshold( scp_service, port_handle, mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_set_offset(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, int val)
{
    mn_sensor.ps.ps_offset = val;
    OPLUS_ALS_PS_LOG("mn78911_set_offset(%d) \n", val);
	temp_offset = val;
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_get_ps_data(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, uint16_t *val)
{
    uint8_t ps_status;
	//sns_busy_wait(sns_convert_ns_to_ticks(30*1000*1000));

	mn78xxx_read_ps(scp_service, port_handle);

    if(mn_sensor.ps.compare_high == mn_sensor.ps.compare_low)
	{
		OPLUS_ALS_PS_LOG("[mn78911_get_ps_data]: ps don't ready \n");
		return SNS_RC_FAILED;
	}
    ps_status = (mn_sensor.ps.compare_low >> 3);

    if(mn_sensor.ps.ps_first_flag)
    {
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET) );
        if(mn_sensor.mode == MN_MODE_PS) 	//add for ps consume issue emn 20200720
            mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, (MN_WAIT_200_MS | mn_sensor.mode) );
        else
            mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, (mn_sensor.wait | mn_sensor.mode) );
        mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );
//		mn78_sensor_dump_reg(scp_service, port_handle);
    }
	mn_sensor.ps.ps_first_flag = false;

    if(ps_status)
    	set_psensor_intr_threshold( scp_service, port_handle, mn_sensor.ps.low_threshold, mn_sensor.ps.high_threshold);
    //report PS //pls check
    OPLUS_ALS_PS_LOG("[mn78911_get_ps_data]: -------------------  ps_status=%u", ps_status);
    //*val = ps_status;

    if ((mn_sensor.ps.data.pdata - temp_offset) >= 0){
		*val = mn_sensor.ps.data.pdata - temp_offset;
	} else {
		*val = 0;
	}

    if(mn_sensor.ps.interrupt_flag == MN_INT_TRIGGER)
 	{
         //mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RESET | MN_UN_LOCK) );
	    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );
	}
	OPLUS_ALS_PS_LOG("[mn78911_get_ps_data]: ps temp_offset =%d \n",temp_offset);
    //mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_STATUS, (MN_CMP_RUN | MN_UN_LOCK) );

    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_get_ps_original_data(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, int *val)
{
/*    OPLUS_ALS_PS_LOG("mn78911_get_ps_original_data: set ps_offset=0 \n");
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_OFSL, 0);
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_OFSH, 0);
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, MN_MODE_PS ); //emi by 20200602
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) ); */

    mn78xxx_read_ps(scp_service, port_handle);
    *val = mn_sensor.ps.data.pdata;

/*    OPLUS_ALS_PS_LOG("mn78911_get_ps_original_data: recover ps_offset=%d \n", mn_sensor.ps.ps_offset);
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_OFF | MN_RESETN_RESET) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_OFSL, (uint8_t)(mn_sensor.ps.ps_offset& 0xff) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_PS_OFSH, ((mn_sensor.ps.ps_offset & 0xff00) >> 8) );
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_ENABLE, (mn_sensor.wait | mn_sensor.mode) ); //emi by 20200602
    mn_sensor_I2C_Write( scp_service, port_handle, DEVREG_RESET, (MN_POWER_ON | MN_RESETN_RUN) );  */
    
    return SNS_RC_SUCCESS;
}

static sns_rc mn78911_get_ps_off_data(sns_sync_com_port_service *scp_service, sns_sync_com_port_handle *port_handle, int *val)
{
    UNUSED_VAR(scp_service);
    UNUSED_VAR(port_handle);

    OPLUS_ALS_PS_LOG("mn78911_get_ps_off_data: ir_data=%d \n", mn_sensor.ps.data.ir_data);

    *val = mn_sensor.ps.data.ir_data; //pls check
    return SNS_RC_SUCCESS;
}

static bool mn78911_prox_need_ir_info(void)
{
    return true;
}

//static bool mn78911_prox_need_prox(void)
//{
//    return false;
//}

static void mn78911_reconfig_reg(uint8_t reg_num, uint8_t* reg_table)
{
    UNUSED_VAR(reg_num);
    UNUSED_VAR(reg_table);
    return;
}

/****************************************************************************************************/
struct alsps_ps_operations mn78911_ps_ops = {
 .get_who_am_i = mn78911_get_who_am_i,
 .init_driver = mn78911_init_als,
 .ps_enable = mn78911_ps_enable,
 .clear_ps_int = mn78911_clear_ps_int,
 .init_irq = mn78911_init_ps_irq,
 .get_ps_data = mn78911_get_ps_data,
 .ps_set_thd = mn78911_ps_set_thd,
 .dump_reg = mn78911_dump_reg,
 .set_offset = mn78911_set_offset,
 .get_ps_device_irq_mask = mn78911_get_ps_device_irq_mask,
 .deinit_driver = mn78911_deinit_als,
 .prox_need_ir_info = mn78911_prox_need_ir_info,
 .get_ps_original_data = mn78911_get_ps_original_data,
 .get_ir_data = mn78911_get_ps_off_data,
 .hardware_cali = NULL,
// .prox_need_gesture = mn78911_prox_need_prox,
 .recover_device = NULL,
 .reconfig_reg_table = mn78911_reconfig_reg,
// .sensor_selftest = NULL,
};

struct alsps_als_operations mn78911_als_ops = {
 .get_who_am_i = mn78911_get_who_am_i,
 .init_driver = mn78911_init_als,
 .als_enable = mn78911_als_enable,
 .clear_als_int = NULL,
 .init_irq = mn78911_init_als_irq,
 .get_als_data = mn78911_get_als_data,
 .dump_reg = mn78911_dump_reg,
 .get_als_device_irq_mask = mn78911_get_ps_device_irq_mask,
 .deinit_driver = mn78911_deinit_als,
 .reconfig_reg_table = mn78911_reconfig_reg,
};

#endif