#pragma once
#ifdef SUPPORT_MN78911
#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_gpio_service.h"
#ifndef __ELAN_MN_SENSOR__
#define __ELAN_MN_SENSOR__


#define MN_MODE_IDLE		(0x00)
#define MN_MODE_ALS		    (0x01)
#define MN_MODE_PS			(0x02)
#define MN_MODE_ALS_PS		(0x03)

#define MN_WAIT_0_MS			(0x0<<4)
#define MN_WAIT_2_MS			(0x1<<4)
#define MN_WAIT_4_MS			(0x2<<4)
#define MN_WAIT_8_MS			(0x3<<4)
#define MN_WAIT_12_MS			(0x4<<4)
#define MN_WAIT_20_MS			(0x5<<4)
#define MN_WAIT_30_MS			(0x6<<4)
#define MN_WAIT_40_MS			(0x7<<4)
#define MN_WAIT_50_MS			(0x8<<4)
#define MN_WAIT_75_MS			(0x9<<4)
#define MN_WAIT_100_MS		    (0xA<<4)
#define MN_WAIT_150_MS		    (0xB<<4)
#define MN_WAIT_200_MS		    (0xC<<4)
#define MN_WAIT_300_MS		    (0xD<<4)
#define MN_WAIT_400_MS		    (0xE<<4)
#define MN_WAIT_SINGLE		    (0xF<<4)

//static int mn78_wait_value[] = {0, 2, 4, 8, 12, 20, 30, 40, 50, 75, 100, 150, 200, 300, 400};
//int mn78_wait_len = sizeof(mn78_wait_value)/sizeof(int);
#define MN_AG_3STG_EN           (1 << 1)
#define MN_AG_3STG_DIS          (0 << 1)
#define MN_AG_CH0_SAT_EN        (1 << 2)
#define MN_AG_CH0_SAT_DIS       (0 << 2)
#define WAIT_CLK_1M_DIS         (0 << 3)
#define WAIT_CLK_1M_EN          (1 << 3)
#define MN_INT_CTRL_ALS_OR_PS	(0 << 4)
#define MN_INT_CTRL_ALS		    (1 << 4)
#define MN_INT_CTRL_PS		    (2 << 4)
#define MN_INT_CTRL_ALS_AND_PS	(3 << 4)
#define MN_ADC_FREQ_1M		    (0 << 6)
#define MN_ADC_FREQ_500K 	    (1 << 6)
#define MN_AG_DIS	            (0 << 7)
#define MN_AG_EN	            (1 << 7)

#define MN_RESETN_RESET	        (0 << 1)
#define MN_RESETN_RUN		    (1 << 1)
#define MN_POWER_ON		        (0)
#define MN_POWER_OFF		    (1)

#define DARK_FRMAE_1_2          (0x1)
#define DARK_FRMAE_1_4          (0x2)
#define DARK_FRMAE_1_8          (0x3)
#define DARK_FRMAE_1_16         (0x4)
#define DARK_FRMAE_1_32         (0x5)
#define DARK_FRMAE_1_64         (0x6)
#define DARK_FRMAE_1_128        (0x7)
#define DARK_FRMAE_1_256        (0x8)
#define DARK_FRMAE_1_512        (0x9)
#define DARK_FRMAE_1_1024       (0xA)
#define DARK_FRMAE_1_2048       (0xB)
#define DARK_FRMAE_1_4096       (0xC)
#define DARK_FRMAE_1_8192       (0xD)
#define DARK_FRMAE_1_16384      (0xE)
#define DARK_FRMAE_1_32768      (0xF)
#define ALS_PIXEL_SW            (1 << 6)
#define ALS_PIXEL_HW            (0 << 6)
#define DARK_FRMAE_EN           (1 << 7)
#define DARK_FRMAE_DIS          (0 << 7)

#define ALS_IR0A_EN             (1 << 0)
#define ALS_IR0A_DIS            (0 << 0)
#define ALS_IR0B_EN             (1 << 1)
#define ALS_IR0B_DIS            (0 << 1)
#define ALS_CLR_EN              (1 << 2)
#define ALS_CLR_DIS             (0 << 2)
#define ALS_PIXEL_EN            (1 << 3)
#define ALS_PIXEL_DIS           (0 << 3)
#define ALS_DARK_EN             (1 << 4)
#define ALS_DARK_DIS            (0 << 4)
#define ALS_CLR_CTIA1           (1 << 7)
#define ALS_CLR_CTIA0           (0 << 7)
#define PIXEL_IR0A0B_ALS        0x1B
#define PIXEL_IR0A_ALS        	0x19
#define PIXEL_IR0B_ALS        	0x1A
#define PIXEL_IR0A0B_CLR        0x17
#define PIXEL_IR0A0B_ALS_CLR    0x1F
#define PIXEL_CLR_ALS           0x9C

#define PIXEL_IR_ALS_NODK       0x0B
#define PIXEL_IR0B_ALS_NODK     0x0A
#define PIXEL_IR_ALS_CLR_NODK   0x8F
#define PIXEL_CLR_ALS_NODK      0x0C
#define PIXEL_IR_CLR_NODK       0x87

#define PS_IR0A_EN             (1 << 0)
#define PS_IR0A_DIS            (0 << 0)
#define PS_IR0B_EN             (1 << 1)
#define PS_IR0B_DIS            (0 << 1)
#define PS_IR1_EN              (1 << 2)
#define PS_IR1_DIS             (0 << 2)
#define PS_IR2_EN              (1 << 3)
#define PS_IR2_DIS             (0 << 3)
#define PS_IR3_EN              (1 << 4)
#define PS_IR3_DIS             (0 << 4)
#define PS_IR_CTIA0            (1 << 7)
#define PS_IR_CTIA1            (0 << 7)

#define MN_ALS_INTT_48			(0<<4)
#define MN_ALS_INTT_96			(1<<4)
#define MN_ALS_INTT_192			(2<<4)
#define MN_ALS_INTT_384			(3<<4)
#define MN_ALS_INTT_576			(4<<4)
#define MN_ALS_INTT_768			(5<<4)
#define MN_ALS_INTT_1152	    (6<<4)
#define MN_ALS_INTT_1536	    (7<<4)
#define MN_ALS_INTT_2304	    (8<<4)
#define MN_ALS_INTT_3072	    (9<<4)
#define MN_ALS_INTT_4608		(10<<4)
#define MN_ALS_INTT_6144		(11<<4)
#define MN_ALS_INTT_9216		(12<<4)
#define MN_ALS_INTT_12288		(13<<4)
#define MN_ALS_INTT_18432		(14<<4)
#define MN_ALS_INTT_24576		(15<<4)
//static int mn78_als_intt_value[] = {48, 96, 192, 384, 576, 768, 1152, 1536, 2304, 3072, 4608, 6144, 9216, 12288, 18432, 24576};

#define MN_CYCLE_1			    (0x00)
#define MN_CYCLE_2			    (0x01)
#define MN_CYCLE_4			    (0x02)
#define MN_CYCLE_8			    (0x03)
#define MN_CYCLE_16		        (0x04)
#define MN_CYCLE_32		        (0x05)
#define MN_CYCLE_64		        (0x06)
#define MN_CYCLE_128	        (0x07)
#define MN_CYCLE_256	        (0x08)
//static int mn78_cycle_value[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
#define MN_ALS_PRE              (0 << 6)
#define MN_ALS_STD              (1 << 6)
#define MN_ALS_INT_CHSEL_0	    (0 << 7)
#define MN_ALS_INT_CHSEL_1	    (1 << 7)

#define MN_RS_0			        (0x0<<4)
#define MN_RS_1			        (0x1<<4)
#define MN_RS_2			        (0x2<<4)
#define MN_RS_3			        (0x3<<4)
#define MN_RS_4			        (0x4<<4)
#define MN_RS_5			        (0x5<<4)
#define MN_RS_6		            (0x6<<4)
#define MN_RS_7		            (0x7<<4)
#define MN_RS_8			        (0x8<<4)
#define MN_RS_9			        (0x9<<4)
#define MN_RS_10			    (0xA<<4)
#define MN_RS_11			    (0xB<<4)
#define MN_RS_12			    (0xC<<4)
#define MN_RS_13			    (0xD<<4)
#define MN_RS_14		        (0xE<<4)
#define MN_RS_15		        (0xF<<4)
//static int mn78_als_rs_value[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};
//int mn78_rs_num = sizeof(mn78_als_rs_value)/sizeof(int);

#define MN_ALS_ENH_MODE_1	    (0x00)
#define MN_ALS_ENH_MODE_2	    (0x01)
#define MN_ALS_ENH_MODE_4	    (0x02)
#define MN_ALS_ENH_MODE_8	    (0x03)
#define MN_ALS_MA_DIS           (0<<2)
#define MN_ALS_MA_EN            (1<<2)
#define MN_ALS_SUB_DK_EN        (0<<3)
#define MN_ALS_SUB_DK_DIS       (1<<3)

#define MN_PS_INTT_8			(0<<4)
#define MN_PS_INTT_16			(1<<4)
#define MN_PS_INTT_24			(2<<4)
#define MN_PS_INTT_32			(3<<4)
#define MN_PS_INTT_48			(4<<4)
#define MN_PS_INTT_80			(5<<4)
#define MN_PS_INTT_144			(6<<4)
#define MN_PS_INTT_208			(7<<4)
#define MN_PS_INTT_272			(8<<4)
#define MN_PS_INTT_336			(9<<4)
#define MN_PS_INTT_400			(10<<4)
#define MN_PS_INTT_464			(11<<4)
#define MN_PS_INTT_528		    (12<<4)
#define MN_PS_INTT_656		    (13<<4)
#define MN_PS_INTT_784		    (14<<4)
#define MN_PS_INTT_1040		    (15<<4)
//static int mn78_ps_intt_value[] = {8, 16, 24, 32, 48, 80, 144, 208, 272, 336, 400, 464, 528, 656, 784, 1040};
#define MN_PS_PULSE_16          (0x4)
#define MN_PS_PULSE_32          (0x5)
#define MN_PS_PULSE_64          (0x6)
#define MN_PS_PULSE_128         (0x7)

#define MN_PS_PRE               (0<<6)
#define MN_PS_STD               (1<<6)
#define MN_PS0_INT_CHSEL	    (0<<7)
#define MN_PS1_INT_CHSEL        (1<<7)

#define MN_AVG_16		        (0x4<<4)
#define MN_AVG_32		        (0x5<<4)
#define MN_AVG_64		        (0x6<<4)
#define MN_AVG_128		        (0x7<<4)
#define MN_AVG_256		        (0x8<<4)

#define MN_PS_ENH_MODE_1	    (0x00)
#define MN_PS_ENH_MODE_2	    (0x01)
#define MN_PS_ENH_MODE_4	    (0x02)
#define MN_PS_ENH_MODE_8	    (0x03)

#define MN_IR_ON_CTRL_OFF	    (0<<7)
#define MN_IR_ON_CTRL_ON	    (1<<7)

#define MN_IR_DRIVE_LOW_B       (0x00)
#define MN_IR_DRIVE_15          (0x00)
#define MN_IR_DRIVE_50          (0x01)
#define MN_IR_DRIVE_100         (0x02)
#define MN_IR_DRIVE_200         (0x03)

#define MN_IR_DRIVE_NOR         (4<<2)
#define MN_IR_DRIVE_5_43		(0<<2)
#define MN_IR_DRIVE_8_10		(1<<2)
#define MN_IR_DRIVE_10_52		(2<<2)
#define MN_IR_DRIVE_12_78		(3<<2)
#define MN_IR_DRIVE_14_92		(4<<2)
#define MN_IR_DRIVE_16_98		(5<<2)
#define MN_IR_DRIVE_18_94		(6<<2)
#define MN_IR_DRIVE_20_0	    (7<<2)

#define MN_GAIN0_HH	           (0x00)
#define MN_GAIN0_HIGH          (0x01)
#define MN_GAIN0_MID		   (0x02)
#define MN_GAIN0_LOW		   (0x03)
#define MN_GAIN1_HH	           (0x00<<2)
#define MN_GAIN1_HIGH          (0x01<<2)
#define MN_GAIN1_MID		   (0x02<<2)
#define MN_GAIN1_LOW		   (0x03<<2)

#define MN_INTTY_DISABLE	    (0x00)
#define MN_INTTY_BINARY	        (0x01)
#define MN_INTTY_ACTIVE	        (0x02)
#define MN_INTTY_FRAME	        (0x03)
#define MN_PERIST_1		        (0x00<<2)
#define MN_PERIST_4		        (0x01<<2)
#define MN_PERIST_8		        (0x02<<2)
#define MN_PERIST_16		    (0x03<<2)

//AG
#define MN_AG0_HHG               (0x00<<4)
#define MN_AG0_HG                (0x01<<4)
#define MN_AG0_MG                (0x02<<4)
#define MN_AG0_LG                (0x03<<4)

#define MN_AG1_HHG               (0x00<<6)
#define MN_AG1_HG                (0x01<<6)
#define MN_AG1_MG                (0x02<<6)
#define MN_AG1_LG                (0x03<<6)

#define MN_INT_TRIGGER		(0x01 << 2)
#define MN_INT_CLEAR		(0x00 << 2)

#define MN_CMP_RESET		(0x00 << 1)
#define MN_CMP_RUN			(0x01 << 1)

#define MN_LOCK			(0x01)
#define MN_UN_LOCK		(0x00)

#define MN_REVNO       (0x23)

#define PS_CHANNEL_SIZE	2
struct _ps_data
{
	uint16_t ps0_channel[PS_CHANNEL_SIZE];
	uint16_t ps1_channel[PS_CHANNEL_SIZE];
	uint16_t pdata;
	uint16_t ir_data;
};

struct _ps_factory
{
	uint16_t high_threshold;
	uint16_t low_threshold;
};

struct _ps_setting
{
    bool enable;
    bool ps_first_flag;
    bool polling_mode;
	uint8_t integration_time;
	uint8_t ps_gain0;
	uint8_t ps_gain1;
	uint8_t interrupt_channel_select;
	uint8_t ps_std;
	uint8_t ps_pulse;
	uint8_t ps_avg;
	uint8_t persist;
	uint8_t interrupt_type;
	uint8_t enh_mode;
	uint8_t ir_on_control;
	uint8_t ir_drive;
	uint8_t ir_drive_tune;
	uint16_t ps_offset;
	uint16_t high_threshold;
	uint16_t low_threshold;
	uint8_t sat;
	uint8_t ps0_sat;
	uint8_t ps1_sat;
	uint8_t compare_high;
	uint8_t compare_low;
	uint8_t interrupt_flag;
	uint8_t compare_reset;
	uint8_t lock;
	uint8_t ps_ir0b23_sel;
	uint8_t ps_ir3_en;
	uint8_t ps_ir2_en;
	uint8_t ps_ir1_en;
	uint8_t ps_ir0b_en;
	uint8_t ps_ir0a_en;
	uint16_t ps_max_ct;
	struct _ps_data data;
};

#define ALS_CHANNEL_SIZE	4
struct _als_data
{
	uint16_t channel[ALS_CHANNEL_SIZE];
	uint32_t lux;
};

struct _als_factory
{
	uint16_t lux_per_count;
	uint16_t lux_per_lux;
};
struct _als_setting
{
    bool enable;
    bool polling_mode;
	uint8_t report_type;
	uint8_t integration_time;
	uint8_t als_gain0;
	uint8_t als_gain1;
	uint8_t cycle;
	uint8_t als_std;
	uint8_t interrupt_channel_select;
    uint8_t interrupt_type;
    uint8_t persist;
    uint8_t als_rs;
	uint8_t enh_mode;
	uint8_t ma_en;
	uint8_t als_cal_mode;
	uint16_t als_offset;
	uint16_t als_intr_percent;
	uint16_t high_threshold;
	uint16_t low_threshold;
	uint8_t als_sat;
	uint8_t compare_high;
	uint8_t compare_low;
	uint8_t interrupt_flag;
	uint8_t compare_reset;
	uint8_t lock;
	uint16_t dyn_intt_raw;
	uint8_t dk_period_en;
	uint8_t als_pixel_mode;
	uint8_t dk_period;
	uint8_t als_clr_sel;
	uint8_t als_dk_en;
	uint8_t als_pixel_sel;
	uint8_t als_pixel_en;
	uint8_t als_clr_en;
	uint8_t als_ir0b_en;
	uint8_t als_ir0a_en;
	//auto gain
	uint8_t als_ag_en;
	uint8_t als_ag_3stage;
	uint8_t als_ag_ch0sat_en;
	uint8_t als0_ag_ll;
	uint8_t als1_ag_ll;
	uint8_t als0_ag_l;
	uint8_t als1_ag_l;
	uint8_t als0_ag_m;
	uint8_t als1_ag_m;
	uint8_t als0_ag_h;
	uint8_t als1_ag_h;
	uint8_t als_aintt_ll;
	uint8_t als_aintt_l;
	uint8_t als_aintt_m;
	uint8_t als_aintt_h;
	uint8_t als_aenh_ll;
	uint8_t als_aenh_l;
	uint8_t als_aenh_m;
	uint8_t als_aenh_h;
	uint8_t als_acycle_ll;
	uint8_t als_acycle_l;
	uint8_t als_acycle_m;
	uint8_t als_acycle_h;
    uint16_t als_ag_h2m_thd;
	uint16_t als_ag_m2h_thd;
	uint16_t als_ag_m2l_thd;
	uint16_t als_ag_l2m_thd;
	uint16_t als_ag_l2ll_thd;
	uint16_t als_ag_ll2l_thd;
    uint8_t als_aenh_base;
	uint8_t als_aintt_base;
	uint8_t als_ag0_base;
	uint8_t als_ag1_base;
	uint8_t als_acycle_base;
    uint8_t als_aenh_current;
    uint8_t als_aintt_current;
    uint8_t als_ag0_current;
    uint8_t als_ag1_current;
    uint8_t als_acycle_current;
    uint8_t als_ag_ch0_rs_weight;
    uint8_t als_ag_ch1_rs_weight;
    uint8_t als_ag_ch2_rs_weight;
    uint8_t als_ag_err_flag;
	struct _als_data data;
	struct _als_factory factory;
};

typedef struct _sensor
{
	uint8_t wait;
	uint8_t wait_clk;
	uint8_t mode;
	uint8_t interrupt_control;
	struct _ps_setting ps;
	struct _als_setting als;
	uint16_t revno;
}mn78_optical_sensor;

typedef enum {
    PS_NEAR,
    PS_FAR
}ps_report_status_t;

#define L_SENSOR_LTHD			1000
#define L_SENSOR_HTHD			1000
#define L_SENSOR_LTHD_TRIGER    1001


/*REG Table*/
#define    DEVREG_ENABLE            0x00
#define    DEVREG_INT_CTRL          0x01
#define    DEVREG_RESET             0x02
#define    DEVREG_DARK_FRMAE        0x03
#define    DEVREG_ALS_PIXEL         0x04
#define    DEVREG_PS_PIXEL          0x05
#define    DEVREG_REV_ID            0x06
#define    DEVREG_CHIP_ID           0x07 //REG0x07 not defined in spec
#define    DEVREG_FALL_TIME         0x08
#define    DEVREG_PFALL_TIME        0x09
#define    DEVREG_ALS_CONFIG        0x10
#define    DEVREG_ALS_FILT          0x11
#define    DEVREG_ALS_INT           0x12
#define    DEVREG_ALS_ENH           0x13
#define    DEVREG_ALS_OFSL          0x16
#define    DEVREG_ALS_OFSH          0x17
#define    DEVREG_PS_CONFIG         0x20
#define    DEVREG_PS_PULSE          0x21
#define    DEVREG_PS_INT            0x22
#define    DEVREG_PS_ENH            0x23
#define    DEVREG_LED_CONFIG        0x24
#define    DEVREG_PS_OFSL           0x26
#define    DEVREG_PS_OFSH           0x27
#define    DEVREG_ALS_ILTL          0x30
#define    DEVREG_ALS_ILTH          0x31
#define    DEVREG_ALS_IHTL          0x32
#define    DEVREG_ALS_IHTH          0x33
#define    DEVREG_ALS_STATUS        0x34
#define    DEVREG_ALS_SAT           0x35
#define    DEVREG_C0DATAL           0x36
#define    DEVREG_C0DATAH           0x37
#define    DEVREG_C1DATAL           0x38
#define    DEVREG_C1DATAH           0x39
#define    DEVREG_C2DATAL           0x3A
#define    DEVREG_C2DATAH           0x3B
#define    DEVREG_C3DATAL           0x3C
#define    DEVREG_C3DATAH           0x3D
#define    DEVREG_PS_ILTL           0x40
#define    DEVREG_PS_ILTH           0x41
#define    DEVREG_PS_IHTL           0x42
#define    DEVREG_PS_IHTH           0x43
#define    DEVREG_PS_STATUS         0x44
#define    DEVREG_PS_SAT            0x45
#define    DEVREG_PS1_SAT           0x46
#define    DEVREG_PS_ADATAL         0x47
#define    DEVREG_PS_ADATAH         0x48
#define    DEVREG_PS_RDATAL         0x49
#define    DEVREG_PS_RDATAH         0x4A
#define    DEVREG_PS1_ADATAL        0x4B
#define    DEVREG_PS1_ADATAH        0x4C
#define    DEVREG_PS1_RDATAL        0x4D
#define    DEVREG_PS1_RDATAH        0x4E
//AG Config
#define    DEVREG_ALS_AG_CFG_LL     0x50
#define    DEVREG_ALS_AG_FILT_LL    0x51
#define    DEVREG_ALS_AG_CFG_L      0x52
#define    DEVREG_ALS_AG_FILT_L     0x53
#define    DEVREG_ALS_AG_CFG_M      0x54
#define    DEVREG_ALS_AG_FILT_M     0x55
#define    DEVREG_ALS_AG_CFG_H      0x56
#define    DEVREG_ALS_AG_FILT_H     0x57
#define    DEVREG_ALS_AG_H2M_THL    0x58
#define    DEVREG_ALS_AG_H2M_THH    0x59
#define    DEVREG_ALS_AG_M2H_THL    0x5A
#define    DEVREG_ALS_AG_M2H_THH    0x5B
#define    DEVREG_ALS_AG_M2L_THL    0x5C
#define    DEVREG_ALS_AG_M2L_THH    0x5D
#define    DEVREG_ALS_AG_L2M_THL    0x5E
#define    DEVREG_ALS_AG_L2M_THH    0x5F
#define    DEVREG_ALS_AG_L2LL_THL   0x60
#define    DEVREG_ALS_AG_L2LL_THH   0x61
#define    DEVREG_ALS_AG_LL2L_THL   0x62
#define    DEVREG_ALS_AG_LL2L_THH   0x63
#define    DEVREG_ALS_AG_INFO       0x64
#endif
#endif