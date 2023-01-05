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
#include "sensor_cfg/sns_bmi26x_sensor_cfg.h"


/*! name  Global array that stores the feature input configuration of BMI260 */
bmi26x_feature_config_t bmi260_feat_in[] = {

    //@0x0
    { .type = BMI26X_CONFIG_INDEX_CONFIG_ID,     .page = BMI26X_PAGE_1, .start_addr = BMI26X_CONFIG_ID_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 15, .bit_size_of_feature = 16,
      .block_size = 2
    },

    //@0x009
    { .type = BMI26X_CONFIG_INDEX_MAX_BURST_LEN,     .page = BMI26X_PAGE_1, .start_addr = BMI26X_MAX_BURST_LEN_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 7, .bit_size_of_feature = 8,
      .block_size = 2
    },

    { .type = BMI26X_CONFIG_INDEX_GYR_TRIGGER_SELECT, .page = BMI26X_PAGE_1, .start_addr = BMI26X_SELF_OFF_CORR_STRT_ADDR,
      .bit_start_in_word_adress = 8, .bit_end_in_word_adress = 8, .bit_size_of_feature = 1,
      .block_size = 2
    },

    { .type = BMI26X_CONFIG_INDEX_GYR_ABORT_CRT,     .page = BMI26X_PAGE_1,  .start_addr = BMI26X_GYR_ABORT_CRT,
      .bit_start_in_word_adress = 9, .bit_end_in_word_adress = 9, .bit_size_of_feature = 1,
      .block_size = 2
    },

    // @0x00A
    { .type = BMI26X_CONFIG_INDEX_AXIS_MAP,          .page = BMI26X_PAGE_1,  .start_addr = BMI26X_AXIS_REMAP_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 8, .bit_size_of_feature = 9,
      .block_size = 2
    },

    { .type = BMI26X_CONFIG_INDEX_GYR_SELF_TEST_OFF, .page = BMI26X_PAGE_1,  .start_addr = BMI26X_AXIS_GYR_SELF_OFF_START_ADSDR,
      .bit_start_in_word_adress = 9, .bit_end_in_word_adress = 9, .bit_size_of_feature = 1,
      .block_size = 2
    },

    { .type = BMI26X_CONFIG_INDEX_NVM_PROG_PREP,     .page = BMI26X_PAGE_1,  .start_addr = BMI26X_NVM_PROG_PREP_START_ADSDR,
      .bit_start_in_word_adress = 10, .bit_end_in_word_adress = 10, .bit_size_of_feature = 1,
      .block_size = 2
    },

    //0x00B - 0x00C
    { .type = BMI26X_CONFIG_INDEX_ANY_MOTION,        .page = BMI26X_PAGE_1,  .start_addr = BMI26X_ANY_MOT_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 26, .bit_size_of_feature = 26,
      .block_size = 4
    },

    //0x00D-0x00F
    //ratio.x:{0,10}@0x00D, ratio.y:{0,10}@0x00E, ratio.z:{0,10}, enable:{11:11}@0x00F
    { .type = BMI26X_CONFIG_INDEX_GYRO_GAIN_UPDATE,  .page = BMI26X_PAGE_1,  .start_addr = BMI26X_GYR_USER_GAIN_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 0, .bit_size_of_feature = 34,
      .block_size = 6
    },


    // @page 2
    { .type = BMI26X_CONFIG_INDEX_STEP_COUNTER,      .page = BMI26X_PAGE_2,  .start_addr = BMI26X_STEP_COUNTER_OFFSET,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 12, .bit_size_of_feature = 13,
      .block_size = 2
    },

    //0x010
    //00x11
    { .type = BMI26X_CONFIG_INDEX_STEP_DETECTOR,     .page = BMI26X_PAGE_2,  .start_addr = BMI26X_STEP_DETECTOR_OFFSET,
      .bit_start_in_word_adress = 11, .bit_end_in_word_adress = 11, .bit_size_of_feature = 1,
      .block_size = 2
    },

    // 0x13
    // 0x13
    { .type = BMI26X_CONFIG_INDEX_LP_FILTER,     .page = BMI26X_PAGE_2,  .start_addr = BMI26X_LP_FILTER_OFFSET,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 7, .bit_size_of_feature = 8,
      .block_size = 1
    },

    { .type = BMI26X_CONFIG_INDEX_STEP_ACTIVITY,     .page = BMI26X_PAGE_2,  .start_addr = BMI26X_STEP_COUNTER_OFFSET,
      .bit_start_in_word_adress = 13, .bit_end_in_word_adress = 13, .bit_size_of_feature = 1,
      .block_size = 2
    },
#if BMI26X_CONFIG_ENABLE_LOWG
    // @page 3
    // 0x18 -0x1F
    { .type = BMI26X_CONFIG_INDEX_LOW_G,        .page = BMI26X_PAGE_3,  .start_addr = BMI26X_FEATURE_LOWG_START_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 128, .bit_size_of_feature = 128,
      .block_size = 16
    },
#endif
    // @page 5
    // 0x28 -0x1F
    { .type = BMI26X_CONFIG_INDEX_DOUBLE_TAP_CFG_1,        .page = BMI26X_PAGE_3,  .start_addr = BMI26X_FEATURE_LOWG_START_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 128, .bit_size_of_feature = 128,
      .block_size = 16
    },

    { .type = BMI26X_CONFIG_INDEX_DOUBLE_TAP_CFG_2,        .page = BMI26X_PAGE_4,  .start_addr = BMI26X_FEATURE_LOWG_START_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 128, .bit_size_of_feature = 128,
      .block_size = 16
    }

};


/*! name  Global array that stores the feature out configuration of BMI260 */
bmi26x_feature_config_t bmi260_feat_out[] = {
    //0x000, 0x001
    { .type = BMI26X_CONFIG_INDEX_STEP_COUNTER,     .page = BMI26X_PAGE_0, .start_addr = BMI26X_STEP_CNT_OUT_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 0, .bit_size_of_feature = 32,
      .block_size = 4
    },

    //0x002
    { .type = BMI26X_CONFIG_INDEX_STEP_ACTIVITY,    .page = BMI26X_PAGE_0, .start_addr = BMI26X_STEP_ACT_OUT_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 1, .bit_size_of_feature = 3,
      .block_size = 2
    },

    //0x003
    { .type = BMI26X_CONFIG_INDEX_GYRO_GAIN_UPDATE, .page = BMI26X_PAGE_0, .start_addr = BMI26X_GYR_USER_GAIN_OUT_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 5, .bit_size_of_feature = 6,
      .block_size = 2
    },

    //0x006
    { .type = BMI26X_CONFIG_INDEX_OUT_CROSS_SENSE,  .page = BMI26X_PAGE_0, .start_addr = BMI26X_CROSS_SENSE_STRT_ADDR,
      .bit_start_in_word_adress = 0, .bit_end_in_word_adress = 6, .bit_size_of_feature = 7,
      .block_size = 2
    },

    //0x007
    { .type = BMI26X_CONFIG_INDEX_NVM_STATUS,       .page = BMI26X_PAGE_0, .start_addr = BMI26X_NVM_VFRM_OUT_STRT_ADDR,
      .bit_start_in_word_adress = 8, .bit_end_in_word_adress = 12, .bit_size_of_feature = 5,
      .block_size = 2
    },

    { .type = BMI26X_CONFIG_INDEX_VFRM_STATUS,      .page = BMI26X_PAGE_0, .start_addr = BMI26X_NVM_VFRM_OUT_STRT_ADDR,
      .bit_start_in_word_adress = 13, .bit_end_in_word_adress = 15, .bit_size_of_feature = 3,
      .block_size = 2
    },
};

int bmi260_sensor_feature_confure_in_size(void)
{
    return (int) (sizeof(bmi260_feat_in) / sizeof(bmi26x_feature_config_t));
}

int bmi260_sensor_feature_confure_out_size(void)
{
    return (int) (sizeof(bmi260_feat_out) / sizeof(bmi26x_feature_config_t));
}
