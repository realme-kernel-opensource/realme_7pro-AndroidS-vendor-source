#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include "../AKM_Library/akm_apis.h"
#include "akm_wrapper.h"
#include "AKM_Log.h"

static int init_flag = 0;
static bool hasGyro_flag = 0;

int akm_wrapper_init(bool has_gyro,
    mag_fusion_output initial_result)
{
    UNUSED_VAR(initial_result);

    int ret;
    int32_t akm_algo;

    hasGyro_flag = has_gyro;

    if(has_gyro)
        akm_algo = AKM_ALGO_9D;
    else
        akm_algo = AKM_ALGO_6D;

    if (init_flag == 0) {
        mag_fusion_exit_island();
        ret = AKM_LibraryInit(0x05, 0, akm_algo);

        if(ret == AKM_SUCCESS)
            init_flag = 1;
    }

    return 0;
}

int akm_wrapper_deinit(void)
{
    return 0;
}

int akm_wrapper_start()
{
    int ret = 0;

    if (init_flag == 0) {
        akm_wrapper_init(hasGyro_flag, (mag_fusion_output) {
            .bias = {0, 0, 0},
            .accuracy = 0,
            .result_changed = false
        });
    }

    if(init_flag == 1) {
        mag_fusion_exit_island();
        ret = AKM_LoadAndStart();
    }

    return ret;
}

int akm_wrapper_stop()
{
    int ret = 0;

    if(init_flag == 1) {
        mag_fusion_exit_island();
        ret = AKM_StopAndSave();
    }

    return ret;
}

/*Unit:
 *Mag---uT, Acc---m/s2, Gyro---rad/s
 *ts----us
 */
//int akm_wrapper_data_update(sns_sensor_instance *inst,float x, float y, float z, INPUT_TYPE type, int64_t ts)
int akm_wrapper_data_update(mag_fusion_data_input *input,
    mag_fusion_output *output)
{
    struct AKM_SENSOR_DATA sensordata;

    int ret;

    if (init_flag == 0) {
        akm_wrapper_init(hasGyro_flag, (mag_fusion_output) {
            .bias = {0, 0, 0},
            .accuracy = 0,
            .result_changed = false
        });

        akm_wrapper_start();
        return 0;
    }

    sensordata.time_stamp = (int64_t)SNS_TIME_CONVERT_TO_US(input->timestamp);

    if(input->type == MF_INPUT_TYPE_ACC) {
        sensordata.u.v[0] = input->data[0];
        sensordata.u.v[1] = input->data[1];
        sensordata.u.v[2] = input->data[2];
        sensordata.stype = AKM_VT_ACC;
    } else if(input->type == MF_INPUT_TYPE_MAG) {
        sensordata.u.v[0] = input->data[0];
        sensordata.u.v[1] = input->data[1];
        sensordata.u.v[2] = input->data[2];
        sensordata.stype = AKM_VT_MAG;
    } else if(input->type == MF_INPUT_TYPE_GYR) {
        sensordata.u.v[0] = input->data[0];
        sensordata.u.v[1] = input->data[1];
        sensordata.u.v[2] = input->data[2];
        sensordata.stype = AKM_VT_GYR;
    } else {
        return AKM_ERROR;
    }

    sensordata.status[0] = 0x01;
    sensordata.status[1] = 0X00;

    mag_fusion_exit_island();
    ret = AKM_SetData(&sensordata);

    if(ret != 0) {
        MAG_FUSION_LOGI("akm_wrapper_data_update: AKM_SetData failed with code %d", ret);
        return ret;
    }

    output->result_changed = 0;

    if(input->type == MF_INPUT_TYPE_MAG) {
        int ret;
        int32_t accuracy;
        float32_t sensor_result[6] = {0};
        //Get Mag : uT
        mag_fusion_exit_island();
        ret = AKM_GetData(sensor_result, AKM_VT_MAG, &(accuracy));
        //set output(bias)
        output->bias[0] = sensor_result[3];
        output->bias[1] = sensor_result[4];
        output->bias[2] = sensor_result[5];

        switch(accuracy) {
        case 0:
            output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
            break;

        case 1:
            output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_LOW;
            break;

        case 2:
            output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_MEDIUM;
            break;

        case 3:
            output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
            break;
        }

        output->result_changed = 1;
    }

    return ret;
}

// void akm_wrapper_get_mag(OUTPUT_DATA *data)
// {
// 	int ret;
//     int32_t accuracy;
// 	float32_t sensor_result[6] = {0};
//     //Get Mag : uT
// 	ret = AKM_GetData(sensor_result, AKM_VT_MAG, &(accuracy));
// 	//mag after calibration
//     data->data_output[0] = sensor_result[0];
//     data->data_output[1] = sensor_result[1];
//     data->data_output[2] = sensor_result[2];
// 	//mag before calibration
// 	data->data_output[3] = sensor_result[0] + sensor_result[3];
//     data->data_output[4] = sensor_result[1] + sensor_result[4];
//     data->data_output[5] = sensor_result[2] + sensor_result[5];
// 	//mag bias
// 	data->data_output[6] = sensor_result[3];
//     data->data_output[7] = sensor_result[4];
//     data->data_output[8] = sensor_result[5];

//     data->accuracy = accuracy;
// }

// void akm_wrapper_get_ori(OUTPUT_DATA *data)
// {
// 	int ret;
//     int32_t accuracy;
// 	float32_t sensor_result[6] = {0};
//     //Get Ori ：deg
// 	ret = AKM_GetData(sensor_result, AKM_VT_ORI, &(accuracy));

//     data->data_output[0] = sensor_result[0];
//     data->data_output[1] = sensor_result[1];
//     data->data_output[2] = sensor_result[2];
//     data->accuracy = accuracy;
// }

// void akm_wrapper_get_gravity(OUTPUT_DATA *data)
// {
// 	int ret;
//     int32_t accuracy;
// 	float32_t sensor_result[6] = {0};
//     //Get gravity ：m/s2
// 	ret = AKM_GetData(sensor_result, AKM_VT_GRAVITY, &(accuracy));

//     data->data_output[0] = sensor_result[0];
//     data->data_output[1] = sensor_result[1];
//     data->data_output[2] = sensor_result[2];
//     data->accuracy = accuracy;
// }

// void akm_wrapper_get_lacc(OUTPUT_DATA *data)
// {
// 	int ret;
//     int32_t accuracy;
// 	float32_t sensor_result[6] = {0};
//     //Get lacc ：m/s2
// 	ret = AKM_GetData(sensor_result, AKM_VT_LACC, &(accuracy));

//     data->data_output[0] = sensor_result[0];
//     data->data_output[1] = sensor_result[1];
//     data->data_output[2] = sensor_result[2];
//     data->accuracy = accuracy;
// }

// void akm_wrapper_get_quat(OUTPUT_DATA *data)
// {
// 	int ret;
//     int32_t accuracy;
// 	float32_t sensor_result[6] = {0};
//     //Get quat ：unit 1
// 	ret = AKM_GetData(sensor_result, AKM_VT_QUAT, &(accuracy));

//     data->data_output[0] = sensor_result[0];
//     data->data_output[1] = sensor_result[1];
//     data->data_output[2] = sensor_result[2];
//     data->data_output[3] = sensor_result[3];
//     data->accuracy = accuracy;
// }

mag_fusion_api akm_fusion_api = {
    .algo_init = akm_wrapper_init,
    .algo_deinit = akm_wrapper_deinit,
    .algo_start = akm_wrapper_start,
    .algo_stop = akm_wrapper_stop,
    .algo_update = akm_wrapper_data_update,
    .algo_get_debug_info = NULL
};

