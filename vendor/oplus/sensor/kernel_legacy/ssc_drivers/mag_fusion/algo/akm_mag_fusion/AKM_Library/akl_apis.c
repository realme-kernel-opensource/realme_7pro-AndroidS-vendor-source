/******************************************************************************
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 ******************************************************************************/
#include "akl_apis.h"          /* API decleration */
#include "akl_smart_compass.h" /* Definition of struct */
#include "akl_measure.h"
#include "akl_lib_interface.h"
#include "akm_filter.h"

/*! Identify the nv data. */
#define AKL_NV_MAGIC_NUMBER   (0xdeadbeefU)
#define AKL_INI_MAGIC_NUMBER  (0xbeefcafeU)
/*! The number of default form factor. */
#define AKL_DEFAULT_FORM_NUM  1

#define DEG2RAD             (AKSC_PI/180.f)
#define RAD2DEG             (180.f/AKSC_PI)

//Convert from AKSC(1=0.06uT) to uT
#define MAG_AKSC_TO_UT(x)    ((float32_t)(x) * 0.06f)

#define ACC_1G_IN_MS2       (9.80665f)
#define ACC_1G_IN_Q16         (9.80665f * 65536.0f)
//Convert from AKSC(720=1G) to m/s2
#define ACC_AKSC_TO_MS2(x)    (((float32_t)(x)/ 720.0f)* ACC_1G_IN_MS2)
//Conver from m/s2 to AKSC(720=1G)
#define ACC_MS2_TO_AKSC(x)    (((float32_t)(x) * 720.0f) / ACC_1G_IN_MS2)

//Convert from AKSC(d/s in Q4) to d/s
#define GYR_AKSC_TO_DEG(x)    ((float32_t)(x)/16.0f)
//Convert from deg to AKSC(d/s in Q4)
#define GYR_RAD_TO_AKSC(x)    ((float32_t)(x) * RAD2DEG * 16.0f)

//Convert from AKSC(deg in Q6) to deg
#define ORI_AKSC_TO_DEG(x)    ((float32_t)(x)/ 64.0f)

//The length of measurement and status data
#define AKM_BDATA_SIZE_MAX  10

/***** AKM static functions ***************************************************/
static uint32_t byte_allign(const uint32_t sz)
{
    if (0 >= sz) {
        return (uint32_t)0;
    }

    /* Another method.
     int32_t rem = sz % 4;
     return (rem ? (sz + (4 - rem)) : (sz));
     */
    return ((sz & 0x3) ? ((sz & ~(0x3)) + 0x4) : sz);
}

static int16_t akl_setv_mag(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data)
{
    int32_t   tmp_i32;
    int64_t   tmp_dt;
    int16_t   reg[3];
    int16_t   ret;
    int16     bData[AKM_BDATA_SIZE_MAX];
    int16_t   i;
    float32_t akm_sensitivity;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    if ((mem->m_device == AK8963) ||
        (mem->m_device == AK09912) ||
        (mem->m_device == AK09913) ||
        (mem->m_device == AK09915) ||
        (mem->m_device == AK09916C) ||
        (mem->m_device == AK09916D) ||
        (mem->m_device == AK09918)) {
        akm_sensitivity = (1.f / 0.15f);
    } else if (mem->m_device == AK09911) {
        akm_sensitivity = (1.f / 0.6f);
    } else {
        return AKM_ERR_NOT_SUPPORT;
    }

    for (i = 0; i < 3; i++) {
        tmp_i32 = (int32_t)(data->u.v[i] * akm_sensitivity);

        /* Limit to 16-bit value */
        if ((tmp_i32 < INT16_MIN) || (INT16_MAX < tmp_i32)) {
            return AKM_ERR_INVALID_ARG;
        }

        reg[i] = (int16)tmp_i32;
    }

    /* Inverse decomp, i.e. compose */
    bData[0] = (int16)(data->status[0]);
    bData[1] = (int16)(reg[0] & 0xFF);
    bData[2] = (int16)((reg[0] >> 8) & 0xFF);
    bData[3] = (int16)(reg[1] & 0xFF);
    bData[4] = (int16)((reg[1] >> 8) & 0xFF);
    bData[5] = (int16)(reg[2] & 0xFF);
    bData[6] = (int16)((reg[2] >> 8) & 0xFF);

    if (mem->m_device == AK8963) {
        bData[7] = (int16)(data->status[1]);
    } else if ((mem->m_device == AK09911) ||
        (mem->m_device == AK09912) ||
        (mem->m_device == AK09913) ||
        (mem->m_device == AK09915)) {
        bData[7] = (int16)(0x80);
        bData[8] = (int16)(data->status[1]);
    } else if ((mem->m_device == AK09916C) ||
        (mem->m_device == AK09916D) ||
        (mem->m_device == AK09918)) {
        bData[7] = (int16)(0x80);
        bData[8] = (int16)(data->status[1]);
        bData[9] = (int16)mem->m_pat;
    }

    AKM_MSG_INFO_4("akm_log setv_mag(uT) %d, %d, %d  ts %d", (int32)data->u.v[0], (int32)data->u.v[1], (int32)data->u.v[2], data->time_stamp);

    ret = GetMagneticVectorDecomp(
            bData,
            mem,
            data->time_stamp);

    if (ret != AKRET_PROC_SUCCEED) {
        AKM_MSG_ERR_1("akm_err GetMagneticVectorDecomp failed! ret = %d", ret);
        return AKM_ERROR;
    }

    mem->m_hdt_ag = -1;
    mem->m_gdt_ag = -1;

    /* Calculate hdt_ag */
    /* If m_ts_hvec is 0, it means 'DOEaG is not initialized' */
    if (mem->m_ts_hvec != 0) {
        tmp_dt = data->time_stamp - mem->m_ts_hvec;

        /* Limit to 16-bit value */
        if (tmp_dt > (int64_t)MAXTIME_USEC_IN_Q4) {
            tmp_dt = (int64_t)MAXTIME_USEC_IN_Q4;
        }

        mem->m_hdt_ag = (int16_t)TIME_USEC_TO_AKSC(tmp_dt);
    } else {
        /* negative value means 'not updated' */
        mem->m_hdt_ag = -1;
    }

    AKM_MSG_INFO_1("akm_log setv_mag-hdt_ag(msQ4) = %d", mem->m_hdt_ag);

    if(mem->m_hdt_ag > 0) {
        ret = GetMagneticVectorOffset(mem, data->time_stamp);

        if (ret != AKRET_PROC_SUCCEED) {
            AKM_MSG_ERR_1("akm_err GetMagneticVector Failed ret:%d", ret);
            //return AKM_ERROR;
        }
    }

    /* hvec is successfully updated */
    mem->m_ts_hvec = data->time_stamp;

    return AKM_SUCCESS;
}

static int16_t akl_setv_acc(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data)
{
    int32_t   tmp_i32;
    int16_t   i;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    /* from m/s2 fromat to AKSC format */
    for (i = 0; i < 3; i++) {
        tmp_i32 = (int32_t)ACC_MS2_TO_AKSC(data->u.v[i]);

        if ((tmp_i32 < INT16_MIN) || (INT16_MAX < tmp_i32)) {
            return AKM_ERR_INVALID_ARG;
        }

        mem->v_avec.v[i] = (int16_t)tmp_i32;
    }

#ifdef AKM_ENABLE_DETECT_PHONE_STATE

    //Only 6D mode need detect phone state
    if(mem->m_lib_calib.calib_mode == AKL_MODE_FUSION_6D_PG_ON
        || mem->m_lib_calib.calib_mode == AKL_MODE_FUSION_6D_PG_OFF) {
        akm_detect_phone_state((float32_t)mem->v_avec.v[0], (float32_t)mem->v_avec.v[1], (float32_t)mem->v_avec.v[2]);
    }

#endif
    /* avec is successfully updated */
    mem->m_ts_avec = data->time_stamp;

    AKM_MSG_INFO_4("akm_log setv_acc(720/g) %d, %d, %d ts %d", mem->v_avec.u.x, mem->v_avec.u.y, mem->v_avec.u.z, mem->m_ts_avec);
    return AKM_SUCCESS;
}

static int16_t akl_setv_gyr(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data)
{
    int32_t   tmp_i32;
    int16_t   ret;
    int16_t   i;
    int64_t   tmp_dt;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    //For D9Dv2 lib
    //Calibration need gyro in int16(Q4)
    //cal orientation need gyro in float
    for (i = 0; i < 3; i++) {
        tmp_i32 = (int32_t)GYR_RAD_TO_AKSC(data->u.v[i]);

        /* Limit to 16-bit value */
        if ((tmp_i32 < INT16_MIN) || (INT16_MAX < tmp_i32)) {
            return AKM_ERR_INVALID_ARG;
        }

        mem->v_gvec.v[i] = (int16_t)tmp_i32;
        mem->v_gvecf.v[i] = data->u.v[i] * RAD2DEG;
    }

    mem->m_hdt_ag = -1;
    mem->m_gdt_ag = -1;

    /* calculate gdt_ag */
    if (mem->m_ts_hvec != 0) {
        /* if result is 0, it means 'data is not updated' */
        /* But in the case of DOEaG, negative value means 'not updated' */
        tmp_dt = data->time_stamp - mem->m_ts_hvec;

        /* Limit to 16-bit value */
        if (tmp_dt > (int64_t)MAXTIME_USEC_IN_Q4) {
            tmp_dt = MAXTIME_USEC_IN_Q4;
        }

        mem->m_gdt_ag = (int16_t)TIME_USEC_TO_AKSC(tmp_dt);
    } else {
        mem->m_gdt_ag = -1;
    }

    AKM_MSG_INFO_4("akm_log setv_gyr(d/s*1000) %d, %d, %d ts %d", (int32_t)(mem->v_gvecf.u.x * 1000), (int32_t)(mem->v_gvecf.u.y * 1000),
        (int32_t)(mem->v_gvecf.u.z * 1000), data->time_stamp);
    AKM_MSG_INFO_1("akm_log setv_gyr-dt(us) = %d", data->time_stamp - mem->m_ts_gvec);
    AKM_MSG_INFO_1("akm_log setv_gyr-gdt_ag(msQ4) = %d", mem->m_gdt_ag);

    if(mem->m_gdt_ag > 0) {
        ret = GetMagneticVectorOffset(mem, 0);

        if (ret != AKRET_PROC_SUCCEED) {
            AKM_MSG_ERR_1("akm_err akl_setv_gyr GetMagneticVectorOffset failed! ret = %d", ret);
            //return AKM_ERROR;
        }
    }

    /* gvec is successfully updated */
    mem->m_ts_gvec = data->time_stamp;

    return AKM_SUCCESS;
}

static int32_t akl_get_doelv(const struct AKL_SCL_PRMS *mem)
{
    int32_t lv;

    lv = (int32_t)mem->m_hdst;

    return lv;
}

//Get Mag, Unit: uT
static int16_t akl_getv_mag(
    const struct AKL_SCL_PRMS *mem,
    float32_t                   data[6],
    int32_t                   *status,
    int64_t                   *timestamp)
{
    int16_t   i;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (status == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    if (mem->m_ts_hvec == 0) {
        return AKM_ERR_NOT_YET_CALCULATED;
    }

    for (i = 0; i < 3; i++) {
        data[i] = MAG_AKSC_TO_UT(mem->v_hvec.v[i]);
        data[i + 3] = MAG_AKSC_TO_UT(mem->v_ho.v[i] + mem->v_hbase.v[i]);
    }

    *status = akl_get_doelv(mem);
    *timestamp = mem->m_ts_hvec;

    AKM_MSG_INFO_4("akm_log akl_getv_mag(uT) : %d, %d, %d, Lv: %d", (int32_t)data[0], (int32_t)data[1], (int32_t)data[2], *status);
    AKM_MSG_INFO_3("akm_log akl_getv_mag(uT) offset: %d, %d, %d", (int32_t)data[3], (int32_t)data[4], (int32_t)data[5]);
    return AKM_SUCCESS;
}

//Get Acc, Unit: m/s2
static int16_t akl_getv_acc(
    const struct AKL_SCL_PRMS *mem,
    float32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp)
{
    int16_t   i;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (status == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    if (mem->m_ts_avec == 0) {
        return AKM_ERR_NOT_YET_CALCULATED;
    }

    for (i = 0; i < 3; i++) {
        data[i] = ACC_AKSC_TO_MS2(mem->v_avec.v[i]);
    }

    *status = akl_get_doelv(mem);
    *timestamp = mem->m_ts_avec;

    AKM_MSG_INFO_3("akm_log akl_getv_acc(m/s2*1000): %d, %d, %d", (int32_t)(data[0] * 1000), (int32_t)(data[1] * 1000), (int32_t)(data[2] * 1000));
    return AKM_SUCCESS;
}

//Get Gyro, Unit: rad/s
static int16_t akl_getv_gyr(
    const struct AKL_SCL_PRMS *mem,
    float32_t                   data[6],
    int32_t                   *status,
    int64_t                   *timestamp)
{
    int16_t i;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (status == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    for (i = 0; i < 3; i++) {
        data[i] = mem->v_gvecf.v[i] * DEG2RAD;
    }

    *status = akl_get_doelv(mem);
    *timestamp = mem->m_ts_gvec;

    AKM_MSG_INFO_3("akm_log akl_getv_gyr(rad/s*1000): %d, %d, %d", (int32_t)(data[0] * 1000), (int32_t)(data[1] * 1000), (int32_t)(data[2] * 1000));
    return AKM_SUCCESS;
}

//Get Gravity, Unit: m/s2
static int16_t akl_getv_gravity(
    const struct AKL_SCL_PRMS *mem,
    float32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp)
{
    int16_t   i;
#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (status == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    if (mem->m_ts_ev_d9d == 0) {
        return AKM_ERR_NOT_YET_CALCULATED;
    }

    for (i = 0; i < 3; i++) {
        data[i] = ACC_AKSC_TO_MS2(mem->v_gravity.v[i]);
    }

    *status = akl_get_doelv(mem);
    *timestamp = mem->m_ts_ev_d9d;

    AKM_MSG_INFO_3("akm_log akl_getv_gravity(m/s2*1000): %d, %d, %d", (int32_t)(data[0] * 1000), (int32_t)(data[1] * 1000), (int32_t)(data[2] * 1000));
    return AKM_SUCCESS;
}

//Get linear acc, Unit: m/s2
static int16_t akl_getv_lacc(
    const struct AKL_SCL_PRMS *mem,
    float32_t                  data[3],
    int32_t                   *status,
    int64_t                   *timestamp)
{
    int16_t   i;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (status == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    if (mem->m_ts_ev_d9d == 0) {
        return AKM_ERR_NOT_YET_CALCULATED;
    }

    for (i = 0; i < 3; i++) {
        data[i] = ACC_AKSC_TO_MS2(mem->v_lacc.v[i]);
    }

    *status = akl_get_doelv(mem);
    *timestamp = mem->m_ts_ev_d9d;

    AKM_MSG_INFO_3("akm_log akl_getv_lacc(m/s2*1000): %d, %d, %d", (int32_t)(data[0] * 1000), (int32_t)(data[1] * 1000), (int32_t)(data[2] * 1000));
    return AKM_SUCCESS;
}

//Get Orientation, Unit: deg
static int16_t akl_getv_ori(
    const struct AKL_SCL_PRMS *mem,
    float32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp)
{
    if (mem->m_ts_ev_d9d == 0) {
        return AKM_ERR_NOT_YET_CALCULATED;
    }

    /* Convert unit */
    /* from AKSC Q6 to deg */
    data[0] = ORI_AKSC_TO_DEG(mem->m_theta);
    data[1] = ORI_AKSC_TO_DEG(mem->m_phi180);
    data[2] = ORI_AKSC_TO_DEG(mem->m_eta90);

    *status = akl_get_doelv(mem);
    *timestamp = mem->m_ts_ev_d9d;

    AKM_MSG_INFO_4("akm_log akl_getv_ori(d): %d, %d, %d, Lv: %d", (int32_t)data[0], (int32_t)data[1], (int32_t)data[2], *status);
    return AKM_SUCCESS;
}

//Get Quat, Unit: 1
static int16_t akl_getv_quat(
    const struct AKL_SCL_PRMS *mem,
    float32_t                   data[4],
    int32_t                   *status,
    int64_t                   *timestamp)
{
    if (mem->m_ts_ev_d9d == 0) {
        return AKM_ERR_NOT_YET_CALCULATED;
    }

    /* from AKSC Q14 to Unit */
    data[0] = mem->s_quatf.u.x;
    data[1] = mem->s_quatf.u.y;
    data[2] = mem->s_quatf.u.z;
    data[3] = mem->s_quatf.u.w;

    *status = akl_get_doelv(mem);
    *timestamp = mem->m_ts_ev_d9d;
    AKM_MSG_INFO_4("akm_log akl_getv_quat(*1000): %d, %d, %d, %d", (int32_t)(data[0] * 1000), (int32_t)(data[1] * 1000), (int32_t)(data[2] * 1000),
        (int32_t)(data[3] * 1000));
    return AKM_SUCCESS;
}

/***** AKM public APIs ********************************************************/

/***** Function manual is described in header file. ***************************/
uint32_t AKL_GetParameterSize(const uint8_t max_form)
{
    UNUSED_VAR(max_form);
    return byte_allign(sizeof(struct AKL_SCL_PRMS));
}

uint32_t AKL_GetNVdataSize(const uint8_t max_form)
{
    uint8_t num;

    if (max_form == 0U) {
        num = (uint8_t)AKL_DEFAULT_FORM_NUM;
    } else {
        num = max_form;
    }

    return byte_allign(sizeof(struct AKL_NV_PRMS)) * num;
}

int16_t AKL_SetMode(struct AKL_SCL_PRMS     *mem, uint8_t mode)
{
    uint8_t lib_calib  = mode & 0x0f;
    uint8_t lib_fusion = mode & 0xf0;

    if(lib_calib == AKL_MODE_CALIB_DOEAG) {
        mem->m_lib_calib.calib_mode = AKL_MODE_CALIB_DOEAG;
        mem->m_lib_calib.calib_set_pointer  = AKL_DOEAG_SetPointer;
        mem->m_lib_calib.calib_free_pointer = AKL_DOEAG_FreePointer;
        mem->m_lib_calib.calib_init         = AKL_DOEAG_Init;
        mem->m_lib_calib.calib_calc         = AKL_DOEAG_Calibrate;
        mem->m_lib_calib.calib_set_level    = AKL_DOEAG_SetLevel;
    } else if(lib_calib == AKL_MODE_CALIB_DOEEX) {
        mem->m_lib_calib.calib_mode = AKL_MODE_CALIB_DOEEX;
        mem->m_lib_calib.calib_set_pointer  = AKL_DOEEX_SetPointer;
        mem->m_lib_calib.calib_free_pointer = AKL_DOEEX_FreePointer;
        mem->m_lib_calib.calib_init         = AKL_DOEEX_Init;
        mem->m_lib_calib.calib_calc         = AKL_DOEEX_Calibrate;
        mem->m_lib_calib.calib_set_level    = AKL_DOEEX_SetLevel;
    } else if(lib_calib == AKL_MODE_CALIB_DOE) {
        mem->m_lib_calib.calib_mode = AKL_MODE_CALIB_DOE;
        mem->m_lib_calib.calib_set_pointer  = AKL_DOE_SetPointer;
        mem->m_lib_calib.calib_free_pointer = AKL_DOE_FreePointer;
        mem->m_lib_calib.calib_init         = AKL_DOE_Init;
        mem->m_lib_calib.calib_calc         = AKL_DOE_Calibrate;
        mem->m_lib_calib.calib_set_level    = AKL_DOE_SetLevel;
    } else {
        AKM_MSG_ERR_0("akm_err AKL_SetMode Unknown calib mode");
        return AKM_ERROR;
    }

    if(lib_fusion == AKL_MODE_FUSION_9D) {
        mem->m_lib_fusion.fusion_mode = AKL_MODE_FUSION_9D;
        mem->m_lib_fusion.fusion_set_pointer     = AKL_D9D_SetPointer;
        mem->m_lib_fusion.fusion_free_pointer    = AKL_D9D_FreePointer;
        mem->m_lib_fusion.fusion_init            = AKL_D9D_Init;
        mem->m_lib_fusion.fusion_calc            = AKL_D9D_CalcFusion;
        mem->m_lib_fusion.fusion_pg_set_pointer  = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_free_pointer = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_init         = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_calc         = AKL_NULL_Function;
    } else if(lib_fusion == AKL_MODE_FUSION_6D_PG_ON) {
        mem->m_lib_fusion.fusion_mode = lib_fusion;
        mem->m_lib_fusion.fusion_set_pointer     = AKL_D6D_SetPointer;
        mem->m_lib_fusion.fusion_free_pointer    = AKL_D6D_FreePointer;
        mem->m_lib_fusion.fusion_init            = AKL_D6D_Init;
        mem->m_lib_fusion.fusion_calc            = AKL_D6D_CalcFusion;
        mem->m_lib_fusion.fusion_pg_set_pointer  = AKL_PG_SetPointer;
        mem->m_lib_fusion.fusion_pg_free_pointer = AKL_PG_FreePointer;
        mem->m_lib_fusion.fusion_pg_init         = AKL_PG_Init;
        mem->m_lib_fusion.fusion_pg_calc         = AKL_PG_CalcAngularRate;
    } else if(lib_fusion == AKL_MODE_FUSION_6D_PG_OFF) {
        mem->m_lib_fusion.fusion_mode = lib_fusion;
        mem->m_lib_fusion.fusion_set_pointer     = AKL_D6D_SetPointer;
        mem->m_lib_fusion.fusion_free_pointer    = AKL_D6D_FreePointer;
        mem->m_lib_fusion.fusion_init            = AKL_D6D_Init;
        mem->m_lib_fusion.fusion_calc            = AKL_D6D_CalcFusion;
        mem->m_lib_fusion.fusion_pg_set_pointer  = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_free_pointer = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_init         = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_calc         = AKL_NULL_Function;
    } else if(lib_fusion == AKL_MODE_FUSION_CLOSE) {
        mem->m_lib_fusion.fusion_mode = lib_fusion;
        mem->m_lib_fusion.fusion_set_pointer     = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_free_pointer    = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_init            = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_calc            = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_set_pointer  = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_free_pointer = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_init         = AKL_NULL_Function;
        mem->m_lib_fusion.fusion_pg_calc         = AKL_NULL_Function;
    } else {
        AKM_MSG_ERR_0("akm_err AKL_SetMode Unknown fusion mode");
        return AKM_ERROR;
    }

    return AKM_SUCCESS;
}
int16_t AKL_Init(
    struct AKL_SCL_PRMS                 *mem,
    const struct AKL_CERTIFICATION_INFO *cert,
    const uint8_t                       max_form)
{
    int fret = AKM_SUCCESS;
#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (cert == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif
    //Avoid that just call start, never call stop
    AKL_Release(mem);

    /* Initialize parameter */
    InitAKSCPRMS(mem);

    /* save number of maximum form. */
    if (max_form == 0U) {
        mem->m_maxForm = (uint8_t)AKL_DEFAULT_FORM_NUM;
    } else {
        mem->m_maxForm = max_form;
    }

    mem->s_cert = *cert;
    mem->init = (uint32_t)AKL_INI_MAGIC_NUMBER;

    fret = mem->m_lib_calib.calib_set_pointer(mem);
    fret += mem->m_lib_fusion.fusion_set_pointer(mem);
    fret += mem->m_lib_fusion.fusion_pg_set_pointer(mem);

    if(fret != AKM_SUCCESS) {
        AKM_MSG_ERR_0("akm_err AKL_Init Failed");
        AKL_Release(mem);
        return fret;
    }

    return fret;
}
int16_t AKL_Release(struct AKL_SCL_PRMS *mem)
{
    mem->m_lib_calib.calib_free_pointer(mem);
    mem->m_lib_fusion.fusion_free_pointer(mem);
    mem->m_lib_fusion.fusion_pg_free_pointer(mem);
    return AKM_SUCCESS;
}


int16_t AKL_StartMeasurement(struct AKL_SCL_PRMS *mem)
{
#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    /* Init SmartCompass library functions. */
    if (InitMeasure(mem) != AKRET_PROC_SUCCEED) {
        return AKM_ERROR;
    }

#ifdef AKM_ENABLE_DETECT_PHONE_STATE
    akm_detect_phone_state_init();
#endif
    return AKM_SUCCESS;
}


int16_t AKL_StopMeasurement(const struct AKL_SCL_PRMS *mem)
{
    UNUSED_VAR(mem);
    return AKM_SUCCESS;
}


AKM_DEVICE_TYPE AKL_GetDeviceType(int device_id)
{
    AKM_DEVICE_TYPE result = NO_AKM_DEVICE;

    switch(device_id) {
    case 0x05:
        result = AK09911;
        break;

    case 0x04:
        result = AK09912;
        break;

    case 0x08:
        result = AK09913;
        break;

    case 0x10:
        result = AK09915;
        break;

    case 0x09:
        result = AK09916C;
        break;

    case 0x0b:
        result = AK09916D;
        break;

    case 0x0c:
        result = AK09918;
        break;

    default:
        result = NO_AKM_DEVICE;
    }

    return result;

}

int16_t AKL_SetVector(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA data[],
    const uint8_t                num)
{
    uint8_t i;
    int16_t ret;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        AKM_MSG_ERR_0("akm_err AKL_SetVector");
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        AKM_MSG_ERR_0("akm_err AKL_SetVector");
        return AKM_ERR_INVALID_ARG;
    }

#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        AKM_MSG_ERR_0("akm_err AKL_SetVector");
        return AKM_ERR_INVALID_ARG;
    }

    for (i = 0U; i < num; i++) {
        switch (data[i].stype) {
        case AKM_VT_MAG:
            if (mem->m_ts_hvec == data[i].time_stamp) {
                return AKM_SUCCESS;
            }

            ret = akl_setv_mag(mem, &data[i]);

            break;

        case AKM_VT_ACC:
            if (mem->m_ts_avec == data[i].time_stamp) {
                return AKM_SUCCESS;
            }

            ret = akl_setv_acc(mem, &data[i]);
            break;

        case AKM_VT_GYR:
            if (mem->m_ts_gvec == data[i].time_stamp) {
                return AKM_SUCCESS;
            }

            ret = akl_setv_gyr(mem, &data[i]);
            break;

        default:
            ret = AKM_ERR_NOT_SUPPORT;
            break;
        }

        if (ret != AKM_SUCCESS) {
            AKM_MSG_ERR_1("akm_err AKL_SetVector set data failed ret:%d", ret);
            return ret;
        }

    }

    return AKM_SUCCESS;
}


int16_t AKL_CalcFusion(struct AKL_SCL_PRMS *mem)
{

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    if (CalcDirection(mem) != AKM_SUCCESS) {
        AKM_MSG_ERR_0("akm_err CalcDirection Failed");
        return AKM_ERROR;
    }

    if (CalcAngularRate(mem) != AKM_SUCCESS) {
        AKM_MSG_ERR_0("akm_err CalcAngularRate Failed");
        return AKM_ERROR;
    }

    return AKM_SUCCESS;

}


int16_t AKL_GetVector(
    const AKM_VECTOR_TYPE     vtype,
    const struct AKL_SCL_PRMS *mem,
    float32_t                   *data,
    uint8_t                   size,
    int32_t                   *status,
    int64_t                   *timestamp)
{
    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    if((vtype & AKM_VT_OUTPUT9D) && !(mem->m_lib_fusion.fusion_mode & AKL_MODE_FUSION_OUTPUT9D))
        return AKM_ERR_INVALID_ARG;

    switch (vtype) {
    case AKM_VT_MAG:

        if (AKM_VT_MAG_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }

        return akl_getv_mag(mem, data, status, timestamp);

    case AKM_VT_ACC:

        if (AKM_VT_ACC_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }

        return akl_getv_acc(mem, data, status, timestamp);

    case AKM_VT_ORI:

        if (AKM_VT_ORI_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }

        return akl_getv_ori(mem, data, status, timestamp);

    case AKM_VT_QUAT:

        if (AKM_VT_QUAT_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }

        return akl_getv_quat(mem, data, status, timestamp);

    case AKM_VT_GYR:

        if (AKM_VT_GYR_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }

        return akl_getv_gyr(mem, data, status, timestamp);

    case AKM_VT_GRAVITY:

        if (AKM_VT_GRAVITY_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }

        return akl_getv_gravity(mem, data, status, timestamp);

    case AKM_VT_LACC:

        if (AKM_VT_LACC_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }

        return akl_getv_lacc(mem, data, status, timestamp);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}


void AKL_GetLibraryInfo(struct AKL_LIBRARY_INFO *info)
{
    info->algocode = AKSC_GetVersion_AlgorithmCode();
    info->major = AKSC_GetVersion_Major();
    info->minor = AKSC_GetVersion_Minor();
    info->variation = AKSC_GetVersion_Variation();
    info->revision = AKSC_GetVersion_Revision();
    info->datecode = AKSC_GetVersion_DateCode();
}


void AKL_ForceReCalibration(struct AKL_SCL_PRMS *mem)
{
    /* Check initialized */
    if (mem->init == (uint32_t)AKL_INI_MAGIC_NUMBER) {
        mem->m_lib_calib.calib_set_level(mem, AKSC_HDST_UNSOLVED);
    }
}


int16_t AKL_ChangeFormation(
    struct AKL_SCL_PRMS *mem,
    const uint8_t       formNumber)
{
    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    if (mem->m_maxForm <= formNumber) {
        return AKM_ERR_INVALID_ARG;
    }

    /* Set to struct */
    mem->m_curForm = formNumber;

    return AKM_SUCCESS;
}

int16_t AKL_SetPDC(
    const struct AKL_SCL_PRMS *mem,
    const uint8_t             pdc[AKL_PDC_SIZE],
    const uint8_t             formNumber)
{
#ifdef AKM_ENABLE_PDC
    uint8   *p_pdc;
    uint8_t i;

#ifdef AKL_ARGUMENT_CHECK

    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (pdc == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    if (mem->m_maxForm <= formNumber) {
        return AKM_ERR_INVALID_ARG;
    }

    p_pdc = mem->ps_nv[formNumber].a_pdc;

    for (i = 0U; i < (uint8_t)AKL_PDC_SIZE; i++) {
        p_pdc[i] = pdc[i];
    }

    return AKM_SUCCESS;
#else
    UNUSED_VAR(mem);
    UNUSED_VAR(pdc);
    UNUSED_VAR(formNumber);
    return AKM_ERR_NOT_SUPPORT;
#endif
}
