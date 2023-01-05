#include "akl_lib_interface.h"
#include "akm_filter.h"

#define OFFSET_ESTIMATION_INTERVAL  (320)//(1440)

/* Set DOEaG data pointer */
int16 AKL_DOEAG_SetPointer(struct AKL_SCL_PRMS *prms)
{
    int16 temp_size = 0;

    temp_size = (uint32)AKSC_GetSizeDOEaGVarF() * sizeof(int32);
    prms->ps_doeag_varf = (AKSC_DOEAGVARF *)AKM_MALLOC(temp_size);

    if(prms->ps_doeag_varf == NULL) {
        AKM_MSG_ERR_0("akm_err malloc doeag var failed : NULL");
        return AKM_ERROR;
    }

    return AKM_SUCCESS;
}
int16 AKL_DOEAG_FreePointer(struct AKL_SCL_PRMS *prms)
{
    if(prms->ps_doeag_varf) {
        AKM_FREE(prms->ps_doeag_varf);
        prms->ps_doeag_varf = NULL;
    }

    return AKM_SUCCESS;

}
int16 AKL_DOEAG_Init(struct AKL_SCL_PRMS *prms)
{
    if(prms->ps_doeag_varf == NULL)
        return AKM_ERROR;

    //Initialize HDOEAG parameters
    AKSC_InitHDOEaGF(
        prms->ps_doeag_varf,
        1,
        &prms->m_hlayout,
        &prms->v_ho,
        prms->m_hdst
    );

    /* Set offset estimation interval */
    prms->m_oedt = OFFSET_ESTIMATION_INTERVAL;
    prms->m_mode = 0;//0---DOEaG, 1---DOE

    return AKM_SUCCESS;

}
int16 AKL_DOEAG_Calibrate(struct AKL_SCL_PRMS *prms)
{
    int16 ret = 0;

    if(prms->ps_doeag_varf == NULL)
        return AKM_ERROR;

    ret = AKSC_HDOEaGF(
            prms->s_cert.a_licenser,
            prms->s_cert.a_licensee,
            prms->s_cert.a_key,
            prms->m_mode,
            prms->m_oedt,
            prms->m_hdt_ag,
            prms->m_gdt_ag,
            prms->va_hdata,
            &prms->v_gvec,
            &prms->m_hlayout,
            &prms->m_glayout,
            prms->ps_doeag_varf,
            &prms->v_ho,
            &prms->m_hdst);

    return ret;
}

int16 AKL_DOEAG_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst)
{
    if(prms->ps_doeag_varf == NULL)
        return AKM_ERROR;

    AKSC_SetHDOEaGLevelF(
        prms->ps_doeag_varf,
        &prms->m_hlayout,
        &prms->v_ho,
        hdst,
        1
    );
    prms->m_hdst = hdst;
    prms->ps_nv[prms->m_form].a_hsuc_hdst = hdst;

    return AKM_SUCCESS;

}

int16 AKL_DOEEX_SetPointer(struct AKL_SCL_PRMS *prms)
{
    int16 temp_size = 0;

    temp_size = (uint32)AKSC_GetSizeDOEEXVar() * sizeof(int32);
    prms->ps_doeex_var = (AKSC_DOEEXVAR *)AKM_MALLOC(temp_size);

    if(prms->ps_doeex_var == NULL) {
        AKM_MSG_ERR_0("akm_err malloc doeex var failed");
        return AKM_ERROR;
    }

    return AKM_SUCCESS;

}
int16 AKL_DOEEX_FreePointer(struct AKL_SCL_PRMS *prms)
{
    if(prms->ps_doeex_var) {
        AKM_FREE(prms->ps_doeex_var);
        prms->ps_doeex_var = NULL;
    }

    return AKM_SUCCESS;
}
int16 AKL_DOEEX_Init(struct AKL_SCL_PRMS *prms)
{
    if(prms->ps_doeex_var == NULL)
        return AKM_ERROR;

    AKSC_InitHDOEEX(
        prms->ps_doeex_var,
        1,
        &prms->v_ho,
        prms->m_hdst
    );

    return AKM_SUCCESS;
}
int16 AKL_DOEEX_Calibrate(struct AKL_SCL_PRMS *prms)
{
    int16 ret = 0;

    if(prms->ps_doeex_var == NULL)
        return AKM_ERROR;

    ret = AKSC_HDOEEX(
            prms->s_cert.a_licenser,
            prms->s_cert.a_licensee,
            prms->s_cert.a_key,
            prms->va_hdata,
            prms->ps_doeex_var,
            &prms->v_ho,
            &prms->m_hdst);

    return ret;
}

int16 AKL_DOEEX_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst)
{
    if(prms->ps_doeex_var == NULL)
        return AKM_ERROR;

    //Set a HDOEEX level
    AKSC_SetHDOEEXLevel(
        prms->ps_doeex_var,
        &prms->v_ho,
        hdst,
        1
    );

    prms->m_hdst = hdst;
    prms->ps_nv[prms->m_form].a_hsuc_hdst = hdst;

    return AKM_SUCCESS;

}

int16 AKL_DOE_SetPointer(struct AKL_SCL_PRMS *prms)
{
    prms->ps_doe_var = (AKSC_HDOEVAR *)AKM_MALLOC(sizeof(AKSC_HDOEVAR));

    if(prms->ps_doe_var == NULL) {
        AKM_MSG_ERR_0("akm_err malloc doe var failed");
        return AKM_ERROR;
    }

    return AKM_SUCCESS;
}
int16 AKL_DOE_FreePointer(struct AKL_SCL_PRMS *prms)
{
    if(prms->ps_doe_var) {
        AKM_FREE(prms->ps_doe_var);
        prms->ps_doe_var = NULL;
    }

    return AKM_SUCCESS;

}
int16 AKL_DOE_Init(struct AKL_SCL_PRMS *prms)
{
    if(prms->ps_doe_var == NULL)
        return AKM_ERROR;

    //Initialize HDOE parameters
    AKSC_InitHDOEProcPrmsS3(
        prms->ps_doe_var,
        1,
        &prms->v_ho,
        prms->m_hdst
    );

    return AKM_SUCCESS;
}
int16 AKL_DOE_Calibrate(struct AKL_SCL_PRMS *prms)
{
    int16 ret;

    if(prms->ps_doe_var == NULL)
        return AKM_ERROR;

    ret = AKSC_HDOEProcessS3(
            prms->s_cert.a_licenser,
            prms->s_cert.a_licensee,
            prms->s_cert.a_key,
            prms->ps_doe_var,
            prms->va_hdata,
            prms->m_hn,
            &prms->v_ho,
            &prms->m_hdst);

    return ret;
}

int16 AKL_DOE_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst)
{
    if(prms->ps_doe_var == NULL)
        return AKM_ERROR;

    //Set a HDOE level
    AKSC_SetHDOELevel(
        prms->ps_doe_var,
        &prms->v_ho,
        hdst,
        1
    );

    prms->m_hdst = hdst;
    prms->ps_nv[prms->m_form].a_hsuc_hdst = hdst;

    return AKM_SUCCESS;

}

int16 AKL_D9D_SetPointer(struct AKL_SCL_PRMS *prms)
{
    int16 temp_size = 0;

    temp_size = (uint32)AKSC_GetD9DVerSize() * sizeof(int32);
    prms->m_D9Dvar = (AKSC_D9DVAR *)AKM_MALLOC(temp_size);

    if(prms->m_D9Dvar == NULL) {
        AKM_MSG_ERR_0("akm_err AKL_Init malloc d9d failed");
        return AKM_ERROR;
    }

    return AKM_SUCCESS;
}

int16 AKL_D9D_FreePointer(struct AKL_SCL_PRMS *prms)
{
    if(prms->m_D9Dvar) {
        AKM_FREE(prms->m_D9Dvar);
        prms->m_D9Dvar = NULL;
    }

    return AKM_SUCCESS;
}
int16 AKL_D9D_Init(struct AKL_SCL_PRMS *prms)
{
    AKSC_InitD9Dv2(prms->m_D9Dvar);
    prms->m_D9Dmode = 0;
    /* Set D9Dv2 mode
     *D9DV2_MODE_9D   = 0 // use mag, gyro and acc
     *D9DV2_MODE_GYRO = 1 // use gyro only
     *D9DV2_MODE_6D   = 2 // use mag and acc
    */

#ifdef AKM_USE_D9DV2_ALPHA
    AKSC_SetD9Dv2Prms(1, 0.001, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(2, 0.005, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(3, 0.1, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(4, 0.1, prms->m_D9Dvar);
    //AKSC_SetD9Dv2Prms(5, 1.0, prms->m_D9Dvar); // goff calib by mag: ON
    AKSC_SetD9Dv2Prms(5, 100, prms->m_D9Dvar);      // goff calib by mag: OFF
    AKSC_SetD9Dv2Prms(6, 1.0, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(7, 0.9, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(8, 20, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(9, 70, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(10, 500, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(11, 20000, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(12, 16, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(13, 5, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(14, 1.5, prms->m_D9Dvar);   //A:0.8  B:1.5
    AKSC_SetD9Dv2Prms(15, 0.2, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(16, 0.2, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(17, 5, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(18, 0.7, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(19, 0.7, prms->m_D9Dvar);
    AKSC_SetD9Dv2Prms(20, 0.7, prms->m_D9Dvar);
#else
    AKSC_SetD9Dv2Prms(1, 0.4, prms->m_D9Dvar);    // Q0_cov_diag
    AKSC_SetD9Dv2Prms(2, 0.4, prms->m_D9Dvar);    // Q1_cov_diag
    AKSC_SetD9Dv2Prms(3, 0.1, prms->m_D9Dvar);    // R0_mag_cov_diag
    AKSC_SetD9Dv2Prms(4, 0.1, prms->m_D9Dvar);    // R0_acc_cov_diag
    AKSC_SetD9Dv2Prms(5, 100, prms->m_D9Dvar);    // R1_mag_cov_diag
    AKSC_SetD9Dv2Prms(6, 100, prms->m_D9Dvar);    // R1_acc_cov_diag
    AKSC_SetD9Dv2Prms(7, 0.9, prms->m_D9Dvar);    // slerp_weight
    AKSC_SetD9Dv2Prms(8, 20, prms->m_D9Dvar);     // mag_size_min
    AKSC_SetD9Dv2Prms(9, 70, prms->m_D9Dvar);     // mag_size_max
    AKSC_SetD9Dv2Prms(10, 0.1, prms->m_D9Dvar);   // mag_smooth_coef
    AKSC_SetD9Dv2Prms(11, 12, prms->m_D9Dvar);    // mag_buf_def_min
    AKSC_SetD9Dv2Prms(12, 16, prms->m_D9Dvar);    // mag_num_min
    AKSC_SetD9Dv2Prms(13, 6, prms->m_D9Dvar);     // mag_def_min
    AKSC_SetD9Dv2Prms(14, 1.5, prms->m_D9Dvar);   // mag_size_vari
    AKSC_SetD9Dv2Prms(15, 0.10, prms->m_D9Dvar);  // acc_size_min
    AKSC_SetD9Dv2Prms(16, 0.10, prms->m_D9Dvar);  // acc_def_min
    AKSC_SetD9Dv2Prms(17, 10, prms->m_D9Dvar);    // acc_num_min
    AKSC_SetD9Dv2Prms(18, 0.7, prms->m_D9Dvar);   // acc_bb_minx
    AKSC_SetD9Dv2Prms(19, 0.7, prms->m_D9Dvar);   // acc_bb_miny
    AKSC_SetD9Dv2Prms(20, 0.7, prms->m_D9Dvar);   // acc_bb_minz
    AKSC_SetD9Dv2Prms(21, 24, prms->m_D9Dvar);    // gyro_stb in Q4[dps]
    AKSC_SetD9Dv2Prms(22, 24, prms->m_D9Dvar);    // gyro_dif_stb in Q4[dps]

#endif
    return AKM_SUCCESS;
}
int16 AKL_D9D_CalcFusion(struct AKL_SCL_PRMS *prms)
{
    int16 ret;
    int16     delta, hr, hrhoriz, ar;
    int16     phi90, eta180;
    int16vec  dvec, hvec9d;
    float     swpf;
    int16   flag9D;
    AKSC_FMAT matf;
    int64_t tmp_dt;

    UNUSED_VAR(flag9D);

    //--------calc dt start
    tmp_dt = prms->m_ts_hvec - prms->m_ts_ev_9d_hvec;

    /* Limit to 16-bit value */
    if (tmp_dt > MAXTIME_USEC_IN_Q4) {
        tmp_dt = MAXTIME_USEC_IN_Q4;
    }

    prms->m_hdt = (int16_t)TIME_USEC_TO_AKSC(tmp_dt);

    if (prms->m_hdt == 0) {
        prms->m_hdt = -1;
    }

    tmp_dt = prms->m_ts_avec - prms->m_ts_ev_9d_avec;

    if (tmp_dt > MAXTIME_USEC_IN_Q4) {
        tmp_dt = MAXTIME_USEC_IN_Q4;
    }

    prms->m_adt = (int16_t)TIME_USEC_TO_AKSC(tmp_dt);

    if (prms->m_adt == 0) {
        prms->m_adt = -1;
    }

    tmp_dt = prms->m_ts_gvec - prms->m_ts_ev_9d_gvec;

    if (tmp_dt > MAXTIME_USEC_IN_Q4) {
        tmp_dt = MAXTIME_USEC_IN_Q4;
    }

    prms->m_gdt = (int16_t)TIME_USEC_TO_AKSC(tmp_dt);

    if (prms->m_gdt == 0) {
        prms->m_gdt = -1;
    }

    /* event timestamp should be always updated whether
     * the D9D function success or not. */
    prms->m_ts_ev_9d_hvec = prms->m_ts_hvec;
    prms->m_ts_ev_9d_avec = prms->m_ts_avec;
    prms->m_ts_ev_9d_gvec = prms->m_ts_gvec;

    AKM_MSG_INFO_1("akm_log AKL_D9D_CalcFusion m_hdt(msQ4) = %d", prms->m_hdt);
    AKM_MSG_INFO_1("akm_log AKL_D9D_CalcFusion m_gdt(msQ4) = %d", prms->m_gdt);
    AKM_MSG_INFO_1("akm_log AKL_D9D_CalcFusion m_adt(msQ4) = %d", prms->m_adt);
    //--------calc dt end

    /* dvec is always 0 */
    dvec.u.x = 0;
    dvec.u.y = 0;
    dvec.u.z = 0;

    AKSC_VNorm(
        &prms->va_hdata[0],
        &prms->v_ho,
        &prms->v_hs,
        AKSC_HSENSE_TARGET,
        &hvec9d
    );

    ret = AKSC_D9Dv2(
            prms->m_hdst,
            prms->s_cert.a_licenser,
            prms->s_cert.a_licensee,
            prms->s_cert.a_key,
            prms->m_D9Dmode,
            &hvec9d,
            &prms->v_avec,
            &prms->v_gvecf,
            prms->m_hdt,
            prms->m_adt,
            prms->m_gdt,
            &dvec,
            &prms->m_hlayout,
            &prms->m_alayout,
            &prms->m_glayout,
            prms->m_D9Dvar,
#ifdef AKM_USE_D9DV2_ALPHA
            & flag9D,
#endif
            & prms->m_theta,
            &delta,
            &hr,
            &hrhoriz,
            &ar,
            &prms->m_phi180,
            &phi90,
            &eta180,
            &prms->m_eta90,
            &matf,
            &prms->s_quatf,
            &prms->v_gravity,
            &prms->v_lacc
        );

    if (ret != 1) {
        AKM_MSG_ERR_1("akm_err AKSC_D9Dv2 ERROR: %d", ret);
        return AKM_ERROR;
    }

    //ori
    prms->m_eta90 = -prms->m_eta90;
    //gravity
    swpf = prms->v_gravity.u.x;
    prms->v_gravity.u.x = -prms->v_gravity.u.y;
    prms->v_gravity.u.y = swpf;
    prms->v_gravity.u.z = prms->v_gravity.u.z;
    //lacc
    swpf = prms->v_lacc.u.x;
    prms->v_lacc.u.x = -prms->v_lacc.u.y;
    prms->v_lacc.u.y = swpf;
    prms->v_lacc.u.z = prms->v_lacc.u.z;
    //quat
    swpf = prms->s_quatf.u.x;
    prms->s_quatf.u.x = prms->s_quatf.u.y;
    prms->s_quatf.u.y = -(swpf);
    prms->s_quatf.u.z = -(prms->s_quatf.u.z);

    /* update d9d time stamp which equals to the max of ts_hvec ts_avec ts_gvec*/
    tmp_dt = prms->m_ts_hvec;

    if (prms->m_ts_avec > tmp_dt)
        tmp_dt = prms->m_ts_avec;

    if (prms->m_ts_gvec > tmp_dt)
        tmp_dt = prms->m_ts_gvec;

    prms->m_ts_ev_d9d = tmp_dt;

    return AKM_SUCCESS;
}
int16 AKL_D6D_SetPointer(struct AKL_SCL_PRMS *prms)
{
    //Do nothing for 6d fusion
    UNUSED_VAR(prms);
    return AKM_SUCCESS;
}

int16 AKL_D6D_FreePointer(struct AKL_SCL_PRMS *prms)
{
    //Do nothing for 6d fusion
    UNUSED_VAR(prms);
    return AKM_SUCCESS;
}
int16 AKL_D6D_Init(struct AKL_SCL_PRMS *prms)
{
    //Do nothing for 6d fusion
    UNUSED_VAR(prms);
#ifdef AKM_ENABLE_D6D_YAW_FILTER
    akm_d6d_yaw_filter_init();
#endif
    return AKM_SUCCESS;
}
int16 AKL_D6D_CalcFusion(struct AKL_SCL_PRMS *prms)
{
    int16     ret, swp;
    int16     preThe, delta, hr, hrhoriz, ar;
    int16     phi90, eta180;
    int16vec  dvec;
    I16MATRIX mat;
    int64_t maxt;

    /* store previous value */
    preThe = prms->m_theta;

    /* dvec is always 0 */
    dvec.u.x = 0;
    dvec.u.y = 0;
    dvec.u.z = 0;

    ret = AKSC_DirectionS3(
            prms->s_cert.a_licenser,
            prms->s_cert.a_licensee,
            prms->s_cert.a_key,
            &prms->v_hvec,
            &prms->v_avec,
            &dvec,
            &prms->m_hlayout,
            &prms->m_alayout,
            &prms->m_theta,
            &delta,
            &hr,
            &hrhoriz,
            &ar,
            &prms->m_phi180,
            &phi90,
            &eta180,
            &prms->m_eta90,
            &mat,
            &prms->s_quat
        );
    prms->m_theta = AKSC_ThetaFilter(
            prms->m_theta,
            preThe,
            THETAFILTER_SCALE
        );

    if(ret != 3) {
        AKM_MSG_ERR_1("akm_err AKSC_DirectionS3 ERROR: %d", ret);
        return AKM_ERROR;
    }

#ifdef AKM_ENABLE_D6D_YAW_FILTER
    prms->m_theta = akm_d6d_yaw_filter(prms->m_theta);
#endif
    //ori
    prms->m_eta90 = -prms->m_eta90;

    //Convert Quaternion
    swp = prms->s_quat.u.x;
    prms->s_quat.u.x = prms->s_quat.u.y;
    prms->s_quat.u.y = -(swp);
    prms->s_quat.u.z = -(prms->s_quat.u.z);

    prms->s_quatf.u.x = QUAT_AKSC_TO_UNIT(prms->s_quat.u.x);
    prms->s_quatf.u.y = QUAT_AKSC_TO_UNIT(prms->s_quat.u.y);
    prms->s_quatf.u.z = QUAT_AKSC_TO_UNIT(prms->s_quat.u.z);

    /* update d6d time stamp which equals to the max of ts_hvec ts_avec*/
    maxt = prms->m_ts_hvec;

    if (prms->m_ts_avec > maxt)
        maxt = prms->m_ts_avec;

    prms->m_ts_ev_d9d = maxt;

    return AKM_SUCCESS;

}

int16 AKL_PG_SetPointer(struct AKL_SCL_PRMS *prms)
{
    prms->ps_PGvar = (struct AKL_PG_PRMS *)AKM_MALLOC(sizeof(struct AKL_PG_PRMS));

    if(prms->ps_PGvar == NULL) {
        AKM_MSG_ERR_0("akm_err malloc PG var failed");
        return AKM_ERROR;
    }

    return AKM_SUCCESS;

}
int16 AKL_PG_FreePointer(struct AKL_SCL_PRMS *prms)
{
    if(prms->ps_PGvar) {
        AKM_FREE(prms->ps_PGvar);
        prms->ps_PGvar = NULL;
    }

    return AKM_SUCCESS;

}

int16 AKL_PG_Init(struct AKL_SCL_PRMS *prms)
{
    AKSC_InitPseudoGyroF(&prms->ps_PGvar->pg_cond, &prms->ps_PGvar->pg_var);
#ifdef AKM_CUSTOM_PG_FILTER
    prms->ps_PGvar->pg_filter = AKM_CUSTOM_PG_FILTER;
#else
    prms->ps_PGvar->pg_filter = 7;
#endif
    prms->ps_PGvar->pg_cond.fmode = 1;
    prms->ps_PGvar->pg_cond.th_rdif = 2400;
    prms->ps_PGvar->pg_cond.th_rmax = 2400;
    prms->ps_PGvar->pg_cond.th_rmin = 0;
    prms->ps_PGvar->pg_cond.ihave = 12;
    prms->ps_PGvar->pg_cond.iaave = 12;
    prms->ps_PGvar->pg_cond.ocoef = 0.1;//90;//103;

#ifdef AKM_ENABLE_PG_MAG_FILTER
    akm_pg_mag_filter_init();
#endif

#ifdef AKM_ENABLE_PG_ACC_FILTER

    akm_pg_acc_filter_init();
#endif

    switch(prms->ps_PGvar->pg_filter) {
    case 0:
        prms->ps_PGvar->pg_cond.ihave = 24;
        prms->ps_PGvar->pg_cond.iaave = 24;
        prms->ps_PGvar->pg_cond.ocoef = 0.1;//103;
        break;

    case 1:
        prms->ps_PGvar->pg_cond.ihave = 24;
        prms->ps_PGvar->pg_cond.iaave = 24;
        prms->ps_PGvar->pg_cond.ocoef = 0.2;//205;
        break;

    case 2:
        prms->ps_PGvar->pg_cond.ihave = 24;
        prms->ps_PGvar->pg_cond.iaave = 24;
        prms->ps_PGvar->pg_cond.ocoef = 0.3;//307;
        break;

    case 3:
        prms->ps_PGvar->pg_cond.ihave = 32;
        prms->ps_PGvar->pg_cond.iaave = 32;
        prms->ps_PGvar->pg_cond.ocoef = 0.2;//205;
        break;

    case 4:
        prms->ps_PGvar->pg_cond.ihave = 32;
        prms->ps_PGvar->pg_cond.iaave = 32;
        prms->ps_PGvar->pg_cond.ocoef = 0.3;//307;
        break;

    case 5:
        prms->ps_PGvar->pg_cond.ihave = 12;
        prms->ps_PGvar->pg_cond.iaave = 12;
        prms->ps_PGvar->pg_cond.ocoef = 0.3;//307;
        break;

    case 6:
        prms->ps_PGvar->pg_cond.ihave = 12;
        prms->ps_PGvar->pg_cond.iaave = 12;
        prms->ps_PGvar->pg_cond.ocoef = 0.2;//205;
        break;

    case 7:
        prms->ps_PGvar->pg_cond.ihave = 12;
        prms->ps_PGvar->pg_cond.iaave = 12;
        prms->ps_PGvar->pg_cond.ocoef = 0.1;//103;
        break;

    case 8:
        prms->ps_PGvar->pg_cond.ihave = 32;
        prms->ps_PGvar->pg_cond.iaave = 32;
        prms->ps_PGvar->pg_cond.ocoef = 0.1;//205;
        break;
    }

    return AKM_SUCCESS;
}

int16 AKL_PG_CalcAngularRate(struct AKL_SCL_PRMS *prms)
{
    int16vec tmp_hvec;
    AKSC_FVEC	dhvec;
    AKSC_FVEC	davec;
    AKSC_FLOAT	swpf;
    int16 aksc_ret;

    // Subtract offset from non-averaged value.
    aksc_ret = AKSC_VNorm(
            &prms->v_have,
            &prms->v_ho,
            &prms->v_hs,
            AKSC_HSENSE_TARGET,
            &tmp_hvec
        );

    if (aksc_ret == 0) {
        return AKM_ERROR;
    }

    dhvec.u.x = (AKSC_FLOAT)tmp_hvec.u.x;
    dhvec.u.y = (AKSC_FLOAT)tmp_hvec.u.y;
    dhvec.u.z = (AKSC_FLOAT)tmp_hvec.u.z;

    davec.u.x = (AKSC_FLOAT)prms->v_avec.u.x;
    davec.u.y = (AKSC_FLOAT)prms->v_avec.u.y;
    davec.u.z = (AKSC_FLOAT)prms->v_avec.u.z;

#ifdef AKM_ENABLE_PG_MAG_FILTER
    akm_pg_mag_filter(&(dhvec.u.x), &(dhvec.u.y), &(dhvec.u.z));
#endif

#ifdef AKM_ENABLE_PG_ACC_FILTER

    akm_pg_acc_filter(&(davec.u.x), &(davec.u.y), &(davec.u.z));
#endif

    prms->ps_PGvar->pg_dt = prms->m_hdt_ag;

    aksc_ret = AKSC_PseudoGyroF(
            &prms->ps_PGvar->pg_cond,
            prms->ps_PGvar->pg_dt,
            &dhvec,
            &davec,
            &prms->m_hlayout,
            &prms->m_alayout,
            &prms->ps_PGvar->pg_var,
            &prms->ps_PGvar->pg_out,
            &prms->ps_PGvar->pg_quat,
            &prms->ps_PGvar->pg_gravity,
            &prms->ps_PGvar->pg_linacc
        );

    if(aksc_ret != 1) {
        return AKM_ERROR;
    } else {
        /*Convertion:
          from: AKM coordinate, AKSC units
          to  : Android coordinate, AKSC units. */

        swpf = prms->ps_PGvar->pg_out.u.x;
        prms->ps_PGvar->pg_out.u.x = -prms->ps_PGvar->pg_out.u.y;
        prms->ps_PGvar->pg_out.u.y = swpf;
        prms->ps_PGvar->pg_out.u.z = prms->ps_PGvar->pg_out.u.z;

        swpf = prms->ps_PGvar->pg_quat.u.x;
        prms->ps_PGvar->pg_quat.u.x = prms->ps_PGvar->pg_quat.u.y;
        prms->ps_PGvar->pg_quat.u.y = -swpf;
        prms->ps_PGvar->pg_quat.u.z = -prms->ps_PGvar->pg_quat.u.z;

        swpf = prms->ps_PGvar->pg_gravity.u.x;
        prms->ps_PGvar->pg_gravity.u.x = prms->ps_PGvar->pg_gravity.u.y;
        prms->ps_PGvar->pg_gravity.u.y = -swpf;
        prms->ps_PGvar->pg_gravity.u.z = -prms->ps_PGvar->pg_gravity.u.z;

        swpf = prms->ps_PGvar->pg_linacc.u.x;
        prms->ps_PGvar->pg_linacc.u.x = -prms->ps_PGvar->pg_linacc.u.y;
        prms->ps_PGvar->pg_linacc.u.y = swpf;
        prms->ps_PGvar->pg_linacc.u.z = prms->ps_PGvar->pg_linacc.u.z;


        //Copy PG output to 9D output
        //because akl_apis,c only get 9D output
        prms->v_gvecf.u.x = prms->ps_PGvar->pg_out.u.x;
        prms->v_gvecf.u.y = prms->ps_PGvar->pg_out.u.y;
        prms->v_gvecf.u.z = prms->ps_PGvar->pg_out.u.z;

        prms->v_gravity.u.x = prms->ps_PGvar->pg_gravity.u.x;
        prms->v_gravity.u.y = prms->ps_PGvar->pg_gravity.u.y;
        prms->v_gravity.u.z = prms->ps_PGvar->pg_gravity.u.z;

        prms->v_lacc.u.x = prms->ps_PGvar->pg_linacc.u.x;
        prms->v_lacc.u.y = prms->ps_PGvar->pg_linacc.u.y;
        prms->v_lacc.u.z = prms->ps_PGvar->pg_linacc.u.z;

        prms->s_quatf.u.x = prms->ps_PGvar->pg_quat.u.x;
        prms->s_quatf.u.y = prms->ps_PGvar->pg_quat.u.y;;
        prms->s_quatf.u.z = prms->ps_PGvar->pg_quat.u.z;

        AKM_MSG_INFO_4("akm_log pg_gyr(d/s*16*1000): %d, %d, %d, dt: %d", (int32_t)(prms->v_gvecf.u.x * 1000), (int32_t)(prms->v_gvecf.u.y * 1000),
            (int32_t)(prms->v_gvecf.u.z * 1000), prms->ps_PGvar->pg_dt);
    }

    return AKM_SUCCESS;
}

int16 AKL_NULL_Function(struct AKL_SCL_PRMS *prms)
{
    UNUSED_VAR(prms);
    return AKM_SUCCESS;
}

