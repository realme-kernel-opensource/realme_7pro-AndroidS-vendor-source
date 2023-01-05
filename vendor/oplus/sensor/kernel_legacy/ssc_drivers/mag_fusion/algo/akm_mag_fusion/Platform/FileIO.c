/******************************************************************************
 *
 *  $Id: $
 *
 * -- Copyright Notice --
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
 * -- End Asahi Kasei Microdevices Copyright Notice --
 *
 ******************************************************************************/
#include "FileIO.h"
const uint8 pdc_init[27] = {64, 81, 68, 141, 225, 236, 0, 235, 214,
        54, 236, 237, 148, 129, 0, 57, 255, 176,
        193, 110, 188, 252, 44, 222, 11, 92, 78
    };

//Save data in AKL_NV_PRMS to file
int16 AKL_LoadParameter(struct AKL_NV_PRMS *nv_data)
{
    UNUSED_VAR(nv_data);
    return 0;
}


int16 AKL_LoadPDC(uint8 *pdc)
{
    int i = 0;

    for(i = 0; i < 27; i++) {
        pdc[i] = pdc_init[i];
    }

    return 0;
}

//Load data to AKL_NV_PRMS from file
int16 AKL_SaveParameter(struct AKL_NV_PRMS *nv_data)
{
    UNUSED_VAR(nv_data);
    return 0;
}

